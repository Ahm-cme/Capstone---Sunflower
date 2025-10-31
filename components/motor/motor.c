#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>  
#include "esp_log.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

/*
    Motor Control Module (Linear Actuators for AZ/EL)

    What this module does:
    - Configures LEDC PWM for two actuators (AZ, EL)
    - Drives direction GPIOs
    - Provides time-based open-loop moves to target angles
    - Provides raw timed runs for homing
    - Tracks move statistics for display integration

    Conservative timing strategy:
    - Base calculation from kinematics
    - Apply 90% safety factor (prevents overshoot)
    - Add 100ms minimum buffer (accounts for startup/coast)
    - Result: accurate positioning without overshoot

    Integration points:
    - System check uses motor_init() return code
    - Display shows motor ranges and current position
    - Tracking module uses move statistics for quality metrics

    Quick troubleshooting:
    - Wrong direction: check wiring and DIR level used by homing config
    - Too slow/fast: verify battery voltage, adjust speed_mm_per_s
    - No motion: verify PWM present, driver powered, DIR level changes
    - Drift over days: expected; nightly homing resets reference
    - PWM issues: 5 kHz, 13-bit resolution is a good default

    Log levels:
    - V: detailed math/derived values
    - D: configuration, timing, flow
    - I: move requests and completions
    - W: clamping and safety limits
    - E: failures and critical issues
*/

#define TAG "MOTOR"

// Safety factor: use 90% of calculated time to prevent overshoot
#define TIMING_SAFETY_FACTOR 0.90

// Minimum safety buffer added to all moves (ms)
#define MIN_SAFETY_BUFFER_MS 100

// Self-test pulse duration (ms)
#define SELF_TEST_PULSE_MS 200

// Internal state: keep a copy of config and define LEDC channel assignments
static motor_cfg_t s_cfg;
static ledc_channel_t AZ_CH = LEDC_CHANNEL_0;  // Azimuth actuator PWM
static ledc_channel_t EL_CH = LEDC_CHANNEL_1;  // Elevation actuator PWM

// Move statistics for tracking quality
static motor_stats_t s_stats = {0};
static bool s_initialized = false;

/* ────────────────────── Internal Helper Functions ────────────────────── */

/*
    Map panel angle (deg) to actuator stroke (mm), clamped to [0, stroke_mm].
    Linear model: (angle / max_angle) * stroke_mm.
*/
static double angle_to_mm(double angle_deg, double max_angle_deg){
    // Clamp input to valid range to prevent driving past mechanical limits
    if (angle_deg < 0.0) {
        ESP_LOGD(TAG, "Angle %.2f° < 0, clamping to 0°", angle_deg);
        angle_deg = 0.0;
    }
    if (angle_deg > max_angle_deg) {
        ESP_LOGD(TAG, "Angle %.2f° > max %.2f°, clamping", angle_deg, max_angle_deg);
        angle_deg = max_angle_deg;
    }
    
    // Linear mapping: proportion of max angle × total stroke
    double mm = (angle_deg / max_angle_deg) * s_cfg.stroke_mm;
    ESP_LOGV(TAG, "Angle %.2f° / %.2f° → %.2f mm / %.2f mm", 
             angle_deg, max_angle_deg, mm, s_cfg.stroke_mm);
    return mm;
}

/*
    Compute move time (ms) from distance and nominal speed.
    Conservative approach:
    - Calculate base time from kinematics
    - Apply safety factor (90% of calculated time)
    - Add minimum safety buffer
    
    This prevents overshoot due to:
    - Actuator momentum/inertia
    - Voltage variations
    - Manufacturing tolerances
*/
static uint32_t move_time_ms(double cur_mm, double tgt_mm){
    double distance_mm = fabs(tgt_mm - cur_mm);
    
    // Base time calculation from kinematics
    double base_time_ms = (distance_mm / s_cfg.speed_mm_per_s) * 1000.0;
    
    // Apply safety factor to prevent overshoot
    double safe_time_ms = base_time_ms * TIMING_SAFETY_FACTOR;
    
    // Add minimum safety buffer (prevents ultra-short moves from being too aggressive)
    uint32_t total_ms = (uint32_t)safe_time_ms + MIN_SAFETY_BUFFER_MS;
    
    ESP_LOGD(TAG, "Move %.2f mm @ %.2f mm/s:", distance_mm, s_cfg.speed_mm_per_s);
    ESP_LOGD(TAG, "  Base: %.0f ms", base_time_ms);
    ESP_LOGD(TAG, "  Safe (%.0f%%): %.0f ms", TIMING_SAFETY_FACTOR * 100, safe_time_ms);
    ESP_LOGD(TAG, "  Total: %" PRIu32 " ms (with %d ms buffer)", total_ms, MIN_SAFETY_BUFFER_MS);
    
    return total_ms;
}

/*
    Start PWM on a channel with given duty (0..8191), then update.
*/
static void start_pwm(ledc_channel_t ch, int duty){ 
    ESP_LOGD(TAG, "Start PWM ch=%d duty=%d (%d%%)", (int)ch, duty, (duty * 100) / 8191);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/*
    Stop PWM on a channel (sets duty to 0%).
*/
static void stop_pwm(ledc_channel_t ch){
    ESP_LOGD(TAG, "Stop PWM ch=%d", (int)ch);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ─────────────────────────── Public API ─────────────────────────────── */

/*
    Initialize PWM timer/channels and DIR pins.
    Expects motor_cfg_t with pins, stroke, speed, and limits.
*/
esp_err_t motor_init(const motor_cfg_t *cfg){
    if (!cfg) {
        ESP_LOGE(TAG, "NULL configuration provided");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Store configuration in module-global state
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          MOTOR SYSTEM INITIALIZATION                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Actuator Specifications:");
    ESP_LOGI(TAG, "  - Stroke: %.1f mm", s_cfg.stroke_mm);
    ESP_LOGI(TAG, "  - Speed: %.2f mm/s (nominal @ 12V)", s_cfg.speed_mm_per_s);
    ESP_LOGI(TAG, "  - Full stroke time: ~%.1f seconds", s_cfg.stroke_mm / s_cfg.speed_mm_per_s);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Timing Strategy (Conservative):");
    ESP_LOGI(TAG, "  - Base calculation: distance / speed");
    ESP_LOGI(TAG, "  - Safety factor: %.0f%% (prevents overshoot)", TIMING_SAFETY_FACTOR * 100);
    ESP_LOGI(TAG, "  - Minimum buffer: %d ms (startup/coast)", MIN_SAFETY_BUFFER_MS);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Azimuth Motor (AZ):");
    ESP_LOGI(TAG, "  - Range: 0° to %.1f°", s_cfg.max_az_deg);
    ESP_LOGI(TAG, "  - PWM pin: GPIO%d", s_cfg.az_pwm_pin);
    ESP_LOGI(TAG, "  - DIR pin: GPIO%d", s_cfg.az_dir_pin);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Elevation Motor (EL):");
    ESP_LOGI(TAG, "  - Range: %.1f° to %.1f°", s_cfg.min_el_deg, s_cfg.max_el_deg);
    ESP_LOGI(TAG, "  - PWM pin: GPIO%d", s_cfg.el_pwm_pin);
    ESP_LOGI(TAG, "  - DIR pin: GPIO%d", s_cfg.el_dir_pin);
    ESP_LOGI(TAG, "");

    // Configure LEDC timer (shared by both PWM channels)
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,    // More stable than high-speed mode
        .timer_num        = LEDC_TIMER_0,           // Use timer 0 (could be any available timer)
        .duty_resolution  = LEDC_TIMER_13_BIT,      // 13 bits = 0-8191 duty range
        .freq_hz          = 5000,                   // 5 kHz is good for motor drivers
        .clk_cfg          = LEDC_AUTO_CLK           // Let ESP-IDF choose best clock source
    };
    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ LEDC timer configured:");
    ESP_LOGI(TAG, "  - Frequency: 5 kHz");
    ESP_LOGI(TAG, "  - Resolution: 13-bit (0-8191)");

    // Configure azimuth PWM channel
    ledc_channel_config_t az_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = AZ_CH,
        .timer_sel      = LEDC_TIMER_0,         // Use the timer we just configured
        .gpio_num       = s_cfg.az_pwm_pin,
        .duty           = 0,                    // Start with 0% duty (stopped)
        .intr_type      = LEDC_INTR_DISABLE,    // We don't need PWM interrupts
        .hpoint         = 0                     // Default phase offset
    };
    ret = ledc_channel_config(&az_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ AZ PWM channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ AZ PWM channel configured (CH%d → GPIO%d)", (int)AZ_CH, s_cfg.az_pwm_pin);

    // Configure elevation PWM channel
    ledc_channel_config_t el_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = EL_CH,
        .timer_sel      = LEDC_TIMER_0,         // Same timer as AZ channel
        .gpio_num       = s_cfg.el_pwm_pin,
        .duty           = 0,                    // Start with 0% duty (stopped)
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&el_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ EL PWM channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ EL PWM channel configured (CH%d → GPIO%d)", (int)EL_CH, s_cfg.el_pwm_pin);

    // Configure DIR pins as GPIO outputs (default low)
    ret = gpio_set_direction(s_cfg.az_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ AZ DIR pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_set_direction(s_cfg.el_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ EL DIR pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize DIR pins to known state (retract direction)
    gpio_set_level(s_cfg.az_dir_pin, 0);
    gpio_set_level(s_cfg.el_dir_pin, 0);
    ESP_LOGI(TAG, "✓ DIR pins initialized:");
    ESP_LOGI(TAG, "  - AZ DIR: GPIO%d → LOW (retract)", s_cfg.az_dir_pin);
    ESP_LOGI(TAG, "  - EL DIR: GPIO%d → LOW (retract)", s_cfg.el_dir_pin);
    ESP_LOGI(TAG, "");

    // Initialize statistics
    s_stats.last_az_move_deg = 0.0;
    s_stats.last_el_move_deg = 0.0;
    s_stats.last_az_duration_ms = 0;
    s_stats.last_el_duration_ms = 0;
    s_stats.total_moves = 0;
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "✓ Motor initialization complete");
    ESP_LOGI(TAG, "  System ready for tracking operations");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

/*
    Move AZ to target angle (deg). Applies 0..max_az_deg clamp, sets DIR,
    runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
*/
void motor_move_az(double current_deg, double target_deg){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Motor system not initialized - cannot move AZ");
        return;
    }
    
    ESP_LOGI(TAG, "AZ move requested: %.2f° → %.2f°", current_deg, target_deg);

    // Apply safety limits to target angle
    double clamped_target = target_deg;
    if (target_deg > s_cfg.max_az_deg) {
        clamped_target = s_cfg.max_az_deg;
        ESP_LOGW(TAG, "⚠ AZ target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_az_deg);
    }
    if (target_deg < 0.0) {
        clamped_target = 0.0;
        ESP_LOGW(TAG, "⚠ AZ target %.2f° below min 0°, clamped", target_deg);
    }

    // Calculate move delta for statistics
    double move_delta = fabs(clamped_target - current_deg);
    
    // Skip tiny moves (< 0.1°) to reduce wear
    if (move_delta < 0.1) {
        ESP_LOGD(TAG, "AZ move delta %.3f° too small, skipping", move_delta);
        return;
    }

    // Convert angles to actuator stroke positions
    double current_mm = angle_to_mm(current_deg, s_cfg.max_az_deg);
    double target_mm = angle_to_mm(clamped_target, s_cfg.max_az_deg);
    
    // Determine direction: extend if target > current, retract otherwise
    int dir_level = (target_mm > current_mm) ? 1 : 0;
    const char* dir_name = dir_level ? "EXTEND" : "RETRACT";
    
    // Set direction pin before starting PWM
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    ESP_LOGD(TAG, "AZ direction: %s (GPIO%d = %d)", dir_name, s_cfg.az_dir_pin, dir_level);

    // Calculate conservative move time
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    // Log the complete move plan
    ESP_LOGI(TAG, "AZ executing:");
    ESP_LOGI(TAG, "  Position: %.2f° → %.2f° (Δ=%.2f°)", current_deg, clamped_target, move_delta);
    ESP_LOGI(TAG, "  Stroke: %.1f mm → %.1f mm (Δ=%.1f mm)", current_mm, target_mm, fabs(target_mm - current_mm));
    ESP_LOGI(TAG, "  Direction: %s", dir_name);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms (conservative)", move_ms);

    // Record start time for statistics
    uint32_t start_tick = xTaskGetTickCount();
    
    // Execute the move at full speed with conservative timing
    start_pwm(AZ_CH, 8191);                     // 100% duty
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Conservative duration
    stop_pwm(AZ_CH);                            // Stop PWM, actuator coasts
    
    // Calculate actual duration
    uint32_t actual_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    
    // Update statistics
    s_stats.last_az_move_deg = move_delta;
    s_stats.last_az_duration_ms = actual_ms;
    s_stats.total_moves++;
    
    ESP_LOGI(TAG, "✓ AZ move complete:");
    ESP_LOGI(TAG, "  Final position: ~%.1f°", clamped_target);
    ESP_LOGI(TAG, "  Actual duration: %" PRIu32 " ms", actual_ms);
    ESP_LOGI(TAG, "  Total moves: %" PRIu32, s_stats.total_moves);
}

/*
    Move EL to target angle (deg). Applies min_el_deg..max_el_deg clamp,
    sets DIR, runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
*/
void motor_move_el(double current_deg, double target_deg){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Motor system not initialized - cannot move EL");
        return;
    }
    
    ESP_LOGI(TAG, "EL move requested: %.2f° → %.2f°", current_deg, target_deg);

    // Apply elevation-specific safety limits
    double clamped_target = target_deg;
    if (target_deg > s_cfg.max_el_deg) {
        clamped_target = s_cfg.max_el_deg;
        ESP_LOGW(TAG, "⚠ EL target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_el_deg);
    }
    if (target_deg < s_cfg.min_el_deg) {
        clamped_target = s_cfg.min_el_deg;
        ESP_LOGW(TAG, "⚠ EL target %.2f° below min %.2f°, clamped", target_deg, s_cfg.min_el_deg);
    }

    // Calculate move delta
    double move_delta = fabs(clamped_target - current_deg);
    
    // Skip tiny moves
    if (move_delta < 0.1) {
        ESP_LOGD(TAG, "EL move delta %.3f° too small, skipping", move_delta);
        return;
    }

    // Convert angles to actuator stroke positions
    // Note: EL uses max_el_deg for scaling, not the full 0-max range like AZ
    double current_mm = angle_to_mm(current_deg, s_cfg.max_el_deg);
    double target_mm = angle_to_mm(clamped_target, s_cfg.max_el_deg);
    
    // Determine direction
    int dir_level = (target_mm > current_mm) ? 1 : 0;
    const char* dir_name = dir_level ? "EXTEND" : "RETRACT";
    
    // Set direction before starting motion
    gpio_set_level(s_cfg.el_dir_pin, dir_level);
    ESP_LOGD(TAG, "EL direction: %s (GPIO%d = %d)", dir_name, s_cfg.el_dir_pin, dir_level);

    // Calculate conservative timing
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    ESP_LOGI(TAG, "EL executing:");
    ESP_LOGI(TAG, "  Position: %.2f° → %.2f° (Δ=%.2f°)", current_deg, clamped_target, move_delta);
    ESP_LOGI(TAG, "  Stroke: %.1f mm → %.1f mm (Δ=%.1f mm)", current_mm, target_mm, fabs(target_mm - current_mm));
    ESP_LOGI(TAG, "  Direction: %s", dir_name);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms (conservative)", move_ms);

    // Record start time
    uint32_t start_tick = xTaskGetTickCount();
    
    // Execute the move
    start_pwm(EL_CH, 8191);                     // 100% duty
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Conservative duration
    stop_pwm(EL_CH);                            // Stop
    
    // Calculate actual duration
    uint32_t actual_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    
    // Update statistics
    s_stats.last_el_move_deg = move_delta;
    s_stats.last_el_duration_ms = actual_ms;
    s_stats.total_moves++;
    
    ESP_LOGI(TAG, "✓ EL move complete:");
    ESP_LOGI(TAG, "  Final position: ~%.1f°", clamped_target);
    ESP_LOGI(TAG, "  Actual duration: %" PRIu32 " ms", actual_ms);
    ESP_LOGI(TAG, "  Total moves: %" PRIu32, s_stats.total_moves);
}

/*
    Raw timed AZ run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
*/
void motor_run_az_ms(int dir_level, uint32_t ms){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Motor system not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGW(TAG, "║          AZ HOMING MODE - NO LIMITS                        ║");
    ESP_LOGW(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "AZ timed run:");
    ESP_LOGI(TAG, "  Direction: %s (GPIO%d = %d)", dir_level ? "EXTEND" : "RETRACT", s_cfg.az_dir_pin, dir_level);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms", ms);
    ESP_LOGW(TAG, "  ⚠ Will drive to mechanical stop!");
    ESP_LOGI(TAG, "");
    
    // Set direction immediately
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    
    // Run at full power for specified duration (no safety factor for homing)
    start_pwm(AZ_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(ms));
    stop_pwm(AZ_CH);
    
    ESP_LOGI(TAG, "✓ AZ homing run complete");
    ESP_LOGI(TAG, "");
}

/*
    Raw timed EL run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
*/
void motor_run_el_ms(int dir_level, uint32_t ms){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Motor system not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGW(TAG, "║          EL HOMING MODE - NO LIMITS                        ║");
    ESP_LOGW(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EL timed run:");
    ESP_LOGI(TAG, "  Direction: %s (GPIO%d = %d)", dir_level ? "EXTEND" : "RETRACT", s_cfg.el_dir_pin, dir_level);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms", ms);
    ESP_LOGW(TAG, "  ⚠ Will drive to mechanical stop!");
    ESP_LOGI(TAG, "");
    
    // Set direction
    gpio_set_level(s_cfg.el_dir_pin, dir_level);
    
    // Execute timed run (no safety factor for homing)
    start_pwm(EL_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(ms));
    stop_pwm(EL_CH);
    
    ESP_LOGI(TAG, "✓ EL homing run complete");
    ESP_LOGI(TAG, "");
}

/*
    Park sequence: move AZ, then EL to specified park angles (sequential to reduce current).
*/
void motor_park(double park_az_deg, double park_el_deg, double cur_az, double cur_el){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Motor system not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              PARKING SEQUENCE                              ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Park positions:");
    ESP_LOGI(TAG, "  AZ: %.2f° → %.2f° (Δ=%.2f°)", cur_az, park_az_deg, fabs(park_az_deg - cur_az));
    ESP_LOGI(TAG, "  EL: %.2f° → %.2f° (Δ=%.2f°)", cur_el, park_el_deg, fabs(park_el_deg - cur_el));
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Sequential move (AZ first, then EL):");
    ESP_LOGI(TAG, "  - Reduces peak current draw");
    ESP_LOGI(TAG, "  - 500ms pause between axes");
    ESP_LOGI(TAG, "");
    
    // Move axes sequentially (not simultaneously to reduce peak current)
    motor_move_az(cur_az, park_az_deg);
    ESP_LOGI(TAG, "Pausing between moves...");
    vTaskDelay(pdMS_TO_TICKS(500));             // Brief pause between moves
    motor_move_el(cur_el, park_el_deg);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ Park sequence complete");
    ESP_LOGI(TAG, "  System in sleep/storage position");
    ESP_LOGI(TAG, "");
}

/*
    Emergency stop: immediately stop PWM on both channels (DIR levels unchanged).
*/
void motor_stop_all(void){
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGE(TAG, "║              ⚠  EMERGENCY STOP  ⚠                        ║");
    ESP_LOGE(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "Halting all motor PWM immediately");
    ESP_LOGE(TAG, "  - AZ channel: 0%% duty");
    ESP_LOGE(TAG, "  - EL channel: 0%% duty");
    ESP_LOGE(TAG, "  - DIR pins unchanged");
    
    stop_pwm(AZ_CH);
    stop_pwm(EL_CH);
    
    ESP_LOGE(TAG, "✓ All motors stopped");
    ESP_LOGE(TAG, "  Manual intervention may be required");
    ESP_LOGE(TAG, "");
}

/*
    Get motor configuration (for diagnostics).
*/
const motor_cfg_t* motor_get_config(void){
    return &s_cfg;
}

/*
    Self-test: briefly pulse each motor in both directions.
*/
bool motor_self_test(void){
    if (!s_initialized) {
        ESP_LOGE(TAG, "Cannot self-test: not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          MOTOR SELF-TEST                                   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Testing AZ motor...");
    
    // Test AZ extend
    gpio_set_level(s_cfg.az_dir_pin, 1);
    start_pwm(AZ_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(SELF_TEST_PULSE_MS));
    stop_pwm(AZ_CH);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Test AZ retract
    gpio_set_level(s_cfg.az_dir_pin, 0);
    start_pwm(AZ_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(SELF_TEST_PULSE_MS));
    stop_pwm(AZ_CH);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Testing EL motor...");
    
    // Test EL extend
    gpio_set_level(s_cfg.el_dir_pin, 1);
    start_pwm(EL_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(SELF_TEST_PULSE_MS));
    stop_pwm(EL_CH);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Test EL retract
    gpio_set_level(s_cfg.el_dir_pin, 0);
    start_pwm(EL_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(SELF_TEST_PULSE_MS));
    stop_pwm(EL_CH);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ Self-test complete");
    ESP_LOGI(TAG, "  Both motors pulsed in both directions");
    ESP_LOGI(TAG, "  Verify physically that motors responded");
    ESP_LOGI(TAG, "");
    
    return true;
}

/*
    Get move statistics.
*/
void motor_get_stats(motor_stats_t *stats){
    if (stats) {
        *stats = s_stats;
    }
}