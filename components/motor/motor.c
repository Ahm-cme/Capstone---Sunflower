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

    Assumptions:
    - Linear mapping angle↔stroke (sufficient for solar tracking accuracy)
    - Conservative timing (85% of calculated time) to prevent overshoot
    - Mechanical limits enforced by angle clamping

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
    - W/E: clamping and failures
*/

#define TAG "MOTOR"

// Safety factor: use 85% of calculated time to prevent overshoot
#define TIMING_SAFETY_FACTOR 0.85

// Minimum safety buffer added to all moves (ms)
#define MIN_SAFETY_BUFFER_MS 100

// Internal state: keep a copy of config and define LEDC channel assignments
static motor_cfg_t s_cfg;
static ledc_channel_t AZ_CH = LEDC_CHANNEL_0;  // Azimuth actuator PWM
static ledc_channel_t EL_CH = LEDC_CHANNEL_1;  // Elevation actuator PWM

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
    - Apply safety factor (85% of calculated time)
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
    // Store configuration in module-global state
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "Initializing motor system...");
    ESP_LOGI(TAG, "  Stroke: %.1f mm @ %.2f mm/s", s_cfg.stroke_mm, s_cfg.speed_mm_per_s);
    ESP_LOGI(TAG, "  Timing safety: %.0f%% + %d ms buffer", 
             TIMING_SAFETY_FACTOR * 100, MIN_SAFETY_BUFFER_MS);
    ESP_LOGI(TAG, "  AZ limits: 0° to %.1f° (PWM=%d DIR=%d)", 
             s_cfg.max_az_deg, s_cfg.az_pwm_pin, s_cfg.az_dir_pin);
    ESP_LOGI(TAG, "  EL limits: %.1f° to %.1f° (PWM=%d DIR=%d)", 
             s_cfg.min_el_deg, s_cfg.max_el_deg, s_cfg.el_pwm_pin, s_cfg.el_dir_pin);

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
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "LEDC timer configured: 5 kHz, 13-bit resolution");

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
        ESP_LOGE(TAG, "AZ PWM channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

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
        ESP_LOGE(TAG, "EL PWM channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "PWM channels configured: AZ=ch%d, EL=ch%d", (int)AZ_CH, (int)EL_CH);

    // Configure DIR pins as GPIO outputs (default low)
    ret = gpio_set_direction(s_cfg.az_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AZ DIR pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_set_direction(s_cfg.el_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EL DIR pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize DIR pins to known state (retract direction)
    gpio_set_level(s_cfg.az_dir_pin, 0);
    gpio_set_level(s_cfg.el_dir_pin, 0);
    ESP_LOGD(TAG, "DIR pins initialized: AZ=%d→0, EL=%d→0", s_cfg.az_dir_pin, s_cfg.el_dir_pin);

    ESP_LOGI(TAG, "Motor initialization complete (conservative)");
    return ESP_OK;
}

/*
    Move AZ to target angle (deg). Applies 0..max_az_deg clamp, sets DIR,
    runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
*/
void motor_move_az(double current_deg, double target_deg){
    ESP_LOGI(TAG, "AZ move requested: %.2f° → %.2f°", current_deg, target_deg);

    // Apply safety limits to target angle
    double clamped_target = target_deg;
    if (target_deg > s_cfg.max_az_deg) {
        clamped_target = s_cfg.max_az_deg;
        ESP_LOGW(TAG, "AZ target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_az_deg);
    }
    if (target_deg < 0.0) {
        clamped_target = 0.0;
        ESP_LOGW(TAG, "AZ target %.2f° below min 0°, clamped", target_deg);
    }

    // Convert angles to actuator stroke positions
    double current_mm = angle_to_mm(current_deg, s_cfg.max_az_deg);
    double target_mm = angle_to_mm(clamped_target, s_cfg.max_az_deg);
    
    // Determine direction: extend if target > current, retract otherwise
    int dir_level = (target_mm > current_mm) ? 1 : 0;
    const char* dir_name = dir_level ? "EXTEND" : "RETRACT";
    
    // Set direction pin before starting PWM
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    ESP_LOGD(TAG, "AZ direction set: %s (DIR pin=%d)", dir_name, dir_level);

    // Calculate conservative move time
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    // Log the complete move plan
    ESP_LOGI(TAG, "AZ executing: %.2f°→%.2f° (%.1f→%.1f mm) %s for %" PRIu32 " ms (conservative)",
             current_deg, target_deg, current_mm, target_mm,
             (target_mm > current_mm) ? "EXTEND" : "RETRACT", move_ms);

    // Execute the move at full speed with conservative timing
    start_pwm(AZ_CH, 8191);                     // 100% duty
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Conservative duration
    stop_pwm(AZ_CH);                            // Stop PWM, actuator coasts
    
    ESP_LOGI(TAG, "AZ move complete (stopped at ~%.1f°)", clamped_target);
}

/*
    Move EL to target angle (deg). Applies min_el_deg..max_el_deg clamp,
    sets DIR, runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
*/
void motor_move_el(double current_deg, double target_deg){
    ESP_LOGI(TAG, "EL move requested: %.2f° → %.2f°", current_deg, target_deg);

    // Apply elevation-specific safety limits
    double clamped_target = target_deg;
    if (target_deg > s_cfg.max_el_deg) {
        clamped_target = s_cfg.max_el_deg;
        ESP_LOGW(TAG, "EL target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_el_deg);
    }
    if (target_deg < s_cfg.min_el_deg) {
        clamped_target = s_cfg.min_el_deg;
        ESP_LOGW(TAG, "EL target %.2f° below min %.2f°, clamped", target_deg, s_cfg.min_el_deg);
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
    ESP_LOGD(TAG, "EL direction set: %s (DIR pin=%d)", dir_name, dir_level);

    // Calculate conservative timing
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    ESP_LOGI(TAG, "EL executing: %.2f°→%.2f° (%.1f→%.1f mm) %s for %" PRIu32 " ms (conservative)",
             current_deg, target_deg, current_mm, target_mm,
             (target_mm > current_mm) ? "EXTEND" : "RETRACT", move_ms);

    // Execute the move
    start_pwm(EL_CH, 8191);                     // 100% duty
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Conservative duration
    stop_pwm(EL_CH);                            // Stop
    
    ESP_LOGI(TAG, "EL move complete (stopped at ~%.1f°)", clamped_target);
}

/*
    Raw timed AZ run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
*/
void motor_run_az_ms(int dir_level, uint32_t ms){
    ESP_LOGI(TAG, "AZ timed run: DIR=%d for %" PRIu32 " ms (homing mode)", dir_level, ms);
    ESP_LOGW(TAG, "Running AZ in HOMING mode - will reach mechanical stop!");
    
    // Set direction immediately
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    
    // Run at full power for specified duration (no safety factor for homing)
    start_pwm(AZ_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(ms));
    stop_pwm(AZ_CH);
    
    ESP_LOGI(TAG, "AZ timed run complete");
}

/*
    Raw timed EL run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
*/
void motor_run_el_ms(int dir_level, uint32_t ms){
    ESP_LOGI(TAG, "EL timed run: DIR=%d for %" PRIu32 " ms (homing mode)", dir_level, ms);
    ESP_LOGW(TAG, "Running EL in HOMING mode - will reach mechanical stop!");
    
    // Set direction
    gpio_set_level(s_cfg.el_dir_pin, dir_level);
    
    // Execute timed run (no safety factor for homing)
    start_pwm(EL_CH, 8191);
    vTaskDelay(pdMS_TO_TICKS(ms));
    stop_pwm(EL_CH);
    
    ESP_LOGI(TAG, "EL timed run complete");
}

/*
    Park sequence: move AZ, then EL to specified park angles (sequential to reduce current).
*/
void motor_park(double park_az_deg, double park_el_deg, double cur_az, double cur_el){
    ESP_LOGI(TAG, "Parking: AZ %.2f°→%.2f°, EL %.2f°→%.2f° (conservative timing)", 
             cur_az, park_az_deg, cur_el, park_el_deg);
    
    // Move axes sequentially (not simultaneously to reduce peak current)
    motor_move_az(cur_az, park_az_deg);
    vTaskDelay(pdMS_TO_TICKS(500));             // Brief pause between moves
    motor_move_el(cur_el, park_el_deg);
    
    ESP_LOGI(TAG, "Park sequence complete");
}

/*
    Emergency stop: immediately stop PWM on both channels (DIR levels unchanged).
*/
void motor_stop_all(void){
    ESP_LOGI(TAG, "EMERGENCY STOP - halting all motor PWM");
    stop_pwm(AZ_CH);
    stop_pwm(EL_CH);
    ESP_LOGI(TAG, "All motors stopped (DIR pins unchanged)");
}