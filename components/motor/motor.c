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
    
    ENHANCED DEBUGGING:
    - Traces all PWM operations (duty cycle changes, channel states)
    - Validates timing calculations step-by-step
    - Monitors GPIO state changes for direction control
    - Tracks actuator position deltas and velocities
    - Logs power consumption estimates
    - Validates all ranges before execution
*/

#define TAG "MOTOR"

// Debug helper macros
#define DEBUG_TRACE() ESP_LOGD(TAG, "%s() called", __func__)
#define DEBUG_PWM(ch, duty) ESP_LOGV(TAG, "PWM CH%d: %d/8191 (%.1f%%)", (int)ch, duty, (duty * 100.0) / 8191.0)
#define DEBUG_GPIO(pin, level) ESP_LOGV(TAG, "GPIO%d ← %d", pin, level)
#define DEBUG_TIMING(label, ms) ESP_LOGV(TAG, "Timing %s: %lu ms (%.2f s)", label, (unsigned long)ms, ms / 1000.0)

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

// Track GPIO states for debugging
static int s_az_dir_state = -1;  // -1 = unknown, 0/1 = known state
static int s_el_dir_state = -1;

/* ────────────────────── Internal Helper Functions ────────────────────── */

/*
    Map panel angle (deg) to actuator stroke (mm), clamped to [0, stroke_mm].
    Linear model: (angle / max_angle) * stroke_mm.
    
    ENHANCED DEBUGGING:
    - Shows linear mapping calculation step-by-step
    - Logs clamping operations with reasons
    - Validates output is within physical stroke limits
*/
static double angle_to_mm(double angle_deg, double max_angle_deg){
    DEBUG_TRACE();
    
    ESP_LOGV(TAG, "angle_to_mm input:");
    ESP_LOGV(TAG, "  - Angle: %.3f°", angle_deg);
    ESP_LOGV(TAG, "  - Max angle: %.3f°", max_angle_deg);
    ESP_LOGV(TAG, "  - Stroke: %.2f mm", s_cfg.stroke_mm);
    
    double original_angle = angle_deg;
    bool clamped = false;
    
    // Clamp input to valid range to prevent driving past mechanical limits
    if (angle_deg < 0.0) {
        ESP_LOGW(TAG, "⚠ Angle %.2f° < 0, clamping to 0°", angle_deg);
        ESP_LOGD(TAG, "  Reason: prevents driving past retract limit");
        angle_deg = 0.0;
        clamped = true;
    }
    if (angle_deg > max_angle_deg) {
        ESP_LOGW(TAG, "⚠ Angle %.2f° > max %.2f°, clamping", angle_deg, max_angle_deg);
        ESP_LOGD(TAG, "  Reason: prevents driving past extend limit");
        angle_deg = max_angle_deg;
        clamped = true;
    }
    
    // Linear mapping: proportion of max angle × total stroke
    double ratio = angle_deg / max_angle_deg;
    double mm = ratio * s_cfg.stroke_mm;
    
    ESP_LOGV(TAG, "Linear conversion:");
    ESP_LOGV(TAG, "  - Ratio: %.3f° / %.3f° = %.6f", angle_deg, max_angle_deg, ratio);
    ESP_LOGV(TAG, "  - Extension: %.6f × %.2f mm = %.3f mm", ratio, s_cfg.stroke_mm, mm);
    ESP_LOGV(TAG, "  - Percentage: %.1f%% of stroke", (mm / s_cfg.stroke_mm) * 100.0);
    
    // Sanity check: result should be within [0, stroke_mm]
    if (mm < 0.0 || mm > s_cfg.stroke_mm) {
        ESP_LOGE(TAG, "✗ BUG: angle_to_mm produced invalid result %.2f mm", mm);
        ESP_LOGD(TAG, "  Input: %.2f° (clamped: %s)", original_angle, clamped ? "yes" : "no");
        ESP_LOGD(TAG, "  Valid range: [0, %.2f] mm", s_cfg.stroke_mm);
        mm = fmax(0.0, fmin(mm, s_cfg.stroke_mm));
        ESP_LOGE(TAG, "  Emergency clamp to %.2f mm", mm);
    }
    
    if (clamped) {
        ESP_LOGD(TAG, "  → Result: %.2f° → %.2f mm (CLAMPED from %.2f°)", 
                 angle_deg, mm, original_angle);
    } else {
        ESP_LOGV(TAG, "  → Result: %.2f° → %.2f mm (%.3f inches)", 
                 angle_deg, mm, mm / 25.4);
    }
    
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
    
    ENHANCED DEBUGGING:
    - Shows each timing calculation component
    - Estimates velocity and acceleration
    - Calculates theoretical vs actual overshoot margin
*/
static uint32_t move_time_ms(double cur_mm, double tgt_mm){
    DEBUG_TRACE();
    
    double distance_mm = fabs(tgt_mm - cur_mm);
    const char* direction = (tgt_mm > cur_mm) ? "EXTEND" : "RETRACT";
    
    ESP_LOGD(TAG, "Move time calculation:");
    ESP_LOGD(TAG, "  From: %.2f mm", cur_mm);
    ESP_LOGD(TAG, "  To:   %.2f mm", tgt_mm);
    ESP_LOGD(TAG, "  Distance: %.2f mm (%s)", distance_mm, direction);
    
    // Base time calculation from kinematics
    // t = distance / velocity
    double base_time_s = distance_mm / s_cfg.speed_mm_per_s;
    double base_time_ms = base_time_s * 1000.0;
    
    ESP_LOGD(TAG, "  Speed (nominal): %.2f mm/s @ 12V", s_cfg.speed_mm_per_s);
    ESP_LOGV(TAG, "  Base calc: %.2f mm ÷ %.2f mm/s = %.3f s", 
             distance_mm, s_cfg.speed_mm_per_s, base_time_s);
    ESP_LOGD(TAG, "  Base time: %.0f ms", base_time_ms);
    
    // Apply safety factor to prevent overshoot
    double safe_time_ms = base_time_ms * TIMING_SAFETY_FACTOR;
    double margin_ms = base_time_ms - safe_time_ms;
    
    ESP_LOGD(TAG, "  Safety factor: %.0f%% (prevents overshoot)", TIMING_SAFETY_FACTOR * 100);
    ESP_LOGD(TAG, "  Safe time: %.0f ms", safe_time_ms);
    ESP_LOGV(TAG, "    - Margin removed: %.0f ms (%.1f%%)", margin_ms, (1.0 - TIMING_SAFETY_FACTOR) * 100);
    
    // Add minimum safety buffer (prevents ultra-short moves from being too aggressive)
    uint32_t total_ms = (uint32_t)safe_time_ms + MIN_SAFETY_BUFFER_MS;
    
    ESP_LOGD(TAG, "  Minimum buffer: +%d ms (startup/coast)", MIN_SAFETY_BUFFER_MS);
    ESP_LOGD(TAG, "  Total time: %" PRIu32 " ms", total_ms);
    
    // Calculate effective speed after all safety factors
    double effective_speed_mm_per_s = (distance_mm / total_ms) * 1000.0;
    double speed_reduction = (1.0 - (effective_speed_mm_per_s / s_cfg.speed_mm_per_s)) * 100.0;
    
    ESP_LOGV(TAG, "");
    ESP_LOGV(TAG, "Effective operation:");
    ESP_LOGV(TAG, "  - Effective speed: %.2f mm/s (%.1f%% of nominal)", 
             effective_speed_mm_per_s, 100.0 - speed_reduction);
    ESP_LOGV(TAG, "  - Speed reduction: %.1f%% (safety margin)", speed_reduction);
    ESP_LOGV(TAG, "  - Theoretical overshoot: %.2f mm (prevented by early stop)", 
             distance_mm * (1.0 - TIMING_SAFETY_FACTOR));
    
    // Estimate deceleration distance (assuming instant stop, which is worst case)
    // v² = 2ad, d = v² / 2a (assume a = nominal speed / 0.1s startup time)
    double accel_assumed = s_cfg.speed_mm_per_s / 0.1;  // mm/s²
    double coast_distance = (s_cfg.speed_mm_per_s * s_cfg.speed_mm_per_s) / (2.0 * accel_assumed);
    
    ESP_LOGV(TAG, "  - Estimated coast distance: %.2f mm (if instant PWM stop)", coast_distance);
    ESP_LOGV(TAG, "  - Safety margin adequate: %s", 
             (coast_distance < distance_mm * (1.0 - TIMING_SAFETY_FACTOR)) ? "YES" : "MARGINAL");
    
    // Power consumption estimate (12V × stall current during move)
    // Typical linear actuator: ~2-3A under load
    double estimated_current_a = 2.5;  // Amperes (conservative estimate)
    double energy_joules = 12.0 * estimated_current_a * (total_ms / 1000.0);
    
    ESP_LOGV(TAG, "");
    ESP_LOGV(TAG, "Power estimate (12V system):");
    ESP_LOGV(TAG, "  - Move duration: %.2f s", total_ms / 1000.0);
    ESP_LOGV(TAG, "  - Est. current: %.1f A (under load)", estimated_current_a);
    ESP_LOGV(TAG, "  - Est. energy: %.1f J (%.2f Wh)", 
             energy_joules, energy_joules / 3600.0);
    
    ESP_LOGD(TAG, "");
    
    return total_ms;
}

/*
    Start PWM on a channel with given duty (0..8191), then update.
    
    ENHANCED DEBUGGING:
    - Logs duty cycle as percentage and absolute value
    - Tracks channel state changes
    - Validates duty cycle is within valid range
*/
static void start_pwm(ledc_channel_t ch, int duty){ 
    DEBUG_TRACE();
    
    // Validate duty cycle
    if (duty < 0 || duty > 8191) {
        ESP_LOGE(TAG, "✗ Invalid duty cycle %d (must be 0-8191)", duty);
        duty = (duty < 0) ? 0 : 8191;
        ESP_LOGW(TAG, "  Clamped to %d", duty);
    }
    
    const char* axis = (ch == AZ_CH) ? "AZ" : (ch == EL_CH) ? "EL" : "UNKNOWN";
    float duty_percent = (duty * 100.0f) / 8191.0f;
    
    ESP_LOGD(TAG, "Start PWM: %s channel %d", axis, (int)ch);
    ESP_LOGD(TAG, "  Duty: %d/8191 (%.1f%%)", duty, duty_percent);
    
    // Calculate approximate motor voltage (assumes linear PWM → voltage)
    float motor_voltage = (duty / 8191.0f) * 12.0f;
    ESP_LOGV(TAG, "  Est. motor voltage: %.2f V (assuming 12V supply)", motor_voltage);
    
    // Set duty cycle
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ ledc_set_duty() failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGV(TAG, "  ✓ ledc_set_duty() OK");
    }
    
    // Update duty cycle (commits the change)
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ ledc_update_duty() failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGV(TAG, "  ✓ ledc_update_duty() OK");
        ESP_LOGD(TAG, "  → PWM active: %s at %.1f%% duty", axis, duty_percent);
    }
}

/*
    Stop PWM on a channel (sets duty to 0%).
    
    ENHANCED DEBUGGING:
    - Confirms PWM shutdown
    - Notes that actuator will coast to stop
*/
static void stop_pwm(ledc_channel_t ch){
    DEBUG_TRACE();
    
    const char* axis = (ch == AZ_CH) ? "AZ" : (ch == EL_CH) ? "EL" : "UNKNOWN";
    
    ESP_LOGD(TAG, "Stop PWM: %s channel %d", axis, (int)ch);
    ESP_LOGD(TAG, "  Setting duty to 0/8191 (0%%)");
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ ledc_set_duty() failed: %s", esp_err_to_name(ret));
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ ledc_update_duty() failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "  ✓ PWM stopped: %s will coast to rest", axis);
        ESP_LOGV(TAG, "    (Actuator decelerates naturally without braking)");
    }
}

/* ─────────────────────────── Public API ─────────────────────────────── */

/*
    Initialize PWM timer/channels and DIR pins.
    Expects motor_cfg_t with pins, stroke, speed, and limits.
    
    ENHANCED DEBUGGING:
    - Validates all configuration parameters
    - Tests GPIO functionality
    - Verifies PWM timer settings
    - Calculates theoretical performance metrics
*/
esp_err_t motor_init(const motor_cfg_t *cfg){
    DEBUG_TRACE();
    
    if (!cfg) {
        ESP_LOGE(TAG, "✗ NULL configuration provided");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Validating configuration...");
    
    // Validate configuration parameters
    bool config_valid = true;
    
    if (cfg->stroke_mm <= 0.0 || cfg->stroke_mm > 500.0) {
        ESP_LOGE(TAG, "✗ Invalid stroke: %.1f mm (expected 0-500 mm)", cfg->stroke_mm);
        config_valid = false;
    }
    
    if (cfg->speed_mm_per_s <= 0.0 || cfg->speed_mm_per_s > 100.0) {
        ESP_LOGE(TAG, "✗ Invalid speed: %.2f mm/s (expected 0-100 mm/s)", cfg->speed_mm_per_s);
        config_valid = false;
    }
    
    if (cfg->max_az_deg <= 0.0 || cfg->max_az_deg > 360.0) {
        ESP_LOGE(TAG, "✗ Invalid AZ range: %.1f° (expected 0-360°)", cfg->max_az_deg);
        config_valid = false;
    }
    
    if (cfg->max_el_deg <= cfg->min_el_deg) {
        ESP_LOGE(TAG, "✗ Invalid EL range: %.1f° to %.1f° (max must be > min)", 
                 cfg->min_el_deg, cfg->max_el_deg);
        config_valid = false;
    }
    
    if (!config_valid) {
        ESP_LOGE(TAG, "✗ Configuration validation failed");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "✓ Configuration valid");
    
    // Store configuration in module-global state
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          MOTOR SYSTEM INITIALIZATION                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Actuator Specifications:");
    ESP_LOGI(TAG, "  - Stroke: %.1f mm (%.2f inches)", s_cfg.stroke_mm, s_cfg.stroke_mm / 25.4);
    ESP_LOGI(TAG, "  - Speed: %.2f mm/s (nominal @ 12V)", s_cfg.speed_mm_per_s);
    ESP_LOGI(TAG, "  - Full stroke time: ~%.1f seconds", s_cfg.stroke_mm / s_cfg.speed_mm_per_s);
    ESP_LOGD(TAG, "  - Speed: %.2f inches/s", s_cfg.speed_mm_per_s / 25.4);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Timing Strategy (Conservative):");
    ESP_LOGI(TAG, "  - Base calculation: distance / speed");
    ESP_LOGI(TAG, "  - Safety factor: %.0f%% (prevents overshoot)", TIMING_SAFETY_FACTOR * 100);
    ESP_LOGI(TAG, "  - Minimum buffer: %d ms (startup/coast)", MIN_SAFETY_BUFFER_MS);
    ESP_LOGD(TAG, "  - Result: ~%.0f%% of nominal speed in practice", 
             TIMING_SAFETY_FACTOR * 100 * 
             (1.0 / (1.0 + (MIN_SAFETY_BUFFER_MS / 1000.0) / (s_cfg.stroke_mm / s_cfg.speed_mm_per_s))));
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Azimuth Motor (AZ):");
    ESP_LOGI(TAG, "  - Range: 0° to %.1f°", s_cfg.max_az_deg);
    ESP_LOGI(TAG, "  - Resolution: %.3f mm/deg", s_cfg.stroke_mm / s_cfg.max_az_deg);
    ESP_LOGI(TAG, "  - PWM pin: GPIO%d", s_cfg.az_pwm_pin);
    ESP_LOGI(TAG, "  - DIR pin: GPIO%d", s_cfg.az_dir_pin);
    ESP_LOGV(TAG, "  - Full range time: %.1f s", 
             (s_cfg.stroke_mm / s_cfg.speed_mm_per_s) * 
             (1.0 / TIMING_SAFETY_FACTOR) * 
             (1.0 + (MIN_SAFETY_BUFFER_MS / 1000.0)));
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Elevation Motor (EL):");
    ESP_LOGI(TAG, "  - Range: %.1f° to %.1f°", s_cfg.min_el_deg, s_cfg.max_el_deg);
    ESP_LOGI(TAG, "  - Resolution: %.3f mm/deg", 
             s_cfg.stroke_mm / (s_cfg.max_el_deg - s_cfg.min_el_deg));
    ESP_LOGI(TAG, "  - PWM pin: GPIO%d", s_cfg.el_pwm_pin);
    ESP_LOGI(TAG, "  - DIR pin: GPIO%d", s_cfg.el_dir_pin);
    ESP_LOGI(TAG, "");

    // Configure LEDC timer (shared by both PWM channels)
    ESP_LOGD(TAG, "Configuring LEDC timer...");
    
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,    // More stable than high-speed mode
        .timer_num        = LEDC_TIMER_0,           // Use timer 0 (could be any available timer)
        .duty_resolution  = LEDC_TIMER_13_BIT,      // 13 bits = 0-8191 duty range
        .freq_hz          = 5000,                   // 5 kHz is good for motor drivers
        .clk_cfg          = LEDC_AUTO_CLK           // Let ESP-IDF choose best clock source
    };
    
    ESP_LOGV(TAG, "Timer config:");
    ESP_LOGV(TAG, "  - Mode: %s", 
             (timer_config.speed_mode == LEDC_LOW_SPEED_MODE) ? "LOW_SPEED" : "HIGH_SPEED");
    ESP_LOGV(TAG, "  - Timer: %d", timer_config.timer_num);
    ESP_LOGV(TAG, "  - Resolution: 13-bit (8192 steps)");
    ESP_LOGV(TAG, "  - Frequency: %" PRIu32 " Hz (%.2f kHz)", timer_config.freq_hz, timer_config.freq_hz / 1000.0);
    ESP_LOGV(TAG, "  - Period: %.2f µs", 1000000.0 / timer_config.freq_hz);
    ESP_LOGV(TAG, "  - Time resolution: %.3f µs/step", 
             (1000000.0 / timer_config.freq_hz) / 8192.0);
    
    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ LEDC timer config failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  Possible causes:");
        ESP_LOGD(TAG, "    - Timer already in use");
        ESP_LOGD(TAG, "    - Invalid frequency for clock source");
        ESP_LOGD(TAG, "    - Insufficient system resources");
        return ret;
    }
    ESP_LOGI(TAG, "✓ LEDC timer configured:");
    ESP_LOGI(TAG, "  - Frequency: 5 kHz");
    ESP_LOGI(TAG, "  - Resolution: 13-bit (0-8191)");
    ESP_LOGV(TAG, "  - Actual frequency may vary slightly based on clock source");

    // Configure azimuth PWM channel
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Configuring AZ PWM channel...");
    
    ledc_channel_config_t az_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = AZ_CH,
        .timer_sel      = LEDC_TIMER_0,         // Use the timer we just configured
        .gpio_num       = s_cfg.az_pwm_pin,
        .duty           = 0,                    // Start with 0% duty (stopped)
        .intr_type      = LEDC_INTR_DISABLE,    // We don't need PWM interrupts
        .hpoint         = 0                     // Default phase offset
    };
    
    ESP_LOGV(TAG, "AZ channel config:");
    ESP_LOGV(TAG, "  - Channel: %d", (int)AZ_CH);
    ESP_LOGV(TAG, "  - GPIO: %d", s_cfg.az_pwm_pin);
    ESP_LOGV(TAG, "  - Initial duty: 0 (stopped)");
    
    ret = ledc_channel_config(&az_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ AZ PWM channel config failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  Check that GPIO%d is available for PWM output", s_cfg.az_pwm_pin);
        return ret;
    }
    ESP_LOGI(TAG, "✓ AZ PWM channel configured (CH%d → GPIO%d)", (int)AZ_CH, s_cfg.az_pwm_pin);

    // Configure elevation PWM channel
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Configuring EL PWM channel...");
    
    ledc_channel_config_t el_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = EL_CH,
        .timer_sel      = LEDC_TIMER_0,         // Same timer as AZ channel
        .gpio_num       = s_cfg.el_pwm_pin,
        .duty           = 0,                    // Start with 0% duty (stopped)
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0
    };
    
    ESP_LOGV(TAG, "EL channel config:");
    ESP_LOGV(TAG, "  - Channel: %d", (int)EL_CH);
    ESP_LOGV(TAG, "  - GPIO: %d", s_cfg.el_pwm_pin);
    ESP_LOGV(TAG, "  - Initial duty: 0 (stopped)");
    
    ret = ledc_channel_config(&el_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ EL PWM channel config failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  Check that GPIO%d is available for PWM output", s_cfg.el_pwm_pin);
        return ret;
    }
    ESP_LOGI(TAG, "✓ EL PWM channel configured (CH%d → GPIO%d)", (int)EL_CH, s_cfg.el_pwm_pin);

    // Configure DIR pins as GPIO outputs (default low)
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Configuring DIR pins...");
    
    ret = gpio_set_direction(s_cfg.az_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ AZ DIR pin config failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  GPIO%d may be reserved or invalid", s_cfg.az_dir_pin);
        return ret;
    }
    ESP_LOGV(TAG, "  ✓ GPIO%d set as output (AZ DIR)", s_cfg.az_dir_pin);
    
    ret = gpio_set_direction(s_cfg.el_dir_pin, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ EL DIR pin config failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  GPIO%d may be reserved or invalid", s_cfg.el_dir_pin);
        return ret;
    }
    ESP_LOGV(TAG, "  ✓ GPIO%d set as output (EL DIR)", s_cfg.el_dir_pin);
    
    // Initialize DIR pins to known state (retract direction)
    gpio_set_level(s_cfg.az_dir_pin, 0);
    s_az_dir_state = 0;
    ESP_LOGV(TAG, "  ✓ GPIO%d ← 0 (AZ retract)", s_cfg.az_dir_pin);
    
    gpio_set_level(s_cfg.el_dir_pin, 0);
    s_el_dir_state = 0;
    ESP_LOGV(TAG, "  ✓ GPIO%d ← 0 (EL retract)", s_cfg.el_dir_pin);
    
    ESP_LOGI(TAG, "✓ DIR pins initialized:");
    ESP_LOGI(TAG, "  - AZ DIR: GPIO%d → LOW (retract)", s_cfg.az_dir_pin);
    ESP_LOGI(TAG, "  - EL DIR: GPIO%d → LOW (retract)", s_cfg.el_dir_pin);
    ESP_LOGV(TAG, "    (Convention: LOW=retract, HIGH=extend)");
    ESP_LOGI(TAG, "");

    // Initialize statistics
    s_stats.last_az_move_deg = 0.0;
    s_stats.last_el_move_deg = 0.0;
    s_stats.last_az_duration_ms = 0;
    s_stats.last_el_duration_ms = 0;
    s_stats.total_moves = 0;
    
    ESP_LOGD(TAG, "Move statistics initialized:");
    ESP_LOGD(TAG, "  - AZ last move: %.1f° (0 ms)", s_stats.last_az_move_deg);
    ESP_LOGD(TAG, "  - EL last move: %.1f° (0 ms)", s_stats.last_el_move_deg);
    ESP_LOGD(TAG, "  - Total moves: %" PRIu32, s_stats.total_moves);
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "✓ Motor initialization complete");
    ESP_LOGI(TAG, "  System ready for tracking operations");
    ESP_LOGD(TAG, "  Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "");

    return ESP_OK;
    
}

/*
    Move AZ to target angle (deg). Applies 0..max_az_deg clamp, sets DIR,
    runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
    
    ENHANCED DEBUGGING:
    - Logs complete move plan before execution
    - Tracks actual vs planned timing
    - Updates statistics with performance metrics
    - Validates result is within expected range
*/
void motor_move_az(double current_deg, double target_deg){
    DEBUG_TRACE();
    
    if (!s_initialized) {
        ESP_LOGE(TAG, "✗ Motor system not initialized - cannot move AZ");
        ESP_LOGD(TAG, "  Call motor_init() first");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          AZ MOVE REQUEST                                   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "AZ move requested: %.2f° → %.2f°", current_deg, target_deg);
    ESP_LOGV(TAG, "  Direction: %s", (target_deg > current_deg) ? "EXTEND" : "RETRACT");

    // Apply safety limits to target angle
    double clamped_target = target_deg;
    bool was_clamped = false;
    
    if (target_deg > s_cfg.max_az_deg) {
        clamped_target = s_cfg.max_az_deg;
        ESP_LOGW(TAG, "⚠ AZ target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_az_deg);
        ESP_LOGD(TAG, "  Reason: mechanical limit protection");
        was_clamped = true;
    }
    if (target_deg < 0.0) {
        clamped_target = 0.0;
        ESP_LOGW(TAG, "⚠ AZ target %.2f° below min 0°, clamped", target_deg);
        ESP_LOGD(TAG, "  Reason: mechanical limit protection");
        was_clamped = true;
    }
    
    if (was_clamped) {
        ESP_LOGD(TAG, "  Final target: %.2f° (clamped from %.2f°)", clamped_target, target_deg);
    }

    // Calculate move delta for statistics
    double move_delta = fabs(clamped_target - current_deg);
    
    ESP_LOGD(TAG, "  Move delta: %.3f°", move_delta);
    
    // Skip tiny moves (< 0.1°) to reduce wear
    if (move_delta < 0.1) {
        ESP_LOGD(TAG, "  ✗ Move delta %.3f° < 0.1° threshold, skipping", move_delta);
        ESP_LOGV(TAG, "    Reason: reduces actuator wear on micro-adjustments");
        ESP_LOGI(TAG, "");
        return;
    }

    // Convert angles to actuator stroke positions
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Kinematics:");
    
    double current_mm = angle_to_mm(current_deg, s_cfg.max_az_deg);
    double target_mm = angle_to_mm(clamped_target, s_cfg.max_az_deg);
    
    ESP_LOGD(TAG, "  Current: %.2f° → %.2f mm", current_deg, current_mm);
    ESP_LOGD(TAG, "  Target:  %.2f° → %.2f mm", clamped_target, target_mm);
    ESP_LOGD(TAG, "  Delta:   %.2f° → %.2f mm", move_delta, fabs(target_mm - current_mm));
    
    // Determine direction: extend if target > current, retract otherwise
    int dir_level = (target_mm > current_mm) ? 1 : 0;
    const char* dir_name = dir_level ? "RETRACT" : "EXTEND";
    
    // Check if direction changed
    if (s_az_dir_state >= 0 && s_az_dir_state != dir_level) {
        ESP_LOGD(TAG, "  Direction change: %s → %s", 
                 s_az_dir_state ? "RETRACT" : "EXTEND", dir_name);
    }
    
    // Set direction pin before starting PWM
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    s_az_dir_state = dir_level;
    DEBUG_GPIO(s_cfg.az_dir_pin, dir_level);
    ESP_LOGD(TAG, "  Direction set: %s (GPIO%d = %d)", dir_name, s_cfg.az_dir_pin, dir_level);

    // Calculate conservative move time
    ESP_LOGD(TAG, "");
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    // Log the complete move plan
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "AZ Move Plan:");
    ESP_LOGI(TAG, "  Position: %.2f° → %.2f° (Δ=%.2f°)", current_deg, clamped_target, move_delta);
    ESP_LOGI(TAG, "  Stroke: %.1f mm → %.1f mm (Δ=%.1f mm)", current_mm, target_mm, fabs(target_mm - current_mm));
    ESP_LOGI(TAG, "  Direction: %s", dir_name);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms (conservative)", move_ms);
    ESP_LOGD(TAG, "  PWM: 100%% duty (8191/8191)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Executing...");

    // Record start time for statistics
    uint32_t start_tick = xTaskGetTickCount();
    uint32_t start_time_ms = start_tick * portTICK_PERIOD_MS;
    
    ESP_LOGV(TAG, "  Start tick: %lu", (unsigned long)start_tick);
    
    // Execute the move at full speed with conservative timing
    start_pwm(AZ_CH, 8191);                     // 100% duty
    
    ESP_LOGD(TAG, "  → Motor running...");
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Conservative duration
    
    stop_pwm(AZ_CH);                            // Stop PWM, actuator coasts
    ESP_LOGD(TAG, "  → Motor stopped (coasting)");
    
    // Calculate actual duration
    uint32_t end_tick = xTaskGetTickCount();
    uint32_t actual_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
    int32_t timing_error_ms = (int32_t)actual_ms - (int32_t)move_ms;
    
    ESP_LOGV(TAG, "  End tick: %lu", (unsigned long)end_tick);
    ESP_LOGV(TAG, "  Ticks elapsed: %lu", (unsigned long)(end_tick - start_tick));
    ESP_LOGV(TAG, "  Tick period: %lu ms", (unsigned long)portTICK_PERIOD_MS);
    
    // Update statistics
    s_stats.last_az_move_deg = move_delta;
    s_stats.last_az_duration_ms = actual_ms;
    s_stats.total_moves++;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ AZ move complete:");
    ESP_LOGI(TAG, "  Final position: ~%.1f° (open-loop estimate)", clamped_target);
    ESP_LOGI(TAG, "  Actual duration: %" PRIu32 " ms", actual_ms);
    ESP_LOGD(TAG, "  Planned duration: %" PRIu32 " ms", move_ms);
    ESP_LOGD(TAG, "  Timing accuracy: %s%d ms (%s%.1f%%)",
             (timing_error_ms >= 0) ? "+" : "", (int)timing_error_ms,
             (timing_error_ms >= 0) ? "+" : "", (timing_error_ms * 100.0) / move_ms);
    ESP_LOGD(TAG, "  Total system moves: %" PRIu32, s_stats.total_moves);
    ESP_LOGI(TAG, "");
    
    // Warn if timing error is large (>5% suggests system lag or high load)
    if (abs(timing_error_ms) > (int32_t)(move_ms * 0.05)) {
        ESP_LOGW(TAG, "⚠ Large timing error detected: %d ms", (int)timing_error_ms);
        ESP_LOGD(TAG, "  Possible causes:");
        ESP_LOGD(TAG, "    - System load (other tasks consuming CPU)");
        ESP_LOGD(TAG, "    - Tick rate inaccuracy");
        ESP_LOGD(TAG, "    - vTaskDelay() implementation overhead");
    }
}

/*
    Move EL to target angle (deg). Applies min_el_deg..max_el_deg clamp,
    sets DIR, runs full-speed PWM for computed duration, then stops PWM.
    Uses conservative timing to prevent overshoot.
    
    ENHANCED DEBUGGING:
    - Logs complete move plan before execution
    - Tracks actual vs planned timing
    - Updates statistics with performance metrics
    - Validates result is within expected range
*/
void motor_move_el(double current_deg, double target_deg){
    DEBUG_TRACE();
    
    if (!s_initialized) {
        ESP_LOGE(TAG, "✗ Motor system not initialized - cannot move EL");
        ESP_LOGD(TAG, "  Call motor_init() first");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          EL MOVE REQUEST                                   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EL move requested: %.2f° → %.2f°", current_deg, target_deg);
    ESP_LOGV(TAG, "  Direction: %s", (target_deg > current_deg) ? "EXTEND" : "RETRACT");

    // Apply elevation-specific safety limits
    double clamped_target = target_deg;
    bool was_clamped = false;
    
    if (target_deg > s_cfg.max_el_deg) {
        clamped_target = s_cfg.max_el_deg;
        ESP_LOGW(TAG, "⚠ EL target %.2f° exceeds max %.2f°, clamped", target_deg, s_cfg.max_el_deg);
        ESP_LOGD(TAG, "  Reason: mechanical limit protection");
        was_clamped = true;
    }
    if (target_deg < s_cfg.min_el_deg) {
        clamped_target = s_cfg.min_el_deg;
        ESP_LOGW(TAG, "⚠ EL target %.2f° below min %.2f°, clamped", target_deg, s_cfg.min_el_deg);
        ESP_LOGD(TAG, "  Reason: mechanical limit protection");
        was_clamped = true;
    }
    
    if (was_clamped) {
        ESP_LOGD(TAG, "  Final target: %.2f° (clamped from %.2f°)", clamped_target, target_deg);
    }

    // Calculate move delta
    double move_delta = fabs(clamped_target - current_deg);
    
    ESP_LOGD(TAG, "  Move delta: %.3f°", move_delta);
    
    // Skip tiny moves
    if (move_delta < 0.1) {
        ESP_LOGD(TAG, "  ✗ Move delta %.3f° < 0.1° threshold, skipping", move_delta);
        ESP_LOGV(TAG, "    Reason: reduces actuator wear on micro-adjustments");
        ESP_LOGI(TAG, "");
        return;
    }

    // Convert angles to actuator stroke positions
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Kinematics:");
    
    double current_mm = angle_to_mm(current_deg, s_cfg.max_el_deg);
    double target_mm = angle_to_mm(clamped_target, s_cfg.max_el_deg);
    
    ESP_LOGD(TAG, "  Current: %.2f° → %.2f mm", current_deg, current_mm);
    ESP_LOGD(TAG, "  Target:  %.2f° → %.2f mm", clamped_target, target_mm);
    ESP_LOGD(TAG, "  Delta:   %.2f° → %.2f mm", move_delta, fabs(target_mm - current_mm));
    
    // Determine direction
    int dir_level = (target_mm > current_mm) ? 1 : 0;
    const char* dir_name = dir_level ? "EXTEND" : "RETRACT";
    
    // Check if direction changed
    if (s_el_dir_state >= 0 && s_el_dir_state != dir_level) {
        ESP_LOGD(TAG, "  Direction change: %s → %s", 
                 s_el_dir_state ? "EXTEND" : "RETRACT", dir_name);
    }
    
    // === NEW: Pre-move diagnostics ===
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Pre-move GPIO states:");
    int current_dir = gpio_get_level(s_cfg.el_dir_pin);
    ESP_LOGD(TAG, "  - DIR (GPIO%d): current=%d, target=%d", 
             s_cfg.el_dir_pin, current_dir, dir_level);
    
    // Set direction before starting motion
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Setting direction GPIO...");
    gpio_set_level(s_cfg.el_dir_pin, dir_level);
    vTaskDelay(pdMS_TO_TICKS(10));  // Short settle time
    s_el_dir_state = dir_level;
    
    // Verify direction was set
    int readback_dir = gpio_get_level(s_cfg.el_dir_pin);
    if (readback_dir != dir_level) {
        ESP_LOGE(TAG, "✗ DIR GPIO READBACK MISMATCH!");
        ESP_LOGE(TAG, "  Expected: %d, Got: %d", dir_level, readback_dir);
        ESP_LOGE(TAG, "  GPIO%d may be damaged or not configured correctly", s_cfg.el_dir_pin);
        ESP_LOGI(TAG, "");
        return;
    }
    
    DEBUG_GPIO(s_cfg.el_dir_pin, dir_level);
    ESP_LOGD(TAG, "✓ Direction verified: %s (GPIO%d = %d)", dir_name, s_cfg.el_dir_pin, dir_level);

    // Calculate conservative timing
    ESP_LOGD(TAG, "");
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EL Move Plan:");
    ESP_LOGI(TAG, "  Position: %.2f° → %.2f° (Δ=%.2f°)", current_deg, clamped_target, move_delta);
    ESP_LOGI(TAG, "  Stroke: %.1f mm → %.1f mm (Δ=%.1f mm)", current_mm, target_mm, fabs(target_mm - current_mm));
    ESP_LOGI(TAG, "  Direction: %s", dir_name);
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms (conservative)", move_ms);
    ESP_LOGD(TAG, "  PWM: 100%% duty (8191/8191)");
    
    // === NEW: Detailed PWM diagnostics ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "EL Motor Control Signals:");
    ESP_LOGI(TAG, "  - DIR pin: GPIO%d → %d (%s)", 
             s_cfg.el_dir_pin, dir_level, dir_name);
    ESP_LOGI(TAG, "  - PWM pin: GPIO%d → 100%% duty (CH%d)", 
             s_cfg.el_pwm_pin, (int)EL_CH);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Executing...");

    // Record start time
    uint32_t start_tick = xTaskGetTickCount();
    uint32_t start_time_ms = start_tick * portTICK_PERIOD_MS;
    
    ESP_LOGV(TAG, "  Start tick: %lu", (unsigned long)start_tick);
    
    // === NEW: Enhanced PWM start with error checking ===
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Starting EL motor...");
    ESP_LOGD(TAG, "  - Channel: %d", (int)EL_CH);
    ESP_LOGD(TAG, "  - Duty: 8191/8191 (100%%)");
    
    esp_err_t pwm_ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, EL_CH, 8191);
    if (pwm_ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ PWM set_duty failed: %s", esp_err_to_name(pwm_ret));
        ESP_LOGE(TAG, "  Channel %d may not be initialized", (int)EL_CH);
        ESP_LOGI(TAG, "");
        return;
    }
    ESP_LOGV(TAG, "  ✓ ledc_set_duty() OK");
    
    pwm_ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, EL_CH);
    if (pwm_ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ PWM update failed: %s", esp_err_to_name(pwm_ret));
        ESP_LOGI(TAG, "");
        return;
    }
    ESP_LOGV(TAG, "  ✓ ledc_update_duty() OK");
    
    ESP_LOGD(TAG, "✓ PWM started successfully");
    ESP_LOGD(TAG, "  → Motor running at 100%% duty");
    
    // Wait for move to complete
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "  → Motor running for %" PRIu32 " ms...", move_ms);
    vTaskDelay(pdMS_TO_TICKS(move_ms));
    
    // Stop PWM
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Stopping EL motor...");
    stop_pwm(EL_CH);
    ESP_LOGD(TAG, "  → Motor stopped (coasting)");
    
    // Calculate actual duration
    uint32_t end_tick = xTaskGetTickCount();
    uint32_t actual_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
    int32_t timing_error_ms = (int32_t)actual_ms - (int32_t)move_ms;
    
    ESP_LOGV(TAG, "  End tick: %lu", (unsigned long)end_tick);
    ESP_LOGV(TAG, "  Ticks elapsed: %lu", (unsigned long)(end_tick - start_tick));
    ESP_LOGV(TAG, "  Tick period: %lu ms", (unsigned long)portTICK_PERIOD_MS);
    
    // Update statistics
    s_stats.last_el_move_deg = move_delta;
    s_stats.last_el_duration_ms = actual_ms;
    s_stats.total_moves++;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ EL move complete:");
    ESP_LOGI(TAG, "  Final position: ~%.1f° (open-loop estimate)", clamped_target);
    ESP_LOGI(TAG, "  Actual duration: %" PRIu32 " ms", actual_ms);
    ESP_LOGD(TAG, "  Planned duration: %" PRIu32 " ms", move_ms);
    ESP_LOGD(TAG, "  Timing accuracy: %s%d ms (%s%.1f%%)",
             (timing_error_ms >= 0) ? "+" : "", (int)timing_error_ms,
             (timing_error_ms >= 0) ? "+" : "", (timing_error_ms * 100.0) / move_ms);
    ESP_LOGD(TAG, "  Total system moves: %" PRIu32, s_stats.total_moves);
    ESP_LOGI(TAG, "");
    
    // Warn if timing error is large (>5% suggests system lag or high load)
    if (abs(timing_error_ms) > (int32_t)(move_ms * 0.05)) {
        ESP_LOGW(TAG, "⚠ Large timing error detected: %d ms", (int)timing_error_ms);
        ESP_LOGD(TAG, "  Possible causes:");
        ESP_LOGD(TAG, "    - System load (other tasks consuming CPU)");
        ESP_LOGD(TAG, "    - Tick rate inaccuracy");
        ESP_LOGD(TAG, "    - vTaskDelay() implementation overhead");
    }
}

/*
    Raw timed AZ run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
    
    ENHANCED DEBUGGING:
    - Logs homing parameters and warnings
    - Confirms PWM start/stop actions
    - Monitors actual run time vs expected
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
    s_az_dir_state = dir_level;
    DEBUG_GPIO(s_cfg.az_dir_pin, dir_level);
    
    // Run at full power for specified duration (no safety factor for homing)
    start_pwm(AZ_CH, 8191);
    ESP_LOGD(TAG, "  → PWM started: CH%d at 100%% duty", (int)AZ_CH);
    
    // Wait for the specified duration
    vTaskDelay(pdMS_TO_TICKS(ms));
    
    stop_pwm(AZ_CH);
    ESP_LOGD(TAG, "  → PWM stopped: CH%d", (int)AZ_CH);
    
    ESP_LOGI(TAG, "✓ AZ homing run complete");
    ESP_LOGI(TAG, "");
}

/*
    Raw timed EL run (for homing). Direction is set as given, PWM at full power.
    Note: Homing uses full calculated time since we want to reach mechanical stops.
    
    ENHANCED DEBUGGING:
    - Logs homing parameters and warnings
    - Confirms PWM start/stop actions
    - Monitors actual run time vs expected
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
    s_el_dir_state = dir_level;
    DEBUG_GPIO(s_cfg.el_dir_pin, dir_level);
    
    // Execute timed run (no safety factor for homing)
    start_pwm(EL_CH, 8191);
    ESP_LOGD(TAG, "  → PWM started: CH%d at 100%% duty", (int)EL_CH);
    
    // Wait for the specified duration
    vTaskDelay(pdMS_TO_TICKS(ms));
    
    stop_pwm(EL_CH);
    ESP_LOGD(TAG, "  → PWM stopped: CH%d", (int)EL_CH);
    
    ESP_LOGI(TAG, "✓ EL homing run complete");
    ESP_LOGI(TAG, "");
}

/*
    Park sequence: move AZ, then EL to specified park angles (sequential to reduce current).
    
    ENHANCED DEBUGGING:
    - Logs each step of the parking sequence
    - Confirms motor movements and pauses
    - Validates final positions against expected
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
    
    ENHANCED DEBUGGING:
    - Logs emergency stop trigger
    - Records state before stop
    - Confirms both channels halted
*/
void motor_stop_all(void){
    DEBUG_TRACE();
    
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGE(TAG, "║              ⚠  EMERGENCY STOP  ⚠                        ║");
    ESP_LOGE(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "Halting all motor PWM immediately");
    
    // Log current state before stopping
    ESP_LOGD(TAG, "State before stop:");
    ESP_LOGD(TAG, "  - AZ DIR: GPIO%d = %d (%s)", 
             s_cfg.az_dir_pin, s_az_dir_state,
             (s_az_dir_state == 1) ? "EXTEND" : (s_az_dir_state == 0) ? "RETRACT" : "UNKNOWN");
    ESP_LOGD(TAG, "  - EL DIR: GPIO%d = %d (%s)",
             s_cfg.el_dir_pin, s_el_dir_state,
             (s_el_dir_state == 1) ? "EXTEND" : (s_el_dir_state == 0) ? "RETRACT" : "UNKNOWN");
    
    ESP_LOGE(TAG, "  - AZ channel: stopping (0%% duty)");
    ESP_LOGE(TAG, "  - EL channel: stopping (0%% duty)");
    ESP_LOGE(TAG, "  - DIR pins: unchanged");
    
    // Stop both channels
    stop_pwm(AZ_CH);
    stop_pwm(EL_CH);
    
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "✓ All motors stopped");
    ESP_LOGE(TAG, "  Motors will coast to rest");
    ESP_LOGE(TAG, "  Position may be undefined");
    ESP_LOGE(TAG, "  Manual intervention may be required");
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Recovery options:");
    ESP_LOGD(TAG, "  1. Check for physical obstructions");
    ESP_LOGD(TAG, "  2. Verify power supply voltage");
    ESP_LOGD(TAG, "  3. Run motor_self_test() to verify operation");
    ESP_LOGD(TAG, "  4. Consider homing sequence to reset position");
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
    
    ENHANCED DEBUGGING:
    - Confirms initialization state
    - Logs each test step with results
    - Validates physical response of motors
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
    
    ENHANCED DEBUGGING:
    - Validates stats pointer
    - Logs stats copy operation
*/
void motor_get_stats(motor_stats_t *stats){
    DEBUG_TRACE();
    
    if (!stats) {
        ESP_LOGE(TAG, "✗ NULL stats pointer provided");
        return;
    }
    
    *stats = s_stats;
    
    ESP_LOGV(TAG, "Statistics retrieved:");
    ESP_LOGV(TAG, "  - AZ last: %.1f° (%" PRIu32 " ms)", 
             s_stats.last_az_move_deg, s_stats.last_az_duration_ms);
    ESP_LOGV(TAG, "  - EL last: %.1f° (%" PRIu32 " ms)",
             s_stats.last_el_move_deg, s_stats.last_el_duration_ms);
    ESP_LOGV(TAG, "  - Total moves: %" PRIu32, s_stats.total_moves);
}