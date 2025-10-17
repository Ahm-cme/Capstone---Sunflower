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
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Implementation Notes & Debugging Hints                               │
    │                                                                       │
    │ Common issues and how to debug them:                                  │
    │                                                                       │
    │ 1) Actuator moves wrong direction:                                    │
    │    - Check DIR pin polarity in your wiring                           │
    │    - Swap DIR logic in homing config, not here                       │
    │    - Use multimeter to verify DIR pin voltage during moves           │
    │                                                                       │
    │ 2) Actuator moves too slow/fast:                                     │
    │    - Check battery voltage (12V nominal, but varies 10-15V)          │
    │    - Measure actual speed with ruler + stopwatch                     │
    │    - Adjust speed_mm_per_s in config                                 │
    │                                                                       │
    │ 3) Actuator doesn't move at all:                                     │
    │    - Check PWM signal with oscilloscope                              │
    │    - Verify motor driver power supply                                │
    │    - Test with external PWM source                                   │
    │                                                                       │
    │ 4) Tracking accuracy degrades over time:                             │
    │    - Normal! Open-loop control accumulates error                     │
    │    - Daily homing should reset this                                  │
    │    - If homing fails, check mechanical stops                        │
    │                                                                       │
    │ 5) PWM frequency too high/low:                                       │
    │    - 5 kHz is good for most motor drivers                           │
    │    - Some drivers prefer 1-10 kHz range                             │
    │    - Higher frequency = smoother but more switching loss             │
    └───────────────────────────────────────────────────────────────────────┘

    The logging levels help during development:

ESP_LOGV: Verbose details (usually disabled)
ESP_LOGD: Debug info (enable during development)
ESP_LOGI: Important operations (always enabled)
ESP_LOGW: Warnings about limit clamping
ESP_LOGE: Serious errors that prevent operation
*/

#define TAG "MOTOR"

// Internal state: keep a copy of config and define LEDC channel assignments
static motor_cfg_t s_cfg;
static ledc_channel_t AZ_CH = LEDC_CHANNEL_0;  // Azimuth actuator PWM
static ledc_channel_t EL_CH = LEDC_CHANNEL_1;  // Elevation actuator PWM

/* ────────────────────── Internal Helper Functions ────────────────────── */

/*
    Convert panel angle (degrees) to actuator position (millimeters).
    
    This uses a simple linear mapping:
      actuator_mm = (angle_deg / max_angle_deg) * total_stroke_mm
    
    Real-world linkages are usually nonlinear (sine/cosine functions), but
    linear approximation works well enough for solar tracking where we only
    need ±2-3° accuracy.
    
    Parameters:
    - angle_deg: desired angle in panel coordinate system
    - max_angle_deg: maximum angle corresponding to full actuator extension
    
    Returns: position along actuator stroke in millimeters
    
    Edge cases:
    - angle_deg < 0: clamps to 0 mm (fully retracted)
    - angle_deg > max: clamps to stroke_mm (fully extended)
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
    Calculate time (ms) needed to move between two actuator positions.
    
    Formula: time = distance / speed + safety_buffer
    
    The safety buffer (300 ms) accounts for:
    - Acceleration/deceleration ramps (actuators aren't instant)
    - Speed variations due to battery voltage or mechanical load
    - vTaskDelay() timing quantization (usually ±10 ms)
    - Better to overshoot slightly than undershoot
    
    Parameters:
    - cur_mm, tgt_mm: current and target positions along actuator stroke
    
    Returns: time in milliseconds to complete the move
*/
static uint32_t move_time_ms(double cur_mm, double tgt_mm){
    double distance_mm = fabs(tgt_mm - cur_mm);
    
    // Base time calculation from kinematics
    double base_time_ms = (distance_mm / s_cfg.speed_mm_per_s) * 1000.0;
    
    // Add safety buffer for real-world variations
    uint32_t total_ms = (uint32_t)base_time_ms + 300;
    
    ESP_LOGD(TAG, "Move %.2f mm @ %.2f mm/s: base=%.0f ms + 300 ms buffer = %" PRIu32 " ms total",
             distance_mm, s_cfg.speed_mm_per_s, base_time_ms, total_ms);
    return total_ms;
}

/*
    Start PWM output on the specified LEDC channel.
    
    We use 13-bit PWM resolution (0-8191 range) and typically run at
    50% duty cycle (4096) which provides good torque without excessive
    power consumption.
    
    Parameters:
    - ch: LEDC channel (AZ_CH or EL_CH)
    - duty: PWM duty cycle (0 = 0%, 8191 = 100%)
    
    UPDATED: Always use maximum duty cycle for fastest movement
*/
static void start_pwm(ledc_channel_t ch, int duty){
    // FORCE MAXIMUM SPEED: Override duty parameter with full 13-bit resolution
    // duty = 8191 corresponds to 100% PWM (255/255 in 8-bit terms)
    duty = 8191;  // Maximum duty cycle for 13-bit LEDC (2^13 - 1)
    
    ESP_LOGD(TAG, "Starting PWM ch=%d duty=%d (100%% - MAX SPEED)", ch, duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/*
    Stop PWM output on the specified LEDC channel.
    
    Sets duty cycle to 0%, allowing the actuator to coast to a stop.
    DIR pin level is unchanged, so it still shows the last commanded direction.
*/
static void stop_pwm(ledc_channel_t ch){
    ESP_LOGD(TAG, "Stop PWM ch=%d", (int)ch);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ─────────────────────────── Public API ─────────────────────────────── */

esp_err_t motor_init(const motor_cfg_t *cfg){
    // Store configuration in module-global state
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "Initializing motor system...");
    ESP_LOGI(TAG, "  Stroke: %.1f mm @ %.2f mm/s", s_cfg.stroke_mm, s_cfg.speed_mm_per_s);
    ESP_LOGI(TAG, "  AZ limits: 0° to %.1f° (PWM=%d DIR=%d)", s_cfg.max_az_deg, s_cfg.az_pwm_pin, s_cfg.az_dir_pin);
    ESP_LOGI(TAG, "  EL limits: %.1f° to %.1f° (PWM=%d DIR=%d)", s_cfg.min_el_deg, s_cfg.max_el_deg, s_cfg.el_pwm_pin, s_cfg.el_dir_pin);

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

    ESP_LOGI(TAG, "Motor initialization complete");
    return ESP_OK;
}

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

    // Calculate move time based on distance and nominal speed
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    // Log the complete move plan
    ESP_LOGI(TAG, "AZ executing: %.2f°→%.2f° (%.1f→%.1f mm) %s for %" PRIu32 " ms",
             current_deg, target_deg, current_mm, target_mm,
             (target_mm > current_mm) ? "EXTEND" : "RETRACT", move_ms);

    // Execute the move: start PWM, wait, stop PWM
    start_pwm(AZ_CH, 4096);                     // 50% duty cycle
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Block for move duration
    stop_pwm(AZ_CH);                            // Stop PWM, actuator coasts
    
    ESP_LOGI(TAG, "AZ move complete");
}

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

    // Calculate timing
    uint32_t move_ms = move_time_ms(current_mm, target_mm);
    
    ESP_LOGI(TAG, "EL executing: %.2f°→%.2f° (%.1f→%.1f mm) %s for %" PRIu32 " ms",
             current_deg, target_deg, current_mm, target_mm,
             (target_mm > current_mm) ? "EXTEND" : "RETRACT", move_ms);

    // Execute the move
    start_pwm(EL_CH, 4096);                     // 50% duty
    vTaskDelay(pdMS_TO_TICKS(move_ms));         // Wait for completion
    stop_pwm(EL_CH);                            // Stop
    
    ESP_LOGI(TAG, "EL move complete");
}

void motor_run_az_ms(int dir_level, uint32_t ms){
    ESP_LOGI(TAG, "AZ timed run: DIR=%d for %" PRIu32 " ms (raw mode)", dir_level, ms);
    ESP_LOGW(TAG, "Running AZ in RAW mode - may exceed normal limits!");
    
    // Set direction immediately
    gpio_set_level(s_cfg.az_dir_pin, dir_level);
    
    // Run at full power for specified duration
    start_pwm(AZ_CH, 4096);                     // 50% duty (could go higher for homing)
    vTaskDelay(pdMS_TO_TICKS(ms));              // Block for entire duration
    stop_pwm(AZ_CH);                            // Stop PWM
    
    ESP_LOGI(TAG, "AZ timed run complete");
}

void motor_run_el_ms(int dir_level, uint32_t ms){
    ESP_LOGI(TAG, "EL timed run: DIR=%d for %" PRIu32 " ms (raw mode)", dir_level, ms);
    ESP_LOGW(TAG, "Running EL in RAW mode - may exceed normal limits!");
    
    // Set direction
    gpio_set_level(s_cfg.el_dir_pin, dir_level);
    
    // Execute timed run
    start_pwm(EL_CH, 4096);
    vTaskDelay(pdMS_TO_TICKS(ms));
    stop_pwm(EL_CH);
    
    ESP_LOGI(TAG, "EL timed run complete");
}

void motor_park(double park_az_deg, double park_el_deg, double cur_az, double cur_el){
    ESP_LOGI(TAG, "Parking: AZ %.2f°→%.2f°, EL %.2f°→%.2f°", 
             cur_az, park_az_deg, cur_el, park_el_deg);
    
    // Move axes sequentially (not simultaneously to reduce peak current)
    motor_move_az(cur_az, park_az_deg);
    motor_move_el(cur_el, park_el_deg);
    
    ESP_LOGI(TAG, "Park sequence complete");
}

void motor_stop_all(void){
    ESP_LOGI(TAG, "EMERGENCY STOP - halting all motor PWM");
    stop_pwm(AZ_CH);
    stop_pwm(EL_CH);
    ESP_LOGI(TAG, "All motors stopped (DIR pins unchanged)");
}