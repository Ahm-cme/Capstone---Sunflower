#pragma once
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>

/*
    Motor driver (2x linear actuators via PWM + DIR)

    Scope:
    - Controls AZ (azimuth) and EL (elevation) linear actuators.
    - Open-loop, time-based motion (no encoders).
    - Angle↔stroke uses a simple linear map (good enough for solar tracking).

    Hardware/LEDC:
    - LEDC low-speed, 5 kHz, 13-bit duty (0..8191).
    - One PWM + one DIR GPIO per actuator.

    Coordinate system:
    - AZ: 0°..max_az_deg
    - EL: min_el_deg..max_el_deg (0°=horizon, 90°=zenith)

    Behavior:
    - Clamp commands to configured angle limits.
    - Convert degrees to mm; compute time from distance and nominal speed.
    - Conservative timing (90% + buffer) to prevent overshoot.
    - Tracking module performs homing to hard stops daily to reset drift.
    
    Integration with system check:
    - motor_init() returns ESP_OK on success for validation
    - Provides current position query for diagnostics
    - Detailed logging for troubleshooting
*/

typedef struct {
    // GPIOs
    int az_pwm_pin; int az_dir_pin;     // Azimuth PWM and DIR pins
    int el_pwm_pin; int el_dir_pin;     // Elevation PWM and DIR pins

    // Actuator mechanics/kinematics
    double stroke_mm;                   // Full actuator stroke (mm)
    double speed_mm_per_s;              // Nominal speed at operating voltage (mm/s)

    // Logical angle limits
    double max_az_deg;                  // Usable AZ range: 0..max_az_deg
    double max_el_deg;                  // Upper EL limit (e.g., 85°)
    double min_el_deg;                  // Lower EL limit (e.g., 10°)
} motor_cfg_t;

/*
    Initialize LEDC timer/channels and DIR GPIOs.

    LEDC:
    - Timer: LEDC_TIMER_0, 5 kHz, 13-bit
    - Channels: CH0=AZ, CH1=EL (low-speed mode)

    Returns:
    - ESP_OK on success, error code on failure.
    
    For system check:
    - Verifies GPIO configuration
    - Confirms PWM timer setup
    - Tests initial DIR pin states
*/
esp_err_t motor_init(const motor_cfg_t *cfg);

/*
    Move AZ from current_deg to target_deg (blocking, open-loop).
    - Clamp target to [0, max_az_deg].
    - Map degrees→mm; choose DIR; compute run time; run PWM at full duty; stop.
    - Uses conservative timing (90% + buffer) to prevent overshoot.
    - Logs complete move details for diagnostics.
*/
void motor_move_az(double current_deg, double target_deg);

/*
    Move EL from current_deg to target_deg (blocking, open-loop).
    - Clamp target to [min_el_deg, max_el_deg].
    - Map degrees→mm; choose DIR; compute run time; run PWM at full duty; stop.
    - Uses conservative timing (90% + buffer) to prevent overshoot.
    - Logs complete move details for diagnostics.
*/
void motor_move_el(double current_deg, double target_deg);

/*
    Park sequence: move AZ then EL to given angles (sequential to limit current).
    - Used during sleep/shutdown
    - Moves axes one at a time to reduce peak current draw
*/
void motor_park(double park_az_deg, double park_el_deg, double cur_az, double cur_el);

/*
    Emergency stop: immediately set PWM duty to 0% on both channels.
    - DIR levels remain unchanged.
    - Use for fault conditions or manual abort.
*/
void motor_stop_all(void);

/*
    Raw timed runs for homing to mechanical stops (no angle math).
    - dir_level: 0=retract, 1=extend (as wired)
    - ms: duration at full duty
    - WARNING: These will drive actuators to hard stops!
    - Used only during nightly homing sequence.
*/
void motor_run_az_ms(int dir_level, uint32_t ms);
void motor_run_el_ms(int dir_level, uint32_t ms);

/*
    Get motor configuration (for diagnostics/display).
    - Returns pointer to internal config structure
    - Used by system check to display motor parameters
*/
const motor_cfg_t* motor_get_config(void);

/*
    Test motor functionality (quick self-check).
    - Briefly pulses each motor in both directions
    - Non-blocking, ~2 second duration
    - Returns true if motors respond, false if stuck
    - Used during system initialization
*/
bool motor_self_test(void);

/*
    Get last move statistics (for tracking quality monitoring).
    - Fills provided structure with last move details
    - Used to calculate tracking quality on display
*/
typedef struct {
    double last_az_move_deg;        // Last AZ movement in degrees
    double last_el_move_deg;        // Last EL movement in degrees
    uint32_t last_az_duration_ms;   // Actual AZ move duration
    uint32_t last_el_duration_ms;   // Actual EL move duration
    uint32_t total_moves;           // Total moves since boot
} motor_stats_t;

void motor_get_stats(motor_stats_t *stats);