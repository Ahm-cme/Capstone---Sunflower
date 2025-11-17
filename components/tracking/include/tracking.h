#pragma once
#include <time.h>
#include <stdbool.h>

/*
    Solar Tracking Controller

    Summary:
    - Sensorless control with daily homing to mechanical stops.
    - Dynamic loop cadence (fast while waiting to move, slower after moves).
    - Deep sleep at night, wake before sunrise.
    - Install-time mount offsets decouple base orientation from tracking.

    Architecture:
    - Main tracking loop runs at variable cadence (5-15 min periods)
    - Computes sun position from GPS location and time
    - Applies mount offsets to convert earth frame → mount frame
    - Moves motors when angular error exceeds threshold
    - Homes to midpoint nightly (resets drift without hard stops)
    - Deep sleeps at night, wakes before sunrise

    Coordinate Systems:
    - Earth frame: azimuth 0°=North, 90°=East, 180°=South, 270°=West
                   elevation 0°=horizon, 90°=zenith
    - Mount frame: panel angles after applying learned az/el offsets
                   accounts for base orientation and mounting errors

    Mount Offsets:
    - Learned during manual calibration (point panel at sun, press button)
    - Or auto-calibrated using compass (if available)
    - Stored in NVS flash, persist across reboots
    - Offset = earth_angle - mount_angle

    Homing Strategy:
    - Nightly homing to midpoint (actuators at 50% stroke)
    - Safer than driving to hard stops (reduces mechanical stress)
    - Uses normal motor_move_* functions with conservative timing
    - Resets accumulated open-loop drift
    - Position: AZ=135° (half of 270° range), EL=47.5° (midpoint of 10-85° range)

    Movement Thresholds:
    - Tolerance: 10° (must exceed to trigger move)
    - Minimum step: 2° (prevents tiny unnecessary moves)
    - Dynamic cadence: 5 min while waiting, 15 min after move

    Integration with Main:
    - Called via tracking_start() to begin autonomous operation
    - Provides tracking_get_current_angles() for display
    - Provides tracking_get_move_stats() for telemetry
    - Calibration functions for manual/auto offset learning

    Conventions:
    - All angles in decimal degrees
    - All times in UTC (Unix epoch seconds)
    - State persisted to NVS flash periodically
*/

typedef struct {
    // Current/target (mount frame)
    double az_cur, el_cur;     // Last commanded/assumed pose (mount frame)
    double az_tgt, el_tgt;     // Next targets (computed from sun position)

    // NEW: Actuator position tracking (0-200mm)
    double az_actuator_mm;     // Current AZ actuator extension
    double el_actuator_mm;     // Current EL actuator extension

    // Motion thresholds/limits
    double tol_deg;            // Error required to move (e.g., 10°)
    double min_step_deg;       // Minimum commanded step (e.g., 2°)
    int    update_period_s;    // Legacy; unused by new cadence
    double sleep_thresh_el;    // Sleep if sun elevation below this (deg)

    // Cadence (loop period selection)
    int base_period_s;         // Period after a move (e.g., 900 s = 15 min)
    int fast_period_s;         // Period while waiting to move (e.g., 300 s = 5 min)
    int cur_period_s;          // Active period (runtime)

    // Sleep/wake management
    int prewake_min;           // Wake this many minutes before sunrise (e.g., 10)

    // Install-time mount offsets (earth → mount frame)
    double az_mount_offset_deg;  // Subtracted from earth azimuth
    double el_mount_offset_deg;  // Subtracted from earth elevation

    // Homing configuration (mechanical stop pose)
    double home_az_deg;        // Assigned AZ angle at stop (e.g., 135° = midpoint)
    double home_el_deg;        // Assigned EL angle at stop (e.g., 47.5° = midpoint)
    int    homing_time_ms;     // Time to reach stops (includes safety margin)
    int    az_home_dir_level;  // DIR level that drives AZ toward its stop
    int    el_home_dir_level;  // DIR level that drives EL toward its stop

    // Move history (for cadence decisions)
    double last_move_az_tgt;   // Last commanded AZ target
    double last_move_el_tgt;   // Last commanded EL target

    // Stats/persistence
    unsigned moves_today;      // Resets at UTC midnight
    unsigned total_moves;      // Lifetime counter
    time_t   last_move;        // Timestamp of last move

    // NEW: Mount orientation
    double mount_base_heading_deg;   // System back direction in earth frame (180° = south)
    double compass_heading_deg;      // Current compass reading (for logging)
} tracker_state_t;

/*
    Start the tracking task.
    
    What it does:
    - Loads persisted state from NVS flash
    - Initializes CSV logging with header
    - Creates FreeRTOS tracking task (runs until deep sleep)
    - Returns immediately; tracking loop runs autonomously
    
    Called from:
    - main.c after system initialization
    - After button press to start tracking
    
    Task behavior:
    - Runs at variable cadence (5-15 min loops)
    - Polls GPS for position/time
    - Computes sun position and target angles
    - Moves motors when needed
    - Logs telemetry to SD card
    - Homes and sleeps at night
    
    Returns immediately - does not block.
*/
void tracking_start(void);

/*
    Calibrate mount offsets using manual sun alignment.
    
    Procedure:
    1. Manually point panel directly at the sun
    2. Hold calibration button for 3 seconds
    3. System computes offsets: offset = sun_angle - current_panel_angle
    4. Offsets saved to NVS flash
    5. Future tracking uses these offsets automatically
    
    Preconditions:
    - Valid GPS fix (position + time)
    - Sun above horizon (daylight)
    - Panel manually aligned to sun
    
    What it calculates:
    - az_mount_offset = sun_azimuth (earth) - current_az (mount)
    - el_mount_offset = sun_elevation (earth) - current_el (mount)
    
    Example:
    - Sun at 180° azimuth (south)
    - Panel physically pointing south but tracker thinks it's at 200°
    - Offset = 180° - 200° = -20°
    - Future: tracker will add -20° to all azimuth commands
    
    Can be recalibrated anytime - just overwrites old offsets.
    Useful after reinstalling hardware or if tracking accuracy degrades.
*/
void tracking_calibrate_mount_offset_now(void);

/*
    Auto-calibrate azimuth offset using compass.
    
    Procedure:
    1. Ensure compass is calibrated (double-press button first)
    2. Call this function (or press button in future auto-mode)
    3. System uses compass + GPS + sun position to compute az offset
    4. Elevation offset left at 0 (requires manual calibration)
    
    Preconditions:
    - Compass calibrated (gps_calibrate_compass() completed)
    - Valid GPS fix
    - Sun elevation > 15° (accurate sun azimuth needed)
    
    Advantages:
    - No manual alignment required
    - Can be done anytime sun is visible
    - Useful for field deployment
    
    Limitations:
    - Only calibrates azimuth (elevation still needs manual)
    - Requires compass calibration first
    - Less accurate than manual calibration
    
    Algorithm:
    - Reads magnetic heading from HMC5883L compass
    - Applies magnetic declination (GPS provides this)
    - Compares true heading to calculated sun azimuth
    - Computes offset to align tracking with compass/sun
    
    Stores az_mount_offset in NVS flash.
*/
void tracking_auto_calibrate_with_compass(void);

/*
    Get current tracking angles (mount frame).
    
    Returns:
    - az_deg: Current azimuth in mount frame (0-360°)
    - el_deg: Current elevation in mount frame (typically 10-85°)
    
    Used by:
    - Main loop for WiFi transmission to LCD display
    - System check to show current position
    - Diagnostics and debugging
    
    Thread-safe: can be called from any task.
    
    Example:
    ```c
    float az, el;
    tracking_get_current_angles(&az, &el);
    printf("Panel at: %.1f° az, %.1f° el\n", az, el);
    ```
*/
void tracking_get_current_angles(float *az_deg, float *el_deg);

/*
    Get target sun angles (where we're trying to point).
    
    Returns:
    - az_deg: Target azimuth in earth frame (0-360°)
    - el_deg: Target elevation in earth frame (typically 0-90°)
    
    Used by:
    - Main loop for WiFi transmission to LCD display
    - System check to show current target position
    - Diagnostics and debugging
    
    Thread-safe: can be called from any task.
    
    Example:
    ```c
    float az, el;
    tracking_get_target_angles(&az, &el);
    printf("Target: %.1f° az, %.1f° el\n", az, el);
    ```
*/
void tracking_get_target_angles(float *az_deg, float *el_deg);

/*
    Get move statistics for telemetry display.
    
    Returns:
    - moves_today: Count of moves since UTC midnight
    - total_moves: Lifetime move count (persisted in NVS)
    
    Used by:
    - Main loop to populate tracker_data_t for WiFi/display
    - SD card logging for performance analysis
    - System diagnostics
    
    Daily reset:
    - moves_today resets to 0 at UTC midnight
    - total_moves never resets (lifetime counter)
    
    Thread-safe: can be called from any task.
    
    Example:
    ```c
    uint32_t today, total;
    tracking_get_move_stats(&today, &total);
    printf("Moves: %u today, %u lifetime\n", today, total);
    ```
*/
void tracking_get_move_stats(uint32_t *moves_today, uint32_t *total_moves);

/*
    Get mount calibration offsets.
    
    Returns:
    - az_offset_deg: Azimuth offset in degrees
    - el_offset_deg: Elevation offset in degrees
    
    Offsets are learned during calibration and persist in NVS.
    Relationship: mount_angle = earth_angle - offset
    
    Thread-safe: can be called from any task.
    
    Example:
    ```c
    float az_offset, el_offset;
    tracking_get_mount_offsets(&az_offset, &el_offset);
    printf("Offsets: %.1f° az, %.1f° el\n", az_offset, el_offset);
    ```
*/
void tracking_get_mount_offsets(float *az_offset_deg, float *el_offset_deg);