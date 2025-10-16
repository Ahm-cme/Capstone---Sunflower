#pragma once
#include <time.h>
#include <stdbool.h>

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Solar Tracking System - Core Controller                               │
    │ - Implements sensorless "install once + auto-learn offset" algorithm  │
    │ - Nightly homing to mechanical stops eliminates open-loop drift       │
    │ - Dynamic cadence: 15min after moves, 5min when waiting for motion    │
    │ - Deep sleep scheduling based on sunrise/sunset calculations           │
    └───────────────────────────────────────────────────────────────────────┘

    Algorithm Overview:
    The tracking system uses time-of-motion control with daily homing to mechanical
    stops. This eliminates the need for position sensors while maintaining reasonable
    accuracy over long deployments.

    Key Features:
    1. Install-time calibration: manually align to sun, long-press button to learn
       mount orientation offsets. After this, base can be bolted any direction.
    2. Nightly homing: drives both axes to hard stops before sleep, assigns known
       angles. This bounds accumulated error from open-loop control.
    3. Dynamic cadence: checks every 5 minutes until sun moves ≥10°, then every
       15 minutes after commanding a move.
    4. Deep sleep: powers down during night/low sun, wakes before sunrise.

    Coordinate Systems:
    - "Earth frame": standard geographic coordinates (0°=North, elevation from horizon)
    - "Mount frame": panel coordinate system after applying installation offsets
    - Conversion: mount_target = earth_sun - mount_offset (learned during calibration)

    Tracking Strategy:
    - Only move when sun changes ≥10° (avoids unnecessary wear and power consumption)
    - Minimum 2° steps to avoid tiny jitter movements
    - 10° tolerance accounts for linkage backlash and open-loop uncertainty
    - Priority on robustness over precision (solar panels are forgiving)

    Error Sources & Mitigation:
    - Battery voltage variation: daily homing resets accumulated position error
    - Temperature effects on actuator speed: bounded by nightly calibration  
    - Wind loading during moves: 300ms timing buffer helps ensure completion
    - Mechanical backlash: 10° tolerance masks small positioning errors
    - GPS time drift: system syncs time from GPS periodically

    Power Management:
    - Deep sleep during night reduces average power to ~10-50µA
    - Wake-before-sunrise ensures panels positioned for maximum morning capture
    - Motor moves are brief (few seconds) with long idle periods (5-15 minutes)
    - LED patterns optimized for low duty cycle during normal tracking

    Failure Modes:
    - GPS loss: uses last known position until signal recovers (LED_ERROR pattern)
    - SD card failure: continues operation but loses data logging
    - Motor stall: daily homing against stops verifies mechanical operation
    - Power brownout: NVS persistence preserves state across reboots
*/

/*
    Tracker state and configuration.

    Angles:
      - All angles in degrees.
      - "Earth frame" azimuth = 0..360° from true North, positive eastward.
      - "Mount frame" = the panel/mount's own coordinate system after applying
        installation offsets (az_mount_offset_deg, el_mount_offset_deg).

    Offsets:
      - az_mount_offset_deg/el_mount_offset_deg are learned once (manually align to sun,
        long-press button). They let the algorithm ignore how the base is bolted.

    Homing:
      - Sensorless homing at sunset: drive to mechanical stops for a known pose
        (AZ fully retracted, EL fully extended). This bounds daily drift.
*/
typedef struct {
    // Current and target angles (mount frame)
    double az_cur, el_cur;   // last commanded/assumed pose of the panel
    double az_tgt, el_tgt;   // next target to move toward

    // Movement thresholds and limits
    double tol_deg;          // minimum error to trigger a move (10° target tolerance)
                            // Large tolerance accounts for open-loop uncertainty and backlash
    double min_step_deg;     // avoid tiny moves; require at least this step (2°)
                            // Prevents actuator wear from small jitter corrections
    int    update_period_s;  // legacy (kept for backward compat with old configs)
    double sleep_thresh_el;  // below this elevation we consider sleeping (e.g., 5°)
                            // Set high enough to avoid tracking when sun provides little energy

    // Cadence control (dynamic check interval)
    int base_period_s;       // nominal loop period after a move (15 min = 900s)
                            // Slow cadence after moves since sun changes slowly near solar noon
    int fast_period_s;       // faster loop when sun hasn't changed enough (5 min = 300s)
                            // Faster checks when waiting for accumulated change to reach tolerance
    int cur_period_s;        // currently active period (switches between base/fast)
                            // Runtime variable, not persisted to NVS
    int prewake_min;         // wake this many minutes before sunrise
                            // Early wake ensures panels positioned for maximum morning energy capture

    // Mount alignment offsets (earth -> mount frame)
    // These are learned during install-time calibration and stored in NVS.
    // Example: if your base points East when az_cur=0, set az_mount_offset_deg ≈ 90°.
    double az_mount_offset_deg; // subtract from earth azimuth to get mount az target
    double el_mount_offset_deg; // subtract from earth elevation to get mount el target

    // Homing configuration (mechanical stop pose)
    // At sleep, we drive AZ toward retract stop and EL toward extend stop for homing_time_ms.
    // These values depend on your actuator wiring and mechanical design.
    double home_az_deg;         // angle we assign at AZ stop (e.g., 0° = retracted)
    double home_el_deg;         // angle we assign at EL stop (e.g., 85° = extended)
    int    homing_time_ms;      // motor run time to guarantee reaching the stops
                               // Formula: (stroke_mm / speed_mm_per_s) * 1000 + safety_margin_ms
    int    az_home_dir_level;   // MD20A DIR level that drives AZ toward retract stop (0 or 1)
    int    el_home_dir_level;   // MD20A DIR level that drives EL toward extend stop (0 or 1)

    // Motion tracking for cadence decisions
    // Used to detect when sun has moved enough to justify faster checking
    double last_move_az_tgt;    // last commanded azimuth target
    double last_move_el_tgt;    // last commanded elevation target

    // Statistics and persistence
    unsigned moves_today;       // resets at midnight (useful for maintenance scheduling)
    unsigned total_moves;       // cumulative lifetime counter (wear tracking)
    time_t   last_move;         // epoch time of last move (diagnostics)
} tracker_state_t;

/*
    Initialize and start the tracking system.
    
    Creates a FreeRTOS task that:
    1. Loads persistent state from NVS
    2. Initializes CSV logging with headers
    3. Runs main tracking loop with GPS polling and solar calculations
    4. Handles sleep/wake cycles based on sun position
    5. Manages dynamic cadence based on sun motion
    
    Task specifications:
    - Stack: 4KB (handles GPS I/O, solar math, NVS operations)
    - Priority: 5 (higher than LED patterns, lower than critical drivers)
    - Persistent: runs continuously until deep sleep
    
    The task is non-blocking - this function returns immediately after task creation.
    Use LED patterns and SD logs to monitor tracking system health.
*/
void tracking_start(void);

/*
    Perform install-time mount offset calibration.
    
    This function should be called when:
    1. The panel is manually aligned to point directly at the sun
    2. GPS has a valid fix (for accurate sun position calculation)
    3. User confirms alignment (typically via long-press button)
    
    Process:
    1. Read current GPS position and UTC time
    2. Calculate sun position in earth coordinates
    3. Compute offsets: mount_offset = sun_earth - panel_mount_current
    4. Store offsets in NVS for future tracking operations
    
    After calibration:
    - The base orientation no longer matters for tracking accuracy
    - Future sun targets are: mount_target = sun_earth - stored_offset
    - Calibration can be repeated anytime (e.g., after mechanical adjustments)
    
    Error handling:
    - Silently fails if no GPS fix available
    - Logs calibration values to SD card and console
    - Safe to call from any task (uses mutex if needed)
    
    Use when the panel is manually aligned to the sun; computes az/el offsets so
    future tracking doesn't depend on base orientation.
*/
void tracking_calibrate_mount_offset_now(void);

/*
    Automatic mount orientation calibration using compass.
    
    This replaces manual alignment - system automatically learns mount
    orientation by comparing sun azimuth (calculated) vs compass heading.
    
    Algorithm:
    1. Get GPS position and current time
    2. Calculate sun's true azimuth angle
    3. Read compass heading (mount's magnetic north reference)
    4. Compute offset: az_mount_offset = compass_heading - sun_azimuth
    5. Store offset in NVS for future boots
    
    Requirements:
    - Valid GPS fix (lat/lon/time)
    - Sun elevation > 15° (avoid horizon refraction errors)
    - Compass calibrated (run gps_calibrate_compass() first)
    
    Called automatically on first boot or when user requests re-calibration.
*/

void tracking_auto_calibrate_with_compass(void);

/*
    Development and debugging functions (add these for testing):
    
    void tracking_get_state(tracker_state_t *state);     // Read current state
    void tracking_force_move(double az, double el);      // Manual move command  
    void tracking_set_sleep_threshold(double el_deg);    // Adjust sleep elevation
    void tracking_trigger_homing(void);                  // Force homing sequence
    bool tracking_is_sleeping(void);                     // Check if in deep sleep mode
*/