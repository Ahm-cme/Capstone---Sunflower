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

    Conventions:
    - Angles in degrees.
    - Earth frame: azimuth 0°=North, +east; elevation 0°=horizon.
    - Mount frame: panel angles after applying learned az/el offsets.
*/

typedef struct {
    // Current/target (mount frame)
    double az_cur, el_cur;     // Last commanded/assumed pose
    double az_tgt, el_tgt;     // Next targets

    // Motion thresholds/limits
    double tol_deg;            // Error required to move (e.g., 10°)
    double min_step_deg;       // Minimum commanded step (e.g., 2°)
    int    update_period_s;    // Legacy; unused by new cadence
    double sleep_thresh_el;    // Sleep if sun elevation below this (deg)

    // Cadence (loop period selection)
    int base_period_s;         // Period after a move (e.g., 900 s)
    int fast_period_s;         // Period while waiting to move (e.g., 300 s)
    int cur_period_s;          // Active period (runtime)

    // Sleep/wake management
    int prewake_min;           // Wake this many minutes before sunrise (e.g., 10)

    // Install-time mount offsets (earth → mount frame)
    double az_mount_offset_deg;  // Subtracted from earth azimuth
    double el_mount_offset_deg;  // Subtracted from earth elevation

    // Homing configuration (mechanical stop pose)
    double home_az_deg;        // Assigned AZ angle at stop (e.g., 0°)
    double home_el_deg;        // Assigned EL angle at stop (e.g., 85°)
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
} tracker_state_t;

/*
    Start the tracking task.
    - Loads state from NVS, prepares logging, then runs the control loop.
    - Returns immediately; task runs until deep sleep.
*/
void tracking_start(void);

/*
    Calibrate mount offsets using manual sun alignment.
    - Preconditions: panel manually aligned to sun, valid GPS/time.
    - Computes az/el offsets (earth − current_mount) and stores in NVS.
*/
void tracking_calibrate_mount_offset_now(void);

/*
    Auto-calibrate azimuth offset using compass.
    - Preconditions: valid GPS/time, sun elevation > 15°, compass calibrated.
    - Stores az_mount_offset in NVS.
*/
void tracking_auto_calibrate_with_compass(void);

/*
    Get current tracking angles (mount frame).
*/
void tracking_get_current_angles(float *az_deg, float *el_deg);