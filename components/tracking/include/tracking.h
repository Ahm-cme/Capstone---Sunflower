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
    3. NVS persistence: position estimates and calibration survive power brownout
*/

/*
    Tracker state structure - persisted to NVS flash.
    
    All fields are saved/restored across reboots to maintain continuity.
    
    IMPORTANT: This struct is stored as a binary blob in NVS.
    If you modify this structure, old NVS data will be incompatible.
    Increment a version field or clear NVS after firmware updates.
*/
typedef struct {
    // Tracking parameters
    double   tol_deg;            // Movement tolerance threshold (degrees)
    double   min_step_deg;       // Minimum movement step to avoid jitter (degrees)
    uint32_t base_period_s;      // Slow cadence check interval (seconds)
    uint32_t fast_period_s;      // Fast cadence check interval (seconds)
    double   sleep_thresh_el;    // Sun elevation threshold for night detection (degrees)
    
    // Homing configuration - MUST MATCH .c FILE
    double   az_home_deg;        // Azimuth angle at home position (degrees)
    double   el_home_deg;        // Elevation angle at home position (degrees)
    int      az_home_dir_level;  // DIR pin level to reach azimuth home (0 or 1)
    int      el_home_dir_level;  // DIR pin level to reach elevation home (0 or 1)
    uint32_t homing_time_ms;     // Time to drive to stops (milliseconds)
    
    // Current position estimates
    double   az_cur;             // Current azimuth estimate (degrees)
    double   el_cur;             // Current elevation estimate (degrees)
    
    // Last commanded targets (for detecting stuck conditions)
    double   last_move_az_tgt;   // Last azimuth target (degrees)
    double   last_move_el_tgt;   // Last elevation target (degrees)
    
    // Mount orientation offsets (learned during calibration)
    double   az_mount_offset_deg; // Azimuth offset (degrees)
    double   el_mount_offset_deg; // Elevation offset (degrees)
    
    // Statistics - MUST USE 'moves_today' NOT 'num_moves_today'
    uint32_t moves_today;        // Daily move counter (reset at midnight)
    uint32_t total_moves;        // Lifetime move counter
    time_t   last_move;          // Epoch time of last move (diagnostics)
} tracker_state_t;

/*
    Start the tracking system.
    
    Creates FreeRTOS task that runs the main tracking loop.
    Task runs with 4KB stack at priority 5.
    
    Call once during system initialization after:
    - NVS initialized
    - GPS initialized
    - Motors initialized
    - SD card mounted
    - Status LED initialized
    
    Never returns - task runs until deep sleep or reset.
*/
void tracking_start(void);

/*
    Perform mount orientation calibration.
    
    Two modes:
    
    1. HARDCODED_LOCATION mode (testing):
       - Uses compile-time mount orientation constants
       - No GPS or sun alignment required
       - Stores hardcoded offsets to NVS
    
    2. Real GPS mode (production):
       - User manually aligns panel to sun
       - Requires valid GPS fix
       - Calculates offsets from sun position vs panel position
       - Stores learned offsets to NVS
    
    After calibration:
    - Future tracking uses stored offsets automatically
    - Base orientation becomes irrelevant (offsets compensate)
    - Can be repeated anytime (e.g., after mechanical adjustments)
    
    Trigger:
    - Long-press button (>3 seconds) in production
    - Automatic on first boot in hardcoded mode
*/
void tracking_calibrate_mount_offset_now(void);

/*
    Future API extensions (not yet implemented):
    
    bool tracking_is_sleeping(void);                     // Check if in deep sleep mode
    void tracking_set_tolerance(double deg);             // Adjust movement threshold
    void tracking_get_state(tracker_state_t *out);       // Read current state
    void tracking_force_home(void);                      // Manually trigger homing sequence
    void tracking_emergency_stop(void);                  // Stop all motors immediately
*/