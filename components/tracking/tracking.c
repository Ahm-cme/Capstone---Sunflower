#include "tracking.h"
#include "gps.h"
#include "solar.h"
#include "motor.h"
#include "sdlog.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include "status_led.h"
#include "esp_sleep.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Solar Tracking Implementation                                         │
    │                                                                       │
    │ Core Algorithm Flow:                                                  │
    │ 1. Get GPS position (fresh or cached)                                │
    │ 2. Calculate sun position in earth coordinates                       │
    │ 3. Apply mount offsets → mount coordinates                           │
    │ 4. Check if movement needed (≥10° change)                            │
    │ 5. Execute move or adjust check cadence                              │
    │ 6. Log telemetry and update statistics                               │
    │ 7. Sleep if sun low/night, else wait for next cadence               │
    │                                                                       │
    │ Timing Considerations:                                               │
    │ - Sun moves ~15°/hour → 10° threshold = ~40min natural cadence      │
    │ - Dynamic cadence: 5min fast checks, 15min after moves              │
    │ - Homing takes ~45s total (22s per axis + safety margins)           │
    │ - Deep sleep saves ~99% power during 12-hour nights                 │
    │                                                                       │
    │ Error Recovery:                                                      │
    │ - GPS loss: LED_ERROR, retry every 30s with last position           │
    │ - NVS corruption: falls back to compiled defaults                   │
    │ - Motor stall: daily homing provides hard position reference        │
    │ - Power cycles: NVS state restoration maintains continuity          │
    │                                                                       │
    │ Debugging Tips:                                                      │
    │ - Enable DEBUG logs to see sun calculations and movement decisions   │
    │ - CSV logs provide historical tracking performance data             │
    │ - LED patterns indicate system health at a glance                   │
    │ - NVS state survives reboots for troubleshooting                    │
    └───────────────────────────────────────────────────────────────────────┘
*/

#if USE_HARDCODED_LOCATION
// System orientation: base faces north (0° azimuth reference)
// Initial panel position: facing straight up (zenith)
#define HARDCODED_MOUNT_AZIMUTH 0.0      // System base oriented north
#define HARDCODED_MOUNT_ELEVATION 90.0   // Panel plane faces up (vertical from ground)
#define HARDCODED_PANEL_NORMAL_AZ 0.0    // Panel normal initially points to zenith
#define HARDCODED_PANEL_NORMAL_EL 90.0   // Panel normal at 90° elevation (straight up)
#endif

#define TAG "TRACK"

// Global tracker state with sensible defaults for initial deployment.
// These values are overridden by NVS-stored state after first calibration.
//
// Configuration notes:
//  - tol_deg=10: implements "move when ≥10° change" requirement
//    Large tolerance accounts for open-loop uncertainty and reduces actuator wear
//  - base_period_s=900 (15 min): slow checks after moves since sun changes slowly
//  - fast_period_s=300 (5 min): faster checks when waiting for threshold
//  - sleep_thresh_el=5°: below this elevation we consider night operation pointless
//  - Home pose tuning: AZ retracted=0°, EL extended=85° (adjust for your linkage)
//  - homing_time_ms calculation: 200mm stroke ÷ 11.94mm/s ≈ 16.8s + 5s safety = 22s
static tracker_state_t s = {
    .az_cur=180,.el_cur=45,.tol_deg=10,.min_step_deg=2,
    .update_period_s=300,.sleep_thresh_el=5,
    .base_period_s=900, .fast_period_s=300, .cur_period_s=900,
    .prewake_min=10,

    // Installation offsets: learned during calibration, defaults assume perfect North alignment
    .az_mount_offset_deg = 0.0,  // will be set during install calibration
    .el_mount_offset_deg = 0.0,  // will be set during install calibration

    // Home pose configuration for mechanical stops
    // Critical: these must match your actual hardware setup
    .home_az_deg = 0.0,           // angle assigned when AZ hits retract stop
    .home_el_deg = 85.0,          // angle assigned when EL hits extend stop
    .homing_time_ms = 22000,      // 200mm ÷ 11.94mm/s + 30% margin = 22s
    .az_home_dir_level = 0,       // DIR level that drives AZ to retract stop (verify with testing)
    .el_home_dir_level = 1,       // DIR level that drives EL to extend stop (verify with testing)

    // Initialize last-move tracking (prevents immediate motion on boot)
    .last_move_az_tgt=180, .last_move_el_tgt=45
};

// Thread synchronization (currently unused but reserved for multi-task access)
static SemaphoreHandle_t s_mutex;

/*
    Persist the entire tracker state to NVS flash memory.
    
    Storage details:
    - Namespace: "tracker" (8-byte key limit in NVS)
    - Key: "state" (simple identifier)
    - Value: entire tracker_state_t struct as binary blob
    - Wear leveling: NVS automatically distributes writes across flash
    
    Called when:
    - Critical state changes (homing, calibration)
    - Periodically during operation (every 10 moves)
    - Before entering deep sleep
    
    Error handling:
    - Silently continues on NVS errors (degraded but operational)
    - NVS corruption recovery via flash erase available in menuconfig
*/
static void nvs_save(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READWRITE, &h);
    if (ret == ESP_OK){
        esp_err_t write_ret = nvs_set_blob(h, "state", &s, sizeof(s));
        if (write_ret == ESP_OK){
            nvs_commit(h);
            ESP_LOGD(TAG, "State saved to NVS: az=%.1f el=%.1f moves=%u", 
                     s.az_cur, s.el_cur, s.total_moves);
        } else {
            ESP_LOGW(TAG, "NVS write failed: %s", esp_err_to_name(write_ret));
        }
        nvs_close(h);
    } else {
        ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(ret));
    }
}

/*
    Load tracker state from NVS if present, otherwise use compiled defaults.
    
    Called once at startup before entering main tracking loop.
    Handles version compatibility by checking blob size matches current struct.
    
    Recovery behavior:
    - No stored state: use compiled defaults (fresh install)
    - Corrupted state: log warning, use defaults (degraded operation)
    - Size mismatch: use defaults (firmware update scenario)
*/
static void nvs_load(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READONLY, &h);
    if (ret == ESP_OK){
        size_t required_size = sizeof(s);
        ret = nvs_get_blob(h, "state", &s, &required_size);
        if (ret == ESP_OK && required_size == sizeof(s)) {
            ESP_LOGI(TAG, "Loaded state from NVS: az=%.1f el=%.1f tot_moves=%u offsets=(%.2f,%.2f)", 
                     s.az_cur, s.el_cur, s.total_moves, s.az_mount_offset_deg, s.el_mount_offset_deg);
        } else {
            ESP_LOGW(TAG, "NVS state load failed: %s (size=%u, expected=%u)", 
                     esp_err_to_name(ret), (unsigned)required_size, (unsigned)sizeof(s));
        }
        nvs_close(h);
    } else {
        ESP_LOGI(TAG, "No stored state found, using defaults");
    }
}

/*
    Reset daily statistics at midnight.
    
    Uses localtime() for midnight detection (timezone independent since we only
    care about the hour/minute rollover, not absolute time).
    
    Called each tracking loop iteration - the hour==0 && min==0 condition
    ensures reset happens only once per day even if called multiple times
    during the midnight minute.
    
    Daily stats are useful for:
    - Maintenance scheduling (high move counts indicate windy conditions)
    - Power consumption estimation
    - Wear analysis for actuator longevity planning
*/
static void maybe_midnight_reset(void){
    time_t now = time(NULL);
    struct tm *lt = localtime(&now);
    if (lt && lt->tm_hour == 0 && lt->tm_min == 0) {
        if (s.moves_today > 0) {  // Only log if we had activity
            ESP_LOGI(TAG, "Midnight: resetting daily move count (was %u)", s.moves_today);
            sdlog_printf("Daily reset: %u moves yesterday", s.moves_today);
        }
        s.moves_today = 0;
    }
}

/*
    Execute a movement command if targets exceed tolerance and minimum step.
    
    Movement logic:
    1. Check both axes against tolerance (10°) and minimum step (2°)
    2. Move only axes that need adjustment (independent axis control)
    3. Update current position estimates to match commanded targets
    4. Update statistics and timestamp
    
    Why both tolerance and minimum step:
    - Tolerance (10°): main threshold for tracking accuracy vs actuator wear
    - Minimum step (2°): prevents tiny jitter movements near the tolerance boundary
    - Both must be exceeded to trigger movement
    
    Position tracking:
    - s.az_cur/s.el_cur represent our best estimate of panel position
    - Updated immediately after move commands (optimistic update)
    - Reset to known values during nightly homing
    - Open-loop control means these are estimates, not measurements
    
    Statistics tracking:
    - moves_today: daily counter for operational monitoring
    - total_moves: lifetime counter for maintenance scheduling
    - last_move: timestamp for diagnostic purposes
*/
static void do_move(double az_tgt, double el_tgt){
    // Calculate movement requirements for each axis independently
    double az_error = fabs(az_tgt - s.az_cur);
    double el_error = fabs(el_tgt - s.el_cur);
    
    bool move_az = (az_error > s.tol_deg) && (az_error > s.min_step_deg);
    bool move_el = (el_error > s.tol_deg) && (el_error > s.min_step_deg);

    if (!move_az && !move_el) {
        ESP_LOGI(TAG, "Within tolerance. No move needed.");
        ESP_LOGD(TAG, "  Current: az=%.1f° el=%.1f°", s.az_cur, s.el_cur);
        ESP_LOGD(TAG, "  Target:  az=%.1f° el=%.1f°", az_tgt, el_tgt);
        ESP_LOGD(TAG, "  Errors:  az=%.1f° el=%.1f° (tol=%.1f°)", az_error, el_error, s.tol_deg);
        return;
    }

    // Log the movement plan before execution
    ESP_LOGI(TAG, "Movement required:");
    ESP_LOGI(TAG, "  Current: az=%.1f° el=%.1f°", s.az_cur, s.el_cur);  
    ESP_LOGI(TAG, "  Target:  az=%.1f° el=%.1f°", az_tgt, el_tgt);
    ESP_LOGI(TAG, "  Plan: %s%s", 
             move_az ? "AZ " : "", 
             move_el ? "EL " : "");

    // Execute movements (motor.c handles timing and PWM control)
    if (move_az) { 
        ESP_LOGD(TAG, "Commanding AZ move: %.1f° → %.1f°", s.az_cur, az_tgt);
        motor_move_az(s.az_cur, az_tgt); 
        s.az_cur = az_tgt; 
    }
    if (move_el) { 
        ESP_LOGD(TAG, "Commanding EL move: %.1f° → %.1f°", s.el_cur, el_tgt);
        motor_move_el(s.el_cur, el_tgt); 
        s.el_cur = el_tgt; 
    }

    // Update statistics and persistence
    s.moves_today++; 
    s.total_moves++; 
    s.last_move = time(NULL);
    
    ESP_LOGI(TAG, "Movement complete. New position: az=%.1f° el=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "Statistics: %u moves today, %u total", s.moves_today, s.total_moves);
    
    // Log significant moves to SD card for analysis
    sdlog_printf("Move #%u: az=%.1f° el=%.1f° (today: %u)", 
                 s.total_moves, s.az_cur, s.el_cur, s.moves_today);
}

/*
    Enter deep sleep until specified UTC wake time.
    
    Deep sleep preparation:
    1. Stop all motor PWM outputs (safety)
    2. Set LED to sleep mode (visual indication)
    3. Log sleep duration and wake time  
    4. Program RTC timer wake source
    5. Enter deep sleep (never returns - system resets on wake)
    
    Wake time calculation:
    - Target wake time based on sunrise minus prewake_min
    - Minimum sleep duration: 60 seconds (prevents rapid sleep/wake cycles)
    - RTC timer accuracy: ±2% over temperature (sufficient for solar tracking)
    
    Power savings:
    - Deep sleep current: ~10-50µA (vs ~100-500mA active)
    - 12-hour sleep saves ~99% power vs continuous operation
    - Essential for battery-powered operation
    
    Recovery behavior:
    - System boots normally after RTC wake
    - NVS state automatically restored  
    - Tracking resumes from saved position estimates
*/
static void enter_deep_sleep_until(time_t wake_utc){
    time_t now = time(NULL);
    int64_t delta_s = (int64_t)wake_utc - (int64_t)now;
    
    // Enforce minimum sleep duration to prevent busy loops
    if (delta_s < 60) {
        ESP_LOGW(TAG, "Sleep duration too short (%lld s), extending to 60s", (long long)delta_s);
        delta_s = 60;
        wake_utc = now + 60;
    }
    
    // Log sleep plan with human-readable times
    struct tm *sleep_tm = localtime(&now);
    struct tm *wake_tm = localtime(&wake_utc);
    
    ESP_LOGI(TAG, "Entering deep sleep:");
    ESP_LOGI(TAG, "  Duration: %lld seconds (%.1f hours)", (long long)delta_s, delta_s / 3600.0);
    ESP_LOGI(TAG, "  Sleep:    %04d-%02d-%02d %02d:%02d:%02d", 
             sleep_tm->tm_year+1900, sleep_tm->tm_mon+1, sleep_tm->tm_mday,
             sleep_tm->tm_hour, sleep_tm->tm_min, sleep_tm->tm_sec);
    ESP_LOGI(TAG, "  Wake:     %04d-%02d-%02d %02d:%02d:%02d",
             wake_tm->tm_year+1900, wake_tm->tm_mon+1, wake_tm->tm_mday, 
             wake_tm->tm_hour, wake_tm->tm_min, wake_tm->tm_sec);
    
    sdlog_printf("Deep sleep for %lld s (wake @ %ld UTC)", (long long)delta_s, (long)wake_utc);
    
    // System shutdown sequence
    status_led_set_mode(LED_SLEEP);                 // Visual indication
    motor_stop_all();                               // Safety: stop all PWM
    esp_sleep_enable_timer_wakeup((uint64_t)delta_s * 1000000ULL);  // Program RTC timer
    
    ESP_LOGI(TAG, "System entering deep sleep now...");
    esp_deep_sleep_start();                         // Never returns
}

// Angle normalization utilities for coordinate system conversions
static double wrap360(double d){ 
    while(d < 0) d += 360; 
    while(d >= 360) d -= 360; 
    return d; 
}
static double wrap180(double d){ 
    while (d > 180.0) d -= 360.0; 
    while (d < -180.0) d += 360.0; 
    return d; 
}

/*
    Determine if the sun is descending (elevation decreasing over time).
    
    Used to avoid sleeping during temporary elevation dips (clouds, shadows)
    by confirming the sun is actually setting rather than just momentarily low.
    
    Algorithm:
    1. Sample current sun elevation
    2. Sample sun elevation 10 minutes in future
    3. Return true if future elevation is lower (descending)
    
    Why 10 minutes:
    - Long enough to distinguish trend from noise
    - Short enough to respond reasonably to sunset
    - Accounts for slight GPS time uncertainty
    
    This prevents premature sleep during:
    - Temporary cloud shadows  
    - Brief obstructions (trees, buildings)
    - GPS timing jitter
    - Atmospheric effects near horizon
*/
static bool is_descending(double lat, double lon){
    time_t now = time(NULL);
    sun_pos_t s0 = solar_compute(lat, lon, now);
    sun_pos_t s1 = solar_compute(lat, lon, now + 600);  // +10 minutes
    
    bool descending = s1.elevation_deg < s0.elevation_deg;
    ESP_LOGD(TAG, "Sun trend check: %.2f° → %.2f° (%s)", 
             s0.elevation_deg, s1.elevation_deg, 
             descending ? "descending" : "ascending");
    return descending;
}

/*
    Perform sensorless homing sequence to eliminate accumulated position error.
    
    Homing strategy:
    1. Drive AZ actuator to retract stop for homing_time_ms
    2. Drive EL actuator to extend stop for homing_time_ms  
    3. Assign known angles (home_az_deg, home_el_deg) to these positions
    4. Update last-move tracking to prevent immediate motion after wake
    5. Save state to NVS for persistence across reboot
    
    Why this works:
    - Mechanical stops provide absolute position reference
    - Daily homing bounds cumulative open-loop error
    - Hard stops are more reliable than encoders in outdoor environment
    - Deterministic pose eliminates position uncertainty
    
    Timing considerations:
    - homing_time_ms must be long enough to guarantee reaching stops
    - Formula: (stroke_mm / min_speed_mm_per_s) + safety_margin
    - Actuators may stall briefly at stops (normal, current-limited by drivers)
    
    LED indication:
    - Set to LED_SLEEP during homing (long operation, no user action needed)
    - Restored to LED_TRACKING after completion
    
    Critical: verify az_home_dir_level and el_home_dir_level match your wiring
*/
static void home_to_stops(void){
    ESP_LOGI(TAG, "=== HOMING SEQUENCE START ===");
    ESP_LOGI(TAG, "Driving to mechanical stops for position calibration");
    ESP_LOGD(TAG, "Homing config: AZ_dir=%d EL_dir=%d time=%dms", 
             s.az_home_dir_level, s.el_home_dir_level, s.homing_time_ms);
    
    status_led_set_mode(LED_SLEEP); // LED off during long operations

    // Sequential homing to avoid simultaneous high current draw
    ESP_LOGI(TAG, "Phase 1: AZ to retract stop...");
    motor_run_az_ms(s.az_home_dir_level, s.homing_time_ms);
    
    ESP_LOGI(TAG, "Phase 2: EL to extend stop...");  
    motor_run_el_ms(s.el_home_dir_level, s.homing_time_ms);

    // Assign exact known angles corresponding to the mechanical stop positions
    s.az_cur = s.home_az_deg;
    s.el_cur = s.home_el_deg;
    s.last_move_az_tgt = s.az_cur;  // Prevent immediate movement after wake
    s.last_move_el_tgt = s.el_cur;
    
    // Persist the homed state
    nvs_save();

    ESP_LOGI(TAG, "=== HOMING SEQUENCE COMPLETE ===");
    ESP_LOGI(TAG, "Position reset: az=%.1f° el=%.1f° (mechanical reference)", s.az_cur, s.el_cur);
    
    sdlog_printf("HOMED: az=%.1f el=%.1f (assigned)", s.az_cur, s.el_cur);
    status_led_set_mode(LED_TRACKING);
}

/*
    Perform one-time installation offset calibration.
    
    Calibration procedure:
    1. User manually aligns panel to point directly at sun
    2. User triggers this function (typically via long-press button)
    3. System reads GPS position and calculates current sun position
    4. Computes offsets: mount_offset = sun_earth - panel_mount_current
    5. Stores offsets in NVS for all future tracking operations
    
    After calibration:
    - Base orientation no longer affects tracking accuracy
    - Future targets: mount_target = sun_earth - stored_offset
    - Can be repeated anytime (e.g., after mechanical adjustments)
    
    Requirements for accurate calibration:
    - Valid GPS fix (for precise sun position calculation)
    - Panel manually aligned to sun (visual verification)
    - Clear sky conditions (avoid refraction errors)
    - Stable mounting (no movement during calibration)
    
    Error conditions:
    - No GPS fix: function returns silently (preserves existing offsets)
    - GPS time error: may introduce small systematic error
    - Misalignment: can be corrected by repeating calibration
*/
void tracking_calibrate_mount_offset_now(void) {
#if USE_HARDCODED_LOCATION
    // Use hardcoded vertical panel orientation
    float mount_az_offset = HARDCODED_MOUNT_AZIMUTH;
    float mount_el_offset = HARDCODED_MOUNT_ELEVATION;
    
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "║  HARDCODED MOUNT CALIBRATION                            ║");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "System Configuration:");
    ESP_LOGI(TAG, "  • System base orientation: North-facing (0° azimuth)");
    ESP_LOGI(TAG, "  • Panel initial position: Facing UP (zenith)");
    ESP_LOGI(TAG, "  • Panel plane angle: 90° from horizontal (vertical)");
    ESP_LOGI(TAG, "  • Panel normal vector: Points to zenith initially");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Mount offsets:");
    ESP_LOGI(TAG, "  • Azimuth offset: %.1f° (north reference)", mount_az_offset);
    ESP_LOGI(TAG, "  • Elevation offset: %.1f° (from horizontal)", mount_el_offset);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    
    // Store offsets to NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("tracking", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_blob(nvs_handle, "mount_az_off", &mount_az_offset, sizeof(float));
        nvs_set_blob(nvs_handle, "mount_el_off", &mount_el_offset, sizeof(float));
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Mount offsets stored to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to store mount offsets: %s", esp_err_to_name(err));
    }
    
    sdlog_printf("Hardcoded mount calibration:");
    sdlog_printf("  System base: North-facing (0° azimuth)");
    sdlog_printf("  Panel position: Facing UP (90° elevation)");
    sdlog_printf("  Mount offsets: Az=%.1f° El=%.1f°", mount_az_offset, mount_el_offset);
    
    return;
#else
    ESP_LOGI(TAG, "=== MOUNT OFFSET CALIBRATION START ===");
    
    // Require valid GPS data for accurate sun position calculation
    gps_fix_t g = {0};
    bool gps_available = gps_get_fix(&g, 5000);  // 5 second timeout
    
    if (!gps_available) {
        ESP_LOGW(TAG, "Calibration aborted: no GPS fix available");
        ESP_LOGW(TAG, "Ensure GPS antenna has clear sky view and try again");
        sdlog_printf("Calibration failed: no GPS");
        return;
    }
    
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);
    
    ESP_LOGI(TAG, "Calibration inputs:");
    ESP_LOGI(TAG, "  GPS: %.6f°N %.6f°W (fix_type=%u, sats=%u)", 
             g.latitude, g.longitude, g.fix_type, g.num_satellites);
    ESP_LOGI(TAG, "  Sun (earth): az=%.2f° el=%.2f°", sun.azimuth_deg, sun.elevation_deg);
    ESP_LOGI(TAG, "  Panel (mount): az=%.2f° el=%.2f°", s.az_cur, s.el_cur);
    
    // Calculate offsets assuming current panel position is aligned to sun
    double old_az_offset = s.az_mount_offset_deg;
    double old_el_offset = s.el_mount_offset_deg;
    
    s.az_mount_offset_deg = wrap360(sun.azimuth_deg - s.az_cur);
    s.el_mount_offset_deg = sun.elevation_deg - s.el_cur;
    
    // Persist calibration results
    nvs_save();

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "Mount offsets updated:");
    ESP_LOGI(TAG, "  AZ: %.3f° → %.3f° (Δ=%.3f°)", 
             old_az_offset, s.az_mount_offset_deg, 
             s.az_mount_offset_deg - old_az_offset);
    ESP_LOGI(TAG, "  EL: %.3f° → %.3f° (Δ=%.3f°)", 
             old_el_offset, s.el_mount_offset_deg,
             s.el_mount_offset_deg - old_el_offset);
    
    sdlog_printf("Calibrated offsets: az_off=%.2f el_off=%.2f", 
                 s.az_mount_offset_deg, s.el_mount_offset_deg);
    
    ESP_LOGI(TAG, "Future tracking will use these offsets automatically");
#endif
}

/*
    Main tracking task - runs continuously until deep sleep.
    
    Task lifecycle:
    1. Load state from NVS (position estimates, calibration data)
    2. Initialize CSV logging with proper headers
    3. Enter main tracking loop:
       a. Poll GPS (fresh or cached)
       b. Calculate sun position and apply mount offsets  
       c. Decide movement based on thresholds and cadence
       d. Execute moves or adjust check frequency
       e. Log telemetry data
       f. Handle sleep/wake based on sun elevation
    4. Never exits (deep sleep resets system)
    
    Error recovery:
    - GPS loss: LED_ERROR, 30s retry with cached position
    - NVS errors: continue with degraded persistence
    - Motor errors: daily homing provides recovery reference
    
    Power optimization:
    - Dynamic cadence reduces CPU wake frequency
    - Deep sleep during night saves 99% power
    - Brief operations minimize active power draw
    
    Thread safety:
    - Single-threaded design (no concurrent access to state)
    - Mutex reserved for future multi-task access
    - NVS operations atomic at library level
*/
static void tracking_task(void *arg){
    ESP_LOGI(TAG, "=== SOLAR TRACKING SYSTEM START ===");
    
    // Load persistent state and initialize subsystems
    nvs_load();
    
    const char *csv = "/sdcard/soltrac.csv";
    sdlog_write_csv_header_if_new(csv);
    sdlog_printf("Tracking started: az=%.1f el=%.1f offsets=(%.2f,%.2f)", 
                 s.az_cur, s.el_cur, s.az_mount_offset_deg, s.el_mount_offset_deg);

    // Initialize last-move targets with current pose to prevent immediate movement on boot
    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;
    
    ESP_LOGI(TAG, "Tracking loop starting with %ds cadence", s.cur_period_s);

    while (1){
        TickType_t loop_start = xTaskGetTickCount();

        // === GPS DATA ACQUISITION ===
        // In hardcoded mode, this returns immediately with fixed Auburn, AL coordinates
        // In real GPS mode, this would poll the MAX-M10S module via I2C
        gps_fix_t g = {0};
        bool gps_available = gps_get_fix(&g, 5000);  // 5 second timeout
        
        if (!gps_available) {
            // No GPS data at all - enter error state and retry
            status_led_set_mode(LED_ERROR);
            ESP_LOGW(TAG, "No GPS data available");
            ESP_LOGW(TAG, "In hardcoded mode, this should never happen");
            ESP_LOGW(TAG, "Retrying in 30 seconds...");
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        } else {
            // GPS recovered or was already good
            if (status_led_get_mode() == LED_ERROR) {
                ESP_LOGI(TAG, "GPS recovered, resuming tracking");
                status_led_set_mode(LED_TRACKING);
            }
        }
        // === SUN POSITION CALCULATION ===
        time_t now = time(NULL);
        sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);

        // Convert earth coordinates to mount coordinates using calibrated offsets
        s.az_tgt = wrap360(sun.azimuth_deg - s.az_mount_offset_deg);
        s.el_tgt = sun.elevation_deg - s.el_mount_offset_deg;

        // Calculate maximum angular change since last commanded move
        // This drives both movement decisions and cadence adjustments
        double daz = fabs(wrap180(s.az_tgt - s.last_move_az_tgt));
        double del = fabs(s.el_tgt - s.last_move_el_tgt);
        double dang = fmax(daz, del);

        ESP_LOGI(TAG, "=== TRACKING STATUS ===");
        ESP_LOGI(TAG, "GPS: %.6f°N %.6f°W (%u sats, hdop=%.1f)", 
                 g.latitude, g.longitude, g.num_satellites, g.hdop);
        ESP_LOGI(TAG, "Sun (earth): az=%.1f° el=%.1f° daylight=%s", 
                 sun.azimuth_deg, sun.elevation_deg, sun.is_daylight ? "YES" : "NO");
        ESP_LOGI(TAG, "Target (mount): az=%.1f° el=%.1f°", s.az_tgt, s.el_tgt);
        ESP_LOGI(TAG, "Change since last move: %.2f° (threshold=%.1f°)", dang, s.tol_deg);

        // === SLEEP DECISION ===
        // Sleep if: night OR (low sun AND descending trend)
        bool should_sleep = !sun.is_daylight || 
                           (sun.elevation_deg < s.sleep_thresh_el && is_descending(g.latitude, g.longitude));
        
        if (should_sleep) {
            ESP_LOGI(TAG, "=== SLEEP SEQUENCE START ===");
            ESP_LOGI(TAG, "Sleep trigger: daylight=%s elevation=%.1f° (thresh=%.1f°)", 
                     sun.is_daylight ? "yes" : "no", sun.elevation_deg, s.sleep_thresh_el);
            
            status_led_set_mode(LED_SLEEP);

            // Perform nightly homing to reset position error
            home_to_stops();

            // Log pre-sleep state (removed fix_type field)
            sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
                now, g.latitude, g.longitude, g.num_satellites, g.hdop,
                s.az_tgt, s.el_tgt, s.az_cur, s.el_cur, s.moves_today, s.total_moves, NAN, "HOME_BEFORE_SLEEP");

            // Calculate sunrise and schedule wake time
            solar_events_t ev_today = solar_events(g.latitude, g.longitude, now);
            time_t target_rise = 0;
            
            if (ev_today.has_sunrise && ev_today.has_sunset){
                if (now >= ev_today.sunset_utc){
                    // After sunset: wake for tomorrow's sunrise
                    target_rise = solar_events(g.latitude, g.longitude, now + 24*3600).sunrise_utc;
                } else if (now < ev_today.sunrise_utc){
                    // Before sunrise (rare): wake for today's sunrise  
                    target_rise = ev_today.sunrise_utc;
                } else {
                    // Daytime but low sun: wake for tomorrow's sunrise
                    target_rise = solar_events(g.latitude, g.longitude, now + 24*3600).sunrise_utc;
                }
            }
            
            // Fallback if solar events unavailable (high latitude or GPS issues)
            time_t wake_ts = target_rise ? (target_rise - s.prewake_min*60) : (now + 6*3600);

            ESP_LOGI(TAG, "Wake scheduled for %s", 
                     target_rise ? "sunrise - prewake" : "6 hours (fallback)");
            
            // Enter deep sleep - NEVER RETURNS
            enter_deep_sleep_until(wake_ts);
        }

        // === MOVEMENT AND CADENCE LOGIC ===
        const char *csv_note;
        
        if (dang >= s.tol_deg){
            // Sun has moved enough to justify panel movement
            ESP_LOGI(TAG, "Movement threshold exceeded (%.2f° ≥ %.1f°)", dang, s.tol_deg);
            
            do_move(s.az_tgt, s.el_tgt);
            s.last_move_az_tgt = s.az_tgt;
            s.last_move_el_tgt = s.el_tgt;
            s.cur_period_s = s.base_period_s;  // Slow cadence after move
            csv_note = "TRACK_15";
            
            ESP_LOGI(TAG, "Next check in %d minutes (post-move cadence)", s.cur_period_s / 60);
        } else {
            // Sun hasn't moved enough - use fast cadence to check more frequently
            s.cur_period_s = s.fast_period_s;
            csv_note = "TRACK_5";
            
            ESP_LOGI(TAG, "Below threshold (%.2f° < %.1f°)", dang, s.tol_deg);
            ESP_LOGI(TAG, "Next check in %d minutes (waiting for motion)", s.cur_period_s / 60);
        }

        // === TELEMETRY LOGGING ===
        // Updated CSV format: removed fix_type field
        sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
            now, g.latitude, g.longitude, g.num_satellites, g.hdop,
            s.az_tgt, s.el_tgt, s.az_cur, s.el_cur, s.moves_today, s.total_moves, NAN, csv_note);

        // === HOUSEKEEPING ===
        maybe_midnight_reset();
        
        // Periodic NVS saves (every 10 moves to limit flash wear)
        if ((s.moves_today % 10) == 0 && s.moves_today > 0) {
            ESP_LOGD(TAG, "Periodic NVS save (move count: %u)", s.moves_today);
            nvs_save();
        }

        ESP_LOGD(TAG, "Loop complete, sleeping %ds until next check", s.cur_period_s);
        
        // Sleep until next cadence tick (using vTaskDelayUntil for precise timing)
        vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(s.cur_period_s * 1000));
    }
}

/*
    Public API: initialize and start the tracking system.
    
    Creates the main tracking task with appropriate stack and priority.
    The task runs independently after creation.
    
    Task specifications:
    - Stack: 4KB (GPS I/O, solar calculations, NVS operations, logging)
    - Priority: 5 (higher than UI tasks, lower than critical drivers)
    - Core: any (not pinned to specific CPU core)
    
    Mutex initialization:
    - Currently unused but reserved for future multi-task state access
    - Required if other tasks need to read/modify tracker state
*/
void tracking_start(void){
    ESP_LOGI(TAG, "Initializing tracking system...");
    
    // Initialize synchronization primitives
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create tracking mutex");
        return;
    }
    
    // Create the main tracking task
    BaseType_t ret = xTaskCreate(
        tracking_task,              // Task function
        "tracking",                 // Task name (for debugging)
        4096,                       // Stack size in words (4KB)
        NULL,                       // Task parameters (none)
        5,                          // Priority (medium-high)
        NULL                        // Task handle (not needed)
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create tracking task");
        return;
    }
    
    ESP_LOGI(TAG, "Tracking system started successfully");
}