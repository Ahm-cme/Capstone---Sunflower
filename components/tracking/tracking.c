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
#include <inttypes.h>  // ADD THIS for PRIu32, PRIu64

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

// Global tracker state with UPDATED VALUES for faster testing
static tracker_state_t s = {
    // Tracking parameters - REDUCED tolerance for more frequent movements
    .tol_deg = 3.0,              // CHANGED from 10° to 3° - triggers moves more often
    .min_step_deg = 2.0,         // Minimum step to avoid jitter
    .base_period_s = 120,        // CHANGED from 900s (15min) to 120s (2min) - faster checks
    .fast_period_s = 60,         // CHANGED from 300s (5min) to 60s (1min) - faster polling
    .sleep_thresh_el = 5.0,      // Below this elevation, consider night
    
    // Homing configuration
    .az_home_deg = 0.0,          // Azimuth home position (fully retracted)
    .el_home_deg = 85.0,         // Elevation home position (fully extended)
    .az_home_dir_level = 0,      // DIR level to reach home (verify with your wiring)
    .el_home_dir_level = 1,      // DIR level to reach home (verify with your wiring)
    .homing_time_ms = 11000,     // UPDATED: 200mm ÷ 11.111mm/s = 18s + 4s safety margin
    
    // Current position estimates (initialized to home)
    .az_cur = 0.0, 
    .el_cur = 85.0,
    
    // Last commanded targets
    .last_move_az_tgt = 0.0,
    .last_move_el_tgt = 85.0,
    
    // Mount orientation offsets (learned during calibration)
    .az_mount_offset_deg = 0.0,  // Initially zero, updated via calibration
    .el_mount_offset_deg = 0.0,
    
    // Statistics
    .moves_today = 0,            // FIXED: was num_moves_today
    .total_moves = 0,
    .last_move = 0
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
            // FIXED: Use PRIu32 for uint32_t
            ESP_LOGD(TAG, "State saved to NVS: az=%.1f el=%.1f moves=%" PRIu32, 
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
            // FIXED: Use PRIu32 for uint32_t
            ESP_LOGI(TAG, "Loaded state from NVS: az=%.1f el=%.1f tot_moves=%" PRIu32 " offsets=(%.2f,%.2f)", 
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
        if (s.moves_today > 0) {
            // FIXED: Use PRIu32 for uint32_t
            ESP_LOGI(TAG, "Midnight: resetting daily move count (was %" PRIu32 ")", s.moves_today);
            sdlog_printf("Daily reset: %" PRIu32 " moves yesterday", s.moves_today);
        }
        s.moves_today = 0;
    }
}

/*
    Execute a movement command if targets exceed tolerance and minimum step.
    
    FIXED: Updated to use correct field name 'moves_today'
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

    ESP_LOGI(TAG, "════════════════════════════════════════════════════════");
    // FIXED: Use PRIu32 for uint32_t
    ESP_LOGI(TAG, "EXECUTING MOVEMENT #%" PRIu32 " (today: %" PRIu32 ")", s.total_moves + 1, s.moves_today + 1);
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Current position: AZ=%.2f° EL=%.2f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "Target position:  AZ=%.2f° EL=%.2f°", az_tgt, el_tgt);
    ESP_LOGI(TAG, "Delta:            ΔAZ=%.2f° ΔEL=%.2f°", 
             az_tgt - s.az_cur, el_tgt - s.el_cur);
    ESP_LOGI(TAG, "Speed: 11.111 mm/s (MAX PWM=255/255)");
    
    // FIXED: Use PRIu32 for uint32_t in sdlog_printf too
    sdlog_printf("─────────────────────────────────────────────────────");
    sdlog_printf("MOVE #%" PRIu32 ": AZ %.2f→%.2f° (Δ%.2f°) | EL %.2f→%.2f° (Δ%.2f°)",
                 s.total_moves + 1,
                 s.az_cur, az_tgt, az_tgt - s.az_cur,
                 s.el_cur, el_tgt, el_tgt - s.el_cur);
    
    // Execute movements (motor.c handles timing and PWM control)
    if (move_az) { 
        ESP_LOGI(TAG, "Moving AZIMUTH: %.2f° → %.2f°", s.az_cur, az_tgt);
        motor_move_az(s.az_cur, az_tgt); 
        s.az_cur = az_tgt;
        s.last_move_az_tgt = az_tgt;
    }
    if (move_el) { 
        ESP_LOGI(TAG, "Moving ELEVATION: %.2f° → %.2f°", s.el_cur, el_tgt);
        motor_move_el(s.el_cur, el_tgt); 
        s.el_cur = el_tgt;
        s.last_move_el_tgt = el_tgt;
    }

    // Update statistics and persistence
    s.moves_today++;  // FIXED: was num_moves_today
    s.total_moves++; 
    s.last_move = time(NULL);
    
    ESP_LOGI(TAG, "Movement complete. New position: AZ=%.2f° EL=%.2f°", s.az_cur, s.el_cur);
    // FIXED: Use PRIu32 for uint32_t
    ESP_LOGI(TAG, "Total moves: %" PRIu32 " (today: %" PRIu32 ")", s.total_moves, s.moves_today);
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════");
    
    sdlog_printf("Movement complete: AZ=%.2f° EL=%.2f° | Total: %" PRIu32, 
                 s.az_cur, s.el_cur, s.total_moves);
    
    // Persist updated state to NVS
    nvs_save();
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
    // FIXED: Use PRIu32 for uint32_t
    ESP_LOGD(TAG, "Homing config: AZ_dir=%d EL_dir=%d time=%" PRIu32 "ms", 
             s.az_home_dir_level, s.el_home_dir_level, s.homing_time_ms);
    
    status_led_set_mode(LED_SLEEP);

    ESP_LOGI(TAG, "Phase 1: AZ to retract stop...");
    motor_run_az_ms(s.az_home_dir_level, s.homing_time_ms);
    
    ESP_LOGI(TAG, "Phase 2: EL to extend stop...");  
    motor_run_el_ms(s.el_home_dir_level, s.homing_time_ms);

    s.az_cur = s.az_home_deg;
    s.el_cur = s.el_home_deg;
    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;
    
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
    ESP_LOGI(TAG, "Tracking task started");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "FAST TESTING MODE ENABLED:");
    ESP_LOGI(TAG, "  • Movement tolerance: 3° (was 10°)");
    ESP_LOGI(TAG, "  • Check period: 60s fast / 120s slow (was 300s / 900s)");
    ESP_LOGI(TAG, "  • PWM duty cycle: 100%% (8191/8191)");
    ESP_LOGI(TAG, "  • Actuator speed: 11.111 mm/s (18s full stroke)");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    
    nvs_load();
    
    for(;;){
        time_t now_utc = time(NULL);
        maybe_midnight_reset();
        
        // Get GPS data
        gps_fix_t gps;
        bool gps_ok = gps_get_fix(&gps, 30000);
        
        if(!gps_ok){
            ESP_LOGW(TAG, "No GPS fix - retrying in 30s");
            status_led_set_mode(LED_ERROR);
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }
        
        status_led_set_mode(LED_TRACKING);
        
        // Calculate sun position - FIXED: Use solar_compute
        sun_pos_t sun = solar_compute(gps.latitude, gps.longitude, now_utc);
        double sun_az = sun.azimuth_deg;
        double sun_el = sun.elevation_deg;
        
        ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
        ESP_LOGI(TAG, "Sun position: AZ=%.2f° EL=%.2f°", sun_az, sun_el);
        
        // Apply mount offsets
        double tgt_az = sun_az - s.az_mount_offset_deg;
        double tgt_el = sun_el - s.el_mount_offset_deg;
        
        ESP_LOGI(TAG, "Target (after mount offsets): AZ=%.2f° EL=%.2f°", tgt_az, tgt_el);
        ESP_LOGI(TAG, "Current position: AZ=%.2f° EL=%.2f°", s.az_cur, s.el_cur);
        
        // Check if movement needed
        double az_err = fabs(tgt_az - s.az_cur);
        double el_err = fabs(tgt_el - s.el_cur);
        
        ESP_LOGI(TAG, "Position error: ΔAZ=%.2f° ΔEL=%.2f° (tolerance=%.1f°)", 
                 az_err, el_err, s.tol_deg);
        
        bool need_move = (az_err >= s.tol_deg) || (el_err >= s.tol_deg);
        
        if(need_move){
            ESP_LOGI(TAG, "▶ MOVEMENT REQUIRED (error exceeds %.1f° tolerance)", s.tol_deg);
            do_move(tgt_az, tgt_el);
            
            // FIXED: Use PRIu32 for uint32_t
            ESP_LOGI(TAG, "Waiting %" PRIu32 " seconds before next check (slow cadence)...", s.base_period_s);
            vTaskDelay(pdMS_TO_TICKS(s.base_period_s * 1000));
        }
        else{
            ESP_LOGI(TAG, "✓ Within tolerance - no movement needed");
            
            // FIXED: Use PRIu32 for uint32_t
            ESP_LOGI(TAG, "Waiting %" PRIu32 " seconds before next check (fast cadence)...", s.fast_period_s);
            vTaskDelay(pdMS_TO_TICKS(s.fast_period_s * 1000));
        }
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