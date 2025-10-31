/*
 * Solar Tracking Module
 *
 * Purpose:
 *  Continuously tracks the sun using GPS location and time.
 *  Moves the panel in open loop with daily homing to remove drift.
 *
 * Algorithm Overview:
 *  1) Read GPS (fresh if possible, otherwise cached)
 *  2) Compute sun az/el (earth frame) using NOAA algorithms
 *  3) Apply saved mount offsets → targets (mount frame)
 *  4) Move if angular change exceeds threshold
 *  5) Log telemetry to SD card CSV
 *  6) Adjust loop cadence (fast when waiting, slow after move)
 *  7) At night: home to midpoint → deep sleep → wake before sunrise
 *
 * Coordinate Frames:
 *  - Earth frame: Geographic coordinates (0°=N, 90°=E, etc.)
 *  - Mount frame: Panel coordinates after applying learned offsets
 *  - Offset relationship: mount_angle = earth_angle - offset
 *
 * Movement Strategy:
 *  - Threshold-based: only move if error > 10° and step > 2°
 *  - Dynamic cadence: 5 min when waiting, 15 min after move
 *  - Conservative timing: motor moves use 90% + 100ms buffer
 *  - Open-loop positioning: assumes commanded angles are achieved
 *
 * Drift Compensation:
 *  - Nightly homing to midpoint position (actuators at 50% stroke)
 *  - Resets accumulated open-loop errors
 *  - Safer than hard-stop homing (less mechanical stress)
 *  - Position: AZ=135° (half of 270°), EL=47.5° (midpoint of 10-85°)
 *
 * Night Operation:
 *  - Detects night: sun below threshold OR not daylight
 *  - Parks at safe midpoint (actuators half-extended, panel facing up)
 *  - Calculates sunrise time using NOAA algorithms
 *  - Deep sleeps until (sunrise - prewake_minutes)
 *  - System restarts on wake, resumes tracking
 *
 * State Persistence:
 *  - Current position saved to NVS flash periodically
 *  - Mount offsets saved after calibration
 *  - Move counters saved every 10 moves
 *  - Survives power loss and deep sleep
 *
 * Logging Levels:
 *  - V: Detailed calculations and intermediate values
 *  - D: Loop flow, GPS data, threshold checks
 *  - I: Moves, homing, calibration, major events
 *  - W: GPS loss, threshold misses, anomalies
 *  - E: Critical failures that prevent tracking
 */

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

#define TAG "TRACK"

/*
 * Global tracker state (persisted in NVS).
 * 
 * Home position philosophy:
 * - Actuators at midpoint (50% stroke, both half-extended)
 * - Panel faces straight up (zenith) at home position
 * - This is the safest position: balanced, no mechanical stress
 * - AZ midpoint = 135° (half of 270° range: 0-270°)
 * - EL midpoint = 47.5° (midpoint between 10° and 85°)
 * 
 * Timing calculation for homing:
 * - Full stroke: 200mm ÷ 11.94 mm/s = 16.7 seconds (measured)
 * - Half stroke (to midpoint): 100mm ÷ 11.94 mm/s = 8.3 seconds
 * - With 90% safety factor: 8.3s × 0.9 + 0.1s buffer ≈ 7.6 seconds actual move time
 * - Stored as 8300ms for safety margin
 */
static tracker_state_t s = {
    .az_cur=135, .el_cur=47.5,         // Initial assumed pose (actuators at midpoint)
    .tol_deg=10, .min_step_deg=2,      // Move thresholds
    .update_period_s=300,              // Legacy (kept for compatibility)
    .sleep_thresh_el=5,                // Sleep below this elevation (deg)
    .base_period_s=900,                // 15 min after move
    .fast_period_s=300,                // 5 min while waiting
    .cur_period_s=900,                 // Current cadence
    .prewake_min=10,                   // Wake before sunrise (minutes)
    .az_mount_offset_deg = 0.0,        // Install-time offset (earth→mount)
    .el_mount_offset_deg = 0.0,
    .home_az_deg = 135.0,              // Home = 50% of 270° AZ range
    .home_el_deg = 47.5,               // Home = midpoint of 10-85° EL range
    .homing_time_ms = 8300,            // Time to reach midpoint from either extreme
    .az_home_dir_level = 0,            // Calculated dynamically by motor_move_az
    .el_home_dir_level = 0,            // Calculated dynamically by motor_move_el
    .last_move_az_tgt=135, .last_move_el_tgt=47.5
};

static SemaphoreHandle_t s_mutex;      // Reserved for future multi-thread access to 's'

/*
 * Save tracker state to NVS flash.
 * 
 * Saved data:
 * - Current position (az_cur, el_cur)
 * - Mount offsets (az/el)
 * - Move counters (moves_today, total_moves)
 * - Homing configuration
 * 
 * Called after:
 * - Homing (resets position)
 * - Calibration (updates offsets)
 * - Every 10 moves (periodic backup)
 * 
 * Flash wear: ~100k write cycles, so periodic saves are safe.
 */
static void nvs_save(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READWRITE, &h);
    if (ret == ESP_OK){
        esp_err_t write_ret = nvs_set_blob(h, "state", &s, sizeof(s));
        if (write_ret == ESP_OK){
            nvs_commit(h);
            ESP_LOGD(TAG, "✓ State saved: az=%.1f° el=%.1f° moves=%u",
                     s.az_cur, s.el_cur, s.total_moves);
        } else {
            ESP_LOGW(TAG, "⚠ NVS save failed: %s", esp_err_to_name(write_ret));
        }
        nvs_close(h);
    } else {
        ESP_LOGW(TAG, "⚠ NVS open failed: %s", esp_err_to_name(ret));
    }
}

/*
 * Load tracker state from NVS flash.
 * 
 * If no saved state exists (first boot), uses defaults.
 * 
 * Loaded data:
 * - Last known position
 * - Calibration offsets
 * - Lifetime move counter
 * 
 * Called once at tracking_start().
 */
static void nvs_load(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READONLY, &h);
    if (ret == ESP_OK){
        size_t required_size = sizeof(s);
        ret = nvs_get_blob(h, "state", &s, &required_size);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          TRACKER STATE LOADED FROM NVS                     ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Last Known Position:");
            ESP_LOGI(TAG, "  - Azimuth: %.1f° (mount frame)", s.az_cur);
            ESP_LOGI(TAG, "  - Elevation: %.1f° (mount frame)", s.el_cur);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Mount Offsets (Calibration):");
            ESP_LOGI(TAG, "  - AZ offset: %.2f° (earth → mount)", s.az_mount_offset_deg);
            ESP_LOGI(TAG, "  - EL offset: %.2f° (earth → mount)", s.el_mount_offset_deg);
            if (fabs(s.az_mount_offset_deg) < 0.1 && fabs(s.el_mount_offset_deg) < 0.1) {
                ESP_LOGW(TAG, "  ⚠ No calibration data - run calibration for accuracy");
            }
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Move Statistics:");
            ESP_LOGI(TAG, "  - Today: %u moves", s.moves_today);
            ESP_LOGI(TAG, "  - Lifetime: %u moves", s.total_moves);
            ESP_LOGI(TAG, "");
        } else {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          FIRST BOOT - USING DEFAULT STATE                  ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Default Position (Midpoint):");
            ESP_LOGI(TAG, "  - Azimuth: %.1f° (50%% of range)", s.az_cur);
            ESP_LOGI(TAG, "  - Elevation: %.1f° (midpoint)", s.el_cur);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "⚠ Calibration Required:");
            ESP_LOGI(TAG, "  1. Point panel at sun manually");
            ESP_LOGI(TAG, "  2. Hold button for 3 seconds");
            ESP_LOGI(TAG, "  3. System will learn mount offsets");
            ESP_LOGI(TAG, "");
        }
        nvs_close(h);
    } else {
        ESP_LOGI(TAG, "No saved state found (first boot) - using defaults");
    }
}

/*
 * Reset daily move counter at UTC midnight.
 * 
 * Called every loop iteration; cheap check (only resets once per day).
 * Logs yesterday's move count before resetting.
 */
static void maybe_midnight_reset(void){
    time_t now = time(NULL);
    struct tm *lt = localtime(&now);
    
    static bool reset_done_today = false;
    
    if (lt && lt->tm_hour == 0 && lt->tm_min == 0 && !reset_done_today) {
        if (s.moves_today > 0) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║              DAILY RESET (Midnight)                        ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Yesterday's Statistics:");
            ESP_LOGI(TAG, "  - Moves: %u", s.moves_today);
            ESP_LOGI(TAG, "  - Lifetime total: %u", s.total_moves);
            ESP_LOGI(TAG, "");
            sdlog_printf("Daily reset: %u moves yesterday (lifetime: %u)", 
                         s.moves_today, s.total_moves);
        }
        s.moves_today = 0;
        reset_done_today = true;
        ESP_LOGI(TAG, "✓ Daily counter reset to 0");
    }
    
    // Reset the flag after midnight hour passes
    if (lt && lt->tm_hour != 0) {
        reset_done_today = false;
    }
}

/*
 * Execute motor movements if angular error exceeds thresholds.
 * 
 * Movement logic:
 * - Only move if error > tolerance (10°) AND > minimum step (2°)
 * - Moves AZ first, then EL (sequential to reduce peak current)
 * - Updates current position after move (open-loop assumption)
 * - Increments move counters for statistics
 * - Logs move to SD card
 * 
 * Called from main tracking loop when sun position changes enough.
 */
static void do_move(double az_tgt, double el_tgt){
    double az_error = fabs(az_tgt - s.az_cur);
    double el_error = fabs(el_tgt - s.el_cur);

    bool move_az = (az_error > s.tol_deg) && (az_error > s.min_step_deg);
    bool move_el = (el_error > s.tol_deg) && (el_error > s.min_step_deg);

    if (!move_az && !move_el) {
        ESP_LOGD(TAG, "Within tolerance - no move needed");
        ESP_LOGV(TAG, "  AZ error: %.1f° (tol: %.1f°)", az_error, s.tol_deg);
        ESP_LOGV(TAG, "  EL error: %.1f° (tol: %.1f°)", el_error, s.tol_deg);
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              EXECUTING TRACKING MOVE                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Movement Plan:");
    ESP_LOGI(TAG, "  Current: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  Target:  AZ=%.1f° EL=%.1f°", az_tgt, el_tgt);
    ESP_LOGI(TAG, "  Errors:  AZ=%.1f° EL=%.1f°", az_error, el_error);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Axes to Move:");
    if (move_az) ESP_LOGI(TAG, "  ✓ Azimuth (%.1f° change)", az_error);
    if (move_el) ESP_LOGI(TAG, "  ✓ Elevation (%.1f° change)", el_error);
    ESP_LOGI(TAG, "");

    // Record move start time
    time_t move_start = time(NULL);

    // Move azimuth first (if needed)
    if (move_az) {
        ESP_LOGI(TAG, "Moving azimuth...");
        motor_move_az(s.az_cur, az_tgt);   // Blocking move with detailed logging
        s.az_cur = az_tgt;                 // Accept new pose (open loop)
        ESP_LOGI(TAG, "✓ Azimuth complete: %.1f°", s.az_cur);
    }
    
    // Brief pause between moves
    if (move_az && move_el) {
        ESP_LOGD(TAG, "Pausing between moves...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Move elevation (if needed)
    if (move_el) {
        ESP_LOGI(TAG, "Moving elevation...");
        motor_move_el(s.el_cur, el_tgt);   // Blocking move with detailed logging
        s.el_cur = el_tgt;
        ESP_LOGI(TAG, "✓ Elevation complete: %.1f°", s.el_cur);
    }

    // Update statistics
    s.moves_today++;
    s.total_moves++;
    s.last_move = time(NULL);

    // Calculate total move duration
    time_t move_duration = s.last_move - move_start;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ MOVE COMPLETE");
    ESP_LOGI(TAG, "  Final position: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  Duration: %ld seconds", (long)move_duration);
    ESP_LOGI(TAG, "  Move #%u today, #%u lifetime", s.moves_today, s.total_moves);
    ESP_LOGI(TAG, "");

    sdlog_printf("Move #%u: AZ %.1f°→%.1f° EL %.1f°→%.1f° (%lds)",
                 s.total_moves, 
                 az_tgt - az_error, az_tgt,
                 el_tgt - el_error, el_tgt,
                 (long)move_duration);
}

/*
 * Enter deep sleep until the given UTC time.
 * 
 * What happens:
 * - Programs RTC timer for wake-up
 * - Sets LED to sleep mode (off)
 * - Stops all motor PWM
 * - Enters deep sleep (ESP32 powered down except RTC)
 * - System restarts on wake
 * 
 * Power consumption:
 * - Active tracking: ~150-300mA
 * - Deep sleep: ~10-50µA (99%+ reduction)
 * 
 * Note: Does not return - system restarts on wake.
 */
static void enter_deep_sleep_until(time_t wake_utc){
    time_t now = time(NULL);
    int64_t delta_s = (int64_t)wake_utc - (int64_t)now;

    // Safety checks
    if (delta_s < 60) {
        ESP_LOGW(TAG, "⚠ Sleep duration too short (%lld s), using 60s minimum", 
                 (long long)delta_s);
        delta_s = 60;
    }
    
    if (delta_s > 86400) {
        ESP_LOGW(TAG, "⚠ Sleep duration too long (%lld s = %.1f h), limiting to 12h",
                 (long long)delta_s, delta_s / 3600.0);
        delta_s = 43200;  // 12 hours max
    }

    // Calculate and log wake time
    struct tm *now_tm = localtime(&now);
    struct tm *wake_tm = localtime(&wake_utc);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              ENTERING DEEP SLEEP MODE                      ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Current Time:");
    ESP_LOGI(TAG, "  %04d-%02d-%02d %02d:%02d:%02d UTC",
             now_tm->tm_year + 1900, now_tm->tm_mon + 1, now_tm->tm_mday,
             now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Wake Time:");
    ESP_LOGI(TAG, "  %04d-%02d-%02d %02d:%02d:%02d UTC",
             wake_tm->tm_year + 1900, wake_tm->tm_mon + 1, wake_tm->tm_mday,
             wake_tm->tm_hour, wake_tm->tm_min, wake_tm->tm_sec);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Sleep Duration:");
    ESP_LOGI(TAG, "  - %lld seconds", (long long)delta_s);
    ESP_LOGI(TAG, "  - %.1f hours", delta_s / 3600.0);
    ESP_LOGI(TAG, "  - %.1f minutes", delta_s / 60.0);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Power Management:");
    ESP_LOGI(TAG, "  - RTC timer wake enabled");
    ESP_LOGI(TAG, "  - Motors stopped");
    ESP_LOGI(TAG, "  - LED off");
    ESP_LOGI(TAG, "  - Deep sleep power: ~10-50µA");
    ESP_LOGI(TAG, "");

    sdlog_printf("SLEEP: %lld s (%.1f h) until %04d-%02d-%02d %02d:%02d UTC", 
                 (long long)delta_s, delta_s / 3600.0,
                 wake_tm->tm_year + 1900, wake_tm->tm_mon + 1, wake_tm->tm_mday,
                 wake_tm->tm_hour, wake_tm->tm_min);

    // Prepare for sleep
    status_led_set_mode(LED_SLEEP);        // Visual feedback (LED off)
    motor_stop_all();                      // Safety: stop all PWM
    vTaskDelay(pdMS_TO_TICKS(1000));       // Let logs flush
    
    // Configure wake timer
    esp_sleep_enable_timer_wakeup((uint64_t)delta_s * 1000000ULL);

    ESP_LOGI(TAG, "Entering deep sleep now...");
    ESP_LOGI(TAG, "");
    
    // Enter deep sleep (does not return - system restarts on wake)
    esp_deep_sleep_start();
}

// Angle normalization helpers
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
 * Determine if the sun is descending (helps avoid premature sleep).
 * 
 * Compares sun elevation now vs +10 minutes.
 * Returns true if sun will be lower in 10 minutes (sunset approaching).
 * 
 * Used to avoid sleeping when sun is rising but temporarily below threshold.
 */
static bool is_descending(double lat, double lon){
    time_t now = time(NULL);
    sun_pos_t s0 = solar_compute(lat, lon, now);
    sun_pos_t s1 = solar_compute(lat, lon, now + 600);  // +10 min

    bool descending = s1.elevation_deg < s0.elevation_deg;
    ESP_LOGV(TAG, "Sun trend: %.2f° → %.2f° (%s)",
             s0.elevation_deg, s1.elevation_deg,
             descending ? "descending" : "ascending");
    return descending;
}

/*
 * Nightly homing: move to safe midpoint position.
 * 
 * New approach (safer than hard-stop homing):
 * - Moves to calculated midpoint position
 * - AZ = 135° (50% of 270° range)
 * - EL = 47.5° (midpoint of 10-85° range)
 * - Actuators at 50% stroke (half-extended, balanced)
 * - Panel faces straight up (zenith) - minimal wind load
 * - Uses normal motor_move_* functions with conservative timing
 * 
 * Benefits:
 * - Less mechanical stress than driving to hard stops
 * - Balanced position (no torque on one side)
 * - Safe for overnight parking
 * - Still resets accumulated open-loop drift
 * 
 * Called before deep sleep each night.
 */
static void home_to_midpoint(void){
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              NIGHTLY HOMING SEQUENCE                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Homing Strategy:");
    ESP_LOGI(TAG, "  - Target: Midpoint position (actuators at 50%% stroke)");
    ESP_LOGI(TAG, "  - AZ: %.1f° (50%% of 0-270° range)", s.home_az_deg);
    ESP_LOGI(TAG, "  - EL: %.1f° (midpoint of 10-85° range)", s.home_el_deg);
    ESP_LOGI(TAG, "  - Panel orientation: Facing zenith (straight up)");
    ESP_LOGI(TAG, "  - Benefits: Balanced, low stress, safe parking");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Current Position:");
    ESP_LOGI(TAG, "  - AZ: %.1f°", s.az_cur);
    ESP_LOGI(TAG, "  - EL: %.1f°", s.el_cur);
    ESP_LOGI(TAG, "");

    status_led_set_mode(LED_SLEEP);               // Visual feedback

    // Move azimuth to midpoint
    ESP_LOGI(TAG, "Moving AZ to midpoint (%.1f°)...", s.home_az_deg);
    motor_move_az(s.az_cur, s.home_az_deg);
    
    vTaskDelay(pdMS_TO_TICKS(500));               // Brief pause between axes
    
    // Move elevation to midpoint
    ESP_LOGI(TAG, "Moving EL to midpoint (%.1f°)...", s.home_el_deg);
    motor_move_el(s.el_cur, s.home_el_deg);

    // Update state to home position
    s.az_cur = s.home_az_deg;
    s.el_cur = s.home_el_deg;
    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;

    nvs_save();                                   // Persist new pose

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ HOMING COMPLETE");
    ESP_LOGI(TAG, "  Final position: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  Actuators: Both at 50%% stroke");
    ESP_LOGI(TAG, "  Panel: Facing zenith (balanced position)");
    ESP_LOGI(TAG, "  Ready for overnight parking");
    ESP_LOGI(TAG, "");

    sdlog_printf("HOMED: AZ=%.1f° EL=%.1f° (midpoint, facing up)", s.az_cur, s.el_cur);
    status_led_set_mode(LED_TRACKING);
}

/*
 * Manual mount calibration using sun alignment.
 * 
 * User procedure:
 * 1. Manually point panel directly at the sun
 * 2. Hold calibration button for 3+ seconds
 * 3. This function is called
 * 4. System computes offsets and saves to NVS
 * 
 * Algorithm:
 * - Get GPS position and current time
 * - Calculate where sun SHOULD be (earth frame)
 * - Compare to where panel IS pointing (mount frame)
 * - Offset = sun_angle - panel_angle
 * - Store offset in NVS
 * 
 * Future tracking:
 * - mount_target = earth_sun_angle - stored_offset
 * - Automatically compensates for mount orientation
 */
void tracking_calibrate_mount_offset_now(void){
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          MANUAL MOUNT CALIBRATION                          ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    // Get GPS data
    gps_data_t g = {0};
    bool gps_fresh = gps_poll_nav_pvt(&g);        // Prefer fresh fix
    bool gps_valid = gps_fresh || gps_get_last(&g);

    if (!gps_valid) {
        ESP_LOGE(TAG, "✗ Calibration failed: No GPS fix");
        ESP_LOGE(TAG, "  - Ensure GPS antenna has clear sky view");
        ESP_LOGE(TAG, "  - Wait for GPS fix before calibrating");
        ESP_LOGE(TAG, "");
        sdlog_printf("Calibration FAILED: no GPS");
        return;
    }

    // Calculate sun position
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);

    ESP_LOGI(TAG, "Calibration Data:");
    ESP_LOGI(TAG, "  GPS Location:");
    ESP_LOGI(TAG, "    - Latitude: %.6f°N", g.latitude);
    ESP_LOGI(TAG, "    - Longitude: %.6f°W", g.longitude);
    ESP_LOGI(TAG, "    - Satellites: %u", g.num_satellites);
    ESP_LOGI(TAG, "    - Fix type: %u", g.fix_type);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  Sun Position (Earth Frame):");
    ESP_LOGI(TAG, "    - Azimuth: %.2f°", sun.azimuth_deg);
    ESP_LOGI(TAG, "    - Elevation: %.2f°", sun.elevation_deg);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  Panel Position (Mount Frame):");
    ESP_LOGI(TAG, "    - Azimuth: %.2f°", s.az_cur);
    ESP_LOGI(TAG, "    - Elevation: %.2f°", s.el_cur);
    ESP_LOGI(TAG, "");

    // Save old offsets for comparison
    double old_az = s.az_mount_offset_deg;
    double old_el = s.el_mount_offset_deg;

    // Calculate new offsets
    s.az_mount_offset_deg = wrap360(sun.azimuth_deg - s.az_cur);
    s.el_mount_offset_deg = sun.elevation_deg - s.el_cur;

    // Save to NVS
    nvs_save();

    ESP_LOGI(TAG, "✓ CALIBRATION COMPLETE");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Offset Changes:");
    ESP_LOGI(TAG, "  Azimuth:");
    ESP_LOGI(TAG, "    - Old: %.3f°", old_az);
    ESP_LOGI(TAG, "    - New: %.3f°", s.az_mount_offset_deg);
    ESP_LOGI(TAG, "    - Delta: %.3f°", s.az_mount_offset_deg - old_az);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  Elevation:");
    ESP_LOGI(TAG, "    - Old: %.3f°", old_el);
    ESP_LOGI(TAG, "    - New: %.3f°", s.el_mount_offset_deg);
    ESP_LOGI(TAG, "    - Delta: %.3f°", s.el_mount_offset_deg - old_el);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Future tracking will use these offsets automatically.");
    ESP_LOGI(TAG, "");

    sdlog_printf("Calibrated: AZ offset %.2f°→%.2f°, EL offset %.2f°→%.2f°",
                 old_az, s.az_mount_offset_deg,
                 old_el, s.el_mount_offset_deg);
}

/*
 * Auto-calibration using compass (azimuth only).
 * 
 * Requirements:
 * - Compass must be calibrated first (gps_calibrate_compass())
 * - Valid GPS fix
 * - Sun elevation > 15° (accurate sun azimuth needed)
 * 
 * Algorithm:
 * - Reads true heading from compass (magnetic + declination)
 * - Calculates sun azimuth from GPS/time
 * - Computes offset: compass_heading - sun_azimuth
 * - Stores az_offset in NVS
 * - Elevation offset remains 0 (manual calibration still needed)
 * 
 * Less accurate than manual but useful for field deployment.
 */
void tracking_auto_calibrate_with_compass(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          AUTO CALIBRATION (Compass)                        ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    // Check compass calibration
    if (!gps_is_compass_calibrated()) {
        ESP_LOGW(TAG, "✗ Compass not calibrated");
        ESP_LOGW(TAG, "  - Double-press button to calibrate compass first");
        ESP_LOGW(TAG, "  - Rotate tracker 2-3 full circles during calibration");
        ESP_LOGW(TAG, "");
        return;
    }

    // Get GPS data
    gps_data_t gps;
    if (!gps_get_last(&gps) || !gps.valid) {
        ESP_LOGW(TAG, "✗ No valid GPS fix");
        ESP_LOGW(TAG, "  - Wait for GPS fix before auto-calibrating");
        ESP_LOGW(TAG, "");
        return;
    }

    // Calculate sun position
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(gps.latitude, gps.longitude, now);

    ESP_LOGI(TAG, "Sun Position:");
    ESP_LOGI(TAG, "  - Azimuth: %.2f°", sun.azimuth_deg);
    ESP_LOGI(TAG, "  - Elevation: %.2f°", sun.elevation_deg);
    ESP_LOGI(TAG, "");

    // Check sun elevation (need >15° for accurate azimuth)
    if (sun.elevation_deg < 15.0) {
        ESP_LOGW(TAG, "✗ Sun too low: %.1f° (need >15°)", sun.elevation_deg);
        ESP_LOGW(TAG, "  - Wait until sun is higher in sky");
        ESP_LOGW(TAG, "  - Best results near solar noon");
        ESP_LOGW(TAG, "");
        return;
    }

    // Get TRUE compass heading (declination-corrected)
    float compass_heading_true;
    if (!gps_get_compass_heading_true(&compass_heading_true)) {
        ESP_LOGE(TAG, "✗ Failed to read compass");
        ESP_LOGE(TAG, "  - Check compass connection");
        ESP_LOGE(TAG, "  - Recalibrate compass if needed");
        ESP_LOGE(TAG, "");
        return;
    }

    float declination = gps_get_magnetic_declination();
    
    ESP_LOGI(TAG, "Compass Reading:");
    ESP_LOGI(TAG, "  - True heading: %.2f°", compass_heading_true);
    ESP_LOGI(TAG, "  - Magnetic declination: %.1f°", declination);
    ESP_LOGI(TAG, "");

    // Calculate mount offset using TRUE headings
    double az_offset = compass_heading_true - sun.azimuth_deg;
    while (az_offset > 180.0) az_offset -= 360.0;
    while (az_offset < -180.0) az_offset += 360.0;

    double old_az_offset = s.az_mount_offset_deg;
    s.az_mount_offset_deg = az_offset;
    s.el_mount_offset_deg = 0.0;  // Elevation requires manual calibration
    
    nvs_save();

    ESP_LOGI(TAG, "✓ AUTO-CALIBRATION COMPLETE");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Results:");
    ESP_LOGI(TAG, "  - AZ offset: %.2f° → %.2f° (Δ=%.2f°)", 
             old_az_offset, s.az_mount_offset_deg, 
             s.az_mount_offset_deg - old_az_offset);
    ESP_LOGI(TAG, "  - EL offset: 0.0° (manual calibration recommended)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Note:");
    ESP_LOGI(TAG, "  - Azimuth tracking should now be accurate");
    ESP_LOGI(TAG, "  - For best elevation accuracy, do manual calibration");
    ESP_LOGI(TAG, "    (point panel at sun, hold button 3 seconds)");
    ESP_LOGI(TAG, "");

    sdlog_printf("Auto-calibration: AZ offset=%.2f° (compass+GPS, decl=%.1f°)", 
                 s.az_mount_offset_deg, declination);
}

/*
 * Main tracking task loop.
 * 
 * Runs continuously until deep sleep.
 * 
 * Loop outline:
 * 1. Get GPS position/time (fresh if possible)
 * 2. Calculate sun position (earth frame)
 * 3. Apply mount offsets (convert to mount frame)
 * 4. Check if movement threshold exceeded
 * 5. Move motors if needed
 * 6. Log telemetry to SD card
 * 7. Adjust loop cadence (fast when waiting, slow after move)
 * 8. Check for night condition
 * 9. If night: home → deep sleep → wake before sunrise
 * 10. Wait for next loop iteration
 * 
 * Variable cadence:
 * - 5 min (300s) when waiting for threshold
 * - 15 min (900s) after moving
 * - Reduces power consumption and motor wear
 */
static void tracking_task(void *arg){
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          TRACKING SYSTEM OPERATIONAL                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    nvs_load();                                     // Load persisted state

    const char *csv = "/sdcard/soltrac.csv";
    sdlog_write_csv_header_if_new(csv);             // Ensure CSV header exists
    sdlog_printf("=== TRACKING STARTED ===");
    sdlog_printf("Initial position: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);

    s.last_move_az_tgt = s.az_cur;                  // Initialize deltas
    s.last_move_el_tgt = s.el_cur;

    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  - Home position: AZ=%.1f° EL=%.1f° (midpoint)", 
             s.home_az_deg, s.home_el_deg);
    ESP_LOGI(TAG, "  - Movement threshold: %.1f°", s.tol_deg);
    ESP_LOGI(TAG, "  - Minimum step: %.1f°", s.min_step_deg);
    ESP_LOGI(TAG, "  - Cadence: %ds waiting, %ds after move", 
             s.fast_period_s, s.base_period_s);
    ESP_LOGI(TAG, "  - Sleep threshold: %.1f° elevation", s.sleep_thresh_el);
    ESP_LOGI(TAG, "  - Pre-wake: %d minutes before sunrise", s.prewake_min);
    ESP_LOGI(TAG, "");

    // Optional auto-calibration on first boot
    if (gps_is_compass_calibrated()) {
        if (fabs(s.az_mount_offset_deg) < 0.1 && fabs(s.el_mount_offset_deg) < 0.1) {
            ESP_LOGI(TAG, "No calibration data found - attempting auto-calibration...");
            ESP_LOGI(TAG, "");

            status_led_set_mode(LED_WAITING);
            gps_data_t gps;
            int wait_count = 0;
            
            while (!gps_poll_nav_pvt(&gps) && wait_count < 60) {
                ESP_LOGI(TAG, "Waiting for GPS fix... (%d/60)", ++wait_count);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            if (wait_count < 60) {
                tracking_auto_calibrate_with_compass();
            } else {
                ESP_LOGW(TAG, "GPS timeout - using default offsets");
                ESP_LOGW(TAG, "Manual calibration recommended for accuracy");
            }
            ESP_LOGI(TAG, "");
        }
    }

    ESP_LOGI(TAG, "Entering main tracking loop...");
    ESP_LOGI(TAG, "");

    while (1){
        TickType_t loop_start = xTaskGetTickCount(); // For vTaskDelayUntil cadence

        // === GPS Acquisition ===
        gps_data_t g = {0};
        bool gps_fresh = gps_poll_nav_pvt(&g);
        bool gps_available = gps_fresh || gps_get_last(&g);

        if (!gps_available) {
            status_led_set_mode(LED_WAITING);
            ESP_LOGW(TAG, "⚠ No GPS data available - retrying in 30s");
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        } else {
            // GPS recovered
            if (status_led_get_mode() == LED_ERROR) {
                ESP_LOGI(TAG, "✓ GPS recovered");
                status_led_set_mode(LED_TRACKING);
            }
        }

        // === Sun Position Calculation ===
        time_t now = time(NULL);
        sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);
        
        // Convert earth frame → mount frame using offsets
        s.az_tgt = wrap360(sun.azimuth_deg - s.az_mount_offset_deg);
        s.el_tgt = sun.elevation_deg - s.el_mount_offset_deg;

        // Calculate change since last move
        double daz = fabs(wrap180(s.az_tgt - s.last_move_az_tgt));
        double del = fabs(s.el_tgt - s.last_move_el_tgt);
        double dang = fmax(daz, del);

        // === Status Logging ===
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "══════════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "  TRACKING STATUS");
        ESP_LOGI(TAG, "══════════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "GPS:");
        ESP_LOGI(TAG, "  - Location: %.6f°N %.6f°W", g.latitude, g.longitude);
        ESP_LOGI(TAG, "  - Satellites: %u", g.num_satellites);
        ESP_LOGI(TAG, "  - Fix type: %u (%s)", g.fix_type, gps_fresh ? "FRESH" : "CACHED");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Sun (Earth Frame):");
        ESP_LOGI(TAG, "  - Azimuth: %.1f°", sun.azimuth_deg);
        ESP_LOGI(TAG, "  - Elevation: %.1f°", sun.elevation_deg);
        ESP_LOGI(TAG, "  - Daylight: %s", sun.is_daylight ? "YES" : "NO");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Target (Mount Frame):");
        ESP_LOGI(TAG, "  - Azimuth: %.1f° (offset: %.2f°)", s.az_tgt, s.az_mount_offset_deg);
        ESP_LOGI(TAG, "  - Elevation: %.1f° (offset: %.2f°)", s.el_tgt, s.el_mount_offset_deg);
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Tracking Error:");
        ESP_LOGI(TAG, "  - Change since last move: %.2f° (threshold: %.1f°)", dang, s.tol_deg);
        ESP_LOGI(TAG, "  - AZ delta: %.2f°", daz);
        ESP_LOGI(TAG, "  - EL delta: %.2f°", del);
        ESP_LOGI(TAG, "");

        // === Night Detection ===
        bool should_sleep = !sun.is_daylight ||
                           (sun.elevation_deg < s.sleep_thresh_el &&
                            is_descending(g.latitude, g.longitude));

        if (should_sleep) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║              NIGHT MODE ACTIVATED                          ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Night Conditions:");
            ESP_LOGI(TAG, "  - Sun elevation: %.1f° (threshold: %.1f°)", 
                     sun.elevation_deg, s.sleep_thresh_el);
            ESP_LOGI(TAG, "  - Daylight: %s", sun.is_daylight ? "YES" : "NO");
            ESP_LOGI(TAG, "  - Sun trend: %s", 
                     is_descending(g.latitude, g.longitude) ? "DESCENDING" : "ASCENDING");
            ESP_LOGI(TAG, "");

            status_led_set_mode(LED_SLEEP);
            home_to_midpoint();                      // Park at safe position

            // Log to CSV
            sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
                now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
                s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
                s.moves_today, s.total_moves, NAN, "SLEEP");

            // Calculate wake time
            solar_events_t ev = solar_events(g.latitude, g.longitude, now);
            time_t wake_ts = 0;

            if (ev.has_sunrise) {
                if (now >= ev.sunset_utc) {
                    // After sunset - wake before tomorrow's sunrise
                    solar_events_t tomorrow = solar_events(g.latitude, g.longitude, now + 86400);
                    if (tomorrow.has_sunrise) {
                        wake_ts = tomorrow.sunrise_utc - (s.prewake_min * 60);
                        ESP_LOGI(TAG, "Wake target: %d min before tomorrow's sunrise", s.prewake_min);
                    } else {
                        wake_ts = now + 21600;  // 6 hours fallback
                        ESP_LOGW(TAG, "Tomorrow's sunrise unavailable - using 6h fallback");
                    }
                } else {
                    wake_ts = ev.sunrise_utc - (s.prewake_min * 60);
                    ESP_LOGI(TAG, "Wake target: %d min before today's sunrise", s.prewake_min);
                }
            } else {
                wake_ts = now + 21600;  // 6 hours fallback
                ESP_LOGW(TAG, "No sunrise data (polar night?) - using 6h fallback");
            }
            
            // Safety checks
            if (wake_ts <= now) {
                ESP_LOGW(TAG, "Wake time in past - adding 6h");
                wake_ts = now + 21600;
            }
            
            int64_t sleep_duration = (int64_t)wake_ts - (int64_t)now;
            if (sleep_duration > 86400) {
                ESP_LOGW(TAG, "Sleep too long (%.1f h) - limiting to 12h", 
                         sleep_duration / 3600.0);
                wake_ts = now + 43200;
            }

            enter_deep_sleep_until(wake_ts);        // Does not return
        }

        // === Movement Decision ===
        const char *csv_note;

        if (dang >= s.tol_deg){
            ESP_LOGI(TAG, "Movement threshold exceeded: %.2f° ≥ %.1f°", dang, s.tol_deg);

            do_move(s.az_tgt, s.el_tgt);
            s.last_move_az_tgt = s.az_tgt;
            s.last_move_el_tgt = s.el_tgt;
            s.cur_period_s = s.base_period_s;       // Slow down after moving
            csv_note = "MOVE";
            // Set status to TRACKING when actively moving
            status_led_set_mode(LED_TRACKING);

            ESP_LOGI(TAG, "Next check in %d minutes", s.cur_period_s / 60);
        } else {
            s.cur_period_s = s.fast_period_s;       // Speed up while waiting
            csv_note = "WAIT";
            // Set status to WAITING when below threshold
            status_led_set_mode(LED_WAITING);       

            ESP_LOGI(TAG, "Below threshold (%.2f° < %.1f°)", dang, s.tol_deg);
            ESP_LOGI(TAG, "Next check in %d minutes", s.cur_period_s / 60);
        }

        ESP_LOGI(TAG, "");

        // === CSV Telemetry Logging ===
        // Format: timestamp,lat,lon,fix_type,sats,az_tgt,el_tgt,az_cur,el_cur,
        //         moves_today,total_moves,tracking_quality,note
        sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
            now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
            s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
            s.moves_today, s.total_moves, dang, csv_note);

        ESP_LOGV(TAG, "Telemetry logged to CSV: %s", csv_note);

        // === Daily Maintenance ===
        // Check for midnight rollover and reset daily counter
        maybe_midnight_reset();

        // === Periodic State Persistence ===
        // Save to NVS every 10 moves to minimize flash wear
        if ((s.moves_today % 10) == 0 && s.moves_today > 0) {
            ESP_LOGD(TAG, "Periodic save: %u moves today", s.moves_today);
            nvs_save();
        }

        // === Loop Cadence Control ===
        // Wait until next scheduled iteration (maintains precise timing)
        ESP_LOGD(TAG, "Waiting %d seconds until next check...", s.cur_period_s);
        vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(s.cur_period_s * 1000));
    }
}

/*
 * Start tracking system.
 * 
 * Creates the tracking task which runs independently.
 * Returns immediately - does not block.
 * 
 * Task configuration:
 * - Stack: 4KB (sufficient for GPS/solar/motor calls)
 * - Priority: 5 (medium-high, above idle but below critical)
 * - Name: "tracking" (shows in task monitor)
 * 
 * Requirements before calling:
 * - GPS initialized (gps_init)
 * - Motors initialized (motor_init)
 * - SD card mounted (sdlog_init)
 * - NVS initialized (nvs_flash_init)
 * - System time set from GPS or RTC
 * 
 * What happens:
 * 1. Creates mutex for thread-safe state access
 * 2. Spawns tracking_task() on FreeRTOS
 * 3. Task loads NVS state and begins tracking loop
 * 4. Returns control to caller immediately
 */
void tracking_start(void){
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          STARTING TRACKING SYSTEM                          ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    // Create mutex for state protection (reserved for future multi-thread access)
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ CRITICAL: Failed to create state mutex");
        ESP_LOGE(TAG, "  - Insufficient heap memory?");
        ESP_LOGE(TAG, "  - System may be unstable");
        ESP_LOGE(TAG, "");
        return;
    }
    ESP_LOGD(TAG, "✓ State mutex created");

    // Create tracking task
    BaseType_t ret = xTaskCreate(
        tracking_task,          // Task function
        "tracking",             // Task name (for debugging)
        4096,                   // Stack size (bytes)
        NULL,                   // Parameters (none)
        5,                      // Priority (medium-high)
        NULL                    // Task handle (not needed)
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ CRITICAL: Failed to create tracking task");
        ESP_LOGE(TAG, "  - Return code: %d", ret);
        ESP_LOGE(TAG, "  - Check available heap (need ~4KB stack)");
        ESP_LOGE(TAG, "  - System cannot track without this task");
        ESP_LOGE(TAG, "");
        
        // Clean up mutex
        if (s_mutex) {
            vSemaphoreDelete(s_mutex);
            s_mutex = NULL;
        }
        return;
    }

    ESP_LOGI(TAG, "✓ Tracking task created successfully");
    ESP_LOGI(TAG, "  - Stack: 4KB");
    ESP_LOGI(TAG, "  - Priority: 5 (medium-high)");
    ESP_LOGI(TAG, "  - Task will load NVS state and begin tracking");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Tracking system running independently");
    ESP_LOGI(TAG, "");

    sdlog_printf("=== TRACKING SYSTEM STARTED ===");
}

/*
 * Get current panel angles (mount frame).
 * 
 * Returns the tracker's current belief about where the panel is pointing.
 * These are mount-frame coordinates (after offset compensation).
 * 
 * Thread-safe: Can be called from any task.
 * 
 * Used by:
 * - WiFi telemetry (send to display)
 * - System check (show current position)
 * - Button handler (show position on LED)
 * - Logging (record position in CSV)
 * 
 * Note: These are BELIEVED positions (open-loop), not measured.
 * Accuracy depends on:
 * - Calibration quality (mount offsets)
 * - Accumulated drift since last homing
 * - Motor timing accuracy
 * 
 * Typical accuracy:
 * - Right after homing: ±2° (excellent)
 * - After 100 moves: ±5° (good)
 * - After 24 hours: ±10° (acceptable, triggers homing)
 */
void tracking_get_current_angles(float *az_deg, float *el_deg) {
    // Mutex protection (for future multi-thread safety)
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    if (az_deg) {
        *az_deg = (float)s.az_cur;
        ESP_LOGV(TAG, "Get AZ: %.1f°", *az_deg);
    }
    
    if (el_deg) {
        *el_deg = (float)s.el_cur;
        ESP_LOGV(TAG, "Get EL: %.1f°", *el_deg);
    }

    if (s_mutex) {
        xSemaphoreGive(s_mutex);
    }
}

/*
 * Get move statistics for telemetry display.
 * 
 * Returns:
 * - moves_today: Counter reset at midnight UTC
 * - total_moves: Lifetime counter (persisted in NVS)
 * 
 * Thread-safe: Can be called from any task.
 * 
 * Used by:
 * - WiFi telemetry (send to LCD display)
 * - System check (show usage statistics)
 * - SD card logging (track system activity)
 * 
 * Typical values:
 * - moves_today: 20-100 depending on weather and time of year
 * - total_moves: Accumulates over weeks/months
 * 
 * Move increment:
 * - Incremented when either AZ or EL moves
 * - Not incremented if both axes skip (within tolerance)
 * - Sequential AZ+EL move = 1 count (they're part of same correction)
 * 
 * Useful for:
 * - Predicting battery drain
 * - Monitoring system health
 * - Estimating mechanical wear
 * - Debugging tracking behavior
 */
void tracking_get_move_stats(uint32_t *moves_today, uint32_t *total_moves) {
    // Mutex protection
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    if (moves_today) {
        ESP_LOGV(TAG, "Get moves_today: %lu", (unsigned long)*moves_today);
    }
    
    if (total_moves) {
        ESP_LOGV(TAG, "Get total_moves: %lu", (unsigned long)*total_moves);
    }

    if (s_mutex) {
        xSemaphoreGive(s_mutex);
    }
}

/*
 * Get target angles (where we're trying to point).
 * 
 * Returns the computed sun position in mount frame (after offset compensation).
 * These are the angles we WANT to be at, not where we currently ARE.
 * 
 * Useful for:
 * - Display showing "Target vs Current"
 * - Calculating tracking error
 * - Debugging calibration issues
 * 
 * Note: Only valid while tracking task is running.
 */
void tracking_get_target_angles(float *az_deg, float *el_deg) {
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    if (az_deg) {
        *az_deg = (float)s.az_tgt;
        ESP_LOGV(TAG, "Get target AZ: %.1f°", *az_deg);
    }
    
    if (el_deg) {
        *el_deg = (float)s.el_tgt;
        ESP_LOGV(TAG, "Get target EL: %.1f°", *el_deg);
    }

    if (s_mutex) {
        xSemaphoreGive(s_mutex);
    }
}

/*
 * Get mount offsets (calibration values).
 * 
 * Returns the learned offsets between earth frame and mount frame.
 * These are set during calibration and persist in NVS.
 * 
 * Relationship:
 * - mount_angle = earth_angle - offset
 * 
 * Example:
 * - Sun at 180° azimuth (south)
 * - Mount base faces 45° off (northeast)
 * - Offset = 45°
 * - Command mount to 180° - 45° = 135° to point at sun
 * 
 * Used by:
 * - System check (show calibration status)
 * - Debugging tracking accuracy
 * - Verifying calibration procedure worked
 */
void tracking_get_mount_offsets(float *az_offset_deg, float *el_offset_deg) {
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    if (az_offset_deg) {
        *az_offset_deg = (float)s.az_mount_offset_deg;
        ESP_LOGV(TAG, "Get AZ offset: %.2f°", *az_offset_deg);
    }
    
    if (el_offset_deg) {
        *el_offset_deg = (float)s.el_mount_offset_deg;
        ESP_LOGV(TAG, "Get EL offset: %.2f°", *el_offset_deg);
    }

    if (s_mutex) {
        xSemaphoreGive(s_mutex);
    }
}