/*
 * Solar Tracking Module
 *
 * Purpose:
 *  Continuously tracks the sun using GPS location and time.
 *  Moves the panel in open loop with daily homing to remove drift.
 *
 * Loop outline:
 *  1) Read GPS (fresh if possible, otherwise cached)
 *  2) Compute sun az/el (earth frame)
 *  3) Apply saved mount offsets → targets (mount frame)
 *  4) Move if angular change exceeds threshold
 *  5) Log telemetry, adjust cadence
 *  6) At night: home → deep sleep → wake before sunrise
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
 * - az/el in mount frame (after offsets)
 * - dynamic cadence (fast while waiting, slower after move)
 * - nightly homing configuration
 */
static tracker_state_t s = {
    .az_cur=180, .el_cur=45,           // Initial assumed pose
    .tol_deg=10, .min_step_deg=2,      // Move thresholds
    .update_period_s=300,              // Legacy (kept for compatibility)
    .sleep_thresh_el=5,                // Sleep below this elevation (deg)
    .base_period_s=900,                // 15 min after move
    .fast_period_s=300,                // 5 min while waiting
    .cur_period_s=900,                 // Current cadence
    .prewake_min=10,                   // Wake before sunrise (minutes)
    .az_mount_offset_deg = 0.0,        // Install-time offset (earth→mount)
    .el_mount_offset_deg = 0.0,
    .home_az_deg = 0.0,                // Pose assigned after homing
    .home_el_deg = 85.0,
    .homing_time_ms = 22000,           // Time to reach mechanical stops
    .az_home_dir_level = 0,            // DIR level toward AZ stop
    .el_home_dir_level = 1,            // DIR level toward EL stop
    .last_move_az_tgt=180, .last_move_el_tgt=45
};

static SemaphoreHandle_t s_mutex;      // Reserved for future multi-thread access to 's'

/*
 * Save 's' to NVS (tracker/state).
 * Called after homing, calibration, and periodically.
 */
static void nvs_save(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READWRITE, &h);
    if (ret == ESP_OK){
        esp_err_t write_ret = nvs_set_blob(h, "state", &s, sizeof(s));
        if (write_ret == ESP_OK){
            nvs_commit(h);
            ESP_LOGD(TAG, "State saved: az=%.1f el=%.1f moves=%u",
                     s.az_cur, s.el_cur, s.total_moves);
        } else {
            ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(write_ret));
        }
        nvs_close(h);
    }
}

/*
 * Load 's' from NVS if present, otherwise keep defaults.
 */
static void nvs_load(void){
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READONLY, &h);
    if (ret == ESP_OK){
        size_t required_size = sizeof(s);
        ret = nvs_get_blob(h, "state", &s, &required_size);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Loaded state: az=%.1f° el=%.1f° moves=%u offsets=(%.2f,%.2f)",
                     s.az_cur, s.el_cur, s.total_moves,
                     s.az_mount_offset_deg, s.el_mount_offset_deg);
        }
        nvs_close(h);
    } else {
        ESP_LOGI(TAG, "No saved state, using defaults");
    }
}

/*
 * Reset daily counters at local midnight.
 * Called once per loop; cheap check.
 */
static void maybe_midnight_reset(void){
    time_t now = time(NULL);
    struct tm *lt = localtime(&now);
    if (lt && lt->tm_hour == 0 && lt->tm_min == 0) {
        if (s.moves_today > 0) {
            ESP_LOGI(TAG, "Midnight reset: %u moves yesterday", s.moves_today);
            sdlog_printf("Daily reset: %u moves", s.moves_today);
        }
        s.moves_today = 0;
    }
}

/*
 * Execute motion if either axis exceeds thresholds.
 * Updates current pose and stats after moves.
 */
static void do_move(double az_tgt, double el_tgt){
    double az_error = fabs(az_tgt - s.az_cur);
    double el_error = fabs(el_tgt - s.el_cur);

    bool move_az = (az_error > s.tol_deg) && (az_error > s.min_step_deg);
    bool move_el = (el_error > s.tol_deg) && (el_error > s.min_step_deg);

    if (!move_az && !move_el) {
        ESP_LOGI(TAG, "Within tolerance. No move needed.");
        ESP_LOGD(TAG, "  Errors: az=%.1f° el=%.1f° (tol=%.1f°)",
                 az_error, el_error, s.tol_deg);
        return;
    }

    ESP_LOGI(TAG, "Movement required:");
    ESP_LOGI(TAG, "  Current: az=%.1f° el=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  Target:  az=%.1f° el=%.1f°", az_tgt, el_tgt);
    ESP_LOGI(TAG, "  Moving: %s%s",
             move_az ? "AZ " : "",
             move_el ? "EL " : "");

    if (move_az) {
        motor_move_az(s.az_cur, az_tgt);   // Blocking move
        s.az_cur = az_tgt;                 // Accept new pose (open loop)
    }
    if (move_el) {
        motor_move_el(s.el_cur, el_tgt);   // Blocking move
        s.el_cur = el_tgt;
    }

    s.moves_today++;
    s.total_moves++;
    s.last_move = time(NULL);

    ESP_LOGI(TAG, "Move complete: az=%.1f° el=%.1f° (total: %u)",
             s.az_cur, s.el_cur, s.total_moves);

    sdlog_printf("Move #%u: az=%.1f° el=%.1f°",
                 s.total_moves, s.az_cur, s.el_cur);
}

/*
 * Enter deep sleep until the given UTC time.
 * Programs RTC timer, stops motors, sets LED, and sleeps.
 * System restarts on wake.
 */
static void enter_deep_sleep_until(time_t wake_utc){
    time_t now = time(NULL);
    int64_t delta_s = (int64_t)wake_utc - (int64_t)now;

    if (delta_s < 60) {                    // Avoid ultra-short sleeps
        ESP_LOGW(TAG, "Sleep too short (%lld s), using 60s", (long long)delta_s);
        delta_s = 60;
    }

    struct tm *wake_tm = localtime(&wake_utc);
    ESP_LOGI(TAG, "Deep sleep: %lld seconds (%.1f hours)",
             (long long)delta_s, delta_s / 3600.0);
    ESP_LOGI(TAG, "Wake time: %04d-%02d-%02d %02d:%02d:%02d",
             wake_tm->tm_year+1900, wake_tm->tm_mon+1, wake_tm->tm_mday,
             wake_tm->tm_hour, wake_tm->tm_min, wake_tm->tm_sec);

    sdlog_printf("Sleep: %lld s (wake @ %ld UTC)", (long long)delta_s, (long)wake_utc);

    status_led_set_mode(LED_SLEEP);        // Visual feedback
    motor_stop_all();                      // Safety: stop PWM
    esp_sleep_enable_timer_wakeup((uint64_t)delta_s * 1000000ULL);

    ESP_LOGI(TAG, "Entering deep sleep now...");
    esp_deep_sleep_start();                // Does not return
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
 * Compares elevation now vs +10 minutes.
 */
static bool is_descending(double lat, double lon){
    time_t now = time(NULL);
    sun_pos_t s0 = solar_compute(lat, lon, now);
    sun_pos_t s1 = solar_compute(lat, lon, now + 600);  // +10 min

    bool descending = s1.elevation_deg < s0.elevation_deg;
    ESP_LOGD(TAG, "Sun trend: %.2f° → %.2f° (%s)",
             s0.elevation_deg, s1.elevation_deg,
             descending ? "descending" : "ascending");
    return descending;
}

/*
 * Nightly homing: drive to mechanical stops and assign known angles.
 * Resets open-loop drift; persists updated pose.
 */
static void home_to_stops(void){
    ESP_LOGI(TAG, "=== HOMING SEQUENCE START ===");

    status_led_set_mode(LED_SLEEP);               // Solid off during homing

    ESP_LOGI(TAG, "AZ to retract stop...");
    motor_run_az_ms(s.az_home_dir_level, s.homing_time_ms);

    ESP_LOGI(TAG, "EL to extend stop...");
    motor_run_el_ms(s.el_home_dir_level, s.homing_time_ms);

    s.az_cur = s.home_az_deg;                     // Assign reference pose
    s.el_cur = s.home_el_deg;
    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;

    nvs_save();                                   // Persist new pose

    ESP_LOGI(TAG, "=== HOMING COMPLETE ===");
    ESP_LOGI(TAG, "Position reset: az=%.1f° el=%.1f°", s.az_cur, s.el_cur);

    sdlog_printf("HOMED: az=%.1f el=%.1f", s.az_cur, s.el_cur);
    status_led_set_mode(LED_TRACKING);
}

/*
 * Manual mount calibration:
 * - User points panel at the sun, then triggers calibration.
 * - Offsets = sun(earth) − panel(mount).
 * - Stores offsets in NVS.
 */
void tracking_calibrate_mount_offset_now(void){
    ESP_LOGI(TAG, "=== MOUNT CALIBRATION START ===");

    gps_data_t g = {0};
    bool gps_fresh = gps_poll_nav_pvt(&g);        // Prefer fresh fix
    bool gps_valid = gps_fresh || gps_get_last(&g);

    if (!gps_valid) {
        ESP_LOGW(TAG, "Calibration failed: no GPS fix");
        sdlog_printf("Calibration failed: no GPS");
        return;
    }

    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);

    ESP_LOGI(TAG, "Calibration data:");
    ESP_LOGI(TAG, "  GPS: %.6f°N %.6f°W (%u sats)",
             g.latitude, g.longitude, g.num_satellites);
    ESP_LOGI(TAG, "  Sun: az=%.2f° el=%.2f°", sun.azimuth_deg, sun.elevation_deg);
    ESP_LOGI(TAG, "  Panel: az=%.2f° el=%.2f°", s.az_cur, s.el_cur);

    double old_az = s.az_mount_offset_deg;
    double old_el = s.el_mount_offset_deg;

    s.az_mount_offset_deg = wrap360(sun.azimuth_deg - s.az_cur); // Earth→mount
    s.el_mount_offset_deg = sun.elevation_deg - s.el_cur;

    nvs_save();

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "Offsets: AZ %.3f° → %.3f°, EL %.3f° → %.3f°",
             old_az, s.az_mount_offset_deg,
             old_el, s.el_mount_offset_deg);

    sdlog_printf("Calibrated: az_off=%.2f el_off=%.2f",
                 s.az_mount_offset_deg, s.el_mount_offset_deg);
}

/*
 * Auto-calibration using compass:
 * - Requires compass calibration and valid GPS.
 * - Uses sun azimuth vs magnetic heading to derive az offset.
 * - EL offset left as 0.
 */
void tracking_auto_calibrate_with_compass(void) {
    ESP_LOGI(TAG, "=== AUTO CALIBRATION (COMPASS) ===");

    if (!gps_is_compass_calibrated()) {
        ESP_LOGW(TAG, "Compass not calibrated - run calibration first");
        return;
    }

    gps_data_t gps;
    if (!gps_get_last(&gps) || !gps.valid) {
        ESP_LOGW(TAG, "No valid GPS fix");
        return;
    }

    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(gps.latitude, gps.longitude, now);

    ESP_LOGI(TAG, "Sun: az=%.2f° el=%.2f°", sun.azimuth_deg, sun.elevation_deg);

    if (sun.elevation_deg < 15.0) {               // Avoid low-elevation error
        ESP_LOGW(TAG, "Sun too low (%.1f°), wait until >15°", sun.elevation_deg);
        return;
    }

    float compass_heading;
    if (!gps_get_compass_heading(&compass_heading)) {
        ESP_LOGE(TAG, "Failed to read compass");
        return;
    }

    ESP_LOGI(TAG, "Compass: %.2f° magnetic", compass_heading);

    double az_offset = compass_heading - sun.azimuth_deg;  // Magnetic vs sun
    while (az_offset > 180.0) az_offset -= 360.0;          // Normalize to [-180,180]
    while (az_offset < -180.0) az_offset += 360.0;

    s.az_mount_offset_deg = az_offset;
    s.el_mount_offset_deg = 0.0;
    nvs_save();

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "Mount offset: az=%.2f° el=%.2f°",
             s.az_mount_offset_deg, s.el_mount_offset_deg);

    sdlog_printf("Auto-calibration: az_off=%.2f", s.az_mount_offset_deg);
}

/*
 * Tracking task:
 * - Handles movement cadence, night transitions, and logging.
 */
static void tracking_task(void *arg){
    ESP_LOGI(TAG, "=== TRACKING SYSTEM START ===");

    nvs_load();                                     // Load persisted state

    const char *csv = "/sdcard/soltrac.csv";
    sdlog_write_csv_header_if_new(csv);             // Ensure header exists
    sdlog_printf("Tracking started: az=%.1f el=%.1f", s.az_cur, s.el_cur);

    s.last_move_az_tgt = s.az_cur;                  // Initialize deltas
    s.last_move_el_tgt = s.el_cur;

    ESP_LOGI(TAG, "Tracking loop starting (%ds cadence)", s.cur_period_s);

    // Optional auto-calibration on first boot (if compass ready and no offsets)
    if (gps_is_compass_calibrated()) {
        if (fabs(s.az_mount_offset_deg) < 0.1 && fabs(s.el_mount_offset_deg) < 0.1) {
            ESP_LOGI(TAG, "No offset stored - attempting auto-calibration");

            status_led_set_mode(LED_WAITING);
            gps_data_t gps;
            int wait_count = 0;
            while (!gps_poll_nav_pvt(&gps) && wait_count < 60) {
                ESP_LOGI(TAG, "Waiting for GPS... (%d/60)", ++wait_count);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            if (wait_count < 60) {
                tracking_auto_calibrate_with_compass();
            } else {
                ESP_LOGW(TAG, "GPS timeout - using defaults");
            }
        }
    }

    while (1){
        TickType_t loop_start = xTaskGetTickCount(); // For vTaskDelayUntil cadence

        // GPS: prefer fresh, fallback to cached
        gps_data_t g = {0};
        bool gps_fresh = gps_poll_nav_pvt(&g);
        bool gps_available = gps_fresh || gps_get_last(&g);

        if (!gps_available) {
            status_led_set_mode(LED_ERROR);
            ESP_LOGW(TAG, "No GPS data - retrying in 30s");
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        } else {
            if (status_led_get_mode() == LED_ERROR) {
                ESP_LOGI(TAG, "GPS recovered");
                status_led_set_mode(LED_TRACKING);
            }
        }

        // Sun position (earth frame) → targets (mount frame)
        time_t now = time(NULL);
        sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);
        s.az_tgt = wrap360(sun.azimuth_deg - s.az_mount_offset_deg);
        s.el_tgt = sun.elevation_deg - s.el_mount_offset_deg;

        // Change since last move (for threshold decision)
        double daz = fabs(wrap180(s.az_tgt - s.last_move_az_tgt));
        double del = fabs(s.el_tgt - s.last_move_el_tgt);
        double dang = fmax(daz, del);

        ESP_LOGI(TAG, "=== TRACKING STATUS ===");
        ESP_LOGI(TAG, "GPS: %.6f°N %.6f°W (%u sats)",
                 g.latitude, g.longitude, g.num_satellites);
        ESP_LOGI(TAG, "Sun: az=%.1f° el=%.1f° daylight=%s",
                 sun.azimuth_deg, sun.elevation_deg,
                 sun.is_daylight ? "YES" : "NO");
        ESP_LOGI(TAG, "Target: az=%.1f° el=%.1f°", s.az_tgt, s.el_tgt);
        ESP_LOGI(TAG, "Change: %.2f° (threshold=%.1f°)", dang, s.tol_deg);

        // Night detection: either not daylight or below threshold and descending
        bool should_sleep = !sun.is_daylight ||
                           (sun.elevation_deg < s.sleep_thresh_el &&
                            is_descending(g.latitude, g.longitude));

        if (should_sleep) {
            ESP_LOGI(TAG, "=== ENTERING SLEEP MODE ===");

            status_led_set_mode(LED_SLEEP);
            home_to_stops();                         // Reset reference before sleep

            sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
                now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
                s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
                s.moves_today, s.total_moves, NAN, "SLEEP");

            // Compute wake time (sunrise - prewake)
            solar_events_t ev = solar_events(g.latitude, g.longitude, now);
            time_t wake_ts = 0;

            if (ev.has_sunrise) {
                if (now >= ev.sunset_utc) {
                    wake_ts = solar_events(g.latitude, g.longitude, now + 24*3600).sunrise_utc;
                } else {
                    wake_ts = ev.sunrise_utc;
                }
                wake_ts -= s.prewake_min * 60;
            } else {
                wake_ts = now + 6*3600;             // Fallback if polar conditions
            }

            enter_deep_sleep_until(wake_ts);        // Does not return
        }

        // Movement and cadence selection
        const char *csv_note;

        if (dang >= s.tol_deg){
            ESP_LOGI(TAG, "Movement threshold exceeded (%.2f° ≥ %.1f°)",
                     dang, s.tol_deg);

            do_move(s.az_tgt, s.el_tgt);
            s.last_move_az_tgt = s.az_tgt;
            s.last_move_el_tgt = s.el_tgt;
            s.cur_period_s = s.base_period_s;       // Slow down after moving
            csv_note = "MOVE";

            ESP_LOGI(TAG, "Next check in %d minutes", s.cur_period_s / 60);
        } else {
            s.cur_period_s = s.fast_period_s;       // Speed up while waiting
            csv_note = "WAIT";

            ESP_LOGI(TAG, "Below threshold (%.2f° < %.1f°)", dang, s.tol_deg);
            ESP_LOGI(TAG, "Next check in %d minutes", s.cur_period_s / 60);
        }

        // CSV telemetry
        sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
            now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
            s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
            s.moves_today, s.total_moves, NAN, csv_note);

        // Daily maintenance and periodic persistence
        maybe_midnight_reset();

        if ((s.moves_today % 10) == 0 && s.moves_today > 0) {
            nvs_save();
        }

        // Wait until next loop (fixed cadence from loop_start)
        vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(s.cur_period_s * 1000));
    }
}

/*
 * Start tracking:
 * - Creates the task; returns immediately.
 */
void tracking_start(void){
    ESP_LOGI(TAG, "Initializing tracking...");

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    BaseType_t ret = xTaskCreate(tracking_task, "tracking", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return;
    }

    ESP_LOGI(TAG, "Tracking system started");
}

/*
 * Get current panel angles (mount frame).
 * Used by comms/UI.
 */
void tracking_get_current_angles(float *az_deg, float *el_deg) {
    if (az_deg) *az_deg = (float)s.az_cur;
    if (el_deg) *el_deg = (float)s.el_cur;
}

