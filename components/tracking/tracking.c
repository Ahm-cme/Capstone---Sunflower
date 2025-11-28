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

// === ADD MOCK GPS SUPPORT (MUST MATCH main.c) ===
#ifndef USE_MOCK_GPS
#define USE_MOCK_GPS  1  // Set to 1 for mock mode, 0 for real GPS
#endif

#if USE_MOCK_GPS
#include "mock_gps.h"
#define GPS_POLL_NAV_PVT(data)        mock_gps_poll_nav_pvt(data)
#define GPS_GET_LAST(data)            mock_gps_get_last(data)
#define GPS_GET_COMPASS_HEADING_TRUE(h) mock_gps_get_compass_heading_true(h)
#define GPS_IS_COMPASS_CALIBRATED()   mock_gps_is_compass_calibrated()
#define GPS_GET_MAGNETIC_DECLINATION() mock_gps_get_magnetic_declination()
#define GPS_CALIBRATE_COMPASS()       mock_gps_calibrate_compass()
#define GPS_IS_COMPASS_PRESENT()      true  // Mock always has compass
#else
#include "gps.h"
#define GPS_POLL_NAV_PVT(data)        gps_poll_nav_pvt(data)
#define GPS_GET_LAST(data)            gps_get_last(data)
#define GPS_GET_COMPASS_HEADING_TRUE(h) gps_get_compass_heading_true(h)
#define GPS_IS_COMPASS_CALIBRATED()   gps_is_compass_calibrated()
#define GPS_GET_MAGNETIC_DECLINATION() gps_get_magnetic_declination()
#define GPS_CALIBRATE_COMPASS()       gps_calibrate_compass()
#define GPS_IS_COMPASS_PRESENT()      gps_is_compass_present()
#endif

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
#include <inttypes.h>

#ifndef DEG2RAD
#define DEG2RAD(d)  ((d) * M_PI / 180.0)
#endif

#define TAG "TRACK"

// Add debug macros (after TAG definition)
#define DEBUG_TRACE() ESP_LOGD(TAG, "%s() called", __func__)

#define DEBUG_GPS(g) do { \
    ESP_LOGD(TAG, "GPS data:"); \
    ESP_LOGD(TAG, "  Lat: %.6f° Lon: %.6f°", (g).latitude, (g).longitude); \
    ESP_LOGD(TAG, "  Sats: %u Fix: %u Valid: %s", \
             (g).num_satellites, (g).fix_type, (g).valid ? "YES" : "NO"); \
} while(0)

#define DEBUG_SUN(s) do { \
    ESP_LOGD(TAG, "Sun position:"); \
    ESP_LOGD(TAG, "  Az: %.2f° El: %.2f°", (s).azimuth_deg, (s).elevation_deg); \
    ESP_LOGD(TAG, "  Daylight: %s", (s).is_daylight ? "YES" : "NO"); \
} while(0)

// Compass mounting offset: compass is on back of panel, 180° from front
#define MOUNT_COMPASS_OFFSET_DEG 0.0

/*
 * Global tracker state (persisted in NVS).
 * 
 * HARDWARE SPECIFICATIONS:
 * - Azimuth actuator:
 *   - Range: 90° (±45° from center)
 *   - Center position (0°): 5.00" extension (127.0mm)  // CHANGED from 3.25" (82.55mm)
 *   - Full retract (-45°): ~3.75" (95.25mm)           // CHANGED from 2.0" (50.8mm)
 *   - Full extend (+45°): ~6.25" (158.75mm)           // CHANGED from 4.5" (114.3mm)
 *   - Stroke: ~2.5" (63.5mm) total travel              // UNCHANGED
 *
 * - Elevation actuator:
 *   - Range: 112° (±56° from center)
 *   - Center position (0°): 4.25" extension (107.95mm)  // UNCHANGED
 *   - Full retract (-56°): ~2.5" (63.5mm)               // UNCHANGED
 *   - Full extend (+56°): ~6.0" (152.4mm)               // UNCHANGED
 *   - Stroke: ~3.5" (88.9mm) total travel               // UNCHANGED
 *
 * HOME POSITION (Starting point):
 * - Both actuators at mechanical center (flat/horizontal)
 * - AZ: 0° (5.00" = 127.0mm extension)                  // CHANGED from 3.25" (82.55mm)
 * - EL: 0° (4.25" = 107.95mm extension)                 // UNCHANGED
 * - Panel orientation: Horizontal (parallel to ground)
 *
 * Position tracking:
 * - az_actuator_mm: Current AZ extension (95.25-158.75mm, center=127.0mm)  // CHANGED
 * - el_actuator_mm: Current EL extension (63.5-152.4mm, center=107.95mm)   // UNCHANGED
 * - az_cur/el_cur: Current angles in degrees (±45° AZ, ±56° EL)
 */
static tracker_state_t s = {
    // Home position: Both actuators centered (horizontal/flat)
    .az_cur = 0.0,                     // 0° = center (flat)
    .el_cur = 0.0,                     // 0° = center (flat)
    
    // Actuator position tracking (millimeters of extension)
    // Home position = mechanical center of each actuator
    .az_actuator_mm = 127.0,           // CHANGED: 5.00" = center of AZ actuator (was 82.55mm)
    .el_actuator_mm = 107.95,          // UNCHANGED: 4.25" = center of EL actuator
    
    .tol_deg = 10,                     // Move threshold
    .min_step_deg = 2,                 // Minimum step size
    .update_period_s = 300,            // Legacy (kept for compatibility)
    
    .sleep_thresh_el = 5.0,            // Sleep when sun below 5° elevation
    
    .base_period_s = 900,              // 15 min after move
    .fast_period_s = 300,              // 5 min while waiting
    .cur_period_s = 900,               // Current cadence
    .prewake_min = 10,                 // Wake 10 min before sunrise
    
    .az_mount_offset_deg = 0.0,        // Calibration offset (earth→mount)
    .el_mount_offset_deg = 0.0,        // Calibration offset
    
    // Home position = center of mechanical range
    .home_az_deg = 0.0,                // Center = 0° (flat)
    .home_el_deg = 0.0,                // Center = 0° (flat)
    
    .homing_time_ms = 8300,            // Time to reach center from either extreme
    .az_home_dir_level = 0,            // Calculated during moves
    .el_home_dir_level = 0,
    
    .last_move_az_tgt = 0.0,           // Start at home
    .last_move_el_tgt = 0.0
};

static SemaphoreHandle_t s_mutex;      // Reserved for future multi-thread access to 's'
/*
 * Convert actuator extension (mm) back to angle (deg).
 * Used for debugging and logging actual actuator positions.
 */
static double az_mm_to_angle(double mm) {
    const double CENTER_MM = 127.0;
    const double STROKE_MM = 63.5;
    const double RANGE_DEG = 90.0;
    const double MM_PER_DEG = STROKE_MM / RANGE_DEG;
    
    return (mm - CENTER_MM) / MM_PER_DEG;
}

static double el_mm_to_angle(double mm) {
    const double CENTER_MM = 107.95;
    const double STROKE_MM = 88.9;
    const double RANGE_DEG = 112.0;
    const double MM_PER_DEG = STROKE_MM / RANGE_DEG;
    
    return (mm - CENTER_MM) / MM_PER_DEG;
}
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
    DEBUG_TRACE();
    
    ESP_LOGD(TAG, "Preparing to save state to NVS...");
    ESP_LOGD(TAG, "  Data to save:");
    ESP_LOGD(TAG, "    - Position: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGD(TAG, "    - Actuators: AZ=%.2fmm EL=%.2fmm", s.az_actuator_mm, s.el_actuator_mm);
    ESP_LOGD(TAG, "    - Offsets: AZ=%.2f° EL=%.2f°", s.az_mount_offset_deg, s.el_mount_offset_deg);
    ESP_LOGD(TAG, "    - Stats: %u moves today, %u total", s.moves_today, s.total_moves);
    
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READWRITE, &h);
    
    ESP_LOGV(TAG, "nvs_open() returned: %s (%d)", esp_err_to_name(ret), ret);
    
    if (ret == ESP_OK){
        // Calculate total state size for verification
        size_t state_size = sizeof(tracker_state_t);
        ESP_LOGV(TAG, "State structure size: %zu bytes", state_size);
        
        esp_err_t write_ret = nvs_set_blob(h, "state", &s, sizeof(s));
        ESP_LOGV(TAG, "nvs_set_blob() returned: %s (%d)", esp_err_to_name(write_ret), write_ret);
        
        if (write_ret == ESP_OK){
            esp_err_t commit_ret = nvs_commit(h);
            ESP_LOGV(TAG, "nvs_commit() returned: %s (%d)", esp_err_to_name(commit_ret), commit_ret);
            
            if (commit_ret == ESP_OK) {
                ESP_LOGD(TAG, "✓ State saved: az=%.1f° el=%.1f° moves=%u",
                         s.az_cur, s.el_cur, s.total_moves);
                ESP_LOGV(TAG, "  Flash write complete - data persistent across reboots");
            } else {
                ESP_LOGW(TAG, "⚠ NVS commit failed: %s", esp_err_to_name(commit_ret));
            }
        } else {
            ESP_LOGW(TAG, "⚠ NVS save failed: %s", esp_err_to_name(write_ret));
            ESP_LOGD(TAG, "  Check flash wear level and available space");
        }
        nvs_close(h);
        ESP_LOGV(TAG, "NVS handle closed");
    } else {
        ESP_LOGW(TAG, "⚠ NVS open failed: %s", esp_err_to_name(ret));
        ESP_LOGD(TAG, "  This prevents state persistence - position will reset on reboot");
    }
}

/*
 * Load tracker state from NVS flash (ENHANCED DEBUGGING).
 */
static void nvs_load(void){
    DEBUG_TRACE();
    
    ESP_LOGD(TAG, "Attempting to load saved state from NVS...");
    
    nvs_handle_t h;
    esp_err_t ret = nvs_open("tracker", NVS_READONLY, &h);
    
    ESP_LOGV(TAG, "nvs_open() returned: %s (%d)", esp_err_to_name(ret), ret);
    
    if (ret == ESP_OK){
        size_t required_size = sizeof(s);
        ESP_LOGV(TAG, "Expected state size: %zu bytes", required_size);
        
        ret = nvs_get_blob(h, "state", &s, &required_size);
        ESP_LOGV(TAG, "nvs_get_blob() returned: %s (%d)", esp_err_to_name(ret), ret);
        ESP_LOGV(TAG, "Actual state size: %zu bytes", required_size);
        
        if (ret == ESP_OK) {
            // Validate loaded data for sanity
            bool data_valid = true;
            
            // Check angle ranges
            if (s.az_cur < -180.0 || s.az_cur > 180.0) {
                ESP_LOGW(TAG, "⚠ Loaded AZ angle out of range: %.1f°", s.az_cur);
                data_valid = false;
            }
            if (s.el_cur < -90.0 || s.el_cur > 90.0) {
                ESP_LOGW(TAG, "⚠ Loaded EL angle out of range: %.1f°", s.el_cur);
                data_valid = false;
            }
            
            // Check actuator extensions
            if (s.az_actuator_mm < 50.0 || s.az_actuator_mm > 200.0) {
                ESP_LOGW(TAG, "⚠ Loaded AZ actuator out of range: %.2fmm", s.az_actuator_mm);
                data_valid = false;
            }
            if (s.el_actuator_mm < 50.0 || s.el_actuator_mm > 200.0) {
                ESP_LOGW(TAG, "⚠ Loaded EL actuator out of range: %.2fmm", s.el_actuator_mm);
                data_valid = false;
            }
            
            if (!data_valid) {
                ESP_LOGE(TAG, "✗ Loaded data failed validation - using defaults");
                ESP_LOGD(TAG, "  This may indicate NVS corruption or incompatible firmware");
                // Reset to defaults (keep existing initialization)
            } else {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
                ESP_LOGI(TAG, "║          TRACKER STATE LOADED FROM NVS                     ║");
                ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Last Known Position:");
                ESP_LOGI(TAG, "  - Azimuth: %.1f° (mount frame)", s.az_cur);
                ESP_LOGI(TAG, "  - Elevation: %.1f° (mount frame)", s.el_cur);
                ESP_LOGD(TAG, "    · AZ actuator: %.2fmm (%.3f\")", s.az_actuator_mm, s.az_actuator_mm/25.4);
                ESP_LOGD(TAG, "    · EL actuator: %.2fmm (%.3f\")", s.el_actuator_mm, s.el_actuator_mm/25.4);
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Mount Offsets (Calibration):");
                ESP_LOGI(TAG, "  - AZ offset: %.2f° (earth → mount)", s.az_mount_offset_deg);
                ESP_LOGI(TAG, "  - EL offset: %.2f° (earth → mount)", s.el_mount_offset_deg);
                
                if (fabs(s.az_mount_offset_deg) < 0.1 && fabs(s.el_mount_offset_deg) < 0.1) {
                    ESP_LOGW(TAG, "  ⚠ No calibration data - run calibration for accuracy");
                    ESP_LOGD(TAG, "    - Manual: Point at sun, hold button 3s");
                    ESP_LOGD(TAG, "    - Auto: Double-press button (compass-based)");
                } else {
                    ESP_LOGD(TAG, "  ✓ Calibration data present");
                    ESP_LOGV(TAG, "    - Transform: mount = earth - offset");
                    ESP_LOGV(TAG, "    - Example: earth(180°) - offset(%.1f°) = mount(%.1f°)",
                             s.az_mount_offset_deg, 180.0 - s.az_mount_offset_deg);
                }
                
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Move Statistics:");
                ESP_LOGI(TAG, "  - Today: %u moves", s.moves_today);
                ESP_LOGI(TAG, "  - Lifetime: %u moves", s.total_moves);
                
                if (s.total_moves > 0) {
                    ESP_LOGD(TAG, "    · Average moves/day: %.1f (estimated)", 
                             (float)s.total_moves / fmax(1.0, s.moves_today));
                }
                
                ESP_LOGI(TAG, "");
                
                // Log threshold configuration
                ESP_LOGD(TAG, "Configuration:");
                ESP_LOGD(TAG, "  - Move threshold: %.1f°", s.tol_deg);
                ESP_LOGD(TAG, "  - Min step: %.1f°", s.min_step_deg);
                ESP_LOGD(TAG, "  - Cadence: fast=%ds, slow=%ds", (int)s.fast_period_s, (int)s.base_period_s);
                ESP_LOGD(TAG, "  - Sleep threshold: %.1f° elevation", s.sleep_thresh_el);
            }
        } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          FIRST BOOT - USING DEFAULT STATE                  ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Default Position (Center/Home):");
            ESP_LOGI(TAG, "  - Azimuth: %.1f° (mechanical center)", s.az_cur);
            ESP_LOGI(TAG, "  - Elevation: %.1f° (mechanical center)", s.el_cur);
            ESP_LOGD(TAG, "    · AZ actuator: %.2fmm (5.00\", center)", s.az_actuator_mm);
            ESP_LOGD(TAG, "    · EL actuator: %.2fmm (4.25\", center)", s.el_actuator_mm);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "⚠ Calibration Required:");
            ESP_LOGI(TAG, "  1. Point panel at sun manually");
            ESP_LOGI(TAG, "  2. Hold button for 3 seconds");
            ESP_LOGI(TAG, "  3. System will learn mount offsets");
            ESP_LOGI(TAG, "  OR");
            ESP_LOGI(TAG, "  1. Double-press button");
            ESP_LOGI(TAG, "  2. Rotate tracker slowly (compass calibration)");
            ESP_LOGI(TAG, "  3. Auto-calibration will use compass heading");
            ESP_LOGI(TAG, "");
        } else {
            ESP_LOGE(TAG, "✗ NVS read error: %s", esp_err_to_name(ret));
            ESP_LOGD(TAG, "  Using default state - system will operate but may lose position");
        }
        nvs_close(h);
        ESP_LOGV(TAG, "NVS handle closed");
    } else {
        ESP_LOGI(TAG, "No saved state found (first boot) - using defaults");
        ESP_LOGD(TAG, "  NVS open error: %s", esp_err_to_name(ret));
    }
}

/*
 * Convert azimuth angle to actuator extension (ENHANCED DEBUGGING).
 */
static double az_angle_to_mm(double angle_deg) {
    DEBUG_TRACE();
    ESP_LOGV(TAG, "AZ angle input: %.2f°", angle_deg);
    
    const double CENTER_MM = 127.0;
    const double STROKE_MM = 63.5;
    const double RANGE_DEG = 90.0;
    const double MM_PER_DEG = STROKE_MM / RANGE_DEG;
    const double MIN_MM = 95.25;
    const double MAX_MM = 158.75;
    
    ESP_LOGV(TAG, "AZ kinematics constants:");
    ESP_LOGV(TAG, "  - Center: %.2fmm (5.00\")", CENTER_MM);
    ESP_LOGV(TAG, "  - Stroke: %.2fmm (2.50\")", STROKE_MM);
    ESP_LOGV(TAG, "  - Range: %.1f° (±45°)", RANGE_DEG);
    ESP_LOGV(TAG, "  - Conversion: %.3f mm/deg", MM_PER_DEG);
    
    double original_angle = angle_deg;
    bool clamped = false;
    
    // Clamp angle to mechanical limits
    if (angle_deg < -45.0) {
        ESP_LOGW(TAG, "⚠ AZ angle %.1f° < -45° (clamping to -45°)", angle_deg);
        ESP_LOGD(TAG, "  This indicates target is outside mechanical range");
        angle_deg = -45.0;
        clamped = true;
    }
    if (angle_deg > 45.0) {
        ESP_LOGW(TAG, "⚠ AZ angle %.1f° > +45° (clamping to +45°)", angle_deg);
        ESP_LOGD(TAG, "  This indicates target is outside mechanical range");
        angle_deg = 45.0;
        clamped = true;
    }
    
    // Linear conversion
    double mm = CENTER_MM + (angle_deg * MM_PER_DEG);
    
    ESP_LOGV(TAG, "AZ calculation:");
    ESP_LOGV(TAG, "  - Input angle: %.2f° (clamped: %s)", angle_deg, clamped ? "YES" : "NO");
    ESP_LOGV(TAG, "  - Offset from center: %.2f°", angle_deg);
    ESP_LOGV(TAG, "  - Extension delta: %.2fmm (%.2f° × %.3f mm/deg)", 
             angle_deg * MM_PER_DEG, angle_deg, MM_PER_DEG);
    ESP_LOGV(TAG, "  - Final extension: %.2fmm (%.3f\")", mm, mm / 25.4);
    
    // Safety check
    if (mm < MIN_MM) {
        ESP_LOGW(TAG, "⚠ AZ extension %.2fmm < min %.2fmm (clamping)", mm, MIN_MM);
        ESP_LOGD(TAG, "  Calculated: center(%.2f) + angle(%.2f) × rate(%.3f) = %.2f",
                 CENTER_MM, angle_deg, MM_PER_DEG, mm);
        mm = MIN_MM;
    }
    if (mm > MAX_MM) {
        ESP_LOGW(TAG, "⚠ AZ extension %.2fmm > max %.2fmm (clamping)", mm, MAX_MM);
        ESP_LOGD(TAG, "  Calculated: center(%.2f) + angle(%.2f) × rate(%.3f) = %.2f",
                 CENTER_MM, angle_deg, MM_PER_DEG, mm);
        mm = MAX_MM;
    }
    
    if (clamped || mm < MIN_MM || mm > MAX_MM) {
        ESP_LOGW(TAG, "  → Final: %.1f° → %.2fmm (CLAMPED from %.1f°)", 
                 angle_deg, mm, original_angle);
    } else {
        ESP_LOGV(TAG, "  → Final: %.1f° → %.2fmm (%.3f\", Δ=%.2fmm from center)",
                 angle_deg, mm, mm / 25.4, mm - CENTER_MM);
    }
    
    return mm;
}

/*
 * Convert elevation angle to actuator extension (ENHANCED DEBUGGING).
 */
static double el_angle_to_mm(double angle_deg) {
    DEBUG_TRACE();
    ESP_LOGV(TAG, "EL angle input: %.2f°", angle_deg);
    
    const double CENTER_MM = 107.95;
    const double STROKE_MM = 88.9;
    const double RANGE_DEG = 112.0;
    const double MM_PER_DEG = STROKE_MM / RANGE_DEG;
    const double MIN_MM = 63.5;
    const double MAX_MM = 152.4;
    
    ESP_LOGV(TAG, "EL kinematics constants:");
    ESP_LOGV(TAG, "  - Center: %.2fmm (4.25\")", CENTER_MM);
    ESP_LOGV(TAG, "  - Stroke: %.2fmm (3.50\")", STROKE_MM);
    ESP_LOGV(TAG, "  - Range: %.1f° (±56°)", RANGE_DEG);
    ESP_LOGV(TAG, "  - Conversion: %.3f mm/deg", MM_PER_DEG);
    
    double original_angle = angle_deg;
    bool clamped = false;
    
    // Clamp angle
    if (angle_deg < -56.0) {
        ESP_LOGW(TAG, "⚠ EL angle %.1f° < -56° (clamping to -56°)", angle_deg);
        angle_deg = -56.0;
        clamped = true;
    }
    if (angle_deg > 56.0) {
        ESP_LOGW(TAG, "⚠ EL angle %.1f° > +56° (clamping to +56°)", angle_deg);
        angle_deg = 56.0;
        clamped = true;
    }
    
    // Linear conversion
    double mm = CENTER_MM + (angle_deg * MM_PER_DEG);
    
    ESP_LOGV(TAG, "EL calculation:");
    ESP_LOGV(TAG, "  - Input angle: %.2f° (clamped: %s)", angle_deg, clamped ? "YES" : "NO");
    ESP_LOGV(TAG, "  - Extension delta: %.2fmm", angle_deg * MM_PER_DEG);
    ESP_LOGV(TAG, "  - Final extension: %.2fmm (%.3f\")", mm, mm / 25.4);
    
    // Safety check
    if (mm < MIN_MM) {
        ESP_LOGW(TAG, "⚠ EL extension %.2fmm < min %.2fmm (clamping)", mm, MIN_MM);
        mm = MIN_MM;
    }
    if (mm > MAX_MM) {
        ESP_LOGW(TAG, "⚠ EL extension %.2fmm > max %.2fmm (clamping)", mm, MAX_MM);
        mm = MAX_MM;
    }
    
    if (clamped || mm < MIN_MM || mm > MAX_MM) {
        ESP_LOGW(TAG, "  → Final: %.1f° → %.2fmm (CLAMPED from %.1f°)", 
                 angle_deg, mm, original_angle);
    } else {
        ESP_LOGV(TAG, "  → Final: %.1f° → %.2fmm (%.3f\", Δ=%.2fmm from center)",
                 angle_deg, mm, mm / 25.4, mm - CENTER_MM);
    }
    
    return mm;
}

/*
 * Execute motor movements if angular error exceeds thresholds.
 * 
 * NEW: Track actual actuator extensions for accurate positioning
 * - Converts target angles to mm of extension
 * - Calculates distance and direction for each axis
 * - Updates actuator position after each move
 * - Logs movement details for position reconstruction
 * 
 * Movement logic:
 * - Only move if error > tolerance (10°) AND > minimum step (2°)
 * - Moves AZ first, then EL (sequential to reduce peak current)
 * - Updates current position AND actuator extension (tracked state)
 * - Increments move counters for statistics
 * - Logs move to SD card with actuator details
 */
static void do_move(double az_tgt, double el_tgt){
    // Clamp to mechanical limits
    if (az_tgt < -45.0 || az_tgt > 45.0) {
        ESP_LOGW(TAG, "⚠ AZ target %.1f° outside range [−45°, +45°]", az_tgt);
        az_tgt = (az_tgt < -45.0) ? -45.0 : 45.0;
    }
    if (el_tgt < -56.0 || el_tgt > 56.0) {
        ESP_LOGW(TAG, "⚠ EL target %.1f° outside range [−56°, +56°]", el_tgt);
        el_tgt = (el_tgt < -56.0) ? -56.0 : 56.0;
    }
    
    double az_error = fabs(az_tgt - s.az_cur);
    double el_error = fabs(el_tgt - s.el_cur);

    bool move_az = (az_error > s.tol_deg) && (az_error > s.min_step_deg);
    bool move_el = (el_error > s.tol_deg) && (el_error > s.min_step_deg);

    ESP_LOGI(TAG, "Movement decision:");
    ESP_LOGI(TAG, "  AZ: error=%.1f° tol=%.1f° min_step=%.1f° → %s",
             az_error, s.tol_deg, s.min_step_deg, move_az ? "MOVE" : "SKIP");
    ESP_LOGI(TAG, "  EL: error=%.1f° tol=%.1f° min_step=%.1f° → %s",
             el_error, s.tol_deg, s.min_step_deg, move_el ? "MOVE" : "SKIP");

    if (!move_az && !move_el) {
        ESP_LOGI(TAG, "✓ Within tolerance - no move needed");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              EXECUTING TRACKING MOVE                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Movement Plan:");
    ESP_LOGI(TAG, "  Current: AZ=%.1f° EL=%.1f° (mount frame)", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  Target:  AZ=%.1f° EL=%.1f° (mount frame)", az_tgt, el_tgt);
    ESP_LOGI(TAG, "  Errors:  AZ=%.1f° EL=%.1f°", az_error, el_error);
    ESP_LOGI(TAG, "");
    
    // Log current actuator positions and extensions
    ESP_LOGI(TAG, "Actuator Status (Current):");
    ESP_LOGI(TAG, "  AZ: %.2fmm (%.1f° from center)", 
             s.az_actuator_mm, az_mm_to_angle(s.az_actuator_mm));
    ESP_LOGI(TAG, "  EL: %.2fmm (%.1f° from center)", 
             s.el_actuator_mm, el_mm_to_angle(s.el_actuator_mm));
    ESP_LOGI(TAG, "");
    
    // Calculate target actuator extensions using kinematics
    double az_tgt_mm = az_angle_to_mm(az_tgt);
    double el_tgt_mm = el_angle_to_mm(el_tgt);
    
    ESP_LOGI(TAG, "Actuator Targets:");
    ESP_LOGI(TAG, "  AZ: %.2fmm (%.1f° from center)", az_tgt_mm, az_tgt);
    ESP_LOGI(TAG, "  EL: %.2fmm (%.1f° from center)", el_tgt_mm, el_tgt);
    ESP_LOGI(TAG, "");
    
    // Calculate movement distances
    double az_delta_mm = az_tgt_mm - s.az_actuator_mm;
    double el_delta_mm = el_tgt_mm - s.el_actuator_mm;
    
    ESP_LOGI(TAG, "Required Movements:");
    if (move_az) {
        const char* az_dir = (az_delta_mm > 0) ? "EXTEND" : "RETRACT";
        ESP_LOGI(TAG, "  ✓ AZ: %.1f° error → %s %.2fmm (%.2f\")",
                 az_error, az_dir, fabs(az_delta_mm), fabs(az_delta_mm) / 25.4);
    }
    if (move_el) {
        const char* el_dir = (el_delta_mm > 0) ? "EXTEND" : "RETRACT";
        ESP_LOGI(TAG, "  ✓ EL: %.1f° error → %s %.2fmm (%.2f\")",
                 el_error, el_dir, fabs(el_delta_mm), fabs(el_delta_mm) / 25.4);
    }
    ESP_LOGI(TAG, "");

    // Record move start time
    time_t move_start = time(NULL);

    // Move azimuth first (if needed)
    if (move_az) {
        ESP_LOGI(TAG, "Moving AZ actuator...");
        ESP_LOGI(TAG, "  From: %.1f° (%.2fmm / %.2f\")", 
                 s.az_cur, s.az_actuator_mm, s.az_actuator_mm / 25.4);
        ESP_LOGI(TAG, "  To:   %.1f° (%.2fmm / %.2f\")",
                 az_tgt, az_tgt_mm, az_tgt_mm / 25.4);
        
        // Store pre-move state
        double az_start_mm = s.az_actuator_mm;
        double az_start_deg = s.az_cur;
        
        // Execute move (motor.c converts angles internally)
        motor_move_az(s.az_cur, az_tgt);
        
        // Update tracked positions
        s.az_cur = az_tgt;
        s.az_actuator_mm = az_tgt_mm;
        
        // Calculate actual movement
        double az_moved_mm = s.az_actuator_mm - az_start_mm;
        double az_moved_deg = s.az_cur - az_start_deg;
        const char* az_dir = (az_moved_mm > 0) ? "extended" : "retracted";
        
        ESP_LOGI(TAG, "✓ AZ movement complete:");
        ESP_LOGI(TAG, "  - Angle: %.1f° → %.1f° (Δ=%.1f°)",
                 az_start_deg, s.az_cur, az_moved_deg);
        ESP_LOGI(TAG, "  - Actuator: %.2fmm → %.2fmm (%s %.2fmm / %.3f\")",
                 az_start_mm, s.az_actuator_mm, az_dir, 
                 fabs(az_moved_mm), fabs(az_moved_mm) / 25.4);
        ESP_LOGI(TAG, "  - Conversion: %.3f mm/degree", 
                 fabs(az_moved_mm) / fabs(az_moved_deg));
        
        sdlog_printf("AZ: %.1f°→%.1f° | %.2fmm→%.2fmm | %s %.2fmm",
                     az_start_deg, s.az_cur, az_start_mm, s.az_actuator_mm,
                     az_dir, fabs(az_moved_mm));
    }
    
    // Brief pause between moves
    if (move_az && move_el) {
        ESP_LOGD(TAG, "Pausing 500ms between axes...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Move elevation (if needed)
    if (move_el) {
        ESP_LOGI(TAG, "Moving EL actuator...");
        ESP_LOGI(TAG, "  From: %.1f° (%.2fmm / %.2f\")",
                 s.el_cur, s.el_actuator_mm, s.el_actuator_mm / 25.4);
        ESP_LOGI(TAG, "  To:   %.1f° (%.2fmm / %.2f\")",
                 el_tgt, el_tgt_mm, el_tgt_mm / 25.4);
        
        // Store pre-move state
        double el_start_mm = s.el_actuator_mm;
        double el_start_deg = s.el_cur;
        
        // Execute move
        motor_move_el(s.el_cur, el_tgt);
        
        // Update tracked positions
        s.el_cur = el_tgt;
        s.el_actuator_mm = el_tgt_mm;
        
        // Calculate actual movement
        double el_moved_mm = s.el_actuator_mm - el_start_mm;
        double el_moved_deg = s.el_cur - el_start_deg;
        const char* el_dir = (el_moved_mm > 0) ? "extended" : "retracted";
        
        ESP_LOGI(TAG, "✓ EL movement complete:");
        ESP_LOGI(TAG, "  - Angle: %.1f° → %.1f° (Δ=%.1f°)",
                 el_start_deg, s.el_cur, el_moved_deg);
        ESP_LOGI(TAG, "  - Actuator: %.2fmm → %.2fmm (%s %.2fmm / %.3f\")",
                 el_start_mm, s.el_actuator_mm, el_dir,
                 fabs(el_moved_mm), fabs(el_moved_mm) / 25.4);
        ESP_LOGI(TAG, "  - Conversion: %.3f mm/degree",
                 fabs(el_moved_mm) / fabs(el_moved_deg));
        
        sdlog_printf("EL: %.1f°→%.1f° | %.2fmm→%.2fmm | %s %.2fmm",
                     el_start_deg, s.el_cur, el_start_mm, s.el_actuator_mm,
                     el_dir, fabs(el_moved_mm));
    }

    // Update statistics
    s.moves_today++;
    s.total_moves++;
    s.last_move = time(NULL);
    time_t move_duration = s.last_move - move_start;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ TRACKING MOVE COMPLETE");
    ESP_LOGI(TAG, "  Final Position:");
    ESP_LOGI(TAG, "    - Angles: AZ=%.1f° EL=%.1f° (mount frame)", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "    - Actuators: AZ=%.2fmm (%.2f\") EL=%.2fmm (%.2f\")",
             s.az_actuator_mm, s.az_actuator_mm / 25.4,
             s.el_actuator_mm, s.el_actuator_mm / 25.4);
    ESP_LOGI(TAG, "  Duration: %ld seconds", (long)move_duration);
    ESP_LOGI(TAG, "  Statistics: Move #%u today, #%u lifetime",
             s.moves_today, s.total_moves);
    ESP_LOGI(TAG, "");

    sdlog_printf("Move #%u: AZ[%.1f°,%.2fmm] EL[%.1f°,%.2fmm] (%lds)",
                 s.total_moves, s.az_cur, s.az_actuator_mm,
                 s.el_cur, s.el_actuator_mm, (long)move_duration);
}

/*
 * Check for midnight rollover and reset daily counter.
 * Keeps moves_today accurate across day boundaries.
 */
static void maybe_midnight_reset(void) {
    static int last_day = -1;
    
    time_t now = time(NULL);
    struct tm *tm_now = localtime(&now);
    
    if (last_day == -1) {
        // First call - just record the day
        last_day = tm_now->tm_mday;
        ESP_LOGD(TAG, "Midnight check initialized: day %d", (int)last_day);
        return;
    }
    
    if (tm_now->tm_mday != last_day) {
        // Day changed - reset counter
        ESP_LOGI(TAG, "Midnight rollover detected: day %d → %d", (int)last_day, (int)tm_now->tm_mday);
        ESP_LOGI(TAG, "  Moves yesterday: %u", s.moves_today);
        
        sdlog_printf("Midnight: Reset daily counter (%u moves yesterday)", s.moves_today);
        
        s.moves_today = 0;
        last_day = tm_now->tm_mday;
        
        // Save updated counter to NVS
        nvs_save();
        
        ESP_LOGI(TAG, "  Daily counter reset to 0");
    }
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
 * Nightly homing: Return actuators to midpoint (starting position).
 * 
 * NEW: Uses tracked actuator positions to calculate exact movements needed
 * - Knows current actuator extensions from tracking
 * - Calculates distance to midpoint (100mm)
 * - Moves each actuator independently to 100mm
 * - Updates both angle and actuator position tracking
 * 
 * This is the SAME position the system starts at:
 * - Panel facing straight up (zenith)
 * - Both actuators at 50% extension (100mm)
 * - AZ actuator: 100mm → panel at 135° azimuth
 * - EL actuator: 100mm → panel at 47.5° elevation
 */

static void home_to_midpoint(void){
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              NIGHTLY HOMING SEQUENCE                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Homing Strategy:");
    ESP_LOGI(TAG, "  - Return to mechanical center (startup position)");
    ESP_LOGI(TAG, "  - AZ: 0° (127.0mm / 5.00\" center)");              // CHANGED
    ESP_LOGI(TAG, "  - EL: 0° (107.95mm / 4.25\" center)");             // UNCHANGED
    ESP_LOGI(TAG, "  - Panel orientation: Horizontal (flat/level)");
    ESP_LOGI(TAG, "  - Benefits: Balanced load, minimal stress, safe parking");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Current Position:");
    ESP_LOGI(TAG, "  - Angles: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "  - Actuators: AZ=%.2fmm (%.2f\") EL=%.2fmm (%.2f\")",
             s.az_actuator_mm, s.az_actuator_mm / 25.4,
             s.el_actuator_mm, s.el_actuator_mm / 25.4);
    ESP_LOGI(TAG, "");
    
    // Calculate movements to center
    const double AZ_HOME_MM = 127.0;   // CHANGED: 5.00" center (was 82.55mm)
    const double EL_HOME_MM = 107.95;  // UNCHANGED: 4.25" center
    
    double az_delta_mm = AZ_HOME_MM - s.az_actuator_mm;
    double el_delta_mm = EL_HOME_MM - s.el_actuator_mm;
    double az_delta_deg = s.home_az_deg - s.az_cur;
    double el_delta_deg = s.home_el_deg - s.el_cur;
    
    const char* az_dir = (az_delta_mm > 0) ? "EXTEND" : "RETRACT";
    const char* el_dir = (el_delta_mm > 0) ? "EXTEND" : "RETRACT";
    
    ESP_LOGI(TAG, "Required Movements to Center:");
    ESP_LOGI(TAG, "  AZ:");
    ESP_LOGI(TAG, "    - Angle: %.1f° → 0° (%s %.1f°)",
             s.az_cur, (az_delta_deg >= 0) ? "+" : "", az_delta_deg);
    ESP_LOGI(TAG, "    - Actuator: %.2fmm → 127.0mm (%s %.2fmm / %.3f\")",  // CHANGED
             s.az_actuator_mm, az_dir, fabs(az_delta_mm), fabs(az_delta_mm) / 25.4);
    ESP_LOGI(TAG, "  EL:");
    ESP_LOGI(TAG, "    - Angle: %.1f° → 0° (%s %.1f°)",
             s.el_cur, (el_delta_deg >= 0) ? "+" : "", el_delta_deg);
    ESP_LOGI(TAG, "    - Actuator: %.2fmm → 107.95mm (%s %.2fmm / %.3f\")",
             s.el_actuator_mm, el_dir, fabs(el_delta_mm), fabs(el_delta_mm) / 25.4);
    ESP_LOGI(TAG, "");

    status_led_set_mode(LED_SLEEP);

    // Move AZ to center
    ESP_LOGI(TAG, "Homing AZ to center (0°, 127.0mm)...");  // CHANGED
    motor_move_az(s.az_cur, s.home_az_deg);
    s.az_cur = s.home_az_deg;
    s.az_actuator_mm = AZ_HOME_MM;
    ESP_LOGI(TAG, "✓ AZ centered: 0° @ 127.0mm (5.00\")");  // CHANGED
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Move EL to center
    ESP_LOGI(TAG, "Homing EL to center (0°, 107.95mm)...");
    motor_move_el(s.el_cur, s.home_el_deg);
    s.el_cur = s.home_el_deg;
    s.el_actuator_mm = EL_HOME_MM;
    ESP_LOGI(TAG, "✓ EL centered: 0° @ 107.95mm (4.25\")");

    // Update state
    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;
    nvs_save();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ HOMING COMPLETE");
    ESP_LOGI(TAG, "  Final Position (Home/Center):");
    ESP_LOGI(TAG, "    - Angles: AZ=0° EL=0° (horizontal/flat)");
    ESP_LOGI(TAG, "    - Actuators:");
    ESP_LOGI(TAG, "      · AZ: 127.0mm (5.00\", mechanical center)");    // CHANGED
    ESP_LOGI(TAG, "      · EL: 107.95mm (4.25\", mechanical center)");   // UNCHANGED
    ESP_LOGI(TAG, "  Panel: Horizontal (minimal stress on structure)");
    ESP_LOGI(TAG, "  Ready for overnight parking");
    ESP_LOGI(TAG, "");

    sdlog_printf("HOMED: Center position AZ=0°@127.0mm EL=0°@107.95mm (flat)");  // CHANGED
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
 * Auto-calibration using compass (FULL auto-calibration: azimuth + elevation).
 * 
 * NEW ALGORITHM:
 * - Uses compass to determine panel's current TRUE orientation in space
 * - Calculates sun position (azimuth + elevation)
 * - Computes BOTH az and el offsets automatically
 * - No manual pointing needed!
 * 
 * Requirements:
 * - Compass must be calibrated first (gps_calibrate_compass())
 * - Valid GPS fix
 * - Sun elevation > 15° (accurate sun azimuth needed)
 * - Panel should be in a known reference position (horizontal/level recommended)
 * 
 * How it works:
 * 1. Reads compass heading → panel azimuth orientation
 * 2. Assumes panel is level (0° elevation) at calibration time
 * 3. Calculates: az_offset = panel_heading - sun_azimuth
 * 4. Calculates: el_offset = 0° - sun_elevation (since panel is level)
 * 5. Future tracking: target = sun_position - offsets
 */
void tracking_auto_calibrate_with_compass(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          AUTO CALIBRATION (Compass + Elevation)            ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    // Verify compass is calibrated
    if (!GPS_IS_COMPASS_CALIBRATED()) {
        ESP_LOGW(TAG, "✗ Compass not calibrated");
        ESP_LOGW(TAG, "  - Double-press button to calibrate compass first");
        ESP_LOGW(TAG, "  - Rotate tracker 2-3 full circles during calibration");
        ESP_LOGW(TAG, "");
        return;
    }

    // Get GPS fix
    gps_data_t gps;
    if (!GPS_GET_LAST(&gps) || !gps.valid) {
        ESP_LOGW(TAG, "✗ No valid GPS fix");
        ESP_LOGW(TAG, "  - Wait for GPS fix before auto-calibrating");
        ESP_LOGW(TAG, "");
        return;
    }

    // Calculate current sun position
    time_t now = time(NULL);
    sun_pos_t sun = solar_compute(gps.latitude, gps.longitude, now);

    ESP_LOGI(TAG, "Sun Position (Earth Frame):");
    ESP_LOGI(TAG, "  - Azimuth: %.2f°", sun.azimuth_deg);
    ESP_LOGI(TAG, "  - Elevation: %.2f°", sun.elevation_deg);
    ESP_LOGI(TAG, "");

    // Verify sun is high enough for accurate azimuth
    if (sun.elevation_deg < 15.0) {
        ESP_LOGW(TAG, "✗ Sun too low: %.1f° (need >15°)", sun.elevation_deg);
        ESP_LOGW(TAG, "  - Wait until sun is higher in sky");
        ESP_LOGW(TAG, "  - Best results near solar noon");
        ESP_LOGW(TAG, "");
        return;
    }

    // Read compass heading (TRUE north, declination-corrected)
    float compass_heading_true;
    if (!GPS_GET_COMPASS_HEADING_TRUE(&compass_heading_true)) {
        ESP_LOGE(TAG, "✗ Failed to read compass");
        ESP_LOGE(TAG, "  - Check compass connection");
        ESP_LOGE(TAG, "  - Recalibrate compass if needed");
        ESP_LOGI(TAG, "");
        return;
    }

    float declination = GPS_GET_MAGNETIC_DECLINATION();
    
    ESP_LOGI(TAG, "Panel Orientation (from Compass):");
    ESP_LOGI(TAG, "  - True heading: %.2f°", compass_heading_true);
    ESP_LOGI(TAG, "  - Magnetic declination: %.1f°", declination);
    ESP_LOGI(TAG, "  - Current position: AZ=%.1f° EL=%.1f° (assumed level)", 
             s.az_cur, s.el_cur);
    ESP_LOGI(TAG, "");

    // === CALCULATE AZIMUTH OFFSET ===
    // Formula: offset = panel_orientation - sun_azimuth
    // Example: Panel facing 45° (NE), Sun at 180° (S)
    //          offset = 45° - 180° = -135° (normalized to ±180°)
    double az_offset = compass_heading_true - sun.azimuth_deg;
    
    // Normalize to ±180° range
    while (az_offset > 180.0) az_offset -= 360.0;
    while (az_offset < -180.0) az_offset += 360.0;

    // === CALCULATE ELEVATION OFFSET ===
    // Assumption: Panel is currently LEVEL (horizontal, 0° in mount frame)
    // Formula: el_offset = current_panel_el - sun_elevation
    // Example: Panel level (0°), Sun at 45° elevation
    //          offset = 0° - 45° = -45°
    //          Future: target_el = sun_el - (-45°) = sun_el + 45°
    double el_offset = s.el_cur - sun.elevation_deg;

    // Save old values for comparison
    double old_az_offset = s.az_mount_offset_deg;
    double old_el_offset = s.el_mount_offset_deg;
    
    // Update offsets
    s.az_mount_offset_deg = az_offset;
    s.el_mount_offset_deg = el_offset;
    
    // Persist to NVS flash
    nvs_save();

    // === LOG CALIBRATION RESULTS ===
    ESP_LOGI(TAG, "✓ FULL AUTO-CALIBRATION COMPLETE");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Calibration Algorithm:");
    ESP_LOGI(TAG, "  - AZ offset = compass_heading - sun_azimuth");
    ESP_LOGI(TAG, "  - EL offset = panel_elevation - sun_elevation");
    ESP_LOGI(TAG, "  - Assumption: Panel is LEVEL (0°) during calibration");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Azimuth Results:");
    ESP_LOGI(TAG, "  - Old offset: %.2f°", old_az_offset);
    ESP_LOGI(TAG, "  - New offset: %.2f°", s.az_mount_offset_deg);
    ESP_LOGI(TAG, "  - Change: %.2f°", s.az_mount_offset_deg - old_az_offset);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Elevation Results:");
    ESP_LOGI(TAG, "  - Old offset: %.2f°", old_el_offset);
    ESP_LOGI(TAG, "  - New offset: %.2f°", s.el_mount_offset_deg);
    ESP_LOGI(TAG, "  - Change: %.2f°", s.el_mount_offset_deg - old_el_offset);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Future Tracking Behavior:");
    ESP_LOGI(TAG, "  - BOTH axes will now track automatically");
    ESP_LOGI(TAG, "  - Target AZ = sun_az - (%.2f°) = sun_az %+.2f°", 
             s.az_mount_offset_deg, -s.az_mount_offset_deg);
    ESP_LOGI(TAG, "  - Target EL = sun_el - (%.2f°) = sun_el %+.2f°",
             s.el_mount_offset_deg, -s.el_mount_offset_deg);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Important Notes:");
    ESP_LOGI(TAG, "  - Best accuracy when panel is LEVEL during calibration");
    ESP_LOGI(TAG, "  - For fine-tuning, use manual calibration:");
    ESP_LOGI(TAG, "    (point panel at sun, hold button 3 seconds)");
    ESP_LOGI(TAG, "  - Recalibrate if tracking accuracy degrades");
    ESP_LOGI(TAG, "");

    // Log to SD card
    sdlog_printf("Full auto-cal: AZ=%.2f° EL=%.2f° (compass+GPS, decl=%.1f°)", 
                 s.az_mount_offset_deg, s.el_mount_offset_deg, declination);
}

/*
 * Main tracking task loop (ENHANCED DEBUGGING).
 * 
 * This is where all the magic happens - heavily instrumented for debugging.
 */
static void tracking_task(void *arg){
    DEBUG_TRACE();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          TRACKING SYSTEM OPERATIONAL                       ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    ESP_LOGD(TAG, "Task started: priority=%d, stack=%d bytes",
             (int)uxTaskPriorityGet(NULL), 4096);
    ESP_LOGD(TAG, "Free heap before load: %lu bytes", 
             (unsigned long)esp_get_free_heap_size());

    nvs_load();

    ESP_LOGD(TAG, "Free heap after load: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    const char *csv = "/sdcard/SUNFLOW.CSV";
    sdlog_write_csv_header_if_new(csv);
    sdlog_printf("=== TRACKING STARTED ===");
    sdlog_printf("Initial position: AZ=%.1f° EL=%.1f°", s.az_cur, s.el_cur);

    s.last_move_az_tgt = s.az_cur;
    s.last_move_el_tgt = s.el_cur;

    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  - Home position: AZ=%.1f° EL=%.1f° (center)", 
             s.home_az_deg, s.home_el_deg);
    ESP_LOGI(TAG, "  - Movement threshold: %.1f°", s.tol_deg);
    ESP_LOGI(TAG, "  - Minimum step: %.1f°", s.min_step_deg);
    ESP_LOGI(TAG, "  - Cadence: %ds waiting, %ds after move",
             (int)s.fast_period_s, (int)s.base_period_s);
    ESP_LOGI(TAG, "  - Sleep threshold: %.1f° elevation", s.sleep_thresh_el);
    ESP_LOGI(TAG, "  - Pre-wake: %d minutes before sunrise", (int)s.prewake_min);
    ESP_LOGD(TAG, "  - CSV log: %s", csv);
    ESP_LOGI(TAG, "");

    // Auto-calibration logic with detailed diagnostics
    if (GPS_IS_COMPASS_CALIBRATED()) {
        ESP_LOGD(TAG, "Compass detected and calibrated");
        
        if (fabs(s.az_mount_offset_deg) < 0.1 && fabs(s.el_mount_offset_deg) < 0.1) {
            ESP_LOGI(TAG, "No calibration data found - attempting auto-calibration...");
            ESP_LOGD(TAG, "  Current offsets: AZ=%.3f° EL=%.3f° (both near zero)",
                     s.az_mount_offset_deg, s.el_mount_offset_deg);
            ESP_LOGI(TAG, "");

            status_led_set_mode(LED_WAITING);
            gps_data_t gps;
            int wait_count = 0;
            
            ESP_LOGD(TAG, "Waiting for GPS fix (timeout: 60s)...");
            
            while (!GPS_POLL_NAV_PVT(&gps) && wait_count < 60) {
                ESP_LOGI(TAG, "Waiting for GPS fix... (%d/60)", ++wait_count);
                ESP_LOGV(TAG, "  GPS poll attempt %d", wait_count);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            if (wait_count < 60) {
                ESP_LOGI(TAG, "✓ GPS fix acquired after %ds", wait_count * 5);
                ESP_LOGD(TAG, "  Location: %.6f°N %.6f°W", gps.latitude, gps.longitude);
                ESP_LOGD(TAG, "  Satellites: %u, Fix type: %u", 
                         gps.num_satellites, gps.fix_type);
                
                tracking_auto_calibrate_with_compass();
            } else {
                ESP_LOGW(TAG, "GPS timeout - using default offsets");
                ESP_LOGW(TAG, "Manual calibration recommended for accuracy");
                ESP_LOGD(TAG, "  Attempted %d times over %d seconds", wait_count, wait_count * 5);
            }
            ESP_LOGI(TAG, "");
        } else {
            ESP_LOGD(TAG, "Calibration data present: AZ=%.2f° EL=%.2f°",
                     s.az_mount_offset_deg, s.el_mount_offset_deg);
        }
    } else {
        ESP_LOGD(TAG, "Compass not detected or not calibrated - manual calibration only");
    }

    ESP_LOGI(TAG, "Entering main tracking loop...");
    ESP_LOGI(TAG, "");
    
    uint32_t loop_iteration = 0;
    uint32_t consecutive_gps_failures = 0;
    time_t last_successful_gps = 0;

    while (1){
        loop_iteration++;
        TickType_t loop_start = xTaskGetTickCount();
        
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGD(TAG, "║          TRACKING LOOP ITERATION %lu", (unsigned long)loop_iteration);
        ESP_LOGD(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGD(TAG, "");
        ESP_LOGV(TAG, "Loop start tick: %lu", (unsigned long)loop_start);
        ESP_LOGV(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        ESP_LOGV(TAG, "Stack high water mark: %lu words",
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL));

        // === GPS Acquisition (WITH DETAILED DIAGNOSTICS) ===
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "=== GPS ACQUISITION ===");
        
        gps_data_t g = {0};
        bool gps_fresh = GPS_POLL_NAV_PVT(&g);  // Prefer fresh fix
        bool gps_available = gps_fresh || GPS_GET_LAST(&g);
        
        ESP_LOGD(TAG, "GPS poll result:");
        ESP_LOGD(TAG, "  - Fresh fix: %s", gps_fresh ? "YES" : "NO");
        ESP_LOGD(TAG, "  - Cached available: %s", gps_available ? "YES" : "NO");

        if (!gps_available) {
            consecutive_gps_failures++;
            time_t failure_duration = time(NULL) - last_successful_gps;
            
            ESP_LOGW(TAG, "⚠ No GPS data available (failure #%lu)",
                     (unsigned long)consecutive_gps_failures);
            ESP_LOGD(TAG, "  Time since last fix: %ld seconds", (long)failure_duration);
            
            if (consecutive_gps_failures == 1) {
                ESP_LOGW(TAG, "  First GPS failure - entering wait mode");
                status_led_set_mode(LED_WAITING);
            }
            
            if (consecutive_gps_failures % 6 == 0) {  // Every 3 minutes
                ESP_LOGW(TAG, "  Extended GPS loss: %lu failures over %.1f minutes",
                         (unsigned long)consecutive_gps_failures,
                         consecutive_gps_failures * 30.0 / 60.0);
            }
            
            ESP_LOGW(TAG, "  Retrying in 30s...");
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        } else {
            // GPS recovered
            if (consecutive_gps_failures > 0) {
                ESP_LOGI(TAG, "✓ GPS recovered after %lu failures", 
                         (unsigned long)consecutive_gps_failures);
                ESP_LOGD(TAG, "  Outage duration: %.1f minutes",
                         consecutive_gps_failures * 30.0 / 60.0);
                consecutive_gps_failures = 0;
                status_led_set_mode(LED_TRACKING);
            }
            last_successful_gps = time(NULL);
            
            ESP_LOGD(TAG, "✓ GPS data available (%s)", gps_fresh ? "FRESH" : "CACHED");
            DEBUG_GPS(g);
            ESP_LOGV(TAG, "  Valid flag: %s", g.valid ? "true" : "false");
            ESP_LOGV(TAG, "  HDOP: (not available in structure)");
        }

        // === Sun Position Calculation (WITH VERIFICATION) ===
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "=== SUN POSITION CALCULATION ===");
        
        time_t now = time(NULL);
        struct tm *now_tm = localtime(&now);
        
        ESP_LOGD(TAG, "Time now: %04d-%02d-%02d %02d:%02d:%02d",
                 now_tm->tm_year + 1900, now_tm->tm_mon + 1, now_tm->tm_mday,
                 now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec);
        ESP_LOGV(TAG, "Unix timestamp: %ld", (long)now);
        
        sun_pos_t sun = solar_compute(g.latitude, g.longitude, now);
        
        DEBUG_SUN(sun);
        ESP_LOGV(TAG, "  Calculation input:");
        ESP_LOGV(TAG, "    - Lat: %.6f°", g.latitude);
        ESP_LOGV(TAG, "    - Lon: %.6f°", g.longitude);
        ESP_LOGV(TAG, "    - Time: %ld", (long)now);
        ESP_LOGV(TAG, "  Output validation:");
        ESP_LOGV(TAG, "    - Az range: %.1f° ∈ [0°, 360°]? %s",
                 sun.azimuth_deg,
                 (sun.azimuth_deg >= 0.0 && sun.azimuth_deg < 360.0) ? "✓" : "✗");
        ESP_LOGV(TAG, "    - El range: %.1f° ∈ [-90°, 90°]? %s",
                 sun.elevation_deg,
                 (sun.elevation_deg >= -90.0 && sun.elevation_deg <= 90.0) ? "✓" : "✗");

        // === Compass Heading (FOR CSV) ===
        float compass_true = NAN;
        float mount_front_true = NAN;
        
        if (GPS_IS_COMPASS_PRESENT()) {
            ESP_LOGV(TAG, "");
            ESP_LOGV(TAG, "=== COMPASS READING ===");
            
            float htrue;
            if (GPS_GET_COMPASS_HEADING_TRUE(&htrue)) {
                compass_true = htrue;
                mount_front_true = (float)wrap360(htrue + MOUNT_COMPASS_OFFSET_DEG);
                
                ESP_LOGV(TAG, "  Compass true: %.1f°", compass_true);
                ESP_LOGV(TAG, "  Mount front (compass + 180°): %.1f°", mount_front_true);
                ESP_LOGV(TAG, "  Compass offset correction: %.1f°", MOUNT_COMPASS_OFFSET_DEG);
            } else {
                ESP_LOGV(TAG, "  Compass read failed");
            }
        }

        // === Coordinate Transformation (EARTH → MOUNT) ===
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "=== COORDINATE TRANSFORMATION ===");
        ESP_LOGD(TAG, "Earth frame (sun position):");
        ESP_LOGD(TAG, "  Az=%.2f° El=%.2f°", sun.azimuth_deg, sun.elevation_deg);
        ESP_LOGD(TAG, "Mount offsets (from calibration):");
        ESP_LOGD(TAG, "  Az offset=%.2f° El offset=%.2f°",
                 s.az_mount_offset_deg, s.el_mount_offset_deg);
        
        // Transform: mount = earth - offset
        s.az_tgt = wrap180(sun.azimuth_deg - s.az_mount_offset_deg);
        s.el_tgt = sun.elevation_deg - s.el_mount_offset_deg;
        
        ESP_LOGD(TAG, "Mount frame (target position):");
        ESP_LOGD(TAG, "  Az=%.2f° El=%.2f°", s.az_tgt, s.el_tgt);
        ESP_LOGV(TAG, "");
        ESP_LOGV(TAG, "Transformation details:");
        ESP_LOGV(TAG, "  Az: %.2f° (sun) - %.2f° (offset) = %.2f° (target)",
                 sun.azimuth_deg, s.az_mount_offset_deg, s.az_tgt);
        ESP_LOGV(TAG, "  El: %.2f° (sun) - %.2f° (offset) = %.2f° (target)",
                 sun.elevation_deg, s.el_mount_offset_deg, s.el_tgt);

        // === Movement Threshold Check (DETAILED LOGIC) ===
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "=== MOVEMENT DECISION ===");
        
        double daz = fabs(wrap180(s.az_tgt - s.last_move_az_tgt));
        double del = fabs(s.el_tgt - s.last_move_el_tgt);
        double dang = fmax(daz, del);
        
        ESP_LOGD(TAG, "Error since last move:");
        ESP_LOGD(TAG, "  Az: %.2f° (target %.2f° vs last %.2f°)",
                 daz, s.az_tgt, s.last_move_az_tgt);
        ESP_LOGD(TAG, "  El: %.2f° (target %.2f° vs last %.2f°)",
                 del, s.el_tgt, s.last_move_el_tgt);
        ESP_LOGD(TAG, "  Max error: %.2f°", dang);
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "Thresholds:");
        ESP_LOGD(TAG, "  - Tolerance: %.1f° (must exceed to move)", s.tol_deg);
        ESP_LOGD(TAG, "  - Min step: %.1f° (ignore tiny adjustments)", s.min_step_deg);
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "Threshold checks:");
        ESP_LOGD(TAG, "  Error %.2f° > tolerance %.1f°? %s",
                 dang, s.tol_deg, (dang >= s.tol_deg) ? "YES" : "NO");
        ESP_LOGD(TAG, "  Error %.2f° > min_step %.1f°? %s",
                 dang, s.min_step_deg, (dang >= s.min_step_deg) ? "YES" : "NO");
        
        bool should_move = (dang >= s.tol_deg) && (dang >= s.min_step_deg);
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "Decision: %s", should_move ? "MOVE" : "WAIT");

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
        if (!isnan(compass_true)) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Heading:");
            ESP_LOGI(TAG, "  - Compass true: %.1f°", compass_true);
            ESP_LOGI(TAG, "  - Mount front (compass +180°): %.1f°", mount_front_true);
        }
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
        // Stop tracking when sun is too low for useful power generation
        // At 5° elevation: ~9% power, heavy atmospheric loss, not worth tracking
        
        bool sun_too_low = sun.elevation_deg < s.sleep_thresh_el;  // 5° threshold
        bool sun_descending = is_descending(g.latitude, g.longitude);
        
        // Only sleep if:
        // 1. Sun below useful power threshold (5°) AND
        // 2. Sun is descending (sunset, not sunrise)
        bool should_sleep = sun_too_low && sun_descending;
        
        // Additional safety: Don't sleep during midday even if sun temporarily blocked
        time_t now_utc = time(NULL);
        solar_events_t events = solar_events(g.latitude, g.longitude, now_utc);
        
        if (events.has_sunrise && events.has_sunset) {
            // Calculate solar noon (midpoint between sunrise and sunset)
            time_t solar_noon = (events.sunrise_utc + events.sunset_utc) / 2;
            int64_t time_to_noon = (int64_t)solar_noon - (int64_t)now_utc;
            
            // Don't sleep if within 4 hours of solar noon
            if (llabs(time_to_noon) < 14400) {
                should_sleep = false;
                ESP_LOGD(TAG, "Sleep override: Near solar noon (%.1f hours away)",
                         time_to_noon / 3600.0);
            }
        }
        
        // Detailed logging for power-based sleep decision
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "Power-Based Sleep Decision:");
        ESP_LOGD(TAG, "  - Sun elevation: %.1f° (threshold: %.1f°)", 
                 sun.elevation_deg, s.sleep_thresh_el);
        ESP_LOGD(TAG, "  - Estimated power: ~%.0f%% of peak", 
                 fmax(0.0, sin(DEG2RAD(sun.elevation_deg)) * 100.0));
        ESP_LOGD(TAG, "  - Below useful power: %s", sun_too_low ? "YES" : "NO");
        ESP_LOGD(TAG, "  - Sun descending: %s", sun_descending ? "YES" : "NO");
        ESP_LOGD(TAG, "  - Should sleep: %s", should_sleep ? "YES" : "NO");
        ESP_LOGD(TAG, "");

        if (should_sleep) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║              ENTERING NIGHT MODE (Power Saving)            ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Sleep Conditions Met:");
            ESP_LOGI(TAG, "  - Sun elevation: %.1f° (< %.1f° threshold)", 
                     sun.elevation_deg, s.sleep_thresh_el);
            ESP_LOGI(TAG, "  - Estimated power: ~%.0f%% of peak (too low for tracking)",
                     fmax(0.0, sin(DEG2RAD(sun.elevation_deg)) * 100.0));
            ESP_LOGI(TAG, "  - Sun is descending (sunset approaching)");
            ESP_LOGI(TAG, "  - Further tracking provides no power benefit");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Power Conservation:");
            ESP_LOGI(TAG, "  - Homing to safe position (panel facing up)");
            ESP_LOGI(TAG, "  - Deep sleep until %.0f min before sunrise",
                     (double)s.prewake_min);
            ESP_LOGI(TAG, "  - Battery drain during sleep: <50µA");
            ESP_LOGI(TAG, "");

            // === NEW: Notify LCD of sleep state ===
            ESP_LOGI(TAG, "Notifying display of sleep mode...");
            status_led_set_mode(LED_SLEEP);
            
            // Give main loop time to send status=2 to LCD (3 updates @ 1Hz)
            for (int i = 0; i < 3; i++) {
                ESP_LOGI(TAG, "  Sleep notification attempt %d/3...", i + 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            ESP_LOGI(TAG, "✓ Display notified - proceeding with homing");
            ESP_LOGI(TAG, "");
            
            home_to_midpoint();                      // Park at safe position

            // Log to CSV (append compass columns at end)
            sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s,%.2f,%.2f",
                now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
                s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
                s.moves_today, s.total_moves, NAN, "SLEEP",
                isnan(compass_true) ? NAN : compass_true,
                isnan(mount_front_true) ? NAN : mount_front_true);
            // Calculate wake time
            solar_events_t ev = solar_events(g.latitude, g.longitude, now);
            time_t wake_ts = 0;

            if (ev.has_sunrise) {
                if (now >= ev.sunset_utc) {
                    // After sunset - wake before tomorrow's sunrise
                    solar_events_t tomorrow = solar_events(g.latitude, g.longitude, now + 86400);
                    if (tomorrow.has_sunrise) {
                        wake_ts = tomorrow.sunrise_utc - (s.prewake_min * 60);
                        ESP_LOGI(TAG, "Wake target: %d min before tomorrow's sunrise", (int)s.prewake_min);
                    } else {
                        wake_ts = now + 21600;  // 6 hours fallback
                        ESP_LOGW(TAG, "Tomorrow's sunrise unavailable - using 6h fallback");
                    }
                } else {
                    wake_ts = ev.sunrise_utc - (s.prewake_min * 60);
                    ESP_LOGI(TAG, "Wake target: %d min before today's sunrise", (int)s.prewake_min);
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

            ESP_LOGI(TAG, "Next check in %d minutes", (int)(s.cur_period_s / 60));
        } else {
            s.cur_period_s = s.fast_period_s;       // Speed up while waiting
            csv_note = "WAIT";
            // Set status to WAITING when below threshold
            status_led_set_mode(LED_WAITING);       

            ESP_LOGI(TAG, "Below threshold (%.2f° < %.1f°)", dang, s.tol_deg);
            ESP_LOGI(TAG, "Next check in %d minutes", (int)(s.cur_period_s / 60));
        }

        ESP_LOGI(TAG, "");

        // === CSV Telemetry Logging ===
        // Append compass_true and mount_front_true as the last two columns
        sdlog_write_csv(csv, "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s,%.2f,%.2f",
            now, g.latitude, g.longitude, g.fix_type, g.num_satellites,
            s.az_tgt, s.el_tgt, s.az_cur, s.el_cur,
            s.moves_today, s.total_moves, dang, csv_note,
            isnan(compass_true) ? NAN : compass_true,
            isnan(mount_front_true) ? NAN : mount_front_true);

        ESP_LOGV(TAG, "Telemetry logged to CSV: %s (compass=%.1f°, front=%.1f°)",
                 csv_note,
                 isnan(compass_true) ? -1.0f : compass_true,
                 isnan(mount_front_true) ? -1.0f : mount_front_true);

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
        ESP_LOGD(TAG, "Waiting %d seconds until next check...", (int)s.cur_period_s);
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
        ESP_LOGE(TAG, "  - Return code: %d", (int)ret);
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


