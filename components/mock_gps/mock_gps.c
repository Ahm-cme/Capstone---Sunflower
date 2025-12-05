#include "mock_gps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <sys/time.h>

#define TAG "MOCK_GPS"

// Mock location: Auburn, AL area
#define MOCK_LATITUDE   32.604498
#define MOCK_LONGITUDE  -85.486718
#define MOCK_ALTITUDE   219.0f  // meters (approx elevation of Auburn)

// Mock compass heading range
#define MOCK_HEADING_MIN  175.0f
#define MOCK_HEADING_MAX  185.0f
#define MOCK_HEADING_CENTER 180.0f

// Mock magnetic declination for Auburn, AL
#define MOCK_DECLINATION  -4.0f  // 4° West

// Simulated fix quality
#define MOCK_SATELLITES  12
#define MOCK_FIX_TYPE    3  // 3D fix
#define MOCK_HDOP        1.2f

// Last mock data
static gps_data_t s_last_mock_data = {0};
static bool s_initialized = false;

esp_err_t mock_gps_init(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          MOCK GPS MODULE (Testing Mode)                   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "⚠ Using SIMULATED GPS data - for testing only!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Mock Configuration:");
    ESP_LOGI(TAG, "  Location: %.6f°N, %.6f°W", MOCK_LATITUDE, fabs(MOCK_LONGITUDE));
    ESP_LOGI(TAG, "  Altitude: %.1f meters", MOCK_ALTITUDE);
    ESP_LOGI(TAG, "  Satellites: %d (simulated)", MOCK_SATELLITES);
    ESP_LOGI(TAG, "  Compass: %.0f° ± %.0f°", MOCK_HEADING_CENTER, 
             (MOCK_HEADING_MAX - MOCK_HEADING_MIN) / 2);
    ESP_LOGI(TAG, "  Declination: %.1f° (Auburn, AL)", MOCK_DECLINATION);
    ESP_LOGI(TAG, "");
    
    // === SET MOCK TIME: November 25, 2025 11:00 AM EST (16:00 UTC) ===
    struct tm mock_time = {
        .tm_year = 2025 - 1900,  // Years since 1900
        .tm_mon = 12 - 1,        // 0-11 (December = 11)
        .tm_mday = 5,           // Day of month
        .tm_hour = 19,           // 16:00 UTC = 11:00 AM EST
        .tm_min = 0,
        .tm_sec = 0,
        .tm_isdst = 0            // Not daylight saving (standard time)
    };
    
    time_t mock_timestamp = mktime(&mock_time);
    struct timeval tv = {
        .tv_sec = mock_timestamp,
        .tv_usec = 0
    };
    settimeofday(&tv, NULL);
    
    ESP_LOGI(TAG, "  Mock Time Set: 2025-11-25 11:00:00 EST (16:00 UTC)");
    ESP_LOGI(TAG, "");
    
    // Initialize last mock data
    memset(&s_last_mock_data, 0, sizeof(gps_data_t));
    s_last_mock_data.latitude = MOCK_LATITUDE;
    s_last_mock_data.longitude = MOCK_LONGITUDE;
    s_last_mock_data.altitude_m = MOCK_ALTITUDE;
    s_last_mock_data.num_satellites = MOCK_SATELLITES;
    s_last_mock_data.fix_type = MOCK_FIX_TYPE;
    s_last_mock_data.timestamp = time(NULL);  // Now returns mock time
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "✓ Mock GPS initialized successfully");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

bool mock_gps_poll_nav_pvt(gps_data_t *data) {
    // CHANGED: Always return data, even if not fully initialized
    if (!data) {
        return false;
    }
    
    // Initialize on first call if needed (safety fallback)
    if (!s_initialized) {
        ESP_LOGW(TAG, "Mock GPS not initialized - auto-initializing");
        mock_gps_init();
    }
    
    // Update timestamp to current time
    time_t now = time(NULL);
    
    // Copy mock data
    *data = s_last_mock_data;
    data->timestamp = now;
    
    // Add small random variation to altitude (±2m) for realism
    static int alt_offset = 0;
    alt_offset = (alt_offset + 1) % 5 - 2;  // Cycles -2, -1, 0, 1, 2
    data->altitude_m = MOCK_ALTITUDE + (float)alt_offset * 0.5f;
    
    ESP_LOGV(TAG, "Mock GPS poll: %.6f,%.6f @ %ld (%u sats)",
             data->latitude, data->longitude, (long)data->timestamp, 
             data->num_satellites);
    
    return true;  // Always succeed
}

bool mock_gps_get_last(gps_data_t *data) {
    // CHANGED: Remove initialization check
    if (!data) {
        return false;
    }
    
    // Auto-init if needed
    if (!s_initialized) {
        mock_gps_init();
    }
    
    *data = s_last_mock_data;
    data->timestamp = time(NULL);
    
    return true;  // Always succeed
}

bool mock_gps_get_compass_heading_true(float *heading) {
    // CHANGED: Remove initialization check
    if (!heading) {
        return false;
    }
    
    // Auto-init if needed
    if (!s_initialized) {
        mock_gps_init();
    }
    
    // Simulate heading variation: cycles slowly between min and max
    static float current_heading = MOCK_HEADING_CENTER;
    static float heading_delta = 0.1f;  // Change per call
    
    current_heading += heading_delta;
    
    // Bounce between min and max
    if (current_heading >= MOCK_HEADING_MAX) {
        current_heading = MOCK_HEADING_MAX;
        heading_delta = -0.1f;
    } else if (current_heading <= MOCK_HEADING_MIN) {
        current_heading = MOCK_HEADING_MIN;
        heading_delta = 0.1f;
    }
    
    *heading = current_heading;
    
    ESP_LOGV(TAG, "Mock compass: %.1f°", current_heading);
    
    return true;  // Always succeed
}

bool mock_gps_is_compass_calibrated(void) {
    // Auto-init if needed
    if (!s_initialized) {
        mock_gps_init();
    }
    return true;  // Always calibrated in mock mode
}

float mock_gps_get_magnetic_declination(void) {
    // Auto-init if needed (safe fallback)
    if (!s_initialized) {
        mock_gps_init();
    }
    return MOCK_DECLINATION;
}

bool mock_gps_calibrate_compass(void) {
    // Auto-init if needed
    if (!s_initialized) {
        mock_gps_init();
    }
    
    ESP_LOGI(TAG, "Mock compass calibration (simulated)");
    ESP_LOGI(TAG, "  - In real mode, rotate panel 2-3 full circles");
    ESP_LOGI(TAG, "  - Mock mode: calibration always succeeds");
    
    // Simulate calibration delay
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "✓ Mock compass calibration complete");
    
    return true;
}