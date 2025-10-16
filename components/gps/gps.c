#include "gps.h"
#include "esp_log.h"
#include <time.h>
#include <sys/time.h>

static const char *TAG = "GPS";

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * HARDCODED LOCATION MODE - Testing Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * This mode bypasses all GPS hardware and returns fixed coordinates.
 * 
 * Purpose:
 *  - Test solar tracking algorithms without GPS hardware
 *  - Simulate specific geographic locations
 *  - Reduce power consumption during development
 *  - Enable operation without satellite visibility
 * 
 * Configuration:
 *  - Location: Auburn, Alabama
 *  - Latitude: 32.5990°N (positive = North)
 *  - Longitude: 85.4808°W (stored as negative = West)
 *  - Altitude: 200m above mean sea level
 *  - Time: Uses system clock (set manually at boot)
 * 
 * Hardware Impact:
 *  - I2C bus NOT initialized
 *  - GPIO pins available for other uses
 *  - GPS power supply not needed
 *  - No satellite signal required
 * 
 * To switch to real GPS mode:
 *  1. Set USE_HARDCODED_LOCATION=0 in CMakeLists.txt
 *  2. Implement real GPS hardware functions below
 *  3. Connect MAX-M10S module to I2C pins
 * 
 * Note: USE_HARDCODED_LOCATION is defined in project CMakeLists.txt
 *       Do not redefine it here - use the compiler definition
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

// Check if USE_HARDCODED_LOCATION is defined (should come from CMakeLists.txt)
#ifndef USE_HARDCODED_LOCATION
#warning "USE_HARDCODED_LOCATION not defined, defaulting to 0 (real GPS mode)"
#define USE_HARDCODED_LOCATION 0
#endif

#if USE_HARDCODED_LOCATION

// Auburn, Alabama coordinates (Auburn University campus area)
#define HARDCODED_LAT  32.5990   // Degrees north of equator
#define HARDCODED_LON  -85.4808  // Degrees west of prime meridian (negative)
#define HARDCODED_ALT  200.0     // Meters above mean sea level

/**
 * Initialize GPS subsystem - HARDCODED MODE
 * 
 * In hardcoded mode, this function:
 *  - Does NOT initialize I2C hardware
 *  - Does NOT configure GPIO pins
 *  - Does NOT communicate with GPS module
 *  - Simply logs configuration and returns success
 * 
 * The cfg parameter is accepted for API compatibility but ignored.
 */
esp_err_t gps_init(const gps_cfg_t *cfg) {
    (void)cfg;  // Unused parameter - no hardware to configure
    
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "║  GPS Module - HARDCODED LOCATION MODE                   ║");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  • Mode: Hardcoded (no GPS hardware)");
    ESP_LOGI(TAG, "  • Location: Auburn, Alabama");
    ESP_LOGI(TAG, "  • Coordinates: %.4f°N, %.4f°W", HARDCODED_LAT, -HARDCODED_LON);
    ESP_LOGI(TAG, "  • Altitude: %.1f m ASL", HARDCODED_ALT);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Hardware Status:");
    ESP_LOGI(TAG, "  • I2C bus: NOT initialized (not needed)");
    ESP_LOGI(TAG, "  • GPS module: NOT required");
    ESP_LOGI(TAG, "  • GPIO pins: Available for other uses");
    ESP_LOGI(TAG, "  • Time source: System clock (set at boot)");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    
    return ESP_OK;
}

/**
 * Get GPS fix - HARDCODED MODE
 * 
 * Returns fixed Auburn, AL coordinates with simulated quality metrics.
 * 
 * @param fix Output structure to receive GPS data
 * @param timeout_ms Ignored in hardcoded mode (returns immediately)
 * @return true (always succeeds in hardcoded mode)
 */
bool gps_get_fix(gps_fix_t *fix, uint32_t timeout_ms) {
    (void)timeout_ms;  // Unused - no waiting needed
    
    if (!fix) {
        ESP_LOGE(TAG, "NULL fix pointer provided");
        return false;
    }

    // Populate fix structure with hardcoded Auburn, AL data
    fix->latitude = HARDCODED_LAT;
    fix->longitude = HARDCODED_LON;
    fix->altitude = HARDCODED_ALT;
    
    // Simulate excellent GPS quality (would come from satellites normally)
    fix->num_satellites = 12;  // Typical good fix has 8-12 satellites
    fix->hdop = 0.8;           // HDOP < 1.0 is excellent accuracy
    
    // Get current system time (must be set manually in main.c)
    time_t now;
    time(&now);
    fix->time = now;
    
    // Log at debug level to avoid spam (called frequently)
    ESP_LOGD(TAG, "GPS fix: (%.4f, %.4f) alt=%.1fm, sats=%d, hdop=%.1f", 
             HARDCODED_LAT, HARDCODED_LON, HARDCODED_ALT, 
             fix->num_satellites, fix->hdop);
    
    return true;
}

/**
 * Wait for GPS fix - HARDCODED MODE
 * 
 * Always returns immediately since hardcoded location is always "ready".
 * 
 * @param timeout_ms Ignored (no waiting needed)
 * @return true (always succeeds)
 */
bool gps_wait_for_fix(uint32_t timeout_ms) {
    (void)timeout_ms;  // Unused - no waiting needed
    
    ESP_LOGI(TAG, "GPS fix available immediately (hardcoded mode)");
    return true;
}

/**
 * Deinitialize GPS - HARDCODED MODE
 * 
 * Does nothing since no hardware was initialized.
 */
void gps_deinit(void) {
    ESP_LOGI(TAG, "GPS deinit (no hardware to shutdown)");
}

#else
/*
 * ═══════════════════════════════════════════════════════════════════════════
 * REAL GPS MODE - Hardware Implementation
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * To implement real GPS functionality:
 * 
 * 1. Initialize I2C bus with provided configuration
 * 2. Configure MAX-M10S module:
 *    - Set navigation mode (stationary for solar tracker)
 *    - Enable UBX-NAV-PVT messages (position/velocity/time)
 *    - Set update rate (1 Hz typical)
 * 3. Implement UBX protocol parser to extract:
 *    - Latitude/longitude (convert from scaled integers)
 *    - Altitude above sea level
 *    - UTC time (convert to Unix timestamp)
 *    - Number of satellites used
 *    - HDOP quality metric
 * 4. Handle errors: timeout, checksum failures, lost fix
 * 
 * Reference: u-blox MAX-M10S Integration Manual
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

// Stub implementations for real GPS mode (to be implemented)
esp_err_t gps_init(const gps_cfg_t *cfg) {
    (void)cfg;
    ESP_LOGE(TAG, "Real GPS mode not implemented yet");
    return ESP_ERR_NOT_SUPPORTED;
}

bool gps_get_fix(gps_fix_t *fix, uint32_t timeout_ms) {
    (void)fix;
    (void)timeout_ms;
    ESP_LOGE(TAG, "Real GPS mode not implemented yet");
    return false;
}

bool gps_wait_for_fix(uint32_t timeout_ms) {
    (void)timeout_ms;
    ESP_LOGE(TAG, "Real GPS mode not implemented yet");
    return false;
}

void gps_deinit(void) {
    ESP_LOGI(TAG, "Real GPS mode not implemented yet");
}

#endif