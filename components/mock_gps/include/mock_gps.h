#pragma once

#include <stdbool.h>
#include <time.h>
#include "gps.h"

/**
 * Mock GPS Module
 * 
 * Provides simulated GPS and compass data for testing when hardware is unavailable.
 * 
 * Mock Data:
 *  - Location: 32.604498°N, -85.486718°W (Auburn, AL area)
 *  - Time: System time (uses ESP32 RTC)
 *  - Satellites: 12 (good fix)
 *  - Compass: 180° ± 5° (simulated variation)
 *  - Fix type: 3D fix
 */

/**
 * Initialize mock GPS module
 * 
 * This replaces the real GPS module with simulated data.
 * Call this INSTEAD of gps_init() when testing.
 * 
 * Returns:
 *   ESP_OK on success
 */
esp_err_t mock_gps_init(void);

/**
 * Get mock GPS data (simulates gps_poll_nav_pvt)
 * 
 * Always returns true (fix available) with hardcoded location.
 * Updates timestamp to current system time.
 * 
 * Args:
 *   data: Pointer to gps_data_t structure to fill
 * 
 * Returns:
 *   true (always has fix in mock mode)
 */
bool mock_gps_poll_nav_pvt(gps_data_t *data);

/**
 * Get last mock GPS data (simulates gps_get_last)
 */
bool mock_gps_get_last(gps_data_t *data);

/**
 * Get mock compass heading (simulates gps_get_compass_heading_true)
 * 
 * Returns heading between 175-185° (simulated variation).
 * 
 * Args:
 *   heading: Pointer to store heading in degrees (0-360)
 * 
 * Returns:
 *   true (always available in mock mode)
 */
bool mock_gps_get_compass_heading_true(float *heading);

/**
 * Check if mock compass is "calibrated"
 * 
 * Always returns true in mock mode.
 */
bool mock_gps_is_compass_calibrated(void);

/**
 * Get mock magnetic declination
 * 
 * Returns approximate declination for Auburn, AL area (~4° West).
 */
float mock_gps_get_magnetic_declination(void);

/**
 * Mock compass calibration (does nothing, returns success)
 */
bool mock_gps_calibrate_compass(void);