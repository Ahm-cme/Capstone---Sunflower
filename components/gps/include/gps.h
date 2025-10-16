#pragma once
/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ GPS (MAX‑M10S over I2C)                                               │
    │ - Lightweight UBX NAV‑PVT poller and last‑fix cache                  │
    │ - Used by tracking to get lat/lon/time and basic motion info         │
    └───────────────────────────────────────────────────────────────────────┘

    Wiring (project default):
    - I2C bus: I2C_NUM_0, SDA=GPIO18, SCL=GPIO19, 400 kHz
    - GPS address: 0x42 (u‑blox I2C default)

    Notes :
    - We only use UBX NAV‑PVT. No NMEA, no continuous streaming.
    - We actively poll once per loop; keep stack small and code simple.
    - If you later add time sync from GPS to RTC: do it where a valid fix exists.
*/

#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * GPS Module - Hardware Abstraction Layer
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * This module provides GPS functionality with two operation modes:
 * 
 * 1. HARDCODED_LOCATION Mode (USE_HARDCODED_LOCATION=1):
 *    - No GPS hardware required
 *    - Returns fixed coordinates (Auburn, AL)
 *    - I2C bus not initialized
 *    - Useful for testing/simulation
 * 
 * 2. Real GPS Mode (USE_HARDCODED_LOCATION=0):
 *    - Requires MAX-M10S GPS module via I2C
 *    - Acquires satellite fix for real-time positioning
 *    - Full UBX protocol implementation
 * 
 * Configuration is controlled by USE_HARDCODED_LOCATION compile definition
 * set in the project's CMakeLists.txt file.
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief GPS configuration structure
 * 
 * I2C settings for MAX-M10S GPS module communication.
 * In hardcoded mode, these parameters are ignored.
 */
typedef struct {
    i2c_port_t i2c_port;    ///< I2C port number (I2C_NUM_0 or I2C_NUM_1)
    int sda_io;             ///< GPIO pin for I2C SDA line
    int scl_io;             ///< GPIO pin for I2C SCL line
    uint32_t clk_hz;        ///< I2C clock frequency (typically 400kHz)
    uint8_t addr;           ///< GPS I2C address (0x42 for MAX-M10S)
} gps_cfg_t;

/**
 * @brief GPS fix data structure
 * 
 * Contains position, time, and quality information.
 * In hardcoded mode, this is populated with fixed Auburn, AL coordinates.
 */
typedef struct {
    double latitude;        ///< Latitude in decimal degrees (positive = North)
    double longitude;       ///< Longitude in decimal degrees (positive = East)
    double altitude;        ///< Altitude above sea level in meters
    time_t time;            ///< Unix timestamp (seconds since Jan 1, 1970)
    uint8_t num_satellites; ///< Number of satellites used in fix
    float hdop;             ///< Horizontal dilution of precision (lower = better)
} gps_fix_t;

/**
 * @brief Initialize GPS subsystem
 * 
 * Behavior depends on USE_HARDCODED_LOCATION setting:
 * - Hardcoded mode: Returns ESP_OK immediately, no hardware access
 * - Real GPS mode: Initializes I2C bus and configures GPS module
 * 
 * @param cfg Pointer to GPS configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gps_init(const gps_cfg_t *cfg);

/**
 * @brief Get current GPS fix
 * 
 * Behavior depends on USE_HARDCODED_LOCATION setting:
 * - Hardcoded mode: Returns fixed Auburn, AL coordinates immediately
 * - Real GPS mode: Polls GPS for latest position and time
 * 
 * @param fix Pointer to structure to receive GPS data
 * @param timeout_ms Maximum time to wait for data (ignored in hardcoded mode)
 * @return true if valid fix obtained, false otherwise
 */
bool gps_get_fix(gps_fix_t *fix, uint32_t timeout_ms);

/**
 * @brief Wait for valid GPS fix
 * 
 * Behavior depends on USE_HARDCODED_LOCATION setting:
 * - Hardcoded mode: Returns true immediately (always "ready")
 * - Real GPS mode: Blocks until satellite fix is acquired
 * 
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if fix available, false on timeout
 */
bool gps_wait_for_fix(uint32_t timeout_ms);

/**
 * @brief Deinitialize GPS subsystem
 * 
 * Behavior depends on USE_HARDCODED_LOCATION setting:
 * - Hardcoded mode: Does nothing (no hardware to shutdown)
 * - Real GPS mode: Powers down GPS module and releases I2C bus
 */
void gps_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_H