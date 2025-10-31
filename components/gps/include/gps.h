#pragma once
/*
    GPS + Compass Module (BN-880 over UART, HMC5883 over I2C)

    Purpose:
    - Parse GPS position/time from NMEA (GGA, RMC) at 9600 bps, 1 Hz.
    - Read magnetic heading from HMC5883 for automatic mount orientation.
    - Provide a cached last-fix for use when live data is momentarily unavailable.

    Project wiring (default):
    - UART2: TX=GPIO17 (ESP32→GPS RX, optional), RX=GPIO16 (GPS TX→ESP32)
    - I2C0:  SDA=GPIO21, SCL=GPIO22 (HMC5883)
    - GPS power: VCC 5V, GND

    Notes:
    - Magnetic heading is relative to magnetic north (apply declination externally if needed).
    - Call gps_init() once before using other functions.
*/

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_err.h"

/*
    Parsed GPS fix (latest valid values).
*/
typedef struct {
    double   latitude;          // degrees (+N, -S)
    double   longitude;         // degrees (+E, -W)
    double   altitude_m;        // meters MSL
    uint8_t  fix_type;          // 0=no fix, 1=GPS, 2=DGPS
    uint8_t  num_satellites;    // satellites used
    bool     valid;             // true if fields are valid
    time_t   timestamp;         // local epoch when parsed
    float    ground_speed_mps;  // m/s (from RMC)
    float    heading_deg;       // course over ground (deg, from RMC)
} gps_data_t;

/*
    Static configuration for UART (GPS) and I2C (compass).
*/
typedef struct {
    // UART (GPS)
    int uart_port;   // e.g., UART_NUM_2
    int tx_io;       // ESP32 TX → GPS RX (optional)
    int rx_io;       // ESP32 RX ← GPS TX
    int baud_rate;   // 9600

    // I2C (HMC5883)
    int i2c_port;    // e.g., I2C_NUM_0
    int sda_io;      // GPIO for SDA
    int scl_io;      // GPIO for SCL
} gps_cfg_t;

/*
    Initialize UART (NMEA 9600-8-N-1) and I2C compass.
    - Sets UART RX buffer for NMEA lines.
    - Puts HMC5883 into continuous-conversion mode.
    - Returns ESP_OK on success, ESP_FAIL if compass failed (GPS still works)
    - Returns ESP_ERR_INVALID_STATE if UART init failed (critical)
*/
esp_err_t gps_init(const gps_cfg_t *cfg);

/*
    Poll UART, parse NMEA (GGA/RMC).
    - On success: fills 'out', updates internal last-fix cache, returns true.
    - May block briefly while reading (2 second timeout).
    - Returns false if no valid fix within timeout.
*/
bool gps_poll_nav_pvt(gps_data_t *out);

/*
    Quick communication test - checks if GPS is responding.
    - Waits up to timeout_ms for any NMEA sentence.
    - Does NOT require valid fix, just communication.
    - Returns true if GPS is sending data, false if no response.
*/
bool gps_test_communication(uint32_t timeout_ms);

/*
    Copy last valid fix (from internal cache) into 'out'.
    - Returns false if no valid fix has been seen since boot.
*/
bool gps_get_last(gps_data_t *out);

/*
    Read magnetic heading from HMC5883.
    - Returns true on success.
    - heading_deg: 0–360 (0=North, 90=East), magnetic north.
*/
bool gps_get_compass_heading(float *heading_deg);

/*
 * Get compass heading corrected to TRUE north (not magnetic).
 * 
 * Applies magnetic declination correction:
 *   true_heading = magnetic_heading + declination
 * 
 * Declination sources (in priority order):
 *   1. User-configured value (via gps_set_magnetic_declination)
 *   2. Estimated from GPS location (regional approximation)
 *   3. Zero if no GPS fix available
 * 
 * Returns:
 *   true  - heading retrieved successfully
 *   false - compass read failed
 */
bool gps_get_compass_heading_true(float *heading_true_deg);

/*
 * Set magnetic declination for current location (degrees).
 * 
 * Call once after installation with value from:
 *   https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 * 
 * Positive = magnetic north is EAST of true north
 * Negative = magnetic north is WEST of true north
 * 
 * Example values:
 *   New York, NY: -13°
 *   Los Angeles, CA: +12°
 *   London, UK: +1°
 *   Sydney, Australia: +12°
 * 
 * Stored in NVS, persists across reboots.
 */
void gps_set_magnetic_declination(float declination_deg);

/*
 * Get current declination setting (returns 0 if not configured).
 */
float gps_get_magnetic_declination(void);

/*
 * Calibrate compass by rotating system 360° horizontally.
 * - Takes 20 seconds, logs progress.
 * - Returns true on success, false if insufficient rotation.
 * - Saves calibration to NVS automatically.
 */
bool gps_calibrate_compass(void);

/*
 * Check if compass has been calibrated.
 * - Returns true if calibration data exists in NVS.
 */
bool gps_is_compass_calibrated(void);

/*
 * Check if compass hardware is present and responding.
 * - Returns true if HMC5883 is detected on I2C bus.
 */
bool gps_is_compass_present(void);