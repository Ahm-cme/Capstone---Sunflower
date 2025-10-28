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
    - Returns ESP_OK on success.
*/
esp_err_t gps_init(const gps_cfg_t *cfg);

/*
    Poll UART, parse NMEA (GGA/RMC).
    - On success: fills 'out', updates internal last-fix cache, returns true.
    - May block briefly while reading.
*/
bool gps_poll_nav_pvt(gps_data_t *out);

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
    Run compass calibration routine.
    - Rotate system 2–3 full turns horizontally.
    - Captures min/max, stores calibration in NVS.
    - Returns true when enough samples are collected.
*/
bool gps_calibrate_compass(void);

/*
    Query whether compass calibration data exists in NVS.
*/
bool gps_is_compass_calibrated(void);