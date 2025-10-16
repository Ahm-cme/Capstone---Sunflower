#pragma once
/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ GPS (BN-880 over UART) + HMC5883 Compass (I2C)                       │
    │ - NMEA-0183 sentence parser (GGA, RMC)                               │
    │ - 9600 bps, 1Hz update rate                                          │
    │ - HMC5883 compass for automatic mount orientation detection           │
    └───────────────────────────────────────────────────────────────────────┘

    Wiring (project default):
    - UART: UART_NUM_2, TX=GPIO17, RX=GPIO16
    - GPS TX → ESP32 RX (GPIO16)
    - GPS RX → ESP32 TX (GPIO17) [optional, for config]
    - I2C: SDA=GPIO21, SCL=GPIO22 (for HMC5883 compass)
    - GPS VCC → 5V, GND → GND

    Notes:
    - BN-880 outputs NMEA sentences at 9600 baud, 1Hz by default
    - We parse GGA (position) and RMC (time, speed, heading)
    - HMC5883 compass on I2C for automatic calibration
*/

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"

typedef struct {
    double   latitude;          // degrees
    double   longitude;         // degrees
    double   altitude_m;        // meters (MSL)
    uint8_t  fix_type;          // 0=no fix, 1=GPS fix, 2=DGPS fix
    uint8_t  num_satellites;    // SVs used
    bool     valid;             // true if fields are valid
    time_t   timestamp;         // local epoch when fix parsed
    float    ground_speed_mps;  // m/s (from RMC)
    float    heading_deg;       // degrees (course over ground from RMC)
} gps_data_t;

typedef struct {
    int      uart_port;  // UART_NUM_2 recommended
    int      tx_io;      // GPIO17 (ESP32 TX → GPS RX)
    int      rx_io;      // GPIO16 (ESP32 RX ← GPS TX)
    int      baud_rate;  // 9600 (BN-880 default)
    
    // I2C for HMC5883 compass
    int      i2c_port;   // I2C_NUM_0 recommended
    int      sda_io;     // GPIO21 (I2C data)
    int      scl_io;     // GPIO22 (I2C clock)
} gps_cfg_t;

/*
    Initialize UART and GPS module + HMC5883 compass.
    - Configures UART with 9600-8-N-1
    - Sets up RX buffer for NMEA sentences
    - Initializes I2C for compass
    - Configures HMC5883 for continuous mode
    - Returns ESP_OK on success
*/
esp_err_t gps_init(const gps_cfg_t *cfg);

/*
    Read and parse NMEA sentences from GPS.
    - Returns true if a valid fix is parsed and 'out' is filled
    - Also updates the internal last-fix cache on success
    - Blocks briefly to read UART data
*/
bool gps_poll_nav_pvt(gps_data_t *out);

/*
    Copy the last known good fix into 'out'.
    - Returns false if no valid fix has ever been parsed this boot
*/
bool gps_get_last(gps_data_t *out);

/*
    Read magnetic heading from HMC5883 compass.
    
    Returns: true if valid heading obtained
    
    Parameters:
    - heading_deg: output magnetic heading (0-360°, 0=North, 90=East)
    
    Notes:
    - Heading is magnetic north (not true north)
    - Add local magnetic declination for true north
    - For solar tracking, magnetic north is fine (consistent reference)
    - Compass should be calibrated away from motors/metal
    
    Usage example:
        float mount_heading;
        if (gps_get_compass_heading(&mount_heading)) {
            ESP_LOGI("APP", "Mount facing: %.1f° magnetic", mount_heading);
        }
*/
bool gps_get_compass_heading(float *heading_deg);

/*
    Perform compass calibration routine.
    
    Instructions:
    1. Call this function to start calibration
    2. Slowly rotate the entire system 360° horizontally (2-3 full circles)
    3. Calibration captures min/max values for each axis
    4. Returns true when sufficient data collected
    
    This should be done once after hardware assembly, away from:
    - Power lines
    - Large metal objects
    - Motors (while off)
    - Magnetic materials
    
    Calibration data is stored in NVS and persists across reboots.
    Re-calibrate if you move the system or add nearby metal.
*/
bool gps_calibrate_compass(void);

/*
    Get compass calibration status.
    Returns: true if compass has been calibrated
*/
bool gps_is_compass_calibrated(void);