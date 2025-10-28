#ifndef WIFI_CLIENT_H
#define WIFI_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
===============================================================================
WiFi Client <-> Tracker Data Contract
-------------------------------------------------------------------------------
This header defines the binary payload exchanged with the master controller.
It must match the sender's struct layout (types, order, alignment, endianness).

Notes:
- Endianness: ESP32 is little-endian. Ensure the sender uses little-endian
  or serialize fields explicitly if cross-platform.
- Packing/alignment: Compilers may insert padding (e.g., after uint8_t).
  If the master uses different packing, consider using a framed protocol
  (e.g., length-prefixed) or enforce packing on both sides.
  Example (if absolutely needed on both ends):
    #pragma pack(push, 1)
    typedef struct { ... } tracker_data_t;
    #pragma pack(pop)
- Versioning: If fields change, add a version byte/field to the frame or
  switch to a JSON/CBOR format for flexibility.
- Field ranges:
  - elevation: 0..90 deg, azimuth: 0..360 deg
  - delta_*: deg/s (displayed as D:…)
  - battery_adc: 0..4095 (ADC raw)
  - battery_voltage: scaled volts
  - timestamp: seconds since epoch or monotonic counter
  - status: maps to LCD tracking_status (0=standby, 1=tracking, 2=sleep,
    3=calibrating, 255=error)
  - gps_valid: 0/1
===============================================================================
*/

// Data packet received from tracker (must match master)
typedef struct {
    // Panel angles
    float   elevation;        // degrees (0..90)
    float   azimuth;          // degrees (0..360)
    float   delta_elevation;  // degrees/sec
    float   delta_azimuth;    // degrees/sec

    // Power
    uint16_t battery_adc;     // raw ADC (0..4095)
    float    battery_voltage; // volts (scaled by master)

    // Time/status
    uint32_t timestamp;       // seconds (UTC or uptime)
    uint8_t  status;          // tracking status (see mapping above)

    // GPS
    float   latitude;         // degrees (-90..+90)
    float   longitude;        // degrees (-180..+180)
    uint8_t gps_valid;        // 0 = no fix, 1 = valid

    // NOTE: Potential padding after uint8_t fields depends on compiler ABI.
    // Keep sender and receiver toolchains aligned or serialize explicitly.
} tracker_data_t;

/*
-------------------------------------------------------------------------------
WiFi Client API
-------------------------------------------------------------------------------
- wifi_client_init:
    Initializes STA mode, connects to tracker AP, opens TCP socket.
    Returns ESP_OK on success.
- wifi_client_receive_data:
    Blocking read with timeout_ms. Returns:
      ESP_OK            -> full frame received into 'data'
      ESP_ERR_TIMEOUT   -> no data within timeout
      ESP_ERR_INVALID_SIZE -> partial frame (protocol mismatch)
      ESP_ERR_INVALID_STATE -> not connected / socket closed
      ESP_FAIL          -> other socket error
- wifi_client_is_connected:
    True when WiFi link is up AND TCP socket is open.
-------------------------------------------------------------------------------
*/

// Initialize WiFi as Station (connect to tracker)
esp_err_t wifi_client_init(void);

// Receive tracker data (blocks until data received or timeout)
esp_err_t wifi_client_receive_data(tracker_data_t *data, uint32_t timeout_ms);

// Check connection status (WiFi + socket)
bool wifi_client_is_connected(void);

#endif // WIFI_CLIENT_H