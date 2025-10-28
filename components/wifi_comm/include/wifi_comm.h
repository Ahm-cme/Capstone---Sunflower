/*
 * WiFi Communication Module - Header
 * 
 * Purpose:
 *  Wireless communication between main tracker and remote LCD display.
 * 
 * Architecture:
 *  Main Tracker (AP) ─WiFi─> LCD Display (Station)
 *  - Tracker broadcasts "SunflowerTracker" WiFi network
 *  - Display connects and receives tracking data
 *  - TCP socket on port 8888
 *  - 1 Hz update rate (1 packet/second)
 * 
 * Usage - Main Tracker:
 *  wifi_comm_init_ap();           // Start WiFi AP
 *  tracker_data_t data = {...};   // Fill with sensor data
 *  wifi_comm_send_data(&data);    // Send to display
 * 
 * Usage - LCD Display:
 *  wifi_comm_init_station();      // Connect to tracker
 *  tracker_data_t data;
 *  wifi_comm_receive_data(&data, 2000);  // Receive data
 * 
 * Network Config:
 *  - SSID: "SunflowerTracker"
 *  - Password: "sunflower2025"
 *  - Tracker IP: 192.168.4.1
 *  - Display IP: 192.168.4.2 (DHCP)
 *  - Port: 8888
 * 
 * Notes:
 *  - Tracker operates independently if display disconnects
 *  - Display can reconnect anytime
 *  - WiFi adds ~100-150mA power consumption
 */

#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * Tracker Data Packet
 * 
 * Sent from main tracker to LCD display every second.
 * 
 * IMPORTANT: Must be identical on both ESP32s!
 *            Update both if you change this.
 * 
 * Status codes:
 *  0   = standby (waiting to start)
 *  1   = tracking (normal operation)
 *  2   = sleep (deep sleep mode)
 *  3   = calibrating (manual calibration in progress)
 *  255 = error (system fault)
 * 
 * Size: 36 bytes (packed struct)
 */
typedef struct {
    float    elevation;           // Panel elevation angle (0-90°)
    float    azimuth;             // Panel azimuth angle (0-360°)
    float    delta_elevation;     // Change in elevation since last move
    float    delta_azimuth;       // Change in azimuth since last move
    uint16_t battery_adc;         // Battery ADC reading (0-4095)
    float    battery_voltage;     // Battery voltage (V)
    uint32_t timestamp;           // Unix timestamp (seconds since epoch)
    uint8_t  status;              // System status (0=standby,1=tracking,2=sleep,3=calib,255=error)
    float    latitude;            // GPS latitude (decimal degrees)
    float    longitude;           // GPS longitude (decimal degrees)
    uint8_t  gps_valid;           // GPS fix status: 0=invalid, 1=valid
    uint8_t  gps_satellites;      // Number of satellites in view (0-255)
    float    sun_elevation;       // Calculated sun elevation (0-90°, from solar.c)
    float    sun_azimuth;         // Calculated sun azimuth (0-360°, from solar.c)
    uint32_t moves_today;         // Number of moves since midnight
    uint32_t total_moves;         // Total moves since deployment
    uint16_t uptime_hours;        // Hours since last deep sleep wake
    int8_t   wifi_rssi;           // WiFi signal strength (dBm, -128 to 0)
    uint8_t  tracking_quality;    // Tracking error: abs(panel - sun) in degrees (0-180)
    
} tracker_data_t;

/*
 * Initialize WiFi Access Point (Main Tracker)
 * 
 * Sets up WiFi AP and TCP server for display communication.
 * Call once during startup.
 * 
 * Returns:
 *  ESP_OK   - WiFi started successfully
 *  ESP_FAIL - Failed to start
 */
esp_err_t wifi_comm_init_ap(void);

/*
 * Send Data to Display (Main Tracker)
 * 
 * Sends current tracking data to connected display.
 * Call from main loop at 1 Hz.
 * 
 * Returns:
 *  ESP_OK                - Data sent
 *  ESP_ERR_INVALID_STATE - Display not connected
 *  ESP_ERR_NOT_FOUND     - Display connecting (wait)
 *  ESP_FAIL              - Send failed
 */
esp_err_t wifi_comm_send_data(const tracker_data_t *data);

/*
 * Check Display Connection (Main Tracker)
 * 
 * Returns:
 *  true  - Display connected and ready
 *  false - Display not connected
 */
bool wifi_comm_is_connected(void);

#endif // WIFI_COMM_H