/*
 * WiFi Communication Module - Header
 * 
 * Purpose:
 *  Wireless communication between main tracker and remote LCD display.
 * 
 * Architecture:
 *  Main Tracker (ESP32-CAM) ─WiFi─> LCD Display (ESP32-WROOM)
 *  - Tracker broadcasts "SunflowerTracker" WiFi network (AP mode)
 *  - Display connects as station and receives tracking data
 *  - TCP socket on port 8888 (unidirectional: tracker → display)
 *  - 1 Hz update rate (1 packet/second, 92 bytes/packet)
 * 
 * Usage - Main Tracker:
 *  wifi_comm_init_ap();           // Start WiFi AP + TCP server
 *  tracker_data_t data = {...};   // Fill with sensor data
 *  wifi_comm_send_data(&data);    // Send to display (blocking)
 * 
 * Usage - LCD Display (separate firmware):
 *  wifi_comm_init_station();      // Connect to tracker AP
 *  tracker_data_t data;
 *  wifi_comm_receive_data(&data, 2000);  // Receive with timeout
 * 
 * Network Configuration:
 *  - SSID: "SunflowerTracker"
 *  - Password: "sunflower2025" (WPA2-PSK)
 *  - Channel: 1 (2.4GHz, least interference)
 *  - Max stations: 2 (allow 2 displays simultaneously)
 *  - Tracker IP: 192.168.4.1 (fixed)
 *  - Display IP: 192.168.4.x (DHCP assigned)
 *  - Port: 8888 (TCP)
 * 
 * Performance Optimizations:
 *  - Maximum TX power (19.5 dBm) for extended range
 *  - Power-saving DISABLED (WiFi always-on for instant response)
 *  - TCP_NODELAY enabled (low latency, no Nagle delay)
 *  - TCP keepalive (5s idle, 2s probe interval, 3 probes)
 *  - 8KB send buffer (prevents packet loss during bursts)
 *  - 20MHz bandwidth (stable, better range than 40MHz)
 * 
 * Power Consumption:
 *  Main Tracker with WiFi:
 *  - Active + WiFi: 150-300mA (GPS + CPU + WiFi)
 *  - Deep sleep: 10-50µA (WiFi off during sleep)
 *  - WiFi overhead: ~100-150mA additional
 * 
 * Range & Reliability:
 *  - Line-of-sight: 50-100m (depends on obstacles)
 *  - Through walls: 20-30m (typical indoor)
 *  - Best placement: Tracker elevated, minimal obstructions
 *  - Interference: Avoid microwave ovens, Bluetooth devices
 * 
 * Fault Tolerance:
 *  - Tracker operates independently if display disconnects
 *  - Display can reconnect anytime (auto-reconnect loop)
 *  - TCP keepalive detects dead connections (11s timeout)
 *  - Socket auto-closes on error, retries next send
 *  - Data continues logging to SD card regardless of WiFi status
 * 
 * Integration Notes:
 *  - Call wifi_comm_init_ap() after nvs_flash_init()
 *  - Send data from main loop at 1 Hz (don't flood)
 *  - Check wifi_comm_is_connected() before sending
 *  - Non-critical: System works without WiFi (display is optional)
 */

#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * Tracker Data Packet (Main Tracker → LCD Display)
 * 
 * Complete system telemetry sent every second over WiFi.
 * 
 * ⚠ CRITICAL: This struct MUST be IDENTICAL on both ESP32s!
 *             Any changes require reflashing BOTH devices.
 * 
 * Packet Structure (92 bytes packed):
 *  - Panel position (16 bytes): current + delta angles
 *  - Battery (13 bytes): voltage, SoC, charging status
 *  - Timing (12 bytes): timestamp, sunrise, sunset
 *  - GPS (18 bytes): position, fix quality, age
 *  - Sun position (8 bytes): calculated from NOAA algorithms
 *  - Statistics (14 bytes): moves, uptime
 *  - Health (5 bytes): WiFi, SD card, tracking quality
 *  - System status (1 byte): tracking/sleep/error state
 * 
 * System Status Codes (status field):
 *  0   = STANDBY    - Waiting to start (button press needed)
 *  1   = TRACKING   - Normal operation (following sun)
 *  2   = SLEEP      - Deep sleep mode (night time)
 *  3   = CALIBRATING - Manual calibration in progress
 *  255 = ERROR      - System fault detected
 * 
 * Battery SOC Levels (battery_soc field):
 *  0 = CRITICAL  - <10% (shutdown imminent)
 *  1 = LOW       - 10-20% (charging recommended)
 *  2 = MEDIUM    - 20-50% (normal operation)
 *  3 = GOOD      - 50-80% (healthy)
 *  4 = FULL      - >80% (fully charged)
 * 
 * SD Card Status (sd_card_status field):
 *  0 = OK        - Working normally (<200ms writes)
 *  1 = SLOW      - Write delays detected (200-500ms)
 *  2 = FULL      - >90% capacity used
 *  3 = FAILED    - Mount/write errors
 * 
 * GPS Fix Types (gps_valid field):
 *  0 = NO_FIX    - No satellites, position invalid
 *  1 = FIX_2D    - 2D fix (lat/lon only)
 *  2 = FIX_3D    - 3D fix (lat/lon/altitude)
 *  3 = DGPS      - Differential GPS (enhanced accuracy)
 * 
 * Tracking Quality (tracking_quality field):
 *  0-5°   = EXCELLENT - Panel aligned with sun
 *  5-10°  = GOOD      - Normal tracking accuracy
 *  10-20° = FAIR      - May need recalibration
 *  >20°   = POOR      - Calibration or homing needed
 * 
 * Notes:
 *  - All floats are 32-bit IEEE 754
 *  - All multi-byte integers are little-endian
 *  - Struct is packed (no padding bytes)
 *  - Total size: exactly 92 bytes
 */
typedef struct {
    // ═══ Panel Position (16 bytes) ═══
    float    elevation;              // Current panel elevation angle (0-90°)
    float    azimuth;                // Current panel azimuth angle (0-360°)
    float    delta_elevation;        // Change in elevation since last move (degrees)
    float    delta_azimuth;          // Change in azimuth since last move (degrees)
    
    // ═══ Battery Monitoring (13 bytes) ═══
    uint16_t battery_adc;            // Raw ADC reading (0-4095, 12-bit)
    float    battery_voltage;        // Actual battery voltage (V, after divider)
    float    battery_soc_percent;    // State of charge percentage (0.0-100.0%)
    uint8_t  battery_soc;            // SOC level enum (0-4, see above)
    uint8_t  battery_charging;       // Charging status: 0=discharging, 1=charging
    
    // ═══ Timing (12 bytes) ═══
    uint32_t timestamp;              // Current Unix timestamp (seconds since epoch)
    uint32_t sunrise_time;           // Today's sunrise time (Unix timestamp)
    uint32_t sunset_time;            // Today's sunset time (Unix timestamp)
    
    // ═══ System Status (1 byte) ═══
    uint8_t  status;                 // System state (0-3, 255, see codes above)
    
    // ═══ GPS Data (18 bytes) ═══
    float    latitude;               // GPS latitude (decimal degrees, ±90°)
    float    longitude;              // GPS longitude (decimal degrees, ±180°)
    uint8_t  gps_valid;              // GPS fix quality (0-3, see types above)
    uint8_t  gps_satellites;         // Number of satellites tracked (0-255)
    uint32_t last_gps_fix_age_sec;   // Seconds since last valid GPS fix
    
    // ═══ Sun Position (8 bytes) ═══
    float    sun_elevation;          // Calculated sun elevation (0-90°, from NOAA algorithm)
    float    sun_azimuth;            // Calculated sun azimuth (0-360°, from NOAA algorithm)
    
    // ═══ Statistics (14 bytes) ═══
    uint32_t moves_today;            // Number of motor moves since midnight UTC
    uint32_t total_moves;            // Total moves since deployment (persisted in NVS)
    uint16_t uptime_hours;           // Hours since last deep sleep wake
    
    // ═══ Network & Health (5 bytes) ═══
    int8_t   wifi_rssi;              // WiFi signal strength (dBm, -128 to 0)
    uint8_t  wifi_clients;           // Number of connected WiFi clients (LCD displays)
    uint8_t  sd_card_status;         // SD card health (0-3, see codes above)
    uint8_t  tracking_quality;       // Tracking error magnitude (0-180°)
    
} __attribute__((packed)) tracker_data_t;

/*
 * Initialize WiFi Access Point (Main Tracker Only)
 * 
 * Sets up ESP32 as WiFi AP with optimized settings for solar tracker.
 * Call once during system initialization (after nvs_flash_init).
 * 
 * What it does:
 *  1. Initializes WiFi stack (esp_netif, event loop)
 *  2. Configures AP mode (SSID, password, channel)
 *  3. Optimizes for range and reliability:
 *     - Maximum TX power (19.5 dBm)
 *     - Power-saving disabled (always-on for instant response)
 *     - 20MHz bandwidth (better range than 40MHz)
 *     - TCP keepalive and nodelay enabled
 *  4. Creates TCP server socket on port 8888
 *  5. Sets socket to non-blocking mode
 *  6. Starts WiFi and begins advertising
 * 
 * Returns:
 *  ESP_OK   - WiFi AP started, TCP server listening
 *  ESP_FAIL - Initialization failed (socket error)
 * 
 * On failure:
 *  - System continues without WiFi (display unavailable)
 *  - Data logging to SD card unaffected
 *  - Check console for detailed error messages
 * 
 * Troubleshooting:
 *  - If display can't see SSID: Check channel 1 isn't jammed
 *  - If connection drops: Reduce distance or add antenna
 *  - If slow: Verify 20MHz bandwidth setting
 *  - If power issues: Check battery can supply WiFi current
 * 
 * Power note:
 *  - Adds ~100-150mA continuous draw while active
 *  - WiFi is OFF during deep sleep (automatically)
 *  - Consider disabling WiFi if battery life critical
 */
esp_err_t wifi_comm_init_ap(void);

/*
 * Send Data Packet to LCD Display (Main Tracker Only)
 * 
 * Transmits complete tracker_data_t struct to connected display.
 * Call from main loop at 1 Hz (once per second).
 * 
 * Behavior:
 *  - Non-blocking accept() checks for new connections
 *  - If no client connected, returns ESP_ERR_NOT_FOUND immediately
 *  - If client connected, sends 92-byte packet via TCP
 *  - On send error, closes socket and returns ESP_FAIL
 *  - Automatic reconnection on next call (stateless)
 * 
 * Params:
 *  data - Pointer to populated tracker_data_t structure
 * 
 * Returns:
 *  ESP_OK                - Packet sent successfully
 *  ESP_ERR_INVALID_STATE - WiFi AP not ready (no station associated)
 *  ESP_ERR_NOT_FOUND     - No TCP client connected (waiting)
 *  ESP_FAIL              - Send failed (socket closed, will retry)
 * 
 * Timing:
 *  - Typical send time: 1-5ms (fast)
 *  - Timeout: 2 seconds (prevents blocking)
 *  - Rate limit: 1 Hz recommended (don't flood)
 * 
 * Error recovery:
 *  - Socket auto-closes on error
 *  - Next call will accept new connection
 *  - Display reconnects automatically
 *  - No action needed from caller
 * 
 * Usage:
 *  ```c
 *  tracker_data_t data = {
 *      .elevation = 45.0f,
 *      .azimuth = 180.0f,
 *      // ... fill all fields ...
 *  };
 *  
 *  esp_err_t ret = wifi_comm_send_data(&data);
 *  if (ret == ESP_OK) {
 *      ESP_LOGI(TAG, "Data sent to display");
 *  } else if (ret == ESP_ERR_NOT_FOUND) {
 *      ESP_LOGD(TAG, "Display not connected");
 *  }
 *  ```
 */
esp_err_t wifi_comm_send_data(const tracker_data_t *data);

/*
 * Check if LCD Display is Connected (Main Tracker Only)
 * 
 * Queries WiFi AP and TCP connection status.
 * 
 * Returns:
 *  true  - Display connected and TCP session active
 *  false - Display not connected or TCP not established
 * 
 * Use cases:
 *  - UI indicator (LED/console showing display status)
 *  - Conditional logging (log connection events)
 *  - Power management (disable WiFi if unused)
 * 
 * Note:
 *  - Connection can change at any time (asynchronous)
 *  - Check before send() is optional (send handles it)
 *  - Useful for statistics/monitoring only
 * 
 * Example:
 *  ```c
 *  if (wifi_comm_is_connected()) {
 *      ESP_LOGI(TAG, "Display active - streaming data");
 *  } else {
 *      ESP_LOGD(TAG, "Display offline - logging to SD only");
 *  }
 *  ```
 */
bool wifi_comm_is_connected(void);

/*
 * Get number of connected WiFi clients (Main Tracker Only)
 * 
 * Returns count of stations associated with our AP.
 * 
 * Returns:
 *  0-2 - Number of connected displays (max 2 allowed)
 * 
 * Use cases:
 *  - Display count on LCD
 *  - Log connection/disconnection events
 *  - Debug connectivity issues
 * 
 * Note:
 *  - Association != TCP connection (client may be connected but not receiving data)
 *  - Use wifi_comm_is_connected() for TCP status
 * 
 * Example:
 *  ```c
 *  uint8_t clients = wifi_comm_get_client_count();
 *  ESP_LOGI(TAG, "%u display(s) connected", clients);
 *  ```
 */
uint8_t wifi_comm_get_client_count(void);

/*
 * Get WiFi statistics (Main Tracker Only)
 * 
 * Fills structure with WiFi performance metrics.
 * 
 * Metrics:
 *  - TX/RX packet counts
 *  - Error counts
 *  - Signal strength (if available)
 *  - Connection duration
 * 
 * Used for:
 *  - Troubleshooting connectivity
 *  - Performance monitoring
 *  - Long-term statistics logging
 * 
 * Example:
 *  ```c
 *  wifi_stats_t stats;
 *  wifi_comm_get_stats(&stats);
 *  ESP_LOGI(TAG, "Sent: %lu packets, Errors: %lu", 
 *           stats.tx_packets, stats.tx_errors);
 *  ```
 */
typedef struct {
    uint32_t tx_packets;             // Total packets sent
    uint32_t rx_packets;             // Total packets received
    uint32_t tx_errors;              // Send errors
    uint32_t rx_errors;              // Receive errors
    uint32_t uptime_sec;             // WiFi uptime (seconds)
    int8_t   avg_rssi;               // Average RSSI (dBm, if available)
} wifi_stats_t;

void wifi_comm_get_stats(wifi_stats_t *stats);

#endif // WIFI_COMM_H