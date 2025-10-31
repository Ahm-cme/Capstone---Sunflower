/*
 * WiFi Communication Module - Client (LCD Display Side)
 * 
 * Purpose:
 *  Receives real-time tracking data from main tracker for LCD display.
 * 
 * Architecture:
 *  LCD Display (ESP32-WROOM) ─WiFi─> Main Tracker (ESP32-CAM)
 *  - Display connects as WiFi station to tracker's AP
 *  - Receives tracking data via TCP client on port 8888
 *  - Updates LCD display with received data at 1 Hz
 *  - Auto-reconnects if connection lost
 * 
 * Usage - LCD Display:
 *  wifi_client_init();                    // Connect to tracker AP
 *  tracker_data_t data;
 *  while (1) {
 *      if (wifi_client_receive_data(&data, 5000) == ESP_OK) {
 *          update_lcd_display(&data);     // Show data on LCD
 *      }
 *      vTaskDelay(pdMS_TO_TICKS(100));
 *  }
 * 
 * Network Configuration:
 *  - Connects to SSID: "SunflowerTracker"
 *  - Password: "sunflower2025" (WPA2-PSK)
 *  - Server IP: 192.168.4.1 (tracker's fixed AP address)
 *  - Server Port: 8888 (TCP)
 *  - Client IP: 192.168.4.x (DHCP assigned)
 * 
 * Connection Flow:
 *  1. Scan for "SunflowerTracker" SSID
 *  2. Connect using WPA2-PSK authentication
 *  3. Wait for DHCP IP assignment
 *  4. Connect TCP socket to 192.168.4.1:8888
 *  5. Receive 92-byte packets at ~1 Hz
 *  6. Parse and display data
 *  7. Auto-reconnect on disconnect
 * 
 * Auto-Reconnect Strategy:
 *  - WiFi disconnected: Attempt reconnect every 5 seconds
 *  - TCP disconnected: Attempt reconnect every 2 seconds
 *  - Max retries: Infinite (always trying to connect)
 *  - Exponential backoff: No (fixed intervals for simplicity)
 * 
 * Performance Optimizations:
 *  - Maximum TX power (19.5 dBm) for extended range
 *  - Power-saving DISABLED (always-on for instant updates)
 *  - TCP_NODELAY enabled (low latency reception)
 *  - Large receive buffer (8KB, prevents packet loss)
 *  - 20MHz bandwidth (stable, better range)
 * 
 * Power Consumption:
 *  LCD Display (ESP32-WROOM + ILI9341):
 *  - Active + WiFi + LCD: 200-400mA
 *  - WiFi overhead: ~100-150mA
 *  - LCD backlight: ~50-100mA
 *  - Total: Requires wall power or large battery
 * 
 * Error Handling:
 *  - WiFi timeout: Return ESP_ERR_TIMEOUT, caller retries
 *  - TCP timeout: Return ESP_ERR_TIMEOUT, caller retries
 *  - Connection lost: Auto-reconnect in background
 *  - Malformed packet: Log warning, request retransmit
 * 
 * Integration Notes:
 *  - Call wifi_client_init() in app_main()
 *  - Call wifi_client_receive_data() in display loop
 *  - Check wifi_client_is_connected() for status LED
 *  - Use wifi_client_get_signal_strength() for WiFi gauge
 */

#ifndef WIFI_CLIENT_H
#define WIFI_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Import shared data structure from main tracker
// ⚠ CRITICAL: Must be identical to tracker's definition!
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
    uint8_t  battery_soc;            // SOC level enum (0-4: CRITICAL/LOW/MEDIUM/GOOD/FULL)
    uint8_t  battery_charging;       // Charging status: 0=discharging, 1=charging
    
    // ═══ Timing (12 bytes) ═══
    uint32_t timestamp;              // Current Unix timestamp (seconds since epoch)
    uint32_t sunrise_time;           // Today's sunrise time (Unix timestamp)
    uint32_t sunset_time;            // Today's sunset time (Unix timestamp)
    
    // ═══ System Status (1 byte) ═══
    uint8_t  status;                 // System state (0=STANDBY, 1=TRACKING, 2=SLEEP, 3=CALIBRATING, 255=ERROR)
    
    // ═══ GPS Data (18 bytes) ═══
    float    latitude;               // GPS latitude (decimal degrees, ±90°)
    float    longitude;              // GPS longitude (decimal degrees, ±180°)
    uint8_t  gps_valid;              // GPS fix quality (0=NO_FIX, 1=FIX_2D, 2=FIX_3D, 3=DGPS)
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
    uint8_t  sd_card_status;         // SD card health (0=OK, 1=SLOW, 2=FULL, 3=FAILED)
    uint8_t  tracking_quality;       // Tracking error magnitude (0-180°)
    
} __attribute__((packed)) tracker_data_t;

/*
 * Initialize WiFi Station and Connect to Tracker
 * 
 * Sets up ESP32 as WiFi station and connects to tracker's AP.
 * Call once during display initialization.
 * 
 * What it does:
 *  1. Initializes WiFi stack (esp_netif, event loop)
 *  2. Configures station mode (SSID, password)
 *  3. Applies performance optimizations:
 *     - Maximum TX power (19.5 dBm)
 *     - Power-saving disabled (always-on for instant updates)
 *     - 20MHz bandwidth (better range)
 *  4. Connects to "SunflowerTracker" AP
 *  5. Waits for DHCP IP assignment (timeout: 15 seconds)
 *  6. Creates TCP client socket
 *  7. Connects to 192.168.4.1:8888
 * 
 * Returns:
 *  ESP_OK               - Connected successfully, ready to receive
 *  ESP_ERR_WIFI_TIMEOUT - WiFi connection timeout (tracker not found)
 *  ESP_ERR_WIFI_CONN    - WiFi connection failed (wrong password?)
 *  ESP_FAIL             - TCP connection failed
 * 
 * On failure:
 *  - Auto-retry in background (call wifi_client_reconnect())
 *  - Display shows "Connecting..." message
 *  - Keep calling until ESP_OK returned
 * 
 * Troubleshooting:
 *  - Timeout: Check tracker is powered on and WiFi started
 *  - Connection failed: Verify password matches tracker
 *  - TCP failed: Check tracker's TCP server is listening
 * 
 * Example:
 *  ```c
 *  esp_err_t ret = wifi_client_init();
 *  if (ret == ESP_OK) {
 *      ESP_LOGI(TAG, "Connected to tracker");
 *  } else {
 *      ESP_LOGE(TAG, "Connection failed, retrying...");
 *      vTaskDelay(pdMS_TO_TICKS(5000));
 *      wifi_client_reconnect();
 *  }
 *  ```
 */
esp_err_t wifi_client_init(void);

/*
 * Receive Tracking Data from Tracker
 * 
 * Receives one tracker_data_t packet (92 bytes) via TCP.
 * Call from display loop at 1-10 Hz.
 * 
 * Behavior:
 *  - Blocks until data received or timeout expires
 *  - If TCP disconnected, attempts reconnect automatically
 *  - Validates packet size (must be exactly 92 bytes)
 *  - Returns data in little-endian format (ESP32 native)
 * 
 * Params:
 *  data       - Pointer to tracker_data_t structure to fill
 *  timeout_ms - Maximum wait time in milliseconds (0 = no timeout)
 * 
 * Returns:
 *  ESP_OK               - Data received successfully
 *  ESP_ERR_TIMEOUT      - Timeout expired, no data received
 *  ESP_ERR_INVALID_SIZE - Received wrong number of bytes
 *  ESP_FAIL             - Connection lost, reconnecting
 * 
 * Timing:
 *  - Typical receive time: 1-10ms (fast)
 *  - Timeout: User-specified (recommend 5000ms for 1Hz updates)
 *  - Auto-reconnect delay: 2 seconds between attempts
 * 
 * Error recovery:
 *  - Timeout: Normal if tracker not sending yet (display shows "Waiting...")
 *  - Invalid size: Log warning, ignore packet (request retransmit)
 *  - Connection lost: Auto-reconnect in background
 * 
 * Example:
 *  ```c
 *  tracker_data_t data;
 *  esp_err_t ret = wifi_client_receive_data(&data, 5000);
 *  
 *  if (ret == ESP_OK) {
 *      printf("Panel: Az=%.1f° El=%.1f°\n", data.azimuth, data.elevation);
 *      printf("Battery: %.2fV (%.0f%%)\n", data.battery_voltage, data.battery_soc_percent);
 *      printf("GPS: %.6f, %.6f (%u sats)\n", data.latitude, data.longitude, data.gps_satellites);
 *  } else if (ret == ESP_ERR_TIMEOUT) {
 *      printf("Waiting for data...\n");
 *  } else {
 *      printf("Connection error, retrying...\n");
 *  }
 *  ```
 */
esp_err_t wifi_client_receive_data(tracker_data_t *data, uint32_t timeout_ms);

/*
 * Attempt Reconnection to Tracker
 * 
 * Manually triggers reconnection attempt.
 * Called automatically by wifi_client_receive_data() on disconnect.
 * Can also be called manually for explicit reconnect.
 * 
 * Returns:
 *  ESP_OK   - Reconnected successfully
 *  ESP_FAIL - Reconnection failed (will retry)
 * 
 * Note:
 *  - Non-blocking: Returns immediately
 *  - Retries happen in background WiFi task
 *  - Check wifi_client_is_connected() for status
 * 
 * Example:
 *  ```c
 *  if (!wifi_client_is_connected()) {
 *      ESP_LOGI(TAG, "Disconnected, reconnecting...");
 *      wifi_client_reconnect();
 *  }
 *  ```
 */
esp_err_t wifi_client_reconnect(void);

/*
 * Check if Connected to Tracker
 * 
 * Queries WiFi and TCP connection status.
 * 
 * Returns:
 *  true  - Connected to AP and TCP session active
 *  false - Not connected or TCP session closed
 * 
 * Use cases:
 *  - Display WiFi status icon
 *  - Decide whether to show "Connecting..." message
 *  - Trigger manual reconnect if needed
 * 
 * Example:
 *  ```c
 *  if (wifi_client_is_connected()) {
 *      lcd_show_wifi_icon(WIFI_CONNECTED);
 *  } else {
 *      lcd_show_wifi_icon(WIFI_DISCONNECTED);
 *      lcd_show_message("Connecting to tracker...");
 *  }
 *  ```
 */
bool wifi_client_is_connected(void);

/*
 * Get WiFi Signal Strength (RSSI)
 * 
 * Returns current WiFi signal strength in dBm.
 * 
 * Returns:
 *  -30 to -60 dBm  - Excellent signal (5 bars)
 *  -60 to -70 dBm  - Good signal (4 bars)
 *  -70 to -80 dBm  - Fair signal (3 bars)
 *  -80 to -90 dBm  - Weak signal (2 bars)
 *  < -90 dBm       - Very weak (1 bar, may drop)
 *  -128 dBm        - Not connected
 * 
 * Use cases:
 *  - Display signal strength bars on LCD
 *  - Warn user if signal too weak
 *  - Suggest repositioning display
 * 
 * Example:
 *  ```c
 *  int8_t rssi = wifi_client_get_signal_strength();
 *  
 *  if (rssi > -60) {
 *      lcd_show_wifi_bars(5);  // Excellent
 *  } else if (rssi > -70) {
 *      lcd_show_wifi_bars(4);  // Good
 *  } else if (rssi > -80) {
 *      lcd_show_wifi_bars(3);  // Fair
 *  } else if (rssi > -90) {
 *      lcd_show_wifi_bars(2);  // Weak
 *  } else {
 *      lcd_show_wifi_bars(1);  // Very weak
 *  }
 *  ```
 */
int8_t wifi_client_get_signal_strength(void);

/*
 * Get WiFi IP Address (assigned by DHCP)
 * 
 * Returns display's IP address as string.
 * 
 * Returns:
 *  "192.168.4.x" - IP address if connected
 *  "0.0.0.0"     - Not connected or no IP assigned
 * 
 * Use cases:
 *  - Display IP on LCD for diagnostics
 *  - Verify DHCP assignment
 *  - Troubleshooting network issues
 * 
 * Example:
 *  ```c
 *  const char *ip = wifi_client_get_ip_address();
 *  lcd_printf("IP: %s", ip);
 *  ```
 */
const char* wifi_client_get_ip_address(void);

/*
 * Get Connection Statistics
 * 
 * Fills structure with connection performance metrics.
 * 
 * Metrics:
 *  - Packets received
 *  - Receive errors
 *  - Connection uptime
 *  - Average signal strength
 *  - Reconnect count
 * 
 * Used for:
 *  - Troubleshooting connectivity
 *  - Performance monitoring
 *  - Display diagnostics screen
 * 
 * Example:
 *  ```c
 *  wifi_client_stats_t stats;
 *  wifi_client_get_stats(&stats);
 *  
 *  lcd_printf("Packets: %lu", stats.rx_packets);
 *  lcd_printf("Errors: %lu", stats.rx_errors);
 *  lcd_printf("Uptime: %lu sec", stats.uptime_sec);
 *  lcd_printf("Reconnects: %u", stats.reconnect_count);
 *  ```
 */
typedef struct {
    uint32_t rx_packets;             // Total packets received
    uint32_t rx_errors;              // Receive errors
    uint32_t rx_timeouts;            // Timeout count
    uint32_t uptime_sec;             // Connected time (seconds)
    int8_t   avg_rssi;               // Average RSSI (dBm)
    uint16_t reconnect_count;        // Number of reconnections
} wifi_client_stats_t;

void wifi_client_get_stats(wifi_client_stats_t *stats);

#endif // WIFI_COMM_CLIENT_H