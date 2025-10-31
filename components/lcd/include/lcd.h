/*
 * LCD Display Client for Sunflower Solar Tracker
 * 
 * Professional ILI9486 dashboard with WiFi data reception.
 * 
 * Dashboard Layout (480x320):
 * ┌─────────────────────────────────────────────────────────────┐
 * │ [Logo] SUNFLOWER TRACKER              12/25/2024 14:30:15   │ 30px
 * ├─────────────────────────────────────────────────────────────┤
 * │ ELEVATION  │  AZIMUTH   │  BATTERY                          │ 70px
 * │  45.2°     │  180.5°    │  12.6V  85%                       │
 * │  Δ:2.1°    │  Δ:5.3°    │  [████████] CHG                   │
 * ├─────────────────────────────────────────────────────────────┤
 * │ SUN POSITION│ SYS STATUS │ GPS / STATS                      │ 70px
 * │ El:47.3°    │ TRACKING   │ 45.5234,-122.6762                │
 * │ Az:182.1°   │ Err:2.1°   │ Today:23 Total:1547              │
 * │ R:06:45     │ SD:OK      │ Up:12h WiFi:-65dBm               │
 * ├─────────────────────────────────────────────────────────────┤
 * │ Battery Voltage History (Last 100 Readings)                 │ 150px
 * │ [Graph showing voltage trend over time]                     │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * Features:
 * - Real-time tracking data display
 * - Battery voltage graph with 100-point history
 * - Status indicators (tracking, GPS fix, SD card)
 * - WiFi signal strength display
 * - Sunrise/sunset times
 * - Move statistics
 * - Professional color scheme (golden/teal/sage theme)
 * 
 * Hardware:
 * - Display: ILI9486 3.5" 480x320 TFT (landscape orientation)
 * - Interface: SPI (10 MHz)
 * - Backlight: PWM controlled (optional)
 * - Font: Built-in 5x7 bitmap font (scalable)
 * 
 * Pin Configuration (ESP32-WROOM):
 * - MOSI: GPIO23 (SPI data to display)
 * - MISO: GPIO19 (not used for display, but needed for SPI bus)
 * - SCLK: GPIO18 (SPI clock)
 * - CS:   GPIO5  (chip select)
 * - DC:   GPIO21 (data/command select)
 * - RST:  GPIO4  (hardware reset)
 * - BL:   GPIO22 (backlight PWM, optional)
 * 
 * Integration:
 * 1. Call lcd_client_init() during startup
 * 2. Call lcd_client_show_init_screen() while connecting to WiFi
 * 3. Call lcd_client_display_dashboard() at 1 Hz when data received
 * 4. Call lcd_client_show_error() if connection lost
 * 
 * Performance:
 * - Full screen refresh: ~200ms (acceptable for 1 Hz updates)
 * - Partial updates possible (not implemented for simplicity)
 * - No flicker (uses double-buffering technique in critical areas)
 * 
 * Memory:
 * - Stack: ~2KB (deep call chains during drawing)
 * - Heap: ~5KB (line buffers, font rendering)
 * - Flash: ~15KB (code + font data)
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "wifi_client.h"  // Import tracker_data_t

// ═══════════════════════════════════════════════════════════════
// DISPLAY CONFIGURATION
// ═══════════════════════════════════════════════════════════════

// Display dimensions (ILI9486 landscape mode)
#define LCD_WIDTH   480
#define LCD_HEIGHT  320

// Layout dimensions
#define HEADER_HEIGHT       30    // Top bar with logo and time
#define PANEL_HEIGHT        70    // Height of data panels
#define PANEL_SPACING       5     // Gap between panels
#define GRAPH_Y_START       185   // Y position of voltage graph
#define GRAPH_HEIGHT        135   // Height of voltage graph

// ═══════════════════════════════════════════════════════════════
// COLOR PALETTE - PROFESSIONAL SUNFLOWER THEME
// ═══════════════════════════════════════════════════════════════
// RGB565 format: RRRRR GGGGGG BBBBB

// Neutral colors (backgrounds, borders, text)
#define LCD_BLACK       0x0000  // Pure black
#define LCD_WHITE       0xFFFF  // Pure white
#define LCD_CHARCOAL    0x18C3  // Dark blue-gray (header background)
#define LCD_STEEL       0x3186  // Medium blue-gray (secondary backgrounds)
#define LCD_SILVER      0xC618  // Light gray (labels)
#define LCD_SLATE       0x4A49  // Slate blue (borders)
#define LCD_DARKBLUE    0x2945  // Dark navy blue (panel backgrounds)

// Warm colors (primary values, sunlight theme)
#define LCD_GOLDEN      0xFEA0  // Golden yellow (primary values)
#define LCD_AMBER       0xFD20  // Deep amber (elevation values)
#define LCD_SUNGLOW     0xFFE0  // Bright yellow (azimuth values)
#define LCD_ORANGE      0xFD20  // Orange (standby status)

// Cool colors (secondary values, nature theme)
#define LCD_TEAL        0x07FF  // Cyan-blue (delta values)
#define LCD_MINT        0x87F0  // Light teal (GPS data)
#define LCD_SAGE        0x07E0  // Green (good status, battery)

// Alert colors (status indicators)
#define LCD_CORAL       0xFBEA  // Light red (warnings)
#define LCD_CRIMSON     0xF800  // Pure red (errors, critical)

// ═══════════════════════════════════════════════════════════════
// LCD HARDWARE CONFIGURATION
// ═══════════════════════════════════════════════════════════════

/*
 * LCD Pin Configuration
 * 
 * Standard SPI + control pins for ILI9486.
 * 
 * Fields:
 *  mosi_pin      - SPI Master Out Slave In (data to display)
 *  miso_pin      - SPI Master In Slave Out (not used by display, but required for SPI bus)
 *  sclk_pin      - SPI clock
 *  cs_pin        - Chip select (active low)
 *  dc_pin        - Data/Command select (0=command, 1=data)
 *  rst_pin       - Hardware reset (active low)
 *  backlight_pin - PWM backlight control (-1 if always-on or manually controlled)
 * 
 * Example:
 *  lcd_config_t config = {
 *      .mosi_pin = 23,
 *      .miso_pin = 19,
 *      .sclk_pin = 18,
 *      .cs_pin = 5,
 *      .dc_pin = 21,
 *      .rst_pin = 4,
 *      .backlight_pin = 22
 *  };
 */
typedef struct {
    int mosi_pin;         // GPIO23 (standard)
    int miso_pin;         // GPIO19 (standard, not used by display)
    int sclk_pin;         // GPIO18 (standard)
    int cs_pin;           // GPIO5 (user choice)
    int dc_pin;           // GPIO21 (user choice)
    int rst_pin;          // GPIO4 (user choice)
    int backlight_pin;    // GPIO22 (user choice, -1 to disable)
} lcd_config_t;

// ═══════════════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════════════

/*
 * Initialize LCD Display
 * 
 * Complete initialization sequence:
 * 1. Configure GPIO pins (DC, RST, backlight)
 * 2. Initialize SPI bus and device
 * 3. Hardware reset display
 * 4. Send ILI9486 initialization commands
 * 5. Set display orientation to landscape
 * 6. Clear screen to black
 * 7. Configure backlight PWM (if enabled)
 * 
 * Parameters:
 *  config - Pin configuration (see lcd_config_t)
 * 
 * Returns:
 *  ESP_OK   - Display initialized and ready
 *  ESP_FAIL - Initialization failed (check logs)
 * 
 * Error conditions:
 *  - SPI bus initialization failed
 *  - Device add failed
 *  - Backlight PWM setup failed (non-fatal)
 * 
 * Timing:
 *  - Total init time: ~200ms (reset delays + command sequence)
 *  - Non-blocking after return
 * 
 * Notes:
 *  - Call this once during app initialization
 *  - Must be called before any drawing functions
 *  - Display will show black screen after init
 *  - Safe to call multiple times (will re-init)
 * 
 * Example:
 *  ```c
 *  lcd_config_t config = {
 *      .mosi_pin = 23, .miso_pin = 19, .sclk_pin = 18,
 *      .cs_pin = 5, .dc_pin = 21, .rst_pin = 4,
 *      .backlight_pin = 22
 *  };
 *  
 *  esp_err_t ret = lcd_client_init(&config);
 *  if (ret != ESP_OK) {
 *      ESP_LOGE(TAG, "LCD init failed");
 *      return;
 *  }
 *  ```
 */
esp_err_t lcd_client_init(const lcd_config_t *config);

/*
 * Set Display Brightness
 * 
 * Controls backlight via PWM (if backlight_pin configured).
 * Allows dimming for nighttime use or power saving.
 * 
 * Parameters:
 *  brightness - Brightness level (0-100%)
 *               0   = Backlight off (screen black but still powered)
 *               50  = Half brightness (good for night)
 *               100 = Full brightness (daylight use)
 * 
 * Notes:
 *  - Has no effect if backlight_pin = -1
 *  - PWM frequency: 5 kHz (flicker-free)
 *  - Changes take effect immediately (<1ms)
 *  - Does not affect display power consumption significantly
 *    (backlight is separate from LCD driver)
 * 
 * Example:
 *  ```c
 *  // Dim display at night
 *  if (hour >= 22 || hour < 6) {
 *      lcd_client_set_brightness(30);  // 30% for night
 *  } else {
 *      lcd_client_set_brightness(100); // Full brightness day
 *  }
 *  ```
 */
void lcd_client_set_brightness(uint8_t brightness);

/*
 * Show Initialization Screen
 * 
 * Displays splash screen while connecting to tracker.
 * Shows logo, title, and animated connection status.
 * 
 * Screen layout:
 * ┌─────────────────────────────────────────┐
 * │                                         │
 * │         [Sunflower Logo]                │
 * │                                         │
 * │      SUNFLOWER TRACKER                  │
 * │                                         │
 * │      Connecting to tracker...           │
 * │                                         │
 * └─────────────────────────────────────────┘
 * 
 * Parameters:
 *  message - Status message to display
 *            Examples: "Connecting to WiFi..."
 *                     "Waiting for tracker..."
 *                     "Receiving data..."
 * 
 * Features:
 *  - Animated dots (cycles 0-3 dots for activity indication)
 *  - Centered logo and text
 *  - Professional color scheme
 * 
 * Usage:
 *  Call repeatedly in connection loop (every 500ms):
 *  ```c
 *  while (!wifi_client_is_connected()) {
 *      lcd_client_show_init_screen("Connecting to WiFi");
 *      vTaskDelay(pdMS_TO_TICKS(500));
 *  }
 *  
 *  while (!tcp_connected) {
 *      lcd_client_show_init_screen("Waiting for tracker");
 *      vTaskDelay(pdMS_TO_TICKS(500));
 *  }
 *  ```
 * 
 * Performance:
 *  - Refresh time: ~100ms (clears and redraws entire screen)
 *  - Acceptable for splash screen (not used during normal operation)
 */
void lcd_client_show_init_screen(const char *message);

/*
 * Display Full Dashboard
 * 
 * Renders complete tracker dashboard with all telemetry.
 * This is the main display function - call at 1 Hz when connected.
 * 
 * What it displays:
 * - Header: Logo, title, current time
 * - Panel angles: Elevation, azimuth, deltas
 * - Battery: Voltage, SoC%, bar graph, charging indicator
 * - Sun position: Calculated elevation/azimuth, sunrise/sunset
 * - System status: TRACKING/SLEEP/STANDBY/ERROR with color coding
 * - GPS: Lat/lon, satellite count, fix age
 * - Statistics: Moves today/total, uptime, WiFi RSSI
 * - Voltage graph: 100-point history with auto-scaling
 * 
 * Parameters:
 *  data - Complete tracker telemetry (received via WiFi)
 * 
 * Performance:
 *  - Full refresh time: ~200ms (acceptable for 1 Hz updates)
 *  - No flicker (uses efficient drawing primitives)
 *  - Voltage graph uses line-drawing optimization
 * 
 * Data validation:
 *  - Displays "NO FIX" if GPS invalid
 *  - Shows "ERROR" status in red if system fault
 *  - Clamps values to reasonable ranges (prevents display corruption)
 * 
 * Color coding:
 *  - Green/sage: Normal operation, good status
 *  - Yellow/amber: Tracking values, sun position
 *  - Teal/mint: GPS data, statistics
 *  - Orange: Standby mode
 *  - Red: Errors, critical battery
 * 
 * Usage:
 *  ```c
 *  tracker_data_t data;
 *  while (1) {
 *      esp_err_t ret = wifi_client_receive_data(&data, 5000);
 *      if (ret == ESP_OK) {
 *          lcd_client_display_dashboard(&data);  // Update display
 *      } else if (ret == ESP_ERR_TIMEOUT) {
 *          // No data yet, keep showing old data or error
 *          lcd_client_show_error("Waiting for data");
 *      }
 *      vTaskDelay(pdMS_TO_TICKS(100));  // Don't hog CPU
 *  }
 *  ```
 * 
 * Notes:
 *  - Maintains internal state (voltage history buffer)
 *  - Each call adds one point to voltage graph
 *  - Graph auto-scales to show min/max range
 *  - Safe to call with invalid data (displays errors gracefully)
 */
void lcd_client_display_dashboard(const tracker_data_t *data);

/*
 * Show Connection Error Screen
 * 
 * Displays full-screen error when connection lost.
 * Bright red background grabs attention immediately.
 * 
 * Screen layout:
 * ┌─────────────────────────────────────────┐
 * │                                         │
 * │         CONNECTION LOST                 │
 * │                                         │
 * │      [Custom error message]             │
 * │                                         │
 * └─────────────────────────────────────────┘
 * 
 * Parameters:
 *  message - Specific error message
 *            Examples: "WiFi disconnected"
 *                     "Tracker not responding"
 *                     "TCP timeout"
 * 
 * Features:
 *  - Bold white text on red background
 *  - Centered message
 *  - High visibility
 * 
 * Usage:
 *  ```c
 *  if (!wifi_client_is_connected()) {
 *      lcd_client_show_error("WiFi disconnected");
 *      vTaskDelay(pdMS_TO_TICKS(1000));
 *      wifi_client_reconnect();
 *  }
 *  
 *  if (receive_timeout_count > 10) {
 *      lcd_client_show_error("Tracker not responding");
 *  }
 *  ```
 * 
 * Recovery:
 *  - Error screen remains until:
 *    - Connection restored → call lcd_client_display_dashboard()
 *    - User resets device
 *  - Consider alternating with init screen for active reconnection
 */
void lcd_client_show_error(const char *message);

/*
 * Get Battery Voltage History
 * 
 * Returns internal voltage history buffer for external analysis.
 * Useful for diagnostics or alternative visualization.
 * 
 * Returns:
 *  Pointer to 100-element array of ADC readings (uint16_t)
 *  - Readings are raw ADC values (0-4095)
 *  - Ordered oldest to newest
 *  - Zero values indicate no data yet
 * 
 * Example:
 *  ```c
 *  const uint16_t *history = lcd_client_get_voltage_history();
 *  for (int i = 0; i < 100; i++) {
 *      if (history[i] > 0) {
 *          float voltage = (history[i] / 4095.0f) * 15.0f;
 *          printf("%.2fV ", voltage);
 *      }
 *  }
 *  ```
 */
const uint16_t* lcd_client_get_voltage_history(void);

/*
 * Get Voltage History Index
 * 
 * Returns current position in circular voltage buffer.
 * Useful for determining newest data point.
 * 
 * Returns:
 *  Index of next write position (0-99)
 */
int lcd_client_get_history_index(void);

#endif // LCD_H