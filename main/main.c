/*
 * Sunflower - LCD Display Unit (Client)
 * ESP32 WROOM + 3.5" ILI9486 TFT Display
 *
 * This ESP32 connects to the main Sunflower via WiFi
 * and displays real-time tracking data.
 * 
 * Hardware:
 *  - ESP32 WROOM or ESP32-CAM (without camera)
 *  - 3.5" ILI9486 TFT LCD (480x320, SPI)
 * 
 * What it does:
 *  - Connects to "SunflowerTracker" WiFi AP
 *  - Receives tracking data every second
 *  - Shows elevation, azimuth, battery, GPS location, sun position, stats
 *  - Displays real-time graphs
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_client.h"
#include "lcd.h"

#define TAG "LCD_MAIN"

// Pin definitions for ESP32 WROOM + ILI9486
#define LCD_MOSI  23
#define LCD_MISO  19    // Not used by display but needed for SPI bus
#define LCD_SCLK  18
#define LCD_CS    5
#define LCD_DC    21
#define LCD_RST   4
#define LCD_BL    22

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          SUNFLOWER LCD DISPLAY CLIENT                      ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "");
    
    // ═══════════════════════════════════════════════════════════════
    // STEP 1: Initialize NVS (Required for WiFi)
    // ═══════════════════════════════════════════════════════════════
    ESP_LOGI(TAG, "Initializing NVS flash storage...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS initialized");
    ESP_LOGI(TAG, "");
    
    // ═══════════════════════════════════════════════════════════════
    // STEP 2: Initialize LCD Display
    // ═══════════════════════════════════════════════════════════════
    ESP_LOGI(TAG, "Initializing LCD display...");
    lcd_config_t lcd_cfg = {
        .mosi_pin = LCD_MOSI,
        .miso_pin = LCD_MISO,
        .sclk_pin = LCD_SCLK,
        .cs_pin = LCD_CS,
        .dc_pin = LCD_DC,
        .rst_pin = LCD_RST,
        .backlight_pin = LCD_BL
    };
    
    ret = lcd_client_init(&lcd_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ LCD initialization failed!");
        ESP_LOGE(TAG, "Check wiring and pin assignments");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "✓ LCD initialized");
    ESP_LOGI(TAG, "  Display: ILI9486 480x320");
    ESP_LOGI(TAG, "  Interface: SPI @ 10 MHz");
    ESP_LOGI(TAG, "");
    
    // Set brightness and show splash screen
    lcd_client_set_brightness(80);  // 80% brightness
    lcd_client_show_init_screen("Initializing WiFi");
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    // ═══════════════════════════════════════════════════════════════
    // STEP 3: Connect to Tracker WiFi
    // ═══════════════════════════════════════════════════════════════
    ESP_LOGI(TAG, "Connecting to tracker WiFi...");
    lcd_client_show_init_screen("Connecting to WiFi");
    
    ret = wifi_client_init();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ WiFi connection failed");
        lcd_client_show_error("WiFi Connection Failed");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. Tracker not powered on");
        ESP_LOGE(TAG, "  2. Tracker WiFi not started");
        ESP_LOGE(TAG, "  3. Wrong SSID/password");
        ESP_LOGE(TAG, "  4. Out of range");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Device will keep retrying in background...");
        
        // Keep showing error but don't block - reconnect happens automatically
        while (!wifi_client_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            lcd_client_show_init_screen("Reconnecting");
        }
    }
    
    ESP_LOGI(TAG, "✓ Connected to tracker");
    ESP_LOGI(TAG, "  SSID: SunflowerTracker");
    ESP_LOGI(TAG, "  IP: %s", wifi_client_get_ip_address());
    ESP_LOGI(TAG, "  Signal: %d dBm", wifi_client_get_signal_strength());
    ESP_LOGI(TAG, "");
    
    lcd_client_show_init_screen("Connected! Loading data");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // ═══════════════════════════════════════════════════════════════
    // STEP 4: Main Display Loop
    // ═══════════════════════════════════════════════════════════════
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          ENTERING MAIN DISPLAY LOOP                        ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    uint32_t loop_count = 0;
    uint32_t no_data_counter = 0;
    uint32_t last_log_time = 0;
    bool first_data_received = false;  // Track if we've ever received data
    
    while (1) {
        tracker_data_t data;
        memset(&data, 0, sizeof(data));
        
        // Check connection status
        if (!wifi_client_is_connected()) {
            ESP_LOGW(TAG, "⚠ WiFi disconnected");
            lcd_client_show_init_screen("Reconnecting to WiFi");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Receive tracking data (2 second timeout)
        esp_err_t rx_ret = wifi_client_receive_data(&data, 2000);
        
        if (rx_ret == ESP_OK) {
            // ═══ SUCCESS - Update Display ═══
            no_data_counter = 0;
            first_data_received = true;  // Mark that we've received data
            
            // Update dashboard with new data
            lcd_client_display_dashboard(&data);
            
            loop_count++;
            
            // Periodic logging (every 10 seconds)
            uint32_t now = esp_log_timestamp() / 1000;
            if ((now - last_log_time) >= 10) {
                // Get fresh RSSI from client side
                int8_t client_rssi = wifi_client_get_signal_strength();
                
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
                ESP_LOGI(TAG, "Display Update #%lu", loop_count);
                ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
                ESP_LOGI(TAG, "Panel Position:");
                ESP_LOGI(TAG, "  Elevation: %.1f° (Δ%.2f°)", data.elevation, data.delta_elevation);
                ESP_LOGI(TAG, "  Azimuth:   %.1f° (Δ%.2f°)", data.azimuth, data.delta_azimuth);
                ESP_LOGI(TAG, "Sun Position:");
                ESP_LOGI(TAG, "  Elevation: %.1f°", data.sun_elevation);
                ESP_LOGI(TAG, "  Azimuth:   %.1f°", data.sun_azimuth);
                ESP_LOGI(TAG, "Battery:");
                ESP_LOGI(TAG, "  Voltage:   %.2fV", data.battery_voltage);
                ESP_LOGI(TAG, "  SoC:       %.0f%% (Level %u)", data.battery_soc_percent, data.battery_soc);
                ESP_LOGI(TAG, "  Charging:  %s", data.battery_charging ? "YES" : "NO");
                ESP_LOGI(TAG, "GPS:");
                ESP_LOGI(TAG, "  Position:  %.6f, %.6f", data.latitude, data.longitude);
                ESP_LOGI(TAG, "  Satellites: %u", data.gps_satellites);
                ESP_LOGI(TAG, "  Valid:     %s", data.gps_valid ? "YES" : "NO");
                ESP_LOGI(TAG, "  Fix Age:   %lu sec", data.last_gps_fix_age_sec);
                ESP_LOGI(TAG, "System:");
                ESP_LOGI(TAG, "  Status:    %u", data.status);
                ESP_LOGI(TAG, "  Quality:   %u°", data.tracking_quality);
                ESP_LOGI(TAG, "  WiFi RSSI (Client): %d dBm", client_rssi);  // Client's view of signal
                ESP_LOGI(TAG, "  WiFi RSSI (Tracker): %d dBm", data.wifi_rssi);  // Tracker's view (from received packet)
                ESP_LOGI(TAG, "Statistics:");
                ESP_LOGI(TAG, "  Moves Today: %lu", data.moves_today);
                ESP_LOGI(TAG, "  Total Moves: %lu", data.total_moves);
                ESP_LOGI(TAG, "  Uptime:      %u hours", data.uptime_hours);
                ESP_LOGI(TAG, "─────────────────────────────────────────────────────");
                ESP_LOGI(TAG, "");
                
                last_log_time = now;
            }
            
        } else if (rx_ret == ESP_ERR_TIMEOUT) {
            // ═══ TIMEOUT - No Data Yet ═══
            no_data_counter++;
            
            if (no_data_counter == 1) {
                ESP_LOGD(TAG, "Waiting for data (timeout)");
            } else if (no_data_counter == 3) {
                ESP_LOGW(TAG, "⚠ No data for %lu seconds", no_data_counter * 2);
            } else if (no_data_counter >= 10 && first_data_received) {
                // Only show error if we've successfully received data before
                // This prevents showing error during initial startup
                ESP_LOGW(TAG, "⚠ No data for %lu seconds - displaying error", no_data_counter * 2);
                lcd_client_show_error("Waiting for tracker data");
            }
            // If first_data_received is false, we're still at startup - keep showing init screen
            
        } else if (rx_ret == ESP_ERR_INVALID_SIZE) {
            // ═══ INVALID PACKET ═══
            ESP_LOGW(TAG, "⚠ Received invalid packet size");
            no_data_counter++;
            
        } else {
            // ═══ CONNECTION ERROR ═══
            ESP_LOGE(TAG, "✗ Receive error: %s", esp_err_to_name(rx_ret));
            
            // Only show error screen if we've received data before
            if (first_data_received) {
                lcd_client_show_error("Connection error");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // ═══ Print Connection Statistics (Every 60 Seconds) ═══
        static uint32_t last_stats_time = 0;
        uint32_t now = esp_log_timestamp() / 1000;
        if ((now - last_stats_time) >= 60) {
            wifi_client_stats_t stats;
            wifi_client_get_stats(&stats);
            
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║          CONNECTION STATISTICS                             ║");
            ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "Packets Received:   %lu", stats.rx_packets);
            ESP_LOGI(TAG, "Receive Errors:     %lu", stats.rx_errors);
            ESP_LOGI(TAG, "Receive Timeouts:   %lu", stats.rx_timeouts);
            ESP_LOGI(TAG, "Connection Uptime:  %lu sec", stats.uptime_sec);
            ESP_LOGI(TAG, "Average RSSI:       %d dBm", stats.avg_rssi);
            ESP_LOGI(TAG, "Reconnect Count:    %u", stats.reconnect_count);
            ESP_LOGI(TAG, "");
            
            last_stats_time = now;
        }
        
        // Brief delay to prevent tight loop (display updates at ~1 Hz anyway)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}