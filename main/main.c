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
 *  - Connects to "Sunflower" WiFi AP
 *  - Receives tracking data every second
 *  - Shows elevation, azimuth, battery, GPS location
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

#define TAG "LCD_DISPLAY"

void app_main(void)
{
    ESP_LOGI(TAG, "=== SUNFLOWER LCD DISPLAY ===");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
    
    // Initialize flash storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Flash storage OK");
    
    // Initialize LCD
    ESP_LOGI(TAG, "Starting LCD...");
    ret = lcd_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD failed!");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Show splash
    lcd_show_splash("Initializing...");
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    // Show init screen
    const char *init_steps[] = {
        "LCD Display",
        "WiFi Connection",
        "Tracker Link"
    };
    bool init_results[3] = {true, false, false};
    
    lcd_show_init_screen(init_steps, init_results, 3);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // Connect to tracker WiFi
    ESP_LOGI(TAG, "Connecting to tracker...");
    ret = wifi_client_init();
    init_results[1] = (ret == ESP_OK);
    lcd_show_init_screen(init_steps, init_results, 3);
    
    if (ret != ESP_OK) {
        lcd_show_error("WiFi Connection Failed");
        ESP_LOGE(TAG, "Failed to connect!");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test data reception
    ESP_LOGI(TAG, "Testing data reception...");
    tracker_data_t test_data;
    ret = wifi_client_receive_data(&test_data, 5000);
    init_results[2] = (ret == ESP_OK);
    lcd_show_init_screen(init_steps, init_results, 3);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd_draw_string(120, 280, "Initialization Complete!", TFT_SAGE, TFT_BLACK, 2);
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    // Setup initial display
    lcd_display_data_t display_data = {
        .elevation = 0.0f,
        .azimuth = 0.0f,
        .delta_elevation = 0.0f,
        .delta_azimuth = 0.0f,
        .battery_adc = 2500,
        .battery_voltage = 12.0f,
        .timestamp = 0,
        .tracking_status = 0,
        .latitude = 0.0f,
        .longitude = 0.0f,
        .gps_valid = false
    };
    
    // Draw dashboard
    lcd_draw_dashboard(&display_data);
    ESP_LOGI(TAG, "Dashboard ready");
    
    // Main loop
    ESP_LOGI(TAG, "Starting main loop...");
    uint32_t last_update = 0;
    
    while (1) {
        // Receive data from tracker
        tracker_data_t rx_data;
        ret = wifi_client_receive_data(&rx_data, 2000);
        
        if (ret == ESP_OK) {
            // Update display data
            display_data.elevation = rx_data.elevation;
            display_data.azimuth = rx_data.azimuth;
            display_data.delta_elevation = rx_data.delta_elevation;
            display_data.delta_azimuth = rx_data.delta_azimuth;
            display_data.battery_adc = rx_data.battery_adc;
            display_data.battery_voltage = rx_data.battery_voltage;
            display_data.timestamp = rx_data.timestamp;
            display_data.tracking_status = rx_data.status;
            display_data.latitude = rx_data.latitude;
            display_data.longitude = rx_data.longitude;
            display_data.gps_valid = (rx_data.gps_valid == 1);
            
            // Update display
            lcd_update_display(&display_data);
            
            last_update = esp_log_timestamp();
            
            ESP_LOGI(TAG, "Updated - El: %.1f° Az: %.1f° Batt: %.2fV Status: %d",
                     display_data.elevation, display_data.azimuth,
                     display_data.battery_voltage, display_data.tracking_status);
        } else if (ret == ESP_ERR_TIMEOUT) {
            // No data timeout
            if (esp_log_timestamp() - last_update > 10000) {
                ESP_LOGW(TAG, "No data for 10s");
                lcd_show_error("Connection Lost");
            }
        } else {
            ESP_LOGE(TAG, "Receive error, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}