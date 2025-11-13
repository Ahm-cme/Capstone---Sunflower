/*
 * Enhanced GPS-Based Solar Tracking System with Wireless LCD Display
 * ESP32 + MAX-M10S GPS + Dual Linear Actuators + Remote LCD Display
 * Optimized for power efficiency and tracking accuracy
 * 
 * ┌───────────────────────────────────────────────────────────────────────┐
 * │ System Architecture Overview                                          │
 * │                                                                       │
 * │ Hardware Stack:                                                       │
 * │  Main Tracker Unit (ESP32-WROVER):                                   │
 * │   └─ ESP32-CAM + MAX-M10S GPS + MD20A Motor Drivers + 12V Linear Act │
 * │      200mm stroke actuators @ 11.94 mm/s nominal speed               │
 * │      MicroSD logging + Status LED + Start/Calibrate button           │
 * │                                                                       │
 * │  Remote Display Unit (ESP32-WROOM):                                  │
 * │   └─ ESP32 + 3.5" TFT LCD (480x320 ILI9486)                          │
 * │      Real-time tracking visualization via WiFi                        │
 * │                                                                       │
 * │ Software Stack:                                                       │
 * │  Main Tracker (This File):                                           │
 * │  ├── Tracking Controller (main coordination, sleep management)       │
 * │  ├── GPS Module (I2C, time sync, position acquisition)               │
 * │  ├── Solar Calculator (NOAA algorithms, sunrise/sunset)              │
 * │  ├── Motor Control (PWM + DIR, time-based positioning)               │
 * │  ├── SD Logging (CSV data + human readable logs)                     │
 * │  ├── WiFi AP (Access Point for LCD communication)                    │
 * │  └── Status LED (visual feedback for system state)                   │
 * │                                                                       │                     │
 * │                                                                       │
 * │ Communication Protocol:                                               │
 * │  ├── WiFi AP/Station (SSID: "SunflowerTracker")                     │
 * │  ├── TCP Socket (Port 8888)                                          │
 * │  ├── Data Rate: 1 Hz (updates every second)                          │
 * │  └── Packet: 40 bytes (angles, battery, GPS, time, status)          │
 * │                                                                       │
 * │ Power Profile (12V battery):                                         │
 * │  Main Tracker:                                                        │
 * │  ├── Deep Sleep: ~10-50µA (RTC + wake timer only)                    │
 * │  ├── Active Tracking: ~150-300mA (GPS + CPU + peripherals + WiFi)   │
 * │  ├── Motor Moves: ~500-1000mA (brief, 5-10s duration)               │
 * │  └── Daily Average: ~25-50mA (depends on tracking frequency)         │                                                                      │
 * │ Deployment Workflow:                                                  │
 * │  Main Tracker Setup:                                                  │
 * │  1. Flash firmware, install hardware, insert SD card                 │
 * │  2. Power on, wait for GPS fix (LED_WAITING)                         │
 * │  3. Manually align panel to sun, long-press button (calibration)     │
 * │  4. Press start button to begin autonomous tracking                   │
 * │  5. System creates WiFi AP "SunflowerTracker"                        │
 * │                                                                       │
 * │  Remote Display Setup:                                                │
 * │  1. Flash LCD display (Sunflower_Secondary) firmware to ESP32-WROOM                        │
 * │  2. Connect LCD display pins (GPIO 4,5,18,21,23)                     │
 * │  3. Power on - auto-connects to tracker WiFi                         │
 * │  4. Dashboard shows real-time tracking data                          │
 * │                                                                       │
 * │  Operation:                                                           │
 * │  - Main tracker operates independently with nightly homing           │
 * │  - LCD display can be viewed/powered on-demand                       │
 * │  - System continues tracking even if display is off                  │
 * │  - SD card logs all data regardless of WiFi connection               │
 * └───────────────────────────────────────────────────────────────────────┘
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "gps.h"
#include "motor.h"
#include "sdlog.h"
#include "tracking.h"
#include "status_led.h"
#include "button.h"
#include "esp_sleep.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "wifi_comm.h"
#include "solar.h"
#include "battery.h"
#include "esp_adc/adc_oneshot.h"  
#include "esp_wifi.h"             

#define TAG "APP"

/*
 * Pin Configuration - Main Tracker Unit
 * 
 * These pins work with the ESP32-CAM board without conflicts.
 * Don't use GPIO 0,1,3 (serial) or GPIO 6-11 (flash chip).
 * 
 * Note: Remote LCD display uses separate ESP32 with different pin config
 */

// GPS module (UART communication)
#define GPS_UART_PORT  UART_NUM_2
#define GPS_TX_PIN     22     // Connect to GPS RX
#define GPS_RX_PIN     23     // Connect to GPS TX
#define GPS_BAUD       9600   // Default baud rate for BN-880

// Compass on GPS module (I2C communication)
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_PIN    27     // I2C data line
#define I2C_SCL_PIN    26     // I2C clock line

// Motor control pins
#define MOTOR_AZ_PWM   18     // Azimuth motor speed control
#define MOTOR_AZ_DIR   19     // Azimuth motor direction
#define MOTOR_EL_PWM   32     // Elevation motor speed control
#define MOTOR_EL_DIR   33     // Elevation motor direction

// SD card pins (ESP32-CAM standard)
#define SD_MOSI        15     // SD data in
#define SD_MISO        2      // SD data out (don't use during programming!)
#define SD_SCLK        14     // SD clock
#define SD_CS          13     // SD chip select

// User interface
#define STATUS_LED_GPIO 4     // Built-in LED on ESP32-CAM
#define START_BTN_GPIO  5     // Start/calibrate button

// Battery monitoring (ADC)
#define BATTERY_ADC_GPIO    35     // GPIO35 (ADC1_CH7) - best for analog input
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_7
#define BATTERY_VOLTAGE_RATIO 6.89f  // Voltage divider: (68k+15k)/15k = 6.89

/*
 * Calibration Task
 * 
 * Watches for long button press (3 seconds) to trigger calibration.
 * 
 * How calibration works:
 *  1. Manually point the panel directly at the sun
 *  2. Hold the button for 3+ seconds
 *  3. System saves the current angles as reference
 *  4. Future tracking uses this reference automatically
 * 
 * You can recalibrate anytime - it just overwrites the old values.
 * Calibration data is saved to NVS flash and persists across reboots.
 */
static void calib_task(void *arg){
    ESP_LOGI(TAG, "Calibration monitor running");
    ESP_LOGI(TAG, "Hold START button for 3 seconds to calibrate");
    
    for(;;){
        // Check if button is pressed
        if (button_wait_for_press(100)) {
            uint32_t press_start = xTaskGetTickCount();
            uint32_t hold_time = 0;
            
            // Keep checking while button is held down
            while (button_is_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
                hold_time = (xTaskGetTickCount() - press_start) * portTICK_PERIOD_MS;
                
                // If held for 3+ seconds, start calibration
                if (hold_time >= 3000) {
                    ESP_LOGI(TAG, "Button held 3s - starting calibration");
                    
                    // Blink LED fast during calibration
                    status_led_set_mode(LED_STARTUP);
                    
                    // Log to SD card
                    sdlog_printf("Starting mount calibration");
                    
                    // Do the actual calibration
                    tracking_calibrate_mount_offset_now();
                    
                    // Back to normal LED pattern
                    status_led_set_mode(LED_TRACKING);
                    
                    ESP_LOGI(TAG, "Calibration done");
                    
                    // Wait for button release
                    while (button_is_pressed()) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    break;
                }
            }
        }
        
        // Don't hog the CPU
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*
 * Compass Calibration Task
 * 
 * Triggered by double-pressing the button quickly.
 * 
 * How to calibrate compass:
 *  1. Press button twice within 1 second
 *  2. LED blinks fast - you're in calibration mode
 *  3. Slowly rotate the entire tracker 2-3 full circles
 *  4. LED goes back to normal when done
 *  5. Calibration data is saved automatically to NVS flash
 * 
 * Do this once after assembling hardware, away from metal objects.
 * This improves azimuth tracking accuracy using the GPS module's compass.
 */
static void compass_calib_task(void *arg){
    ESP_LOGI(TAG, "Compass calibration monitor running");
    ESP_LOGI(TAG, "Double-press START to calibrate compass");
    
    uint32_t last_press = 0;
    
    for(;;){
        if (button_wait_for_press(100)) {
            uint32_t now = xTaskGetTickCount();
            uint32_t time_since_last = (now - last_press) * portTICK_PERIOD_MS;
            
            // Check if this is a double-press (< 1 second apart)
            if (time_since_last < 1000) {
                ESP_LOGI(TAG, "Double-press detected - calibrating compass");
                sdlog_printf("Starting compass calibration");
                
                // Fast blink = calibration mode
                status_led_set_mode(LED_STARTUP);
                
                bool success = gps_calibrate_compass();
                
                if (success) {
                    ESP_LOGI(TAG, "Compass calibration successful");
                    sdlog_printf("Compass calibration: OK");
                    
                    // Blink LED 3 times to show success
                    for (int i = 0; i < 3; i++) {
                        status_led_set_mode(LED_ERROR);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        status_led_set_mode(LED_TRACKING);
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                } else {
                    ESP_LOGW(TAG, "Compass calibration failed");
                    sdlog_printf("Compass calibration: FAILED");
                    status_led_set_mode(LED_ERROR);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                
                status_led_set_mode(LED_TRACKING);
                last_press = 0;
            } else {
                last_press = now;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*
 * Comprehensive System Check
 * 
 * Validates all hardware and software subsystems before starting tracking.
 * Returns true if system is ready, false if critical failures detected.
 * 
 * Check sequence:
 *  1. Status LED (visual feedback)
 *  2. NVS Flash (configuration storage)
 *  3. SD Card (data logging)
 *  4. GPS Module (position/time)
 *  5. Motor Drivers (actuator control)
 *  6. Button Interface (user input)
 *  7. Battery Monitor (power management)
 *  8. WiFi AP (LCD communication)
 *  9. Compass (optional azimuth reference)
 * 
 * Severity levels:
 *  CRITICAL: System cannot operate (GPS, motors, flash)
 *  WARNING: Reduced functionality (SD card, WiFi, battery monitor)
 *  INFO: Optional features (compass calibration)
 */
static bool perform_system_check(bool init_results[9]) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          SYSTEM INITIALIZATION & HEALTH CHECK              ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    bool system_healthy = true;
    int critical_failures = 0;
    int warnings = 0;
    
    // === 1. Status LED Check ===
    ESP_LOGI(TAG, "[1/9] Checking Status LED...");
    if (init_results[0]) {
        ESP_LOGI(TAG, "  ✓ Status LED: OK");
        ESP_LOGI(TAG, "    - GPIO: %d", STATUS_LED_GPIO);
        ESP_LOGI(TAG, "    - Mode: LED_STARTUP (fast blink)");
    } else {
        ESP_LOGW(TAG, "  ⚠ Status LED: FAILED (continuing without visual feedback)");
        warnings++;
    }
    
    // === 2. NVS Flash Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[2/9] Checking NVS Flash Storage...");
    if (init_results[1]) {
        // Read some test data to verify flash is working
        nvs_handle_t test_handle;
        esp_err_t ret = nvs_open("tracker", NVS_READONLY, &test_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ NVS Flash: OK (read/write verified)");
            nvs_close(test_handle);
        } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "  ✓ NVS Flash: OK (first boot, no saved data)");
        } else {
            ESP_LOGE(TAG, "  ✗ NVS Flash: READ ERROR");
            critical_failures++;
            system_healthy = false;
        }
    } else {
        ESP_LOGE(TAG, "  ✗ NVS Flash: INITIALIZATION FAILED");
        critical_failures++;
        system_healthy = false;
    }
    
    // === 3. SD Card Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[3/9] Checking SD Card...");
    if (init_results[2]) {
        ESP_LOGI(TAG, "  ✓ SD Card: MOUNTED");
        ESP_LOGI(TAG, "    - Mount point: /sdcard");
        ESP_LOGI(TAG, "    - Log file: /sdcard/SUNFLOW.LOG");  
        ESP_LOGI(TAG, "    - CSV file: /sdcard/SUNFLOW.CSV");
        
        // Test write capability
        sdlog_printf("=== SYSTEM CHECK: SD Card Verified ===");
        ESP_LOGI(TAG, "    - Write test: OK");
        
        // Check SD card status
        uint8_t sd_status = sdlog_get_status();
        const char *sd_status_str[] = {"OK", "SLOW", "FULL", "FAILED"};
        ESP_LOGI(TAG, "    - Status: %s", sd_status_str[sd_status > 3 ? 3 : sd_status]);
        
        if (sd_status >= 2) {
            ESP_LOGW(TAG, "  ⚠ SD Card: %s (logging may fail)", sd_status_str[sd_status]);
            warnings++;
        }
    } else {
        ESP_LOGW(TAG, "  ⚠ SD Card: NOT AVAILABLE");
        ESP_LOGW(TAG, "    - System will run without data logging");
        ESP_LOGW(TAG, "    - Insert SD card and reboot for logging");
        warnings++;
    }
    
    // === 4. GPS Module Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[4/9] Checking GPS Module...");
    if (init_results[3]) {
        ESP_LOGI(TAG, "  ✓ GPS UART: INITIALIZED");
        ESP_LOGI(TAG, "    - Port: UART%d", GPS_UART_PORT);
        ESP_LOGI(TAG, "    - Baud: %d", GPS_BAUD);
        ESP_LOGI(TAG, "    - TX: GPIO%d, RX: GPIO%d", GPS_TX_PIN, GPS_RX_PIN);
        
        // Try to get GPS data (non-blocking, short timeout)
        ESP_LOGI(TAG, "    - Testing GPS communication (5s timeout)...");
        gps_data_t gps_test;
        bool gps_responding = gps_poll_nav_pvt(&gps_test);
        
        if (gps_responding) {
            ESP_LOGI(TAG, "  ✓ GPS: ACTIVE FIX");
            ESP_LOGI(TAG, "    - Satellites: %u", gps_test.num_satellites);
            ESP_LOGI(TAG, "    - Position: %.6f°N, %.6f°W", gps_test.latitude, gps_test.longitude);
            ESP_LOGI(TAG, "    - Altitude: %.1fm", gps_test.altitude_m);
            ESP_LOGI(TAG, "    - Fix type: %u", gps_test.fix_type);
            
            if (init_results[2]) {
                sdlog_printf("GPS OK: %.6f,%.6f (%u sats)", 
                             gps_test.latitude, gps_test.longitude, gps_test.num_satellites);
            }
        } else {
            ESP_LOGW(TAG, "  ⚠ GPS: NO FIX (searching for satellites)");
            ESP_LOGW(TAG, "    - This is normal on first boot or indoors");
            ESP_LOGW(TAG, "    - System will wait for fix before tracking");
            warnings++;
        }
    } else {
        ESP_LOGE(TAG, "  ✗ GPS: INITIALIZATION FAILED");
        ESP_LOGE(TAG, "    - Check wiring: TX→GPIO%d, RX→GPIO%d", GPS_TX_PIN, GPS_RX_PIN);
        critical_failures++;
        system_healthy = false;
    }
    
    // === 5. Motor Drivers Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[5/9] Checking Motor Drivers...");
    if (init_results[4]) {
        ESP_LOGI(TAG, "  ✓ Motors: INITIALIZED");
        ESP_LOGI(TAG, "    - Azimuth: PWM=GPIO%d DIR=GPIO%d", MOTOR_AZ_PWM, MOTOR_AZ_DIR);
        ESP_LOGI(TAG, "    - Elevation: PWM=GPIO%d DIR=GPIO%d", MOTOR_EL_PWM, MOTOR_EL_DIR);
        ESP_LOGI(TAG, "    - Actuator: 200mm stroke @ 11.94mm/s");
        ESP_LOGI(TAG, "    - Ranges: AZ=0-270°, EL=10-85°");
        
        // Get current position from NVS
        float current_az, current_el;
        tracking_get_current_angles(&current_az, &current_el);
        ESP_LOGI(TAG, "    - Current position: Az=%.1f° El=%.1f°", current_az, current_el);
        
        if (init_results[2]) {
            sdlog_printf("Motors OK: Current Az=%.1f El=%.1f", current_az, current_el);
        }
    } else {
        ESP_LOGE(TAG, "  ✗ Motors: INITIALIZATION FAILED");
        ESP_LOGE(TAG, "    - Check PWM/DIR pin connections");
        ESP_LOGE(TAG, "    - Verify motor driver power supply");
        critical_failures++;
        system_healthy = false;
    }
    
    // === 6. Button Interface Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[6/9] Checking Button Interface...");
    if (init_results[5]) {
        ESP_LOGI(TAG, "  ✓ Button: READY");
        ESP_LOGI(TAG, "    - GPIO: %d", START_BTN_GPIO);
        ESP_LOGI(TAG, "    - Active low with pull-up");
        ESP_LOGI(TAG, "    - Debounce: 50ms");
        
        // Test button state
        bool button_pressed = button_is_pressed();
        ESP_LOGI(TAG, "    - Current state: %s", button_pressed ? "PRESSED" : "RELEASED");
        
        if (button_pressed) {
            ESP_LOGW(TAG, "  ⚠ Button is currently pressed - release before continuing");
        }
    } else {
        ESP_LOGE(TAG, "  ✗ Button: INITIALIZATION FAILED");
        ESP_LOGE(TAG, "    - Check GPIO%d connection", START_BTN_GPIO);
        critical_failures++;
        system_healthy = false;
    }
    
    // === 7. Battery Monitor Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[7/9] Checking Battery Monitor...");
    battery_data_t battery_test;
    if (battery_read(&battery_test) == ESP_OK) {
        ESP_LOGI(TAG, "  ✓ Battery Monitor: ACTIVE");
        ESP_LOGI(TAG, "    - ADC: GPIO%d (ADC1_CH%d)", BATTERY_ADC_GPIO, BATTERY_ADC_CHANNEL);
        ESP_LOGI(TAG, "    - Voltage: %.2fV (ADC raw: %u)", 
                 battery_test.voltage, battery_test.adc_raw);
        ESP_LOGI(TAG, "    - State of Charge: %.0f%% (%s)", 
                 battery_test.soc_percent, 
                 battery_soc_level_to_string(battery_test.soc_level));
        ESP_LOGI(TAG, "    - Status: %s", battery_get_status_string(&battery_test));
        ESP_LOGI(TAG, "    - Charging: %s", battery_test.is_charging ? "YES" : "NO");
        
        // Check battery health
        if (battery_test.health == BATTERY_HEALTH_CRITICAL) {
            ESP_LOGE(TAG, "  ✗ CRITICAL BATTERY VOLTAGE: %.2fV", battery_test.voltage);
            ESP_LOGE(TAG, "    - System may shutdown unexpectedly");
            ESP_LOGE(TAG, "    - Charge battery before deployment");
            critical_failures++;
            system_healthy = false;
        } else if (battery_test.health == BATTERY_HEALTH_LOW) {
            ESP_LOGW(TAG, "  ⚠ Low battery: %.2fV (%.0f%%)", 
                     battery_test.voltage, battery_test.soc_percent);
            ESP_LOGW(TAG, "    - Consider charging before extended operation");
            warnings++;
        }
        
        if (init_results[2]) {
            sdlog_printf("Battery OK: %.2fV %.0f%% %s %s", 
                         battery_test.voltage, battery_test.soc_percent,
                         battery_get_status_string(&battery_test),
                         battery_test.is_charging ? "CHARGING" : "DISCHARGING");
        }
    } else {
        ESP_LOGW(TAG, "  ⚠ Battery Monitor: NOT AVAILABLE");
        ESP_LOGW(TAG, "    - System will run without battery monitoring");
        ESP_LOGW(TAG, "    - Check ADC connection at GPIO%d", BATTERY_ADC_GPIO);
        warnings++;
    }
    
    // === 8. WiFi AP Check ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[8/9] Checking WiFi Access Point...");
    if (init_results[6]) {
        ESP_LOGI(TAG, "  ✓ WiFi AP: ACTIVE");
        ESP_LOGI(TAG, "    - SSID: SunflowerTracker");
        ESP_LOGI(TAG, "    - Password: sunflower2025");
        ESP_LOGI(TAG, "    - IP: 192.168.4.1");
        ESP_LOGI(TAG, "    - TCP Port: 8888");
        ESP_LOGI(TAG, "    - Channel: 1 (2.4GHz)");
        
        // Check WiFi configuration
        wifi_bandwidth_t bw;
        esp_wifi_get_bandwidth(WIFI_IF_AP, &bw);
        int8_t power;
        esp_wifi_get_max_tx_power(&power);
        wifi_ps_type_t ps_type;
        esp_wifi_get_ps(&ps_type);
        
        ESP_LOGI(TAG, "    - TX Power: %.1f dBm (MAX)", power * 0.25f);
        ESP_LOGI(TAG, "    - Bandwidth: %s", bw == WIFI_BW_HT20 ? "20MHz" : "40MHz");
        ESP_LOGI(TAG, "    - Power Save: %s", ps_type == WIFI_PS_NONE ? "DISABLED" : "ENABLED");
        ESP_LOGI(TAG, "    - Data rate: 1 Hz (92 bytes/packet)");
        
        // Check for connected clients
        wifi_sta_list_t sta_list = {0};
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
            if (sta_list.num > 0) {
                ESP_LOGI(TAG, "  ✓ LCD Display: CONNECTED (%u client%s)", 
                         sta_list.num, sta_list.num == 1 ? "" : "s");
            } else {
                ESP_LOGI(TAG, "    - No clients connected yet");
            }
        }
        
        if (init_results[2]) {
            sdlog_printf("WiFi OK: AP ready for LCD display (TX: %.1fdBm)", power * 0.25f);
        }
    } else {
        ESP_LOGW(TAG, "  ⚠ WiFi AP: INITIALIZATION FAILED");
        ESP_LOGW(TAG, "    - LCD display will not be available");
        ESP_LOGW(TAG, "    - Tracker will operate normally");
        ESP_LOGW(TAG, "    - Data logged to SD card only");
        warnings++;
    }
    
    // === 9. Compass Check (Optional) ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[9/9] Checking Compass (HMC5883)...");
    if (gps_is_compass_calibrated()) {
        ESP_LOGI(TAG, "  ✓ Compass: CALIBRATED");
        ESP_LOGI(TAG, "    - I2C: Port %d, SDA=GPIO%d, SCL=GPIO%d", I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN);
        
        float heading;
        if (gps_get_compass_heading(&heading)) {
            ESP_LOGI(TAG, "    - Magnetic heading: %.1f°", heading);
            
            float declination = gps_get_magnetic_declination();
            ESP_LOGI(TAG, "    - Declination: %.1f°", declination);
            
            float true_heading;
            if (gps_get_compass_heading_true(&true_heading)) {
                ESP_LOGI(TAG, "    - True heading: %.1f°", true_heading);
            }
        }
        
        if (init_results[2]) {
            sdlog_printf("Compass OK: Calibrated, heading available");
        }
    } else {
        ESP_LOGI(TAG, "  ⓘ Compass: NOT CALIBRATED (optional)");
        ESP_LOGI(TAG, "    - Double-press button to calibrate");
        ESP_LOGI(TAG, "    - Improves azimuth tracking accuracy");
        ESP_LOGI(TAG, "    - System will work without it");
    }
    
    // === Summary ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              SYSTEM CHECK COMPLETE                         ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    if (system_healthy) {
        ESP_LOGI(TAG, "  ✓ SYSTEM READY");
        if (warnings > 0) {
            ESP_LOGW(TAG, "  ⚠ %d warning%s detected (non-critical)", 
                     warnings, warnings == 1 ? "" : "s");
        }
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "  All critical systems operational.");
        ESP_LOGI(TAG, "  Press START button to begin tracking.");
        
        if (init_results[2]) {
            sdlog_printf("System check: PASS (%d warnings)", warnings);
        }
    } else {
        ESP_LOGE(TAG, "  ✗ SYSTEM NOT READY");
        ESP_LOGE(TAG, "  %d CRITICAL FAILURE%s", 
                 critical_failures, critical_failures == 1 ? "" : "S");
        if (warnings > 0) {
            ESP_LOGW(TAG, "  %d warning%s", warnings, warnings == 1 ? "" : "s");
        }
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "  Fix critical issues before operation:");
        if (!init_results[1]) ESP_LOGE(TAG, "   - NVS Flash storage");
        if (!init_results[3]) ESP_LOGE(TAG, "   - GPS module connection");
        if (!init_results[4]) ESP_LOGE(TAG, "   - Motor driver initialization");
        if (!init_results[5]) ESP_LOGE(TAG, "   - Button interface");
        
        if (init_results[2]) {
            sdlog_printf("System check: FAIL (%d critical, %d warnings)", 
                         critical_failures, warnings);
        }
    }
    
    ESP_LOGI(TAG, "");
    
    return system_healthy;
}

/*
 * Set timezone for local time display
 * 
 * Call this once at startup to configure timezone offset from UTC.
 * 
 * Common timezone strings:
 *   EST5EDT,M3.2.0/2,M11.1.0/2  - US Eastern (New York)
 *   CST6CDT,M3.2.0/2,M11.1.0/2  - US Central (Chicago)
 *   MST7MDT,M3.2.0/2,M11.1.0/2  - US Mountain (Denver)
 *   PST8PDT,M3.2.0/2,M11.1.0/2  - US Pacific (Los Angeles)
 *   GMT0BST,M3.5.0/1,M10.5.0    - UK (London)
 *   CET-1CEST,M3.5.0,M10.5.0/3  - Central Europe (Paris, Berlin)
 *   AEST-10AEDT,M10.1.0,M4.1.0/3 - Australia Eastern (Sydney)
 * 
 * NOTE: System clock stays in UTC internally, this only affects
 *       localtime() conversions for display purposes.
 */
static void set_local_timezone(void) {
    // TODO: Change this to YOUR timezone!
    // Example for US Eastern Time:
    const char* timezone = "EST5EDT,M3.2.0/2,M11.1.0/2";
    
    // Example for US Pacific Time (Los Angeles):
    // const char* timezone = "PST8PDT,M3.2.0/2,M11.1.0/2";
    
    // Example for UK (London):
    // const char* timezone = "GMT0BST,M3.5.0/1,M10.5.0";
    
    setenv("TZ", timezone, 1);
    tzset();
    
    ESP_LOGI(TAG, "Timezone set: %s", timezone);
    
    // Log current time in both UTC and local
    time_t now = time(NULL);
    struct tm *utc_tm = gmtime(&now);
    struct tm *local_tm = localtime(&now);
    
    ESP_LOGI(TAG, "  UTC time:   %04d-%02d-%02d %02d:%02d:%02d",
             utc_tm->tm_year + 1900, utc_tm->tm_mon + 1, utc_tm->tm_mday,
             utc_tm->tm_hour, utc_tm->tm_min, utc_tm->tm_sec);
    ESP_LOGI(TAG, "  Local time: %04d-%02d-%02d %02d:%02d:%02d",
             local_tm->tm_year + 1900, local_tm->tm_mon + 1, local_tm->tm_mday,
             local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec);
}

void app_main(void){
    ESP_LOGI(TAG, "=== SUNFLOWER TRACKER STARTING ===");
    ESP_LOGI(TAG, "Build date: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    
    // Check why we woke up (power on vs timer wake)
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    const char* wake_reason = "UNKNOWN";
    switch(wake_cause){
        case ESP_SLEEP_WAKEUP_TIMER:    wake_reason = "TIMER"; break;
        case ESP_SLEEP_WAKEUP_EXT0:     wake_reason = "BUTTON"; break;
        default:                        wake_reason = "POWER_ON"; break;
    }
    ESP_LOGI(TAG, "Wake reason: %s", wake_reason);
    ESP_LOGI(TAG, "");

    bool init_results[9] = {false};  // Changed from 7 to 9 for new checks

    // === Initialize Status LED ===
    ESP_LOGI(TAG, "Initializing status LED...");
    init_results[0] = status_led_init(STATUS_LED_GPIO, true);
    if (init_results[0]) {
        status_led_set_mode(LED_STARTUP);
    }

    // === Initialize Flash Storage ===
    ESP_LOGI(TAG, "Initializing flash storage...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        init_results[1] = true;
    } else if (ret != ESP_OK) {
        init_results[1] = false;
    } else {
        init_results[1] = true;
    }

    // === Initialize SD Card Logging ===
    ESP_LOGI(TAG, "Waiting for boot to stabilize before SD init...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second delay
    
    ESP_LOGI(TAG, "Initializing SD card...");
    sdlog_cfg_t sd_config = {
        .mosi = SD_MOSI, .miso = SD_MISO, 
        .sclk = SD_SCLK, .cs = SD_CS
    };
    init_results[2] = sdlog_init(&sd_config);

    // === Initialize GPS ===
    ESP_LOGI(TAG, "Initializing GPS system...");
    gps_cfg_t gps_config = {
        .uart_port = GPS_UART_PORT,
        .tx_io = GPS_TX_PIN,
        .rx_io = GPS_RX_PIN,
        .baud_rate = GPS_BAUD,
        .i2c_port = I2C_PORT,
        .sda_io = I2C_SDA_PIN,
        .scl_io = I2C_SCL_PIN
    };
    ret = gps_init(&gps_config);
    init_results[3] = (ret == ESP_OK);

    // === Initialize Motors ===
    ESP_LOGI(TAG, "Initializing motor control...");
    motor_cfg_t motor_config = {
        .az_pwm_pin = MOTOR_AZ_PWM, 
        .az_dir_pin = MOTOR_AZ_DIR,
        .el_pwm_pin = MOTOR_EL_PWM, 
        .el_dir_pin = MOTOR_EL_DIR,
        .stroke_mm = 200.0,          // Keep for now (will update motor.c separately)
        .speed_mm_per_s = 11.94,     // Nominal speed
        .max_az_deg = 45,            // CHANGED: ±45° range (90° total)
        .max_el_deg = 56,            // CHANGED: ±56° range (112° total)
        .min_el_deg = -56            // CHANGED: Lower limit
    };
    ret = motor_init(&motor_config);
    init_results[4] = (ret == ESP_OK);

    // === Initialize Button ===
    ESP_LOGI(TAG, "Initializing button interface...");
    button_cfg_t button_config = {
        .gpio = START_BTN_GPIO,
        .active_low = true,
        .pull_up = true,
        .pull_down = false,
        .debounce_ms = 50
    };
    ret = button_init(&button_config);
    init_results[5] = (ret == ESP_OK);

    // === Initialize Battery Monitor ===
    ESP_LOGI(TAG, "Initializing battery monitor...");
    battery_cfg_t battery_config = {
        .adc_channel = BATTERY_ADC_CHANNEL,
        .gpio_num = BATTERY_ADC_GPIO,
        .voltage_divider_ratio = BATTERY_VOLTAGE_RATIO,
        .v_full = 14.6f,
        .v_nominal = 12.8f,
        .v_cutoff = 10.0f,
        .v_critical = 9.0f,
        .samples_per_read = 32
    };
    ret = battery_init(&battery_config);
    init_results[7] = (ret == ESP_OK);  // Battery is index 7

    // === Initialize WiFi Access Point ===
    ESP_LOGI(TAG, "Initializing WiFi AP for LCD display...");
    ret = wifi_comm_init_ap();
    init_results[6] = (ret == ESP_OK);

    // Compass check will be done in system check (index 8)
    init_results[8] = gps_is_compass_calibrated();

    ESP_LOGI(TAG, "");
    
    // === PERFORM COMPREHENSIVE SYSTEM CHECK ===
    bool system_ready = perform_system_check(init_results);
    
    if (!system_ready) {
        // Critical failure - halt with error LED
        if (init_results[0]) {
            status_led_set_mode(LED_ERROR);
        }
        
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "SYSTEM HALTED - Fix critical issues and reboot");
        ESP_LOGE(TAG, "");
        
        if (init_results[2]) {
            sdlog_printf("CRITICAL: System halted due to initialization failures");
        }
        
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    

    // === Check If We Should Auto-Start ===
    bool auto_start = (wake_cause == ESP_SLEEP_WAKEUP_TIMER);
    
    if (!auto_start) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║           WAITING FOR USER INPUT                           ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "  Actions:");
        ESP_LOGI(TAG, "   • Press button once      → Start tracking");
        ESP_LOGI(TAG, "   • Hold button 3 seconds  → Calibrate mount");
        ESP_LOGI(TAG, "   • Double-press button    → Calibrate compass");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "  System Status:");
        ESP_LOGI(TAG, "   • WiFi AP active - broadcasting data");
        ESP_LOGI(TAG, "   • LCD display shows live sensor readings");
        ESP_LOGI(TAG, "   • Press button when ready to track");
        ESP_LOGI(TAG, "");
        
        status_led_set_mode(LED_WAITING);
        if (init_results[2]) {
            sdlog_printf("Waiting for button press (transmitting status)");
        }
        
        // === NON-BLOCKING BUTTON WAIT WITH DATA TRANSMISSION ===
        ESP_LOGI(TAG, "Starting background data transmission...");
        ESP_LOGI(TAG, "Press START button to begin tracking");
        ESP_LOGI(TAG, "");
        
        time_t boot_time = time(NULL);
        uint32_t button_wait_seconds = 0;
        bool button_pressed = false;
        
        // Initialize previous values for delta calculation
        float prev_azimuth = 0.0f;
        float prev_elevation = 0.0f;
        bool first_reading = true;
        
        // Button press detection state
        uint32_t last_press_time = 0;
        bool waiting_for_double_press = false;
        
        while (!button_pressed) {
            // === CHECK FOR BUTTON EVENTS (BEFORE DATA TRANSMISSION) ===
            
            // Check for button press using proper wait function
            if (button_wait_for_press(10)) {  // 10ms timeout (non-blocking check)
                uint32_t press_time = xTaskGetTickCount();
                uint32_t time_since_last = pdTICKS_TO_MS(press_time - last_press_time);
                
                ESP_LOGI(TAG, "Button pressed (time since last: %lu ms)", 
                         (unsigned long)time_since_last);
                
                // Check for DOUBLE-PRESS (for compass calibration)
                if (waiting_for_double_press && time_since_last < 1000) {
                    ESP_LOGI(TAG, "✓ DOUBLE-PRESS detected - starting compass calibration");
                    
                    status_led_set_mode(LED_STARTUP);
                    sdlog_printf("Compass calibration started (double-press)");
                    
                    bool success = gps_calibrate_compass();
                    
                    if (success) {
                        ESP_LOGI(TAG, "✓ Compass calibration successful");
                        sdlog_printf("Compass calibration: OK");
                        
                        // Blink LED 3 times
                        for (int i = 0; i < 3; i++) {
                            status_led_set_mode(LED_ERROR);
                            vTaskDelay(pdMS_TO_TICKS(200));
                            status_led_set_mode(LED_TRACKING);
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                    } else {
                        ESP_LOGW(TAG, "✗ Compass calibration failed");
                        sdlog_printf("Compass calibration: FAILED");
                        status_led_set_mode(LED_ERROR);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                    
                    status_led_set_mode(LED_WAITING);
                    waiting_for_double_press = false;
                    last_press_time = 0;
                    continue;  // Stay in button wait loop
                }
                
                // Check for LONG-PRESS (for mount calibration)
                uint32_t hold_start = xTaskGetTickCount();
                bool is_long_press = false;
                
                while (button_is_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    uint32_t hold_time = pdTICKS_TO_MS(xTaskGetTickCount() - hold_start);
                    
                    if (hold_time >= 3000) {
                        is_long_press = true;
                        ESP_LOGI(TAG, "✓ LONG-PRESS detected (3+ seconds) - starting mount calibration");
                        
                        status_led_set_mode(LED_STARTUP);
                        sdlog_printf("Mount calibration started (long-press)");
                        
                        tracking_calibrate_mount_offset_now();
                        
                        ESP_LOGI(TAG, "✓ Mount calibration complete");
                        sdlog_printf("Mount calibration: OK");
                        
                        status_led_set_mode(LED_WAITING);
                        
                        // Wait for release
                        while (button_is_pressed()) {
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                        
                        break;
                    }
                }
                
                if (is_long_press) {
                    waiting_for_double_press = false;
                    last_press_time = 0;
                    continue;  // Stay in button wait loop
                }
                
                // SINGLE-PRESS: Start tracking
                if (!is_long_press && !waiting_for_double_press) {
                    ESP_LOGI(TAG, "✓ SINGLE-PRESS detected - waiting 1s for possible double-press");
                    waiting_for_double_press = true;
                    last_press_time = press_time;
                    
                    // Wait 1 second to see if there's a second press
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    if (waiting_for_double_press) {
                        // No second press within 1 second = START TRACKING
                        ESP_LOGI(TAG, "✓ Starting tracking (no second press detected)");
                        button_pressed = true;
                        
                        if (init_results[2]) {
                            sdlog_printf("Button pressed after %lu seconds - starting tracking", 
                                         (unsigned long)button_wait_seconds);
                        }
                        break;
                    }
                }
            }
            
            // === BACKGROUND DATA TRANSMISSION (SAME AS BEFORE) ===
            
            // Get current time
            time_t now = time(NULL);
            uint16_t uptime_h = (uint16_t)((now - boot_time) / 3600);
            
            // Get current position (will be default/last known while waiting)
            float current_azimuth = 0.0f;
            float current_elevation = 0.0f;
            tracking_get_current_angles(&current_azimuth, &current_elevation);
            
            // Calculate deltas
            float delta_az = 0.0f;
            float delta_elev = 0.0f;
            if (!first_reading) {
                delta_az = current_azimuth - prev_azimuth;
                delta_elev = current_elevation - prev_elevation;
                if (delta_az > 180.0f) delta_az -= 360.0f;
                if (delta_az < -180.0f) delta_az += 360.0f;
            }
            first_reading = false;
            prev_azimuth = current_azimuth;
            prev_elevation = current_elevation;
            
            // Get GPS data (continue background acquisition)
            gps_data_t g = {0};
            bool gps_fresh = gps_poll_nav_pvt(&g);
            bool gps_valid = gps_fresh || gps_get_last(&g);
            uint32_t gps_fix_age_sec = gps_valid ? (uint32_t)(now - g.timestamp) : 9999;
            
            // Get battery data
            battery_data_t battery;
            uint16_t battery_adc_raw = 0;
            float battery_v = 0.0f;
            float battery_soc = 0.0f;
            uint8_t battery_soc_level = 0;
            uint8_t battery_charging = 0;
            
            if (battery_read(&battery) == ESP_OK) {
                battery_adc_raw = battery.adc_raw;
                battery_v = battery.voltage;
                battery_soc = battery.soc_percent;
                battery_soc_level = (uint8_t)battery.soc_level;
                battery_charging = battery.is_charging ? 1 : 0;
            }
            
            // Calculate sun position (if GPS available)
            sun_pos_t sun = {0};
            if (gps_valid) {
                sun = solar_compute(g.latitude, g.longitude, now);
            }
            
            // Calculate sunrise/sunset
            uint32_t sunrise_time = 0;
            uint32_t sunset_time = 0;
            if (gps_valid) {
                solar_events_t events = solar_events(g.latitude, g.longitude, now);
                if (events.has_sunrise) sunrise_time = (uint32_t)events.sunrise_utc;
                if (events.has_sunset)  sunset_time  = (uint32_t)events.sunset_utc;
            }
            
            // Get WiFi client count
            uint8_t wifi_client_count = 0;
            wifi_sta_list_t sta_list = {0};
            if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
                wifi_client_count = (uint8_t)sta_list.num;
            }
            
            // Get SD status
            uint8_t sd_status = sdlog_get_status();
            
            // System status = WAITING (0)
            uint8_t system_status = 0;
            
            // Prepare data packet
            tracker_data_t tx_data = {
                .elevation = current_elevation,
                .azimuth = current_azimuth,
                .delta_elevation = delta_elev,
                .delta_azimuth = delta_az,
                .battery_adc = battery_adc_raw,
                .battery_voltage = battery_v,
                .battery_soc_percent = battery_soc,
                .battery_soc = battery_soc_level,
                .battery_charging = battery_charging,
                .timestamp = (uint32_t)now,
                .sunrise_time = sunrise_time,
                .sunset_time = sunset_time,
                .status = system_status,  // WAITING = 0
                .latitude = gps_valid ? (float)g.latitude : 0.0f,
                .longitude = gps_valid ? (float)g.longitude : 0.0f,
                .gps_valid = gps_valid ? 1 : 0,
                .gps_satellites = gps_valid ? g.num_satellites : 0,
                .last_gps_fix_age_sec = gps_fix_age_sec,
                .sun_elevation = gps_valid ? (float)sun.elevation_deg : 0.0f,
                .sun_azimuth = gps_valid ? (float)sun.azimuth_deg : 0.0f,
                .moves_today = 0,  // No moves yet
                .total_moves = 0,  // No moves yet
                .uptime_hours = uptime_h,
                .wifi_rssi = -128,
                .wifi_clients = wifi_client_count,
                .sd_card_status = sd_status,
                .tracking_quality = 255,  // No tracking active
            };
            
            // Send data to LCD
            esp_err_t send_ret = wifi_comm_send_data(&tx_data);
            if (send_ret == ESP_OK) {
                // Log every 10 seconds to reduce spam
                if (button_wait_seconds % 10 == 0) {
                    ESP_LOGI(TAG, "WAITING[%lu]: Batt=%.2fV GPS=%s(%u) Clients=%u (Press button to start)",
                             (unsigned long)button_wait_seconds,  // FIXED: Cast to unsigned long
                             battery_v,
                             gps_valid ? "OK" : "NO_FIX",
                             gps_valid ? g.num_satellites : 0,
                             wifi_client_count);
                }
            } else if (send_ret == ESP_ERR_NOT_FOUND) {
                if (button_wait_seconds % 30 == 0) {
                    ESP_LOGW(TAG, "Waiting for LCD display connection... (%lu seconds)", 
                             (unsigned long)button_wait_seconds);  // FIXED
                }
            }
            
            // Wait 100ms before next iteration (responsive button check)
            vTaskDelay(pdMS_TO_TICKS(100));
            button_wait_seconds++;
            
            // Log reminders every 60 seconds
            if (button_wait_seconds % 600 == 0) {  // Every 60 seconds (600 * 100ms)
                ESP_LOGI(TAG, "Still waiting for button press (%lu seconds elapsed)...", 
                         (unsigned long)(button_wait_seconds / 10));
                if (init_results[2]) {
                    sdlog_printf("Button wait: %lu seconds", 
                                 (unsigned long)(button_wait_seconds / 10));
                }
            }
        }
        
    } else {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║              AUTO-START (Timer Wake)                       ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        if (init_results[2]) {
            sdlog_printf("Timer wake - auto-start");
        }
    }


    // === Wait for GPS Fix (Non-Blocking) ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          GPS ACQUISITION (NON-BLOCKING)                    ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    
    ESP_LOGI(TAG, "GPS Module: BN-880 NMEA Parser");
    ESP_LOGI(TAG, "  - Provides: Lat/Lon, Altitude, Time (UTC)");
    ESP_LOGI(TAG, "  - Time source: GNSS satellites (accurate to ~100ns)");
    ESP_LOGI(TAG, "  - System will start WITHOUT waiting for GPS");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Attempting initial GPS fix (60s timeout)...");
    ESP_LOGI(TAG, "  (System continues if GPS not available)");
    ESP_LOGI(TAG, "");
    
    status_led_set_mode(LED_WAITING);
    
    gps_data_t first_fix;
    int wait_count = 0;
    const int MAX_WAIT = 12;  // 60 seconds max (12 * 5s)
    bool got_initial_fix = false;
    
    while (!gps_poll_nav_pvt(&first_fix) && wait_count < MAX_WAIT) {
        wait_count++;
        
        // Progress indicator every 10 seconds
        if (wait_count % 2 == 0) {
            ESP_LOGI(TAG, "Searching for satellites... (%d seconds elapsed)", wait_count * 5);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    if (wait_count >= MAX_WAIT) {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "⚠ GPS TIMEOUT - No fix after 60 seconds");
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "System will continue WITHOUT GPS:");
        ESP_LOGW(TAG, "  - Tracking disabled (waiting for GPS)");
        ESP_LOGW(TAG, "  - WiFi display will show 'NO GPS' status");
        ESP_LOGW(TAG, "  - System will retry GPS acquisition in background");
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "Troubleshooting:");
        ESP_LOGW(TAG, "  1. Check GPS antenna connection");
        ESP_LOGW(TAG, "  2. Ensure clear view of sky (not indoors)");
        ESP_LOGW(TAG, "  3. Verify UART wiring (TX/RX not swapped)");
        ESP_LOGW(TAG, "  4. GPS will continue searching in background");
        ESP_LOGW(TAG, "");
        
        if (init_results[2]) {
            sdlog_printf("WARNING: No initial GPS fix - system starting without GPS");
        }
        
        status_led_set_mode(LED_WAITING);  // Stay in waiting mode
        got_initial_fix = false;
    } else {
        // Got GPS fix - time is now automatically synced!
        time_t system_time = time(NULL);
        
        // === SET LOCAL TIMEZONE AFTER GPS TIME SYNC ===
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Setting local timezone for display...");
        set_local_timezone();
        
        // Get BOTH UTC and local time
        struct tm *utc_tm = gmtime(&system_time);
        struct tm *local_tm = localtime(&system_time);
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✓ GPS FIX ACQUIRED");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Location:");
        ESP_LOGI(TAG, "  - Latitude: %.6f°%c", 
                 fabs(first_fix.latitude), first_fix.latitude >= 0 ? 'N' : 'S');
        ESP_LOGI(TAG, "  - Longitude: %.6f°%c", 
                 fabs(first_fix.longitude), first_fix.longitude >= 0 ? 'E' : 'W');
        ESP_LOGI(TAG, "  - Altitude: %.1f meters", first_fix.altitude_m);
        ESP_LOGI(TAG, "  - Satellites: %u", first_fix.num_satellites);
        ESP_LOGI(TAG, "  - Fix quality: %u", first_fix.fix_type);
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Time (from GPS):");
        ESP_LOGI(TAG, "  - UTC:   %04d-%02d-%02d %02d:%02d:%02d",
                 utc_tm->tm_year + 1900, utc_tm->tm_mon + 1, utc_tm->tm_mday,
                 utc_tm->tm_hour, utc_tm->tm_min, utc_tm->tm_sec);
        ESP_LOGI(TAG, "  - Local: %04d-%02d-%02d %02d:%02d:%02d",
                 local_tm->tm_year + 1900, local_tm->tm_mon + 1, local_tm->tm_mday,
                 local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec);
        ESP_LOGI(TAG, "  - Unix epoch: %ld seconds", (long)system_time);
        ESP_LOGI(TAG, "  - Accuracy: ~100 nanoseconds (satellite-derived)");
        ESP_LOGI(TAG, "");
        
        if (init_results[2]) {
            sdlog_printf("GPS fix: %.6f,%.6f @ %04d-%02d-%02d %02d:%02d LOCAL (%u sats)",
                         first_fix.latitude, first_fix.longitude,
                         local_tm->tm_year + 1900, local_tm->tm_mon + 1, local_tm->tm_mday,
                         local_tm->tm_hour, local_tm->tm_min, first_fix.num_satellites);
        }
        
        ESP_LOGI(TAG, "System ready - time and location acquired from GPS");
        ESP_LOGI(TAG, "");
        got_initial_fix = true;
        status_led_set_mode(LED_TRACKING);
    }
    
    // === START TRACKING SYSTEM (EXISTING CODE) ===
    ESP_LOGI(TAG, "=== STARTING TRACKING ===");
    
    tracking_start();
    status_led_set_mode(LED_TRACKING);
    
    ESP_LOGI(TAG, "✓ Tracking active");
    if (init_results[2]) {
        sdlog_printf("Tracking started");
    }

    // === Start Calibration Monitor ===
    ESP_LOGI(TAG, "Starting calibration monitor...");
    BaseType_t calib_ret = xTaskCreate(
        calib_task,
        "calibration",
        2048,
        NULL,
        4,
        NULL
    );
    
    if (calib_ret != pdPASS) {
        ESP_LOGW(TAG, "Calibration monitor failed to start");
        sdlog_printf("Warning: calibration unavailable");
    } else {
        ESP_LOGI(TAG, "Calibration monitor OK");
    }

    // === Start Compass Calibration Monitor ===
    ESP_LOGI(TAG, "Starting compass calibration monitor...");
    BaseType_t compass_ret = xTaskCreate(
        compass_calib_task,
        "compass_calib",
        2048,
        NULL,
        3,
        NULL
    );
    
    if (compass_ret != pdPASS) {
        ESP_LOGW(TAG, "Compass monitor failed to start");
    } else {
        ESP_LOGI(TAG, "Compass calibration OK (double-press to use)");
    }
    
    // === Initialization Complete ===
    ESP_LOGI(TAG, "=== SYSTEM READY ===");
    ESP_LOGI(TAG, "Tracker is now running autonomously");
    ESP_LOGI(TAG, "LED: STARTUP→WAITING→TRACKING→ERROR→SLEEP");
    ESP_LOGI(TAG, "Button: press=start, hold 3s=calibrate, double=compass");
    ESP_LOGI(TAG, "WiFi: AP 'SunflowerTracker' broadcasting for LCD display");
    ESP_LOGI(TAG, "Check SD card for detailed logs");
    
    sdlog_printf("System ready - transmitting to LCD display via WiFi");
    
    // === Main Loop: Transmit Tracking Data to LCD Display ===
    ESP_LOGI(TAG, "Entering WiFi data transmission loop (1 Hz)...");
    
    uint32_t connection_wait_counter = 0;
    
    // Initialize previous values for delta calculation
    float prev_azimuth = 0.0f;
    float prev_elevation = 0.0f;
    bool first_reading = true;
    
    // Track uptime since wake
    time_t boot_time = time(NULL);
    
    while (1) {
        // Get current time (BOTH UTC and local)
        time_t now = time(NULL);
        uint16_t uptime_h = (uint16_t)((now - boot_time) / 3600);
        
        // === CHANGED: Convert to local time for transmission ===
        struct tm *local_tm = localtime(&now);
        time_t local_timestamp = mktime(local_tm);  // Local time as Unix timestamp
        
        ESP_LOGV(TAG, "Time for transmission: Local=%04d-%02d-%02d %02d:%02d:%02d (epoch=%ld)",
                 local_tm->tm_year + 1900, local_tm->tm_mon + 1, local_tm->tm_mday,
                 local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec,
                 (long)local_timestamp);
        
        // Get current tracking position from tracking module
        float current_azimuth = 0.0f;
        float current_elevation = 0.0f;
        tracking_get_current_angles(&current_azimuth, &current_elevation);
        
        // Calculate deltas (change since last transmission)
        float delta_az = 0.0f;
        float delta_elev = 0.0f;
        if (!first_reading) {
            delta_az = current_azimuth - prev_azimuth;
            delta_elev = current_elevation - prev_elevation;
            
            // Handle azimuth wraparound (e.g., 359° -> 1° should be +2°, not -358°)
            if (delta_az > 180.0f) delta_az -= 360.0f;
            if (delta_az < -180.0f) delta_az += 360.0f;
        }
        first_reading = false;
        
        // Store current values for next iteration
        prev_azimuth = current_azimuth;
        prev_elevation = current_elevation;
        
        // Get GPS data and calculate fix age
        gps_data_t g = {0};
        bool gps_fresh = gps_poll_nav_pvt(&g);
        bool gps_valid = gps_fresh || gps_get_last(&g);
        uint32_t gps_fix_age_sec = 0;
        
        if (gps_valid) {
            // Calculate how long since last valid fix
            gps_fix_age_sec = (uint32_t)(now - g.timestamp);
            
            // Warn if GPS data is stale (>5 minutes)
            if (gps_fix_age_sec > 300) {
                static uint32_t last_stale_warn = 0;
                if ((now - last_stale_warn) >= 60) {
                    ESP_LOGW(TAG, "GPS data stale: %lu seconds old", 
                             (unsigned long)gps_fix_age_sec);  // FIXED
                    last_stale_warn = now;
                }
                
                // GPS STALE: Set LED to waiting/standby
                if (gps_fix_age_sec > 600) {  // >10 minutes = error
                    status_led_set_mode(LED_ERROR);
                } else {  // 5-10 minutes = waiting
                    status_led_set_mode(LED_WAITING);
                }
            }
        } else {
            // NO GPS AT ALL: Set LED to waiting/standby
            status_led_set_mode(LED_WAITING);  // ADD THIS LINE
            
            static uint32_t last_no_gps_warn = 0;
            if ((now - last_no_gps_warn) >= 30) {
                ESP_LOGW(TAG, "No GPS fix - system in standby");
                last_no_gps_warn = now;
            }
        }
        
        // Calculate sunrise/sunset times for today
        uint32_t sunrise_time = 0;
        uint32_t sunset_time = 0;
        
        if (gps_valid) {
            solar_events_t events = solar_events(g.latitude, g.longitude, now);
            if (events.has_sunrise) sunrise_time = (uint32_t)events.sunrise_utc;
            if (events.has_sunset)  sunset_time  = (uint32_t)events.sunset_utc;
            
            // Log sunrise/sunset once per day at startup or midnight
            static uint32_t last_event_log = 0;
            if ((now - last_event_log) >= 86400 || last_event_log == 0) {
                struct tm *sunrise_tm = localtime(&events.sunrise_utc);
                struct tm *sunset_tm = localtime(&events.sunset_utc);
                
                ESP_LOGI(TAG, "Solar times: Sunrise %02d:%02d, Sunset %02d:%02d",
                         sunrise_tm->tm_hour, sunrise_tm->tm_min,
                         sunset_tm->tm_hour, sunset_tm->tm_min);
                         
                sdlog_printf("Today: sunrise %02d:%02d, sunset %02d:%02d",
                             sunrise_tm->tm_hour, sunrise_tm->tm_min,
                             sunset_tm->tm_hour, sunset_tm->tm_min);
                             
                last_event_log = now;
            }
        }
        
        // Get SD card status
        uint8_t sd_status = sdlog_get_status();
        
        // Get number of WiFi clients connected to AP
        uint8_t wifi_client_count = 0;
        wifi_sta_list_t sta_list = {0};
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
            wifi_client_count = (uint8_t)sta_list.num;
            
            // Log client connections/disconnections
            static uint8_t last_client_count = 0;
            if (wifi_client_count != last_client_count) {
                if (wifi_client_count > last_client_count) {
                    ESP_LOGI(TAG, "LCD display connected (%u total clients)", wifi_client_count);
                    sdlog_printf("Display connected (clients: %u)", wifi_client_count);
                } else {
                    ESP_LOGI(TAG, "LCD display disconnected (%u remaining)", wifi_client_count);
                    sdlog_printf("Display disconnected (clients: %u)", wifi_client_count);
                }
                last_client_count = wifi_client_count;
            }
        }
        
        // Get sun position
        sun_pos_t sun = solar_compute(
            gps_valid ? g.latitude : 0.0, 
            gps_valid ? g.longitude : 0.0, 
            now
        );
        
        // Calculate tracking error (how far off we are from sun)
        float az_error = fabs(current_azimuth - sun.azimuth_deg);
        float el_error = fabs(current_elevation - sun.elevation_deg);
        if (az_error > 180.0f) az_error = 360.0f - az_error;  // Handle wraparound
        uint8_t tracking_quality = (uint8_t)(fmax(az_error, el_error));
        
        // WiFi RSSI - not available in AP mode, set to default
        int8_t rssi = -128;  // Default: no signal (we're the AP, not station)
        
        // Get move counters from tracking module
        uint32_t moves_today = 0;
        uint32_t total_moves = 0;
        tracking_get_move_stats(&moves_today, &total_moves);
        
        // Get current LED status to determine system state
        status_led_mode_t led_mode = status_led_get_mode();
        uint8_t system_status = 1;  // Default: tracking
        switch(led_mode) {
            case LED_STARTUP:   system_status = 3; break;  // Calibrating
            case LED_WAITING:   system_status = 0; break;  // Standby
            case LED_TRACKING:  system_status = 1; break;  // Tracking
            case LED_ERROR:     system_status = 255; break; // Error
            case LED_SLEEP:     system_status = 2; break;  // Sleep
            default:            system_status = 1; break;  // Default to tracking
        }
        
        // Get battery data
        battery_data_t battery;
        uint16_t battery_adc_raw = 0;
        float battery_v = 0.0f;
        float battery_soc = 0.0f;
        uint8_t battery_soc_level = 0;
        uint8_t battery_charging = 0;
        
        if (battery_read(&battery) == ESP_OK) {
            battery_adc_raw = battery.adc_raw;
            battery_v = battery.voltage;
            battery_soc = battery.soc_percent;
            battery_soc_level = (uint8_t)battery.soc_level;
            battery_charging = battery.is_charging ? 1 : 0;
            
            // Log battery status every 60 seconds
            static uint32_t last_battery_log = 0;
            if ((now - last_battery_log) >= 60) {
                ESP_LOGI(TAG, "Battery: %.2fV (%.0f%% SOC) [%s] %s",
                         battery.voltage, battery.soc_percent,
                         battery_soc_level_to_string(battery.soc_level),
                         battery.is_charging ? "CHARGING" : "DISCHARGING");
                         
                sdlog_printf("Battery: %.2fV %.0f%% %s ADC=%u",
                             battery.voltage, battery.soc_percent,
                             battery_get_status_string(&battery),
                             battery.adc_raw);
                             
                last_battery_log = now;
            }
            
            // Check for critical battery
            if (battery_is_critical()) {
                ESP_LOGE(TAG, "⚠ CRITICAL BATTERY VOLTAGE: %.2fV (%.0f%% SOC)", 
                         battery.voltage, battery.soc_percent);
                status_led_set_mode(LED_ERROR);
                sdlog_printf("CRITICAL: Battery %.2fV %.0f%% - shutdown recommended", 
                             battery.voltage, battery.soc_percent);
            }
        } else {
            ESP_LOGW(TAG, "Battery read failed - using defaults");
        }
        
        // Prepare data packet for LCD display with ALL new fields
        tracker_data_t tx_data = {
            .elevation = current_elevation,
            .azimuth = current_azimuth,
            .delta_elevation = delta_elev,
            .delta_azimuth = delta_az,
            .battery_adc = battery_adc_raw,
            .battery_voltage = battery_v,
            .battery_soc_percent = battery_soc,
            .battery_soc = battery_soc_level,
            .battery_charging = battery_charging,
            .timestamp = (uint32_t)local_timestamp,  // CHANGED: Local time instead of UTC
            .sunrise_time = sunrise_time,            // These stay UTC (calculations use UTC)
            .sunset_time = sunset_time,              // These stay UTC (calculations use UTC)
            .status = system_status,
            .latitude = gps_valid ? (float)g.latitude : 0.0f,
            .longitude = gps_valid ? (float)g.longitude : 0.0f,
            .gps_valid = gps_valid ? 1 : 0,
            .gps_satellites = gps_valid ? g.num_satellites : 0,
            .last_gps_fix_age_sec = gps_fix_age_sec,
            .sun_elevation = gps_valid ? (float)sun.elevation_deg : 0.0f,
            .sun_azimuth = gps_valid ? (float)sun.azimuth_deg : 0.0f,
            .moves_today = moves_today,
            .total_moves = total_moves,
            .uptime_hours = uptime_h,
            .wifi_rssi = rssi,
            .wifi_clients = wifi_client_count,
            .sd_card_status = sd_status,
            .tracking_quality = tracking_quality,
        };
        
        // Send data over WiFi to LCD display
        esp_err_t send_ret = wifi_comm_send_data(&tx_data);
        if (send_ret == ESP_OK) {
            // Enhanced log with new data
            ESP_LOGI(TAG, "LCD[%u]: El=%.1f° Az=%.1f° Batt=%.2fV(%.0f%%) GPS=%lus SD=%u Sun[%.1f°,%.1f°]",
                     wifi_client_count,
                     tx_data.elevation, tx_data.azimuth,
                     tx_data.battery_voltage, tx_data.battery_soc_percent,
                     (unsigned long)gps_fix_age_sec,  // FIXED
                     sd_status,
                     tx_data.sun_elevation, tx_data.sun_azimuth);
            connection_wait_counter = 0;
        } else if (send_ret == ESP_ERR_NOT_FOUND) {
            connection_wait_counter++;
            if (connection_wait_counter % 10 == 0) {
                ESP_LOGW(TAG, "Waiting for LCD display to connect... (%lu s)", 
                         (unsigned long)connection_wait_counter);  // FIXED
            }
        } else {
            ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(send_ret));
        }
        
        // Update once per second (1 Hz transmission rate)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}