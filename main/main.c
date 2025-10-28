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
 * │                                                                       │
 * │  Remote Display:                                                      │
 * │  ├── WiFi Station (connects to main tracker)                         │
 * │  ├── LCD Driver (SPI display control)                                │
 * │  ├── Dashboard UI (angles, battery, GPS, status)                     │
 * │  └── Real-time Graph (battery voltage history)                       │
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
 * │  └── Daily Average: ~25-50mA (depends on tracking frequency)         │
 * │                                                                       │
 * │  Remote Display:                                                      │
 * │  ├── Active Display: ~80-120mA (WiFi + LCD backlight)               │
 * │  └── Can be powered independently from main tracker                  │
 * │                                                                       │
 * │ Deployment Workflow:                                                  │
 * │  Main Tracker Setup:                                                  │
 * │  1. Flash firmware, install hardware, insert SD card                 │
 * │  2. Power on, wait for GPS fix (LED_WAITING)                         │
 * │  3. Manually align panel to sun, long-press button (calibration)     │
 * │  4. Press start button to begin autonomous tracking                   │
 * │  5. System creates WiFi AP "SunflowerTracker"                        │
 * │                                                                       │
 * │  Remote Display Setup:                                                │
 * │  1. Flash LCD display firmware to ESP32-WROOM                        │
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
#include "solar.h"          // ADD THIS LINE

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
#define GPS_TX_PIN     17     // Connect to GPS RX
#define GPS_RX_PIN     16     // Connect to GPS TX
#define GPS_BAUD       9600   // Default baud rate for BN-880

// Compass on GPS module (I2C communication)
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_PIN    21     // I2C data line
#define I2C_SCL_PIN    22     // I2C clock line

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
 * Main Application Entry Point - Main Tracker Unit
 * 
 * This runs when the ESP32 first powers on or wakes from sleep.
 * 
 * Startup sequence:
 *  1. Initialize all hardware peripherals (LED, flash, SD, GPS, motors, button)
 *  2. Initialize WiFi Access Point for LCD display communication
 *  3. Wait for button press (unless auto-starting from sleep timer)
 *  4. Start autonomous solar tracking
 *  5. Enter main loop: continuously send tracking data to LCD display
 * 
 * Main loop operation:
 *  - Reads current tracking position (azimuth, elevation)
 *  - Reads GPS location and time
 *  - Packages data into tracker_data_t structure
 *  - Transmits data over WiFi to LCD display (1 Hz update rate)
 *  - Logs connection status and important events
 * 
 * Error handling:
 *  - Critical failures (GPS, motors): System halts with LED_ERROR
 *  - Non-critical failures (SD card, WiFi): System continues with warning
 *  - LCD display disconnection: System continues tracking, data buffered
 * 
 * Power management:
 *  - Timer wake: Auto-starts tracking without button press
 *  - Button wake: Waits for user input before tracking
 *  - WiFi can be disabled to save power if LCD not in use
 */
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

    bool init_results[7] = {false, false, false, false, false, false, false};

    // === Initialize Status LED ===
    ESP_LOGI(TAG, "Starting status LED...");
    init_results[0] = status_led_init(STATUS_LED_GPIO, true);
    if (!init_results[0]) {
        ESP_LOGE(TAG, "Status LED failed - continuing anyway");
    } else {
        status_led_set_mode(LED_STARTUP);
        ESP_LOGI(TAG, "Status LED OK");
    }

    // === Initialize Flash Storage ===
    ESP_LOGI(TAG, "Starting flash storage...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Flash needs erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_LOGI(TAG, "Flash erased and reinitialized");
        init_results[1] = true;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Flash storage failed: %s", esp_err_to_name(ret));
        init_results[1] = false;
        status_led_set_mode(LED_ERROR);
        ESP_LOGE(TAG, "CRITICAL: System halted");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        init_results[1] = true;
    }
    ESP_LOGI(TAG, "Flash storage OK");

    // === Initialize SD Card Logging ===
    ESP_LOGI(TAG, "Starting SD card...");
    sdlog_cfg_t sd_config = {
        .mosi = SD_MOSI, .miso = SD_MISO, 
        .sclk = SD_SCLK, .cs = SD_CS
    };
    init_results[2] = sdlog_init(&sd_config);
    if (init_results[2]) {
        ESP_LOGI(TAG, "SD card OK");
        sdlog_printf("=== SYSTEM BOOT ===");
        sdlog_printf("Wake reason: %s", wake_reason);
        sdlog_printf("Build: %s %s", __DATE__, __TIME__);
    } else {
        ESP_LOGW(TAG, "SD card failed - no logging");
        status_led_set_mode(LED_ERROR); 
        vTaskDelay(pdMS_TO_TICKS(2000));
        status_led_set_mode(LED_STARTUP);
    }

    // === Initialize GPS ===
    ESP_LOGI(TAG, "Starting GPS system...");
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
    if (!init_results[3]) {
        ESP_LOGE(TAG, "GPS failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("GPS failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "CRITICAL: System halted");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "GPS OK");

    // === Initialize Motors ===
    ESP_LOGI(TAG, "Starting motor control...");
    motor_cfg_t motor_config = {
        .az_pwm_pin = MOTOR_AZ_PWM, .az_dir_pin = MOTOR_AZ_DIR,
        .el_pwm_pin = MOTOR_EL_PWM, .el_dir_pin = MOTOR_EL_DIR,
        .stroke_mm = 200.0,
        .speed_mm_per_s = 11.94,       // Measured: 200mm ÷ 16.7s = 11.94 mm/s
        .max_az_deg = 270,
        .max_el_deg = 85,
        .min_el_deg = 10
    };
    ret = motor_init(&motor_config);
    init_results[4] = (ret == ESP_OK);
    if (!init_results[4]) {
        ESP_LOGE(TAG, "Motors failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("Motors failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "CRITICAL: System halted");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Motors OK");

    // === Initialize Button ===
    ESP_LOGI(TAG, "Starting button interface...");
    button_cfg_t button_config = {
        .gpio = START_BTN_GPIO,
        .active_low = true,
        .pull_up = true,
        .pull_down = false,
        .debounce_ms = 50
    };
    ret = button_init(&button_config);
    init_results[5] = (ret == ESP_OK);
    if (!init_results[5]) {
        ESP_LOGE(TAG, "Button failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("Button failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "CRITICAL: System halted");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Button OK");

    // === Initialize WiFi Access Point ===
    ESP_LOGI(TAG, "Starting WiFi AP for LCD display...");
    ret = wifi_comm_init_ap();
    init_results[6] = (ret == ESP_OK);
    if (!init_results[6]) {
        ESP_LOGW(TAG, "WiFi failed - no remote display (tracker will still work)");
    } else {
        ESP_LOGI(TAG, "WiFi OK - AP 'SunflowerTracker' ready for LCD display");
    }

    // === Check If We Should Auto-Start ===
    bool auto_start = (wake_cause == ESP_SLEEP_WAKEUP_TIMER);
    
    if (!auto_start) {
        ESP_LOGI(TAG, "=== WAITING FOR START BUTTON ===");
        ESP_LOGI(TAG, "Press button to start tracking");
        ESP_LOGI(TAG, "Or hold button 3s to calibrate");
        
        status_led_set_mode(LED_WAITING);
        sdlog_printf("Waiting for button press");
        
        ESP_LOGI(TAG, "Waiting...");
        button_wait_for_press(-1);
        
        sdlog_printf("Button pressed - starting");
        ESP_LOGI(TAG, "Button pressed!");
        
    } else {
        ESP_LOGI(TAG, "=== AUTO-START ===");
        ESP_LOGI(TAG, "Timer wake - starting immediately");
        sdlog_printf("Timer wake - auto-start");
    }

    // === Start Tracking System ===
    ESP_LOGI(TAG, "=== STARTING TRACKING ===");
    
    tracking_start();
    status_led_set_mode(LED_TRACKING);
    
    ESP_LOGI(TAG, "Tracking active");
    sdlog_printf("Tracking started");

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
        // Get current time
        time_t now = time(NULL);
        uint16_t uptime_h = (uint16_t)((now - boot_time) / 3600);
        
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
        
        // Get GPS data
        gps_data_t gps;
        bool gps_valid = gps_get_last(&gps) && gps.valid;
        
        // Get sun position (already calculated by tracking module)
        sun_pos_t sun = solar_compute(
            gps_valid ? gps.latitude : 0.0, 
            gps_valid ? gps.longitude : 0.0, 
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
        
        // TODO: Replace with real battery ADC reading
        uint16_t battery_adc_raw = 2800;  // Placeholder (12-bit ADC)
        float battery_v = 13.2f;          // Placeholder voltage
        
        // Prepare data packet for LCD display (matches LCD's expected format exactly)
        tracker_data_t tx_data = {
            .elevation = current_elevation,
            .azimuth = current_azimuth,
            .delta_elevation = delta_elev,
            .delta_azimuth = delta_az,
            .battery_adc = battery_adc_raw,
            .battery_voltage = battery_v,
            .timestamp = (uint32_t)now,
            .status = system_status,
            .latitude = gps_valid ? (float)gps.latitude : 0.0f,
            .longitude = gps_valid ? (float)gps.longitude : 0.0f,
            .gps_valid = gps_valid ? 1 : 0,
            .gps_satellites = gps_valid ? gps.num_satellites : 0,
            .sun_elevation = gps_valid ? (float)sun.elevation_deg : 0.0f,
            .sun_azimuth = gps_valid ? (float)sun.azimuth_deg : 0.0f,
            .moves_today = moves_today,
            .total_moves = total_moves,
            .uptime_hours = uptime_h,
            .wifi_rssi = rssi,
            .tracking_quality = tracking_quality,
        };
        
        // Send data over WiFi to LCD display
        esp_err_t send_ret = wifi_comm_send_data(&tx_data);
        if (send_ret == ESP_OK) {
            ESP_LOGI(TAG, "LCD: El=%.1f° Az=%.1f° Sun[%.1f°,%.1f°] Err=%d° Moves=%lu/%lu",
                     tx_data.elevation, tx_data.azimuth,
                     tx_data.sun_elevation, tx_data.sun_azimuth,
                     tx_data.tracking_quality,
                     tx_data.moves_today, tx_data.total_moves);
            connection_wait_counter = 0;
        } else if (send_ret == ESP_ERR_NOT_FOUND) {
            connection_wait_counter++;
            if (connection_wait_counter % 10 == 0) {
                ESP_LOGW(TAG, "Waiting for LCD display to connect... (%lu s)", connection_wait_counter);
            }
        } else {
            ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(send_ret));
        }
        
        // Update once per second (1 Hz transmission rate)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}