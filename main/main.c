/*
 * Enhanced GPS-Based Solar Tracking System
 * ESP32 + Dual Linear Actuators
 * 
 * ════════════════════════════════════════════════════════════════════════
 * ║  HARDCODED_LOCATION BRANCH - TESTING CONFIGURATION                   ║
 * ════════════════════════════════════════════════════════════════════════
 * ║  Location:        Auburn, AL (32.5990°N, 85.4808°W, 200m elev)      ║
 * ║  Date/Time:       October 15, 2025, 2:30 PM CST                     ║
 * ║  System Base:     North-facing (0° azimuth reference)               ║
 * ║  Panel Position:  Facing STRAIGHT UP (zenith, 90° elevation)        ║
 * ║  Panel Plane:     Vertical from ground (normal to horizontal)       ║
 * ║  GPS Hardware:    NOT REQUIRED - using fixed coordinates            ║
 * ════════════════════════════════════════════════════════════════════════
 * 
 * Physical Configuration:
 *   - System base/frame oriented with "front" pointing North (0°)
 *   - Panel starts in vertical position (plane facing skyward)
 *   - Panel normal vector initially points to zenith (90° elevation)
 *   - Actuators tilt panel from this vertical reference position
 *   - Azimuth rotation: clockwise from north (0°=N, 90°=E, 180°=S, 270°=W)
 *   - Elevation range: 10° (near-horizontal) to 90° (vertical/zenith)
 * 
 * At the configured time (2:30 PM, October 15, 2025), Auburn AL sun position:
 *   - Sun Azimuth: ~205° (Southwest)
 *   - Sun Elevation: ~40° (mid-afternoon, moderate angle)
 *   - Required panel tilt: FROM vertical (90°) TO 40° elevation
 *   - Required panel rotation: FROM north (0°) TO southwest (205°)
 *   - Panel will tilt DOWN 50° from vertical and rotate 205° clockwise
 * 
 * The tracker will:
 *   1. Calculate sun position relative to Auburn, AL at 2:30 PM
 *   2. Compute required actuator movements from vertical starting position
 *   3. Move elevation actuator to tilt panel down to 40° from horizontal
 *   4. Move azimuth actuator to rotate panel to 205° (southwest-facing)
 *   5. Panel will be perpendicular to sun rays for maximum power output
 * 
 * ┌───────────────────────────────────────────────────────────────────────┐
 * │ System Architecture Overview                                          │
 * │                                                                       │
 * │ Hardware Stack:                                                       │
 * │  ESP32-CAM + MAX-M10S GPS + MD20A Motor Drivers + 12V Linear Acts     │
 * │  200mm stroke actuators @ 11.94 mm/s nominal speed                   │
 * │  MicroSD logging + Status LED + Start/Calibrate button               │
 * │                                                                       │
 * │ Software Stack:                                                       │
 * │  ├── Tracking Controller (main coordination, sleep management)       │
 * │  ├── GPS Module (I2C, time sync, position acquisition)               │
 * │  ├── Solar Calculator (NOAA algorithms, sunrise/sunset)              │
 * │  ├── Motor Control (PWM + DIR, time-based positioning)               │
 * │  ├── SD Logging (CSV data + human readable logs)                     │
 * │  └── Status LED (visual feedback for remote monitoring)              │
 * │                                                                       │
 * │ Power Profile (12V battery):                                         │
 * │  ├── Deep Sleep: ~10-50µA (RTC + wake timer only)                    │
 * │  ├── Active Tracking: ~150-300mA (GPS + CPU + peripherals)          │
 * │  ├── Motor Moves: ~500-1000mA (brief, 5-10s duration)               │
 * │  └── Daily Average: ~20-40mA (depends on tracking frequency)         │
 * │                                                                       │
 * │ Deployment Workflow:                                                  │
 * │  1. Flash firmware, install hardware, insert SD card                 │
 * │  2. Power on, wait for GPS fix (LED_WAITING)                         │
 * │  3. Manually align panel to sun, long-press button (calibration)     │
 * │  4. Press start button to begin autonomous tracking                   │
 * │  5. System operates independently with nightly homing cycles         │
 * └───────────────────────────────────────────────────────────────────────┘
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"  // Add this for system functions
#include "nvs_flash.h"
#include "gps.h"
#include "motor.h"
#include "sdlog.h"
#include "tracking.h"
#include "status_led.h"
#include "button.h"
#include "esp_sleep.h"
#include "driver/i2c.h"

#define TAG "APP"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Hardware Configuration                                                │
    │                                                                       │
    │ Pin assignments optimized for ESP32-CAM board compatibility:         │
    │  - Avoids GPIO 0,1,3 (UART/boot)                                     │
    │  - Avoids GPIO 6-11 (flash)                                          │
    │  - Works around GPIO 2,12,13,14,15 (SD card)                         │
    │  - Uses available GPIO 18,19,25,26,27,32,33                          │
    │                                                                       │
    │ I2C Bus (GPS):                                                        │
    │  - SDA=18, SCL=19: Safe pins with good drive strength                │
    │  - 400kHz: Fast enough for GPS without EMI issues                    │
    │  - GPS addr 0x42: MAX-M10S default I2C address                       │
    │                                                                       │
    │ Motor Control (PWM + DIR):                                           │
    │  - Uses LEDC hardware PWM (5kHz, 13-bit resolution)                  │
    │  - GPIO 32,33,26,27: High-current capable pins                       │
    │  - DIR pins control MD20A driver direction (extend/retract)          │
    │                                                                       │
    │ SD Card (SPI Mode):                                                   │
    │  - Standard ESP32-CAM pinout (MOSI=15, MISO=2, SCK=14, CS=13)        │
    │  - GPIO 2 is boot strap: remove SD during firmware flashing          │
    │  - GPIO 15 is boot strap: avoid pulling low during boot              │
    │                                                                       │
    │ User Interface:                                                       │
    │  - Status LED: GPIO 4 (ESP32-CAM built-in flash LED)                 │
    │  - Start Button: GPIO 25 (safe, available pin)                       │
    │  - Wire: GPIO → momentary switch → GND, internal pull-up enabled      │
    └───────────────────────────────────────────────────────────────────────┘
*/

// Hardware config (adjust pins to match your wiring)
// I2C NOT USED - GPS hardware not required in hardcoded location mode
// These pins (GPIO 26, 27) can be repurposed if needed
#define I2C_NUM        I2C_NUM_0
#define I2C_SDA        26     // Available (not used for GPS)
#define I2C_SCL        27     // Available (not used for GPS)
#define GPS_ADDR       0x42   // Not used

#define MOTOR_AZ_PWM   32     // Azimuth actuator PWM (high current capable)
#define MOTOR_AZ_DIR   33     // Azimuth actuator direction
#define MOTOR_EL_PWM   18     // Elevation actuator PWM
#define MOTOR_EL_DIR   19     // Elevation actuator direction

// SD card (ESP32-CAM SPI mode defaults - don't change without hardware mod)
#define SD_MOSI        15     // CMD/MOSI (boot strap pin)
#define SD_MISO        2      // D0/MISO (boot strap pin; keep default pull)
#define SD_SCLK        14     // CLK/SCK
#define SD_CS          13     // D3/CS

// User interface pins
#define STATUS_LED_GPIO 4     // ESP32-CAM built-in flash LED (active high)
#define START_BTN_GPIO 5     // Start/calibrate button (change if needed)

/*
    Calibration task: monitors button for long-press events.
    
    Calibration procedure:
    1. Manually align panel to point directly at sun (visual confirmation)
    2. Press and hold START button for 3+ seconds
    3. System calculates and stores mount offset angles
    4. Future tracking uses these offsets automatically
    
    Task design:
    - Low priority background task (doesn't interfere with tracking)
    - Polls button every 200ms (responsive but not CPU intensive)
    - Infinite loop (runs for entire system lifetime)
    - Uses blocking button API to wait for long press events
    
    Safety considerations:
    - Safe to trigger calibration anytime (overwrites previous offsets)
    - Requires valid GPS fix (function fails gracefully if no GPS)
    - SD logging provides record of calibration events for maintenance
    
    Installation workflow:
    - Best done during sunny conditions with clear sky view
    - Can be repeated if mount hardware is adjusted or relocated
    - Calibration accuracy directly affects tracking performance
*/
static void calib_task(void *arg){
    ESP_LOGI(TAG, "Calibration monitor task started");
    ESP_LOGI(TAG, "Long-press START button (3s) to calibrate mount offsets");
    
    for(;;){
        // Wait for long press event (3 second threshold, infinite timeout)
        // Check for button press, then measure hold time manually
        if (button_wait_for_press(100)) {  // 100ms timeout for polling
            // Button was pressed, now measure hold duration
            uint32_t press_start = xTaskGetTickCount();
            uint32_t hold_time = 0;
            
            // Wait while button remains pressed
            while (button_is_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
                hold_time = (xTaskGetTickCount() - press_start) * portTICK_PERIOD_MS;
                
                // If held for 3+ seconds, trigger calibration
                if (hold_time >= 3000) {
                    ESP_LOGI(TAG, "=== CALIBRATION TRIGGER ===");
                    ESP_LOGI(TAG, "Long-press detected: starting mount offset calibration");
                    
                    // Visual feedback during calibration
                    status_led_set_mode(LED_STARTUP);  // Fast blink indicates calibration active
                    
                    sdlog_printf("Long-press detected: calibrate mount offsets");
                    tracking_calibrate_mount_offset_now();
                    
                    // Return to normal tracking indication
                    status_led_set_mode(LED_TRACKING);
                    
                    ESP_LOGI(TAG, "Calibration complete, resuming normal operation");
                    
                    // Wait for button release to avoid repeat triggers
                    while (button_is_pressed()) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    break;  // Exit the hold-time measurement loop
                }
            }
        }
        
        // Brief yield to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*
    Main application entry point.
    
    Initialization sequence:
    1. Status LED (immediate visual feedback)
    2. NVS flash (persistent storage for settings)
    3. SD card logging (capture all events from boot)
    4. GPS subsystem (position and time reference)
    5. Motor control (actuator interface)
    6. User interface (button input)
    7. Wake cause analysis (timer vs manual start)
    8. Tracking system startup (main application logic)
    
    Error handling strategy:
    - Critical failures (NVS, GPS, Motors): LED_ERROR and halt
    - Optional failures (SD card): LED indication but continue operation
    - User interaction: LED_WAITING until start button pressed
    
    Boot behavior:
    - Cold boot: wait for user start button (LED_WAITING)
    - Timer wake: automatic tracking start (scheduled sunrise)
    - Error wake: attempt normal startup with error indication
    
    Memory management:
    - Static allocation preferred for stability
    - NVS handles persistent data automatically
    - FreeRTOS tasks for concurrent operation
*/
void app_main(void){
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "║  SUNFLOWER SOLAR TRACKER - HARDCODED LOCATION MODE        ║");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Firmware build: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuration Summary:");
    ESP_LOGI(TAG, "  • Location: Auburn, AL (32.5990°N, 85.4808°W)");
    ESP_LOGI(TAG, "  • Time: October 15, 2025, 2:30 PM CST");
    ESP_LOGI(TAG, "  • System base: North-facing (0° azimuth reference)");
    ESP_LOGI(TAG, "  • Panel initial: Facing UP (90° from horizontal)");
    ESP_LOGI(TAG, "  • GPS hardware: Not required (hardcoded mode)");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    
    // Get wake cause early for startup decision logic
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    const char* wake_reason = "UNKNOWN";
    switch(wake_cause){
        case ESP_SLEEP_WAKEUP_TIMER:    wake_reason = "RTC_TIMER"; break;
        case ESP_SLEEP_WAKEUP_EXT0:     wake_reason = "EXT0_GPIO"; break;
        case ESP_SLEEP_WAKEUP_EXT1:     wake_reason = "EXT1_GPIO"; break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: wake_reason = "TOUCHPAD"; break;
        case ESP_SLEEP_WAKEUP_ULP:      wake_reason = "ULP"; break;
        default:                        wake_reason = "POWER_ON"; break;
    }
    ESP_LOGI(TAG, "Wake cause: %s", wake_reason);

    // === STATUS LED INITIALIZATION ===
    // First priority: visual feedback for remote debugging
    ESP_LOGI(TAG, "Initializing status LED on GPIO%d...", STATUS_LED_GPIO);
    if (!status_led_init(STATUS_LED_GPIO, true)) {
        ESP_LOGE(TAG, "Failed to initialize status LED - continuing anyway");
    }
    status_led_set_mode(LED_STARTUP);
    ESP_LOGI(TAG, "Status LED: STARTUP pattern active");

    // === NVS FLASH INITIALIZATION ===
    // Critical: required for persistent settings and calibration data
    ESP_LOGI(TAG, "Initializing NVS flash storage...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or version mismatch, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_LOGI(TAG, "NVS flash erased and reinitialized");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        ESP_LOGE(TAG, "System halted due to NVS failure");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));  // Halt with error indication
    }
    ESP_LOGI(TAG, "NVS flash ready");

    // === SYSTEM TIME INITIALIZATION (HARDCODED) ===
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Setting system time to October 15, 2025, 3:30 PM CST...");
    struct tm timeinfo = {0};
    timeinfo.tm_year = 2025 - 1900;  // Years since 1900
    timeinfo.tm_mon = 10 - 1;        // Months since January (0-11)
    timeinfo.tm_mday = 22;           // Day of month (1-31)
    timeinfo.tm_hour = 12;           // Hour (0-23) - 12:00 PM
    timeinfo.tm_min = 0;            // Minutes (0-59)
    timeinfo.tm_sec = 0;             // Seconds (0-59)
    timeinfo.tm_isdst = 0;           // Daylight saving time flag
    
    time_t t = mktime(&timeinfo);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
    
    // Set timezone to Central Time (Auburn, AL)
    setenv("TZ", "CST6CDT,M3.2.0,M11.1.0", 1);
    tzset();
    
    char strftime_buf[64];
    localtime_r(&t, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "System time set to: %s", strftime_buf);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    
    // === SD CARD LOGGING INITIALIZATION ===
    // High priority: capture boot events and system state
    ESP_LOGI(TAG, "Initializing SD card logging...");
    sdlog_cfg_t sd_config = {
        .mosi = SD_MOSI, .miso = SD_MISO, 
        .sclk = SD_SCLK, .cs = SD_CS
    };
    if (sdlog_init(&sd_config)) {
        ESP_LOGI(TAG, "SD card logging active");
        sdlog_printf("=== SYSTEM BOOT - HARDCODED LOCATION MODE ===");
        sdlog_printf("Location: Auburn, AL (32.5990°N, 85.4808°W)");
        sdlog_printf("System time: %s (CST)", strftime_buf);
        sdlog_printf("System base: North-facing (0° azimuth)");
        sdlog_printf("Panel position: Facing UP (90° from horizontal)");
        sdlog_printf("Wake cause: %s", wake_reason);
        sdlog_printf("Build: %s %s", __DATE__, __TIME__);
    } else {
        ESP_LOGW(TAG, "SD card initialization failed - continuing without logging");
        status_led_set_mode(LED_ERROR); 
        vTaskDelay(pdMS_TO_TICKS(2000));  // Brief error indication
        status_led_set_mode(LED_STARTUP); // Continue startup
    }

    // === GPS SUBSYSTEM INITIALIZATION (STUB ONLY) ===
    ESP_LOGI(TAG, "Initializing GPS subsystem (hardcoded mode)...");
    gps_cfg_t gps_config = {
        .i2c_port = I2C_NUM,
        .sda_io = I2C_SDA,
        .scl_io = I2C_SCL,
        .clk_hz = 400000,
        .addr = GPS_ADDR
    };
    ret = gps_init(&gps_config);  // Will skip I2C init, return ESP_OK immediately
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS initialization failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("GPS init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted due to GPS failure");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "GPS subsystem ready (using hardcoded Auburn, AL location)");
    ESP_LOGI(TAG, "GPS hardware NOT required for this build");

    // === MOTOR CONTROL INITIALIZATION ===
    // Critical: required for panel positioning
    ESP_LOGI(TAG, "Initializing motor control subsystem...");
    motor_cfg_t motor_config = {
        // PWM and direction pins
        .az_pwm_pin = MOTOR_AZ_PWM, .az_dir_pin = MOTOR_AZ_DIR,
        .el_pwm_pin = MOTOR_EL_PWM, .el_dir_pin = MOTOR_EL_DIR,
        
        // Actuator specifications (UPDATED FOR FASTER TESTING)
        .stroke_mm = 200.0,         // 200mm stroke linear actuators
        .speed_mm_per_s = 11.111,   // 200mm ÷ 18s = 11.111 mm/s at PWM=255
        
        // Panel range limits (adjust to match your mechanical design)
        .max_az_deg = 270,          // Maximum azimuth range
        .max_el_deg = 85,           // Maximum elevation (avoid zenith for stability)
        .min_el_deg = 10            // Minimum elevation (avoid ground obstacles)
    };
    ret = motor_init(&motor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor initialization failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("Motor init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted due to motor failure");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Motor control ready");

    // === USER INTERFACE INITIALIZATION ===
    ESP_LOGI(TAG, "Initializing user interface (button on GPIO%d)...", START_BTN_GPIO);
    button_cfg_t button_config = {
        .gpio = START_BTN_GPIO,
        .active_low = true,         // Button pulls GPIO to GND when pressed
        .pull_up = true,            // Enable internal pull-up resistor
        .pull_down = false,         // Disable pull-down (conflicts with pull-up)
        .debounce_ms = 50           // 50ms debounce for mechanical switch
    };
    ret = button_init(&button_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Button initialization failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_ERROR);
        sdlog_printf("Button init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted due to button failure");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "User interface ready");

    // === STARTUP DECISION LOGIC ===
    bool auto_start = (wake_cause == ESP_SLEEP_WAKEUP_TIMER);
    
    if (!auto_start) {
        // Manual startup: wait for user interaction
        ESP_LOGI(TAG, "=== WAITING FOR USER START ===");
        ESP_LOGI(TAG, "System ready - press START button on GPIO%d to begin tracking", START_BTN_GPIO);
        ESP_LOGI(TAG, "Or long-press START button (3s) to calibrate mount offsets");
        
        status_led_set_mode(LED_WAITING);
        sdlog_printf("Waiting for START button on GPIO%d", START_BTN_GPIO);
        
        // Block until user presses start button
        ESP_LOGI(TAG, "Waiting for button press...");
        button_wait_for_press(-1);  // Infinite timeout
        
        sdlog_printf("START button pressed - beginning tracking operations");
        ESP_LOGI(TAG, "START button pressed - beginning tracking operations");
        
    } else {
        // Automatic startup: scheduled wake from deep sleep
        ESP_LOGI(TAG, "=== AUTOMATIC STARTUP ===");
        ESP_LOGI(TAG, "Woke from scheduled timer - starting tracking immediately");
        sdlog_printf("Timer wake: auto-starting tracking operations");
    }

    // === TRACKING SYSTEM STARTUP ===
    ESP_LOGI(TAG, "=== STARTING TRACKING SYSTEM ===");
    ESP_LOGI(TAG, "Initializing main tracking controller...");
    
    tracking_start();  // Create and start main tracking task
    status_led_set_mode(LED_TRACKING);
    
    ESP_LOGI(TAG, "Tracking system active");
    sdlog_printf("Tracking system started");

    // === CALIBRATION MONITOR STARTUP ===
    ESP_LOGI(TAG, "Starting calibration monitor task...");
    BaseType_t calib_ret = xTaskCreate(
        calib_task,                 // Task function
        "calibration_monitor",      // Task name
        2048,                       // Stack size (2KB sufficient for button handling)
        NULL,                       // Task parameters
        4,                          // Priority (lower than tracking, higher than idle)
        NULL                        // Task handle (not needed)
    );
    
    if (calib_ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create calibration task - manual calibration unavailable");
        sdlog_printf("Warning: calibration monitor task creation failed");
    } else {
        ESP_LOGI(TAG, "Calibration monitor active");
    }

    // === INITIALIZATION COMPLETE ===
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "║  SYSTEM INITIALIZATION COMPLETE                         ║");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "Solar tracker is now operational");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  • Location: Auburn, AL (32.5990°N, 85.4808°W)");
    ESP_LOGI(TAG, "  • Time: October 16, 2025, 12:00 PM CST");
    ESP_LOGI(TAG, "  • System base: North-facing (0° azimuth)");
    ESP_LOGI(TAG, "  • Panel initial: Vertical/UP (90° elevation)");
    ESP_LOGI(TAG, "  • GPS Hardware: Not required (hardcoded)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Expected Sun Position at 2:30 PM:");
    ESP_LOGI(TAG, "  • Azimuth: ~205° (Southwest)");
    ESP_LOGI(TAG, "  • Elevation: ~40° (mid-afternoon)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Expected Panel Movement:");
    ESP_LOGI(TAG, "  • From: Vertical (90° el) pointing North (0° az)");
    ESP_LOGI(TAG, "  • To: 40° elevation, 205° azimuth (Southwest)");
    ESP_LOGI(TAG, "  • Tilt: DOWN 50° from vertical");
    ESP_LOGI(TAG, "  • Rotate: 205° clockwise from north");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    
    sdlog_printf("System initialization complete - entering autonomous operation");
    sdlog_printf("Hardcoded mode: No GPS hardware needed");
    sdlog_printf("Panel starts vertical, will tilt to track sun at Az=205° El=40°");
}