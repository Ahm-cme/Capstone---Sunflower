#include "status_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/*
    Status LED Implementation

    Patterns (timing chosen for outdoor visibility and low power):
    - LED_STARTUP  : 250 ms on, 250 ms off (2 Hz)
    - LED_WAITING  : Solid on
    - LED_TRACKING : 200 ms on, 1800 ms off (~10% duty)
    - LED_ERROR    : 100 ms on, 100 ms off (5 Hz)
    - LED_SLEEP    : Off

    Notes:
    - Works with active-high and active-low wiring.
    - A background task loops patterns until mode changes.
    - s_mode is volatile; writes are atomic in this use.
*/

#define TAG "STATUS_LED"

// Module state
static int s_led_gpio = -1;                            // LED GPIO (-1 until initialized)
static bool s_active_high = true;                      // True: GPIO=1 turns LED on
static volatile status_led_mode_t s_mode = LED_SLEEP;  // Current mode (read by task)

/*
    Drive the LED considering polarity.
    on=true  -> LED should light
    on=false -> LED should turn off
*/
static inline void led_write(bool on){
    if (s_led_gpio < 0) return;                        // Ignore if not initialized
    int level = on ? 1 : 0;                            // Logical level
    if (!s_active_high) level = !level;                // Invert for active-low wiring
    gpio_set_level(s_led_gpio, level);                 // Apply to pin
    ESP_LOGV(TAG, "LED %s (GPIO%d=%d)", on ? "ON" : "OFF", s_led_gpio, level);
}

/*
    Background task generating LED patterns based on s_mode.
    Runs forever; each case performs one pattern cycle.
*/
static void pattern_task(void *arg){
    ESP_LOGI(TAG, "LED pattern task started");

    for(;;){
        status_led_mode_t current_mode = s_mode;       // Sample once per loop

        switch(current_mode){
            case LED_STARTUP:
                // 2 Hz blink
                ESP_LOGD(TAG, "Pattern: STARTUP");
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(250));
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(250));
                break;

            case LED_WAITING:
                // Solid on (yield periodically)
                ESP_LOGD(TAG, "Pattern: WAITING");
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            case LED_TRACKING:
                // Short pulse, long off (low power heartbeat)
                ESP_LOGD(TAG, "Pattern: TRACKING");
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(200));
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(1800));
                break;

            case LED_ERROR:
                // Rapid blink (urgent)
                ESP_LOGD(TAG, "Pattern: ERROR");
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case LED_SLEEP:
            default:
                // Off
                if (current_mode != LED_SLEEP) {
                    ESP_LOGW(TAG, "Unknown mode %d -> SLEEP", current_mode);
                }
                ESP_LOGD(TAG, "Pattern: SLEEP");
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
    }

    // Not expected to exit
    ESP_LOGE(TAG, "LED pattern task exited unexpectedly");
    vTaskDelete(NULL);
}

/*
    Configure GPIO and start the pattern task.
*/
bool status_led_init(int gpio_num, bool active_high){
    ESP_LOGI(TAG, "Init LED: GPIO%d (active-%s)", gpio_num, active_high ? "high" : "low");

    s_led_gpio = gpio_num;                              // Save GPIO
    s_active_high = active_high;                        // Save polarity

    gpio_config_t io_config = {                         // Configure as push-pull output
        .pin_bit_mask = 1ULL << s_led_gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_config);            // Apply GPIO config
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        s_led_gpio = -1;                                // Mark unusable
        return false;
    }

    led_write(false);                                   // Start off
    ESP_LOGD(TAG, "LED configured OFF");

    BaseType_t t = xTaskCreate(                        // Spawn pattern task
        pattern_task, "status_led", 2048, NULL, 4, NULL);
    if (t != pdPASS) {
        ESP_LOGE(TAG, "Task create failed");
        return false;
    }

    ESP_LOGI(TAG, "Status LED ready");
    return true;
}

/*
    Change the current LED mode (logged on change).
*/
void status_led_set_mode(status_led_mode_t mode){
    if (s_mode != mode) {
        const char* names[] = {"STARTUP","WAITING","TRACKING","ERROR","SLEEP"};
        const char* oldn = (s_mode < 5) ? names[s_mode] : "UNKNOWN";
        const char* newn = (mode   < 5) ? names[mode]   : "UNKNOWN";
        ESP_LOGI(TAG, "Mode: %s -> %s", oldn, newn);
        s_mode = mode;                                  // Commit new mode
    }
}

/*
    Read current mode (as seen by next pattern cycle).
*/
status_led_mode_t status_led_get_mode(void){
    return s_mode;
}