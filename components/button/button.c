#include "button.h"                 
#include "driver/gpio.h"          
#include "freertos/FreeRTOS.h"    
#include "freertos/task.h"        

/*
    Module notes (internal):
    - Keep this dead simple. Single global config, no dynamic allocation.
    - We intentionally avoid logging here to keep the path quiet. The caller
      knows when it's waiting for input and can log context-sensitive messages.
    - Debounce is "press-only": we require stability when entering pressed, but
      we do not debounce release (makes long-press more responsive).
    - CPU usage: Loops sleep in 10 ms increments; good balance vs responsiveness.
*/

/*
    Internal copy of the configuration.
    We keep a single global button instance for simplicity.
*/
static button_cfg_t s_cfg;          // Global singleton config used by all helpers

/*
    Helper: convert raw GPIO level to a boolean "pressed" according to active_low.
    - If active_low == true, level 0 means pressed.
    - If active_low == false, level 1 means pressed.
*/
static inline bool level_pressed(int level){         // Map electrical level -> logical pressed
    return s_cfg.active_low ? (level == 0) : (level != 0);  // Interpret based on polarity
}

/*
    Configure the GPIO as an input with desired pull resistors.
    Note: On pins 34..39 the pull settings are ignored by hardware; use external resistors.
    Returns ESP_OK on success.
*/
esp_err_t button_init(const button_cfg_t *cfg){      // Initialize the button GPIO and store config
    s_cfg = *cfg;                                    // Copy caller's config into module state

    gpio_config_t io = {                             // Build GPIO configuration for input + pulls
        .pin_bit_mask = 1ULL << s_cfg.gpio,          // Select the target GPIO
        .mode         = GPIO_MODE_INPUT,             // Input mode (polled)
        .pull_up_en   = s_cfg.pull_up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,   // Optional pull-up
        .pull_down_en = s_cfg.pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE, // Optional pull-down
        .intr_type    = GPIO_INTR_DISABLE            // No interrupts (pure polling)
    };
    return gpio_config(&io);                         // Apply configuration to the hardware
}

/*
    Read the pin and report whether the button is currently pressed.
    This is a raw snapshot (no debounce).
*/
bool button_is_pressed(void){                        // Instantaneous pressed state
    int lvl = gpio_get_level(s_cfg.gpio);            // Read raw logic level from GPIO
    return level_pressed(lvl);                       // Convert to logical "pressed" according to polarity
}

/*
    Wait for a debounced press edge:
      1) Ensure we start from the "released" state (prevents auto-trigger if already held).
      2) Wait until the signal indicates "pressed".
      3) Delay for debounce_ms and re-check to confirm stability.

    Implementation details & cautions:
    - We measure timeout relative to the first call. Intermediate sleeps consume
      the budget; good enough for human interaction.
    - If debounce_ms == 0, we still perform the re-check immediately to catch
      extremely short bounces.
    - We poll every ~10 ms. If sdkconfig tickrate changes, adjust if needed.
*/
bool button_wait_for_press(int timeout_ms){          // Block until a debounced press or timeout
    TickType_t start = xTaskGetTickCount();          // Mark start time for timeout accounting
    TickType_t tmo   = (timeout_ms < 0) ?            // Convert ms to ticks, or infinite wait
                       portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // Phase 1: wait until released (edge qualification)
    for(;;){                                         // Loop until we see a released state
        if (tmo != portMAX_DELAY &&                  // Timeout active?
            (xTaskGetTickCount() - start) >= tmo)    // Exhausted time budget?
            return false;                            // Give up on timeout
        if (!button_is_pressed()) break;             // Already released → proceed to wait for press
        vTaskDelay(pdMS_TO_TICKS(10));               // Back off to reduce CPU usage
    }

    // Phase 2: wait for press, then debounce
    for(;;){                                         // Loop until debounced press or timeout
        if (tmo != portMAX_DELAY &&                  // Timeout active?
            (xTaskGetTickCount() - start) >= tmo)    // Exhausted time budget?
            return false;                            // Give up on timeout

        if (button_is_pressed()){                    // Detected a press edge
            vTaskDelay(pdMS_TO_TICKS(s_cfg.debounce_ms)); // Debounce window (press-only)
            if (button_is_pressed()) return true;    // Still pressed after debounce → success
            // If it bounced back to released, keep waiting
        }

        vTaskDelay(pdMS_TO_TICKS(10));               // Polling cadence
    }
}

/*
    Wait for a "long press":
      - Block until the button becomes pressed.
      - Measure continuous press duration.
      - If still pressed after hold_ms, succeed; otherwise, fail and return to caller.

    UX notes:
    - This does not "debounce" the initial transition beyond the 10 ms polling pace.
      If needed, add a short fixed debounce here too (not required in practice).
    - For calibration, we intentionally want it to be forgiving: if the user
      slightly jitters but maintains the hold, we still count it.

    Power/latency:
    - 10 ms polling is a sweet spot. If we ever aim for light sleep while waiting
      for input, consider GPIO wake + ISR variant instead.
*/
bool button_wait_for_long_press(int hold_ms, int timeout_ms){ // Block until held for hold_ms or timeout
    TickType_t start = xTaskGetTickCount();          // Start time for overall timeout
    TickType_t tmo   = (timeout_ms < 0) ?            // Convert ms to ticks, or infinite
                       portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // Wait for initial press
    while (!button_is_pressed()){                    // Spin until first press detected
        if (tmo != portMAX_DELAY &&                  // Timeout active?
            (xTaskGetTickCount() - start) >= tmo)    // Exhausted time budget?
            return false;                            // Timed out before press
        vTaskDelay(pdMS_TO_TICKS(10));               // Poll at ~10 ms
    }

    // Measure continuous hold duration
    TickType_t t0 = xTaskGetTickCount();             // Timestamp when press started
    while (button_is_pressed()){                     // Stay in loop while button held
        if (pdTICKS_TO_MS(xTaskGetTickCount() - t0)  // Elapsed hold time (ms)
            >= hold_ms) return true;                 // Reached required hold duration → success
        vTaskDelay(pdMS_TO_TICKS(10));               // Polling cadence for hold check
    }

    // Released too early
    return false;                                    // Did not reach hold_ms before release
}