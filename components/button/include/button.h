#pragma once
#include <stdbool.h>
#include "esp_err.h"

/*
    Button Driver (single, polled)

    Overview:
    - One user button (START/CALIBRATE).
    - Polled, debounced in software; no interrupts.
    - Global singleton configuration.

    Behavior:
    - Debounce on press only (release is immediate).
    - Polling-friendly API for FreeRTOS tasks.

    Wiring (recommended):
      GPIO ---[momentary button]--- GND
      active_low = true, pull_up = true, pull_down = false

    ESP32 notes:
    - GPIO 34..39 are input-only; no internal pulls.
    - Avoid boot strap pins (0, 2, 12, 15).
*/
typedef struct {
    int  gpio;         // Button GPIO.
    bool active_low;   // true: pressed = 0; false: pressed = 1.
    bool pull_up;      // Enable internal pull-up (ignored on 34..39).
    bool pull_down;    // Enable internal pull-down (ignored on 34..39).
    int  debounce_ms;  // Required stable time for press detection.
} button_cfg_t;

/*
    Initialize the button GPIO and pulls.

    Returns:
      ESP_OK on success.
*/
esp_err_t button_init(const button_cfg_t *cfg);

/*
    Read instantaneous pressed state (no debounce).
    Non-blocking snapshot suitable for UI loops.
*/
bool button_is_pressed(void);

/*
    Wait for a debounced press edge, with timeout.

    Press edge definition:
      1) Ensure button was released first.
      2) Detect pressed.
      3) Verify pressed remains after debounce_ms.

    Params:
      timeout_ms < 0 → wait forever.

    Returns:
      true  -> press detected
      false -> timeout
*/
bool button_wait_for_press(int timeout_ms);

/*
    Block until the button is pressed and held continuously for hold_ms, or a timeout.
    - Used for "long-press to calibrate mount offset".
    
    Returns:
      true  -> held for full duration
      false -> released early or timeout
*/
bool button_wait_for_long_press(int hold_ms, int timeout_ms);