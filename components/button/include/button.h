#pragma once
#include <stdbool.h>
#include "esp_err.h"

/*
    Button (single) – simple polled driver for ESP-IDF (no interrupts)

    Why this exists:
    - We only need one user button (START / CALIBRATE) and we want deterministic,
      low-overhead debouncing without ISR complexity. This module centralizes the
      wiring assumptions and debounce logic.

    Design constraints / reminders (me-to-me):
    - No ISR: Everything is polled from a FreeRTOS task with short vTaskDelay()
      yields to avoid burning CPU. This is “good enough” for human input.
    - Global singleton: Keep it simple. One instance is enough for this project.
      If someday we need more buttons, copy this into a tiny array-backed driver.
    - Debounce: Coarse but robust. We require a stable pressed level after
      debounce_ms. Release is not debounced (intentionally) to keep latency low.
    - Ticks vs ms: We accept ms in the API for readability and convert to ticks
      inside. Tick rate is usually 100 Hz (10 ms) on ESP-IDF; don’t expect 1 ms
      resolution.

    Hardware wiring (preferred):
      GPIO ----[momentary button]---- GND
      cfg.active_low = true
      cfg.pull_up    = true   (enable internal pull-up)
      cfg.pull_down  = false

    ESP32 pin caveats:
    - Inputs 34..39 are input-only and DO NOT have internal pull-ups/downs.
      If you use those, provide an external pull-up and set pull_up=false here.
    - Avoid boot strap pins (0, 2, 12, 15) to prevent weird boot modes.
    - For ESP32-CAM style boards, GPIO4 is often tied to the flash LED – avoid
      sharing with the button.

    Concurrency:
    - Reads the same global state from multiple tasks safely (read-only).
      Do not re-call button_init from multiple places.

    Future wish list:
    - Optional long-press/short-press double-click detector with time windows.
    - ISR + queue variant if we ever need ultra-low-latency or wake-from-light-sleep.
*/

typedef struct {
    int  gpio;         // GPIO number connected to the button.
    bool active_low;   // true: pressed = logic 0; false: pressed = logic 1.
    bool pull_up;      // enable internal pull-up (ignored by input-only pins 34..39).
    bool pull_down;    // enable internal pull-down (ignored by input-only pins 34..39).
    int  debounce_ms;  // minimum stable time to accept a press after detection.
} button_cfg_t;

/*
    Initialize the button GPIO with the requested pulls.
    - Idempotent in practice (re-calls just reconfigure the pin).
    - Returns ESP_OK on success; logs are in the .c if we ever add them.
*/
esp_err_t button_init(const button_cfg_t *cfg);

/*
    Return current pressed state (reads the GPIO once and applies active_low).
    - Non-blocking.
    - No debounce: this is an instantaneous snapshot, useful for UI loops.
*/
bool button_is_pressed(void);

/*
    Block the calling task until a debounced "press edge" occurs or timeout elapses.
    Definition of “press edge”:
      1) We first observe the button released (prevents instant success if
         the user booted while holding the button).
      2) We then observe pressed.
      3) We wait debounce_ms and recheck pressed to ensure a stable press.

    timeout_ms < 0 means wait forever.
    Returns:
      true  -> press detected
      false -> timed out
*/
bool button_wait_for_press(int timeout_ms);

/*
    Block until the button is pressed and held continuously for hold_ms, or a timeout.
    - Used for “long-press to calibrate mount offset”.*/