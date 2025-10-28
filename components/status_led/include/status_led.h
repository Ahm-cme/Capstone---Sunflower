#pragma once
#include <stdbool.h>

/*
    Status LED Driver

    Purpose:
    - Background task drives simple patterns indicating system state.

    Notes:
    - Works with active-high or active-low wiring.
    - Non-blocking; patterns loop until mode changes.
    - Keep GPIO off boot-strap pins (0, 2, 12, 15) and flash pins (6–11).
*/

typedef enum {
    LED_STARTUP,    // Fast blink: system initializing
    LED_WAITING,    // Solid on: waiting for GPS/user
    LED_TRACKING,   // Slow pulse: normal operation
    LED_ERROR,      // Rapid blink: fault condition
    LED_SLEEP       // Off: deep sleep
} status_led_mode_t;

/*
    Initialize LED GPIO and start the pattern task.

    Params:
    - gpio_num: LED GPIO (output-capable)
    - active_high: true if GPIO=1 turns LED on

    Returns:
    - true on success, false on failure
*/
bool status_led_init(int gpio_num, bool active_high);

/*
    Set current LED mode (thread-safe).
    Change takes effect on next pattern cycle.
*/
void status_led_set_mode(status_led_mode_t mode);

/*
    Get current LED mode (may lag LED by one cycle).
*/
status_led_mode_t status_led_get_mode(void);