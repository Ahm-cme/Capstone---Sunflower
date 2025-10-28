#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
-------------------------------------------------------------------------------
LCD Module (ILI9486 over SPI) - Public API
-------------------------------------------------------------------------------
Responsibilities:
- Expose a simple dashboard/UI API to the rest of the app (init, splash,
  init checklist, error banner, dashboard draw + incremental updates).
- Provide minimal drawing primitives for internal rendering (rect, line, text).

Notes:
- Coordinates are 0..(TFT_WIDTH-1) x 0..(TFT_HEIGHT-1) in landscape.
- All drawing is synchronous/blocking; call from a UI/task context.
- Thread-safety: this module is not thread-safe; serialize access.
-------------------------------------------------------------------------------
*/

// Display dimensions (landscape). Change if rotation is altered in lcd.c.
#define TFT_WIDTH   480
#define TFT_HEIGHT  320

// RGB565 Colors - curated palette used by the UI
// Tip: Keep functional names (status/panels) for consistency across screens.
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_NAVY        0x000F      // Dark blue (unused placeholder)
#define TFT_CHARCOAL    0x3186      // Header background
#define TFT_STEEL       0x6B4D      // Panel background
#define TFT_SILVER      0xC618      // Neutral text
#define TFT_SLATE       0x9CF3      // Borders/grid
#define TFT_DARKGREY    0x5ACB      // Battery panel fill
#define TFT_GOLDEN      0xFEA0      // Accent (titles/time/voltage)
#define TFT_AMBER       0xFD20      // Standby/sleeping/calibrating/status amber
#define TFT_SUNGLOW     0xFFE0      // Azimuth numeric
#define TFT_TEAL        0x07FF      // Delta values / GPS text
#define TFT_MINT        0x87F0      // Secondary info (e.g., version)
#define TFT_SAGE        0x07E0      // Good/OK/Tracking
#define TFT_LIME        0x07E0      // Alias of SAGE (same code)
#define TFT_AZURE       0x001F      // Blue status (reserved)
#define TFT_CORAL       0xFBEA      // Warning tones
#define TFT_CRIMSON     0xF800      // Error/critical

// Display data structure used to render the dashboard.
// Populate this from your data source (WiFi, test stubs, etc.).
// tracking_status semantics:
//   0=STANDBY (amber), 1=TRACKING (green), 2=SLEEPING (amber),
//   3=CALIBRATING (amber), 255=ERROR (red)
typedef struct {
    // Panel position data
    float elevation;              // deg (0..90)
    float azimuth;                // deg (0..360)
    float delta_elevation;        // deg/s (displayed as D:…)
    float delta_azimuth;          // deg/s

    // Power/battery
    uint16_t battery_adc;         // Raw ADC (0..4095)
    float battery_voltage;        // Volts (derived/scaled)

    // Time/system
    uint32_t timestamp;           // Seconds (used for HH:MM:SS in header)
    uint8_t tracking_status;      // See mapping above

    // GPS/position
    float latitude;               // deg (-90..+90)
    float longitude;              // deg (-180..+180)
    bool gps_valid;               // true if GPS fix OK

    // Optional fields (not strictly required by renderer yet)
    uint8_t time_hour;            // If you prefer explicit time fields
    uint8_t time_min;
    bool motors_active;           // For future indicators
} lcd_display_data_t;

/*
===============================================================================
High-level UI API
-------------------------------------------------------------------------------
These are the functions you call from app_main / UI task.
===============================================================================
*/

// Initialize LCD hardware and internal state (SPI, panel, defaults).
// Returns ESP_OK on success; logs details on failure.
esp_err_t lcd_init(void);

// Show a startup checklist screen.
// status: array of item labels; success: array of bools; count: number of items.
// Draws green check or red X per item.
void lcd_show_init_screen(const char *status[], bool success[], int count);

// Show centered splash with optional message (e.g., "System Starting...").
void lcd_show_splash(const char *message);

// Show a red error banner at the bottom with given message.
void lcd_show_error(const char *error_msg);

// Full dashboard draw (header + panels + graph). Use once at start.
void lcd_draw_dashboard(const lcd_display_data_t *data);

// Incremental dashboard update (updates numeric/time/status/graph).
// Call periodically with fresh data (e.g., at 5–10 Hz or on data arrival).
void lcd_update_display(const lcd_display_data_t *data);

// Clear entire screen to a solid color (used by splash/init screens).
void lcd_clear_screen(uint16_t color);

/*
===============================================================================
Low-level drawing primitives
-------------------------------------------------------------------------------
Primarily used internally; kept public for potential reuse.
All coordinates are inclusive pixel positions in screen space.
===============================================================================
*/

// Fill rectangle (clips to screen). w/h are in pixels.
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

// Draw ASCII text using built-in 5x7 font scaled by 'size'.
// Background is actively drawn to avoid artifacts/ghosting.
void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size);

// Bresenham line between (x0,y0) and (x1,y1) inclusive.
void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

// Rectangle outline using 4 lines.
void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

// Single pixel write (clipped).
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

#endif // LCD_H