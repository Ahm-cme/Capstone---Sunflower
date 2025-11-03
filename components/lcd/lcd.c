/*
 * LCD Display Client Implementation
 * 
 * Professional dashboard for Sunflower Solar Tracker
 * Displays real-time tracking data from main tracker unit
 */

#include "lcd.h"
#include "sunflower_logo.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define TAG "LCD"

// ILI9486 Commands
#define ILI9486_SWRESET     0x01
#define ILI9486_SLPOUT      0x11
#define ILI9486_NORON       0x13
#define ILI9486_DISPON      0x29
#define ILI9486_CASET       0x2A
#define ILI9486_PASET       0x2B
#define ILI9486_RAMWR       0x2C
#define ILI9486_MADCTL      0x36
#define ILI9486_PIXFMT      0x3A
#define ILI9486_FRMCTR1     0xB1
#define ILI9486_INVCTR      0xB4
#define ILI9486_DFUNCTR     0xB6
#define ILI9486_PWCTR1      0xC0
#define ILI9486_PWCTR2      0xC1
#define ILI9486_PWCTR3      0xC2
#define ILI9486_VMCTR1      0xC5
#define ILI9486_GMCTRP1     0xE0
#define ILI9486_GMCTRN1     0xE1

// 5x7 Bitmap Font (ASCII 32-126)
const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
};

// Private variables
static spi_device_handle_t spi_device;
static int dc_pin, rst_pin, backlight_pin;
static uint16_t voltage_history[100] = {0};
static int history_index = 0;
static tracker_data_t last_data = {0};
static bool full_redraw_needed = true;

// === Low-Level SPI Functions ===

static void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
    gpio_set_level(dc_pin, (int)t->user);
}

static void lcd_cmd(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device, &t));
}

static void lcd_data(uint8_t data) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .user = (void*)1
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device, &t));
}

static void lcd_data_buf(const uint8_t *data, int len) {
    if (len == 0) return;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device, &t));
}

static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_cmd(ILI9486_CASET);
    lcd_data(x0 >> 8); lcd_data(x0 & 0xFF);
    lcd_data(x1 >> 8); lcd_data(x1 & 0xFF);
    lcd_cmd(ILI9486_PASET);
    lcd_data(y0 >> 8); lcd_data(y0 & 0xFF);
    lcd_data(y1 >> 8); lcd_data(y1 & 0xFF);
    lcd_cmd(ILI9486_RAMWR);
}

// === Drawing Primitives ===

static void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || w <= 0 || h <= 0) return;
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    lcd_set_addr_window(x, y, x + w - 1, y + h - 1);
    
    uint8_t *line = malloc(w * 2);
    for (int i = 0; i < w; i++) {
        line[i * 2] = color >> 8;
        line[i * 2 + 1] = color & 0xFF;
    }
    
    gpio_set_level(dc_pin, 1);
    for (int i = 0; i < h; i++) {
        lcd_data_buf(line, w * 2);
    }
    free(line);
}

static void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    lcd_set_addr_window(x, y, x, y);
    lcd_data(color >> 8);
    lcd_data(color & 0xFF);
}

static void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0), dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        lcd_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

static void lcd_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    lcd_draw_line(x, y, x + w - 1, y, color);
    lcd_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
    lcd_draw_line(x + w - 1, y + h - 1, x, y + h - 1, color);
    lcd_draw_line(x, y + h - 1, x, y, color);
}

static void lcd_draw_char(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if (c < ' ' || c > 'z') c = '?';
    int index = c - ' ';
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[index][i];
        for (int j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) lcd_draw_pixel(x + i, y + j, color);
                else lcd_fill_rect(x + i * size, y + j * size, size, size, color);
            } else if (bg != color) {
                if (size == 1) lcd_draw_pixel(x + i, y + j, bg);
                else lcd_fill_rect(x + i * size, y + j * size, size, size, bg);
            }
            line >>= 1;
        }
    }
}

static void lcd_draw_string(int16_t x, int16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size) {
    int cursor_x = x;
    while (*str) {
        if (*str == '\n') {
            cursor_x = x;
            y += 8 * size;
        } else {
            lcd_draw_char(cursor_x, y, *str, color, bg, size);
            cursor_x += 6 * size;
        }
        str++;
    }
}

static void lcd_draw_image(int16_t x, int16_t y, const uint16_t *image, int16_t w, int16_t h, uint16_t transparent_color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            uint16_t pixel = image[j * w + i];
            // Skip transparent pixels (0x0000 = black/transparent)
            if (pixel != transparent_color) {
                if (x + i >= 0 && x + i < LCD_WIDTH && y + j >= 0 && y + j < LCD_HEIGHT) {
                    lcd_draw_pixel(x + i, y + j, pixel);
                }
            }
        }
    }
}

static void lcd_draw_image_simple(int16_t x, int16_t y, const uint16_t *image, int16_t w, int16_t h) {
    if (x < 0 || y < 0 || x + w > LCD_WIDTH || y + h > LCD_HEIGHT) return;
    
    lcd_set_addr_window(x, y, x + w - 1, y + h - 1);
    
    // Allocate buffer for entire image
    int total_pixels = w * h;
    uint8_t *buffer = malloc(total_pixels * 2);
    
    if (buffer) {
        // FIX: Correct RGB565 byte order for ILI9486
        for (int i = 0; i < total_pixels; i++) {
            uint16_t pixel = image[i];
            buffer[i * 2] = pixel >> 8;        // High byte first
            buffer[i * 2 + 1] = pixel & 0xFF;  // Low byte second
        }
        
        // Send entire buffer at once
        gpio_set_level(dc_pin, 1);  // Data mode
        lcd_data_buf(buffer, total_pixels * 2);
        
        free(buffer);
    } else {
        // Fallback: send pixel by pixel if malloc fails
        gpio_set_level(dc_pin, 1);
        for (int i = 0; i < total_pixels; i++) {
            uint16_t pixel = image[i];
            uint8_t hi = pixel >> 8;     // High byte first
            uint8_t lo = pixel & 0xFF;   // Low byte second
            lcd_data(hi);
            lcd_data(lo);
        }
    }
}

// === Dashboard Components ===

static void draw_header(const tracker_data_t *data) {
    if (!full_redraw_needed && data->timestamp == last_data.timestamp) return;
    
    lcd_fill_rect(0, 0, LCD_WIDTH, HEADER_HEIGHT, LCD_CHARCOAL);
    
    lcd_draw_image_simple(10, 3, sunflower_logo, SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT);
    
    // Draw "SUNFLOWER" text shifted right to make room for logo
    lcd_draw_string(40, 10, "SUNFLOWER", LCD_SUNGLOW, LCD_CHARCOAL, 2);
    
    // Draw time
    time_t now = (time_t)data->timestamp;
    struct tm *tm = localtime(&now);
    char time_str[80];
    snprintf(time_str, sizeof(time_str), "%02d/%02d/%04d %02d:%02d:%02d",
             tm->tm_mon + 1, tm->tm_mday, tm->tm_year + 1900,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    lcd_draw_string(350, 10, time_str, LCD_WHITE, LCD_CHARCOAL, 1);
}

static void draw_elevation_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->elevation == last_data.elevation && 
        data->delta_elevation == last_data.delta_elevation) return;
    
    int16_t x = 5, y = 35;
    lcd_fill_rect(x, y, 150, PANEL_HEIGHT, LCD_DARKBLUE);
    lcd_draw_rect(x, y, 150, PANEL_HEIGHT, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "ELEVATION", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", data->elevation);
    lcd_draw_string(x + 5, y + 25, angle_str, LCD_SUNGLOW, LCD_DARKBLUE, 3);
    lcd_draw_string(x + 120, y + 32, "deg", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.2f", data->delta_elevation);
    lcd_draw_string(x + 5, y + 52, delta_str, LCD_TEAL, LCD_DARKBLUE, 1);
}

static void draw_azimuth_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->azimuth == last_data.azimuth && 
        data->delta_azimuth == last_data.delta_azimuth) return;
    
    int16_t x = 165, y = 35;
    lcd_fill_rect(x, y, 150, PANEL_HEIGHT, LCD_DARKBLUE);
    lcd_draw_rect(x, y, 150, PANEL_HEIGHT, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "AZIMUTH", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", data->azimuth);
    lcd_draw_string(x + 5, y + 25, angle_str, LCD_SUNGLOW, LCD_DARKBLUE, 3);
    lcd_draw_string(x + 120, y + 32, "deg", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.2f", data->delta_azimuth);
    lcd_draw_string(x + 5, y + 52, delta_str, LCD_TEAL, LCD_DARKBLUE, 1);
}

static void draw_battery_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->battery_voltage == last_data.battery_voltage && 
        data->battery_soc_percent == last_data.battery_soc_percent &&
        data->battery_charging == last_data.battery_charging) return;
    
    int16_t x = 325, y = 35;
    lcd_fill_rect(x, y, 150, PANEL_HEIGHT, LCD_STEEL);
    lcd_draw_rect(x, y, 150, PANEL_HEIGHT, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "BATTERY", LCD_WHITE, LCD_STEEL, 1);
    
    char volt_str[16];
    snprintf(volt_str, sizeof(volt_str), "%.2fV", data->battery_voltage);
    lcd_draw_string(x + 5, y + 20, volt_str, LCD_SUNGLOW, LCD_STEEL, 2);
    
    char soc_str[16];
    snprintf(soc_str, sizeof(soc_str), "%.0f%%", data->battery_soc_percent);
    lcd_draw_string(x + 5, y + 40, soc_str, LCD_WHITE, LCD_STEEL, 1);
    
    char adc_str[16];
    snprintf(adc_str, sizeof(adc_str), "ADC:%u", data->battery_adc);
    lcd_draw_string(x + 5, y + 52, adc_str, LCD_WHITE, LCD_STEEL, 1);
    
    // FIXED: Calculate bar width directly from voltage for 12V LiFePO4
    // LiFePO4 range: ~10.0V (empty) to 14.6V (full)
    float min_voltage = 10.0f;
    float max_voltage = 14.6f;
    float voltage_range = max_voltage - min_voltage;
    
    // Clamp voltage to valid range
    float clamped_voltage = data->battery_voltage;
    if (clamped_voltage < min_voltage) clamped_voltage = min_voltage;
    if (clamped_voltage > max_voltage) clamped_voltage = max_voltage;
    
    // Calculate percentage for bar display
    float voltage_percent = (clamped_voltage - min_voltage) / voltage_range;
    if (voltage_percent > 1.0f) voltage_percent = 1.0f;
    if (voltage_percent < 0.0f) voltage_percent = 0.0f;
    
    int bar_width = (int)(voltage_percent * 60.0f); // 60 pixels max width
    
    uint16_t bar_color = LCD_SAGE; // Green for normal
    if (data->battery_voltage < 11.5f) bar_color = LCD_CRIMSON; // Red for critical
    else if (data->battery_voltage < 12.2f) bar_color = LCD_ORANGE; // Orange for low
    
    lcd_fill_rect(x + 80, y + 50, 60, 10, LCD_BLACK);
    lcd_fill_rect(x + 80, y + 50, bar_width, 10, bar_color);
    lcd_draw_rect(x + 80, y + 50, 60, 10, LCD_WHITE);
    
    // Charging indicator
    if (data->battery_charging) {
        lcd_draw_string(x + 60, y + 20, "CHG", LCD_MINT, LCD_STEEL, 1);
    }
}

static void draw_sun_position_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->sun_elevation == last_data.sun_elevation && 
        data->sun_azimuth == last_data.sun_azimuth) return;
    
    int16_t x = 5, y = 110;
    lcd_fill_rect(x, y, 150, PANEL_HEIGHT, LCD_DARKBLUE);
    lcd_draw_rect(x, y, 150, PANEL_HEIGHT, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "SUN POSITION", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char el_str[32];
    snprintf(el_str, sizeof(el_str), "El:%.1f", data->sun_elevation);
    lcd_draw_string(x + 5, y + 22, el_str, LCD_SUNGLOW, LCD_DARKBLUE, 1);
    lcd_draw_string(x + 55, y + 22, "deg", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char az_str[32];
    snprintf(az_str, sizeof(az_str), "Az:%.1f", data->sun_azimuth);
    lcd_draw_string(x + 5, y + 35, az_str, LCD_SUNGLOW, LCD_DARKBLUE, 1);
    lcd_draw_string(x + 55, y + 35, "deg", LCD_WHITE, LCD_DARKBLUE, 1);
    
    // Sunrise/Sunset
    if (data->sunrise_time > 0 && data->sunset_time > 0) {
        struct tm *sr = localtime((time_t*)&data->sunrise_time);
        struct tm *ss = localtime((time_t*)&data->sunset_time);
        
        char sun_times[32];
        snprintf(sun_times, sizeof(sun_times), "R:%02d:%02d S:%02d:%02d",
                 sr->tm_hour, sr->tm_min, ss->tm_hour, ss->tm_min);
        lcd_draw_string(x + 5, y + 52, sun_times, LCD_TEAL, LCD_DARKBLUE, 1);
    }
}

static void draw_status_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->status == last_data.status && 
        data->tracking_quality == last_data.tracking_quality &&
        data->sd_card_status == last_data.sd_card_status) return;
    
    int16_t x = 165, y = 110;
    
    const char *status_text;
    uint16_t status_color;
    uint16_t text_color;
    
    switch(data->status) {
        case 0:
            status_text = "STANDBY";
            status_color = LCD_ORANGE;
            text_color = LCD_BLACK;
            break;
        case 1:
            status_text = "TRACKING";
            status_color = LCD_SAGE;
            text_color = LCD_BLACK;
            break;
        case 2:
            status_text = "SLEEP";
            status_color = LCD_CHARCOAL;
            text_color = LCD_WHITE;
            break;
        case 3:
            status_text = "CALIBRATE";
            status_color = LCD_TEAL;
            text_color = LCD_BLACK;
            break;
        default:
            status_text = "ERROR";
            status_color = LCD_CRIMSON;
            text_color = LCD_WHITE;
            break;
    }
    
    lcd_fill_rect(x, y, 150, PANEL_HEIGHT, status_color);
    lcd_draw_rect(x, y, 150, PANEL_HEIGHT, LCD_WHITE);
    
    lcd_draw_string(x + 5, y + 5, "SYSTEM STATUS", text_color, status_color, 1);
    lcd_draw_string(x + 20, y + 28, status_text, text_color, status_color, 2);
    
    char error_str[16];
    snprintf(error_str, sizeof(error_str), "Err:%udeg", data->tracking_quality);
    lcd_draw_string(x + 5, y + 52, error_str, text_color, status_color, 1);
    
    const char *sd_status[] = {"SD:OK", "SD:SLOW", "SD:FULL", "SD:FAIL"};
    uint8_t sd_idx = (data->sd_card_status > 3) ? 3 : data->sd_card_status;
    lcd_draw_string(x + 75, y + 52, sd_status[sd_idx], text_color, status_color, 1);
}

static void draw_gps_panel(const tracker_data_t *data) {
    if (!full_redraw_needed && data->latitude == last_data.latitude && 
        data->longitude == last_data.longitude && data->gps_valid == last_data.gps_valid &&
        data->gps_satellites == last_data.gps_satellites) return;
    
    int16_t x = 325, y = 110;
    int16_t h = 35;
    
    lcd_fill_rect(x, y, 150, h, LCD_DARKBLUE);
    lcd_draw_rect(x, y, 150, h, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "GPS", LCD_WHITE, LCD_DARKBLUE, 1);
    
    if (data->gps_valid) {
        char gps_str[32];
        snprintf(gps_str, sizeof(gps_str), "%.4f,%.4f", data->latitude, data->longitude);
        lcd_draw_string(x + 5, y + 15, gps_str, LCD_TEAL, LCD_DARKBLUE, 1);
        
        snprintf(gps_str, sizeof(gps_str), "Sats:%u Age:%lus", 
                 data->gps_satellites, data->last_gps_fix_age_sec);
        lcd_draw_string(x + 5, y + 25, gps_str, LCD_TEAL, LCD_DARKBLUE, 1);
    } else {
        lcd_draw_string(x + 5, y + 15, "NO FIX", LCD_CRIMSON, LCD_DARKBLUE, 1);
        char age_str[24];
        snprintf(age_str, sizeof(age_str), "Age:%lus", data->last_gps_fix_age_sec);
        lcd_draw_string(x + 5, y + 25, age_str, LCD_ORANGE, LCD_DARKBLUE, 1);
    }
}

static void draw_stats_panel(const tracker_data_t *data) {
    // Get client's local RSSI for comparison
    int8_t client_rssi = wifi_client_get_signal_strength();
    
    if (!full_redraw_needed && data->moves_today == last_data.moves_today && 
        data->total_moves == last_data.total_moves && data->uptime_hours == last_data.uptime_hours &&
        data->wifi_rssi == last_data.wifi_rssi && client_rssi == wifi_client_get_signal_strength()) return;
    
    int16_t x = 325, y = 147;
    int16_t h = 33;
    
    lcd_fill_rect(x, y, 150, h, LCD_DARKBLUE);
    lcd_draw_rect(x, y, 150, h, LCD_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "STATISTICS", LCD_WHITE, LCD_DARKBLUE, 1);
    
    char stats_str[32];
    snprintf(stats_str, sizeof(stats_str), "Today:%lu Total:%lu", 
             data->moves_today, data->total_moves);
    lcd_draw_string(x + 5, y + 15, stats_str, LCD_TEAL, LCD_DARKBLUE, 1);
    
    // Show both WiFi RSSI values: Tracker's view / Client's view
    snprintf(stats_str, sizeof(stats_str), "Up:%uh WiFi:%d/%ddBm", 
             data->uptime_hours, data->wifi_rssi, client_rssi);
    lcd_draw_string(x + 5, y + 24, stats_str, LCD_TEAL, LCD_DARKBLUE, 1);
}

static void draw_voltage_graph(const tracker_data_t *data) {
    int16_t x = 5, y = GRAPH_Y_START;
    int16_t w = 470, h = GRAPH_HEIGHT;
    
    // FIXED: Updated voltage range for 12V LiFePO4 battery (10V-14.8V)
    float min_voltage = 10.0f;   // Minimum safe voltage
    float max_voltage = 14.8f;   // Maximum charging voltage (updated from 15.0V)
    
    if (full_redraw_needed) {
        // Clear entire graph area including old data
        lcd_fill_rect(x, y, w, h, LCD_BLACK);
        lcd_draw_rect(x, y, w, h, LCD_SLATE);
        
        lcd_draw_string(x + 5, y + 5, "Battery Voltage History (Last 100 Readings)", 
                        LCD_WHITE, LCD_BLACK, 1);
        
        // Draw grid lines
        for (int i = 0; i <= 4; i++) {
            int grid_y = y + 20 + (h - 30) * i / 4;
            lcd_draw_line(x + 35, grid_y, x + w - 5, grid_y, LCD_SILVER);
        }
        
        // Y-axis labels - UPDATED for new range
        char label[8];
        snprintf(label, sizeof(label), "%.1fV", max_voltage);  // Top (14.8V)
        lcd_draw_string(x + 5, y + 20, label, LCD_WHITE, LCD_BLACK, 1);
        
        snprintf(label, sizeof(label), "%.1fV", 12.4f);  // Middle (updated from 12.5V)
        lcd_draw_string(x + 3, y + 20 + (h - 30) / 2, label, LCD_WHITE, LCD_BLACK, 1);
        
        snprintf(label, sizeof(label), "%.1fV", min_voltage);  // Bottom (10.0V)
        lcd_draw_string(x + 5, y + h - 12, label, LCD_WHITE, LCD_BLACK, 1);
    } else {
        // Clear only the graph drawing area (not labels/grid)
        lcd_fill_rect(x + 35, y + 20, w - 40, h - 30, LCD_BLACK);
        
        // Redraw grid lines
        for (int i = 0; i <= 4; i++) {
            int grid_y = y + 20 + (h - 30) * i / 4;
            lcd_draw_line(x + 35, grid_y, x + w - 5, grid_y, LCD_SILVER);
        }
    }
    
    // Draw graph
    int graph_w = w - 45;
    int graph_h = h - 35;
    int graph_x = x + 35;
    int graph_y = y + 25;
    
    for (int i = 1; i < 100; i++) {
        int prev_idx = (history_index + i - 1) % 100;
        int curr_idx = (history_index + i) % 100;
        
        if (voltage_history[prev_idx] == 0 || voltage_history[curr_idx] == 0) continue;
        
        // FIXED: Use the stored voltage values (already calculated by tracker)
        float v1 = (float)voltage_history[prev_idx] / 1000.0f;  // Convert back from mV to V
        float v2 = (float)voltage_history[curr_idx] / 1000.0f;  // Convert back from mV to V
        
        // DEBUG: Print current voltage for troubleshooting
        if (i == 1) {
            ESP_LOGI(TAG, "Graph voltage: Stored=%.2fV, Expected=%.2fV", 
                    v2, data->battery_voltage);
        }
        
        // Clamp to display range
        if (v1 < min_voltage) v1 = min_voltage;
        if (v1 > max_voltage) v1 = max_voltage;
        if (v2 < min_voltage) v2 = min_voltage;
        if (v2 > max_voltage) v2 = max_voltage;
        
        // Calculate Y position (inverted: high voltage = low Y)
        // Map voltage range [min_voltage-max_voltage] to graph height [0-graph_h]
        float norm1 = (v1 - min_voltage) / (max_voltage - min_voltage);
        float norm2 = (v2 - min_voltage) / (max_voltage - min_voltage);
        
        int x1 = graph_x + (graph_w * (i - 1) / 99);
        int y1 = graph_y + graph_h - (int)(norm1 * (float)graph_h);
        int x2 = graph_x + (graph_w * i / 99);
        int y2 = graph_y + graph_h - (int)(norm2 * (float)graph_h);
        
        lcd_draw_line(x1, y1, x2, y2, LCD_SAGE);
    }
}

// === Public API ===

esp_err_t lcd_client_init(const lcd_config_t *config) {
    ESP_LOGI(TAG, "Initializing LCD display...");
    
    dc_pin = config->dc_pin;
    rst_pin = config->rst_pin;
    backlight_pin = config->backlight_pin;
    
    gpio_set_direction(dc_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    spi_bus_config_t buscfg = {
        .miso_io_num = config->miso_pin,
        .mosi_io_num = config->mosi_pin,
        .sclk_io_num = config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * 2 * 40
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = config->cs_pin,
        .queue_size = 7,
        .pre_cb = lcd_spi_pre_transfer_callback
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_device));
    
    lcd_cmd(ILI9486_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0xB0); lcd_data(0x00);
    lcd_cmd(ILI9486_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(ILI9486_PIXFMT); lcd_data(0x55);
    lcd_cmd(ILI9486_PWCTR1); lcd_data(0x19); lcd_data(0x1A);
    lcd_cmd(ILI9486_PWCTR2); lcd_data(0x45); lcd_data(0x00);
    lcd_cmd(ILI9486_PWCTR3); lcd_data(0x33);
    lcd_cmd(ILI9486_VMCTR1); lcd_data(0x00); lcd_data(0x12); lcd_data(0x80);
    lcd_cmd(ILI9486_MADCTL); lcd_data(0x28);
    lcd_cmd(ILI9486_DFUNCTR); lcd_data(0x00); lcd_data(0x02); lcd_data(0x3B);
    lcd_cmd(ILI9486_FRMCTR1); lcd_data(0xB0); lcd_data(0x11);
    lcd_cmd(ILI9486_INVCTR); lcd_data(0x02);
    //GAMMA CORRECTION COMMANDS:
    lcd_cmd(ILI9486_GMCTRP1);
    lcd_data(0x0F); lcd_data(0x24); lcd_data(0x1C); lcd_data(0x0A);
    lcd_data(0x0F); lcd_data(0x08); lcd_data(0x43); lcd_data(0x88);
    lcd_data(0x32); lcd_data(0x0F); lcd_data(0x10); lcd_data(0x06);
    lcd_data(0x0F); lcd_data(0x07); lcd_data(0x00);

    lcd_cmd(ILI9486_GMCTRN1);
    lcd_data(0x0F); lcd_data(0x38); lcd_data(0x30); lcd_data(0x09);
    lcd_data(0x0F); lcd_data(0x0F); lcd_data(0x4E); lcd_data(0x77);
    lcd_data(0x3C); lcd_data(0x07); lcd_data(0x10); lcd_data(0x05);
    lcd_data(0x23); lcd_data(0x1B); lcd_data(0x00);
    lcd_cmd(ILI9486_DISPON);
    vTaskDelay(pdMS_TO_TICKS(25));
    lcd_cmd(ILI9486_NORON);
    
    if (backlight_pin >= 0) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);
        
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = backlight_pin,
            .duty = 255,
            .hpoint = 0
        };
        ledc_channel_config(&ledc_channel);
    }
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK);
    
    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

void lcd_client_set_brightness(uint8_t brightness) {
    if (backlight_pin >= 0) {
        uint32_t duty = (brightness * 255) / 100;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void lcd_client_show_init_screen(const char *message) {
    full_redraw_needed = true;
    
    // CLEAR ENTIRE SCREEN FIRST
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK);
    
    // FIX: Use transparent drawing for init screen too
    int logo_x = (LCD_WIDTH - SUNFLOWER_LOGO_WIDTH) / 2;
    int logo_y = 100;
    lcd_draw_image_simple(logo_x, logo_y, sunflower_logo, 
                        SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT);
    // Draw title below logo
    const char *title = "SUNFLOWER";
    int title_x = (LCD_WIDTH - (strlen(title) * 12)) / 2;
    lcd_draw_string(title_x, 160, title, LCD_SUNGLOW, LCD_BLACK, 2);
    
    // Draw message centered
    int msg_x = (LCD_WIDTH - (strlen(message) * 6)) / 2;
    lcd_draw_string(msg_x, 200, message, LCD_WHITE, LCD_BLACK, 1);
    
    // Draw animated dots
    static int dot_count = 0;
    char dots[5] = "";
    for (int i = 0; i < (dot_count % 4); i++) {
        strcat(dots, ".");
    }
    lcd_draw_string(msg_x + strlen(message) * 6, 200, dots, LCD_MINT, LCD_BLACK, 1);
    lcd_draw_string(msg_x + strlen(message) * 6 + 24, 200, "    ", LCD_BLACK, LCD_BLACK, 1);
    dot_count++;
}

void lcd_client_display_dashboard(const tracker_data_t *data) {
    // CLEAR SCREEN ON FIRST DASHBOARD DRAW
    if (full_redraw_needed) {
        lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK);
    }
    
    // FIXED: Store the actual voltage from tracker data, not recalculated
    // The tracker already calculates the voltage correctly using battery.c
    // Store voltage in millivolts to avoid floating point precision issues
    uint16_t voltage_mv = (uint16_t)(data->battery_voltage * 1000.0f);
    voltage_history[history_index] = voltage_mv;
    history_index = (history_index + 1) % 100;
    
    draw_header(data);
    draw_elevation_panel(data);
    draw_azimuth_panel(data);
    draw_battery_panel(data);
    draw_sun_position_panel(data);
    draw_status_panel(data);
    draw_gps_panel(data);
    draw_stats_panel(data);
    draw_voltage_graph(data);
    
    memcpy(&last_data, data, sizeof(tracker_data_t));
    full_redraw_needed = false;
}

void lcd_client_show_error(const char *message) {
    full_redraw_needed = true;
    
    // CLEAR ENTIRE SCREEN
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_CRIMSON);
    
    const char *title = "CONNECTION LOST";
    int title_x = (LCD_WIDTH - (strlen(title) * 12)) / 2;
    lcd_draw_string(title_x, 140, title, LCD_WHITE, LCD_CRIMSON, 2);
    
    int msg_x = (LCD_WIDTH - (strlen(message) * 6)) / 2;
    lcd_draw_string(msg_x, 170, message, LCD_WHITE, LCD_CRIMSON, 1);
}