#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "sunflower_logo.h"  // Add after other includes

// Pin definitions
#define TFT_MOSI    23
#define TFT_SCLK    18
#define TFT_CS      5
#define TFT_DC      21
#define TFT_RST     4

// Display dimensions (landscape)
#define TFT_WIDTH   480
#define TFT_HEIGHT  320

// ILI9486 Commands
#define ILI9486_NOP         0x00
#define ILI9486_SWRESET     0x01
#define ILI9486_SLPOUT      0x11
#define ILI9486_NORON       0x13
#define ILI9486_INVOFF      0x20
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

// RGB565 Colors - Brighter, more vibrant palette
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF

// Header and panels (brighter greys with blue tint)
#define TFT_CHARCOAL    0x3186      // Brighter charcoal blue-grey (header)
#define TFT_STEEL       0x6B4D      // Bright steel blue-grey (panels)
#define TFT_SILVER      0xC618      // Bright silver
#define TFT_SLATE       0x9CF3      // Bright slate blue
#define TFT_DARKGREY    0x5ACB      // Medium grey (brighter)

// Sunflower accent colors (bright and vibrant)
#define TFT_GOLDEN      0xFEA0      // Bright golden yellow
#define TFT_AMBER       0xFD20      // Vibrant orange-amber
#define TFT_SUNGLOW     0xFFE0      // Pure bright yellow

// Circuit/Tech colors (bright)
#define TFT_TEAL        0x07FF      // Bright cyan/teal
#define TFT_MINT        0x87F0      // Bright mint green

// Status colors (vibrant)
#define TFT_SAGE        0x07E0      // Bright lime green (active/good)
#define TFT_CORAL       0xFBEA      // Bright coral/salmon
#define TFT_CRIMSON     0xF800      // Bright red (error/standby)

// Simple 5x7 bitmap font (ASCII 32-126)
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

// Solar Panel Data Structure
typedef struct {
    float elevation;
    float azimuth;
    float delta_elevation;
    float delta_azimuth;
    uint16_t battery_adc;
    float battery_voltage;
    uint16_t battery_history[100];
    int history_index;
    uint32_t timestamp;
} solar_data_t;

static const char *TAG = "SOLAR_TRACKER";
static spi_device_handle_t spi;
static solar_data_t solar_data;

// Add these variables to track battery cycle
static int battery_direction = 1;  // 1 = increasing, -1 = decreasing
static int battery_cycle_speed = 8; // How fast the battery changes

// ===== Low-level SPI functions =====

static void tft_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(TFT_DC, dc);
}

void tft_cmd(uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void tft_data(uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void tft_data_buf(uint8_t *data, int len)
{
    if (len == 0) return;
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

// ===== Hardware initialization =====

void tft_init_pins(void)
{
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "GPIO pins configured");
}

void tft_reset(void)
{
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
}

void tft_init_spi(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_WIDTH * 2 * 40,
        .flags = 0
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 7,
        .pre_cb = tft_spi_pre_transfer_callback,
        .flags = 0
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "SPI initialized at 10 MHz");
}

void tft_init_display(void)
{
    tft_cmd(ILI9486_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    tft_cmd(0xB0);
    tft_data(0x00);
    
    tft_cmd(ILI9486_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    tft_cmd(ILI9486_PIXFMT);
    tft_data(0x55);
    
    tft_cmd(ILI9486_PWCTR1);
    tft_data(0x19);
    tft_data(0x1A);
    
    tft_cmd(ILI9486_PWCTR2);
    tft_data(0x45);
    tft_data(0x00);
    
    tft_cmd(ILI9486_PWCTR3);
    tft_data(0x33);
    
    tft_cmd(ILI9486_VMCTR1);
    tft_data(0x00);
    tft_data(0x12);
    tft_data(0x80);
    
    tft_cmd(ILI9486_MADCTL);
    tft_data(0x28);
    
    tft_cmd(ILI9486_DFUNCTR);
    tft_data(0x00);
    tft_data(0x02);
    tft_data(0x3B);
    
    tft_cmd(ILI9486_FRMCTR1);
    tft_data(0xB0);
    tft_data(0x11);
    
    tft_cmd(ILI9486_INVCTR);
    tft_data(0x02);
    
    tft_cmd(ILI9486_GMCTRP1);
    tft_data(0x0F); tft_data(0x24); tft_data(0x1C); tft_data(0x0A);
    tft_data(0x0F); tft_data(0x08); tft_data(0x43); tft_data(0x88);
    tft_data(0x32); tft_data(0x0F); tft_data(0x10); tft_data(0x06);
    tft_data(0x0F); tft_data(0x07); tft_data(0x00);
    
    tft_cmd(ILI9486_GMCTRN1);
    tft_data(0x0F); tft_data(0x38); tft_data(0x30); tft_data(0x09);
    tft_data(0x0F); tft_data(0x0F); tft_data(0x4E); tft_data(0x77);
    tft_data(0x3C); tft_data(0x07); tft_data(0x10); tft_data(0x05);
    tft_data(0x23); tft_data(0x1B); tft_data(0x00);
    
    tft_cmd(ILI9486_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    tft_cmd(ILI9486_DISPON);
    vTaskDelay(pdMS_TO_TICKS(25));
    
    tft_cmd(ILI9486_NORON);
    ESP_LOGI(TAG, "Display initialized");
}

// ===== Drawing functions =====

void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    tft_cmd(ILI9486_CASET);
    tft_data(x0 >> 8);
    tft_data(x0 & 0xFF);
    tft_data(x1 >> 8);
    tft_data(x1 & 0xFF);

    tft_cmd(ILI9486_PASET);
    tft_data(y0 >> 8);
    tft_data(y0 & 0xFF);
    tft_data(y1 >> 8);
    tft_data(y1 & 0xFF);

    tft_cmd(ILI9486_RAMWR);
}

void tft_fill_screen(uint16_t color)
{
    tft_set_addr_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

    const int lines_per_chunk = 10;
    uint8_t *buffer = malloc(TFT_WIDTH * 2 * lines_per_chunk);
    
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer!");
        return;
    }

    for (int i = 0; i < TFT_WIDTH * lines_per_chunk; i++) {
        buffer[i * 2] = color >> 8;
        buffer[i * 2 + 1] = color & 0xFF;
    }

    gpio_set_level(TFT_DC, 1);
    
    int remaining_lines = TFT_HEIGHT;
    while (remaining_lines > 0) {
        int lines_to_send = (remaining_lines >= lines_per_chunk) ? lines_per_chunk : remaining_lines;
        tft_data_buf(buffer, TFT_WIDTH * 2 * lines_to_send);
        remaining_lines -= lines_to_send;
    }
    
    free(buffer);
}

void tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || w <= 0 || h <= 0) return;
    if (x + w > TFT_WIDTH) w = TFT_WIDTH - x;
    if (y + h > TFT_HEIGHT) h = TFT_HEIGHT - y;

    tft_set_addr_window(x, y, x + w - 1, y + h - 1);

    uint8_t *line = malloc(w * 2);
    if (line == NULL) return;

    for (int i = 0; i < w; i++) {
        line[i * 2] = color >> 8;
        line[i * 2 + 1] = color & 0xFF;
    }

    gpio_set_level(TFT_DC, 1);
    for (int i = 0; i < h; i++) {
        tft_data_buf(line, w * 2);
    }
    
    free(line);
}

void tft_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT) return;
    tft_set_addr_window(x, y, x, y);
    tft_data(color >> 8);
    tft_data(color & 0xFF);
}

void tft_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        tft_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void tft_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    tft_draw_line(x, y, x + w - 1, y, color);
    tft_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
    tft_draw_line(x + w - 1, y + h - 1, x, y + h - 1, color);
    tft_draw_line(x, y + h - 1, x, y, color);
}

// ===== REAL Text Rendering with Bitmap Font =====

void tft_draw_char(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size)
{
    if (c < ' ' || c > 'z') c = '?';
    int index = c - ' ';
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[index][i];
        for (int j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) {
                    tft_draw_pixel(x + i, y + j, color);
                } else {
                    tft_fill_rect(x + i * size, y + j * size, size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    tft_draw_pixel(x + i, y + j, bg);
                } else {
                    tft_fill_rect(x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

void tft_draw_string(int16_t x, int16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size)
{
    int cursor_x = x;
    while (*str) {
        if (*str == '\n') {
            cursor_x = x;
            y += 8 * size;
        } else {
            tft_draw_char(cursor_x, y, *str, color, bg, size);
            cursor_x += 6 * size; // 5 pixels + 1 space
        }
        str++;
    }
}

void tft_draw_image(int16_t x, int16_t y, const uint16_t *image, int16_t w, int16_t h, uint16_t transparent_color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            uint16_t pixel = image[j * w + i];
            // Skip transparent pixels (0x0000 = black/transparent)
            if (pixel != transparent_color) {
                if (x + i >= 0 && x + i < TFT_WIDTH && y + j >= 0 && y + j < TFT_HEIGHT) {
                    tft_draw_pixel(x + i, y + j, pixel);
                }
            }
        }
    }
}

// ===== Solar Panel Simulation =====

void init_solar_data(void)
{
    solar_data.elevation = 45.0f;
    solar_data.azimuth = 180.0f;
    solar_data.delta_elevation = 0.0f;
    solar_data.delta_azimuth = 0.0f;
    solar_data.battery_adc = 2500;
    solar_data.battery_voltage = 12.2f;
    solar_data.history_index = 0;
    // Start at 12:50 AM (50 minutes past midnight)
    solar_data.timestamp = 50 * 60; // 3000 seconds
    
    for (int i = 0; i < 100; i++) {
        solar_data.battery_history[i] = 2500;
    }
}

void update_solar_data(void)
{
    float time_factor = (solar_data.timestamp % 86400) / 86400.0f;
    solar_data.elevation = 5.0f + 85.0f * sin(time_factor * M_PI);
    
    solar_data.azimuth = 90.0f + 180.0f * time_factor;
    if (solar_data.azimuth > 360.0f) solar_data.azimuth -= 360.0f;
    
    solar_data.delta_elevation = ((esp_random() % 20) - 10) / 10.0f;
    solar_data.delta_azimuth = ((esp_random() % 30) - 15) / 10.0f;
    
    // Cyclical battery pattern: up to max, down to min, repeat
    if (battery_direction == 1) {
        // Going up
        solar_data.battery_adc += battery_cycle_speed + (esp_random() % 3);
        if (solar_data.battery_adc >= 3400) {
            solar_data.battery_adc = 3400;
            battery_direction = -1; // Start going down
        }
    } else {
        // Going down
        solar_data.battery_adc -= battery_cycle_speed + (esp_random() % 3);
        if (solar_data.battery_adc <= 2000) {
            solar_data.battery_adc = 2000;
            battery_direction = 1; // Start going up
        }
    }
    
    // Keep within bounds
    if (solar_data.battery_adc > 3400) solar_data.battery_adc = 3400;
    if (solar_data.battery_adc < 2000) solar_data.battery_adc = 2000;
    
    solar_data.battery_voltage = (solar_data.battery_adc / 4095.0f) * 15.0f;
    
    solar_data.battery_history[solar_data.history_index] = solar_data.battery_adc;
    solar_data.history_index = (solar_data.history_index + 1) % 100;
    
    solar_data.timestamp++;
}

// ===== Dashboard Drawing =====

void draw_header(void)
{
    tft_fill_rect(0, 0, TFT_WIDTH, 30, TFT_CHARCOAL);
    
    // Draw logo (24x24 at position 10, 3 to center vertically in 30px header)
    tft_draw_image(10, 3, sunflower_logo, SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT, TFT_CHARCOAL);
    
    // Draw "SUNFLOWER" text shifted right to make room for logo
    tft_draw_string(40, 10, "SUNFLOWER", TFT_SILVER, TFT_CHARCOAL, 2);
    
    // Calculate hours, minutes, seconds
    uint32_t hours = (solar_data.timestamp / 3600) % 24;
    uint32_t minutes = (solar_data.timestamp / 60) % 60;
    uint32_t seconds = solar_data.timestamp % 60;
    
    // Determine AM/PM
    const char *period = (hours < 12) ? "AM" : "PM";
    uint32_t display_hours = hours % 12;
    if (display_hours == 0) display_hours = 12;
    
    char time_str[24];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu%s", display_hours, minutes, seconds, period);
    tft_draw_string(330, 10, time_str, TFT_GOLDEN, TFT_CHARCOAL, 2);
}

void draw_angle_panel(int16_t x, int16_t y, const char *label, float angle, float delta, uint16_t color)
{
    tft_fill_rect(x, y, 150, 70, TFT_STEEL);
    tft_draw_rect(x, y, 150, 70, TFT_SLATE);
    
    tft_draw_string(x + 5, y + 5, label, TFT_SILVER, TFT_STEEL, 1);
    
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", angle);
    tft_draw_string(x + 5, y + 25, angle_str, color, TFT_STEEL, 3);
    tft_draw_string(x + 5, y + 52, "deg", TFT_SILVER, TFT_STEEL, 1);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.3f", delta);
    tft_draw_string(x + 50, y + 52, delta_str, TFT_TEAL, TFT_STEEL, 1);
}

void draw_battery_panel(int16_t x, int16_t y)
{
    tft_fill_rect(x, y, 150, 70, TFT_DARKGREY);
    tft_draw_rect(x, y, 150, 70, TFT_SAGE);
    
    tft_draw_string(x + 5, y + 5, "BATTERY", TFT_SILVER, TFT_DARKGREY, 1);
    
    char volt_str[16];
    snprintf(volt_str, sizeof(volt_str), "%.2fV", solar_data.battery_voltage);
    tft_draw_string(x + 5, y + 25, volt_str, TFT_GOLDEN, TFT_DARKGREY, 3);
    
    char adc_str[16];
    snprintf(adc_str, sizeof(adc_str), "ADC:%d", solar_data.battery_adc);
    tft_draw_string(x + 5, y + 52, adc_str, TFT_SILVER, TFT_DARKGREY, 1);
    
    // Battery level bar
    int bar_width = (int)(((solar_data.battery_adc - 2000) / 1400.0f) * 60);
    if (bar_width > 60) bar_width = 60;
    if (bar_width < 0) bar_width = 0;
    
    tft_fill_rect(x + 80, y + 50, 60, 8, TFT_BLACK);
    tft_fill_rect(x + 80, y + 50, bar_width, 8, TFT_SAGE);
    tft_draw_rect(x + 80, y + 50, 60, 8, TFT_SILVER);
}

void draw_status_panel(int16_t x, int16_t y)
{
    uint16_t status_color = (solar_data.elevation > 10.0f) ? TFT_SAGE : TFT_CRIMSON;
    tft_fill_rect(x, y, 150, 70, status_color);
    tft_draw_rect(x, y, 150, 70, TFT_SILVER);
    
    tft_draw_string(x + 5, y + 5, "STATUS", TFT_BLACK, status_color, 1);
    
    const char *status_text;
    if (solar_data.elevation > 10.0f) {
        status_text = "TRACKING";
        tft_draw_string(x + 10, y + 30, status_text, TFT_BLACK, status_color, 2);
        tft_draw_string(x + 25, y + 50, "SUN", TFT_BLACK, status_color, 2);
    } else {
        status_text = "STANDBY";
        tft_draw_string(x + 10, y + 30, status_text, TFT_SILVER, status_color, 2);
        tft_draw_string(x + 20, y + 50, "MODE", TFT_SILVER, status_color, 2);
    }
}

void draw_battery_graph(int16_t x, int16_t y, int16_t w, int16_t h)
{
    tft_fill_rect(x, y, w, h, TFT_BLACK);
    tft_draw_rect(x, y, w, h, TFT_SLATE);
    
    tft_draw_string(x + 5, y + 5, "Battery Voltage History (Last 100 Readings)", TFT_SILVER, TFT_BLACK, 1);
    
    // Draw grid lines
    for (int i = 0; i <= 4; i++) {
        int grid_y = y + 20 + (h - 30) * i / 4;
        tft_draw_line(x + 35, grid_y, x + w - 5, grid_y, TFT_DARKGREY);
    }
    
    // Find min/max for scaling
    uint16_t min_val = 4095, max_val = 0;
    for (int i = 0; i < 100; i++) {
        if (solar_data.battery_history[i] < min_val) min_val = solar_data.battery_history[i];
        if (solar_data.battery_history[i] > max_val) max_val = solar_data.battery_history[i];
    }
    
    if (max_val == min_val) max_val = min_val + 100;
    
    // Draw graph line
    int graph_w = w - 45;
    int graph_h = h - 35;
    
    for (int i = 1; i < 100; i++) {
        int prev_index = (solar_data.history_index + i - 1) % 100;
        int curr_index = (solar_data.history_index + i) % 100;
        
        int x1 = x + 35 + (graph_w * (i - 1) / 99);
        int y1 = y + 25 + graph_h - ((solar_data.battery_history[prev_index] - min_val) * graph_h / (max_val - min_val));
        
        int x2 = x + 35 + (graph_w * i / 99);
        int y2 = y + 25 + graph_h - ((solar_data.battery_history[curr_index] - min_val) * graph_h / (max_val - min_val));
        
        tft_draw_line(x1, y1, x2, y2, TFT_SAGE);
        
        // Mark current data point
        if (i == 99) {
            tft_fill_rect(x2 - 1, y2 - 1, 3, 3, TFT_AMBER);
        }
    }
    
    // Y-axis labels
    char min_str[8], max_str[8], mid_str[8];
    snprintf(min_str, sizeof(min_str), "%.1fV", (min_val / 4095.0f) * 15.0f);
    snprintf(max_str, sizeof(max_str), "%.1fV", (max_val / 4095.0f) * 15.0f);
    uint16_t mid_val = (min_val + max_val) / 2;
    snprintf(mid_str, sizeof(mid_str), "%.1fV", (mid_val / 4095.0f) * 15.0f);
    
    tft_draw_string(x + 5, y + h - 12, min_str, TFT_SILVER, TFT_BLACK, 1);
    tft_draw_string(x + 5, y + (h / 2) - 3, mid_str, TFT_SILVER, TFT_BLACK, 1);
    tft_draw_string(x + 5, y + 20, max_str, TFT_SILVER, TFT_BLACK, 1);
}

// ===== Update Functions =====

void update_header_time(void)
{
    // Only update the time portion - don't redraw logo and title
    uint32_t hours = (solar_data.timestamp / 3600) % 24;
    uint32_t minutes = (solar_data.timestamp / 60) % 60;
    uint32_t seconds = solar_data.timestamp % 60;
    
    const char *period = (hours < 12) ? "AM" : "PM";
    uint32_t display_hours = hours % 12;
    if (display_hours == 0) display_hours = 12;
    
    char time_str[24];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu%s", display_hours, minutes, seconds, period);
    
    // Clear only the time area and redraw
    tft_fill_rect(330, 10, 140, 16, TFT_CHARCOAL);
    tft_draw_string(330, 10, time_str, TFT_GOLDEN, TFT_CHARCOAL, 2);
}

void update_angle_panel_value(int16_t x, int16_t y, float angle, float delta, uint16_t color)
{
    // Clear only the value areas
    tft_fill_rect(x + 5, y + 25, 140, 24, TFT_STEEL);
    tft_fill_rect(x + 50, y + 52, 95, 8, TFT_STEEL);
    
    // Redraw values
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", angle);
    tft_draw_string(x + 5, y + 25, angle_str, color, TFT_STEEL, 3);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.3f", delta);
    tft_draw_string(x + 50, y + 52, delta_str, TFT_TEAL, TFT_STEEL, 1);
}

void update_battery_panel_value(int16_t x, int16_t y)
{
    // Clear only the value areas
    tft_fill_rect(x + 5, y + 25, 140, 24, TFT_DARKGREY);
    tft_fill_rect(x + 5, y + 52, 140, 8, TFT_DARKGREY);
    
    // Redraw voltage
    char volt_str[16];
    snprintf(volt_str, sizeof(volt_str), "%.2fV", solar_data.battery_voltage);
    tft_draw_string(x + 5, y + 25, volt_str, TFT_GOLDEN, TFT_DARKGREY, 3);
    
    // Redraw ADC
    char adc_str[16];
    snprintf(adc_str, sizeof(adc_str), "ADC:%d", solar_data.battery_adc);
    tft_draw_string(x + 5, y + 52, adc_str, TFT_SILVER, TFT_DARKGREY, 1);
    
    // Update battery bar
    int bar_width = (int)(((solar_data.battery_adc - 2000) / 1400.0f) * 60);
    if (bar_width > 60) bar_width = 60;
    if (bar_width < 0) bar_width = 0;
    
    tft_fill_rect(x + 80, y + 50, 60, 8, TFT_BLACK);
    tft_fill_rect(x + 80, y + 50, bar_width, 8, TFT_SAGE);
    tft_draw_rect(x + 80, y + 50, 60, 8, TFT_SILVER);
}

void update_status_panel(int16_t x, int16_t y)
{
    uint16_t status_color = (solar_data.elevation > 10.0f) ? TFT_SAGE : TFT_CRIMSON;
    
    // Clear and redraw entire status panel (it changes color)
    tft_fill_rect(x, y, 150, 70, status_color);
    tft_draw_rect(x, y, 150, 70, TFT_SILVER);
    
    tft_draw_string(x + 5, y + 5, "STATUS", TFT_BLACK, status_color, 1);
    
    if (solar_data.elevation > 10.0f) {
        tft_draw_string(x + 10, y + 30, "TRACKING", TFT_BLACK, status_color, 2);
        tft_draw_string(x + 25, y + 50, "SUN", TFT_BLACK, status_color, 2);
    } else {
        tft_draw_string(x + 10, y + 30, "STANDBY", TFT_SILVER, status_color, 2);
        tft_draw_string(x + 20, y + 50, "MODE", TFT_SILVER, status_color, 2);
    }
}

void update_battery_graph_incremental(int16_t x, int16_t y, int16_t w, int16_t h)
{
    // Clear only graph area (not title or labels)
    tft_fill_rect(x + 35, y + 20, w - 45, h - 30, TFT_BLACK);
    
    // Redraw grid lines
    for (int i = 0; i <= 4; i++) {
        int grid_y = y + 20 + (h - 30) * i / 4;
        tft_draw_line(x + 35, grid_y, x + w - 5, grid_y, TFT_DARKGREY);
    }
    
    // Find min/max for scaling
    uint16_t min_val = 4095, max_val = 0;
    for (int i = 0; i < 100; i++) {
        if (solar_data.battery_history[i] < min_val) min_val = solar_data.battery_history[i];
        if (solar_data.battery_history[i] > max_val) max_val = solar_data.battery_history[i];
    }
    
    if (max_val == min_val) max_val = min_val + 100;
    
    // Draw graph line
    int graph_w = w - 45;
    int graph_h = h - 35;
    
    for (int i = 1; i < 100; i++) {
        int prev_index = (solar_data.history_index + i - 1) % 100;
        int curr_index = (solar_data.history_index + i) % 100;
        
        int x1 = x + 35 + (graph_w * (i - 1) / 99);
        int y1 = y + 25 + graph_h - ((solar_data.battery_history[prev_index] - min_val) * graph_h / (max_val - min_val));
        
        int x2 = x + 35 + (graph_w * i / 99);
        int y2 = y + 25 + graph_h - ((solar_data.battery_history[curr_index] - min_val) * graph_h / (max_val - min_val));
        
        tft_draw_line(x1, y1, x2, y2, TFT_SAGE);
        
        // Mark current data point
        if (i == 99) {
            tft_fill_rect(x2 - 1, y2 - 1, 3, 3, TFT_AMBER);
        }
    }
    
    // Update Y-axis labels
    tft_fill_rect(x + 5, y + 20, 25, 8, TFT_BLACK);
    tft_fill_rect(x + 5, y + (h / 2) - 3, 25, 8, TFT_BLACK);
    tft_fill_rect(x + 5, y + h - 12, 25, 8, TFT_BLACK);
    
    char min_str[8], max_str[8], mid_str[8];
    snprintf(min_str, sizeof(min_str), "%.1fV", (min_val / 4095.0f) * 15.0f);
    snprintf(max_str, sizeof(max_str), "%.1fV", (max_val / 4095.0f) * 15.0f);
    uint16_t mid_val = (min_val + max_val) / 2;
    snprintf(mid_str, sizeof(mid_str), "%.1fV", (mid_val / 4095.0f) * 15.0f);
    
    tft_draw_string(x + 5, y + h - 12, min_str, TFT_SILVER, TFT_BLACK, 1);
    tft_draw_string(x + 5, y + (h / 2) - 3, mid_str, TFT_SILVER, TFT_BLACK, 1);
    tft_draw_string(x + 5, y + 20, max_str, TFT_SILVER, TFT_BLACK, 1);
}

void update_dashboard(void)
{
    // Update only the changing values (no full screen clear)
    update_header_time();
    
    update_angle_panel_value(5, 35, solar_data.elevation, solar_data.delta_elevation, TFT_AMBER);
    update_angle_panel_value(165, 35, solar_data.azimuth, solar_data.delta_azimuth, TFT_SUNGLOW);
    
    update_battery_panel_value(325, 35);
    
    // Only update status if it changed
    static int prev_status = -1;
    int curr_status = (solar_data.elevation > 10.0f) ? 1 : 0;
    if (curr_status != prev_status) {
        update_status_panel(5, 110);
        prev_status = curr_status;
    }
    
    update_battery_graph_incremental(5, 185, 470, 130);
}

void draw_dashboard(void)
{
    tft_fill_screen(TFT_BLACK);
    
    // Header (30 pixels tall)
    draw_header();
    
    // Top row: 3 panels side by side (y=35, height=70)
    draw_angle_panel(5, 35, "ELEVATION", solar_data.elevation, solar_data.delta_elevation, TFT_AMBER);
    draw_angle_panel(165, 35, "AZIMUTH", solar_data.azimuth, solar_data.delta_azimuth, TFT_SUNGLOW);
    draw_battery_panel(325, 35);
    
    // Second row: Status panel (y=110, height=70)
    draw_status_panel(5, 110);
    
    // Add location and info text
    tft_draw_string(165, 115, "Location: Auburn, AL", TFT_SILVER, TFT_BLACK, 1);
    tft_draw_string(165, 130, "Lat: 32.6N  Lon: 85.5W", TFT_TEAL, TFT_BLACK, 1);
    tft_draw_string(165, 145, "System v1.0", TFT_MINT, TFT_BLACK, 1);
    tft_draw_string(165, 160, "ESP32 Controller", TFT_SLATE, TFT_BLACK, 1);
    
    // Graph takes bottom section (y=185, height=130)
    draw_battery_graph(5, 185, 470, 130);
}

// ===== Main =====

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Solar Tracker Dashboard");
    ESP_LOGI(TAG, "  Location: Auburn, AL");
    ESP_LOGI(TAG, "========================================");

    tft_init_pins();
    tft_init_spi();
    tft_reset();
    tft_init_display();
    
    init_solar_data();
    
    ESP_LOGI(TAG, "Starting dashboard updates...");
    ESP_LOGI(TAG, "Initial time: 12:50 AM");
    
    // Draw initial dashboard once
    draw_dashboard();
    
    while (1) {
        update_solar_data();
        update_dashboard();  // Use incremental update instead of full redraw
        
        uint32_t hours = (solar_data.timestamp / 3600) % 24;
        uint32_t minutes = (solar_data.timestamp / 60) % 60;
        
        ESP_LOGI(TAG, "Time: %02lu:%02lu | Elev: %.1f° Az: %.1f° Batt: %.2fV (%d ADC)", 
                 hours, minutes,
                 solar_data.elevation, 
                 solar_data.azimuth,
                 solar_data.battery_voltage,
                 solar_data.battery_adc);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
