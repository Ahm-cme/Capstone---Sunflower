#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sunflower_logo.h"

/*
--------------------------------------------------------------------------------
Sunflower Secondary LCD (ILI9486 over SPI)
--------------------------------------------------------------------------------
Responsibilities:
- Initialize SPI + ILI9486 panel
- Provide basic drawing primitives (fill, pixel, line, rect, text)
- Render dashboard: header, angle panels, battery, status, battery graph
- Provide splash/init/error screens

Hardware notes:
- ESP32 classic (SPI2_HOST aka VSPI). MISO unused (display write-only)
- Pins: MOSI=23, SCLK=18, CS=5, DC=21, RST=4
- SPI clk set to 10MHz for reliability; can push higher if wiring is short

Performance notes:
- Uses blocking spi_device_polling_transmit; acceptable for UI rate (<10 FPS)
- Many functions draw per-pixel (slow). For production, batch to line buffers
- Consider heap_caps_malloc(MALLOC_CAP_DMA) for DMA-friendly buffers

Status semantics:
- tracking_status: 0=STANDBY (amber), 1=TRACKING (green), 2=SLEEP (amber),
  3=CALIB (amber), 255=ERROR (red)

-------------------------------------------------------------------------------
*/

// Pin definitions
#define TFT_MOSI    23   // VSPI MOSI
#define TFT_SCLK    18   // VSPI SCLK
#define TFT_CS      5    // VSPI CS (handled by SPI device)
#define TFT_DC      21   // Data/Command select
#define TFT_RST     4    // Panel reset

// ILI9486 Commands
#define ILI9486_NOP         0x00
#define ILI9486_SWRESET     0x01
#define ILI9486_SLPOUT      0x11
#define ILI9486_NORON       0x13
#define ILI9486_INVOFF      0x20
#define ILI9486_DISPON      0x29
#define ILI9486_CASET       0x2A   // Column address set
#define ILI9486_PASET       0x2B   // Page (row) address set
#define ILI9486_RAMWR       0x2C   // Memory write
#define ILI9486_MADCTL      0x36   // Memory Access Control (rotation)
#define ILI9486_PIXFMT      0x3A   // Pixel format (0x55 = RGB565)
#define ILI9486_FRMCTR1     0xB1
#define ILI9486_INVCTR      0xB4
#define ILI9486_DFUNCTR     0xB6
#define ILI9486_PWCTR1      0xC0
#define ILI9486_PWCTR2      0xC1
#define ILI9486_PWCTR3      0xC2
#define ILI9486_VMCTR1      0xC5
#define ILI9486_GMCTRP1     0xE0
#define ILI9486_GMCTRN1     0xE1

// Simple 5x7 bitmap font (ASCII 32-126)
// NOTE: Compact font for low memory footprint; consider larger fonts for readability.
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

static const char *TAG = "LCD";
static spi_device_handle_t spi;
static lcd_display_data_t current_data;   // Mutable copy of what's on-screen
static uint16_t battery_history[100];     // Rolling ADC history for graph
static int history_index = 0;

// ===== Low-level SPI functions =====

static void tft_spi_pre_transfer_callback(spi_transaction_t *t)
{
    // DC=0 for command, DC=1 for data (value passed via txn->user)
    int dc = (int)t->user;
    gpio_set_level(TFT_DC, dc);
}

static void tft_cmd(uint8_t cmd)
{
    // Send single-byte command (blocking)
    // NOTE: Consider queueing for batch performance if needed.
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void tft_data(uint8_t data)
{
    // Send single-byte data (blocking)
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void tft_data_buf(uint8_t *data, int len)
{
    // Send buffer payload (blocking)
    // TODO: Use DMA-capable buffer if we see throughput issues.
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

static void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // Set drawing window for subsequent RAMWR pixel writes
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

void lcd_clear_screen(uint16_t color)
{
    // Fast full-screen fill using line chunks
    // TODO: Reuse a static buffer to avoid malloc/free jitter.
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

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Solid rectangle fill. Clips to screen bounds.
    // FIXME: Allocate 'line' once and reuse; current malloc per call is costly.
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

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    // Single pixel write (slow). Prefer batching when possible.
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    tft_set_addr_window(x, y, x, y);
    tft_data(color >> 8);
    tft_data(color & 0xFF);
}

void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    // Bresenham line algorithm; integer math for speed
    // TODO: Add Cohen–Sutherland clip if needed for off-screen segments.
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        lcd_draw_pixel(x0, y0, color);
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

void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Rectangle outline using 4 lines
    lcd_draw_line(x, y, x + w - 1, y, color);
    lcd_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
    lcd_draw_line(x + w - 1, y + h - 1, x, y + h - 1, color);
    lcd_draw_line(x, y + h - 1, x, y, color);
}

// ===== Text rendering functions =====

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size)
{
    // 5x7 bitmap font scaled by 'size'; draws background to avoid artifacts
    if (c < ' ' || c > 'z') c = '?';
    int index = c - ' ';
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[index][i];
        for (int j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) {
                    lcd_draw_pixel(x + i, y + j, color);
                } else {
                    lcd_fill_rect(x + i * size, y + j * size, size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    lcd_draw_pixel(x + i, y + j, bg);
                } else {
                    lcd_fill_rect(x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size)
{
    // Basic string with newline support; monospaced width 6*size
    uint16_t cursor_x = x;
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

static void tft_draw_image(uint16_t x, uint16_t y, const uint16_t *image, uint16_t w, uint16_t h, uint16_t transparent_color)
{
    // Very simple blit with chroma key (transparent_color). Per-pixel writes.
    // NOTE: 'image' is const -> stored in flash; reads are fine on ESP32.
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            uint16_t pixel = image[j * w + i];
            if (pixel != transparent_color) {
                if (x + i >= 0 && x + i < TFT_WIDTH && y + j >= 0 && y + j < TFT_HEIGHT) {
                    lcd_draw_pixel(x + i, y + j, pixel);
                }
            }
        }
    }
}

// ===== Hardware initialization =====

static void tft_init_pins(void)
{
    // Configure DC/RST as GPIO outputs. CS is managed by SPI driver.
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "GPIO pins configured");
}

static void tft_init_spi(void)
{
    // SPI2_HOST (VSPI): 10MHz, mode 0
    // NOTE: Can try 26-40MHz with short wires and proper level shifting.
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

static void tft_reset(void)
{
    // Hard reset pulse
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static void tft_init_display(void)
{
    // Basic ILI9486 bring-up sequence (RGB565, rotation, gamma, power)
    // MADCTL=0x28 -> landscape orientation chosen for 480x320
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

// ===== Dashboard drawing functions =====

static void draw_header(void)
{
    // Top bar: logo, title, 12-hour clock with AM/PM
    // Time derived from current_data.timestamp (seconds)
    lcd_fill_rect(0, 0, TFT_WIDTH, 30, TFT_CHARCOAL);
    
    tft_draw_image(10, 3, sunflower_logo, SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT, TFT_CHARCOAL);
    
    lcd_draw_string(40, 10, "SUNFLOWER", TFT_GOLDEN, TFT_CHARCOAL, 2);
    
    uint32_t hours = (current_data.timestamp / 3600) % 24;
    uint32_t minutes = (current_data.timestamp / 60) % 60;
    uint32_t seconds = current_data.timestamp % 60;
    
    const char *period = (hours < 12) ? "AM" : "PM";
    uint32_t display_hours = hours % 12;
    if (display_hours == 0) display_hours = 12;
    
    char time_str[24];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu%s", display_hours, minutes, seconds, period);
    lcd_draw_string(330, 10, time_str, TFT_GOLDEN, TFT_CHARCOAL, 2);
}

static void draw_angle_panel(uint16_t x, uint16_t y, const char *label, float angle, float delta, uint16_t color)
{
    // Generic panel used for Elevation/Azimuth with large value + delta
    lcd_fill_rect(x, y, 150, 70, TFT_STEEL);
    lcd_draw_rect(x, y, 150, 70, TFT_SLATE);
    
    lcd_draw_string(x + 5, y + 5, label, TFT_SILVER, TFT_STEEL, 1);
    
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", angle);
    lcd_draw_string(x + 5, y + 25, angle_str, color, TFT_STEEL, 3);
    lcd_draw_string(x + 5, y + 52, "deg", TFT_SILVER, TFT_STEEL, 1);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.3f", delta);
    lcd_draw_string(x + 50, y + 52, delta_str, TFT_TEAL, TFT_STEEL, 1);
}

static void draw_battery_panel(uint16_t x, uint16_t y)
{
    // Battery panel: voltage (big), ADC raw, and a simple level bar
    // FIXME: ADC->bar mapping (2000..3400) is placeholder; calibrate with real pack.
    lcd_fill_rect(x, y, 150, 70, TFT_DARKGREY);
    lcd_draw_rect(x, y, 150, 70, TFT_SAGE);
    
    lcd_draw_string(x + 5, y + 5, "BATTERY", TFT_SILVER, TFT_DARKGREY, 1);
    
    char volt_str[16];
    snprintf(volt_str, sizeof(volt_str), "%.2fV", current_data.battery_voltage);
    lcd_draw_string(x + 5, y + 25, volt_str, TFT_GOLDEN, TFT_DARKGREY, 3);
    
    char adc_str[16];
    snprintf(adc_str, sizeof(adc_str), "ADC:%d", current_data.battery_adc);
    lcd_draw_string(x + 5, y + 52, adc_str, TFT_SILVER, TFT_DARKGREY, 1);
    
    int bar_width = (int)(((current_data.battery_adc - 2000) / 1400.0f) * 60);
    if (bar_width > 60) bar_width = 60;
    if (bar_width < 0) bar_width = 0;
    
    lcd_fill_rect(x + 80, y + 50, 60, 8, TFT_BLACK);
    lcd_fill_rect(x + 80, y + 50, bar_width, 8, TFT_SAGE);
    lcd_draw_rect(x + 80, y + 50, 60, 8, TFT_SILVER);
}

static void draw_status_panel(uint16_t x, uint16_t y)
{
    // Status panel color coding:
    // 1=TRACKING(GREEN), 0/2/3=AMBER, 255=ERROR(RED)
    uint16_t status_color;
    const char *status_text;
    uint16_t text_color;
    
    if (current_data.tracking_status == 1) {
        status_color = TFT_SAGE;
        status_text = "TRACKING";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 2) {
        status_color = TFT_AMBER;
        status_text = "SLEEPING";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 3) {
        status_color = TFT_AMBER;
        status_text = "CALIBRATE";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 255) {
        status_color = TFT_CRIMSON;
        status_text = "ERROR";
        text_color = TFT_WHITE;
    } else {
        status_color = TFT_AMBER;
        status_text = "STANDBY";
        text_color = TFT_BLACK;
    }
    
    lcd_fill_rect(x, y, 150, 70, status_color);
    lcd_draw_rect(x, y, 150, 70, TFT_SILVER);
    
    lcd_draw_string(x + 5, y + 5, "STATUS", TFT_BLACK, status_color, 1);
    lcd_draw_string(x + 10, y + 30, status_text, text_color, status_color, 2);
}

static void draw_battery_graph(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    // Historic battery ADC plot (100 samples). Min/Max autoscale.
    // NOTE: Labels assume 0..4095 -> 0..~15V map; adjust to real divider ratio.
    lcd_fill_rect(x, y, w, h, TFT_BLACK);
    lcd_draw_rect(x, y, w, h, TFT_SLATE);
    
    lcd_draw_string(x + 5, y + 5, "Battery Voltage History (Last 100 Readings)", TFT_SILVER, TFT_BLACK, 1);
    
    for (int i = 0; i <= 4; i++) {
        int grid_y = y + 20 + (h - 30) * i / 4;
        lcd_draw_line(x + 35, grid_y, x + w - 5, grid_y, TFT_DARKGREY);
    }
    
    uint16_t min_val = 4095, max_val = 0;
    for (int i = 0; i < 100; i++) {
        if (battery_history[i] < min_val) min_val = battery_history[i];
        if (battery_history[i] > max_val) max_val = battery_history[i];
    }
    
    if (max_val == min_val) max_val = min_val + 100;
    
    int graph_w = w - 45;
    int graph_h = h - 35;
    
    for (int i = 1; i < 100; i++) {
        int prev_index = (history_index + i - 1) % 100;
        int curr_index = (history_index + i) % 100;
        
        int x1 = x + 35 + (graph_w * (i - 1) / 99);
        int y1 = y + 25 + graph_h - ((battery_history[prev_index] - min_val) * graph_h / (max_val - min_val));
        
        int x2 = x + 35 + (graph_w * i / 99);
        int y2 = y + 25 + graph_h - ((battery_history[curr_index] - min_val) * graph_h / (max_val - min_val));
        
        lcd_draw_line(x1, y1, x2, y2, TFT_SAGE);
        
        if (i == 99) {
            lcd_fill_rect(x2 - 1, y2 - 1, 3, 3, TFT_AMBER);
        }
    }
    
    char min_str[8], max_str[8], mid_str[8];
    snprintf(min_str, sizeof(min_str), "%.1fV", (min_val / 4095.0f) * 15.0f);
    snprintf(max_str, sizeof(max_str), "%.1fV", (max_val / 4095.0f) * 15.0f);
    uint16_t mid_val = (min_val + max_val) / 2;
    snprintf(mid_str, sizeof(mid_str), "%.1fV", (mid_val / 4095.0f) * 15.0f);
    
    lcd_draw_string(x + 5, y + h - 12, min_str, TFT_SILVER, TFT_BLACK, 1);
    lcd_draw_string(x + 5, y + (h / 2) - 3, mid_str, TFT_SILVER, TFT_BLACK, 1);
    lcd_draw_string(x + 5, y + 20, max_str, TFT_SILVER, TFT_BLACK, 1);
}

// ===== Update functions (incremental) =====

static void update_header_time(void)
{
    // Rewrite only the time area in the header to reduce flicker
    uint32_t hours = (current_data.timestamp / 3600) % 24;
    uint32_t minutes = (current_data.timestamp / 60) % 60;
    uint32_t seconds = current_data.timestamp % 60;
    
    const char *period = (hours < 12) ? "AM" : "PM";
    uint32_t display_hours = hours % 12;
    if (display_hours == 0) display_hours = 12;
    
    char time_str[24];
    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu%s", display_hours, minutes, seconds, period);
    
    lcd_fill_rect(330, 10, 140, 16, TFT_CHARCOAL);
    lcd_draw_string(330, 10, time_str, TFT_GOLDEN, TFT_CHARCOAL, 2);
}

static void update_angle_panel_value(uint16_t x, uint16_t y, float angle, float delta, uint16_t color)
{
    // Partial redraw of angle value and delta line only
    lcd_fill_rect(x + 5, y + 25, 140, 24, TFT_STEEL);
    lcd_fill_rect(x + 50, y + 52, 95, 8, TFT_STEEL);
    
    char angle_str[16];
    snprintf(angle_str, sizeof(angle_str), "%.1f", angle);
    lcd_draw_string(x + 5, y + 25, angle_str, color, TFT_STEEL, 3);
    
    char delta_str[16];
    snprintf(delta_str, sizeof(delta_str), "D:%.3f", delta);
    lcd_draw_string(x + 50, y + 52, delta_str, TFT_TEAL, TFT_STEEL, 1);
}

static void update_battery_panel_value(uint16_t x, uint16_t y)
{
    // Partial redraw of battery text + level bar
    lcd_fill_rect(x + 5, y + 25, 140, 24, TFT_DARKGREY);
    lcd_fill_rect(x + 5, y + 52, 140, 8, TFT_DARKGREY);
    
    char volt_str[16];
    snprintf(volt_str, sizeof(volt_str), "%.2fV", current_data.battery_voltage);
    lcd_draw_string(x + 5, y + 25, volt_str, TFT_GOLDEN, TFT_DARKGREY, 3);
    
    char adc_str[16];
    snprintf(adc_str, sizeof(adc_str), "ADC:%d", current_data.battery_adc);
    lcd_draw_string(x + 5, y + 52, adc_str, TFT_SILVER, TFT_DARKGREY, 1);
    
    int bar_width = (int)(((current_data.battery_adc - 2000) / 1400.0f) * 60);
    if (bar_width > 60) bar_width = 60;
    if (bar_width < 0) bar_width = 0;
    
    lcd_fill_rect(x + 80, y + 50, 60, 8, TFT_BLACK);
    lcd_fill_rect(x + 80, y + 50, bar_width, 8, TFT_SAGE);
    lcd_draw_rect(x + 80, y + 50, 60, 8, TFT_SILVER);
}

static void update_status_panel(uint16_t x, uint16_t y)
{
    // Full redraw of status panel due to background color change per state
    uint16_t status_color;
    const char *status_text;
    uint16_t text_color;
    
    if (current_data.tracking_status == 1) {
        status_color = TFT_SAGE;
        status_text = "TRACKING";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 2) {
        status_color = TFT_AMBER;
        status_text = "SLEEPING";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 3) {
        status_color = TFT_AMBER;
        status_text = "CALIBRATE";
        text_color = TFT_BLACK;
    } else if (current_data.tracking_status == 255) {
        status_color = TFT_CRIMSON;
        status_text = "ERROR";
        text_color = TFT_WHITE;
    } else {
        status_color = TFT_AMBER;
        status_text = "STANDBY";
        text_color = TFT_BLACK;
    }
    
    lcd_fill_rect(x, y, 150, 70, status_color);
    lcd_draw_rect(x, y, 150, 70, TFT_SILVER);
    
    lcd_draw_string(x + 5, y + 5, "STATUS", TFT_BLACK, status_color, 1);
    lcd_draw_string(x + 10, y + 30, status_text, text_color, status_color, 2);
}

static void update_battery_graph_incremental(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    // Currently falls back to full redraw. Optimize later to draw only latest segment.
    // FIXME: Draw only last segment between (index-1)->(index) to cut SPI traffic.
    draw_battery_graph(x, y, w, h);
}

// ===== Public API functions =====

esp_err_t lcd_init(void)
{
    // Bring-up sequence: pins -> SPI -> reset -> panel init -> clear state
    ESP_LOGI(TAG, "Initializing LCD display...");
    
    tft_init_pins();
    tft_init_spi();
    tft_reset();
    tft_init_display();
    
    memset(&current_data, 0, sizeof(lcd_display_data_t));
    memset(battery_history, 0, sizeof(battery_history));
    history_index = 0;
    
    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

void lcd_show_init_screen(const char *status[], bool success[], int count)
{
    // Startup checklist screen with per-item OK/FAIL and logo
    lcd_clear_screen(TFT_BLACK);
    
    int logo_x = (TFT_WIDTH - SUNFLOWER_LOGO_WIDTH) / 2;
    tft_draw_image(logo_x, 20, sunflower_logo, SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT, TFT_BLACK);
    
    lcd_draw_string(140, 50, "SUNFLOWER TRACKER", TFT_GOLDEN, TFT_BLACK, 2);
    lcd_draw_string(180, 70, "System Initialization", TFT_SILVER, TFT_BLACK, 1);
    
    lcd_draw_line(40, 90, TFT_WIDTH - 40, 90, TFT_SLATE);
    
    int start_y = 110;
    int item_spacing = 25;
    
    for (int i = 0; i < count; i++) {
        int y_pos = start_y + (i * item_spacing);
        
        uint16_t box_color = success[i] ? TFT_SAGE : TFT_CRIMSON;
        lcd_fill_rect(60, y_pos, 15, 15, box_color);
        lcd_draw_rect(60, y_pos, 15, 15, TFT_SILVER);
        
        if (success[i]) {
            lcd_draw_line(63, y_pos + 7, 67, y_pos + 12, TFT_BLACK);
            lcd_draw_line(67, y_pos + 12, 73, y_pos + 3, TFT_BLACK);
        } else {
            lcd_draw_line(63, y_pos + 3, 72, y_pos + 12, TFT_WHITE);
            lcd_draw_line(72, y_pos + 3, 63, y_pos + 12, TFT_WHITE);
        }
        
        lcd_draw_string(85, y_pos + 3, status[i], TFT_SILVER, TFT_BLACK, 1);
        
        const char *result_text = success[i] ? "OK" : "FAIL";
        uint16_t result_color = success[i] ? TFT_SAGE : TFT_CRIMSON;
        lcd_draw_string(350, y_pos + 3, result_text, result_color, TFT_BLACK, 2);
    }
    
    lcd_draw_line(40, start_y + (count * item_spacing) + 10, 
                  TFT_WIDTH - 40, start_y + (count * item_spacing) + 10, TFT_SLATE);
}

void lcd_show_splash(const char *message)
{
    // Centered logo + optional message
    lcd_clear_screen(TFT_BLACK);
    tft_draw_image(228, 120, sunflower_logo, SUNFLOWER_LOGO_WIDTH, SUNFLOWER_LOGO_HEIGHT, TFT_BLACK);
    
    if (message) {
        int text_width = strlen(message) * 6 * 2;
        int text_x = (TFT_WIDTH - text_width) / 2;
        lcd_draw_string(text_x, 160, message, TFT_GOLDEN, TFT_BLACK, 2);
    }
}

void lcd_show_error(const char *error_msg)
{
    // Red banner at bottom for critical messages
    lcd_fill_rect(0, TFT_HEIGHT - 40, TFT_WIDTH, 40, TFT_CRIMSON);
    lcd_draw_string(10, TFT_HEIGHT - 30, "ERROR:", TFT_WHITE, TFT_CRIMSON, 2);
    lcd_draw_string(10, TFT_HEIGHT - 15, error_msg, TFT_WHITE, TFT_CRIMSON, 1);
}

void lcd_draw_dashboard(const lcd_display_data_t *data)
{
    // Full dashboard draw; seeds battery history with initial value
    memcpy(&current_data, data, sizeof(lcd_display_data_t));
    
    lcd_clear_screen(TFT_BLACK);
    
    draw_header();
    draw_angle_panel(5, 35, "ELEVATION", data->elevation, data->delta_elevation, TFT_AMBER);
    draw_angle_panel(165, 35, "AZIMUTH", data->azimuth, data->delta_azimuth, TFT_SUNGLOW);
    draw_battery_panel(325, 35);
    draw_status_panel(5, 110);
    
    if (data->gps_valid) {
        char lat_str[32], lon_str[32];
        snprintf(lat_str, sizeof(lat_str), "Lat: %.4f%c", fabs(data->latitude), data->latitude >= 0 ? 'N' : 'S');
        snprintf(lon_str, sizeof(lon_str), "Lon: %.4f%c", fabs(data->longitude), data->longitude >= 0 ? 'E' : 'W');
        
        lcd_draw_string(165, 115, "GPS Location:", TFT_SILVER, TFT_BLACK, 1);
        lcd_draw_string(165, 130, lat_str, TFT_TEAL, TFT_BLACK, 1);
        lcd_draw_string(165, 145, lon_str, TFT_TEAL, TFT_BLACK, 1);
    } else {
        lcd_draw_string(165, 115, "GPS: No Fix", TFT_CORAL, TFT_BLACK, 1);
    }
    
    lcd_draw_string(165, 160, "System v1.0", TFT_MINT, TFT_BLACK, 1);
    
    for (int i = 0; i < 100; i++) {
        battery_history[i] = data->battery_adc;
    }
    history_index = 0;
    
    draw_battery_graph(5, 185, 470, 130);
}

void lcd_update_display(const lcd_display_data_t *data)
{
    // Incremental update path; avoids full-screen redraw
    // NOTE: Battery graph still full redraw; optimize later.
    battery_history[history_index] = data->battery_adc;
    history_index = (history_index + 1) % 100;
    
    update_header_time();
    update_angle_panel_value(5, 35, data->elevation, data->delta_elevation, TFT_AMBER);
    update_angle_panel_value(165, 35, data->azimuth, data->delta_azimuth, TFT_SUNGLOW);
    update_battery_panel_value(325, 35);
    
    if (data->tracking_status != current_data.tracking_status) {
        update_status_panel(5, 110);
    }
    
    update_battery_graph_incremental(5, 185, 470, 130);
    
    memcpy(&current_data, data, sizeof(lcd_display_data_t));
}