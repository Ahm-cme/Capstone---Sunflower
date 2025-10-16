#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ BN-880 NMEA Parser + HMC5883 Compass Driver                          │
    │                                                                       │
    │ HMC5883 I2C Protocol:                                                 │
    │  - Address: 0x1E (7-bit)                                              │
    │  - Registers: 0x00-0x0C                                               │
    │  - Data format: 16-bit signed integers (big-endian)                   │
    │  - Update rate: 75 Hz max (we use 15 Hz for power efficiency)        │
    │                                                                       │
    │ Compass Calibration:                                                  │
    │  - Hard iron offset: nearby permanent magnets (motors, batteries)     │
    │  - Soft iron distortion: ferromagnetic materials (steel frame)        │
    │  - Solution: capture min/max during full rotation, apply offset       │
    └───────────────────────────────────────────────────────────────────────┘
*/

#define TAG "GPS"
#define UART_BUF_SIZE 1024
#define NMEA_MAX_LINE 120

// HMC5883 I2C registers
#define HMC5883_ADDR        0x1E
#define HMC5883_REG_CONFIG_A    0x00
#define HMC5883_REG_CONFIG_B    0x01
#define HMC5883_REG_MODE        0x02
#define HMC5883_REG_DATA_X_MSB  0x03
#define HMC5883_REG_STATUS      0x09
#define HMC5883_REG_ID_A        0x0A

// Compass calibration data
typedef struct {
    int16_t x_min, x_max;
    int16_t y_min, y_max;
    int16_t z_min, z_max;
    bool calibrated;
} compass_cal_t;

static gps_cfg_t  s_cfg;
static gps_data_t s_last = {0};
static compass_cal_t s_compass_cal = {0};

/* ────────────────────────── NMEA Helpers ─────────────────────────────── */

// Calculate NMEA checksum (XOR of all chars between $ and *)
static uint8_t nmea_checksum(const char *sentence) {
    uint8_t checksum = 0;
    if (*sentence == '$') sentence++;
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}

// Verify NMEA checksum
static bool nmea_verify(const char *sentence) {
    const char *star = strchr(sentence, '*');
    if (!star) return false;
    
    uint8_t calc = nmea_checksum(sentence);
    uint8_t recv = (uint8_t)strtol(star + 1, NULL, 16);
    
    return calc == recv;
}

// Convert NMEA coordinate format (ddmm.mmmm) to decimal degrees
static double nmea_to_degrees(double raw, char hemisphere) {
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double result = degrees + (minutes / 60.0);
    
    if (hemisphere == 'S' || hemisphere == 'W') {
        result = -result;
    }
    
    return result;
}

/* ────────────────────────── HMC5883 I2C Helpers ──────────────────────── */

// Write single byte to HMC5883 register
static esp_err_t hmc5883_write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(s_cfg.i2c_port, HMC5883_ADDR, 
                                      data, 2, pdMS_TO_TICKS(100));
}

// Read multiple bytes from HMC5883 register
static esp_err_t hmc5883_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(s_cfg.i2c_port, HMC5883_ADDR,
                                        &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

// Initialize HMC5883 compass
static esp_err_t hmc5883_init(void) {
    esp_err_t ret;
    
    // Verify device ID (should read 'H43' = 0x48, 0x34, 0x33)
    uint8_t id[3];
    ret = hmc5883_read_reg(HMC5883_REG_ID_A, id, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HMC5883 not responding on I2C");
        return ret;
    }
    
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) {
        ESP_LOGE(TAG, "HMC5883 ID mismatch: %02X %02X %02X", id[0], id[1], id[2]);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "HMC5883 compass detected");
    
    // Configure: 8 samples averaged, 15 Hz output rate, normal measurement
    ret = hmc5883_write_reg(HMC5883_REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) return ret;
    
    // Set gain to ±1.3 Ga (default, good for most locations)
    ret = hmc5883_write_reg(HMC5883_REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) return ret;
    
    // Set continuous measurement mode
    ret = hmc5883_write_reg(HMC5883_REG_MODE, 0x00);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "HMC5883 configured: 15Hz, continuous mode");
    
    return ESP_OK;
}

// Read raw magnetic field values
static esp_err_t hmc5883_read_raw(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    esp_err_t ret = hmc5883_read_reg(HMC5883_REG_DATA_X_MSB, data, 6);
    if (ret != ESP_OK) return ret;
    
    // HMC5883 data format: X, Z, Y (note: Z in middle!)
    *x = (int16_t)((data[0] << 8) | data[1]);
    *z = (int16_t)((data[2] << 8) | data[3]);
    *y = (int16_t)((data[4] << 8) | data[5]);
    
    return ESP_OK;
}

// Load compass calibration from NVS
static void compass_load_calibration(void) {
    nvs_handle_t h;
    esp_err_t ret = nvs_open("gps", NVS_READONLY, &h);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No compass calibration in NVS");
        s_compass_cal.calibrated = false;
        return;
    }
    
    size_t required_size = sizeof(compass_cal_t);
    ret = nvs_get_blob(h, "compass_cal", &s_compass_cal, &required_size);
    nvs_close(h);
    
    if (ret == ESP_OK && s_compass_cal.calibrated) {
        ESP_LOGI(TAG, "Compass calibration loaded: X[%d,%d] Y[%d,%d] Z[%d,%d]",
                 s_compass_cal.x_min, s_compass_cal.x_max,
                 s_compass_cal.y_min, s_compass_cal.y_max,
                 s_compass_cal.z_min, s_compass_cal.z_max);
    } else {
        ESP_LOGW(TAG, "Compass not calibrated - heading will be inaccurate");
        s_compass_cal.calibrated = false;
    }
}

// Save compass calibration to NVS
static void compass_save_calibration(void) {
    nvs_handle_t h;
    esp_err_t ret = nvs_open("gps", NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for compass calibration");
        return;
    }
    
    ret = nvs_set_blob(h, "compass_cal", &s_compass_cal, sizeof(compass_cal_t));
    if (ret == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "Compass calibration saved to NVS");
    }
    nvs_close(h);
}

/* ────────────────────────── NMEA Parsers ─────────────────────────────── */

// Parse $GPGGA sentence
static bool parse_gga(const char *sentence, gps_data_t *g) {
    char lat_dir, lon_dir;
    double lat_raw, lon_raw;
    int fix_quality;
    
    int parsed = sscanf(sentence, 
        "$G%*cGGA,%*f,%lf,%c,%lf,%c,%d,%hhu,%*f,%lf,M",
        &lat_raw, &lat_dir, &lon_raw, &lon_dir, 
        &fix_quality, &g->num_satellites, &g->altitude_m);
    
    if (parsed < 7) {
        ESP_LOGD(TAG, "GGA parse failed (parsed %d fields)", parsed);
        return false;
    }
    
    g->latitude = nmea_to_degrees(lat_raw, lat_dir);
    g->longitude = nmea_to_degrees(lon_raw, lon_dir);
    g->fix_type = (fix_quality > 0) ? fix_quality : 0;
    
    ESP_LOGD(TAG, "GGA: lat=%.7f lon=%.7f alt=%.1f fix=%u sats=%u",
             g->latitude, g->longitude, g->altitude_m, 
             g->fix_type, g->num_satellites);
    
    return (fix_quality > 0);
}

// Parse $GPRMC sentence
static bool parse_rmc(const char *sentence, gps_data_t *g) {
    char status, lat_dir, lon_dir;
    double lat_raw, lon_raw, speed_knots, track_deg;
    
    int parsed = sscanf(sentence,
        "$G%*cRMC,%*f,%c,%lf,%c,%lf,%c,%lf,%lf",
        &status, &lat_raw, &lat_dir, &lon_raw, &lon_dir,
        &speed_knots, &track_deg);
    
    if (parsed < 7 || status != 'A') {
        ESP_LOGD(TAG, "RMC parse failed or invalid (parsed %d, status %c)", parsed, status);
        return false;
    }
    
    g->latitude = nmea_to_degrees(lat_raw, lat_dir);
    g->longitude = nmea_to_degrees(lon_raw, lon_dir);
    g->ground_speed_mps = speed_knots * 0.514444f;
    g->heading_deg = (float)track_deg;
    
    ESP_LOGD(TAG, "RMC: lat=%.7f lon=%.7f speed=%.2f m/s heading=%.1f°",
             g->latitude, g->longitude, g->ground_speed_mps, g->heading_deg);
    
    return true;
}

/* ─────────────────────────── Public API ──────────────────────────────── */

esp_err_t gps_init(const gps_cfg_t *cfg) {
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "Initializing BN-880 GPS module + HMC5883 compass...");
    
    // Configure UART for GPS
    uart_config_t uart_config = {
        .baud_rate = s_cfg.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_port, 
                                  s_cfg.tx_io, s_cfg.rx_io,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_port, UART_BUF_SIZE, 
                                         0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "GPS UART%d: TX=%d RX=%d @ %d baud",
             s_cfg.uart_port, s_cfg.tx_io, s_cfg.rx_io, s_cfg.baud_rate);
    
    uart_flush(s_cfg.uart_port);
    
    // Configure I2C for compass
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = s_cfg.sda_io,
        .scl_io_num = s_cfg.scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100 kHz for HMC5883
    };
    
    ESP_ERROR_CHECK(i2c_param_config(s_cfg.i2c_port, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(s_cfg.i2c_port, I2C_MODE_MASTER, 0, 0, 0));
    
    ESP_LOGI(TAG, "I2C%d: SDA=%d SCL=%d @ 100kHz",
             s_cfg.i2c_port, s_cfg.sda_io, s_cfg.scl_io);
    
    // Initialize HMC5883 compass
    esp_err_t ret = hmc5883_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HMC5883 init failed - compass features disabled");
    } else {
        compass_load_calibration();
    }
    
    return ESP_OK;
}

bool gps_poll_nav_pvt(gps_data_t *out) {
    char line[NMEA_MAX_LINE];
    int line_pos = 0;
    bool got_gga = false;
    bool got_rmc = false;
    gps_data_t temp = {0};
    
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms) < 2000) {
        uint8_t byte;
        int len = uart_read_bytes(s_cfg.uart_port, &byte, 1, pdMS_TO_TICKS(100));
        
        if (len <= 0) continue;
        
        if (byte == '\n') {
            line[line_pos] = '\0';
            
            if (line_pos > 0 && line[0] == '$') {
                if (!nmea_verify(line)) {
                    ESP_LOGD(TAG, "Checksum fail: %s", line);
                } else {
                    if (strstr(line, "GGA")) {
                        if (parse_gga(line, &temp)) {
                            got_gga = true;
                        }
                    } else if (strstr(line, "RMC")) {
                        if (parse_rmc(line, &temp)) {
                            got_rmc = true;
                        }
                    }
                    
                    if (got_gga && got_rmc) {
                        temp.valid = true;
                        temp.timestamp = time(NULL);
                        s_last = temp;
                        
                        if (out) *out = temp;
                        
                        ESP_LOGI(TAG, "Fix=%u SV=%u Lat=%.7f Lon=%.7f Alt=%.1fm V=%.2fm/s Head=%.1f°",
                                 temp.fix_type, temp.num_satellites, temp.latitude, temp.longitude,
                                 temp.altitude_m, temp.ground_speed_mps, temp.heading_deg);
                        
                        return true;
                    }
                }
            }
            
            line_pos = 0;
        } else if (line_pos < NMEA_MAX_LINE - 1) {
            line[line_pos++] = byte;
        }
    }
    
    ESP_LOGD(TAG, "GPS poll timeout (got_gga=%d got_rmc=%d)", got_gga, got_rmc);
    return false;
}

bool gps_get_last(gps_data_t *out) {
    if (!s_last.valid) return false;
    if (out) *out = s_last;
    return true;
}

bool gps_get_compass_heading(float *heading_deg) {
    int16_t x, y, z;
    
    // Read raw magnetic field
    if (hmc5883_read_raw(&x, &y, &z) != ESP_OK) {
        ESP_LOGD(TAG, "Compass read failed");
        return false;
    }
    
    // Apply calibration if available
    float x_cal = x;
    float y_cal = y;
    
    if (s_compass_cal.calibrated) {
        // Apply hard iron offset correction
        x_cal = x - (s_compass_cal.x_min + s_compass_cal.x_max) / 2.0f;
        y_cal = y - (s_compass_cal.y_min + s_compass_cal.y_max) / 2.0f;
    }
    
    // Calculate heading (atan2 returns -π to π, convert to 0-360°)
    float heading = atan2f(y_cal, x_cal) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    
    *heading_deg = heading;
    
    ESP_LOGD(TAG, "Compass: raw[%d,%d,%d] heading=%.1f°", x, y, z, heading);
    
    return true;
}

bool gps_calibrate_compass(void) {
    ESP_LOGI(TAG, "=== COMPASS CALIBRATION ===");
    ESP_LOGI(TAG, "Slowly rotate system 360° horizontally (2-3 circles)...");
    
    // Reset calibration data
    s_compass_cal.x_min = 32767;
    s_compass_cal.x_max = -32768;
    s_compass_cal.y_min = 32767;
    s_compass_cal.y_max = -32768;
    s_compass_cal.z_min = 32767;
    s_compass_cal.z_max = -32768;
    s_compass_cal.calibrated = false;
    
    // Collect samples for 20 seconds
    uint32_t start = xTaskGetTickCount();
    int sample_count = 0;
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(20000)) {
        int16_t x, y, z;
        
        if (hmc5883_read_raw(&x, &y, &z) == ESP_OK) {
            // Update min/max for each axis
            if (x < s_compass_cal.x_min) s_compass_cal.x_min = x;
            if (x > s_compass_cal.x_max) s_compass_cal.x_max = x;
            if (y < s_compass_cal.y_min) s_compass_cal.y_min = y;
            if (y > s_compass_cal.y_max) s_compass_cal.y_max = y;
            if (z < s_compass_cal.z_min) s_compass_cal.z_min = z;
            if (z > s_compass_cal.z_max) s_compass_cal.z_max = z;
            
            sample_count++;
            
            // Log progress every 2 seconds
            if (sample_count % 30 == 0) {
                ESP_LOGI(TAG, "Samples: %d, X[%d,%d] Y[%d,%d] Z[%d,%d]",
                         sample_count,
                         s_compass_cal.x_min, s_compass_cal.x_max,
                         s_compass_cal.y_min, s_compass_cal.y_max,
                         s_compass_cal.z_min, s_compass_cal.z_max);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(67));  // ~15 Hz
    }
    
    // Validate calibration quality
    int x_range = s_compass_cal.x_max - s_compass_cal.x_min;
    int y_range = s_compass_cal.y_max - s_compass_cal.y_min;
    
    if (x_range < 200 || y_range < 200) {
        ESP_LOGW(TAG, "Calibration failed: insufficient rotation (X=%d Y=%d)",
                 x_range, y_range);
        return false;
    }
    
    s_compass_cal.calibrated = true;
    compass_save_calibration();
    
    ESP_LOGI(TAG, "Calibration complete: %d samples collected", sample_count);
    ESP_LOGI(TAG, "Ranges: X=%d Y=%d Z=%d", x_range, y_range,
             s_compass_cal.z_max - s_compass_cal.z_min);
    
    return true;
}

bool gps_is_compass_calibrated(void) {
    return s_compass_cal.calibrated;
}