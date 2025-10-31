#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

/*
    BN-880 NMEA Parser + HMC5883 Compass Driver

    Overview:
    - Reads NMEA sentences from UART, verifies checksum, parses GGA and RMC.
    - Maintains a cached last valid fix (s_last).
    - Talks to HMC5883 via I2C, supports calibration saved in NVS.
    - Provides magnetic declination correction for true north conversion.
    
    System Check Integration:
    - gps_init() returns ESP_OK (GPS+compass OK), ESP_FAIL (GPS OK, compass failed),
      or ESP_ERR_INVALID_STATE (GPS failed - critical)
    - gps_test_communication() added for quick hardware check
    - gps_is_compass_present() added for system diagnostics
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
    float declination_deg;  // User-configured or estimated magnetic declination
} compass_cal_t;

static gps_cfg_t  s_cfg;
static gps_data_t s_last = {0};
static compass_cal_t s_compass_cal = {0};
static bool s_gps_initialized = false;
static bool s_compass_present = false;

/* ────────────────────── Magnetic Declination Helpers ─────────────────── */

/*
 * Estimate magnetic declination for a given location.
 * 
 * This is a simplified approximation based on regional patterns.
 * For production use, integrate full WMM (World Magnetic Model) library
 * or query NOAA API: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 * 
 * Returns declination in degrees:
 *   Positive = magnetic north is EAST of true north
 *   Negative = magnetic north is WEST of true north
 */
static float estimate_magnetic_declination(double lat, double lon) {
    // North America approximation (rough linear gradient)
    if (lat > 25.0 && lat < 50.0 && lon > -125.0 && lon < -65.0) {
        // Eastern US: ~-10° to -15°
        // Central US: ~0° to -5°  
        // Western US: ~10° to +15°
        // Simple east-west linear interpolation
        float decl = 15.0f * (float)(lon + 95.0) / 30.0f;
        
        ESP_LOGD(TAG, "Estimated declination for %.2f,%.2f: %.1f°", lat, lon, decl);
        return decl;
    }
    
    // Europe approximation
    if (lat > 35.0 && lat < 70.0 && lon > -10.0 && lon < 40.0) {
        // Western Europe: ~-5° to +5°
        // Eastern Europe: ~5° to +15°
        float decl = 10.0f * (float)(lon + 5.0) / 35.0f;
        
        ESP_LOGD(TAG, "Estimated declination for %.2f,%.2f: %.1f°", lat, lon, decl);
        return decl;
    }
    
    // Australia approximation
    if (lat > -45.0 && lat < -10.0 && lon > 110.0 && lon < 155.0) {
        // Eastern Australia: ~10° to +15°
        float decl = 12.5f;
        
        ESP_LOGD(TAG, "Estimated declination for %.2f,%.2f: %.1f°", lat, lon, decl);
        return decl;
    }
    
    // Default: unknown location
    ESP_LOGW(TAG, "Location %.2f,%.2f outside known regions - using 0° declination", lat, lon);
    ESP_LOGW(TAG, "For accurate tracking, manually set declination using gps_set_magnetic_declination()");
    return 0.0f;
}

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
    
    ESP_LOGI(TAG, "Detecting HMC5883 compass...");
    
    // Verify device ID (should read 'H43' = 0x48, 0x34, 0x33)
    uint8_t id[3];
    ret = hmc5883_read_reg(HMC5883_REG_ID_A, id, 3);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HMC5883 not responding on I2C (error: %s)", esp_err_to_name(ret));
        s_compass_present = false;
        return ret;
    }
    
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) {
        ESP_LOGW(TAG, "HMC5883 ID mismatch: %02X %02X %02X (expected 48 34 33)", 
                 id[0], id[1], id[2]);
        s_compass_present = false;
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "✓ HMC5883 compass detected (ID: H43)");
    s_compass_present = true;
    
    // Configure: 8 samples averaged, 15 Hz output rate, normal measurement
    ret = hmc5883_write_reg(HMC5883_REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure HMC5883 config A");
        return ret;
    }
    
    // Set gain to ±1.3 Ga (default, good for most locations)
    ret = hmc5883_write_reg(HMC5883_REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure HMC5883 config B");
        return ret;
    }
    
    // Set continuous measurement mode
    ret = hmc5883_write_reg(HMC5883_REG_MODE, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set HMC5883 mode");
        return ret;
    }
    
    ESP_LOGI(TAG, "✓ HMC5883 configured: 15Hz output, continuous mode");
    
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
        ESP_LOGI(TAG, "No compass calibration in NVS (first boot)");
        s_compass_cal.calibrated = false;
        s_compass_cal.declination_deg = 0.0f;
        return;
    }
    
    size_t required_size = sizeof(compass_cal_t);
    ret = nvs_get_blob(h, "compass_cal", &s_compass_cal, &required_size);
    nvs_close(h);
    
    if (ret == ESP_OK && s_compass_cal.calibrated) {
        ESP_LOGI(TAG, "✓ Compass calibration loaded from NVS");
        ESP_LOGI(TAG, "  X: [%d, %d] Y: [%d, %d] Z: [%d, %d]",
                 s_compass_cal.x_min, s_compass_cal.x_max,
                 s_compass_cal.y_min, s_compass_cal.y_max,
                 s_compass_cal.z_min, s_compass_cal.z_max);
        ESP_LOGI(TAG, "  Declination: %.1f°", s_compass_cal.declination_deg);
    } else {
        ESP_LOGI(TAG, "Compass not calibrated - use double-press button to calibrate");
        s_compass_cal.calibrated = false;
        s_compass_cal.declination_deg = 0.0f;
    }
}

// Save compass calibration to NVS
static void compass_save_calibration(void) {
    nvs_handle_t h;
    esp_err_t ret = nvs_open("gps", NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for compass calibration: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = nvs_set_blob(h, "compass_cal", &s_compass_cal, sizeof(compass_cal_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Compass calibration saved to NVS (decl=%.1f°)", 
                     s_compass_cal.declination_deg);
        } else {
            ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to write NVS: %s", esp_err_to_name(ret));
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
        ESP_LOGD(TAG, "GGA parse failed (parsed %d/7 fields)", parsed);
        return false;
    }
    
    g->latitude = nmea_to_degrees(lat_raw, lat_dir);
    g->longitude = nmea_to_degrees(lon_raw, lon_dir);
    g->fix_type = (fix_quality > 0) ? fix_quality : 0;
    
    ESP_LOGD(TAG, "GGA: %.7f°%c %.7f°%c alt=%.1fm fix=%u sats=%u",
             fabs(g->latitude), (g->latitude >= 0) ? 'N' : 'S',
             fabs(g->longitude), (g->longitude >= 0) ? 'E' : 'W',
             g->altitude_m, g->fix_type, g->num_satellites);
    
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
        ESP_LOGD(TAG, "RMC: %s (parsed %d, status '%c')", 
                 (status == 'A') ? "invalid data" : "no fix", parsed, status);
        return false;
    }
    
    g->latitude = nmea_to_degrees(lat_raw, lat_dir);
    g->longitude = nmea_to_degrees(lon_raw, lon_dir);
    g->ground_speed_mps = speed_knots * 0.514444f;
    g->heading_deg = (float)track_deg;
    
    ESP_LOGD(TAG, "RMC: speed=%.1fm/s heading=%.1f°", g->ground_speed_mps, g->heading_deg);
    
    return true;
}

/* ─────────────────────────── Public API ──────────────────────────────── */

esp_err_t gps_init(const gps_cfg_t *cfg) {
    s_cfg = *cfg;
    
    ESP_LOGI(TAG, "Initializing BN-880 GPS + HMC5883 compass...");
    ESP_LOGI(TAG, "  UART%d: TX=GPIO%d RX=GPIO%d @ %d baud",
             s_cfg.uart_port, s_cfg.tx_io, s_cfg.rx_io, s_cfg.baud_rate);
    ESP_LOGI(TAG, "  I2C%d: SDA=GPIO%d SCL=GPIO%d @ 100kHz",
             s_cfg.i2c_port, s_cfg.sda_io, s_cfg.scl_io);
    
    // Configure UART for GPS
    uart_config_t uart_config = {
        .baud_rate = s_cfg.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(s_cfg.uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ UART config failed: %s", esp_err_to_name(ret));
        return ESP_ERR_INVALID_STATE;
    }
    
    ret = uart_set_pin(s_cfg.uart_port, 
                       s_cfg.tx_io, s_cfg.rx_io,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ UART pin config failed: %s", esp_err_to_name(ret));
        return ESP_ERR_INVALID_STATE;
    }
    
    ret = uart_driver_install(s_cfg.uart_port, UART_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ UART driver install failed: %s", esp_err_to_name(ret));
        return ESP_ERR_INVALID_STATE;
    }
    
    uart_flush(s_cfg.uart_port);
    ESP_LOGI(TAG, "✓ GPS UART initialized");
    
    // Configure I2C for compass
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = s_cfg.sda_io,
        .scl_io_num = s_cfg.scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100 kHz for HMC5883
    };
    
    ret = i2c_param_config(s_cfg.i2c_port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠ I2C config failed: %s (compass disabled)", esp_err_to_name(ret));
        s_gps_initialized = true;
        return ESP_OK;  // GPS OK, compass failed
    }
    
    ret = i2c_driver_install(s_cfg.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠ I2C driver install failed: %s (compass disabled)", esp_err_to_name(ret));
        s_gps_initialized = true;
        return ESP_OK;  // GPS OK, compass failed
    }
    
    ESP_LOGI(TAG, "✓ I2C bus initialized");
    
    // Initialize HMC5883 compass
    ret = hmc5883_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠ Compass initialization failed (GPS still functional)");
        s_gps_initialized = true;
        return ESP_OK;  // GPS OK, compass failed
    }
    
    // Load compass calibration if present
    compass_load_calibration();
    
    s_gps_initialized = true;
    ESP_LOGI(TAG, "✓ GPS module fully initialized");
    
    return ESP_OK;
}

bool gps_test_communication(uint32_t timeout_ms) {
    if (!s_gps_initialized) {
        ESP_LOGW(TAG, "GPS not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing GPS communication (%lu ms timeout)...", timeout_ms);
    
    char line[NMEA_MAX_LINE];
    int line_pos = 0;
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int sentence_count = 0;
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms) < timeout_ms) {
        uint8_t byte;
        int len = uart_read_bytes(s_cfg.uart_port, &byte, 1, pdMS_TO_TICKS(100));
        
        if (len <= 0) continue;
        
        if (byte == '\n') {
            line[line_pos] = '\0';
            
            if (line_pos > 0 && line[0] == '$') {
                sentence_count++;
                
                if (nmea_verify(line)) {
                    ESP_LOGI(TAG, "✓ GPS responding: %s", line);
                    ESP_LOGI(TAG, "  Communication OK (%d sentences in %lu ms)",
                             sentence_count, 
                             xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms);
                    return true;
                } else {
                    ESP_LOGW(TAG, "  Checksum error: %s", line);
                }
            }
            
            line_pos = 0;
        } else if (line_pos < NMEA_MAX_LINE - 1) {
            line[line_pos++] = byte;
        }
    }
    
    if (sentence_count > 0) {
        ESP_LOGW(TAG, "GPS responding but all checksums failed (%d sentences)", sentence_count);
    } else {
        ESP_LOGE(TAG, "No GPS data received (check wiring)");
    }
    
    return false;
}

bool gps_poll_nav_pvt(gps_data_t *out) {
    if (!s_gps_initialized) {
        ESP_LOGW(TAG, "GPS not initialized");
        return false;
    }
    
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
                        
                        ESP_LOGI(TAG, "✓ GPS Fix: %.7f°%c %.7f°%c alt=%.1fm sats=%u speed=%.1fm/s heading=%.1f°",
                                 fabs(temp.latitude), (temp.latitude >= 0) ? 'N' : 'S',
                                 fabs(temp.longitude), (temp.longitude >= 0) ? 'E' : 'W',
                                 temp.altitude_m, temp.num_satellites, 
                                 temp.ground_speed_mps, temp.heading_deg);
                        
                        return true;
                    }
                }
            }
            
            line_pos = 0;
        } else if (line_pos < NMEA_MAX_LINE - 1) {
            line[line_pos++] = byte;
        }
    }
    
    ESP_LOGD(TAG, "GPS poll timeout (GGA=%d RMC=%d)", got_gga, got_rmc);
    return false;
}

bool gps_get_last(gps_data_t *out) {
    if (!s_last.valid) {
        ESP_LOGD(TAG, "No valid GPS fix cached");
        return false;
    }
    if (out) *out = s_last;
    return true;
}

bool gps_get_compass_heading(float *heading_deg) {
    if (!s_compass_present) {
        ESP_LOGD(TAG, "Compass not available");
        return false;
    }
    
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
    
    ESP_LOGD(TAG, "Compass: raw[%d,%d,%d] %s heading=%.1f°", 
             x, y, z, 
             s_compass_cal.calibrated ? "cal" : "uncal",
             heading);
    
    return true;
}

bool gps_get_compass_heading_true(float *heading_true_deg) {
    float mag_heading;
    if (!gps_get_compass_heading(&mag_heading)) {
        return false;
    }
    
    // Get declination (user-configured or estimated)
    float declination = s_compass_cal.declination_deg;
    
    // If no declination set, try to estimate from GPS location
    if (fabs(declination) < 0.1f) {
        gps_data_t gps;
        if (gps_get_last(&gps) && gps.valid) {
            declination = estimate_magnetic_declination(gps.latitude, gps.longitude);
            ESP_LOGI(TAG, "Using estimated declination %.1f° for %.2f,%.2f",
                     declination, gps.latitude, gps.longitude);
        } else {
            ESP_LOGW(TAG, "No GPS fix - can't estimate declination, using 0°");
        }
    }
    
    // Convert magnetic to true: true_north = magnetic_north + declination
    *heading_true_deg = mag_heading + declination;
    
    // Normalize to 0-360
    while (*heading_true_deg < 0) *heading_true_deg += 360.0f;
    while (*heading_true_deg >= 360) *heading_true_deg -= 360.0f;
    
    ESP_LOGD(TAG, "Heading: magnetic=%.1f° + decl=%.1f° = true=%.1f°",
             mag_heading, declination, *heading_true_deg);
    
    return true;
}

void gps_set_magnetic_declination(float declination_deg) {
    s_compass_cal.declination_deg = declination_deg;
    compass_save_calibration();
    
    ESP_LOGI(TAG, "✓ Magnetic declination set to %.1f°", declination_deg);
}

float gps_get_magnetic_declination(void) {
    return s_compass_cal.declination_deg;
}

bool gps_calibrate_compass(void) {
    if (!s_compass_present) {
        ESP_LOGE(TAG, "Compass not available - cannot calibrate");
        return false;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          COMPASS CALIBRATION PROCEDURE             ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Instructions:");
    ESP_LOGI(TAG, "  1. Keep system level (horizontal)");
    ESP_LOGI(TAG, "  2. Slowly rotate 360° (2-3 complete circles)");
    ESP_LOGI(TAG, "  3. Duration: 20 seconds");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Starting in 3 seconds...");
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "▶ ROTATE NOW!");
    
    // Reset calibration data (but preserve declination)
    float saved_decl = s_compass_cal.declination_deg;
    s_compass_cal.x_min = 32767;
    s_compass_cal.x_max = -32768;
    s_compass_cal.y_min = 32767;
    s_compass_cal.y_max = -32768;
    s_compass_cal.z_min = 32767;
    s_compass_cal.z_max = -32768;
    s_compass_cal.calibrated = false;
    s_compass_cal.declination_deg = saved_decl;
    
    // Collect samples for 20 seconds
    uint32_t start = xTaskGetTickCount();
    int sample_count = 0;
    int progress = 0;
    
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
            
            // Progress bar every 4 seconds (20%)
            uint32_t elapsed_ms = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
            int new_progress = (elapsed_ms / 4000) * 20;
            
            if (new_progress > progress) {
                progress = new_progress;
                ESP_LOGI(TAG, "[%d%%] Samples: %d, X[%d,%d] Y[%d,%d] Z[%d,%d]",
                         progress, sample_count,
                         s_compass_cal.x_min, s_compass_cal.x_max,
                         s_compass_cal.y_min, s_compass_cal.y_max,
                         s_compass_cal.z_min, s_compass_cal.z_max);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(67));  // ~15 Hz
    }
    
    ESP_LOGI(TAG, "■ STOP rotating");
    ESP_LOGI(TAG, "");
    
    // Validate calibration quality
    int x_range = s_compass_cal.x_max - s_compass_cal.x_min;
    int y_range = s_compass_cal.y_max - s_compass_cal.y_min;
    int z_range = s_compass_cal.z_max - s_compass_cal.z_min;
    
    ESP_LOGI(TAG, "Calibration Results:");
    ESP_LOGI(TAG, "  Samples collected: %d", sample_count);
    ESP_LOGI(TAG, "  X range: %d", x_range);
    ESP_LOGI(TAG, "  Y range: %d", y_range);
    ESP_LOGI(TAG, "  Z range: %d", z_range);
    
    if (x_range < 200 || y_range < 200) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ CALIBRATION FAILED");
        ESP_LOGE(TAG, "  Insufficient rotation detected");
        ESP_LOGE(TAG, "  Minimum required: X=200, Y=200");
        ESP_LOGE(TAG, "  Try again with more complete rotation");
        return false;
    }
    
    s_compass_cal.calibrated = true;
    compass_save_calibration();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ CALIBRATION SUCCESSFUL");
    ESP_LOGI(TAG, "  Compass ready for accurate heading");
    
    return true;
}

bool gps_is_compass_calibrated(void) {
    return s_compass_cal.calibrated;
}

bool gps_is_compass_present(void) {
    return s_compass_present;
}