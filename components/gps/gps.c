#include "gps.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Implementation notes                                                  │
    │ - UBX transport: [B5 62] [CLS] [ID] [LEN_L LEN_H] [PAYLOAD...] CK_A CK_B
    │ - Checksum is computed over CLS, ID, LEN, and PAYLOAD (not sync bytes).
    │ - We poll NAV‑PVT (CLS=0x01, ID=0x07). Minimum payload length = 92 bytes.
    │ - For simplicity we read a fixed buffer (128 bytes) and parse if present.
    │ - If you want continuous streaming later, switch to a ring buffer + ISR. │
    └───────────────────────────────────────────────────────────────────────┘
*/

#define TAG "GPS"

// Set to 1 to print hex dumps of I2C reads (can be noisy at 1 Hz).
#ifndef GPS_LOG_HEXDUMP
#define GPS_LOG_HEXDUMP 0
#endif

#define USE_HARDCODED_LOCATION 1  // Set to 0 to use real GPS

#if USE_HARDCODED_LOCATION
// Auburn, Alabama coordinates
#define HARDCODED_LAT  32.5990
#define HARDCODED_LON  -85.4808
#define HARDCODED_ALT  200.0  // meters above sea level
#endif

static gps_cfg_t  s_cfg;
static gps_data_t s_last = {0};

/* ────────────────────────── UBX helpers ──────────────────────────────── */

static void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b){
    uint8_t A = 0, B = 0;
    for (size_t i = 0; i < len; ++i){ A += data[i]; B += A; }
    if (ck_a) *ck_a = A;
    if (ck_b) *ck_b = B;
}

/*
    Send a UBX frame (header + payload + checksum) over I2C.
    - cls/id: UBX message class and id.
    - payload/len: may be NULL/0 for empty payload messages (e.g., poll).
*/
static esp_err_t ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck_a=0, ck_b=0;
    ubx_checksum(&hdr[2], 4, &ck_a, &ck_b);  // class..len
    for (int i = 0; i < len; i++){ ck_a += payload ? payload[i] : 0; ck_b += ck_a; }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_cfg.addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, hdr, sizeof(hdr), true);
    if (len && payload) i2c_master_write(cmd, payload, len, true);
    uint8_t cks[2] = {ck_a, ck_b};
    i2c_master_write(cmd, cks, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_cfg.i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGD(TAG, "UBX send: cls=0x%02X id=0x%02X len=%u -> %s", cls, id, (unsigned)len,
             (ret == ESP_OK ? "OK" : "FAIL"));
    return ret;
}

/*
    Parse a UBX NAV‑PVT buffer and fill 'g'.

    Expectations:
    - buf starts with sync and at least a complete NAV‑PVT frame (we read fixed 128 B).
    - We verify length and checksum before extracting fields we care about.

    Returns true on success, false on any check failing.
*/
static bool parse_nav_pvt(const uint8_t *buf, size_t n, gps_data_t *g) {
    // 1) Basic header checks
    if (n < 6 + 92 + 2) { ESP_LOGD(TAG, "NAV-PVT: buffer too short (%u)", (unsigned)n); return false; }
    if (buf[0] != 0xB5 || buf[1] != 0x62 || buf[2] != 0x01 || buf[3] != 0x07) {
        ESP_LOGD(TAG, "NAV-PVT: sync/cls/id mismatch");
        return false;
    }

    // 2) Length and checksum validation
    uint16_t len = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    if ((size_t)(len + 6 + 2) > n) {
        ESP_LOGD(TAG, "NAV-PVT: declared len=%u exceeds buffer", (unsigned)len);
        return false;
    }

    const uint8_t *payload = &buf[6];
    uint8_t ck_a=0, ck_b=0;
    ubx_checksum(&buf[2], 4 + len, &ck_a, &ck_b);
    uint8_t rx_a = buf[6 + len];
    uint8_t rx_b = buf[6 + len + 1];
    if (ck_a != rx_a || ck_b != rx_b) {
        ESP_LOGW(TAG, "NAV-PVT: checksum fail (calc %02X %02X, rx %02X %02X)", ck_a, ck_b, rx_a, rx_b);
        return false;
    }

    // 3) Extract fields (u‑blox M10 NAV‑PVT layout)
    // Offsets below are within the payload region.
    // [20] fixType, [23] numSV, [24] lon(1e-7 deg), [28] lat(1e-7 deg),
    // [36] hMSL(mm), [60] gSpeed(mm/s), [64] headMot(1e-5 deg)
    uint8_t  fixType = payload[20];
    uint8_t  numSV   = payload[23];
    int32_t  lon1e7  = *(int32_t*)&payload[24];
    int32_t  lat1e7  = *(int32_t*)&payload[28];
    int32_t  hMSLmm  = *(int32_t*)&payload[36];
    int32_t  gSpeed  = *(int32_t*)&payload[60]; // mm/s
    int32_t  headDeg = *(int32_t*)&payload[64]; // 1e-5 deg

    g->fix_type          = fixType;
    g->num_satellites    = numSV;

    if (fixType < 2) {
        g->valid = false;
        ESP_LOGD(TAG, "NAV-PVT: no 2D/3D fix yet (fixType=%u, SV=%u)", fixType, numSV);
        return false;
    }

    g->longitude         = lon1e7 / 1e7;
    g->latitude          = lat1e7 / 1e7;
    g->altitude_m        = hMSLmm / 1000.0;
    g->ground_speed_mps  = gSpeed / 1000.0f;
    g->heading_deg       = headDeg / 1e5f;
    g->timestamp         = time(NULL);
    g->valid             = true;

    ESP_LOGD(TAG, "NAV-PVT OK: fix=%u SV=%u lat=%.7f lon=%.7f alt=%.1f v=%.2f head=%.1f",
             g->fix_type, g->num_satellites, g->latitude, g->longitude,
             g->altitude_m, g->ground_speed_mps, g->heading_deg);
    return true;
}

/* ────────────────────────── Public API ──────────────────────────────── */

esp_err_t gps_init(const gps_cfg_t *cfg) {
    s_cfg = *cfg;

#if USE_HARDCODED_LOCATION
    ESP_LOGI(TAG, "GPS module initialized in HARDCODED mode (Auburn, AL)");
    ESP_LOGI(TAG, "Location: %.4f°N, %.4f°W", HARDCODED_LAT, -HARDCODED_LON);
    return ESP_OK;
#else
    // Optional: bump this module to DEBUG without changing global verbosity.
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = s_cfg.sda_io,
        .scl_io_num = s_cfg.scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = s_cfg.clk_hz
    };
    ESP_ERROR_CHECK(i2c_param_config(s_cfg.i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(s_cfg.i2c_port, conf.mode, 0, 0, 0));

    // Configure measurement/navigation rate: 1 Hz
    // UBX-CFG-RATE payload: measRate [ms], navRate [cycles], timeRef [0=UTC,1=GPS]
    uint8_t cfg_rate[] = { 0xE8,0x03,  0x01,0x00,  0x01,0x00 }; // 1000ms, 1, GPS
    esp_err_t r = ubx_send(0x06, 0x08, cfg_rate, sizeof(cfg_rate));
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "CFG-RATE send failed: %s", esp_err_to_name(r));
    }

    ESP_LOGI(TAG, "GPS init: I2C%d SDA=%d SCL=%d @%lu Hz, addr=0x%02X",
             s_cfg.i2c_port, s_cfg.sda_io, s_cfg.scl_io, (unsigned long)s_cfg.clk_hz, s_cfg.addr);
    return ESP_OK;
#endif
}

bool gps_poll_nav_pvt(gps_data_t *out) {
    // Poll request: UBX-NAV-PVT (empty payload)
    if (ubx_send(0x01, 0x07, NULL, 0) != ESP_OK) {
        ESP_LOGD(TAG, "NAV-PVT poll send failed");
        return false;
    }

    // Allow device time to prepare the response on the I2C FIFO
    vTaskDelay(pdMS_TO_TICKS(200));

    // Read a fixed-size buffer; parse_nav_pvt validates size/cksum/content
    uint8_t buf[128] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_cfg.addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, sizeof(buf), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_cfg.i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return false;
    }

#if GPS_LOG_HEXDUMP
    ESP_LOG_BUFFER_HEXDUMP(TAG, buf, sizeof(buf), ESP_LOG_DEBUG);
#endif

    gps_data_t g = {0};
    if (!parse_nav_pvt(buf, sizeof(buf), &g)) return false;

    s_last = g;
    if (out) *out = g;

    // Keep this one at INFO so we see healthy fixes at runtime
    ESP_LOGI(TAG, "Fix=%u SV=%u Lat=%.7f Lon=%.7f Alt=%.1fm V=%.2fm/s Head=%.1f",
             g.fix_type, g.num_satellites, g.latitude, g.longitude,
             g.altitude_m, g.ground_speed_mps, g.heading_deg);
    return true;
}

bool gps_get_last(gps_data_t *out) {
    if (!s_last.valid) return false;
    if (out) *out = s_last;
    return true;
}

bool gps_get_fix(gps_fix_t *fix, uint32_t timeout_ms) {
    if (!fix) return false;

#if USE_HARDCODED_LOCATION
    // Use hardcoded Auburn, AL location
    fix->latitude = HARDCODED_LAT;
    fix->longitude = HARDCODED_LON;
    fix->altitude = HARDCODED_ALT;
    fix->num_satellites = 12;  // Simulate good GPS fix
    fix->hdop = 0.8;           // Good horizontal dilution of precision
    
    // Get current system time (should be set via SNTP or manual)
    time_t now;
    time(&now);
    fix->time = now;
    
    ESP_LOGI(TAG, "Using hardcoded location: Auburn, AL (%.4f, %.4f)", 
             HARDCODED_LAT, HARDCODED_LON);
    
    return true;
#else
    // ...existing GPS acquisition code...
#endif
}