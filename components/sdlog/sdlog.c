#include "sdlog.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include <inttypes.h>

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ SD Logging Implementation Notes                                       │
    │                                                                       │
    │ Key design decisions and why I made them:                            │
    │                                                                       │
    │ 1) Open/write/close on every operation:                              │
    │    - Pro: Data is immediately committed to flash, survives crashes   │
    │    - Pro: No file handle management or cleanup needed                │
    │    - Con: Slower than keeping files open                             │
    │    - Con: More wear on SD card filesystem structures                 │
    │                                                                       │
    │ 2) Silent error handling:                                            │
    │    - Pro: Logging failures don't crash the main application          │
    │    - Pro: System continues working even if SD card fails             │
    │    - Con: You might not notice when logging stops working            │
    │    - Mitigation: Check SD card LED for activity, or add NVS backup   │
    │                                                                       │
    │ 3) No internal buffering:                                            │
    │    - Pro: Simple code, no memory management                          │
    │    - Pro: Every write goes to disk immediately                       │
    │    - Con: Can't optimize multiple small writes                       │
    │    - Future: Could add optional RAM buffer for high-freq logging     │
    │                                                                       │
    │ Common debugging scenarios:                                          │
    │ - SD mount fails: Check wiring, card format, power supply            │
    │ - Files appear empty: Check file system permissions                  │
    │ - Intermittent write failures: Usually power supply brownouts        │
    │ - Corrupted files: Hard power-off during writes, bad SD card         │
    │ - Performance issues: Use faster SD card, reduce write frequency     │
    └───────────────────────────────────────────────────────────────────────┘
*/

#define TAG "SDLOG"

// Module state (keep it simple)
static bool s_mounted = false;
static sdmmc_card_t *s_card = NULL;  // Keep reference for diagnostics

/*
    Format a timestamp string for human-readable log entries.
    Returns pointer to static buffer (not thread-safe, but we're single-threaded).
*/
static const char* get_timestamp_str(void) {
    static char buf[64];  // Increased from 32 to 64 to fix truncation warning
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    return buf;
}

/*
    Helper: write formatted text to an open file with newline and flush.
    This is used by both sdlog_printf and sdlog_write_csv.
*/
static void vwritef(FILE *f, const char *fmt, va_list ap){
    vfprintf(f, fmt, ap);
    fputc('\n', f);
    fflush(f);  // Force immediate write to SD card (critical for crash safety)
}

/* ─────────────────────────── Public API ─────────────────────────────── */

bool sdlog_init(const sdlog_cfg_t *cfg){
    // Prevent double initialization
    if (s_mounted) {
        ESP_LOGW(TAG, "SD already mounted, skipping init");
        return true;
    }

    ESP_LOGI(TAG, "Initializing SD card logging...");
    ESP_LOGI(TAG, "  SPI pins: MOSI=%d MISO=%d SCLK=%d CS=%d", 
             cfg->mosi, cfg->miso, cfg->sclk, cfg->cs);

    // Pin 2 and 15 are boot strap pins on ESP32
    if (cfg->miso == 2) {
        ESP_LOGW(TAG, "MISO on GPIO2 (boot strap pin) - ensure SD card removed during flashing");
    }
    if (cfg->cs == 15) {
        ESP_LOGW(TAG, "CS on GPIO15 (boot strap pin) - may affect boot mode if card inserted");
    }

    // Configure SPI bus for SD card communication
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    ESP_LOGD(TAG, "Using SDSPI host: slot=%d, max_freq=%d Hz", host.slot, host.max_freq_khz * 1000);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,          // Not used in SPI mode
        .quadhd_io_num   = -1,          // Not used in SPI mode
        .max_transfer_sz = 4000         // 4KB max transfer (good balance)
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGD(TAG, "SPI bus initialized successfully");

    // Configure SD card device on the SPI bus
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cfg->cs;
    slot_config.host_id = host.slot;
    ESP_LOGD(TAG, "SD device config: CS=%d, host_id=%d", slot_config.gpio_cs, slot_config.host_id);

    // Mount FAT filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,    // Don't auto-format (preserve user data)
        .max_files = 5,                     // Enough for log + csv + temp files
        .allocation_unit_size = 16 * 1024   // 16KB clusters (good for large files)
    };

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        
        // Provide specific error guidance
        switch (ret) {
            case ESP_ERR_NOT_FOUND:
                ESP_LOGE(TAG, "  → No SD card detected (check card insertion)");
                break;
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE(TAG, "  → Card initialization failed (check wiring/power)");
                break;
            case ESP_FAIL:
                ESP_LOGE(TAG, "  → Mount failed (card may need formatting)");
                break;
            default:
                ESP_LOGE(TAG, "  → Unknown error code: 0x%x", ret);
                break;
        }
        return false;
    }

    s_mounted = true;

    // Log detailed card information for diagnostics
    ESP_LOGI(TAG, "SD card mounted successfully:");
    ESP_LOGI(TAG, "  Name: %s", s_card->cid.name);
    ESP_LOGI(TAG, "  Capacity: %lluMB", ((uint64_t) s_card->csd.capacity) * s_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "  Speed: %s", (s_card->csd.tr_speed > 25000000) ? "High Speed" : "Standard");
    ESP_LOGI(TAG, "  Manufacturer: %s (%02X)", 
             s_card->cid.mfg_id == 0x03 ? "SanDisk" : "Unknown", s_card->cid.mfg_id);

    // Create initial log entry to verify logging works
    sdlog_printf("=== SD LOG SYSTEM INITIALIZED ===");
    sdlog_printf("Card: %s, Size: %llu MB, MFG: 0x%02X", 
                 s_card->cid.name, ((uint64_t)s_card->csd.capacity) / (1024 * 1024), s_card->cid.mfg_id);

    ESP_LOGI(TAG, "SD logging system ready");
    return true;
}

void sdlog_printf(const char *fmt, ...){
    // Silently fail if SD not available (don't crash the main application)
    if (!s_mounted) {
        ESP_LOGD(TAG, "sdlog_printf called but SD not mounted");
        return;
    }

    // Open log file in append mode
    FILE *f = fopen("/sdcard/sunflower.log", "a");
    if (!f) {
        ESP_LOGD(TAG, "Failed to open sunflower.log for writing");
        return;
    }

    // Write timestamped log entry
    fprintf(f, "[%s] ", get_timestamp_str());
    va_list ap; 
    va_start(ap, fmt);
    vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);

    // Also echo to ESP_LOG for console debugging (at debug level to avoid spam)
    va_list ap2; 
    va_start(ap2, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);
    ESP_LOGD(TAG, "LOG: %s", buf);
}

void sdlog_write_csv_header_if_new(const char *path){
    if (!s_mounted) {
        ESP_LOGD(TAG, "write_csv_header called but SD not mounted");
        return;
    }

    // Check if file already exists
    FILE *f = fopen(path, "r");
    if (f) { 
        fclose(f); 
        ESP_LOGD(TAG, "CSV file %s already exists, skipping header", path);
        return; 
    }

    // File doesn't exist, create it with header
    f = fopen(path, "w");
    if (!f) {
        ESP_LOGD(TAG, "Failed to create CSV file %s", path);
        return;
    }

    // Write CSV header row (must match the format used by tracking system)
    fprintf(f, "unix_ts,lat,lon,fix,sats,az_target,el_target,az_cur,el_cur,moves_today,total_moves,batt_v,notes\n");
    fclose(f);

    ESP_LOGI(TAG, "Created CSV file with header: %s", path);
    sdlog_printf("Created CSV file: %s", path);  // Log this event too
}

void sdlog_write_csv(const char *path, const char *fmt, ...){
    if (!s_mounted) {
        ESP_LOGD(TAG, "write_csv called but SD not mounted");
        return;
    }

    // Open CSV file in append mode
    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGD(TAG, "Failed to open CSV file %s for writing", path);
        return;
    }

    // Write the CSV row
    va_list ap; 
    va_start(ap, fmt);
    vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);

    // Optional: log CSV writes at verbose level (can be noisy)
    ESP_LOGV(TAG, "CSV write to %s", path);
}

