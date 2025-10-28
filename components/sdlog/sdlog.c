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
    SD Logging Module

    Summary:
    - Logs text lines and CSV rows to a microSD card.
    - Card is accessed via the onboard microSD reader on the dev board (SPI/SDSPI).
    - Files are opened, written, flushed, and closed on each call (crash-safe).

    Behavior:
    - sdlog_init mounts /sdcard using esp_vfs_fat_sdspi_mount.
    - sdlog_printf appends timestamped lines to /sdcard/Sunflower.log.
    - sdlog_write_csv_header_if_new creates a CSV file with a single header row.
    - sdlog_write_csv appends single CSV rows.
    - Logging is non-fatal: if SD is not mounted or a write fails, functions return quietly.
    - A static timestamp buffer is used; not thread-safe (intended for single-task calls).

    Notes:
    - Immediate fflush() after each write to minimize data loss on power loss.
    - GPIO2 and GPIO15 are ESP32 boot strap pins; see warnings during init if used.
*/

#define TAG "SDLOG"

// Module state
static bool s_mounted = false;
static sdmmc_card_t *s_card = NULL;  // Keep reference for diagnostics

/*
    Human-readable timestamp for log lines.
    Returns a pointer to a static buffer (not thread-safe).
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
    Common helper: write formatted text to an open FILE, append newline, and flush.
    Used by both sdlog_printf and sdlog_write_csv.
*/
static void vwritef(FILE *f, const char *fmt, va_list ap){
    vfprintf(f, fmt, ap);
    fputc('\n', f);
    fflush(f);  // Force immediate write to SD card (improves crash resilience)
}

/* ─────────────────────────── Public API ─────────────────────────────── */

bool sdlog_init(const sdlog_cfg_t *cfg){
    // Prevent double initialization
    if (s_mounted) {
        ESP_LOGW(TAG, "SD already mounted, skipping init");
        return true;
    }

    ESP_LOGI(TAG, "Initializing SD card logging (onboard microSD reader)...");
    ESP_LOGI(TAG, "  SPI pins: MOSI=%d MISO=%d SCLK=%d CS=%d", 
             cfg->mosi, cfg->miso, cfg->sclk, cfg->cs);

    // Pin 2 and 15 are boot strap pins on ESP32
    if (cfg->miso == 2) {
        ESP_LOGW(TAG, "MISO on GPIO2 (boot strap pin) - ensure SD card removed during flashing");
    }
    if (cfg->cs == 15) {
        ESP_LOGW(TAG, "CS on GPIO15 (boot strap pin) - may affect boot mode if card inserted");
    }

    // Configure SPI bus (SDSPI host) for the onboard microSD slot
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    ESP_LOGD(TAG, "Using SDSPI host: slot=%d, max_freq=%d Hz", host.slot, host.max_freq_khz * 1000);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,          // Not used in SPI mode
        .quadhd_io_num   = -1,          // Not used in SPI mode
        .max_transfer_sz = 4000         // 4KB max transfer
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGD(TAG, "SPI bus initialized");

    // Configure SD card device on the SPI bus
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cfg->cs;
    slot_config.host_id = host.slot;
    ESP_LOGD(TAG, "SD device config: CS=%d, host_id=%d", slot_config.gpio_cs, slot_config.host_id);

    // Mount FAT filesystem at /sdcard
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,    // Do not auto-format
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        switch (ret) {
            case ESP_ERR_NOT_FOUND:
                ESP_LOGE(TAG, "  → No SD card detected (check card insertion)");
                break;
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE(TAG, "  → Card initialization failed (check wiring/power)");
                break;
            case ESP_FAIL:
                ESP_LOGE(TAG, "  → Mount failed (filesystem issue)");
                break;
            default:
                ESP_LOGE(TAG, "  → Error code: 0x%x", ret);
                break;
        }
        return false;
    }

    s_mounted = true;

    // Card information (useful when users report issues)
    ESP_LOGI(TAG, "SD card mounted:");
    ESP_LOGI(TAG, "  Name: %s", s_card->cid.name);
    ESP_LOGI(TAG, "  Capacity: %lluMB", ((uint64_t) s_card->csd.capacity) * s_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "  Speed: %s", (s_card->csd.tr_speed > 25000000) ? "High Speed" : "Standard");
    ESP_LOGI(TAG, "  Manufacturer ID: 0x%02X", s_card->cid.mfg_id);

    // Test log entries to confirm write path
    sdlog_printf("=== SD LOG SYSTEM INITIALIZED ===");
    sdlog_printf("Card: %s, Size: %llu MB, MFG: 0x%02X", 
                 s_card->cid.name, ((uint64_t)s_card->csd.capacity) / (1024 * 1024), s_card->cid.mfg_id);

    ESP_LOGI(TAG, "SD logging ready");
    return true;
}

/*
    Append a timestamped line to /sdcard/Sunflower.log.
    Safe to call periodically; returns quietly if SD is not mounted.
*/
void sdlog_printf(const char *fmt, ...){
    if (!s_mounted) {
        ESP_LOGD(TAG, "sdlog_printf called but SD not mounted");
        return;
    }

    FILE *f = fopen("/sdcard/Sunflower.log", "a");
    if (!f) {
        ESP_LOGD(TAG, "Failed to open Sunflower.log for writing");
        return;
    }

    fprintf(f, "[%s] ", get_timestamp_str());
    va_list ap; 
    va_start(ap, fmt);
    vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);

    // Echo to console at debug level
    va_list ap2; 
    va_start(ap2, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);
    ESP_LOGD(TAG, "LOG: %s", buf);
}

/*
    Create CSV file with header if it does not exist.
    Header matches the format used by the tracking module.
*/
void sdlog_write_csv_header_if_new(const char *path){
    if (!s_mounted) {
        ESP_LOGD(TAG, "write_csv_header called but SD not mounted");
        return;
    }

    FILE *f = fopen(path, "r");
    if (f) { 
        fclose(f); 
        ESP_LOGD(TAG, "CSV file %s already exists, skipping header", path);
        return; 
    }

    f = fopen(path, "w");
    if (!f) {
        ESP_LOGD(TAG, "Failed to create CSV file %s", path);
        return;
    }

    fprintf(f, "unix_ts,lat,lon,fix,sats,az_target,el_target,az_cur,el_cur,moves_today,total_moves,batt_v,notes\n");
    fclose(f);

    ESP_LOGI(TAG, "Created CSV file with header: %s", path);
    sdlog_printf("Created CSV file: %s", path);
}

/*
    Append a single CSV row to the file at 'path'.
    The row format must match the header written above.
*/
void sdlog_write_csv(const char *path, const char *fmt, ...){
    if (!s_mounted) {
        ESP_LOGD(TAG, "write_csv called but SD not mounted");
        return;
    }

    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGD(TAG, "Failed to open CSV file %s for writing", path);
        return;
    }

    va_list ap; 
    va_start(ap, fmt);
    vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);

    ESP_LOGV(TAG, "CSV write to %s", path);
}

