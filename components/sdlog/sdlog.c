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
    - Provides real-time health monitoring for display integration.

    Behavior:
    - sdlog_init mounts /sdcard using esp_vfs_fat_sdspi_mount.
    - sdlog_printf appends timestamped lines to /sdcard/Sunflower.log.
    - sdlog_write_csv_header_if_new creates a CSV file with a single header row.
    - sdlog_write_csv appends single CSV rows matching tracker_data_t structure.
    - Logging is non-fatal: if SD is not mounted or a write fails, functions return quietly.
    - A static timestamp buffer is used; not thread-safe (intended for single-task calls).

    Health Monitoring:
    - Tracks write times: <100ms=OK, 100-200ms=warn, >200ms=SLOW
    - Monitors disk space: >90% used = FULL status
    - Detects mount/write failures: FAILED status
    - Auto-recovery: SLOW→OK when performance improves

    CSV Format (matches tracker_data_t):
    timestamp,latitude,longitude,gps_valid,gps_sats,gps_fix_age_sec,
    sun_elevation,sun_azimuth,panel_elevation,panel_azimuth,
    delta_elevation,delta_azimuth,tracking_quality,
    battery_voltage,battery_soc_percent,battery_soc_level,battery_charging,battery_adc,
    moves_today,total_moves,uptime_hours,
    status,wifi_clients,wifi_rssi,sd_status,
    sunrise_time,sunset_time,notes

    Notes:
    - Immediate fflush() after each write to minimize data loss on power loss.
    - GPIO2 and GPIO15 are ESP32 boot strap pins; see warnings during init if used.
    - Write times logged at WARN level if >200ms, ERROR if >500ms.
*/

#define TAG "SDLOG"

// Performance thresholds (milliseconds)
#define WRITE_TIME_OK        100    // <100ms = excellent
#define WRITE_TIME_WARN      200    // 100-200ms = acceptable
#define WRITE_TIME_SLOW      500    // >200ms = slow (set SLOW status)
#define WRITE_TIME_CRITICAL  500    // >500ms = critical (possible failure)

// Disk space thresholds
#define DISK_FULL_PERCENT    90     // >90% used = FULL status

// Module state
static bool s_mounted = false;
static sdmmc_card_t *s_card = NULL;  // Keep reference for diagnostics

// Health monitoring
static uint8_t s_sd_status = SDLOG_STATUS_OK;
static uint32_t s_last_write_time_ms = 0;
static uint32_t s_total_writes = 0;
static uint32_t s_slow_writes = 0;
static uint32_t s_failed_writes = 0;

// Disk space tracking
static uint64_t s_total_space_mb = 0;
static uint64_t s_free_space_mb = 0;
static uint32_t s_last_space_check = 0;

/* ─────────────────────── Internal Helper Functions ──────────────────── */

/*
    Human-readable timestamp for log lines.
    Returns a pointer to a static buffer (not thread-safe).
*/
static const char* get_timestamp_str(void) {
    static char buf[64];
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    return buf;
}

/*
    Update disk space info (called periodically, max once per 60 seconds).
*/
static void update_disk_space(void) {
    // Don't check more than once per minute
    uint32_t now_sec = time(NULL);
    if ((now_sec - s_last_space_check) < 60) {
        return;
    }
    s_last_space_check = now_sec;
    
    // Use FATFS API to get disk space (ESP-IDF compatible)
    FATFS *fs;
    DWORD fre_clust;
    
    // Get volume information - use /sdcard path
    FRESULT res = f_getfree("/sdcard", &fre_clust, &fs);
    if (res != FR_OK) {
        ESP_LOGW(TAG, "Failed to get disk space info (FATFS error %d)", res);
        // Don't update status - keep using card info capacity
        return;
    }
    
    // Calculate total and free space
    // Total clusters = n_fatent - 2 (exclude reserved clusters)
    // Each cluster = csize sectors
    uint64_t total_clusters = (fs->n_fatent - 2);
    uint64_t free_clusters = fre_clust;
    
    // Sectors per cluster
    uint32_t sectors_per_cluster = fs->csize;
    
    // Get sector size
#if FF_MAX_SS != FF_MIN_SS
    uint32_t sector_size = fs->ssize;
#else
    uint32_t sector_size = FF_MAX_SS;
#endif
    
    // Calculate space in bytes
    uint64_t total_bytes = total_clusters * sectors_per_cluster * sector_size;
    uint64_t free_bytes = free_clusters * sectors_per_cluster * sector_size;
    
    s_total_space_mb = total_bytes / (1024 * 1024);
    s_free_space_mb = free_bytes / (1024 * 1024);
    
    // Calculate percentage used
    if (s_total_space_mb > 0) {
        uint64_t used_mb = s_total_space_mb - s_free_space_mb;
        uint32_t percent_used = (uint32_t)((used_mb * 100) / s_total_space_mb);
        
        ESP_LOGD(TAG, "Disk: %llu MB total, %llu MB free (%lu%% used)",
                 s_total_space_mb, s_free_space_mb, (unsigned long)percent_used);
        
        // Update status if nearly full
        if (percent_used >= DISK_FULL_PERCENT) {
            if (s_sd_status == SDLOG_STATUS_OK || s_sd_status == SDLOG_STATUS_SLOW) {
                ESP_LOGW(TAG, "SD card nearly full: %lu%% used", (unsigned long)percent_used);
                s_sd_status = SDLOG_STATUS_FULL;
            }
        } else if (s_sd_status == SDLOG_STATUS_FULL) {
            // Recovered from FULL (files deleted?)
            ESP_LOGI(TAG, "SD card space recovered: %lu%% used", (unsigned long)percent_used);
            s_sd_status = SDLOG_STATUS_OK;
        }
    }
}

/*
    Common helper: write formatted text to an open FILE, append newline, and flush.
    Used by both sdlog_printf and sdlog_write_csv.
    Returns true on success, false on failure.
*/
static bool vwritef(FILE *f, const char *fmt, va_list ap){
    int ret = vfprintf(f, fmt, ap);
    if (ret < 0) {
        ESP_LOGD(TAG, "vfprintf failed");
        return false;
    }
    
    if (fputc('\n', f) == EOF) {
        ESP_LOGD(TAG, "fputc failed");
        return false;
    }
    
    if (fflush(f) != 0) {
        ESP_LOGD(TAG, "fflush failed");
        return false;
    }
    
    return true;
}

/* ─────────────────────────── Public API ─────────────────────────────── */

bool sdlog_init(const sdlog_cfg_t *cfg){
    // Prevent double initialization
    if (s_mounted) {
        ESP_LOGW(TAG, "SD already mounted, skipping init");
        return true;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          SD CARD INITIALIZATION                            ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "SPI Configuration:");
    ESP_LOGI(TAG, "  MOSI: GPIO%d", cfg->mosi);
    ESP_LOGI(TAG, "  MISO: GPIO%d", cfg->miso);
    ESP_LOGI(TAG, "  SCLK: GPIO%d", cfg->sclk);
    ESP_LOGI(TAG, "  CS:   GPIO%d", cfg->cs);

    // Pin 2 and 15 are boot strap pins on ESP32
    if (cfg->miso == 2) {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "⚠ MISO on GPIO2 (boot strap pin)");
        ESP_LOGW(TAG, "  - Remove SD card when flashing firmware");
        ESP_LOGW(TAG, "  - Card present during boot may prevent startup");
    }
    if (cfg->cs == 15) {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "⚠ CS on GPIO15 (boot strap pin)");
        ESP_LOGW(TAG, "  - May affect boot mode if card inserted");
        ESP_LOGW(TAG, "  - Test with card removed if boot issues occur");
    }
    ESP_LOGI(TAG, "");

    // Configure SPI bus (SDSPI host) for the onboard microSD slot
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    ESP_LOGD(TAG, "SDSPI host: slot=%d, max_freq=%d Hz", host.slot, host.max_freq_khz * 1000);

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
        ESP_LOGE(TAG, "✗ SPI bus initialization failed: %s", esp_err_to_name(ret));
        s_sd_status = SDLOG_STATUS_FAILED;
        return false;
    }
    ESP_LOGI(TAG, "✓ SPI bus initialized");

    // Configure SD card device on the SPI bus
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cfg->cs;
    slot_config.host_id = host.slot;
    ESP_LOGD(TAG, "SD device: CS=%d, host=%d", slot_config.gpio_cs, slot_config.host_id);

    // Mount FAT filesystem at /sdcard
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,    // Do not auto-format
        .max_files = 5,                     // Allow up to 5 open files
        .allocation_unit_size = 16 * 1024   // 16KB clusters
    };

    ESP_LOGI(TAG, "Mounting filesystem...");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ SD card mount failed: %s", esp_err_to_name(ret));
        
        switch (ret) {
            case ESP_ERR_NOT_FOUND:
                ESP_LOGE(TAG, "  Reason: No SD card detected");
                ESP_LOGE(TAG, "  - Check card insertion (fully seated?)");
                ESP_LOGE(TAG, "  - Try different SD card (known good)");
                ESP_LOGE(TAG, "  - Verify card formatted as FAT32");
                break;
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE(TAG, "  Reason: Card initialization failed");
                ESP_LOGE(TAG, "  - Check wiring (loose connections?)");
                ESP_LOGE(TAG, "  - Verify 3.3V power to card");
                ESP_LOGE(TAG, "  - Test card in PC (might be defective)");
                break;
            case ESP_FAIL:
                ESP_LOGE(TAG, "  Reason: Mount/filesystem issue");
                ESP_LOGE(TAG, "  - Card might need formatting (FAT32)");
                ESP_LOGE(TAG, "  - Try different card");
                ESP_LOGE(TAG, "  - Check for write-protect switch");
                break;
            default:
                ESP_LOGE(TAG, "  Error code: 0x%x", ret);
                ESP_LOGE(TAG, "  - Consult ESP-IDF documentation");
                break;
        }
        ESP_LOGE(TAG, "");
        
        s_sd_status = SDLOG_STATUS_FAILED;
        return false;
    }

    s_mounted = true;
    s_sd_status = SDLOG_STATUS_OK;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ SD card mounted successfully");
    ESP_LOGI(TAG, "");
    
    // Card information (useful for diagnostics)
    ESP_LOGI(TAG, "Card Information:");
    ESP_LOGI(TAG, "  Name: %s", s_card->cid.name);
    
    uint64_t capacity_mb = ((uint64_t) s_card->csd.capacity) * s_card->csd.sector_size / (1024 * 1024);
    ESP_LOGI(TAG, "  Capacity: %llu MB", capacity_mb);
    s_total_space_mb = capacity_mb;
    
    const char *speed_str = (s_card->csd.tr_speed > 25000000) ? "High Speed" : "Standard";
    ESP_LOGI(TAG, "  Speed: %s (%lu Hz)", speed_str, (unsigned long)s_card->csd.tr_speed);
    
    ESP_LOGI(TAG, "  Manufacturer: 0x%02X", s_card->cid.mfg_id);
    ESP_LOGI(TAG, "  Sector Size: %u bytes", s_card->csd.sector_size);
    ESP_LOGI(TAG, "");
    
    // Get initial disk space info
    update_disk_space();
    ESP_LOGI(TAG, "Disk Space:");
    ESP_LOGI(TAG, "  Total: %llu MB", s_total_space_mb);
    ESP_LOGI(TAG, "  Free: %llu MB", s_free_space_mb);
    ESP_LOGI(TAG, "  Used: %llu MB (%.0f%%)",
             s_total_space_mb - s_free_space_mb,
             ((float)(s_total_space_mb - s_free_space_mb) / s_total_space_mb) * 100);
    ESP_LOGI(TAG, "");

    // Test log entries to confirm write path
    ESP_LOGI(TAG, "Creating initial log entries...");
    sdlog_printf("=== SD LOGGING SYSTEM INITIALIZED ===");
    sdlog_printf("Card: %s, Capacity: %llu MB, Speed: %s, MFG: 0x%02X", 
                 s_card->cid.name, capacity_mb, speed_str, s_card->cid.mfg_id);
    sdlog_printf("Disk: %llu MB free / %llu MB total", s_free_space_mb, s_total_space_mb);

    ESP_LOGI(TAG, "✓ SD logging ready");
    ESP_LOGI(TAG, "  Log file: /sdcard/Sunflower.log");
    ESP_LOGI(TAG, "  CSV file: /sdcard/soltrac.csv");
    ESP_LOGI(TAG, "");
    
    return true;
}

/*
    Append a timestamped line to /sdcard/Sunflower.log.
    Safe to call periodically; returns quietly if SD is not mounted.
*/
void sdlog_printf(const char *fmt, ...){
    if (!s_mounted) {
        ESP_LOGV(TAG, "sdlog_printf: SD not mounted");
        return;
    }

    uint32_t start_tick = xTaskGetTickCount();
    
    FILE *f = fopen("/sdcard/Sunflower.log", "a");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open Sunflower.log");
        s_failed_writes++;
        if (s_failed_writes > 3) {
            s_sd_status = SDLOG_STATUS_FAILED;
        }
        return;
    }

    // Write timestamp
    fprintf(f, "[%s] ", get_timestamp_str());
    
    // Write message
    va_list ap; 
    va_start(ap, fmt);
    bool success = vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);
    
    // Calculate write time
    uint32_t elapsed_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    s_last_write_time_ms = elapsed_ms;
    s_total_writes++;
    
    if (!success) {
        s_failed_writes++;
        ESP_LOGW(TAG, "Write failed (total failures: %lu)", s_failed_writes);
        if (s_failed_writes > 3) {
            s_sd_status = SDLOG_STATUS_FAILED;
        }
        return;
    }
    
    // Reset failure counter on success
    if (s_failed_writes > 0) s_failed_writes = 0;
    
    // Performance monitoring
    if (elapsed_ms > WRITE_TIME_CRITICAL) {
        ESP_LOGE(TAG, "CRITICAL: Write took %lu ms (>%d ms threshold)", 
                 elapsed_ms, WRITE_TIME_CRITICAL);
        s_sd_status = SDLOG_STATUS_FAILED;
    } else if (elapsed_ms > WRITE_TIME_SLOW) {
        s_slow_writes++;
        ESP_LOGW(TAG, "Slow write: %lu ms (threshold: %d ms)", elapsed_ms, WRITE_TIME_SLOW);
        if (s_sd_status == SDLOG_STATUS_OK) {
            s_sd_status = SDLOG_STATUS_SLOW;
        }
    } else if (elapsed_ms > WRITE_TIME_WARN) {
        ESP_LOGD(TAG, "Write time: %lu ms (acceptable)", elapsed_ms);
    } else {
        ESP_LOGV(TAG, "Write time: %lu ms (excellent)", elapsed_ms);
        // Recover from SLOW status if writes improve
        if (s_sd_status == SDLOG_STATUS_SLOW && s_slow_writes < 3) {
            s_sd_status = SDLOG_STATUS_OK;
            ESP_LOGI(TAG, "SD performance recovered");
        }
    }

    // Echo to console at debug level
    va_list ap2; 
    va_start(ap2, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);
    ESP_LOGD(TAG, "LOG: %s", buf);
    
    // Periodic disk space check
    update_disk_space();
}

/*
    Create CSV file with header matching tracker_data_t structure.
*/
void sdlog_write_csv_header_if_new(const char *path){
    if (!s_mounted) {
        ESP_LOGV(TAG, "write_csv_header: SD not mounted");
        return;
    }

    FILE *f = fopen(path, "r");
    if (f) { 
        fclose(f); 
        ESP_LOGD(TAG, "CSV file exists, skipping header: %s", path);
        return; 
    }

    f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "Failed to create CSV file: %s", path);
        s_failed_writes++;
        return;
    }

    // Header matching tracker_data_t structure
    fprintf(f, "timestamp,latitude,longitude,gps_valid,gps_sats,gps_fix_age_sec,");
    fprintf(f, "sun_elevation,sun_azimuth,panel_elevation,panel_azimuth,");
    fprintf(f, "delta_elevation,delta_azimuth,tracking_quality,");
    fprintf(f, "battery_voltage,battery_soc_percent,battery_soc_level,battery_charging,battery_adc,");
    fprintf(f, "moves_today,total_moves,uptime_hours,");
    fprintf(f, "status,wifi_clients,wifi_rssi,sd_status,");
    fprintf(f, "sunrise_time,sunset_time,notes\n");
    fflush(f);
    fclose(f);

    ESP_LOGI(TAG, "✓ Created CSV file with header: %s", path);
    sdlog_printf("CSV file created: %s", path);
}

/*
    Append a single CSV row matching tracker_data_t structure.
*/
void sdlog_write_csv(const char *path, const char *fmt, ...){
    if (!s_mounted) {
        ESP_LOGV(TAG, "write_csv: SD not mounted");
        return;
    }

    uint32_t start_tick = xTaskGetTickCount();

    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open CSV: %s", path);
        s_failed_writes++;
        if (s_failed_writes > 3) {
            s_sd_status = SDLOG_STATUS_FAILED;
        }
        return;
    }

    va_list ap; 
    va_start(ap, fmt);
    bool success = vwritef(f, fmt, ap);
    va_end(ap);
    fclose(f);
    
    uint32_t elapsed_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    s_last_write_time_ms = elapsed_ms;
    s_total_writes++;
    
    if (!success) {
        s_failed_writes++;
        ESP_LOGW(TAG, "CSV write failed (failures: %lu)", s_failed_writes);
        if (s_failed_writes > 3) {
            s_sd_status = SDLOG_STATUS_FAILED;
        }
        return;
    }
    
    if (s_failed_writes > 0) s_failed_writes = 0;

    // Performance monitoring (same as sdlog_printf)
    if (elapsed_ms > WRITE_TIME_CRITICAL) {
        ESP_LOGE(TAG, "CRITICAL CSV write: %lu ms", elapsed_ms);
        s_sd_status = SDLOG_STATUS_FAILED;
    } else if (elapsed_ms > WRITE_TIME_SLOW) {
        s_slow_writes++;
        ESP_LOGW(TAG, "Slow CSV write: %lu ms", elapsed_ms);
        if (s_sd_status == SDLOG_STATUS_OK) {
            s_sd_status = SDLOG_STATUS_SLOW;
        }
    } else {
        ESP_LOGV(TAG, "CSV write: %lu ms", elapsed_ms);
        if (s_sd_status == SDLOG_STATUS_SLOW && s_slow_writes < 3) {
            s_sd_status = SDLOG_STATUS_OK;
        }
    }

    update_disk_space();
}

// Get current SD status
uint8_t sdlog_get_status(void) {
    return s_sd_status;
}

// Get last write time
uint32_t sdlog_get_last_write_time_ms(void) {
    return s_last_write_time_ms;
}

// Get disk info
bool sdlog_get_disk_info(uint64_t *total_mb, uint64_t *used_mb, uint64_t *free_mb) {
    if (!s_mounted) return false;
    
    update_disk_space();  // Refresh if stale
    
    if (total_mb) *total_mb = s_total_space_mb;
    if (free_mb) *free_mb = s_free_space_mb;
    if (used_mb) *used_mb = s_total_space_mb - s_free_space_mb;
    
    return true;
}

// Check mount status
bool sdlog_is_mounted(void) {
    return s_mounted;
}

// Get card info string
const char* sdlog_get_card_info(void) {
    static char info[128];
    
    if (!s_mounted || !s_card) {
        snprintf(info, sizeof(info), "Not mounted");
        return info;
    }
    
    const char *speed = (s_card->csd.tr_speed > 25000000) ? "High Speed" : "Standard";
    uint64_t capacity_mb = ((uint64_t)s_card->csd.capacity) * s_card->csd.sector_size / (1024 * 1024);
    
    snprintf(info, sizeof(info), "Name: %s, Size: %llu MB, Speed: %s, MFG: 0x%02X",
             s_card->cid.name, capacity_mb, speed, s_card->cid.mfg_id);
    
    return info;
}

