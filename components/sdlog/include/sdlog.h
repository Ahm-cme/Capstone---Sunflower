#pragma once
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

/*
    SD Card Logging (onboard microSD reader, SPI/SDSPI)

    Summary:
    - Append-only logging to FAT filesystem mounted at /sdcard.
    - Files are opened, written, flushed, and closed on each call.
    - Two use cases: human-readable text log + CSV telemetry.
    - Provides real-time health monitoring for display integration.

    Files Created:
    - /sdcard/Sunflower.log    - Human-readable timestamped events
    - /sdcard/Sunflower.csv       - Machine-readable tracking telemetry
    
    Health Monitoring:
    - Tracks write performance, disk space, mount failures
    - Status exposed via sdlog_get_status() for display
    - Automatic recovery from transient errors

    Notes:
    - Not thread-safe; call from one task or protect externally.
    - Strap pins caution: GPIO2 and GPIO15 can affect boot if pulled.
    - Immediate flush after each write for crash resilience.
*/

/*
    SPI pin map for the SDSPI host.
    - mosi/miso/sclk/cs: board wiring for the onboard microSD slot.
    - Avoid GPIO2 and GPIO15 if possible (ESP32 boot strap pins).
*/
typedef struct {
    int mosi;
    int miso;  // GPIO2 is a strap pin on many boards
    int sclk;
    int cs;    // GPIO15 is a strap pin on many boards
} sdlog_cfg_t;

/*
    SD Card Health Status (for display integration)
    
    Returned by sdlog_get_status():
    - 0: OK          - Normal operation, writes fast (<200ms)
    - 1: SLOW        - Writes taking 200-500ms (fragmentation/aging)
    - 2: FULL        - >90% capacity used, nearing full
    - 3: FAILED      - Mount/write errors, card unusable
*/
#define SDLOG_STATUS_OK      0
#define SDLOG_STATUS_SLOW    1
#define SDLOG_STATUS_FULL    2
#define SDLOG_STATUS_FAILED  3

/*
    Initialize and mount /sdcard.

    What it does:
    - Initializes SDSPI host on the given pins.
    - Mounts FAT filesystem at /sdcard using esp_vfs_fat_sdspi_mount.
    - Logs basic card info (name, capacity, speed).
    - Creates initial log entry with card details.

    Returns:
    - true on success (mounted and ready), false on failure.
    
    On failure:
    - Sets internal status to SDLOG_STATUS_FAILED
    - Logs detailed error to console
    - Safe to call other functions (they become no-ops)
*/
bool sdlog_init(const sdlog_cfg_t *cfg);

/*
    Append a timestamped line to /sdcard/Sunflower.log.

    Format:
    [2024-10-29 18:32:45] Your message here
    
    Behavior:
    - Silent no-op if SD not mounted or file can't be opened.
    - Adds newline automatically; fflush() to minimize data loss.
    - Tracks write time for health monitoring.
    - Echoes to console at debug level.

    Params:
    - fmt,...: printf-style message (max 256 chars).
    
    Performance:
    - Typical write: 10-50ms
    - Slow write warning: >200ms
    - Critical slowdown: >500ms (sets SLOW status)
*/
void sdlog_printf(const char *fmt, ...);

/*
    Create CSV file with header if it does not exist.

    Header columns (matches tracker_data_t structure):
    timestamp,latitude,longitude,gps_valid,gps_sats,gps_fix_age_sec,
    sun_elevation,sun_azimuth,panel_elevation,panel_azimuth,
    delta_elevation,delta_azimuth,tracking_quality,
    battery_voltage,battery_soc_percent,battery_soc_level,battery_charging,battery_adc,
    moves_today,total_moves,uptime_hours,
    status,wifi_clients,wifi_rssi,sd_status,
    sunrise_time,sunset_time,notes

    Params:
    - path: target CSV path (e.g., "/sdcard/soltrac.csv").
    
    Behavior:
    - Checks if file exists before creating
    - Only writes header to new files
    - Logs creation to console and text log
*/
void sdlog_write_csv_header_if_new(const char *path);

/*
    Append one CSV row.

    Format (must match header):
    1698592365,43.653225,-79.383186,1,12,2,45.3,180.5,45.0,180.0,0.3,0.5,3,
    12.34,85.5,3,1,3456,45,1234,12,1,2,-45,0,
    1698588000,1698619200,"Normal tracking"

    Behavior:
    - Silent no-op if SD not mounted or file can't be opened.
    - Opens → writes one line → flushes → closes.
    - Tracks write performance for health status.
    - Logs slow writes to console.

    Params:
    - path: CSV file path (typically "/sdcard/soltrac.csv").
    - fmt,...: printf-style format string for the row (match header).
    
    Performance tracking:
    - Normal: <100ms per row
    - Warning: 100-200ms
    - Slow: 200-500ms (status set to SLOW)
    - Failed: >500ms or error (status set to FAILED)
*/
void sdlog_write_csv(const char *path, const char *fmt, ...);

/*
    Get SD card status for health monitoring.
    
    Used by:
    - Main loop to populate tracker_data_t.sd_card_status
    - Display to show SD health indicator
    - Tracking module to log SD issues
    
    Returns:
    - SDLOG_STATUS_OK (0)      - Working normally, writes fast
    - SDLOG_STATUS_SLOW (1)    - Write delays detected (200-500ms)
    - SDLOG_STATUS_FULL (2)    - >90% disk space used
    - SDLOG_STATUS_FAILED (3)  - Mount/write errors, card unusable
    
    Status Updates:
    - OK → SLOW: Sustained slow writes detected
    - SLOW → OK: Write performance recovers
    - Any → FAILED: Mount error or critical write failure
    - FAILED → OK: Only via reboot/remount
    
    Note:
    - Call frequently (every 1-10 seconds) for real-time monitoring
    - Status persists until condition changes or reboot
*/
uint8_t sdlog_get_status(void);

/*
    Get last write performance metric.
    
    Returns:
    - Time in milliseconds for last sdlog_printf() or sdlog_write_csv() call
    - 0 if no writes have occurred since boot
    
    Used for:
    - Debugging slow SD card issues
    - Display performance diagnostics
    - Determining if card is aging/fragmenting
*/
uint32_t sdlog_get_last_write_time_ms(void);

/*
    Get SD card capacity information.
    
    Fills provided pointers with:
    - total_mb: Total card capacity in megabytes
    - used_mb: Space used in megabytes
    - free_mb: Space remaining in megabytes
    
    Returns:
    - true: Values filled successfully
    - false: SD card not mounted or info unavailable
    
    Used by:
    - Display to show disk space gauge
    - System check to warn if nearly full
    - Logging module to predict when to archive/rotate logs
*/
bool sdlog_get_disk_info(uint64_t *total_mb, uint64_t *used_mb, uint64_t *free_mb);

/*
    Check if SD card is mounted and operational.
    
    Returns:
    - true: Card mounted, writes will succeed
    - false: Card not mounted, all writes are no-ops
    
    Quick check before critical writes to avoid delays.
*/
bool sdlog_is_mounted(void);

/*
    Get SD card information string (for diagnostics).
    
    Returns pointer to static buffer with:
    "Name: XXXXX, Size: XXX MB, Speed: High/Standard, MFG: 0xXX"
    
    Used by:
    - System check to display card details
    - Troubleshooting when users report SD issues
    - Logging card changes on remount
    
    Note:
    - Returns "Not mounted" if card not available
    - Buffer is overwritten on each call (not thread-safe)
*/
const char* sdlog_get_card_info(void);

