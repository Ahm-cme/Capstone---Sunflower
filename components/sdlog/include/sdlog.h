#pragma once
#include <stdarg.h>
#include <stdbool.h>

/*
    SD Card Logging (onboard microSD reader, SPI/SDSPI)

    Summary:
    - Append-only logging to FAT filesystem mounted at /sdcard.
    - Files are opened, written, flushed, and closed on each call.
    - Two use cases: human-readable text log + CSV telemetry.

    Notes:
    - Not thread-safe; call from one task or protect externally.
    - Strap pins caution: GPIO2 and GPIO15 can affect boot if pulled.
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
    Initialize and mount /sdcard.

    What it does:
    - Initializes SDSPI host on the given pins.
    - Mounts FAT filesystem at /sdcard using esp_vfs_fat_sdspi_mount.
    - Logs basic card info.

    Returns:
    - true on success (mounted and ready), false on failure.
*/
bool sdlog_init(const sdlog_cfg_t *cfg);

/*
    Append a timestamped line to /sdcard/Sunflower.log.

    Behavior:
    - Silent no-op if SD not mounted or file can't be opened.
    - Adds newline automatically; fflush() to minimize data loss.

    Params:
    - fmt,...: printf-style message.
*/
void sdlog_printf(const char *fmt, ...);

/*
    Create CSV file with header if it does not exist.

    Header columns (must match writes):
    unix_ts,lat,lon,fix,sats,az_target,el_target,az_cur,el_cur,
    moves_today,total_moves,batt_v,notes

    Params:
    - path: target CSV path (e.g., "/sdcard/soltrac.csv").
*/
void sdlog_write_csv_header_if_new(const char *path);

/*
    Append one CSV row.

    Behavior:
    - Silent no-op if SD not mounted or file can't be opened.
    - Opens → writes one line → flushes → closes.

    Params:
    - path: CSV file path.
    - fmt,...: printf-style format string for the row (match header).
*/
void sdlog_write_csv(const char *path, const char *fmt, ...);

