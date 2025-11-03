# Sunflower Secondary (ESP32 LCD Display Client)

ESP-IDF project for the secondary display unit that connects to the master Sunflower tracker controller over Wi‑Fi and renders a real-time dashboard on a 3.5" ILI9486 TFT display (480×320, SPI).

---

## Overview

This client provides a visual interface for monitoring the solar tracker's status in real-time. It displays:
- **Solar panel orientation** (elevation and azimuth angles)
- **Tracking deltas** (rate of change in degrees)
- **Battery voltage & state of charge** with ADC reading
- **Sun position** (calculated elevation and azimuth)
- **GPS coordinates** with satellite count and fix age
- **System statistics** (moves today/total, uptime, WiFi signal strength)
- **Status indicators** (tracking, standby, sleeping, calibrating, error with SD card status)
- **Current time, sunrise & sunset times**
- **Battery voltage history graph** (rolling 100-sample plot)

The display automatically connects to the master controller's Wi-Fi access point and begins receiving telemetry data via TCP socket with automatic reconnection on connection loss.

---

## Features

### Startup Sequence
1. **Splash Screen**: Shows Sunflower logo and "System Starting..." message
2. **WiFi Connection**: "Connecting to WiFi..." with status
3. **Connection Status**: "Connected! Loading data..." 
4. **Dashboard**: Full real-time telemetry interface

### Dashboard Layout

```
┌──────────────────────────────────────────────────────────────┐
│ 🌻 SUNFLOWER TRACKER                     12/25/2024 14:32:15│  ← Header (30px)
├──────────────────┬──────────────────┬────────────────────────┤
│ ELEVATION        │ AZIMUTH          │ BATTERY                │
│                  │                  │                        │
│ 45.3°            │ 180.5°           │ 12.34V    80%          │
│                  │                  │                        │
│ Δ: +2.5°         │ Δ: +15.2°        │ ADC:3456  [████░] CHG  │
├──────────────────┼──────────────────┼────────────────────────┤  Row 1 (70px)
│ SUN POSITION     │ SYSTEM STATUS    │ GPS                    │
│                  │                  │ 43.6532,-79.3832       │
│ El: 48.7°        │ TRACKING         │ Sats:12 Age:2s         │
│ Az: 185.3°       │                  │------------------------│
│                  │ Err:3° SD:OK     │ STATISTICS             │
│ R:06:45 S:18:30  │                  │ Today:45 Total:1234    │
│                  │                  │ Up:12h WiFi:-45dBm     │  ← Tracker RSSI
│                  │                  │            /-24dBm     │  ← Client RSSI
├──────────────────┴──────────────────┴────────────────────────┤  Row 2 (70px)
│ Battery Voltage History (Last 100 Readings)                  │
│ 15.0V ┤                                    ╱──╲              │
│       │                           ╱───────╯    ╲             │
│ 13.0V │                  ╱───────╯              ╲──          │
│       │         ╱───────╯                          ●         │
│ 11.0V └─────────────────────────────────────────────────────→│
│                                              Time →          │
└──────────────────────────────────────────────────────────────┘  Row 3 (135px)
```

### Color-Coded Status Panel
- **🟢 GREEN** (TRACKING): Actively following the sun
- **🟠 AMBER** (STANDBY/SLEEPING/CALIBRATING): Idle or transitioning states
- **🔴 RED** (ERROR): Connection lost or critical fault

### Battery Display
- **Voltage**: Real-time reading (12.34V)
- **Percentage**: State of charge (0-100%)
- **ADC Reading**: Raw ADC value (0-4095)
- **Level Indicator**: ASCII bar graph `[████░]`
- **Charging Status**: "CHG" indicator when charging
- **History Graph**: Rolling 100-sample voltage plot with auto-scaling

### WiFi Signal Strength Display
Shows **dual RSSI values** in format: `WiFi:-45dBm / -24dBm`
- **First value (-45 dBm)**: How well **display sees tracker** (client's perspective)
- **Second value (-24 dBm)**: How well **tracker sees display** (from received data packet)
- Both values updated in real-time
- **Interpretation**:
  - Both strong (-20 to -50 dBm): Excellent bidirectional link
  - Display weak, tracker strong: Display needs repositioning
  - Tracker weak, display strong: Tracker antenna issue

### System Status Panel
Displays multi-line status with:
- **Primary Status**: TRACKING, STANDBY, SLEEPING, CALIBRATING, ERROR
- **Error Metric**: Tracking accuracy in degrees (e.g., "Err:3°")
- **SD Card Status**: OK, ERROR, or none
- Color-coded based on system state

### GPS Panel
- **Coordinates**: Latitude, Longitude (decimal degrees)
- **Satellite Count**: Number of satellites in view
- **Fix Age**: Time since last valid GPS fix (seconds)
- Updates when GPS lock is acquired

### Auto-Reconnection
- **WiFi disconnected**: Automatic reconnection every 5 seconds
- **TCP disconnected**: Automatic reconnection every 2 seconds
- **Connection verification**: Socket health checks on reconnect
- **Graceful degradation**: Shows "Reconnecting..." screen during outages
- **Smart error display**: Only shows error after first successful data reception (prevents false alarms during startup)

---

## Hardware Requirements

### Components
- **MCU**: ESP32 WROOM (or ESP32-CAM without camera)
- **Display**: 3.5" ILI9486 TFT LCD
  - Resolution: 480×320 pixels
  - Color depth: RGB565 (16-bit)
  - Interface: SPI
  - Touch: Not used

### Wiring (ESP32 to ILI9486)

| ESP32 Pin | ILI9486 Pin | Function |
|-----------|-------------|----------|
| GPIO 23   | SDI (MOSI)  | Data out |
| GPIO 18   | SCK         | Clock    |
| GPIO 5    | CS          | Chip select |
| GPIO 21   | DC/RS       | Data/Command |
| GPIO 4    | RESET       | Reset    |
| GPIO 22   | LED (BL)    | Backlight (PWM) |
| 5V        | VCC         | Power    |
| GND       | GND         | Ground   |
| —         | SDO (MISO)  | Not connected |

**Notes**:
- Use short wires (<15cm) for stable SPI communication
- Display should have onboard 5V regulator (most ILI9486 modules use 3.3V)
- Backlight controlled via PWM (brightness adjustable 0-100%)
- If using 5V-only display, add level shifters on data lines

---

## Software Setup

### Prerequisites
- **ESP-IDF v5.x** installed ([installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- USB driver for ESP32 (CP210x or CH340)
- Serial terminal (built into ESP-IDF or use PuTTY/screen)

### Build and Flash

#### Windows (PowerShell with ESP-IDF)
```powershell
# Navigate to project
cd "C:\Users\pathToProjectFolder"

# Set target (only needed once)
idf.py set-target esp32

# Build firmware
idf.py build

# Flash to ESP32 (replace COM3 with your port)
idf.py -p COM3 flash

# Monitor serial output
idf.py -p COM3 monitor

# Exit monitor: Ctrl+]
```

#### Linux/macOS
```bash
cd ~/Sunflower_Secondary

idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash   # or /dev/cu.usbserial-* on macOS
idf.py -p /dev/ttyUSB0 monitor
```

### Configuration

#### Wi-Fi Credentials (Master AP)
Edit `components/wifi_client/wifi_client.c`:
```c
#define WIFI_SSID      "SunflowerTracker"  // Master SoftAP SSID
#define WIFI_PASS      "sunflower2025"     // Password (WPA2-PSK)
#define SERVER_IP      "192.168.4.1"       // Master gateway IP
#define SERVER_PORT    8888                // TCP port
```

#### Display Settings
Change as needed to fit your style.
Edit `components/lcd/include/lcd.h`:
```c
#define LCD_WIDTH   480
#define LCD_HEIGHT  320

// Professional monochrome ASCII theme
#define LCD_BLACK       0x0000  // Pure black background
#define LCD_WHITE       0xFFFF  // White text
#define LCD_CHARCOAL    0x3186  // Dark gray (panel backgrounds)
#define LCD_DARKBLUE    0x2945  // Dark navy (alternate panels)
#define LCD_GOLDEN      0xFEA0  // Golden yellow (primary values)
#define LCD_TEAL        0x07FF  // Cyan-blue (delta values)
#define LCD_SAGE        0x07E0  // Green (good status)
#define LCD_CRIMSON     0xF800  // Red (errors)
```

#### Backlight Brightness
Edit `main/main.c`:
```c
lcd_client_set_brightness(80);  // 0-100%, default 80%
```

#### SPI Speed (if display glitches)
Edit `components/lcd/lcd.c` in `lcd_client_init()`:
```c
.clock_speed_hz = 10 * 1000 * 1000,  // Lower to 5MHz if needed
```

---

## Data Protocol

The client expects binary frames matching this exact structure (must align with master):

```c
typedef struct {
    float    elevation;               // Panel elevation angle (0..90°)
    float    azimuth;                 // Panel azimuth angle (0..360°)
    float    delta_elevation;         // Elevation change rate (°/update)
    float    delta_azimuth;           // Azimuth change rate (°/update)
    uint16_t battery_adc;             // Raw ADC value (0..4095)
    float    battery_voltage;         // Battery voltage (V)
    float    battery_soc_percent;     // State of charge (0..100%)
    uint8_t  battery_soc;             // Battery level (0..5)
    uint8_t  battery_charging;        // Charging status (0/1)
    uint32_t timestamp;               // Current time (Unix timestamp)
    uint32_t sunrise_time;            // Sunrise time today (Unix timestamp)
    uint32_t sunset_time;             // Sunset time today (Unix timestamp)
    uint8_t  status;                  // System status code (see table below)
    float    latitude;                // GPS latitude (-90..+90°)
    float    longitude;               // GPS longitude (-180..+180°)
    uint8_t  gps_valid;               // GPS fix valid (0/1)
    uint8_t  gps_satellites;          // Number of satellites in view
    uint32_t last_gps_fix_age_sec;    // Time since last GPS fix (seconds)
    float    sun_elevation;           // Calculated sun elevation (0..90°)
    float    sun_azimuth;             // Calculated sun azimuth (0..360°)
    uint32_t moves_today;             // Number of moves today
    uint32_t total_moves;             // Total lifetime moves
    uint16_t uptime_hours;            // System uptime (hours)
    int8_t   wifi_rssi;               // WiFi signal strength (tracker's view of client)
    uint8_t  wifi_clients;            // Number of connected clients
    uint8_t  sd_card_status;          // SD card status (0=none, 1=ok, 2=error)
    uint8_t  tracking_quality;        // Tracking quality/error metric (0..100 or error degrees)
} __attribute__((packed)) tracker_data_t;
```

**Important**:
- `__attribute__((packed))` ensures no padding between fields
- Little-endian byte order (ESP32 native)
- Master must send exactly `sizeof(tracker_data_t)` bytes per frame
- TCP stream, frames sent at ~1 Hz
- Frame size: 92 bytes

### Status Code Mapping
| Value | State       | Display Text | Description |
|-------|-------------|--------------|-------------|
| 0     | Standby     | STANDBY      | Waiting for tracking window |
| 1     | Tracking    | TRACKING     | Actively following sun |
| 2     | Sleeping    | SLEEPING     | Night mode (motors off) |
| 3     | Calibrating | CALIBRATING  | Finding home position |
| 255   | Error       | ERROR        | Critical fault detected |

---

## Project Structure

```
Sunflower_Secondary/
├── CMakeLists.txt              # Top-level build config
├── README.md                   # This file
├── sdkconfig                   # ESP-IDF configuration
├── main/
│   ├── CMakeLists.txt
│   └── main.c                  # Entry point, WiFi init, main loop
└── components/
    ├── lcd/                    # Display driver & UI renderer
    │   ├── CMakeLists.txt
    │   ├── lcd.c               # SPI driver, drawing primitives, dashboard panels
    │   └── include/
    │       └── lcd.h           # Public API, color palette, layout constants
    ├── sunflower_logo/         # Logo bitmap (embedded in header)
    │   ├── CMakeLists.txt
    │   └── include/
    │       └── sunflower_logo.h  # 24×24 RGB565 logo data
    └── wifi_client/            # Wi-Fi STA + TCP client
        ├── CMakeLists.txt
        ├── wifi_client.c       # Connection logic, auto-reconnect, receive handler
        └── include/
            └── wifi_client.h   # API, tracker_data_t definition, statistics
```

---

## Runtime Behavior

### Startup Flow
1. **NVS Flash Init**: Initialize non-volatile storage (required for WiFi)
2. **LCD Init**: 
   - Configure SPI bus (10 MHz)
   - Reset ILI9486 display
   - Set landscape orientation (480×320)
   - Initialize PWM for backlight
   - Show splash screen with logo
3. **WiFi Connection**:
   - Initialize WiFi stack
   - Connect to "SunflowerTracker" AP as station
   - Wait for DHCP IP assignment (192.168.4.x)
   - Apply performance optimizations (max TX power, no power saving)
4. **TCP Connection**:
   - Create TCP socket
   - Connect to 192.168.4.1:8888
   - Enable TCP_NODELAY and keepalive
   - Verify connection health
5. **Data Reception**: Wait for first tracker packet
6. **Dashboard**: Switch to full telemetry display

### Main Loop (after initialization)
```c
while (1) {
    // Check WiFi connection status
    if (!wifi_client_is_connected()) {
        lcd_client_show_init_screen("Reconnecting to WiFi");
        continue;
    }
    
    // Receive data from tracker (2s timeout)
    esp_err_t ret = wifi_client_receive_data(&data, 2000);
    
    if (ret == ESP_OK) {
        // Update dashboard with new data
        lcd_client_display_dashboard(&data);
        no_data_counter = 0;
        first_data_received = true;
    } 
    else if (ret == ESP_ERR_TIMEOUT) {
        // No data yet (normal during startup)
        no_data_counter++;
        
        // Only show error after first successful data reception
        if (no_data_counter >= 10 && first_data_received) {
            lcd_client_show_error("Waiting for tracker data");
        }
    }
    else {
        // Connection error - auto-reconnect in background
        if (first_data_received) {
            lcd_client_show_error("Connection error");
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz update rate
}
```

### Auto-Reconnection Strategy
- **WiFi Disconnect Detection**: 
  - Event handler detects `WIFI_EVENT_STA_DISCONNECTED`
  - Closes TCP socket immediately
  - Attempts reconnection every 5 seconds
  - Logs disconnect reason (authentication, beacon timeout, etc.)
  
- **TCP Disconnect Detection**:
  - `recv()` returns 0 (graceful close) or error
  - `getsockopt(SO_ERROR)` checks socket health
  - Closes socket and attempts reconnection every 2 seconds
  
- **Connection Verification**:
  - After TCP reconnect, sends 0-byte test packet
  - Verifies socket is actually writable
  - Prevents "connected but dead" state
  
- **Stale Flag Detection**:
  - Checks actual WiFi state vs. cached flag
  - Corrects mismatches automatically
  - Ensures UI reflects true connection status

### Incremental Dashboard Updates
- **Full redraw** on first data reception or after error screen
- **Partial updates** for changed values only:
  - Compare new data vs. `last_data` structure
  - Redraw only panels with changed values
  - Battery graph appends new point (no full redraw)
- **Update frequency**: ~10 Hz (limited by 100ms delay)
- **Data rate**: Tracker sends ~1 Hz, display interpolates

---

## Troubleshooting

### Display Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Blank screen | No power / wrong wiring | Check 3.3V supply, verify pin connections |
| White screen | Panel not initialized | Check RST pin, try lowering SPI speed to 5MHz |
| Garbled graphics | SPI too fast / loose wires | Shorten wires to <15cm, lower clock speed |
| Partial display | Incorrect rotation | Verify MADCTL command (0x36) in `lcd.c` |
| Flickering | Backlight PWM issues | Check GPIO22 connection, verify LEDC config |
| Text unreadable | Font size too small | Check font rendering in `draw_string()` |

### Wi-Fi Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Cannot join AP | Wrong SSID/password | Check credentials match tracker exactly |
| "Reconnecting" screen at startup | Normal during connection | Wait 10-15 seconds for initial connection |
| Stuck at "Reconnecting" | Tracker not powered on | Start tracker first, check WiFi AP is active |
| IP acquired but no data | TCP server not running | Verify tracker listening on port 8888 |
| Frequent disconnects | Weak signal / interference | Move units closer, check dual RSSI values on display |
| "Connection Lost" immediately after connect | Socket verification failing | Check tracker accepts connections, not at client limit |

### Data Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| All zeros displayed | `memset()` not overwritten | Check tracker is sending valid data |
| Corrupted values | Struct mismatch | Verify `tracker_data_t` identical on both sides |
| Negative angles | Incorrect type interpretation | Ensure both sides use `float` for angles |
| No GPS coordinates | `gps_valid=0` | Normal without GPS fix, wait or check tracker GPS |
| Battery always 0% | ADC scaling issue | Check `battery_voltage` field, verify ADC code |
| Graph not updating | History buffer not appending | Check `voltage_history[]` writes in `draw_voltage_graph()` |
| Wrong WiFi RSSI | Using wrong value | Top value is client's view, bottom is tracker's view |
| Dual RSSI shows -128/-128 | Not connected | Both values update after WiFi connection established |

### Connection Recovery Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Stays on error screen after reconnect | `first_data_received` not set | Should auto-clear after receiving first packet |
| Never reconnects | Event handler not called | Check WiFi stack initialization, event loop running |
| Reconnects but no data | TCP socket not reopening | Check `connect_tcp()` return code in logs |
| Client RSSI shows -128 dBm | `esp_wifi_sta_get_ap_info()` failing | Normal during disconnect, updates on reconnect |
| Tracker RSSI shows 0 dBm | Not in data packet | Tracker must populate `wifi_rssi` field |

### Serial Monitor Output

**Expected logs during normal operation:**
```
I (320) LCD_MAIN: ╔════════════════════════════════════════════════════════════╗
I (320) LCD_MAIN: ║          SUNFLOWER LCD DISPLAY CLIENT                      ║
I (320) LCD_MAIN: ╚════════════════════════════════════════════════════════════╝
I (330) LCD_MAIN: Build: Nov 2 2025 14:30:15
I (450) LCD: ✓ LCD initialized
I (450) LCD: Display: ILI9486 480x320
I (500) WIFI_CLIENT: Connecting to tracker WiFi...
I (2100) WIFI_CLIENT: ╔════════════════════════════════════════════════════════════╗
I (2100) WIFI_CLIENT: ║          WIFI CONNECTED                                    ║
I (2100) WIFI_CLIENT: ╚════════════════════════════════════════════════════════════╝
I (2100) WIFI_CLIENT: ✓ Connected to tracker AP
I (2100) WIFI_CLIENT: IP: 192.168.4.2
I (2100) WIFI_CLIENT: Signal Strength: -45 dBm
I (2200) WIFI_CLIENT: ✓ TCP connection verified and ready
I (2300) LCD_MAIN: ✓ Connected to tracker
I (2300) LCD_MAIN: SSID: SunflowerTracker
I (2300) LCD_MAIN: IP: 192.168.4.2
I (2300) LCD_MAIN: Signal: -45 dBm
I (3400) LCD_MAIN: ╔════════════════════════════════════════════════════════════╗
I (3400) LCD_MAIN: ║          ENTERING MAIN DISPLAY LOOP                        ║
I (3400) LCD_MAIN: ╚════════════════════════════════════════════════════════════╝
I (13500) LCD_MAIN: Display Update #1
I (13500) LCD_MAIN: Panel Position: El:45.3° Az:180.5°
I (13500) LCD_MAIN: Battery: 12.34V 80% Charging:YES
I (13500) LCD_MAIN: GPS: 43.653200, -79.383200 Sats:12 Valid:YES
I (13500) LCD_MAIN: WiFi RSSI (Client): -45 dBm
I (13500) LCD_MAIN: WiFi RSSI (Tracker): -24 dBm
```

**Expected logs during reconnection:**
```
W (45200) WIFI_CLIENT: ╔════════════════════════════════════════════════════════════╗
W (45200) WIFI_CLIENT: ║          WIFI DISCONNECTED                                 ║
W (45200) WIFI_CLIENT: ╚════════════════════════════════════════════════════════════╝
W (45200) WIFI_CLIENT: ⚠ Disconnected from AP
W (45200) WIFI_CLIENT: Reason: 8 (Disassociated)
I (45200) WIFI_CLIENT: Reconnecting in 5 seconds... (attempt 1)
I (50300) WIFI_CLIENT: ✓ Connected to tracker AP
I (50400) WIFI_CLIENT: ✓ TCP connection verified and ready
I (50500) WIFI_CLIENT: ✓ Full connection restored
```

---

## Performance Notes

- **SPI Speed**: 10 MHz (stable with <15cm wires)
- **Frame Rate**: ~5-10 FPS for full dashboard updates
- **Data Latency**: <200ms from tracker send to display update
- **Memory Usage**: 
  - Heap: ~40 KB (line buffers + voltage history)
  - Stack: ~8 KB per task
- **WiFi Performance**:
  - TX Power: 19.5 dBm (maximum)
  - Power saving: Disabled (always-on for low latency)
  - Bandwidth: 20 MHz (better range than 40 MHz)
  - Expected range: 50-100m line-of-sight

### Power Consumption
- **ESP32 + LCD Active**: 200-400 mA @ 3.3V
- **WiFi overhead**: ~100-150 mA
- **LCD backlight**: ~50-100 mA (adjustable)
- **Total**: Requires USB power or large battery

### Optimization Opportunities
- [x] Incremental panel updates (only redraw changed values)
- [x] Auto-reconnection with verification
- [x] Dual RSSI display for diagnostics
- [x] ASCII-based UI (minimal memory, fast rendering)
- [ ] DMA-capable SPI buffers (reduce CPU load)
- [ ] Static line buffers (avoid malloc on each draw)
- [ ] Touch screen support for settings menu

---

## API Reference

### WiFi Client (`wifi_client.h`)

```c
// Initialize WiFi and connect to tracker
esp_err_t wifi_client_init(void);

// Receive tracking data (blocking with timeout)
esp_err_t wifi_client_receive_data(tracker_data_t *data, uint32_t timeout_ms);

// Manually trigger reconnection
esp_err_t wifi_client_reconnect(void);

// Check connection status (verifies socket health)
bool wifi_client_is_connected(void);

// Get client's WiFi signal strength (dBm)
int8_t wifi_client_get_signal_strength(void);

// Get assigned IP address
const char* wifi_client_get_ip_address(void);

// Get connection statistics
void wifi_client_get_stats(wifi_client_stats_t *stats);
```

### LCD Display (`lcd.h`)

```c
// Initialize display with pin configuration
esp_err_t lcd_client_init(const lcd_config_t *config);

// Set backlight brightness (0-100%)
void lcd_client_set_brightness(uint8_t brightness);

// Show initialization/connection screen
void lcd_client_show_init_screen(const char *message);

// Update dashboard with new data (incremental)
void lcd_client_display_dashboard(const tracker_data_t *data);

// Show error message overlay
void lcd_client_show_error(const char *message);
```

---

## Known Limitations

1. **No touch screen support**: Display is output-only, no interactive controls
2. **Fixed WiFi credentials**: Must recompile to change SSID/password
3. **Single tracker support**: Cannot switch between multiple trackers
4. **No SD card logging**: Display does not store historical data
5. **Fixed update rate**: 1 Hz from tracker, 10 Hz display refresh
6. **No OTA updates**: Must reflash via USB for firmware updates
7. **ASCII-only display**: No graphical icons or antialiased fonts

---

## License

Part of the ELEC 4020 Capstone Project - Sunflower Solar Tracker.  
Queen's University, Kingston, ON, Canada.

---

## Authors

- **Display Client Development**: Abdul Mohammed (ESP32, WiFi, LCD driver, UI design)
- **System Architecture**: [Abdul Mohammed]

---

## Support

For issues or questions:
1. Check serial monitor logs for error messages
2. Verify wiring matches pin assignments
3. Test with known-good tracker unit
4. Review troubleshooting section above
5. Contact project team for hardware/integration issues

---

## References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ILI9486 Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ILI9486.pdf)
- [ESP32 WiFi Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- [FreeRTOS Task Management](https://www.freertos.org/a00106.html)
- [RGB565 Color Picker](http://www.barth-dev.de/online/rgb565-color-picker/)
- [SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)
