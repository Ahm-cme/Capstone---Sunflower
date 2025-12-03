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

The display automatically connects to the master controller's Wi-Fi access point and begins receiving telemetry data via TCP socket with **intelligent auto-reconnection** on connection loss.

---

## Recent Updates & Improvements

### Enhanced Connection Reliability (Latest)
- ✅ **Dual RSSI Display**: Shows both tracker→client and client→tracker signal strength for bidirectional diagnostics
- ✅ **Socket Health Verification**: Uses `getsockopt(SO_ERROR)` to verify TCP connection is actually alive, not just "connected"
- ✅ **Proactive TCP Reconnection**: Automatically re-establishes TCP connection immediately when WiFi reconnects
- ✅ **Stale Flag Detection**: Checks actual WiFi state vs. cached flags to prevent false disconnection states
- ✅ **Smart Error Display**: Only shows "Connection Lost" screen after first successful data reception (prevents startup false alarms)
- ✅ **Connection Verification**: Zero-byte send test after reconnect ensures socket is truly writable
- ✅ **Proper Event Bit Clearing**: Prevents stale connection states by clearing event bits before setting new ones

### Why These Changes Matter
**Before**: Display would sometimes show "Reconnecting..." even after WiFi came back, requiring manual reset.

**After**: Display automatically detects reconnection, verifies connection health, and resumes normal operation within 2-5 seconds.

**Key Improvement**: The dual RSSI display (`WiFi:-45dBm / -24dBm`) helps diagnose asymmetric link issues where one direction is weak.

---

## Features

### Startup Sequence
1. **Splash Screen**: Shows Sunflower logo and "System Starting..." message
2. **WiFi Connection**: "Connecting to WiFi..." with status
3. **Connection Status**: "Connected! Loading data..." 
4. **Dashboard**: Full real-time telemetry interface *(no false "Connection Lost" errors)*

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
│                  │                  │ Up:12h WiFi:-45dBm     │  ← Client's RSSI
│                  │                  │            /-24dBm     │  ← Tracker's RSSI
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

### WiFi Signal Strength Display (Dual RSSI)
Shows **bidirectional signal strength** in format: `WiFi:-45dBm / -24dBm`

**Format**: `Top Value / Bottom Value`
- **Top (-45 dBm)**: **Client's perspective** - How well the display sees the tracker
- **Bottom (-24 dBm)**: **Tracker's perspective** - How well the tracker sees the display

**Why show both?**
- **Asymmetric links**: One direction may be weaker than the other
- **Antenna issues**: Identifies which device has poor antenna placement
- **Real-time diagnostics**: See both perspectives simultaneously

**Interpretation Guide**:
| Display → Tracker | Tracker → Display | Diagnosis |
|-------------------|-------------------|-----------|
| Strong (-30 dBm)  | Strong (-30 dBm)  | ✅ Perfect - both sides see each other well |
| Weak (-70 dBm)    | Strong (-30 dBm)  | ⚠️ Display needs better position/antenna |
| Strong (-30 dBm)  | Weak (-70 dBm)    | ⚠️ Tracker needs better position/antenna |
| Weak (-70 dBm)    | Weak (-70 dBm)    | ❌ Too far apart or obstacles blocking signal |

**RSSI Scale**:
- `-30 to -50 dBm`: Excellent (5 bars)
- `-50 to -60 dBm`: Good (4 bars)
- `-60 to -70 dBm`: Fair (3 bars)
- `-70 to -80 dBm`: Weak (2 bars) - may drop occasionally
- `-80+ dBm`: Very weak (1 bar) - frequent disconnects expected

Both values update in real-time (client RSSI refreshes on every call, tracker RSSI comes from data packets).

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

### Auto-Reconnection (Enhanced)
- **WiFi disconnected**: Automatic reconnection every 5 seconds with reason logging
- **TCP disconnected**: Automatic reconnection every 2 seconds with socket health checks
- **Connection verification**: 
  - Socket existence check (`client_socket >= 0`)
  - Socket error state check via `getsockopt(SO_ERROR)`
  - Zero-byte send test to verify writability
- **Proactive reconnection**: TCP re-established immediately when WiFi reconnects (not just on next receive attempt)
- **Graceful degradation**: Shows "Reconnecting..." screen during outages
- **Smart error display**: Only shows error after `first_data_received` flag is set (10+ seconds of timeouts)
- **Event synchronization**: Proper clearing/setting of event group bits prevents stale states
- **Stale flag correction**: Checks `esp_wifi_sta_get_ap_info()` to verify actual WiFi state vs. cached flag

**Reconnection Flow**:
```
1. WiFi disconnect detected
   ├─ Close TCP socket immediately
   ├─ Clear CONNECTED event bit
   ├─ Set FAIL event bit
   └─ Attempt WiFi reconnect every 5 seconds

2. WiFi reconnect successful
   ├─ Get IP from DHCP
   ├─ Update RSSI
   ├─ Clear FAIL event bit
   ├─ Set CONNECTED event bit
   └─ Proactively reconnect TCP

3. TCP reconnection
   ├─ Create new socket
   ├─ Connect to 192.168.4.1:8888
   ├─ Verify with getsockopt(SO_ERROR)
   ├─ Test with zero-byte send
   └─ Resume data reception if all checks pass
```

**Recovery Time**:
- WiFi reconnect: 2-5 seconds (after initial 5s delay)
- TCP reconnect: <1 second (after WiFi is up)
- Socket verification: <100ms
- Total worst-case: ~6 seconds from disconnect to verified full recovery

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
| 3.3V/5V   | VCC         | Power (check display voltage) |
| GND       | GND         | Ground   |
| —         | SDO (MISO)  | Not connected |

**Notes**:
- Use short wires (<15cm) for stable SPI communication
- Most ILI9486 modules have onboard 5V regulator and accept 3.3V or 5V
- Backlight controlled via PWM (brightness adjustable 0-100%)
- If using 5V-only display, ensure ESP32 data lines are 5V tolerant (they are)

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
cd "C:\Users\YourUsername\Path\To\Sunflower_Secondary"

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

**Tip**: Use `idf.py -p COM3 flash monitor` to flash and monitor in one command.

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
#define SERVER_IP      "192.168.4.1"       // Master gateway IP (fixed AP address)
#define SERVER_PORT    8888                // TCP port
```

#### Display Settings
Customize colors and layout to fit your style.  
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
#define LCD_ORANGE      0xFD20  // Orange (warnings)
#define LCD_SUNGLOW     0xFFE0  // Yellow (sun position)
#define LCD_SLATE       0x7BEF  // Light gray (borders)
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
    int8_t   wifi_rssi;               // WiFi RSSI (tracker's view of client)
    uint8_t  wifi_clients;            // Number of connected clients
    uint8_t  sd_card_status;          // SD card status (0=none, 1=ok, 2=error)
    uint8_t  tracking_quality;        // Tracking quality/error metric (degrees or 0-100)
} __attribute__((packed)) tracker_data_t;
```

**Important**:
- `__attribute__((packed))` ensures no padding between fields
- Little-endian byte order (ESP32 native)
- Master must send exactly `sizeof(tracker_data_t)` bytes per frame (92 bytes)
- TCP stream, frames sent at ~1 Hz
- Client displays both tracker's RSSI (from `wifi_rssi` field) and its own calculated RSSI

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
├── sdkconfig                   # ESP-IDF configuration (generated)
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
   - Log RSSI and connection quality
4. **TCP Connection**:
   - Create TCP socket
   - Connect to 192.168.4.1:8888
   - Enable TCP_NODELAY and keepalive
   - **Verify connection health** (new: checks socket error state)
   - **Test writability** with zero-byte send (new)
5. **Data Reception**: Wait for first tracker packet (up to 20 seconds without error)
6. **Dashboard**: Switch to full telemetry display

### Main Loop (after initialization)
```c
bool first_data_received = false;
uint32_t no_data_counter = 0;

while (1) {
    // Check WiFi connection status (with stale flag detection)
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
        first_data_received = true;  // Mark first successful reception
    } 
    else if (ret == ESP_ERR_TIMEOUT) {
        // No data yet (normal during startup)
        no_data_counter++;
        
        // Only show error after first successful data AND 10+ timeouts
        if (no_data_counter >= 10 && first_data_received) {
            lcd_client_show_error("Waiting for tracker data");
        }
        // If first_data_received is false, keep showing init screen
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

### Auto-Reconnection Strategy (Enhanced)

**WiFi Disconnect Detection**: 
```c
// Event handler detects WIFI_EVENT_STA_DISCONNECTED
void wifi_event_handler(...) {
    case WIFI_EVENT_STA_DISCONNECTED:
        // Close TCP socket immediately
        if (client_socket >= 0) {
            close(client_socket);
            client_socket = -1;
        }
        
        // Update flags and event bits
        wifi_connected = false;
        tcp_connected = false;
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Log disconnect reason (auth fail, beacon timeout, etc.)
        ESP_LOGW(TAG, "Disconnected: reason %d", event->reason);
        
        // Attempt reconnection every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS));
        esp_wifi_connect();
        break;
}
```

**TCP Disconnect Detection**:
```c
// In wifi_client_receive_data()
int bytes_received = recv(client_socket, data, sizeof(tracker_data_t), 0);

if (bytes_received < 0) {
    // Error occurred
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return ESP_ERR_TIMEOUT;  // Normal timeout
    } else {
        // Connection error - close and reconnect
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return ESP_FAIL;
    }
} else if (bytes_received == 0) {
    // Graceful close by tracker
    close(client_socket);
    client_socket = -1;
    tcp_connected = false;
    return ESP_FAIL;
}
```

**Connection Health Verification** (New):
```c
bool wifi_client_is_connected(void) {
    // Check WiFi flag
    if (!wifi_connected) return false;
    
    // Check socket exists
    if (client_socket < 0) {
        tcp_connected = false;
        return false;
    }
    
    // Verify socket health using SO_ERROR
    int error = 0;
    socklen_t len = sizeof(error);
    if (getsockopt(client_socket, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
        // getsockopt failed, socket is dead
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return false;
    }
    
    if (error != 0) {
        // Socket has pending error
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return false;
    }
    
    return tcp_connected;
}
```

**Proactive TCP Reconnection** (New):
```c
// In WiFi event handler when IP is obtained
case IP_EVENT_STA_GOT_IP:
    wifi_connected = true;
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    
    // Proactively reconnect TCP immediately
    ESP_LOGI(TAG, "Establishing TCP connection...");
    if (connect_tcp() == ESP_OK) {
        // Verify with zero-byte send test
        int test = send(client_socket, NULL, 0, MSG_DONTWAIT);
        if (test < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "TCP verification failed");
            close(client_socket);
            client_socket = -1;
            tcp_connected = false;
        } else {
            ESP_LOGI(TAG, "✓ Full connection restored");
        }
    }
    break;
```

**Stale Flag Detection** (New):
```c
// In wifi_client_receive_data()
if (!wifi_connected) {
    // Check if we're actually connected but flag is stale
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "WiFi is actually connected, updating flag");
        wifi_connected = true;
    } else {
        return ESP_FAIL;
    }
}
```

**Recovery Time**:
- WiFi reconnect: 2-5 seconds (after initial 5s delay)
- TCP reconnect: <1 second (proactive, happens immediately when WiFi is up)
- Socket verification: <100ms
- Total worst-case: ~6 seconds from disconnect to verified full recovery

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
| Blank screen | No power / wrong wiring | Check 3.3V/5V supply, verify pin connections |
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
| Stuck at "Reconnecting" after WiFi restored | TCP not reconnecting | **Fixed**: TCP now reconnects proactively when WiFi is up |
| IP acquired but no data | TCP server not running | Verify tracker listening on port 8888 |
| Frequent disconnects | Weak signal / interference | Move units closer, check dual RSSI values on display |
| "Connection Lost" immediately after connect | Socket verification failing | Check tracker accepts connections, not at client limit |
| Display shows "Waiting for data" on startup | Normal behavior | **Fixed**: Error only shows after 20+ seconds without data |

### Data Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| All zeros displayed | `memset()` not overwritten | Check tracker is sending valid data |
| Corrupted values | Struct mismatch | Verify `tracker_data_t` identical on both sides (92 bytes) |
| Negative angles | Incorrect type interpretation | Ensure both sides use `float` for angles |
| No GPS coordinates | `gps_valid=0` | Normal without GPS fix, wait or check tracker GPS |
| Battery always 0% | ADC scaling issue | Check `battery_voltage` field, verify ADC code on tracker |
| Graph not updating | History buffer not appending | Check `voltage_history[]` writes in `draw_voltage_graph()` |
| Wrong WiFi RSSI | Using wrong value | **Fixed**: Top is client→tracker, bottom is tracker→client |
| Dual RSSI shows -128/-128 | Not connected | Both values update after WiFi connection established |
| Only one RSSI shows -128 | Asymmetric connection | Check antenna on side showing -128 |

### Connection Recovery Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Stays on error screen after reconnect | `first_data_received` not set | **Fixed**: Auto-clears after receiving first packet |
| Never reconnects | Event handler not called | Check WiFi stack initialization, event loop running |
| Reconnects but no data | TCP socket not reopening | **Fixed**: TCP reconnects proactively when WiFi is up |
| Client RSSI shows -128 dBm | `esp_wifi_sta_get_ap_info()` failing | **Fixed**: Always fetches fresh RSSI on every call |
| Tracker RSSI shows 0 dBm | Not in data packet | Tracker must populate `wifi_rssi` field in packet |
| Socket exists but can't receive | Dead connection not detected | **Fixed**: `getsockopt(SO_ERROR)` verifies socket health |

### Serial Monitor Output

**Expected logs during normal operation:**
```
I (320) LCD_MAIN: ╔════════════════════════════════════════════════════════════╗
I (320) LCD_MAIN: ║          SUNFLOWER LCD DISPLAY CLIENT                      ║
I (320) LCD_MAIN: ╚════════════════════════════════════════════════════════════╝
I (330) LCD_MAIN: Build: Dec 25 2024 14:30:15
I (450) LCD: ✓ LCD initialized
I (450) LCD: Display: ILI9486 480x320
I (500) WIFI_CLIENT: Connecting to tracker WiFi...
I (2100) WIFI_CLIENT: ╔════════════════════════════════════════════════════════════╗
I (2100) WIFI_CLIENT: ║          WIFI CONNECTED                                    ║
I (2100) WIFI_CLIENT: ╚════════════════════════════════════════════════════════════╝
I (2100) WIFI_CLIENT: ✓ Connected to tracker AP
I (2100) WIFI_CLIENT:   IP: 192.168.4.2
I (2100) WIFI_CLIENT:   Signal Strength: -45 dBm
I (2100) WIFI_CLIENT:   Quality: GOOD (4 bars)
I (2200) WIFI_CLIENT: Establishing TCP connection...
I (2250) WIFI_CLIENT: ✓ TCP connection verified and ready
I (2300) LCD_MAIN: ✓ Connected to tracker
I (2300) LCD_MAIN:   SSID: SunflowerTracker
I (2300) LCD_MAIN:   IP: 192.168.4.2
I (2300) LCD_MAIN:   Signal: -45 dBm
I (3400) LCD_MAIN: ╔════════════════════════════════════════════════════════════╗
I (3400) LCD_MAIN: ║          ENTERING MAIN DISPLAY LOOP                        ║
I (3400) LCD_MAIN: ╚════════════════════════════════════════════════════════════╝
I (13500) LCD_MAIN: ─────────────────────────────────────────────────────
I (13500) LCD_MAIN: Display Update #1
I (13500) LCD_MAIN: ─────────────────────────────────────────────────────
I (13500) LCD_MAIN: Panel Position:
I (13500) LCD_MAIN:   Elevation: 45.3° (Δ+2.5°)
I (13500) LCD_MAIN:   Azimuth:   180.5° (Δ+15.2°)
I (13500) LCD_MAIN: Battery:
I (13500) LCD_MAIN:   Voltage:   12.34V
I (13500) LCD_MAIN:   SoC:       80% (Level 4)
I (13500) LCD_MAIN:   Charging:  YES
I (13500) LCD_MAIN: GPS:
I (13500) LCD_MAIN:   Position:  43.653200, -79.383200
I (13500) LCD_MAIN:   Satellites: 12
I (13500) LCD_MAIN:   Valid:     YES
I (13500) LCD_MAIN:   Fix Age:   2 sec
I (13500) LCD_MAIN: WiFi RSSI (Client): -45 dBm
I (13500) LCD_MAIN: WiFi RSSI (Tracker): -24 dBm
```

**Expected logs during reconnection (NEW - Enhanced):**
```
W (45200) WIFI_CLIENT: ╔════════════════════════════════════════════════════════════╗
W (45200) WIFI_CLIENT: ║          WIFI DISCONNECTED                                 ║
W (45200) WIFI_CLIENT: ╚════════════════════════════════════════════════════════════╝
W (45200) WIFI_CLIENT: ⚠ Disconnected from AP
W (45200) WIFI_CLIENT:   Reason: 8 (Disassociated)
I (45200) WIFI_CLIENT:   Closing TCP socket
I (45200) WIFI_CLIENT: Reconnecting in 5 seconds... (attempt 1)
I (50300) WIFI_CLIENT: ╔════════════════════════════════════════════════════════════╗
I (50300) WIFI_CLIENT: ║          WIFI CONNECTED                                    ║
I (50300) WIFI_CLIENT: ╚════════════════════════════════════════════════════════════╝
I (50300) WIFI_CLIENT: ✓ Connected to tracker AP
I (50300) WIFI_CLIENT:   IP: 192.168.4.2
I (50350) WIFI_CLIENT: Establishing TCP connection...
I (50400) WIFI_CLIENT: ✓ TCP connection verified and ready
I (50500) WIFI_CLIENT: ✓ Full connection restored
I (50600) LCD_MAIN: ✓ Resuming dashboard updates
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
- [x] Socket health verification (prevents stale connections)
- [x] Proactive TCP reconnection (faster recovery)
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

// Check connection status (verifies socket health - ENHANCED)
bool wifi_client_is_connected(void);

// Get client's WiFi signal strength in dBm (always fresh - ENHANCED)
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

// Update dashboard with new data (incremental updates)
void lcd_client_display_dashboard(const tracker_data_t *data);

// Show error message overlay
void lcd_client_show_error(const char *message);
```

---

## Known Limitations

1. **No touch screen support**: Display is output-only, no interactive controls
2. **Fixed WiFi credentials**: Must recompile to change SSID/password (no runtime config)
3. **Single tracker support**: Cannot switch between multiple trackers
4. **No SD card logging**: Display does not store historical data locally
5. **Fixed update rate**: 1 Hz from tracker, 10 Hz display refresh
6. **No OTA updates**: Must reflash via USB for firmware updates
7. **ASCII-only display**: No graphical icons or antialiased fonts (by design for performance)

---

## License

Part of the ELEC 4020 Capstone Project - Sunflower Solar Tracker.  
Auburn University, Auburn, AL, USA.

---

## Authors

- **Display Client Development**: Abdul Mohammed (ESP32, WiFi, LCD driver, UI design, connection reliability)
- **System Architecture**: Abdul Mohammed

---

## Support

For issues or questions:
1. Check serial monitor logs for error messages (look for "WIFI_CLIENT" and "LCD_MAIN" tags)
2. Verify wiring matches pin assignments in hardware section
3. Test with known-good tracker unit
4. Review troubleshooting section above (includes recent fixes)
5. Check dual RSSI values on display for signal issues
6. Contact project team for hardware/integration issues

---

## References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ILI9486 Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ILI9486.pdf)
- [ESP32 WiFi Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- [FreeRTOS Task Management](https://www.freertos.org/a00106.html)
- [RGB565 Color Picker](http://www.barth-dev.de/online/rgb565-color-picker/)
- [SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)
- [POSIX Socket API](https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/sys_socket.h.html)
