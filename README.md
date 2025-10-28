# Sunflower Secondary (ESP32 LCD Display Client)

ESP-IDF project for the secondary display unit that connects to the master Sunflower tracker controller over Wi‑Fi and renders a real-time dashboard on a 3.5" ILI9486 TFT display (480×320, SPI).

---

## Overview

This client provides a visual interface for monitoring the solar tracker's status in real-time. It displays:
- **Solar panel orientation** (elevation and azimuth angles)
- **Tracking deltas** (rate of change in degrees/second)
- **Battery voltage** with rolling history graph
- **GPS coordinates** (when valid fix available)
- **System status** (tracking, standby, sleeping, calibrating, error)
- **Current time** (derived from tracker timestamp)

The display automatically connects to the master controller's Wi-Fi access point and begins receiving telemetry data via TCP socket.

---

## Features

### Startup Sequence
1. **Splash Screen**: Shows Sunflower logo and "System Starting..." message
2. **Initialization Checklist**: Visual progress with ✓/✗ for each subsystem:
   - LCD Display Driver
   - SPI Communication
   - WiFi Module
   - Network Connection
   - Tracker Link
   - Data Reception
3. **Dashboard**: Full real-time telemetry interface

### Dashboard Layout

```
┌─────────────────────────────────────────────────────────────┐
│ 🌻 SUNFLOWER                           12:34:56 PM          │  ← Header (dark gray)
├─────────────────────────────────────────────────────────────┤
│ ┌──────────┐ ┌──────────┐ ┌──────────┐                     │
│ │ELEVATION │ │ AZIMUTH  │ │ BATTERY  │                     │
│ │  45.2°   │ │  180.5°  │ │  13.2V   │                     │
│ │ D:0.015  │ │ D:0.023  │ │ ADC:2800 │                     │
│ │          │ │          │ │ [████░░] │                     │
│ └──────────┘ └──────────┘ └──────────┘                     │
│                                                              │
│ ┌──────────┐  GPS Location:                                │
│ │ STATUS   │  Lat: 43.6532°N                               │
│ │TRACKING  │  Lon: 79.3832°W                               │
│ │  (green) │                                                │
│ └──────────┘  System v1.0                                  │
├─────────────────────────────────────────────────────────────┤
│ Battery Voltage History (Last 100 Readings)                │
│ 14.5V ┤                                    ╱──╲             │
│ 13.5V ┤                           ╱───────╯    ╲           │
│ 12.5V ┤                  ╱───────╯              ╲──        │
│ 11.5V ┤         ╱───────╯                          ●       │
│ 10.5V └────────────────────────────────────────────────────┤
│       └─ time ─────────────────────────────────────────────→│
└─────────────────────────────────────────────────────────────┘
```

### Color-Coded Status Panel
- **🟢 GREEN** (TRACKING): Actively following the sun
- **🟠 AMBER** (STANDBY/SLEEPING/CALIBRATING): Idle or transitioning states
- **🔴 RED** (ERROR): Connection lost or critical fault

### Battery Graph
- Displays rolling 100-sample ADC history
- Auto-scales Y-axis to min/max range
- Current reading highlighted with marker dot
- Grid lines for easier reading

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
| 3.3V      | VCC         | Power    |
| GND       | GND         | Ground   |
| —         | SDO (MISO)  | Not connected |

**Notes**:
- Use short wires (<15cm) for stable SPI communication
- Display should have onboard 3.3V regulator (most ILI9486 modules do)
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
cd "C:\Users\moham\OneDrive\Desktop\Elec 4020\Sunflower_Secondary"

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
#define WIFI_PASS      "sunflower2025"     // Password
#define SERVER_IP      "192.168.4.1"       // Master gateway IP
#define SERVER_PORT    8888                // TCP port
```

#### Display Settings
Edit `components/lcd/include/lcd.h`:
```c
#define TFT_WIDTH   480
#define TFT_HEIGHT  320

// Adjust colors if needed
#define TFT_SAGE        0x07E0  // Tracking status (green)
#define TFT_AMBER       0xFD20  // Standby status (orange)
#define TFT_CRIMSON     0xF800  // Error status (red)
```

#### SPI Speed (if display glitches)
Edit `components/lcd/lcd.c` line ~450:
```c
.clock_speed_hz = 10 * 1000 * 1000,  // Lower to 5MHz if needed
```

---

## Data Protocol

The client expects binary frames matching this exact structure (must align with master):

```c
typedef struct {
    float   elevation;        // degrees (0..90)
    float   azimuth;          // degrees (0..360)
    float   delta_elevation;  // degrees/sec
    float   delta_azimuth;    // degrees/sec
    uint16_t battery_adc;     // raw ADC (0..4095)
    float    battery_voltage; // volts (scaled by master)
    uint32_t timestamp;       // seconds (UTC or uptime)
    uint8_t  status;          // 0=standby,1=tracking,2=sleep,3=calib,255=error
    float   latitude;         // degrees (-90..+90)
    float   longitude;        // degrees (-180..+180)
    uint8_t gps_valid;        // 0 = no fix, 1 = valid
} tracker_data_t;
```

**Important**:
- Little-endian byte order (ESP32 native)
- No padding between fields (or use `#pragma pack(1)` on both sides)
- Master must send exactly `sizeof(tracker_data_t)` = 45 bytes per frame
- TCP stream, frames sent ~1 Hz (adjustable)

### Status Code Mapping
| Value | State       | Display Color |
|-------|-------------|---------------|
| 0     | Standby     | 🟠 Amber      |
| 1     | Tracking    | 🟢 Green      |
| 2     | Sleeping    | 🟠 Amber      |
| 3     | Calibrating | 🟠 Amber      |
| 255   | Error       | 🔴 Red        |

---

## Project Structure

```
Sunflower_Secondary/
├── CMakeLists.txt              # Top-level build config
├── README.md                   # This file
├── sdkconfig                   # ESP-IDF configuration
├── main/
│   ├── CMakeLists.txt
│   └── main.c                  # Entry point, init flow, update loop
└── components/
    ├── lcd/                    # Display driver & UI renderer
    │   ├── CMakeLists.txt
    │   ├── lcd.c               # SPI driver, drawing primitives, dashboard
    │   └── include/
    │       └── lcd.h           # Public API, color palette, data struct
    ├── sunflower_logo/         # Logo bitmap (24×24 RGB565)
    │   ├── CMakeLists.txt
    │   └── include/
    │       └── sunflower_logo.h
    └── wifi_client/            # Wi-Fi STA + TCP client
        ├── CMakeLists.txt
        ├── wifi_client.c       # Connection logic, receive handler
        └── include/
            └── wifi_client.h   # tracker_data_t definition
```

---

## Runtime Behavior

### Startup Flow
1. **NVS Flash Init**: Prepare non-volatile storage
2. **LCD Init**: Configure SPI, reset display, apply settings
3. **Splash Screen**: Show logo + "System Starting..."
4. **Initialization Checklist**:
   - ✓ LCD Display Driver
   - ✓ SPI Communication
   - ✓ WiFi Module
   - ⏳ Network Connection (blocks until AP joined + IP obtained)
   - ⏳ Tracker Link (waits for TCP handshake)
   - ⏳ Data Reception (waits for first valid frame)
5. **Dashboard Draw**: Full initial render with default/first received values
6. **Update Loop**: Incremental updates every ~100ms (10 Hz)

### Update Loop
```c
while (1) {
    // Receive data from master (timeout=2000ms)
    ret = wifi_client_receive_data(&rx_data, 2000);
    
    if (ret == ESP_OK) {
        // Map to display structure
        // Update panels (time, angles, battery, status, graph)
        lcd_update_display(&display_data);
    } else if (timeout > 10s) {
        // Show "Connection Lost" banner
        // Set status to ERROR (red)
    }
    
    vTaskDelay(100ms);
}
```

### Error Recovery
- **No data for 10s**: Warning banner + continue retrying
- **No data for 30s**: Status panel turns red (ERROR state)
- **WiFi disconnect**: Auto-reconnect + close/reopen socket
- **Socket error**: Log errno, close socket, UI shows error

---

## Troubleshooting

### Display Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Blank screen | No power / wrong wiring | Check 3.3V supply, verify pin connections |
| White screen | Panel not initialized | Check RST pin, try lowering SPI speed |
| Garbled graphics | SPI too fast / loose wires | Lower clock to 5MHz, shorten wires |
| Partial display | Incorrect rotation | Verify MADCTL command in `lcd.c` |

### Wi-Fi Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Cannot join AP | Wrong SSID/password | Check credentials in `wifi_client.c` |
| IP acquired but no data | Master not running | Start master tracker first |
| Frequent disconnects | Weak signal / interference | Move units closer, check master AP config |
| Stuck at init screen | TCP port blocked | Verify master listening on port 8888 |

### Data Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Corrupted values | Struct mismatch | Verify `tracker_data_t` matches on both sides |
| Negative angles | Byte order mismatch | Both ESP32 = little-endian, check packing |
| No GPS shown | `gps_valid=0` always | Check master GPS module, wait for fix |
| Battery bar wrong | ADC scaling off | Adjust mapping in `draw_battery_panel()` |

### Serial Monitor Output

Expected logs:
```
I (320) LCD_DISPLAY: === SUNFLOWER LCD DISPLAY ===
I (330) LCD: Initializing LCD display...
I (450) LCD: Display initialized
I (500) WIFI_CLIENT: Connecting to tracker...
I (2100) WIFI_CLIENT: Connected! IP: 192.168.4.2
I (2200) LCD_DISPLAY: Update #10 - El:45.2° Az:180.5° Batt:13.20V Status:1
```

---

## Performance Notes

- **SPI Speed**: 10 MHz (can push to 26 MHz with short wires)
- **Frame Rate**: ~10 FPS for full dashboard updates
- **Latency**: <200ms from master send to display update
- **Memory**: ~40 KB heap used (primarily line buffers + history arrays)

### Optimization TODOs
- [ ] Incremental battery graph (only draw new segment vs full redraw)
- [ ] Static line buffers (avoid malloc per `lcd_fill_rect`)
- [ ] DMA-capable buffers for larger blits
- [ ] Dirty-rect tracking for selective panel updates
- [ ] Add larger/antialiased fonts

---

## License

Part of the Capstone Sunflower Tracker project.

---

## Authors

- Display Client: [Your Name]
- Master Controller: [Team Member]
- Hardware Integration: [Team Member]

---

## References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ILI9486 Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ILI9486.pdf)
- [RGB565 Color Picker](http://www.barth-dev.de/online/rgb565-color-picker/)
