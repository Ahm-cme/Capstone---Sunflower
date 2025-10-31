/*
 * WiFi Communication Module (Access Point + TCP server)
 *
 * Architecture Overview:
 *  Main Tracker (ESP32-CAM):
 *   └─ WiFi AP "SunflowerTracker" + TCP server on port 8888
 *      └─ Broadcasts tracking data at 1 Hz (92 bytes/packet)
 *  
 *  LCD Display (ESP32-WROOM):
 *   └─ WiFi Station connects to tracker AP
 *      └─ TCP client receives and displays data
 *
 * Design Decisions:
 *  - Unidirectional: Tracker → Display (display is passive observer)
 *  - TCP not UDP: Reliable delivery, connection awareness
 *  - Binary protocol: Compact, efficient (vs JSON/text)
 *  - Non-blocking sockets: Never stalls main tracking loop
 *  - Stateless send: Auto-recovers from disconnects
 *
 * Network Topology:
 *  Tracker (AP):     192.168.4.1 (fixed)
 *  Display (STA):    192.168.4.x (DHCP)
 *  Subnet:           192.168.4.0/24
 *  Gateway:          192.168.4.1 (tracker)
 *  DNS:              None (no internet needed)
 *
 * Protocol:
 *  - Transport: TCP on port 8888
 *  - Packet size: 92 bytes (tracker_data_t struct)
 *  - Rate: 1 Hz (controlled by caller)
 *  - Encoding: Raw binary (struct memcpy)
 *  - Endianness: Little-endian (ESP32 native)
 *
 * Performance Optimizations:
 *  1. Maximum TX power (19.5 dBm) - extends range 50-100m
 *  2. Power-saving disabled - eliminates latency, instant response
 *  3. TCP_NODELAY enabled - no Nagle delay, immediate send
 *  4. TCP keepalive (5s/2s/3) - detects dead connections in 11s
 *  5. Large send buffer (8KB) - prevents packet loss during bursts
 *  6. 20MHz bandwidth - better range/stability than 40MHz
 *  7. Channel 1 (2.4GHz) - least interference in most environments
 *
 * Fault Handling:
 *  - Display disconnect: Tracker continues independently
 *  - Display reconnect: Automatic, seamless resume
 *  - Send failure: Close socket, retry next call
 *  - No data loss: SD logging independent of WiFi
 *
 * Power Consumption:
 *  - WiFi active: +100-150mA continuous
 *  - Deep sleep: 0mA (WiFi off)
 *  - Total with WiFi: 150-300mA active
 *  - Consider: Disable WiFi for battery-only operation
 *
 * Troubleshooting Guide:
 *  Display can't see SSID:
 *   - Check channel 1 isn't jammed (try channel 6 or 11)
 *   - Verify "SunflowerTracker" SSID not hidden
 *   - Check tracker WiFi init succeeded (console logs)
 *
 *  Display connects but no data:
 *   - Verify TCP port 8888 in display code
 *   - Check wifi_comm_send_data() being called at 1Hz
 *   - Monitor console for send errors
 *
 *  Connection drops frequently:
 *   - Reduce distance between tracker and display
 *   - Check battery voltage (low voltage = weak WiFi)
 *   - Add external antenna if possible
 *   - Check for interference (microwave, Bluetooth)
 *
 *  Data corruption:
 *   - Verify tracker_data_t struct IDENTICAL on both ESP32s
 *   - Check packing (__attribute__((packed)))
 *   - Verify little-endian on both (ESP32 default)
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#include "wifi_comm.h"

// WiFi credentials and configuration
#define WIFI_SSID      "SunflowerTracker"
#define WIFI_PASS      "sunflower2025"
#define WIFI_CHANNEL   1           // Channel 1: least interference, best compatibility
#define MAX_STA_CONN   2           // Allow up to 2 displays simultaneously

// TCP server configuration
#define SERVER_PORT    8888        // Standard port for this application

static const char *TAG = "WIFI_COMM";

// Module state
static int server_socket = -1;          // Listening socket
static int client_socket = -1;          // Active client connection
static bool is_connected = false;       // Station associated with AP

// Statistics tracking
static wifi_stats_t s_stats = {0};
static uint32_t s_wifi_start_time = 0;

/*
 * WiFi AP Event Handler
 * 
 * Handles station association/disassociation events.
 * Updates connection state and closes sockets on disconnect.
 * 
 * Events handled:
 *  - WIFI_EVENT_AP_STACONNECTED: Display connected to AP
 *  - WIFI_EVENT_AP_STADISCONNECTED: Display left AP
 * 
 * On disconnect:
 *  - Closes TCP client socket
 *  - Clears connection flag
 *  - Next send() will accept new connection
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║          LCD DISPLAY CONNECTED                             ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✓ Station associated with AP");
        ESP_LOGI(TAG, "  MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        ESP_LOGI(TAG, "  AID: %u", event->aid);
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Display will receive tracking data at 1 Hz");
        ESP_LOGI(TAG, "");
        
        is_connected = true;
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║          LCD DISPLAY DISCONNECTED                          ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "⚠ Station disassociated from AP");
        ESP_LOGI(TAG, "  MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Tracker continues logging to SD card");
        ESP_LOGI(TAG, "Display can reconnect anytime");
        ESP_LOGI(TAG, "");
        
        is_connected = false;

        // Close TCP socket on disconnect
        if (client_socket >= 0) {
            close(client_socket);
            client_socket = -1;
            ESP_LOGD(TAG, "TCP socket closed");
        }
    }
}

/*
 * Initialize WiFi Access Point and TCP Server
 * 
 * Complete initialization sequence:
 *  1. Initialize ESP-NETIF and event loop
 *  2. Create default WiFi AP interface
 *  3. Initialize WiFi stack with default config
 *  4. Register event handlers
 *  5. Configure AP parameters (SSID, password, channel)
 *  6. Apply performance optimizations (power, protocols, bandwidth)
 *  7. Start WiFi
 *  8. Create and configure TCP server socket
 *  9. Bind to port 8888 and start listening
 * 
 * Returns:
 *  ESP_OK   - WiFi AP running, TCP server listening
 *  ESP_FAIL - Initialization failed (check console for details)
 * 
 * On failure:
 *  - System continues without WiFi
 *  - Display will not be available
 *  - Data logging to SD card unaffected
 */
esp_err_t wifi_comm_init_ap(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          WIFI ACCESS POINT INITIALIZATION                  ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    // === Initialize Network Stack ===
    ESP_LOGI(TAG, "Initializing network stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    ESP_LOGD(TAG, "✓ ESP-NETIF initialized");

    // === Initialize WiFi Stack ===
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGD(TAG, "✓ WiFi stack initialized");

    // === Register Event Handlers ===
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_LOGD(TAG, "✓ Event handlers registered");

    // === Configure AP Parameters ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Access Point Configuration:");
    ESP_LOGI(TAG, "  SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "  Password: %s", WIFI_PASS);
    ESP_LOGI(TAG, "  Channel: %d (2.4 GHz)", WIFI_CHANNEL);
    ESP_LOGI(TAG, "  Max stations: %d", MAX_STA_CONN);
    ESP_LOGI(TAG, "  Auth mode: WPA2-PSK");
    ESP_LOGI(TAG, "  IP address: 192.168.4.1 (default)");
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false            // Disable PMF for compatibility
            },
            .ssid_hidden = 0,                // Broadcast SSID
            .beacon_interval = 100,          // Standard 100ms
            .ftm_responder = false,          // Disable FTM
        },
    };

    // Validate password length
    if (strlen(WIFI_PASS) < 8) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ Password too short: %zu chars (WPA2 requires ≥8)", strlen(WIFI_PASS));
        ESP_LOGE(TAG, "  Update WIFI_PASS in source code");
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_LOGD(TAG, "✓ AP configuration set");
    
    // === Start WiFi FIRST ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Starting WiFi AP...");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "✓ WiFi AP active and broadcasting");
    
    s_wifi_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;

    // ═══════════════════════════════════════════════════════════════
    // PERFORMANCE OPTIMIZATIONS FOR MAXIMUM RANGE & RELIABILITY
    // ═══════════════════════════════════════════════════════════════
    // NOTE: Must be called AFTER esp_wifi_start()
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Applying performance optimizations...");
    
    // 1. Maximum WiFi TX Power (19.5 dBm)
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));  // 78 = 19.5 dBm (0.25 dBm per unit)
    ESP_LOGI(TAG, "  ✓ TX Power: 19.5 dBm (MAXIMUM)");
    ESP_LOGI(TAG, "    - Extends range to 50-100m line-of-sight");
    ESP_LOGI(TAG, "    - Adds ~50mA current draw");
    
    // 2. Disable Power Saving (Always-On WiFi)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "  ✓ Power Saving: DISABLED");
    ESP_LOGI(TAG, "    - WiFi always active (no sleep)");
    ESP_LOGI(TAG, "    - Zero latency for instant response");
    ESP_LOGI(TAG, "    - Adds ~100mA continuous draw");
    
    // 3. Enable All WiFi Protocols (802.11b/g/n)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, 
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_LOGI(TAG, "  ✓ Protocols: 802.11b/g/n");
    ESP_LOGI(TAG, "    - Maximum compatibility with all displays");
    ESP_LOGI(TAG, "    - Auto-negotiates best mode");
    
    // 4. Set 20MHz Bandwidth (Stable Mode)
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_LOGI(TAG, "  ✓ Bandwidth: 20 MHz");
    ESP_LOGI(TAG, "    - Better range than 40MHz");
    ESP_LOGI(TAG, "    - More stable in noisy environments");
    ESP_LOGI(TAG, "    - Sufficient for 1Hz @ 92 bytes/s (~1 kbps)");
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Performance tuning complete");
    ESP_LOGI(TAG, "  Range: 50-100m line-of-sight");
    ESP_LOGI(TAG, "  Power: +150mA WiFi overhead");
    ESP_LOGI(TAG, "  Latency: <10ms typical");
    
    // ═══════════════════════════════════════════════════════════════
    // TCP SERVER SETUP
    // ═══════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuring TCP server...");
    
    server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ Socket creation failed: errno %d (%s)", errno, strerror(errno));
        ESP_LOGE(TAG, "  WiFi AP running but TCP unavailable");
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "✓ Socket created (fd=%d)", server_socket);

    // Enable socket address reuse (helps with quick reconnects)
    int enable = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
    ESP_LOGD(TAG, "✓ SO_REUSEADDR enabled");
    
    // TCP Keepalive: Detect dead connections faster
    // - Start probing after 5 seconds idle
    // - Send probe every 2 seconds
    // - Give up after 3 failed probes
    // - Total timeout: 5 + (2 × 3) = 11 seconds
    int keepalive = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(int));
    
    int keepidle = 5;
    int keepintvl = 2;
    int keepcnt = 3;
    setsockopt(server_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(int));
    setsockopt(server_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(int));
    setsockopt(server_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(int));
    ESP_LOGI(TAG, "  ✓ TCP Keepalive: 5s idle, 2s interval, 3 probes");
    ESP_LOGI(TAG, "    - Detects dead connections in 11 seconds");
    
    // Disable Nagle's algorithm (TCP_NODELAY)
    // - Sends packets immediately without waiting for ACK
    // - Reduces latency from ~200ms to <10ms
    // - Critical for real-time tracking display
    int nodelay = 1;
    setsockopt(server_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));
    ESP_LOGI(TAG, "  ✓ TCP_NODELAY: ENABLED");
    ESP_LOGI(TAG, "    - No Nagle delay, immediate send");
    ESP_LOGI(TAG, "    - Latency: <10ms typical");
    
    // Increase send buffer size (default ~5KB is often too small)
    int sendbuf = 8192;
    setsockopt(server_socket, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(int));
    ESP_LOGI(TAG, "  ✓ Send Buffer: 8 KB");
    ESP_LOGI(TAG, "    - Prevents packet loss during bursts");

    // Set non-blocking mode (never stall main loop)
    int flags = fcntl(server_socket, F_GETFL, 0);
    fcntl(server_socket, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGD(TAG, "✓ Non-blocking mode enabled");

    // Bind to all interfaces on port 8888
    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ Bind failed: errno %d (%s)", errno, strerror(errno));
        ESP_LOGE(TAG, "  Port %d may be in use", SERVER_PORT);
        ESP_LOGE(TAG, "");
        close(server_socket);
        server_socket = -1;
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "✓ Bound to 0.0.0.0:%d", SERVER_PORT);

    // Start listening (queue size = 1, only one display at a time)
    if (listen(server_socket, 1) != 0) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ Listen failed: errno %d (%s)", errno, strerror(errno));
        ESP_LOGE(TAG, "");
        close(server_socket);
        server_socket = -1;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ TCP server listening on port %d", SERVER_PORT);
    ESP_LOGI(TAG, "  Ready to accept connections");

    // === Initialization Complete ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          WIFI SYSTEM READY                                 ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Summary:");
    ESP_LOGI(TAG, "  - WiFi AP broadcasting SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "  - TCP server listening on port: %d", SERVER_PORT);
    ESP_LOGI(TAG, "  - Optimized for maximum range and reliability");
    ESP_LOGI(TAG, "  - Display can connect anytime");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Display Connection Instructions:");
    ESP_LOGI(TAG, "  1. Flash display ESP32 with Sunflower_Secondary firmware");
    ESP_LOGI(TAG, "  2. Power on display");
    ESP_LOGI(TAG, "  3. Display auto-connects to 'SunflowerTracker'");
    ESP_LOGI(TAG, "  4. Dashboard shows real-time tracking data");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

/*
 * Send Tracker Data to Connected Display
 * 
 * Transmits one tracker_data_t packet (92 bytes) via TCP.
 * Call from main loop at 1 Hz.
 * 
 * Behavior flow:
 *  1. Check if WiFi station associated
 *  2. If no TCP client, try non-blocking accept()
 *  3. If client connected, send packet with 2s timeout
 *  4. On error, close socket and return failure
 *  5. Socket auto-reconnects on next call
 * 
 * Returns:
 *  ESP_OK                - Packet sent successfully
 *  ESP_ERR_INVALID_STATE - No WiFi station associated
 *  ESP_ERR_NOT_FOUND     - No TCP client connected (waiting)
 *  ESP_FAIL              - Send failed, socket closed
 */
esp_err_t wifi_comm_send_data(const tracker_data_t *data)
{
    // Check WiFi association first
    if (!is_connected) {
        return ESP_ERR_INVALID_STATE;
    }

    // Try to accept new client if none connected
    if (client_socket < 0) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &addr_len);
        
        if (client_socket >= 0) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "✓ TCP client connected");
            ESP_LOGI(TAG, "  IP: %s", inet_ntoa(client_addr.sin_addr));
            ESP_LOGI(TAG, "  Port: %u", ntohs(client_addr.sin_port));
            ESP_LOGI(TAG, "");
            
            // Apply same TCP optimizations to client socket
            int nodelay = 1;
            setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));
            
            int keepalive = 1;
            setsockopt(client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(int));
            
            int keepidle = 5;
            int keepintvl = 2;
            int keepcnt = 3;
            setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(int));
            setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(int));
            setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(int));
            
            int sendbuf = 8192;
            setsockopt(client_socket, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(int));
            
            ESP_LOGD(TAG, "Client socket optimizations applied");
        } else {
            // No client yet (EAGAIN/EWOULDBLOCK expected in non-blocking mode)
            return ESP_ERR_NOT_FOUND;
        }
    }

    // Set send timeout (prevents blocking if client stalls)
    struct timeval timeout;
    timeout.tv_sec = 2;   // 2 second timeout
    timeout.tv_usec = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // Send packet
    int bytes_sent = send(client_socket, data, sizeof(tracker_data_t), 0);
    
    if (bytes_sent < 0) {
        // Send failed - log error and close socket
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "⚠ TCP send failed: errno %d (%s)", errno, strerror(errno));
        ESP_LOGE(TAG, "  Closing socket, will retry on next call");
        ESP_LOGE(TAG, "");
        
        close(client_socket);
        client_socket = -1;
        s_stats.tx_errors++;
        return ESP_FAIL;
    }
    
    if (bytes_sent != sizeof(tracker_data_t)) {
        // Partial send (shouldn't happen with TCP, but check anyway)
        ESP_LOGW(TAG, "⚠ Partial send: %d/%zu bytes", bytes_sent, sizeof(tracker_data_t));
    }

    // Success
    s_stats.tx_packets++;
    ESP_LOGV(TAG, "Sent %d bytes to display", bytes_sent);
    
    return ESP_OK;
}

/*
 * Check if Display is Connected
 * 
 * Returns true if:
 *  - WiFi station is associated with AP
 *  - TCP socket is open to client
 */
bool wifi_comm_is_connected(void)
{
    return is_connected && (client_socket >= 0);
}

/*
 * Get Number of Connected WiFi Clients
 */
uint8_t wifi_comm_get_client_count(void)
{
    wifi_sta_list_t sta_list = {0};
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
        return (uint8_t)sta_list.num;
    }
    return 0;
}

/*
 * Get WiFi Statistics
 */
void wifi_comm_get_stats(wifi_stats_t *stats)
{
    if (!stats) return;
    
    stats->tx_packets = s_stats.tx_packets;
    stats->rx_packets = s_stats.rx_packets;
    stats->tx_errors = s_stats.tx_errors;
    stats->rx_errors = s_stats.rx_errors;
    
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    stats->uptime_sec = now - s_wifi_start_time;
    
    stats->avg_rssi = -128;  // Not available in AP mode
}