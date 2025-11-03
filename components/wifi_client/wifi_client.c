/*
 * WiFi Communication Module - Client Implementation (LCD Display Side)
 *
 * Architecture Overview:
 *  LCD Display (ESP32-WROOM):
 *   └─ WiFi Station connects to "SunflowerTracker" AP
 *      └─ TCP client receives tracking data from 192.168.4.1:8888
 *         └─ Updates LCD display at ~1 Hz
 *
 * Connection Strategy:
 *  - Always-on WiFi (no power saving)
 *  - Auto-reconnect on disconnect (infinite retries)
 *  - Fast reconnect (2-5 second intervals)
 *  - Graceful degradation (display shows "Connecting..." on loss)
 *
 * Network Flow:
 *  1. Scan for "SunflowerTracker" SSID
 *  2. Connect with WPA2-PSK authentication
 *  3. Wait for DHCP IP (192.168.4.x)
 *  4. Connect TCP to 192.168.4.1:8888
 *  5. Receive 92-byte packets at ~1 Hz
 *  6. Parse and forward to LCD display
 *  7. On disconnect: Close socket → retry connection
 *
 * Performance Optimizations:
 *  1. Maximum TX power (19.5 dBm) - extends range
 *  2. Power-saving disabled - zero latency
 *  3. TCP_NODELAY - immediate receive
 *  4. Large RX buffer (8KB) - prevents packet loss
 *  5. 20MHz bandwidth - stable, better range
 *
 * Error Recovery:
 *  - WiFi disconnect: Reconnect every 5 seconds
 *  - TCP disconnect: Reconnect every 2 seconds
 *  - Receive timeout: Normal (display "Waiting...")
 *  - Invalid packet: Log warning, request retransmit
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#include "wifi_client.h"

// WiFi credentials (must match tracker)
#define WIFI_SSID      "SunflowerTracker"
#define WIFI_PASS      "sunflower2025"

// TCP server address (tracker's fixed IP)
#define SERVER_IP      "192.168.4.1"
#define SERVER_PORT    8888

// Connection retry intervals
#define WIFI_RECONNECT_DELAY_MS   5000   // 5 seconds between WiFi reconnects
#define TCP_RECONNECT_DELAY_MS    2000   // 2 seconds between TCP reconnects

// Event bits for WiFi connection
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1

static const char *TAG = "WIFI_CLIENT";

// Module state
static int client_socket = -1;                    // TCP client socket
static bool wifi_connected = false;               // WiFi station connected
static bool tcp_connected = false;                // TCP session active
static EventGroupHandle_t wifi_event_group;       // WiFi event synchronization
static int retry_count = 0;                       // Connection retry counter

// Statistics tracking
static wifi_client_stats_t s_stats = {0};
static uint32_t s_connection_start_time = 0;
static int8_t s_rssi = -128;                      // Current RSSI
static char s_ip_address[16] = "0.0.0.0";         // Current IP address

// Forward declaration
static esp_err_t connect_tcp(void);

/*
 * WiFi Event Handler (Station Mode)
 * 
 * Handles connection, disconnection, IP assignment events.
 * Sets event bits for synchronization with main task.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, connecting...");
        esp_wifi_connect();
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGW(TAG, "║          WIFI DISCONNECTED                                 ║");
        ESP_LOGW(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "⚠ Disconnected from AP");
        ESP_LOGW(TAG, "  Reason: %d", event->reason);
        
        const char *reason_str = "Unknown";
        switch (event->reason) {
            case WIFI_REASON_AUTH_EXPIRE:          reason_str = "Authentication expired"; break;
            case WIFI_REASON_AUTH_LEAVE:           reason_str = "Deauthenticated"; break;
            case WIFI_REASON_ASSOC_LEAVE:          reason_str = "Disassociated"; break;
            case WIFI_REASON_ASSOC_EXPIRE:         reason_str = "Association expired"; break;
            case WIFI_REASON_NOT_AUTHED:           reason_str = "Not authenticated"; break;
            case WIFI_REASON_NOT_ASSOCED:          reason_str = "Not associated"; break;
            case WIFI_REASON_ASSOC_TOOMANY:        reason_str = "Too many stations"; break;
            case WIFI_REASON_HANDSHAKE_TIMEOUT:    reason_str = "4-way handshake timeout"; break;
            case WIFI_REASON_BEACON_TIMEOUT:       reason_str = "Beacon timeout"; break;
            case WIFI_REASON_NO_AP_FOUND:          reason_str = "AP not found"; break;
            case WIFI_REASON_AUTH_FAIL:            reason_str = "Authentication failed"; break;
            case WIFI_REASON_CONNECTION_FAIL:      reason_str = "Connection failed"; break;
        }
        ESP_LOGW(TAG, "  Details: %s", reason_str);
        ESP_LOGW(TAG, "");
        
        wifi_connected = false;
        tcp_connected = false;
        strcpy(s_ip_address, "0.0.0.0");
        s_rssi = -128;
        
        // Close TCP socket if open
        if (client_socket >= 0) {
            close(client_socket);
            client_socket = -1;
            ESP_LOGD(TAG, "TCP socket closed");
        }
        
        // Clear CONNECTED bit before retrying
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Retry connection
        if (retry_count < 100) {  // Log for first 100 retries
            ESP_LOGI(TAG, "Reconnecting in %d seconds... (attempt %d)",
                     WIFI_RECONNECT_DELAY_MS / 1000, retry_count + 1);
        }
        retry_count++;
        s_stats.reconnect_count++;
        
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS));
        esp_wifi_connect();
        
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║          WIFI CONNECTED                                    ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✓ Connected to tracker AP");
        ESP_LOGI(TAG, "  SSID: %s", WIFI_SSID);
        ESP_LOGI(TAG, "  IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "  Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "  Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "");
        
        // Store IP address
        snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&event->ip_info.ip));
        
        // Get RSSI
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_rssi = ap_info.rssi;
            ESP_LOGI(TAG, "Signal Strength: %d dBm", s_rssi);
            
            if (s_rssi > -60) {
                ESP_LOGI(TAG, "  Quality: EXCELLENT (5 bars)");
            } else if (s_rssi > -70) {
                ESP_LOGI(TAG, "  Quality: GOOD (4 bars)");
            } else if (s_rssi > -80) {
                ESP_LOGI(TAG, "  Quality: FAIR (3 bars)");
            } else if (s_rssi > -90) {
                ESP_LOGW(TAG, "  Quality: WEAK (2 bars) - consider moving closer");
            } else {
                ESP_LOGW(TAG, "  Quality: VERY WEAK (1 bar) - connection may drop");
            }
        }
        ESP_LOGI(TAG, "");
        
        wifi_connected = true;
        retry_count = 0;  // Reset retry counter on success
        s_connection_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        
        // Clear FAIL bit and set CONNECTED bit
        xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Proactively reconnect TCP
        ESP_LOGI(TAG, "Establishing TCP connection...");
        if (connect_tcp() == ESP_OK) {
            ESP_LOGI(TAG, "✓ Full connection restored");
        } else {
            ESP_LOGW(TAG, "⚠ TCP reconnection failed, will retry in receive loop");
        }
    }
}

/*
 * Connect TCP Socket to Tracker
 * 
 * Creates TCP client socket and connects to tracker's server.
 * Called after WiFi connection established.
 */
static esp_err_t connect_tcp(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Connecting to TCP server...");
    ESP_LOGI(TAG, "  Server: %s:%d", SERVER_IP, SERVER_PORT);
    
    // Create socket
    client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (client_socket < 0) {
        ESP_LOGE(TAG, "✗ Socket creation failed: errno %d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "✓ Socket created (fd=%d)", client_socket);
    
    // Configure server address
    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);
    
    // Apply TCP optimizations
    int nodelay = 1;
    setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));
    ESP_LOGD(TAG, "✓ TCP_NODELAY enabled");
    
    int keepalive = 1;
    setsockopt(client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(int));
    
    int keepidle = 5;
    int keepintvl = 2;
    int keepcnt = 3;
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(int));
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(int));
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(int));
    ESP_LOGD(TAG, "✓ TCP keepalive enabled (5s/2s/3)");
    
    int recvbuf = 8192;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVBUF, &recvbuf, sizeof(int));
    ESP_LOGD(TAG, "✓ Receive buffer: 8 KB");
    
    // Connect to server
    if (connect(client_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ TCP connect failed: errno %d (%s)", errno, strerror(errno));
        ESP_LOGE(TAG, "  Check tracker is running and reachable");
        ESP_LOGE(TAG, "  Verify server IP: %s", SERVER_IP);
        ESP_LOGE(TAG, "");
        close(client_socket);
        client_socket = -1;
        return ESP_FAIL;
    }
    
    tcp_connected = true;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ TCP connected to tracker");
    ESP_LOGI(TAG, "  Ready to receive tracking data");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

/*
 * Initialize WiFi Station and Connect to Tracker
 */
esp_err_t wifi_client_init(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          WIFI CLIENT INITIALIZATION                        ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Create event group for WiFi synchronization
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "✗ Failed to create event group");
        return ESP_FAIL;
    }
    
    // === Initialize Network Stack ===
    ESP_LOGI(TAG, "Initializing network stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_LOGD(TAG, "✓ ESP-NETIF initialized");
    
    // === Initialize WiFi Stack ===
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGD(TAG, "✓ WiFi stack initialized");
    
    // === Register Event Handlers ===
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_LOGD(TAG, "✓ Event handlers registered");
    
    // === Configure Station Parameters ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "WiFi Configuration:");
    ESP_LOGI(TAG, "  SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "  Password: %s", WIFI_PASS);
    ESP_LOGI(TAG, "  Auth mode: WPA2-PSK");
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_LOGD(TAG, "✓ Station configuration set");
    
    // === Start WiFi FIRST ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Starting WiFi station...");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "✓ WiFi started");
    
    // === Performance Optimizations (AFTER WiFi started) ===
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Applying performance optimizations...");
    
    // Maximum TX power
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));  // 19.5 dBm
    ESP_LOGI(TAG, "  ✓ TX Power: 19.5 dBm (MAXIMUM)");
    
    // Disable power saving
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "  ✓ Power Saving: DISABLED");
    ESP_LOGI(TAG, "    - Always-on for instant updates");
    
    // Set 20MHz bandwidth
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    ESP_LOGI(TAG, "  ✓ Bandwidth: 20 MHz");
    ESP_LOGI(TAG, "    - Better range and stability");
    
    ESP_LOGI(TAG, "✓ Searching for tracker...");
    
    // Wait for connection (timeout: 15 seconds)
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000)
    );
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✓ WiFi connection successful");
        
        // Connect TCP socket
        esp_err_t ret = connect_tcp();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠ TCP connection failed, will retry");
            return ESP_FAIL;
        }
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║          CLIENT READY                                      ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Display ready to receive tracking data");
        ESP_LOGI(TAG, "");
        
        return ESP_OK;
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ WiFi connection failed");
        ESP_LOGE(TAG, "  - Tracker not found or password incorrect");
        ESP_LOGE(TAG, "  - Will keep retrying in background");
        ESP_LOGE(TAG, "");
        return ESP_ERR_WIFI_CONN;
        
    } else {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ WiFi connection timeout");
        ESP_LOGE(TAG, "  - Tracker AP not detected");
        ESP_LOGE(TAG, "  - Check tracker is powered on");
        ESP_LOGE(TAG, "  - Verify WiFi started on tracker");
        ESP_LOGE(TAG, "");
        return ESP_ERR_WIFI_TIMEOUT;
    }
}

/*
 * Receive Tracking Data from Tracker
 */
esp_err_t wifi_client_receive_data(tracker_data_t *data, uint32_t timeout_ms)
{
    if (!data) {
        ESP_LOGE(TAG, "NULL data pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check WiFi connection - try to reconnect if needed
    if (!wifi_connected) {
        ESP_LOGV(TAG, "WiFi not connected, checking...");
        
        // Check if we're actually connected but flag is stale
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi is actually connected, updating flag");
            wifi_connected = true;
        } else {
            return ESP_FAIL;
        }
    }
    
    // Connect TCP if needed (with verification)
    if (client_socket < 0 || !tcp_connected) {
        ESP_LOGI(TAG, "TCP disconnected, reconnecting...");
        
        // Close old socket if exists
        if (client_socket >= 0) {
            close(client_socket);
            client_socket = -1;
        }
        
        esp_err_t ret = connect_tcp();
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "TCP reconnect failed, retrying in %d sec", TCP_RECONNECT_DELAY_MS / 1000);
            vTaskDelay(pdMS_TO_TICKS(TCP_RECONNECT_DELAY_MS));
            return ESP_FAIL;
        }
        
        // Verify connection with a test send (0-byte send should succeed if connected)
        int test = send(client_socket, NULL, 0, MSG_DONTWAIT);
        if (test < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "TCP connection verification failed: %d (%s)", errno, strerror(errno));
            close(client_socket);
            client_socket = -1;
            tcp_connected = false;
            return ESP_FAIL;
        }
        
        ESP_LOGI(TAG, "✓ TCP connection verified and ready");
    }
    
    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // Receive packet
    int bytes_received = recv(client_socket, data, sizeof(tracker_data_t), 0);
    
    if (bytes_received < 0) {
        // Receive error
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Timeout (normal if no data yet)
            ESP_LOGV(TAG, "Receive timeout (no data available)");
            s_stats.rx_timeouts++;
            return ESP_ERR_TIMEOUT;
        } else {
            // Connection error
            ESP_LOGW(TAG, "⚠ Receive error: errno %d (%s)", errno, strerror(errno));
            ESP_LOGW(TAG, "  Closing socket, will reconnect");
            
            close(client_socket);
            client_socket = -1;
            tcp_connected = false;
            s_stats.rx_errors++;
            return ESP_FAIL;
        }
    }
    
    if (bytes_received == 0) {
        // Connection closed by tracker
        ESP_LOGW(TAG, "⚠ Connection closed by tracker");
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return ESP_FAIL;
    }
    
    if (bytes_received != sizeof(tracker_data_t)) {
        // Invalid packet size
        ESP_LOGW(TAG, "⚠ Invalid packet size: %d bytes (expected %zu)",
                 bytes_received, sizeof(tracker_data_t));
        s_stats.rx_errors++;
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Success
    s_stats.rx_packets++;
    ESP_LOGV(TAG, "Received %d bytes (packet #%lu)", bytes_received, s_stats.rx_packets);
    
    // Update RSSI periodically (every 100 packets)
    if (s_stats.rx_packets % 100 == 0) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_rssi = ap_info.rssi;
            ESP_LOGD(TAG, "Signal strength: %d dBm", s_rssi);
        }
    }
    
    return ESP_OK;
}

/*
 * Attempt Reconnection to Tracker
 */
esp_err_t wifi_client_reconnect(void)
{
    ESP_LOGI(TAG, "Manual reconnect requested");
    
    // Close existing TCP socket
    if (client_socket >= 0) {
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
    }
    
    // Reconnect WiFi if needed
    if (!wifi_connected) {
        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
        
        // Verify we're not already connected with stale flag
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi is actually connected, updating flag");
            wifi_connected = true;
            // Fall through to TCP reconnect
        } else {
            // Actually disconnected, trigger reconnect
            esp_wifi_connect();
            return ESP_FAIL;  // Will succeed on next event
        }
    }
    
    // Reconnect TCP
    esp_err_t ret = connect_tcp();
    if (ret == ESP_OK) {
        // Verify with test send
        int test = send(client_socket, NULL, 0, MSG_DONTWAIT);
        if (test < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "TCP connection verification failed after reconnect");
            close(client_socket);
            client_socket = -1;
            tcp_connected = false;
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "✓ Reconnection successful and verified");
    }
    
    return ret;
}

/*
 * Check if Connected to Tracker
 */
bool wifi_client_is_connected(void)
{
    // Check WiFi first
    if (!wifi_connected) {
        return false;
    }
    
    // Check TCP socket exists
    if (client_socket < 0) {
        tcp_connected = false;
        return false;
    }
    
    // Verify socket is actually alive using SO_ERROR
    int error = 0;
    socklen_t len = sizeof(error);
    if (getsockopt(client_socket, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
        // getsockopt failed, socket is dead
        ESP_LOGW(TAG, "Socket state check failed, marking as disconnected");
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return false;
    }
    
    if (error != 0) {
        // Socket has pending error, it's dead
        ESP_LOGW(TAG, "Socket has error: %d (%s)", error, strerror(error));
        close(client_socket);
        client_socket = -1;
        tcp_connected = false;
        return false;
    }
    
    return tcp_connected;
}

/*
 * Get WiFi Signal Strength (RSSI)
 */
int8_t wifi_client_get_signal_strength(void)
{
    if (!wifi_connected) {
        return -128;  // Not connected
    }
    
    // Get fresh RSSI
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        s_rssi = ap_info.rssi;
    }
    
    return s_rssi;
}

/*
 * Get WiFi IP Address
 */
const char* wifi_client_get_ip_address(void)
{
    return s_ip_address;
}

/*
 * Get Connection Statistics
 */
void wifi_client_get_stats(wifi_client_stats_t *stats)
{
    if (!stats) return;
    
    stats->rx_packets = s_stats.rx_packets;
    stats->rx_errors = s_stats.rx_errors;
    stats->rx_timeouts = s_stats.rx_timeouts;
    stats->reconnect_count = s_stats.reconnect_count;
    
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    stats->uptime_sec = wifi_connected ? (now - s_connection_start_time) : 0;
    
    stats->avg_rssi = s_rssi;
}