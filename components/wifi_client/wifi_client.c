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

#include "wifi_client.h"  // NOTE: tracker_data_t contract lives here; ensure struct packing/endianness matches sender

/*
--------------------------------------------------------------------------------
WiFi Client (ESP32 STA) for Sunflower Tracker Display
--------------------------------------------------------------------------------
Responsibilities:
- Join tracker SoftAP (SunflowerTracker), obtain IP, and open TCP socket
- Receive binary tracker_data_t frames and hand to UI layer
- Handle disconnects and auto-reconnect

Notes:
- Hardcoded SSID/password/IP/port for MVP; consider Kconfig for deploys
- TCP used for simplicity; could switch to UDP for lower latency and lossy tolerance
- Socket is blocking with recv timeout (SO_RCVTIMEO); caller provides timeout_ms

-------------------------------------------------------------------------------
*/

#define WIFI_SSID      "SunflowerTracker"  // Tracker SoftAP SSID
#define WIFI_PASS      "sunflower2025"     // Tracker SoftAP password
#define SERVER_IP      "192.168.4.1"       // Typical ESP32 SoftAP gateway IP
#define SERVER_PORT    8888                // TCP port exposed by master

#define WIFI_CONNECTED_BIT BIT0             // EventGroup bit for IP ready

static const char *TAG = "WIFI_CLIENT";
static EventGroupHandle_t s_wifi_event_group;
static int client_socket = -1;             // Active TCP socket; -1 when disconnected
static bool is_connected = false;          // True once STA has IP (not necessarily socket-open)

// Centralized event handler: WiFi start -> connect; got IP -> set flag; disconnect -> cleanup+reconnect
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to tracker...");
        esp_wifi_connect();  // Kick off STA connection
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        is_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // WiFi layer disconnected (AP down or RSSI bad). Mark state, close socket, and auto-reconnect.
        ESP_LOGW(TAG, "Disconnected, reconnecting...");
        is_connected = false;
        if (client_socket >= 0) {
            close(client_socket);  // Best-effort; ignore errors
            client_socket = -1;
        }
        esp_wifi_connect();  // Immediate reconnect; TODO: consider backoff
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_client_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi client...");
    
    s_wifi_event_group = xEventGroupCreate();  // Lifetime = app process

    // Netif + event loop are idempotent across calls; we do standard setup here
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register both generic WiFi events and the IP acquisition event
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // STA configuration for tracker AP (WPA2)
    // NOTE: These fields are fixed-size arrays; literals are safe here.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for connection...");

    // Block until IP ready; UI shows init checklist during this period
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
        
        // Connect to master via TCP
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

        client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (client_socket < 0) {
            // NOTE: errno provided by LWIP; include <errno.h> if toolchain requires
            ESP_LOGE(TAG, "Socket creation failed: errno %d", errno);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Connecting to tracker at %s:%d...", SERVER_IP, SERVER_PORT);
        int err = connect(client_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Connection failed: errno %d", errno);
            close(client_socket);
            client_socket = -1;
            return ESP_FAIL;
        }

        // TODO: Enable TCP keepalive to detect half-open connections:
        // int opt=1, idle=5, interval=2, count=3;
        // setsockopt(client_socket, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));
        // setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        // setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
        // setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

        ESP_LOGI(TAG, "Connected to tracker!");
        return ESP_OK;
    }
    
    return ESP_FAIL;  // Should not happen with portMAX_DELAY, but keep for completeness
}

esp_err_t wifi_client_receive_data(tracker_data_t *data, uint32_t timeout_ms)
{
    // Precondition: STA connected and socket open
    if (!is_connected || client_socket < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // Per-call receive timeout (non-blocking beyond timeout_ms)
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Receive a full tracker_data_t payload
    // NOTE: This assumes the master sends the struct in a single TCP write.
    // If fragmentation occurs, consider a small header and framed reads.
    int len = recv(client_socket, data, sizeof(tracker_data_t), 0);
    if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return ESP_ERR_TIMEOUT;  // Expected when no data within timeout
        }
        ESP_LOGE(TAG, "Receive error: errno %d", errno);
        return ESP_FAIL;
    } else if (len == 0) {
        // Peer closed the connection
        ESP_LOGW(TAG, "Connection closed");
        return ESP_ERR_INVALID_STATE;
    } else if (len != sizeof(tracker_data_t)) {
        // Partial frame received (likely fragmentation)
        // TODO: Implement a small frame protocol: [len][payload][crc]
        ESP_LOGW(TAG, "Partial frame: got %d of %u bytes", len, (unsigned)sizeof(tracker_data_t));
        return ESP_ERR_INVALID_SIZE;
    }

    // Optional: validate fields (e.g., range-check angles/voltage) before returning
    return ESP_OK;
}

bool wifi_client_is_connected(void)
{
    // Reports WiFi link + socket state; UI can use this for status
    return is_connected && (client_socket >= 0);
}