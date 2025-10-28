/*
 * WiFi Communication Module (Access Point + TCP server)
 *
 * Role:
 * - ESP32 tracker runs as WiFi AP "SunflowerTracker"
 * - Listens on TCP port 8888 and streams tracker_data_t to one client (LCD)
 *
 * Protocol:
 * - Transport: TCP (unidirectional tracker → display)
 * - Rate: caller-driven (intended 1 Hz)
 * - Payload: raw tracker_data_t struct (binary)
 *
 * Network:
 * - AP SSID: SunflowerTracker
 * - Password: sunflower2025 (WPA2-PSK)
 * - Channel: 6
 * - Max stations: 1
 * - AP IP: 192.168.4.1 (default)
 *
 * Behavior:
 * - WiFi event handler marks station connect/disconnect
 * - Non-blocking accept() so main loop is never stalled
 * - On send error, socket is closed and retried next call
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

// WiFi credentials (tracker AP)
#define WIFI_SSID      "SunflowerTracker"
#define WIFI_PASS      "sunflower2025"
#define WIFI_CHANNEL   6
#define MAX_STA_CONN   1

// TCP server configuration
#define SERVER_PORT    8888

static const char *TAG = "WIFI_COMM";

// Sockets and state
static int server_socket = -1;          // Listening socket
static int client_socket = -1;          // Active client connection
static bool is_connected = false;       // Station associated with AP

/*
 * WiFi AP event handler:
 * - STACONNECTED: mark associated, allow TCP accept
 * - STADISCONNECTED: close client socket and clear flags
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        is_connected = true;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station disconnected: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        is_connected = false;

        if (client_socket >= 0) {       // Drop TCP session if present
            close(client_socket);
            client_socket = -1;
        }
    }
}

/*
 * Initialize WiFi AP and start TCP server (non-blocking accept).
 * Returns ESP_OK on success, ESP_FAIL on socket failures.
 */
esp_err_t wifi_comm_init_ap(void)
{
    ESP_LOGI(TAG, "Starting WiFi AP + TCP server...");

    ESP_ERROR_CHECK(esp_netif_init());                           // TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default());            // Event loop
    esp_netif_create_default_wifi_ap();                          // AP netif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();         // WiFi driver
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register AP events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // AP configuration
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));            // AP mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());                           // Radio on

    ESP_LOGI(TAG, "AP up: SSID=%s pass=%s ch=%d", WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

    // TCP server: create, non-block, bind, listen
    server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "Socket() failed: errno %d", errno);
        return ESP_FAIL;
    }

    int flags = fcntl(server_socket, F_GETFL, 0);
    fcntl(server_socket, F_SETFL, flags | O_NONBLOCK);           // Non-blocking accept()

    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Bind failed: errno %d", errno);
        close(server_socket);
        server_socket = -1;
        return ESP_FAIL;
    }

    if (listen(server_socket, 1) != 0) {
        ESP_LOGE(TAG, "Listen failed: errno %d", errno);
        close(server_socket);
        server_socket = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", SERVER_PORT);
    return ESP_OK;
}

/*
 * Send one tracker_data_t to the connected client.
 * - If no client yet, try non-blocking accept() and report NOT_FOUND.
 * - If WiFi not associated, report INVALID_STATE.
 * - On send error, close socket and report FAIL.
 */
esp_err_t wifi_comm_send_data(const tracker_data_t *data)
{
    if (!is_connected) {                                        // Station not on AP yet
        return ESP_ERR_INVALID_STATE;
    }

    if (client_socket < 0) {                                    // No TCP yet → try accept
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &addr_len);
        if (client_socket >= 0) {
            ESP_LOGI(TAG, "Client connected: %s", inet_ntoa(client_addr.sin_addr));
        } else {
            return ESP_ERR_NOT_FOUND;                           // No pending connection
        }
    }

    int err = send(client_socket, data, sizeof(tracker_data_t), 0);
    if (err < 0) {                                              // Broken pipe or similar
        ESP_LOGE(TAG, "Send failed: errno %d", errno);
        close(client_socket);
        client_socket = -1;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*
 * Connection state helper:
 * - true when station is associated AND TCP session is open.
 */
bool wifi_comm_is_connected(void)
{
    return is_connected && (client_socket >= 0);
}