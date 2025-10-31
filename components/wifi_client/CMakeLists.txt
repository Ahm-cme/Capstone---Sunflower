idf_component_register(
    SRCS "wifi_client.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif lwip nvs_flash
)