idf_component_register(SRCS "wifi_comm.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash lwip)