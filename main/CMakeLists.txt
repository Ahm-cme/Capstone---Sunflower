idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES button gps motor sdlog solar status_led tracking wifi_comm esp_http_server battery mock_gps)