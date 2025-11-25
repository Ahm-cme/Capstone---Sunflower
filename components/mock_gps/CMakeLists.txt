idf_component_register(
    SRCS "mock_gps.c"
    INCLUDE_DIRS "include"
    REQUIRES "gps" "esp_timer" "nvs_flash"
)