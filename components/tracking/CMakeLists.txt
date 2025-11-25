idf_component_register(
    SRCS "tracking.c"
    INCLUDE_DIRS "include"
    REQUIRES gps solar motor sdlog nvs_flash status_led mock_gps
)