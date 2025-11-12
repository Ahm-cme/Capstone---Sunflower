idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash 
        gps 
        motor 
        sdlog 
        tracking 
        status_led 
        button 
        solar 
        battery
)