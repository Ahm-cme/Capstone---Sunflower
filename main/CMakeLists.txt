idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES button gps motor sdlog solar status_led tracking wifi_comm battery mock_gps)