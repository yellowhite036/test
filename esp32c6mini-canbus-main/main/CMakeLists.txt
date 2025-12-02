idf_component_register(SRCS "main.c"
                           "motor_control.c"
                           "safety_system.c"
                           "ota_system.c"
                           "web_admin.c"
                           "web_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_twai
                             esp_driver_gpio
                             esp_driver_ledc
                             esp_http_server
                             esp_wifi
                             esp_timer
                             nvs_flash
                             app_update
                             spi_flash
                             esp_partition)