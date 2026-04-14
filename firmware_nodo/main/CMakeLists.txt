idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES led_strip esp_driver_i2c esp_wifi esp_event esp_netif nvs_flash lwip)
