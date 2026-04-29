idf_component_register(SRCS "espnow_handler.c" "main.c" "mpu6050.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_wifi esp_event esp_netif nvs_flash lwip esp_timer)
