idf_component_register(SRCS "main.c" "mpu6050.c"
                       INCLUDE_DIRS "."
                       REQUIRES led_strip esp_driver_i2c esp_driver_gpio esp_wifi esp_event esp_netif nvs_flash lwip esp_timer)
