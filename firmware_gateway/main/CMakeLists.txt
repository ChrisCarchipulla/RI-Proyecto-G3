idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES cjson esp_wifi mqtt esp_event nvs_flash)