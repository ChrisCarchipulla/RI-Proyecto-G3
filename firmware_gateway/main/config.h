#pragma once

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define WIFI_SSID               "Redmi 14C"
#define WIFI_PASS               "123456789"

#define THINGSBOARD_HOST        "mqtt.thingsboard.cloud"
#define THINGSBOARD_PORT        1883
#define THINGSBOARD_TOKEN       "Nxma1JhwUET1vTPrZm3j"
#define THINGSBOARD_TOPIC       "v1/devices/me/telemetry"
#define MQTT_CLIENT_ID          "gateway-esp32"

#define ESPNOW_CHANNEL          1

#define SNTP_SERVER             "pool.ntp.org"