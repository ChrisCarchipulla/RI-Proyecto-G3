#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "mqtt_client.h"
#include "cJSON.h"

#define WIFI_SSID               "FRANKLIN-BENALCAZAR"
#define WIFI_PASS               "0103388724"
#define WIFI_MAX_RETRY          10
#define WIFI_CONNECTED_BIT      BIT0

#define THINGSBOARD_HOST        "mqtt.thingsboard.cloud"
#define THINGSBOARD_PORT        8883
#define THINGSBOARD_TOKEN       "wqnkkgmmlawl31zs4kbc"
#define THINGSBOARD_TOPIC       "v1/devices/me/telemetry"
#define MQTT_CLIENT_ID          "gateway-esp32"

#define ESPNOW_CHANNEL          1

static const char *TAG = "GATEWAY";

typedef struct __attribute__((packed)) {
    uint8_t id_nodo;
    int64_t timestamp_ms;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} sensor_data_t;

static EventGroupHandle_t s_wifi_event_group;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;
static bool s_packet_pending = false;
static portMUX_TYPE s_packet_mux = portMUX_INITIALIZER_UNLOCKED;
static sensor_data_t s_latest_packet;
static uint8_t s_latest_src_mac[ESP_NOW_ETH_ALEN] = {0};

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static void format_mac(const uint8_t *mac, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "WiFi conectado, IP=%s", ip_str);
    }
}

static esp_err_t wifi_init_sta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_err_t ch_err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ch_err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo fijar canal ESP-NOW (%s), se mantiene canal actual", esp_err_to_name(ch_err));
    }

    ESP_LOGI(TAG, "Conectando a SSID: %s", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(20000));

    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGW(TAG, "No se obtuvo IP en el tiempo de espera; se continua en modo asincrono");
    }

    return ESP_OK;
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://mqtt.thingsboard.cloud:1883"
            }
        },
        .credentials = {
            .client_id = MQTT_CLIENT_ID,
            .username = THINGSBOARD_TOKEN,
            .authentication = {
                .password = ""
            }
        }
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Error inicializando cliente MQTT");
        return NULL;
    }

    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
    return client;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT conectado");
        s_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT desconectado");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "Error MQTT");
        s_mqtt_connected = false;
        break;
    default:
        break;
    }
}

static void send_json_to_thingsboard(const sensor_data_t *data, const char *src_mac)
{
    if (!s_mqtt_connected) {
        ESP_LOGW(TAG, "MQTT no conectado, esperando reconexion");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Error creando JSON");
        return;
    }

    cJSON_AddStringToObject(root, "src_mac", src_mac);
    cJSON_AddNumberToObject(root, "node_id", data->id_nodo);
    cJSON_AddNumberToObject(root, "timestamp_ms", data->timestamp_ms);
    cJSON_AddNumberToObject(root, "accel_x_g", data->accel_x_g);
    cJSON_AddNumberToObject(root, "accel_y_g", data->accel_y_g);
    cJSON_AddNumberToObject(root, "accel_z_g", data->accel_z_g);
    cJSON_AddNumberToObject(root, "gyro_x_dps", data->gyro_x_dps);
    cJSON_AddNumberToObject(root, "gyro_y_dps", data->gyro_y_dps);
    cJSON_AddNumberToObject(root, "gyro_z_dps", data->gyro_z_dps);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload == NULL) {
        ESP_LOGE(TAG, "Error serializando JSON");
        cJSON_Delete(root);
        return;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, THINGSBOARD_TOPIC, payload, 0, 1, 0);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Fallo al publicar en ThingsBoard");
    } else {
        ESP_LOGI(TAG, "Publicado mensaje MQTT id=%d", msg_id);
        ESP_LOGI(TAG, "%s", payload);
    }

    cJSON_free(payload);
    cJSON_Delete(root);
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != sizeof(sensor_data_t)) {
        ESP_LOGW(TAG, "Paquete invalido recibido, tamaño %d esperado %d", len, (int)sizeof(sensor_data_t));
        return;
    }

    portENTER_CRITICAL(&s_packet_mux);
    memcpy(&s_latest_packet, data, sizeof(sensor_data_t));
    if (info != NULL && info->src_addr != NULL) {
        memcpy(s_latest_src_mac, info->src_addr, ESP_NOW_ETH_ALEN);
    } else {
        memset(s_latest_src_mac, 0, ESP_NOW_ETH_ALEN);
    }
    s_packet_pending = true;
    portEXIT_CRITICAL(&s_packet_mux);
}

static esp_err_t espnow_init(void)
{
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error esp_now_init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando callback ESP-NOW: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ESP-NOW listo en canal %d", ESPNOW_CHANNEL);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(wifi_init_sta());

    s_mqtt_client = mqtt_app_start();
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "No se pudo iniciar MQTT");
    }

    esp_err_t espnow_err = espnow_init();
    if (espnow_err != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ESP-NOW");
    }

    while (true) {
        if (s_packet_pending) {
            portENTER_CRITICAL(&s_packet_mux);
            sensor_data_t packet = s_latest_packet;
            uint8_t src_mac[ESP_NOW_ETH_ALEN];
            memcpy(src_mac, s_latest_src_mac, ESP_NOW_ETH_ALEN);
            s_packet_pending = false;
            portEXIT_CRITICAL(&s_packet_mux);

            char mac_str[18] = {0};
            format_mac(src_mac, mac_str, sizeof(mac_str));
            ESP_LOGI(TAG, "Recibido paquete de %s", mac_str);
            send_json_to_thingsboard(&packet, mac_str);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
