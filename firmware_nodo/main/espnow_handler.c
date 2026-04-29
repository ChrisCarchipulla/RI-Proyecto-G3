#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "config.h"
#include "espnow_handler.h"

static const char *TAG = "ESPNOW_HANDLER";

static const uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = GATEWAY_MAC_BYTES;
static bool s_espnow_ready = false;

static esp_err_t init_nvs_storage(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_flash_init();
    }

    return err;
}

static esp_err_t wifi_init_sta_radio_only(void)
{
    esp_err_t err;

    err = init_nvs_storage();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t init_espnow_peer(void)
{
    esp_err_t err;
    esp_now_peer_info_t peer_info = {0};

    err = esp_now_init();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        return err;
    }

    if (esp_now_is_peer_exist(s_peer_mac)) {
        return ESP_OK;
    }

    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    return esp_now_add_peer(&peer_info);
}

esp_err_t espnow_handler_init(void)
{
    esp_err_t err;

    if (s_espnow_ready) {
        return ESP_OK;
    }

    err = wifi_init_sta_radio_only();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi STA init fallo: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo fijar canal ESPNOW: %s", esp_err_to_name(err));
        return err;
    }

    err = init_espnow_peer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar ESPNOW: %s", esp_err_to_name(err));
        return err;
    }

    s_espnow_ready = true;
    return ESP_OK;
}

esp_err_t espnow_handler_send(const uint8_t *data, size_t len)
{
    if (!s_espnow_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_now_send(s_peer_mac, data, len);
}
