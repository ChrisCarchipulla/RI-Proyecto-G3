#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"

#include "openthread/error.h"
#include "openthread/ip6.h"
#include "openthread/message.h"
#include "openthread/thread.h"
#include "openthread/udp.h"

#include "config.h"
#include "esp_ot_config.h"
#include "openthread_handler.h"

static const char *TAG = "OT_HANDLER";

static bool s_ot_ready = false;
static otUdpSocket s_udp_socket;
static otIp6Address s_peer_addr;

static esp_err_t map_ot_error(otError err)
{
    switch (err) {
        case OT_ERROR_NONE:
            return ESP_OK;
        case OT_ERROR_INVALID_ARGS:
            return ESP_ERR_INVALID_ARG;
        case OT_ERROR_NO_BUFS:
            return ESP_ERR_NO_MEM;
        case OT_ERROR_INVALID_STATE:
            return ESP_ERR_INVALID_STATE;
        default:
            return ESP_FAIL;
    }
}

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

static esp_err_t init_ot_platform(void)
{
    esp_err_t err;
    esp_vfs_eventfd_config_t eventfd_cfg = {
        .max_fds = 3,
    };

    err = init_nvs_storage();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_vfs_eventfd_register(&eventfd_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t configure_ot_link_mode(otInstance *instance)
{
    otError ot_err;
    otLinkModeConfig link_mode = {0};

    link_mode.mRxOnWhenIdle = false;
    link_mode.mDeviceType = false;
    link_mode.mNetworkData = false;

    ot_err = otLinkSetPollPeriod(instance, OT_POLL_PERIOD_MS);
    if (ot_err != OT_ERROR_NONE) {
        return map_ot_error(ot_err);
    }

    ot_err = otThreadSetLinkMode(instance, link_mode);
    if (ot_err != OT_ERROR_NONE) {
        return map_ot_error(ot_err);
    }

    return ESP_OK;
}

static esp_err_t wait_for_attach(otInstance *instance)
{
    const TickType_t delay_ticks = pdMS_TO_TICKS(200);
    uint32_t waited_ms = 0;

    while (waited_ms < OT_ATTACH_TIMEOUT_MS) {
        otDeviceRole role;

        esp_openthread_lock_acquire(portMAX_DELAY);
        role = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();

        if (role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI(TAG, "OpenThread adjunto. Rol=%d", (int)role);
            return ESP_OK;
        }

        vTaskDelay(delay_ticks);
        waited_ms += 200;
    }

    ESP_LOGW(TAG, "Timeout esperando attach OpenThread");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t init_udp_socket(otInstance *instance)
{
    otError ot_err;
    otSockAddr bind_addr = {0};

    memset(&s_udp_socket, 0, sizeof(s_udp_socket));

    ot_err = otIp6AddressFromString(OT_UDP_PEER_IPV6, &s_peer_addr);
    if (ot_err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "IPv6 destino invalida: %s", OT_UDP_PEER_IPV6);
        return map_ot_error(ot_err);
    }

    ot_err = otUdpOpen(instance, &s_udp_socket, NULL, NULL);
    if (ot_err != OT_ERROR_NONE) {
        return map_ot_error(ot_err);
    }

    bind_addr.mPort = OT_UDP_LOCAL_PORT;
    ot_err = otUdpBind(instance, &s_udp_socket, &bind_addr, OT_NETIF_THREAD);
    if (ot_err != OT_ERROR_NONE) {
        (void)otUdpClose(instance, &s_udp_socket);
        return map_ot_error(ot_err);
    }

    return ESP_OK;
}

esp_err_t openthread_handler_init(void)
{
    esp_err_t err;
    otInstance *instance;
    otOperationalDatasetTlvs active_dataset;
    otError ot_err;

    if (s_ot_ready) {
        return ESP_OK;
    }

    err = init_ot_platform();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo init plataforma OT: %s", esp_err_to_name(err));
        return err;
    }

    static esp_openthread_config_t ot_cfg = {
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
        .platform_config = {
            .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
            .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
            .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
        },
    };

    err = esp_openthread_start(&ot_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_openthread_start fallo: %s", esp_err_to_name(err));
        return err;
    }

    esp_netif_set_default_netif(esp_openthread_get_netif());

    instance = esp_openthread_get_instance();
    if (instance == NULL) {
        return ESP_FAIL;
    }

    esp_openthread_lock_acquire(portMAX_DELAY);

    err = configure_ot_link_mode(instance);
    if (err != ESP_OK) {
        esp_openthread_lock_release();
        return err;
    }

    ot_err = otDatasetGetActiveTlvs(instance, &active_dataset);
    err = esp_openthread_auto_start((ot_err == OT_ERROR_NONE) ? &active_dataset : NULL);

    esp_openthread_lock_release();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_openthread_auto_start fallo: %s", esp_err_to_name(err));
        return err;
    }

    err = wait_for_attach(instance);
    if (err != ESP_OK) {
        return err;
    }

    esp_openthread_lock_acquire(portMAX_DELAY);
    err = init_udp_socket(instance);
    esp_openthread_lock_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo abrir socket UDP Thread: %s", esp_err_to_name(err));
        return err;
    }

    s_ot_ready = true;
    return ESP_OK;
}

esp_err_t openthread_handler_send(const uint8_t *data, size_t len)
{
    otInstance *instance;
    otMessageInfo msg_info = {0};
    otMessage *message = NULL;
    otError ot_err;
    esp_err_t err;

    if (!s_ot_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > TRANSPORT_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    instance = esp_openthread_get_instance();
    if (instance == NULL) {
        return ESP_FAIL;
    }

    esp_openthread_lock_acquire(portMAX_DELAY);

    msg_info.mPeerAddr = s_peer_addr;
    msg_info.mPeerPort = OT_UDP_PEER_PORT;
    msg_info.mSockPort = OT_UDP_LOCAL_PORT;

    message = otUdpNewMessage(instance, NULL);
    if (message == NULL) {
        esp_openthread_lock_release();
        return ESP_ERR_NO_MEM;
    }

    ot_err = otMessageAppend(message, data, len);
    if (ot_err != OT_ERROR_NONE) {
        otMessageFree(message);
        esp_openthread_lock_release();
        return map_ot_error(ot_err);
    }

    ot_err = otUdpSend(instance, &s_udp_socket, message, &msg_info);
    if (ot_err != OT_ERROR_NONE) {
        otMessageFree(message);
    }

    err = map_ot_error(ot_err);
    esp_openthread_lock_release();
    return err;
}

bool openthread_handler_is_attached(void)
{
    otInstance *instance = esp_openthread_get_instance();
    otDeviceRole role;

    if (!s_ot_ready || instance == NULL) {
        return false;
    }

    esp_openthread_lock_acquire(portMAX_DELAY);
    role = otThreadGetDeviceRole(instance);
    esp_openthread_lock_release();

    return (role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER);
}
