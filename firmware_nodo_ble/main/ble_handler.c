#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "ble_handler.h"

/* ─── Tag de log ──────────────────────────────────────────── */
static const char *TAG = "BLE_HANDLER";

/* ─── Estado interno ──────────────────────────────────────── */
static uint16_t s_gatts_if       = ESP_GATT_IF_NONE;
static uint16_t s_conn_id        = 0xFFFF;
static uint16_t s_char_handle    = 0;
static uint16_t s_descr_handle   = 0;
static bool     s_connected      = false;
static bool     s_notify_enabled = false;
static bool     s_ble_ready      = false;

/* ─── UUIDs y propiedades GATT ────────────────────────────── */
static const uint16_t s_primary_svc_uuid  = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t s_char_decl_uuid    = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t s_svc_uuid          = BLE_SERVICE_UUID;
static const uint16_t s_char_uuid         = BLE_CHAR_UUID;
static const uint16_t s_cccd_uuid         = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

/* Propiedad: el Gateway puede leer y suscribirse a notificaciones */
static const uint8_t s_char_props =
    ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;

/* Buffer del valor de la caracteristica (sensor_packet_t cabe aqui) */
static uint8_t s_char_value[BLE_MAX_PAYLOAD] = {0};

/* CCCD inicial: notificaciones deshabilitadas */
static uint8_t s_cccd_value[2] = {0x00, 0x00};

/* ─── Indice de la tabla de atributos ─────────────────────── */
/*
 * Orden fijo requerido por esp_ble_gatts_create_attr_tab():
 *  0 → Servicio primario
 *  1 → Declaracion de caracteristica
 *  2 → Valor de la caracteristica   ← aqui mandamos sensor_packet_t
 *  3 → CCCD (el gateway escribe 0x0001 para activar notificaciones)
 */
enum {
    IDX_SVC = 0,
    IDX_CHAR_DECL,
    IDX_CHAR_VAL,
    IDX_CHAR_CCCD,
    IDX_TOTAL,
};

static uint16_t s_handle_table[IDX_TOTAL];

/* ─── Tabla de atributos GATT ─────────────────────────────── */
static const esp_gatts_attr_db_t s_attr_db[IDX_TOTAL] = {

    /* [0] Servicio primario */
    [IDX_SVC] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&s_primary_svc_uuid,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = sizeof(s_svc_uuid),
            .length      = sizeof(s_svc_uuid),
            .value       = (uint8_t *)&s_svc_uuid,
        },
    },

    /* [1] Declaracion de caracteristica */
    [IDX_CHAR_DECL] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&s_char_decl_uuid,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = sizeof(s_char_props),
            .length      = sizeof(s_char_props),
            .value       = (uint8_t *)&s_char_props,
        },
    },

    /* [2] Valor de la caracteristica — aqui va el sensor_packet_t */
    [IDX_CHAR_VAL] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&s_char_uuid,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = sizeof(s_char_value),
            .length      = sizeof(s_char_value),
            .value       = s_char_value,
        },
    },

    /* [3] CCCD — el Gateway escribe 0x0001 para activar notificaciones */
    [IDX_CHAR_CCCD] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&s_cccd_uuid,
            .perm        = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length  = sizeof(s_cccd_value),
            .length      = sizeof(s_cccd_value),
            .value       = s_cccd_value,
        },
    },
};

/* ─── Advertising ─────────────────────────────────────────── */
static uint16_t s_adv_svc_uuid = BLE_SERVICE_UUID;

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,       /* Incluye BLE_DEVICE_NAME */
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(s_adv_svc_uuid),
    .p_service_uuid      = (uint8_t *)&s_adv_svc_uuid,
    .flag                = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ─── Callback GAP ────────────────────────────────────────── */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        /* Datos de advertising configurados → arrancar */
        esp_ble_gap_start_advertising(&s_adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising BLE iniciado como '%s'", BLE_DEVICE_NAME);
        } else {
            ESP_LOGE(TAG, "Error al iniciar advertising BLE");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising BLE detenido");
        break;

    default:
        break;
    }
}

/* ─── Callback GATTS ──────────────────────────────────────── */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    /* Perfil registrado → configurar nombre, advertising y crear tabla */
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&s_adv_data);
        esp_ble_gatts_create_attr_tab(s_attr_db, gatts_if, IDX_TOTAL, 0);
        ESP_LOGI(TAG, "Perfil GATT registrado");
        break;

    /* Tabla de atributos creada → guardar handles e iniciar servicio */
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Error creando tabla GATT: 0x%x",
                     param->add_attr_tab.status);
            break;
        }
        if (param->add_attr_tab.num_handle != IDX_TOTAL) {
            ESP_LOGE(TAG, "Numero de handles incorrecto: %d",
                     param->add_attr_tab.num_handle);
            break;
        }
        memcpy(s_handle_table, param->add_attr_tab.handles,
               sizeof(s_handle_table));
        s_char_handle  = s_handle_table[IDX_CHAR_VAL];
        s_descr_handle = s_handle_table[IDX_CHAR_CCCD];
        esp_ble_gatts_start_service(s_handle_table[IDX_SVC]);
        ESP_LOGI(TAG, "Servicio GATT iniciado (char_handle=%d)", s_char_handle);
        s_ble_ready = true;
        break;

    /* Gateway conectado */
    case ESP_GATTS_CONNECT_EVT:
        s_conn_id   = param->connect.conn_id;
        s_connected = true;
        ESP_LOGI(TAG, "Gateway conectado (conn_id=%d)", s_conn_id);
        esp_ble_gap_stop_advertising();
        break;

    /* Gateway desconectado → reiniciar advertising para que vuelva a conectar */
    case ESP_GATTS_DISCONNECT_EVT:
        s_connected      = false;
        s_notify_enabled = false;
        ESP_LOGW(TAG, "Gateway desconectado, reiniciando advertising");
        esp_ble_gap_start_advertising(&s_adv_params);
        break;

    /* Gateway escribio el CCCD: 0x0001 = habilitar notificaciones */
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == s_descr_handle &&
            param->write.len == 2) {
            uint16_t cccd = (uint16_t)(param->write.value[0] |
                                       (param->write.value[1] << 8));
            s_notify_enabled = (cccd == 0x0001);
            ESP_LOGI(TAG, "Notificaciones %s",
                     s_notify_enabled ? "habilitadas" : "deshabilitadas");
        }
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU negociada: %d bytes", param->mtu.mtu);
        break;

    default:
        break;
    }
}

/* ─── API publica ─────────────────────────────────────────── */

esp_err_t ble_handler_init(void)
{
    if (s_ble_ready) {
        return ESP_OK;
    }

    /* NVS requerido por el stack BT */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    /* Inicializar controlador BT del ESP32-C6 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable: %s", esp_err_to_name(err));
        return err;
    }

    /* Registrar callbacks */
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) {
        return err;
    }

    /* Registrar perfil → dispara ESP_GATTS_REG_EVT */
    err = esp_ble_gatts_app_register(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gatts_app_register: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BLE inicializado como peripheral '%s'", BLE_DEVICE_NAME);
    return ESP_OK;
}

esp_err_t ble_handler_send(const uint8_t *data, size_t len)
{
    if (!s_ble_ready) {
        ESP_LOGW(TAG, "BLE no listo");
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((int)len > BLE_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload muy grande: %d > %d", (int)len, BLE_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!s_connected) {
        ESP_LOGW(TAG, "Gateway no conectado");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_notify_enabled) {
        ESP_LOGW(TAG, "Gateway aun no activo las notificaciones");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Enviamos el sensor_packet_t como notificacion BLE.
     * need_confirm = false → notificacion (como esp_now_send, fire-and-forget)
     * need_confirm = true  → indicacion con ACK (mas confiable)
     */
    esp_err_t err = esp_ble_gatts_send_indicate(
        s_gatts_if,
        s_conn_id,
        s_char_handle,
        (uint16_t)len,
        (uint8_t *)data,
        false
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error enviando notificacion: %s", esp_err_to_name(err));
    }

    return err;
}

bool ble_handler_is_connected(void)
{
    return s_connected;
}
