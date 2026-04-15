#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "led_strip.h"
#include "nvs_flash.h"

#include "mpu6050.h"

#define RGB_LED_GPIO                    8
#define RGB_LED_COUNT                   1
#define LED_BRIGHTNESS                  32
#define LED_BLINK_DELAY_MS              180

/* ADVERTENCIA: Credenciales de prueba rapida. Evitar credenciales reales en repositorios publicos. */
#define WIFI_STA_SSID                   "GONET_GLADYS"
#define WIFI_STA_PASSWORD               "Gladys-Loma-76."
#define WIFI_MAXIMUM_RETRY              10

#define WIFI_CONNECTED_BIT              BIT0
#define WIFI_FAIL_BIT                   BIT1

#define NODE_ID                         1
#define ESPNOW_CHANNEL                  1
#define ESPNOW_POST_SEND_DELAY_MS       100
#define DEEP_SLEEP_SECONDS              10
#define VALID_EPOCH_2024                1704067200

static const uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0x40, 0x4C, 0xCA, 0x55, 0xAB, 0x10};

static const char *TAG = "NODO_SENSOR";

static led_strip_handle_t s_led_strip = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_wifi_retry_num = 0;

// Estructura empaquetada para garantizar el formato binario exacto al enviar por ESP-NOW.
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

/**
 * @brief Inicializa el dispositivo RGB integrado.
 *
 * @return esp_err_t ESP_OK en caso de exito, o un error del driver LED.
 */
static esp_err_t init_rgb_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = RGB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    return led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
}

/**
 * @brief Establece el color del LED RGB integrado.
 *
 * @param red Intensidad del canal rojo (0-255).
 * @param green Intensidad del canal verde (0-255).
 * @param blue Intensidad del canal azul (0-255).
 * @return esp_err_t ESP_OK si se actualizo correctamente, o un codigo de error.
 */
static esp_err_t set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    esp_err_t err;

    if (s_led_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    err = led_strip_set_pixel(s_led_strip, 0, red, green, blue);
    if (err != ESP_OK) {
        return err;
    }

    return led_strip_refresh(s_led_strip);
}

/**
 * @brief Ejecuta la rutina de arranque: LED rojo parpadea 3 veces.
 *
 * @return esp_err_t ESP_OK si la rutina finaliza correctamente, o un error de LED.
 */
static esp_err_t init_led_piloto_(void)
{
    esp_err_t err = ESP_OK;

    for (int i = 0; i < 3; i++) {
        err = set_led_color(LED_BRIGHTNESS, 0, 0);
        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));

        err = set_led_color(0, 0, 0);
        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
    }

    return ESP_OK;
}

/**
 * @brief Ejecuta el feedback visual de encendido.
 *
 * @return esp_err_t ESP_OK si todo fue correcto, o un error de inicializacion/LED.
 */
static esp_err_t init_visual_feedback(void)
{
    esp_err_t err;

    if (s_led_strip == NULL) {
        err = init_rgb_led();
        if (err != ESP_OK) {
            return err;
        }
    }

    return init_led_piloto_();
}

/**
 * @brief Manejador de eventos WiFi/IP para la conexion STA.
 *
 * @param arg Argumento de callback no utilizado.
 * @param event_base Base del evento recibido.
 * @param event_id Identificador del evento.
 * @param event_data Datos asociados al evento.
 * @return void Esta funcion no retorna valor.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA iniciando conexion...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
            s_wifi_retry_num++;
            ESP_LOGI(TAG, "WiFi reconectando intento %d/%d", s_wifi_retry_num, WIFI_MAXIMUM_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi no pudo conectarse al AP");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_wifi_retry_num = 0;
        ESP_LOGI(TAG, "WiFi conectado, IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Inicializa NVS, requerido por la pila de WiFi.
 *
 * @return esp_err_t ESP_OK en caso de exito, o un error de NVS.
 */
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

/**
 * @brief Inicializa la radio WiFi en modo STA sin conectarse a un AP.
 *
 * @return esp_err_t ESP_OK si la radio queda lista, o un error de inicializacion.
 */
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

    ESP_LOGI(TAG, "WiFi STA iniciado (modo radio para ESP-NOW)");
    return ESP_OK;
}

/**
 * @brief Conecta la interfaz STA al AP y espera resultado.
 *
 * @return esp_err_t ESP_OK si conecta, o ESP_FAIL si agota reintentos.
 */
static esp_err_t wifi_connect_sta(void)
{
    esp_err_t err;
    EventBits_t bits;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    wifi_config_t wifi_config = {0};

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              &instance_any_id);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler,
                                              NULL,
                                              &instance_got_ip);
    if (err != ESP_OK) {
        return err;
    }

    memcpy(wifi_config.sta.ssid, WIFI_STA_SSID, sizeof(WIFI_STA_SSID));
    memcpy(wifi_config.sta.password, WIFI_STA_PASSWORD, sizeof(WIFI_STA_PASSWORD));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    s_wifi_retry_num = 0;
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Esperando conexion al AP para SNTP...");

    bits = xEventGroupWaitBits(s_wifi_event_group,
                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                               pdFALSE,
                               pdFALSE,
                               portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conectado a SSID: %s", WIFI_STA_SSID);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "No fue posible conectar a SSID: %s", WIFI_STA_SSID);
    return ESP_FAIL;
}

/**
 * @brief Verifica si la hora del sistema es valida (anio 2024 o posterior).
 *
 * @return true Si la hora actual ya es valida.
 * @return false Si la hora aun no esta sincronizada.
 */
static bool is_time_valid_from_rtc(void)
{
    time_t now = 0;
    time(&now);
    return now >= VALID_EPOCH_2024;
}

/**
 * @brief Sincroniza la hora por SNTP usando time.google.com y ajusta zona horaria.
 *
 * @return esp_err_t ESP_OK si sincroniza correctamente, o ESP_ERR_TIMEOUT si falla.
 */
static esp_err_t sync_time_with_ntp(void)
{
    int retry = 0;
    const int retry_count = 45;
    const time_t valid_epoch_min = 1700000000;
    time_t now = 0;
    struct tm time_info = {0};
    sntp_sync_status_t sync_status = SNTP_SYNC_STATUS_RESET;

    if (esp_sntp_enabled()) {
        esp_sntp_stop();
    }

    sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.google.com");
    esp_sntp_init();

    while (retry < retry_count) {
        sync_status = sntp_get_sync_status();
        time(&now);

        if (sync_status == SNTP_SYNC_STATUS_COMPLETED || now >= valid_epoch_min) {
            break;
        }

        ESP_LOGI(TAG, "Esperando sincronizacion NTP (%d/%d)", retry + 1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }

    time(&now);
    if (sync_status != SNTP_SYNC_STATUS_COMPLETED && now < valid_epoch_min) {
        ESP_LOGE(TAG, "SNTP no logro sincronizar la hora");
        return ESP_ERR_TIMEOUT;
    }

    setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
    tzset();

    time(&now);
    localtime_r(&now, &time_info);
    ESP_LOGI(TAG,
             "Hora sincronizada: %04d-%02d-%02d %02d:%02d:%02d",
             time_info.tm_year + 1900,
             time_info.tm_mon + 1,
             time_info.tm_mday,
             time_info.tm_hour,
             time_info.tm_min,
             time_info.tm_sec);

    return ESP_OK;
}

/**
 * @brief Inicializa ESP-NOW y registra el gateway como peer.
 *
 * @return esp_err_t ESP_OK si la configuracion fue correcta, o un error de ESP-NOW.
 */
static esp_err_t init_espnow_peer(void)
{
    esp_err_t err;
    esp_now_peer_info_t peer_info = {0};

    err = esp_now_init();
    if (err != ESP_OK) {
        return err;
    }

    if (esp_now_is_peer_exist(s_peer_mac)) {
        ESP_LOGI(TAG, "Peer ESP-NOW ya registrado");
        return ESP_OK;
    }

    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    err = esp_now_add_peer(&peer_info);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Peer ESP-NOW agregado: %02X:%02X:%02X:%02X:%02X:%02X (canal %d)",
                 s_peer_mac[0],
                 s_peer_mac[1],
                 s_peer_mac[2],
                 s_peer_mac[3],
                 s_peer_mac[4],
                 s_peer_mac[5],
                 ESPNOW_CHANNEL);
    }

    return err;
}

/**
 * @brief Obtiene la marca de tiempo Unix Epoch en milisegundos.
 *
 * @return int64_t Marca de tiempo Unix Epoch en milisegundos.
 */
static int64_t get_unix_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((int64_t)tv.tv_sec * 1000LL) + ((int64_t)tv.tv_usec / 1000LL);
}

/**
 * @brief Construye el payload empaquetado con muestra del sensor y timestamp.
 *
 * @param sample Muestra leida del MPU6050.
 * @param timestamp_ms Marca de tiempo Unix Epoch en milisegundos.
 * @param out_payload Estructura de salida a transmitir por ESP-NOW.
 * @return void Esta funcion no retorna valor.
 */
static void fill_sensor_payload(const mpu6050_sample_t *sample, int64_t timestamp_ms, sensor_data_t *out_payload)
{
    out_payload->id_nodo = NODE_ID;
    out_payload->timestamp_ms = timestamp_ms;
    out_payload->accel_x_g = sample->accel_x_g;
    out_payload->accel_y_g = sample->accel_y_g;
    out_payload->accel_z_g = sample->accel_z_g;
    out_payload->gyro_x_dps = sample->gyro_x_dps;
    out_payload->gyro_y_dps = sample->gyro_y_dps;
    out_payload->gyro_z_dps = sample->gyro_z_dps;
}

/**
 * @brief Configura e inicia deep sleep por 10 segundos.
 *
 * @return void Esta funcion no retorna valor.
 */
static void enter_deep_sleep_10s(void)
{
    esp_err_t err;

    err = esp_sleep_enable_timer_wakeup((uint64_t)DEEP_SLEEP_SECONDS * 1000000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar wakeup timer: %s", esp_err_to_name(err));
        return;
    }

    // Punto critico: al entrar en deep sleep el chip reinicia en cada despertar.
    ESP_LOGI(TAG, "Entrando en deep sleep por %d segundos", DEEP_SLEEP_SECONDS);
    esp_deep_sleep_start();
}

/**
 * @brief Punto de entrada principal de la aplicacion.
 *
 * @return void Esta funcion no retorna valor.
 */
void app_main(void)
{
    esp_err_t err;
    mpu6050_sample_t sample;
    sensor_data_t payload;
    int64_t timestamp_ms;
    bool rtc_time_valid;

    err = init_visual_feedback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo en feedback visual de arranque: %s", esp_err_to_name(err));
        return;
    }

    err = wifi_init_sta_radio_only();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar WiFi STA: %s", esp_err_to_name(err));
        return;
    }

    rtc_time_valid = is_time_valid_from_rtc();
    if (!rtc_time_valid) {
        ESP_LOGI(TAG, "Hora RTC no valida, se conectara WiFi al AP para SNTP");

        err = wifi_connect_sta();
        if (err == ESP_OK) {
            err = sync_time_with_ntp();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Sincronizacion NTP no completada: %s", esp_err_to_name(err));
            }

            err = esp_wifi_disconnect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_CONNECT) {
                ESP_LOGW(TAG, "No se pudo desconectar del AP tras SNTP: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "No se pudo conectar al AP para NTP: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "Hora valida detectada en RTC, se omite conexion WiFi/NTP");
    }

    err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo fijar canal WiFi para ESP-NOW: %s", esp_err_to_name(err));
        return;
    }

    err = init_espnow_peer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar ESP-NOW/Peer: %s", esp_err_to_name(err));
        return;
    }

    err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar MPU6050: %s", esp_err_to_name(err));
        return;
    }

    err = mpu6050_read_sample(&sample);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lectura MPU6050 fallo: %s", esp_err_to_name(err));
        return;
    }

    timestamp_ms = get_unix_epoch_ms();
    fill_sensor_payload(&sample, timestamp_ms, &payload);

    ESP_LOGI(TAG,
             "TS(ms):%" PRId64 " | ACC[g] X:% .3f Y:% .3f Z:% .3f | GYR[dps] X:% .3f Y:% .3f Z:% .3f",
             payload.timestamp_ms,
             payload.accel_x_g,
             payload.accel_y_g,
             payload.accel_z_g,
             payload.gyro_x_dps,
             payload.gyro_y_dps,
             payload.gyro_z_dps);

    err = esp_now_send(s_peer_mac, (const uint8_t *)&payload, sizeof(payload));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo envio ESP-NOW: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,
                 "ESP-NOW enviado: nodo=%u bytes=%u destino=%02X:%02X:%02X:%02X:%02X:%02X",
                 payload.id_nodo,
                 (unsigned int)sizeof(payload),
                 s_peer_mac[0],
                 s_peer_mac[1],
                 s_peer_mac[2],
                 s_peer_mac[3],
                 s_peer_mac[4],
                 s_peer_mac[5]);
    }

    vTaskDelay(pdMS_TO_TICKS(ESPNOW_POST_SEND_DELAY_MS));

    // Punto critico: este sleep reduce consumo de forma drastica entre muestreos.
    enter_deep_sleep_10s();
}
