#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "config.h"
#include "ble_handler.h"   /* <-- antes era espnow_handler.h */
#include "mpu6050.h"       /* <-- sin cambios */

static const char *TAG = "NODO_SENSOR";

/* batch_id persiste en RTC durante deep sleep, igual que el original */
RTC_DATA_ATTR static uint16_t batch_id = 0;

/* ─── Estructura del paquete ─────────────────────────────────
 * IDENTICA a la del original y a la del gateway.
 * Si cambias STREAM_SAMPLE_COUNT aqui, cambialo tambien en el gateway.
 */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t start_timestamp_ms;
    uint16_t batch_id;
    mpu_sample_t muestras[STREAM_SAMPLE_COUNT];
} sensor_packet_t;

_Static_assert(sizeof(sensor_packet_t) <= BLE_MAX_PAYLOAD,
               "sensor_packet_t excede el payload maximo BLE");

/* ─── Helpers ────────────────────────────────────────────────*/
static uint32_t get_timestamp_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void log_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT1:
        ESP_LOGI(TAG, "Wakeup: EXT1 (INT MPU)");
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "Wakeup: UNDEFINED (cold boot)");
        break;
    default:
        ESP_LOGI(TAG, "Wakeup: %d", (int)cause);
        break;
    }
}

/* ─── Envio de un paquete ────────────────────────────────────
 * Misma firma que el original.
 * Solo cambia ble_handler_send() en lugar de espnow_handler_send().
 */
static esp_err_t send_packet(const sensor_packet_t *packet)
{
    if (packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * BLE necesita que el Gateway este conectado y haya habilitado
     * notificaciones. Si no esta listo, esperamos hasta 5 segundos.
     */
    const TickType_t wait_step = pdMS_TO_TICKS(100);
    const int max_retries = 50; /* 5 segundos */
    int retries = 0;

    while (!ble_handler_is_connected() && retries < max_retries) {
        ESP_LOGW(TAG, "Esperando conexion BLE... (%d/%d)", retries, max_retries);
        vTaskDelay(wait_step);
        retries++;
    }

    if (!ble_handler_is_connected()) {
        ESP_LOGE(TAG, "Gateway BLE no conectado, paquete descartado");
        return ESP_FAIL;
    }

    esp_err_t err = ble_handler_send((const uint8_t *)packet, sizeof(*packet));
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Paquete %u enviado via BLE. Timestamp(ms): %" PRIu32,
                 (unsigned)packet->batch_id,
                 packet->start_timestamp_ms);
    } else {
        ESP_LOGE(TAG, "Fallo envio BLE: %s", esp_err_to_name(err));
    }

    return err;
}

/* ─── Drenado del FIFO historico ─────────────────────────────
 * IGUAL al original, solo usa send_packet() que ahora llama a BLE.
 */
static esp_err_t send_fifo_history(uint16_t *next_batch_id)
{
    if (next_batch_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mpu6050_enable_fifo(false);
    if (err != ESP_OK) return err;

    uint16_t fifo_bytes = 0;
    err = mpu6050_get_fifo_count(&fifo_bytes);
    if (err != ESP_OK) return err;

    uint16_t fifo_samples = (uint16_t)(fifo_bytes / sizeof(mpu6050_sample_t));
    ESP_LOGI(TAG, "FIFO history: bytes=%u samples=%u",
             (unsigned)fifo_bytes, (unsigned)fifo_samples);

    mpu6050_sample_t raw_samples[STREAM_SAMPLE_COUNT];

    while (fifo_samples > 0U) {
        uint8_t chunk = (fifo_samples > STREAM_SAMPLE_COUNT)
                        ? STREAM_SAMPLE_COUNT
                        : (uint8_t)fifo_samples;

        sensor_packet_t packet = {0};

        err = mpu6050_read_fifo_burst(raw_samples, chunk);
        if (err != ESP_OK) return err;

        packet.start_timestamp_ms = get_timestamp_ms();
        packet.batch_id           = *next_batch_id;

        for (uint8_t i = 0; i < chunk; i++) {
            packet.muestras[i].ax = raw_samples[i].accel_x_raw;
            packet.muestras[i].ay = raw_samples[i].accel_y_raw;
            packet.muestras[i].az = raw_samples[i].accel_z_raw;
            packet.muestras[i].gx = raw_samples[i].gyro_x_raw;
            packet.muestras[i].gy = raw_samples[i].gyro_y_raw;
            packet.muestras[i].gz = raw_samples[i].gyro_z_raw;
        }

        (void)send_packet(&packet);
        (*next_batch_id)++;
        fifo_samples = (uint16_t)(fifo_samples - chunk);
    }

    return ESP_OK;
}

/* ─── Deep sleep ─────────────────────────────────────────────
 * IGUAL al original.
 */
static void enter_deep_sleep(void)
{
    esp_err_t err;

    err = mpu6050_clear_interrupt_status();
    if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo limpiar INT_STATUS");

    err = mpu6050_reset_fifo();
    if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo resetear FIFO");

    err = mpu6050_enable_fifo(true);
    if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo habilitar FIFO");

    err = mpu6050_setup_motion_interrupt(MOTION_THRESHOLD, MOTION_DURATION);
    if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo configurar WOM");

    err = esp_sleep_enable_ext1_wakeup((1ULL << MPU_INT_GPIO),
                                       ESP_EXT1_WAKEUP_ANY_HIGH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar EXT1: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Entrando a deep sleep (EXT1 por INT MPU)");
    esp_deep_sleep_start();
}

/* ─── app_main ───────────────────────────────────────────────
 * MISMO flujo que el original:
 *  1. Inicializar MPU6050
 *  2. Inicializar transporte (antes ESP-NOW, ahora BLE)
 *  3. Si vino de EXT1: drenar FIFO historico
 *  4. Stream activo durante RECORD_TIME_MS
 *  5. Deep sleep
 */
void app_main(void)
{
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();

    mpu6050_config_t mpu_cfg = {
        .sda_io          = I2C_SDA_GPIO,
        .scl_io          = I2C_SCL_GPIO,
        .i2c_addr        = MPU_I2C_ADDR,
        .accel_full_scale = MPU6050_ACCEL_FS_2G,
        .gyro_full_scale  = MPU6050_GYRO_FS_250DPS,
        .sample_rate_hz  = MPU_SAMPLE_RATE_HZ,
    };

    esp_log_level_set(TAG, ESP_LOG_INFO);
    log_wakeup_cause();

    /* 1. MPU6050 — sin cambios */
    esp_err_t err = mpu6050_init(&mpu_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar MPU6050: %s", esp_err_to_name(err));
        return;
    }

    /* 2. BLE — reemplaza espnow_handler_init() */
    err = ble_handler_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar BLE: %s", esp_err_to_name(err));
        return;
    }

    /* 3. Si vino de EXT1: drenar FIFO acumulado durante el sleep */
    if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) {
        err = send_fifo_history(&batch_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Fallo envio FIFO: %s", esp_err_to_name(err));
        }
    } else {
        /* Cold boot: limpiar FIFO antes de empezar */
        err = mpu6050_enable_fifo(false);
        if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo deshabilitar FIFO");
    }

    /* 4. Stream activo durante RECORD_TIME_MS — igual al original */
    {
        err = mpu6050_reset_fifo();
        if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo resetear FIFO");

        err = mpu6050_enable_fifo(true);
        if (err != ESP_OK) ESP_LOGW(TAG, "No se pudo habilitar FIFO");

        int64_t  stream_start    = esp_timer_get_time();
        int64_t  stream_window   = (int64_t)RECORD_TIME_MS * 1000LL;
        uint16_t bytes_necesarios = (uint16_t)(STREAM_SAMPLE_COUNT * sizeof(mpu6050_sample_t));
        mpu6050_sample_t raw_samples[STREAM_SAMPLE_COUNT];

        while ((esp_timer_get_time() - stream_start) < stream_window) {

            uint16_t fifo_bytes = 0;
            err = mpu6050_get_fifo_count(&fifo_bytes);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Error leyendo FIFO count: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (fifo_bytes >= bytes_necesarios) {
                sensor_packet_t packet = {0};

                err = mpu6050_read_fifo_burst(raw_samples, STREAM_SAMPLE_COUNT);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Error leyendo FIFO: %s", esp_err_to_name(err));
                    continue;
                }

                packet.start_timestamp_ms = get_timestamp_ms();
                packet.batch_id           = batch_id++;

                for (uint8_t i = 0; i < STREAM_SAMPLE_COUNT; i++) {
                    packet.muestras[i].ax = raw_samples[i].accel_x_raw;
                    packet.muestras[i].ay = raw_samples[i].accel_y_raw;
                    packet.muestras[i].az = raw_samples[i].accel_z_raw;
                    packet.muestras[i].gx = raw_samples[i].gyro_x_raw;
                    packet.muestras[i].gy = raw_samples[i].gyro_y_raw;
                    packet.muestras[i].gz = raw_samples[i].gyro_z_raw;
                }

                (void)send_packet(&packet);

            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    /* 5. Deep sleep — igual al original */
    enter_deep_sleep();
}
