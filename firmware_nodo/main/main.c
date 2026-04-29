#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "config.h"
#include "espnow_handler.h"
#include "mpu6050.h"

static const char *TAG = "NODO_SENSOR";

RTC_DATA_ATTR static uint16_t batch_id = 0;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu_sample_t;

typedef struct __attribute__((packed)) {
    uint8_t id_nodo;
    int64_t start_timestamp;
    uint16_t batch_id;
    mpu_sample_t muestras[STREAM_SAMPLE_COUNT];
} sensor_packet_t;

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

static esp_err_t send_packet(const sensor_packet_t *packet)
{
    esp_err_t err;

    if (packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = espnow_handler_send((const uint8_t *)packet, sizeof(*packet));
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Paquete %u enviado. Timestamp: %" PRId64,
                 (unsigned)packet->batch_id,
                 packet->start_timestamp);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Fallo envio ESP-NOW: %s", esp_err_to_name(err));
    return err;
}

static esp_err_t send_fifo_history(uint16_t *next_batch_id)
{
    esp_err_t err;
    uint16_t fifo_bytes = 0;
    uint16_t fifo_samples = 0;
    mpu6050_sample_t raw_samples[STREAM_SAMPLE_COUNT];

    if (next_batch_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = mpu6050_enable_fifo(false);
    if (err != ESP_OK) {
        return err;
    }

    err = mpu6050_get_fifo_count(&fifo_bytes);
    if (err != ESP_OK) {
        return err;
    }

    fifo_samples = (uint16_t)(fifo_bytes / sizeof(mpu6050_sample_t));
    ESP_LOGI(TAG, "FIFO history: bytes=%u samples=%u", (unsigned)fifo_bytes, (unsigned)fifo_samples);

    while (fifo_samples > 0U) {
        uint8_t chunk = (fifo_samples > STREAM_SAMPLE_COUNT)
                            ? STREAM_SAMPLE_COUNT
                            : (uint8_t)fifo_samples;
        sensor_packet_t packet = {0};

        err = mpu6050_read_fifo_burst(raw_samples, chunk);
        if (err != ESP_OK) {
            return err;
        }

        packet.id_nodo = NODE_ID;
        packet.start_timestamp = esp_timer_get_time();
        packet.batch_id = *next_batch_id;

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

static void enter_deep_sleep(void)
{
    esp_err_t err;

    err = mpu6050_clear_interrupt_status();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo limpiar INT_STATUS: %s", esp_err_to_name(err));
    }

    err = mpu6050_reset_fifo();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo resetear FIFO: %s", esp_err_to_name(err));
    }

    err = mpu6050_enable_fifo(true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo habilitar FIFO: %s", esp_err_to_name(err));
    }

    err = mpu6050_setup_motion_interrupt(MOTION_THRESHOLD, MOTION_DURATION);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo configurar WOM: %s", esp_err_to_name(err));
    }

    err = esp_sleep_enable_ext1_wakeup((1ULL << MPU_INT_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar EXT1: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Entrando a deep sleep (EXT1 por INT MPU)");
    esp_deep_sleep_start();
}

void app_main(void)
{
    esp_err_t err;
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    mpu6050_config_t mpu_cfg = {
        .sda_io = I2C_SDA_GPIO,
        .scl_io = I2C_SCL_GPIO,
        .i2c_addr = MPU_I2C_ADDR,
        .accel_full_scale = MPU6050_ACCEL_FS_2G,
        .gyro_full_scale = MPU6050_GYRO_FS_250DPS,
        .sample_rate_hz = 100,
    };

    esp_log_level_set(TAG, ESP_LOG_INFO);
    log_wakeup_cause();

    err = mpu6050_init(&mpu_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar MPU6050: %s", esp_err_to_name(err));
        return;
    }

    err = espnow_handler_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar ESP-NOW: %s", esp_err_to_name(err));
        return;
    }

    if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) {
        err = send_fifo_history(&batch_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Fallo envio FIFO: %s", esp_err_to_name(err));
        }
    } else {
        err = mpu6050_enable_fifo(false);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo deshabilitar FIFO: %s", esp_err_to_name(err));
        }
    }

    {
        err = mpu6050_reset_fifo();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo resetear FIFO: %s", esp_err_to_name(err));
        }

        err = mpu6050_enable_fifo(true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo habilitar FIFO: %s", esp_err_to_name(err));
        }

        int64_t stream_start = esp_timer_get_time();
        int64_t stream_window_us = (int64_t)RECORD_TIME_MS * 1000LL;
        uint16_t bytes_necesarios = (uint16_t)(STREAM_SAMPLE_COUNT * sizeof(mpu_sample_t));
        mpu6050_sample_t raw_samples[STREAM_SAMPLE_COUNT];

        while ((esp_timer_get_time() - stream_start) < stream_window_us) {
            uint16_t fifo_bytes = 0;

            err = mpu6050_get_fifo_count(&fifo_bytes);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "No se pudo leer FIFO count: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (fifo_bytes >= bytes_necesarios) {
                sensor_packet_t packet = {0};

                err = mpu6050_read_fifo_burst(raw_samples, STREAM_SAMPLE_COUNT);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "No se pudo leer FIFO: %s", esp_err_to_name(err));
                    continue;
                }

                packet.id_nodo = NODE_ID;
                packet.start_timestamp = esp_timer_get_time();
                packet.batch_id = batch_id++;

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

    enter_deep_sleep();
}
