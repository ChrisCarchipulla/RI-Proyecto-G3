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
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "led_strip.h"
#include "nvs_flash.h"

#define RGB_LED_GPIO                    8
#define RGB_LED_COUNT                   1
#define LED_BRIGHTNESS                  32
#define LED_BLINK_DELAY_MS              180

/* WARNING: Quick test credentials. Avoid committing real credentials to a public repository. */
#define WIFI_STA_SSID                   "GONET_GLADYS"
#define WIFI_STA_PASSWORD               "Gladys-Loma-76."
#define WIFI_MAXIMUM_RETRY              10

#define WIFI_CONNECTED_BIT              BIT0
#define WIFI_FAIL_BIT                   BIT1

#define I2C_SDA_GPIO                    6
#define I2C_SCL_GPIO                    7
#define I2C_PORT_NUM                    0
#define I2C_CLK_HZ                      400000
#define I2C_XFER_TIMEOUT_MS             100

#define MPU6050_I2C_ADDR                0x69
#define MPU6050_WHO_AM_I_REG            0x75
#define MPU6050_PWR_MGMT_1_REG          0x6B
#define MPU6050_ACCEL_CONFIG_REG        0x1C
#define MPU6050_GYRO_CONFIG_REG         0x1B
#define MPU6050_ACCEL_XOUT_H_REG        0x3B

#define MPU6050_WHO_AM_I_EXPECTED       0x68

#define MPU6050_ACCEL_SCALE_2G          16384.0f
#define MPU6050_GYRO_SCALE_250DPS       131.0f

#define SENSOR_TASK_PERIOD_MS           500
#define SENSOR_TASK_STACK_SIZE          4096
#define SENSOR_TASK_PRIORITY            5

static const char *TAG = "NODO_SENSOR";

static led_strip_handle_t s_led_strip = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mpu_dev = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_wifi_retry_num = 0;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} mpu6050_sample_t;

/**
 * @brief Initialize the integrated RGB LED strip device.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the LED driver.
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
 * @brief Set the integrated RGB LED to a given color.
 *
 * @param red Red channel value (0-255).
 * @param green Green channel value (0-255).
 * @param blue Blue channel value (0-255).
 * @return esp_err_t ESP_OK on success, or an error code if LED is not initialized or refresh fails.
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
 * @brief Startup pilot LED routine that blinks the integrated RGB LED in red three times.
 *
 * @return esp_err_t ESP_OK on success, or an error code from LED operations.
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
 * @brief Execute visual startup feedback.
 *
 * @return esp_err_t ESP_OK on success, or an error code from init_led_piloto_.
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
 * @brief Handle WiFi and IP events for STA connection lifecycle.
 *
 * @param arg Unused callback argument.
 * @param event_base Event base identifier.
 * @param event_id Event id within the event base.
 * @param event_data Optional event payload.
 * @return void This callback does not return a value.
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
 * @brief Initialize NVS subsystem required by WiFi stack.
 *
 * @return esp_err_t ESP_OK on success, or an NVS initialization error.
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
 * @brief Configure WiFi in Station mode and block until connected or failed.
 *
 * @return esp_err_t ESP_OK when connected, or an error code on failure.
 */
static esp_err_t wifi_init_sta(void)
{
    esp_err_t err;
    EventBits_t bits;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    wifi_config_t wifi_config = {0};

    err = init_nvs_storage();
    if (err != ESP_OK) {
        return err;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
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

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "wifi_init_sta() finalizado, esperando conexion al AP...");

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
 * @brief Synchronize system time with SNTP using time.google.com and configure Ecuador timezone.
 *
 * @return esp_err_t ESP_OK on successful synchronization, or ESP_ERR_TIMEOUT when sync did not complete.
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
 * @brief Stop WiFi radio after SNTP sync to reduce power consumption.
 *
 * @return esp_err_t ESP_OK on success, or an error code from esp_wifi_stop.
 */
static esp_err_t stop_wifi_for_power_save(void)
{
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi detenido para ahorro de energia");
    }
    return err;
}

/**
 * @brief Initialize I2C master bus on GPIO6 (SDA) and GPIO7 (SCL).
 *
 * @return esp_err_t ESP_OK on success, or an error code from the I2C driver.
 */
static esp_err_t init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    return i2c_new_master_bus(&bus_config, &s_i2c_bus);
}

/**
 * @brief Write one byte to an MPU6050 register.
 *
 * @param reg Register address.
 * @param value Byte value to write.
 * @return esp_err_t ESP_OK on success, or an I2C transfer error.
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(s_mpu_dev, payload, sizeof(payload), I2C_XFER_TIMEOUT_MS);
}

/**
 * @brief Read one or more bytes from an MPU6050 register.
 *
 * @param reg Start register address.
 * @param data Output buffer where bytes will be stored.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success, or an I2C transfer error.
 */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, data, len, I2C_XFER_TIMEOUT_MS);
}

/**
 * @brief Configure MPU6050 on I2C address 0x69 and set measurement ranges.
 *
 * @return esp_err_t ESP_OK on success, or an error code from I2C configuration/validation.
 */
static esp_err_t init_mpu6050(void)
{
    esp_err_t err;
    uint8_t who_am_i = 0;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu_dev);
    if (err != ESP_OK) {
        return err;
    }

    err = mpu6050_read_regs(MPU6050_WHO_AM_I_REG, &who_am_i, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (who_am_i != MPU6050_WHO_AM_I_EXPECTED) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = mpu6050_write_reg(MPU6050_PWR_MGMT_1_REG, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    err = mpu6050_write_reg(MPU6050_ACCEL_CONFIG_REG, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    return mpu6050_write_reg(MPU6050_GYRO_CONFIG_REG, 0x00);
}

/**
 * @brief Read accelerometer and gyroscope sample from MPU6050.
 *
 * @param sample Output sample structure populated with converted units.
 * @return esp_err_t ESP_OK on success, or an I2C communication error.
 */
static esp_err_t mpu6050_read_sample(mpu6050_sample_t *sample)
{
    uint8_t raw[14] = {0};
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    esp_err_t err;

    err = mpu6050_read_regs(MPU6050_ACCEL_XOUT_H_REG, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    ax = (int16_t)((raw[0] << 8) | raw[1]);
    ay = (int16_t)((raw[2] << 8) | raw[3]);
    az = (int16_t)((raw[4] << 8) | raw[5]);

    gx = (int16_t)((raw[8] << 8) | raw[9]);
    gy = (int16_t)((raw[10] << 8) | raw[11]);
    gz = (int16_t)((raw[12] << 8) | raw[13]);

    sample->accel_x_g = ((float)ax) / MPU6050_ACCEL_SCALE_2G;
    sample->accel_y_g = ((float)ay) / MPU6050_ACCEL_SCALE_2G;
    sample->accel_z_g = ((float)az) / MPU6050_ACCEL_SCALE_2G;

    sample->gyro_x_dps = ((float)gx) / MPU6050_GYRO_SCALE_250DPS;
    sample->gyro_y_dps = ((float)gy) / MPU6050_GYRO_SCALE_250DPS;
    sample->gyro_z_dps = ((float)gz) / MPU6050_GYRO_SCALE_250DPS;

    return ESP_OK;
}

/**
 * @brief Get current Unix Epoch timestamp in milliseconds.
 *
 * @return int64_t Unix Epoch time in milliseconds.
 */
static int64_t get_unix_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((int64_t)tv.tv_sec * 1000LL) + ((int64_t)tv.tv_usec / 1000LL);
}

/**
 * @brief FreeRTOS task that reads MPU6050 data every 500 ms and prints a clean log line.
 *
 * @param pvParameters Unused task parameter.
 * @return void This function does not return.
 */
static void mpu6050_task(void *pvParameters)
{
    mpu6050_sample_t sample;
    esp_err_t err;
    int64_t timestamp_ms;

    (void)pvParameters;

    while (1) {
        err = mpu6050_read_sample(&sample);
        if (err == ESP_OK) {
            timestamp_ms = get_unix_epoch_ms();
            ESP_LOGI(TAG,
                     "TS(ms):%" PRId64 " | ACC[g] X:% .3f Y:% .3f Z:% .3f | GYR[dps] X:% .3f Y:% .3f Z:% .3f",
                     timestamp_ms,
                     sample.accel_x_g,
                     sample.accel_y_g,
                     sample.accel_z_g,
                     sample.gyro_x_dps,
                     sample.gyro_y_dps,
                     sample.gyro_z_dps);
        } else {
            ESP_LOGE(TAG, "Lectura MPU6050 fallo: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/**
 * @brief Main application entry point.
 *
 * @return void This function does not return.
 */
void app_main(void)
{
    esp_err_t err;

    err = init_visual_feedback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo en feedback visual de arranque: %s", esp_err_to_name(err));
        return;
    }

    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_init_sta() fallo: %s", esp_err_to_name(err));
        return;
    }

    err = sync_time_with_ntp();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sincronizacion NTP fallo: %s", esp_err_to_name(err));
        return;
    }

    err = stop_wifi_for_power_save();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo detener WiFi: %s", esp_err_to_name(err));
        return;
    }

    err = init_i2c_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar I2C en SDA=%d SCL=%d: %s", I2C_SDA_GPIO, I2C_SCL_GPIO, esp_err_to_name(err));
        return;
    }

    err = init_mpu6050();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar MPU6050 @0x%02X: %s", MPU6050_I2C_ADDR, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MPU6050 inicializado correctamente en direccion 0x%02X", MPU6050_I2C_ADDR);

    if (xTaskCreate(mpu6050_task,
                    "mpu6050_task",
                    SENSOR_TASK_STACK_SIZE,
                    NULL,
                    SENSOR_TASK_PRIORITY,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear la tarea de lectura MPU6050");
        return;
    }
}
