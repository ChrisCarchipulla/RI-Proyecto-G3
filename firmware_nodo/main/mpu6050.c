#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mpu6050.h"

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

static const char *TAG = "MPU6050";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mpu_dev = NULL;

/**
 * @brief Inicializa el bus I2C maestro sobre GPIO6 (SDA) y GPIO7 (SCL).
 *
 * @return esp_err_t ESP_OK en caso de exito, o un error del driver I2C.
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
 * @brief Escribe un byte en un registro del MPU6050.
 *
 * @param reg Direccion del registro.
 * @param value Valor de un byte a escribir.
 * @return esp_err_t ESP_OK si la operacion fue correcta, o un error de transferencia I2C.
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(s_mpu_dev, payload, sizeof(payload), I2C_XFER_TIMEOUT_MS);
}

/**
 * @brief Lee uno o mas bytes desde un registro del MPU6050.
 *
 * @param reg Registro inicial a leer.
 * @param data Buffer de salida para almacenar los datos leidos.
 * @param len Cantidad de bytes a leer.
 * @return esp_err_t ESP_OK si la lectura fue correcta, o un error de transferencia I2C.
 */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, data, len, I2C_XFER_TIMEOUT_MS);
}

/**
 * @brief Inicializa el bus I2C y configura el MPU6050.
 *
 * @return esp_err_t ESP_OK si la inicializacion fue correcta, o un codigo de error en caso contrario.
 */
esp_err_t mpu6050_init(void)
{
    esp_err_t err;
    uint8_t who_am_i = 0;

    if (s_i2c_bus == NULL) {
        err = init_i2c_bus();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No se pudo inicializar I2C en SDA=%d SCL=%d: %s",
                     I2C_SDA_GPIO,
                     I2C_SCL_GPIO,
                     esp_err_to_name(err));
            return err;
        }
    }

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
        ESP_LOGE(TAG, "No se pudo registrar dispositivo I2C MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu6050_read_regs(MPU6050_WHO_AM_I_REG, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo leer WHO_AM_I del MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    if (who_am_i != MPU6050_WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG,
                 "WHO_AM_I invalido. Esperado=0x%02X recibido=0x%02X",
                 MPU6050_WHO_AM_I_EXPECTED,
                 who_am_i);
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

    err = mpu6050_write_reg(MPU6050_GYRO_CONFIG_REG, 0x00);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 inicializado correctamente en direccion 0x%02X", MPU6050_I2C_ADDR);
    }

    return err;
}

/**
 * @brief Lee una muestra de acelerometro y giroscopio del MPU6050.
 *
 * @param sample Puntero de salida donde se almacena la muestra convertida.
 * @return esp_err_t ESP_OK si la lectura fue correcta, o un codigo de error en caso contrario.
 */
esp_err_t mpu6050_read_sample(mpu6050_sample_t *sample)
{
    uint8_t raw[14] = {0};
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    esp_err_t err;

    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_mpu_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

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
