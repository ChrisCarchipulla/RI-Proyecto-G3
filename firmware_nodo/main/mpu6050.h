#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Muestra convertida del sensor MPU6050.
 */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} mpu6050_sample_t;

/**
 * @brief Inicializa el bus I2C y configura el MPU6050.
 *
 * @return esp_err_t ESP_OK si la inicializacion fue correcta, o un codigo de error en caso contrario.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Lee una muestra de acelerometro y giroscopio del MPU6050.
 *
 * @param sample Puntero de salida donde se almacena la muestra convertida.
 * @return esp_err_t ESP_OK si la lectura fue correcta, o un codigo de error en caso contrario.
 */
esp_err_t mpu6050_read_sample(mpu6050_sample_t *sample);

#endif
