#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

/* ─── MPU6050 ─────────────────────────────────────────────── */
#define I2C_SDA_GPIO            6
#define I2C_SCL_GPIO            7
#define MPU_I2C_ADDR            0x69
#define MPU_INT_GPIO            GPIO_NUM_4

/* ─── Motion On Wake ──────────────────────────────────────── */
#define MOTION_THRESHOLD        12
#define MOTION_DURATION         2

/* ─── Muestreo ────────────────────────────────────────────── */
#define MPU_SAMPLE_RATE_HZ      100
#define RECORD_TIME_MS          1000
#define STREAM_SAMPLE_COUNT     10
#define STREAM_SAMPLE_DELAY_MS  10

/* ─── BLE ─────────────────────────────────────────────────── */
/* Nombre con el que el nodo aparece al escanear (el gateway lo busca por este nombre) */
#define BLE_DEVICE_NAME         "NODO_SENSOR"

/* UUID del servicio y caracteristica (deben coincidir en gateway) */
#define BLE_SERVICE_UUID        0xFFFF
#define BLE_CHAR_UUID           0xFF01

/* Tamano maximo del payload BLE (MTU por defecto menos overhead) */
#define BLE_MAX_PAYLOAD         247

#endif /* CONFIG_H */
