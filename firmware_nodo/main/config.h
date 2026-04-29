#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

#define NODE_ID                 1
#define ESPNOW_CHANNEL          1

#define I2C_SDA_GPIO            6
#define I2C_SCL_GPIO            7
#define MPU_I2C_ADDR            0x69
#define MPU_INT_GPIO            GPIO_NUM_4

#define MOTION_THRESHOLD        12
#define MOTION_DURATION         2

#define RECORD_TIME_MS          1000
#define STREAM_SAMPLE_COUNT     10
#define STREAM_SAMPLE_DELAY_MS  10

#define GATEWAY_MAC_BYTES       {0x40, 0x4C, 0xCA, 0x55, 0xAB, 0x10}

#endif
