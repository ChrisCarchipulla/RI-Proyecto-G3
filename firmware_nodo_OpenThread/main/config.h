#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

#define I2C_SDA_GPIO            6
#define I2C_SCL_GPIO            7
#define MPU_I2C_ADDR            0x69
#define MPU_INT_GPIO            GPIO_NUM_4

#define MOTION_THRESHOLD        12
#define MOTION_DURATION         2

#define MPU_SAMPLE_RATE_HZ      100

#define RECORD_TIME_MS          1000
#define STREAM_SAMPLE_COUNT     10
#define STREAM_SAMPLE_DELAY_MS  10

/* Mantiene el mismo limite logico de payload del firmware original. */
#define TRANSPORT_MAX_DATA_LEN  250

/* OpenThread UDP: configuracion del colector (gateway). */
#define OT_UDP_PEER_IPV6        "fdde:ad00:beef:0:0:ff:fe00:fc00"
#define OT_UDP_PEER_PORT        61631
#define OT_UDP_LOCAL_PORT       61631

/* Espera maxima para adjuntar a la red Thread antes de transmitir. */
#define OT_ATTACH_TIMEOUT_MS    15000

/* Sleepy End Device: periodo de polling para downlink. */
#define OT_POLL_PERIOD_MS       30000

#endif
