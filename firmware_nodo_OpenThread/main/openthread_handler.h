#ifndef OPENTHREAD_HANDLER_H
#define OPENTHREAD_HANDLER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t openthread_handler_init(void);

esp_err_t openthread_handler_send(const uint8_t *data, size_t len);

bool openthread_handler_is_attached(void);

#endif
