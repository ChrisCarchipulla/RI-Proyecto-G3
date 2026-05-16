#ifndef ESP_OT_CONFIG_H
#define ESP_OT_CONFIG_H

#include "esp_openthread_types.h"

#if SOC_IEEE802154_SUPPORTED
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG() \
    {                                          \
        .radio_mode = RADIO_MODE_NATIVE,       \
    }
#endif

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()       \
    {                                               \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE, \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG() \
    {                                         \
        .storage_partition_name = "nvs",     \
        .netif_queue_size = 10,               \
        .task_queue_size = 10,                \
    }

#endif
