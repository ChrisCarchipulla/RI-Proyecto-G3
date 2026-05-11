#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Inicializa el stack BLE y arranca el advertising.
 *        El nodo actua como BLE Peripheral (servidor GATT).
 *        Equivalente a espnow_handler_init().
 *
 * @return esp_err_t ESP_OK si la inicializacion fue correcta.
 */
esp_err_t ble_handler_init(void);

/**
 * @brief Envia el sensor_packet_t al Gateway via notificacion GATT.
 *        Equivalente a espnow_handler_send().
 *        Bloquea hasta que el Gateway habilite notificaciones.
 *
 * @param data  Puntero al buffer (sensor_packet_t).
 * @param len   Longitud en bytes.
 * @return esp_err_t ESP_OK si el envio fue correcto.
 */
esp_err_t ble_handler_send(const uint8_t *data, size_t len);

/**
 * @brief Retorna true si hay un Gateway conectado actualmente.
 */
bool ble_handler_is_connected(void);

#endif /* BLE_HANDLER_H */
