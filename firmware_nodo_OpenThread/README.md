# Firmware Nodo Sensor OpenThread (ESP32-C6)

Este firmware adapta el flujo funcional de `firmware_nodo` a OpenThread, manteniendo:

- Mismo sensor y pines: MPU6050 en I2C (GPIO6/GPIO7)
- Mismo mecanismo de wakeup por movimiento (GPIO4)
- Mismo formato de payload (126 bytes con `STREAM_SAMPLE_COUNT=10`)
- Mismo ciclo: deep sleep -> wakeup -> drenado FIFO -> captura activa -> deep sleep

## Transporte de red

- Protocolo: OpenThread sobre IEEE 802.15.4
- Capa de aplicación: UDP/IPv6 unicast
- Rol del nodo: Sleepy End Device

## Formato de paquete

```c
typedef struct __attribute__((packed)) {
    uint32_t start_timestamp_ms;
    uint16_t batch_id;
    struct {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    } muestras[STREAM_SAMPLE_COUNT];
} sensor_packet_t;
```

Con `STREAM_SAMPLE_COUNT=10`, el tamaño es 126 bytes.

## Parametros clave

Editar en `main/config.h`:

- `OT_UDP_PEER_IPV6`: IPv6 del collector/gateway Thread
- `OT_UDP_PEER_PORT`: puerto UDP destino
- `OT_UDP_LOCAL_PORT`: puerto UDP local
- `OT_ATTACH_TIMEOUT_MS`: timeout de attach Thread
- `OT_POLL_PERIOD_MS`: poll period SED

## Dependencia de dataset

La red Thread se inicia con `esp_openthread_auto_start()`:

- Si existe dataset activo en NVS, lo reutiliza
- Si no existe, usa dataset definido por Kconfig (`menuconfig`)

Para una red de varios nodos, todos deben compartir el mismo dataset.

## Compilar y flashear

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p COMXX flash monitor
```
