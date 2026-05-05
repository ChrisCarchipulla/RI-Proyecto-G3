# Firmware Nodo Sensor - Inalámbrico IoT de Bajo Consumo

## Descripción General

Este firmware implementa un **nodo sensor IoT de bajo consumo** basado en **ESP32-C6** que captura datos de vibración/movimiento de un acelerómetro/giroscopio **MPU6050** y los transmite inalámbricamente al Gateway mediante **ESP-NOW**.

El sistema está optimizado para aplicaciones que requieren:
- **Detección de eventos por movimiento** en tuberías de agua u otros sistemas
- **Bajo consumo energético** mediante deep sleep activado por interrupciones
- **Comunicación inalámbrica de corta latencia** sin necesidad de Wi-Fi tradicional
- **Configuración flexible y rápida** de parámetros críticos

## Arquitectura del Sistema

### Flujo de Operación

1. **Estado de reposo (Deep Sleep):**
   - El ESP32-C6 entra en deep sleep para minimizar consumo
   - El MPU6050 sigue activo y capturando datos en su FIFO interno
   - Se configura una interrupción de movimiento (Motion On Wake) en el MPU

2. **Despertar por evento:**
   - Cuando se detecta movimiento, el MPU interrumpe al ESP32-C6 (pin GPIO4)
   - El ESP despierta y entra en modo activo

3. **Drenado de FIFO histórico:**
   - El ESP lee los datos acumulados en el FIFO del MPU mientras estaba dormido
   - Agrupa las muestras en paquetes de `STREAM_SAMPLE_COUNT` (configurable)
   - Envía cada paquete completo al Gateway

4. **Período de captura activa:**
   - Tras drenar el FIFO histórico, el ESP permanece despierto durante `RECORD_TIME_MS`
   - Continúa leyendo nuevas muestras del MPU en tiempo real
   - Transmite paquetes completos al Gateway

5. **Retorno a sleep:**
   - Transcurrido `RECORD_TIME_MS`, el ESP entra nuevamente en deep sleep
   - El ciclo vuelve al paso 1

## Estructura del Paquete (Payload)

Cada paquete ESP-NOW contiene:

| Campo | Tipo | Bytes | Descripción |
|-------|------|-------|-------------|
| `start_timestamp_ms` | `uint32_t` | 4 | Marca de tiempo en ms desde arranque (ajustable en gateway) |
| `batch_id` | `uint16_t` | 2 | ID único del paquete para detectar pérdidas |
| `muestras[]` | array | 12×N | N muestras de acelerómetro y giroscopio |
| **Total** | | **6 + 12×N** | N = `STREAM_SAMPLE_COUNT` (por defecto 10) |

### Ejemplo de tamaños:
- **N=5:** 66 bytes
- **N=10:** 126 bytes (configuración por defecto)
- **N=15:** 186 bytes
- **N=20:** 246 bytes (cercano al límite de 250)

**Nota:** El payload está limitado a **250 bytes** máximo por ESP-NOW. El compilador valida que `STREAM_SAMPLE_COUNT` no exceda este límite.

### Estructura de cada muestra:
```c
struct {
    int16_t ax, ay, az;  // Aceleración en 3 ejes (LSB)
    int16_t gx, gy, gz;  // Velocidad angular en 3 ejes (LSB)
}
```

## Configuración

Todos los parámetros críticos se encuentran en `main/config.h` y pueden editarse rápidamente sin recompilar el código completo (solo requiere rebuild del app):

### Parámetros de Canal y Red
```c
#define ESPNOW_CHANNEL          13          // Canal Wi-Fi para ESP-NOW (debe coincidir con gateway)
#define GATEWAY_MAC_BYTES       {0x40, 0x4C, 0xCA, 0x55, 0xAB, 0x10}  // MAC del Gateway destino
```

### Parámetros del MPU6050
```c
#define I2C_SDA_GPIO            6           // Pin GPIO para SDA (I2C)
#define I2C_SCL_GPIO            7           // Pin GPIO para SCL (I2C)
#define MPU_I2C_ADDR            0x69        // Dirección I2C del MPU6050
#define MPU_INT_GPIO            GPIO_NUM_4  // Pin de interrupción del MPU6050
#define MPU_SAMPLE_RATE_HZ      100         // Frecuencia de muestreo (Hz)
```

### Parámetros de Motion On Wake (WOM)
```c
#define MOTION_THRESHOLD        12          // Umbral de movimiento (unidades del MPU)
#define MOTION_DURATION         2           // Duración mínima de movimiento para despertar
```

### Parámetros de Captura
```c
#define RECORD_TIME_MS          1000        // Tiempo que el ESP permanece despierto (ms)
#define STREAM_SAMPLE_COUNT     10          // Número de muestras por paquete
#define STREAM_SAMPLE_DELAY_MS  10          // Delay entre lecturas (reservado)
```

### Límites del Sistema
```c
#define ESPNOW_MAX_DATA_LEN     250         // Tamaño máximo de payload ESP-NOW (bytes)
```

## Estructura del Protocolo ESP-NOW

### Desglose de Bytes en el Aire

Cuando el nodo transmite un paquete, la estructura completa "en aire" es:

```
┌─────────────────────────────────────────────────────────────────┐
│ Frame 802.11 Completo (encapsulación Wi-Fi)                     │
├─────────────────────────────────────────────────────────────────┤
│ 1. Preamble físico (no cuenta en API)                            │
│    └─ Sincronización y información de velocidad                 │
│                                                                   │
│ 2. MAC Header (~30 bytes)                                        │
│    ├─ Frame Control (2 bytes)                                    │
│    ├─ Duration (2 bytes)                                         │
│    ├─ Dirección destino (6 bytes)                                │
│    ├─ Dirección origen (6 bytes)                                 │
│    ├─ BSSID (6 bytes)                                            │
│    └─ Sequence Control (2 bytes)                                 │
│                                                                   │
│ 3. Payload de ESP-NOW (TU DATO) ◄── MAX 250 BYTES                │
│    └─ Encriptación (opcional, por defecto deshabilitada)        │
│                                                                   │
│ 4. Integridad y Verificación (~4 bytes)                          │
│    └─ Frame Check Sequence (FCS) / CRC-32                        │
└─────────────────────────────────────────────────────────────────┘
```

### Aclaración Importante

Los **250 bytes** que define `ESPNOW_MAX_DATA_LEN` son:
- ✅ **SOLO el payload de datos útil** (lo que pasas a `esp_now_send()`)
- ❌ **NO incluyen** headers 802.11, sincronización ni FCS
- ❌ **NO incluyen** la encapsulación de MAC/PHY

### Tamaño Total en Aire

El tamaño real transmitido "over-the-air" es aproximadamente:

$$\text{Frame Total} \approx 30 \text{ (MAC header)} + \text{Payload} + 4 \text{ (FCS)}$$

**Ejemplo con tu configuración actual (N=10):**
- Payload útil: 126 bytes
- Tamaño aproximado en aire: 30 + 126 + 4 = **160 bytes**
- Eficiencia: 126/160 ≈ **78.75%**

**Con valores extremos:**
- N=5 (66 bytes): ~100 bytes en aire
- N=20 (246 bytes): ~280 bytes en aire

### Bajo Consumo del Protocolo

La transmisión rápida y eficiente de ESP-NOW (comparado con Wi-Fi tradicional) se debe a:
1. Ausencia de handshake (conexión SSID)
2. Frames cortos (sin overhead de aplicación)
3. Modulación directa sin stack TCP/IP
4. Tiempo de transmisión: ~2-5 ms por paquete

Esto minimiza el tiempo que el radio está activo, reduciendo consumo drásticamente.

## Consumo Energético

El diseño optimiza consumo mediante:

1. **Deep Sleep:** El ESP32-C6 consume ~10 µA en sleep mode
2. **MPU6050 en espera:** ~0.5 mA manteniendo FIFO activo
3. **Activación selectiva:** Solo despierta ante movimiento real
4. **Transmisión eficiente:** Agrupa datos antes de transmitir

**Estimado total en standby:** ~1-2 mA (depende de duración del evento)

## Dependencias

- **Framework:** ESP-IDF 6.0
- **Componente (externo):** `espressif__led_strip` (managed)
- **Drivers:**
  - `esp_wifi` (solo radio, no conexión SSID)
  - `esp_now` (comunicación punto a punto)
  - `esp_driver_i2c` (comunicación con MPU6050)
  - `esp_sleep` (deep sleep y EXT1 wakeup)

## Compilación y Flasheo

```bash
# Configurar el target (si no está ya)
idf.py set-target esp32c6

# Compilar
idf.py build

# Flashear el nodo (COM11)
idf.py -p COM11 flash

# Monitor en tiempo real
idf.py -p COM11 monitor
```

## Salida por Monitor Serial

Ejemplo de logs durante operación:

```
[NODO_SENSOR] Wakeup: EXT1 (INT MPU)
[NODO_SENSOR] FIFO history: bytes=120 samples=10
[NODO_SENSOR] Paquete 42 enviado. Timestamp(ms): 5234
[NODO_SENSOR] Paquete 43 enviado. Timestamp(ms): 5244
...
[NODO_SENSOR] Entrando a deep sleep (EXT1 por INT MPU)
```

## Arquitectura de Archivos

```
main/
├── main.c              # Lógica principal (inicialización, flujo de captura)
├── config.h            # Definiciones de configuración
├── espnow_handler.c    # Inicialización y transmisión ESP-NOW
├── espnow_handler.h    # Interface de ESP-NOW
├── mpu6050.c           # Driver del MPU6050 (inicialización, lectura FIFO)
├── mpu6050.h           # Interface del MPU6050
└── CMakeLists.txt      # Build configuration
```

## Troubleshooting

| Problema | Causa | Solución |
|----------|-------|----------|
| **No compila:** `sensor_packet_t exceeds ESP-NOW payload limit` | `STREAM_SAMPLE_COUNT` demasiado grande | Reducir `STREAM_SAMPLE_COUNT` a ≤20 |
| **No despierta:** Sin log de `Wakeup` | Umbral de movimiento muy alto | Reducir `MOTION_THRESHOLD` |
| **Paquetes perdidos:** Log muestra `Fallo envio ESP-NOW` | Mala recepción o canal incorrecto | Verificar canal 13 y MAC del Gateway |
| **FIFO vacío al despertar** | Movimiento sin suficiente amplitud | Aumentar sensibilidad o umbral |

## Notas Importantes

1. **Timestamp local:** El nodo envía tiempo desde arranque (no NTP). El Gateway se encarga de sincronizar con servidor NTP si se requiere timestamp absoluto.

2. **Detección de pérdidas:** El `batch_id` incrementa con cada paquete; gaps indican paquetes perdidos.

3. **MAC del nodo:** El Gateway extrae la MAC real desde el protocolo ESP-NOW, **no desde el payload**.

4. **Compatibilidad:** Diseñado exclusivamente para funcionar con **Gateway compatible** que reciba en canal 13 y procese el formato de payload especificado.

## Futuras Mejoras

- [ ] Soporte para múltiples canales automáticos
- [ ] Encriptación en payload (si es requerido)
- [ ] Compensación de temperatura en MPU6050
- [ ] Compresión de datos antes de transmitir
