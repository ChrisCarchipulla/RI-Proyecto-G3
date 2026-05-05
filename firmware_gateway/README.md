# Firmware Gateway ESP32-C6 -> ThingsBoard

Este proyecto implementa un gateway en ESP32-C6 que:

- Se conecta por Wi-Fi en modo STA.
- Recibe tramas de nodos por ESP-NOW.
- Publica telemetria en ThingsBoard por MQTT.

## 1) Requisitos

- ESP-IDF v6.0 instalado y funcional.
- Target configurado en esp32c6.
- Placa ESP32-C6 conectada por USB.
- Credenciales Wi-Fi disponibles.
- Device creado en ThingsBoard con Access Token.

## 2) Archivos clave del proyecto

- CMake principal: CMakeLists.txt
- Codigo del gateway: main/main.c
- Dependencias del componente: main/idf_component.yml
- Configuracion base de SDK: sdkconfig y sdkconfig.defaults

## 3) Parametros que debes cambiar antes de compilar

Edita estas macros en main/config.h:

- WIFI_SSID
- WIFI_PASS
- THINGSBOARD_TOKEN
- THINGSBOARD_HOST (normalmente mqtt.thingsboard.cloud)
- THINGSBOARD_PORT
- MQTT URI dentro de mqtt_app_start

Estado actual en el codigo:

- Conexion MQTT sin TLS: mqtt://mqtt.thingsboard.cloud:1883

### Nota importante sobre token y red

En operaciones reales, normalmente actualizas juntos los parametros de conectividad cuando cambias de entorno:

- Si cambias de red Wi-Fi, debes actualizar WIFI_SSID y WIFI_PASS.
- Si cambias de dispositivo/tenant en ThingsBoard, debes actualizar THINGSBOARD_TOKEN.

Aunque el token no depende tecnicamente del SSID, en este flujo de trabajo se recomienda revisar siempre los tres valores al migrar de red o entorno para evitar desconexiones.

### Plantilla rapida de configuracion

Copia esta plantilla y reemplaza los valores en main/config.h antes de compilar:

```c
#define WIFI_SSID               "NOMBRE_DE_TU_RED"
#define WIFI_PASS               "CLAVE_DE_TU_RED"

#define THINGSBOARD_HOST        "mqtt.thingsboard.cloud"
#define THINGSBOARD_PORT        1883
#define THINGSBOARD_TOKEN       "TOKEN_DEL_DEVICE_EN_THINGSBOARD"
#define THINGSBOARD_TOPIC       "v1/devices/me/telemetry"
#define MQTT_CLIENT_ID          "gateway-esp32"
```

Y dentro de mqtt_app_start usa la URI segun tu escenario:

```c
.broker = {
  .address = {
    .uri = "mqtt://mqtt.thingsboard.cloud:1883"
  }
}
```

Checklist rapido antes de flashear:

- WIFI_SSID actualizado.
- WIFI_PASS actualizado.
- THINGSBOARD_TOKEN actualizado (token del device activo).
- URI MQTT coincide con el puerto configurado.

## 4) Compilacion (ESP-IDF)

Desde la carpeta del proyecto:

1. Abrir terminal con entorno de ESP-IDF activo.
2. Ejecutar:

idf.py set-target esp32c6
idf.py build

## 5) Flasheo

Con la placa conectada (ajusta el puerto si no es COM13):

idf.py -p COM13 flash

## 6) Monitor serie

idf.py -p COM13 monitor

Para salir del monitor: Ctrl+]

## 7) Validacion esperada en monitor

Busca estas lineas:

- WiFi conectado, IP=...
- MQTT conectado
- Recibido paquete de ...
- Publicado mensaje MQTT id=...

Si aparece MQTT desconectado o Error MQTT, revisar token, host/puerto y conectividad a internet.

## 8) Ver datos en ThingsBoard

1. Ingresar a ThingsBoard.
2. Abrir el Device asociado al token cargado en el firmware.
3. Ir a Latest telemetry.
4. Verificar actualizacion de campos esperados, por ejemplo:

- src_mac
- batch_id
- node_start_timestamp_ms
- accel_x_g, accel_y_g, accel_z_g
- gyro_x_dps, gyro_y_dps, gyro_z_dps

## 9) Flujo recomendado al cambiar de red o entorno

1. Editar en main/config.h: WIFI_SSID, WIFI_PASS y THINGSBOARD_TOKEN.
2. Ejecutar nuevamente build y flash.
3. Abrir monitor y validar WiFi conectado y MQTT conectado.
4. Confirmar telemetria en ThingsBoard.

## 10) Solucion rapida de problemas

- No conecta a Wi-Fi:
  - Revisar SSID/password.
  - Verificar cobertura y banda compatible del AP.

- Conecta a Wi-Fi pero no a MQTT:
  - Revisar THINGSBOARD_TOKEN.
  - Revisar URI/puerto MQTT en mqtt_app_start.
  - Verificar que el device este activo en ThingsBoard.

- No llegan datos a ThingsBoard:
  - Confirmar que llegan paquetes ESP-NOW (linea Recibido paquete de ...).
  - Confirmar que aparece Publicado mensaje MQTT id=... en monitor.

## 11) Seguridad y produccion

Actualmente el proyecto esta usando MQTT sin TLS (puerto 1883) para simplificar pruebas.
Para produccion se recomienda migrar a MQTTS con validacion de certificado del servidor.
