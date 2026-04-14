# RI-Proyecto-G3: Sistema de Nodos Sensores IoT

Sistema embebido enfocado en eficiencia energetica usando ESP32-C6 y ESP-IDF v6.0 en C puro.

## Arquitectura del Repositorio
Este repositorio esta organizado como un monorepo con dos proyectos independientes de ESP-IDF:

- firmware_nodo: firmware orientado a captura y preparacion de datos de sensores.
- firmware_gateway: firmware orientado a recepcion de tramas por ESP-NOW y coordinacion de datos recibidos.

Estructura esperada:

- RI-Proyecto-G3/
- RI-Proyecto-G3/firmware_nodo/
- RI-Proyecto-G3/firmware_gateway/

## Detalles del Nodo (firmware_nodo)
El firmware del nodo implementa una ruta de adquisicion y temporizacion para telemetria IoT de bajo consumo:

- Lectura del MPU6050 (Acelerometro y Giroscopio) mediante bus I2C nativo.
- Pines I2C definidos en GPIO 6 y GPIO 7.
- Sincronizacion de tiempo real por SNTP contra time.google.com.
- Uso de Timestamp y Unix Epoch para etiquetar mediciones.
- Persistencia de referencia temporal en el RTC interno.
- Subrutina de feedback visual inicial con LED RGB para confirmar estado de arranque.

## Guia de Trabajo para el Equipo
Para evitar problemas de entorno y mantener consistencia entre miembros:

1. En VS Code, abrir directamente la subcarpeta del firmware que se va a trabajar:
   - Abrir firmware_nodo para tareas del nodo.
   - Abrir firmware_gateway para tareas del gateway.
2. No abrir la raiz del monorepo para compilar con la extension de ESP-IDF, ya que la extension funciona mejor cuando el proyecto ESP-IDF abierto es unico.
3. No subir credenciales WiFi ni secretos al repositorio.
4. Configurar parametros sensibles por menuconfig o mediante macros locales fuera de control de versiones.
5. Mantener commits separados por firmware para facilitar revisiones y despliegues.

## Nota de Version
Proyecto base alineado con ESP-IDF v6.0 sobre ESP32-C6.
