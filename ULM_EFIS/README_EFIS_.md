<!-- VS Code: Ctrl+Shift+V para vista previa Markdown/KaTeX -->

# ULM‑EFIS con ESP32‑S3, BNO086, BMP280 y GPS

## 1. Descripción general

ULM‑EFIS es un sistema electrónico de instrumentos de vuelo basado en un
ESP32‑S3. El microcontrolador adquiere la actitud y las aceleraciones de un
BNO086, la presión y temperatura de un BMP280 y la navegación de un receptor
GPS. Después calcula las magnitudes derivadas y las presenta en un navegador
mediante una red Wi‑Fi propia, un servidor HTTP y un WebSocket.

La aplicación funciona sin router externo:

1. El ESP32‑S3 crea el punto de acceso `ESP32-FlightDisplay`.
2. La tableta, teléfono u ordenador se conecta a esa red.
3. El usuario abre `http://192.168.4.1`.
4. El navegador descarga la interfaz y abre `/ws` para recibir telemetría y
   enviar órdenes.

El BNO086 trabaja en **Game Rotation Vector (`GAME_RV`)**. Esta solución de
actitud no usa el magnetómetro, por lo que evita perturbaciones magnéticas. En
la configuración habitual, el girodireccional muestra el *Ground Track* GPS.

## 2. Prestaciones

### Instrumentos y datos mostrados

- Altímetro barométrico en pies, con lectura auxiliar en metros.
- Altitud GPS independiente.
- Variometro/VSI barométrico filtrado.
- Horizonte artificial con pitch y roll.
- Girodireccional basado en *Ground Track* GPS.
- Selector manual de curso mediante triángulo amarillo.
- Indicador de giro coordinado: bastón y bola.
- Ground Speed analógica en knots y lectura en km/h.
- Factor de carga actual, mínimo y máximo.
- Latitud, longitud, FIX, fecha y hora UTC.
- Temperatura medida por el BMP280.
- Estado WebSocket y contador de mensajes por segundo.

### Ajustes desde el navegador

- QNH en pasos de **0,1 hPa**.
- Corrección inicial de pitch.
- Curso/heading manual grado a grado.
- Montaje físico vertical `V` u horizontal `H`.
- Puesta a cero de G mínima y máxima.
- Repetición automática al mantener pulsados los botones.

QNH, offset de pitch, curso manual y montaje se guardan en NVS.

## 3. Arquitectura y arranque

```text
BNO086 -- I2C + H_INTN --+
                          |
BMP280 ------ I2C --------+--> ESP32-S3 --> HTTP + WebSocket/TCP --> navegador
                          |
GPS -------- UART1 -------+
```

`app_main()` coordina el arranque:

1. Mantiene el BNO086 inicialmente en reset.
2. Inicializa NVS con `nvs_flash_init()`.
3. Inicializa el bus compartido mediante `efis_i2c_init()`.
4. Arranca el BNO086 con `BNO086_start()`.
5. Arranca el BMP280 con `bmp280_start()`.
6. Arranca el GPS con `GPS_start()`.
7. Inicia opcionalmente `DataLogger_start()`.
8. Crea el AP con `wifi_ap_start()`.
9. Inicia HTTP mediante `webserver_start()`.
10. Inicia telemetría mediante `websocket_start_dummy_stream()`.
11. Configura potencia con `esp_wifi_set_max_tx_power()`.
12. Habilita el diagnóstico BOOT con `boot_button_start()`.

El I2C se comparte mediante mutex. `efis_i2c_lock()` y
`efis_i2c_unlock()` proporcionan exclusión mutua; `efis_i2c_probe()` busca un
dispositivo y `efis_i2c_reset_bus()` recupera el bus.

## 4. BNO086 y actitud

### 4.1 Adquisición SH‑2

El driver habilita a 25 Hz:

- acelerómetro;
- giróscopo;
- aceleración lineal;
- gravedad;
- Game Rotation Vector.

`bno086_int_isr()` despierta `BNO086Task()` cuando `H_INTN` baja. La tarea
vacía todos los paquetes SHTP pendientes y vuelve a dormir, sin polling.

`process_packet()` recorre cada paquete y devuelve una máscara de informes.
`process_sensor_report()` decodifica los informes. La muestra se publica cuando
se han reunido los cinco informes requeridos. Ante timeout o errores
consecutivos, `bno086_driver_recover()` reinicia y reconfigura el sensor.

### 4.2 Conversión de ejes

Se admiten `BNO086_MOUNT_VERTICAL` y `BNO086_MOUNT_HORIZONTAL`. Las funciones
de conversión al sistema del avión son:

- `acceleration_to_aircraft_axes()`;
- `gyro_to_aircraft_axes()`;
- `quaternion_to_aircraft_axes()`.

El montaje se cambia con `BNO086_set_mount_mode()` y se consulta mediante
`BNO086_get_mount_mode()`.

### 4.3 Pitch, roll y heading

`process_sensor_report()` obtiene el cuaternión de `GAME_ROTATION_VECTOR` y
`quaternion_to_euler()` calcula pitch, roll y yaw. La actitud procede de la
fusión SH‑2 del BNO086; no se utiliza el antiguo filtro complementario BNO055.

## 5. Indicador de giro coordinado

### 5.1 Bastón

`compute_vertical_turn_rate_dps()` proyecta el vector tridimensional del
giróscopo sobre la vertical obtenida del vector de gravedad:

$$
\omega_v=-(\omega_x g_x+\omega_y g_y+\omega_z g_z)
$$

Si la gravedad no es válida se utiliza el eje Z como respaldo.
`filter_yaw_rate()` aplica un paso bajo de primer orden:

$$
\alpha=1-e^{-\Delta t/\tau},\qquad
y_k=y_{k-1}+\alpha(x_k-y_{k-1})
$$

La dinámica se ajusta con `TURN_RATE_FILTER_TAU_S` y la zona muerta con
`TURN_RATE_DEADBAND_DPS`. En JavaScript, `turnAircraftAngle()` calcula el
ángulo gráfico y `updateTurnCoordinator()` mueve el bastón.

### 5.2 Bola

SH‑2 entrega la aceleración lineal sin gravedad. Después del cambio de ejes,
`compute_slip_ball_deg()` usa la componente lateral Y:

$$
a_{lat,g}=\frac{a_{linear,y}}{g_0},\qquad
\beta=-\arctan(a_{lat,g})
$$

`SLIP_BALL_GAIN` ajusta la ganancia y `SLIP_BALL_LIMIT_DEG` limita la salida.
`filter_slip_ball()` aplica el filtro definido por `SLIP_BALL_FILTER_TAU_S`.
La zona muerta `SLIP_BALL_DEADBAND_DEG` afecta solo a la salida, sin congelar
el estado interno. `slipBallPosition()` convierte el ángulo en desplazamiento.

## 6. G‑meter

`process_g_meter()` convierte los ejes del acelerómetro a G y calcula:

$$
G=\sqrt{G_x^2+G_y^2+G_z^2}
$$

Conserva mínimo y máximo acumulados. `g_peak_reset()` atiende `G_RESET` y
llama a `BNO086_reset_accel_peaks()`. `updateGText()` actualiza los tres valores
en la página.

## 7. BMP280: altímetro, temperatura y VSI

`bmp280_init()` inicializa el sensor y `BMP280Task()` realiza la adquisición.
Sus funciones principales son:

- `bmp280_read_measurement()`: presión y temperatura compensadas.
- `bmp280_altitude_m()`: altitud a partir de presión y QNH.
- `bmp280_vertical_speed_mps()`: derivada de altitud y filtrado del VSI.
- `bmp280_set_test_altitude()`: entrada de altitud para pruebas.

La altitud barométrica es:

$$
h=44330\left[1-\left(\frac{P}{QNH}\right)^{1/5.255}\right]
$$

El VSI emplea `VSI_FILTER_TAU_S`. En el navegador, `drawAltimeter()` dibuja el
altímetro; `variometerAngle()` y `drawVariometer()` representan el VSI.

## 8. GPS y navegación

El GPS usa UART1, normalmente a 115200 baud. `GPS_start()` crea `gps_task()`.

Funciones principales:

- `gps_uart_init()` y `gps_uart_set_baud()`: configuración UART.
- `gps_ubx_set_rate()`: frecuencia de navegación.
- `gps_ubx_configure_gga_rmc_only()`: activa GGA/RMC y desactiva el resto.
- `gps_ubx_save_configuration()`: guarda la configuración del receptor.
- `gps_nmea_checksum_valid()`: valida checksum.
- `gps_parse_gga()`: FIX, posición y altitud.
- `gps_parse_rmc()`: posición, Ground Speed, Ground Track y UTC.
- `gps_process_nmea_line()`: distribuye las sentencias.
- `GPS_get_data()`: obtiene una instantánea coherente.
- `GPS_is_connected()`: informa de comunicación con el receptor.

El track solo se actualiza con FIX y velocidad mayor que
`GPS_HEADING_MIN_SPEED_KT`; en otro caso se conserva el último válido.

En JavaScript:

- `normalizeHeading()` normaliza a `[0,360)`.
- `shortestAngleDifference()` evita vueltas gráficas largas.
- `createCompassRose()` genera la rosa.
- `updateHeadingInstrument()` actualiza rosa y selector de curso.
- `groundSpeedValueToAngle()` y `drawGroundSpeedGauge()` dibujan GS.

El triángulo amarillo representa el curso manual:

$$
\Delta\psi=\psi_{manual}-\psi_{GPS}
$$

## 9. Wi‑Fi y servidor HTTP

`wifi_ap_start()` inicializa `esp_netif`, crea el AP WPA2 y arranca Wi‑Fi.
`wifi_event_handler()` registra conexiones y desconexiones. La potencia máxima
se expresa en cuartos de dBm; por ejemplo, 32 equivale a 8 dBm.

`webserver_start()` inicia HTTP en el puerto 80 con purga LRU de sockets y
registra:

- `/`: `root_get_handler()` entrega `index.html` sin caché.
- `/ws`: endpoint registrado por `websocket_register_uri()`.
- `/download_log`: descarga CSV si el datalogger está habilitado.

## 10. WebSocket y reducción de latencia

`websocket_handler()` completa la conexión, activa `TCP_NODELAY` y procesa
órdenes. Desactivar Nagle evita retrasar mensajes TCP pequeños.

La IMU y la telemetría se sincronizan con una cola FreeRTOS de longitud uno:

- `BNO086_set_output_queue()` registra la cola.
- `BNO086Task()` publica mediante `xQueueOverwrite()`.
- `telemetry_task()` espera mediante `xQueueReceive()`.

Una muestra nueva sustituye a cualquier muestra pendiente. Además,
`websocket_queue_fast_json()` mantiene un único trabajo rápido pendiente y
`websocket_fast_broadcast_work()` envía la versión más reciente. Así se evita
la saturación `httpd_queue_work: ctrl socket queue full` al desconectar un
cliente o retrasarse el navegador.

Las tramas lentas usan `websocket_queue_json()` y
`websocket_broadcast_work()`. `websocket_broadcast_json()` envía a todos los
clientes activos y `websocket_count_clients()` evita producir telemetría si no
hay ninguno.

## 11. Protocolo JSON

### Trama rápida `F`

Se genera cuando llega una muestra nueva del BNO086:

```text
["F",roll,pitch,heading,turn_rate,slip,g,gmax,gmin]
```

### Navegación `N`, 2 Hz

```text
["N",altitude_m,vertical_speed_mps,temperature_c,gps_fix,
     gps_altitude_m,latitude,longitude,ground_speed_knots,utc_timestamp]
```

### Configuración `C`

Se envía al conectar o cuando cambia `cfg_version`:

```text
["C",qnh_hpa,pitch_offset_deg,heading_manual_deg,mount_mode]
```

Las genera `telemetry_task()`.

## 12. Órdenes y funciones asociadas

| Orden | Acción | Función C |
|---|---|---|
| `QNH_UP` / `QNH_DOWN` | QNH ±0,1 hPa | `qnh_change()` |
| `PITCH_OFFSET_UP` / `PITCH_OFFSET_DOWN` | Ajuste de pitch | `pitch_offset_change()` |
| `HEADING_OFFSET_UP` / `HEADING_OFFSET_DOWN` | Curso manual ±1° | `heading_offset_change()` |
| `MOUNT_MODE_V` / `MOUNT_MODE_H` | Montaje físico | `mount_mode_change()` |
| `G_RESET` | Reinicia extremos G | `g_peak_reset()` |
| `CFG_GET` | Solicita configuración | `settings_mark_changed()` |

`sendCommand()` transmite las órdenes desde el navegador y
`enablePressAndHold()` gestiona la pulsación mantenida.

## 13. Configuración persistente

`settings_load()` recupera QNH, pitch, curso manual y montaje. Las escrituras
se realizan con `settings_save_i32()` y `settings_save_u32()`.
`settings_get_snapshot()` obtiene una copia atómica para telemetría.

El QNH se almacena escalado por 100, pero actualmente se modifica y muestra con
una sola decimal.

## 14. Representación en el navegador

`connect()` crea el WebSocket, envía `CFG_GET` y reconecta tras un cierre.
`setConnectionState()` actualiza el LED y los controles.

- `updateFast()` procesa `F`.
- `updateNavigation()` procesa `N`.
- `updateConfiguration()` procesa `C`.
- `updateTelemetry()` valida y distribuye las tramas lentas.

Para tabletas antiguas, `socket.onmessage` conserva solamente la última `F` en
`latestFastData`. `animateAltimeter()`, ejecutada con
`requestAnimationFrame()`, aplica como máximo una muestra rápida por refresco.
Las posiciones antiguas se descartan. El horizonte, coordinador y textos G se
redibujan únicamente al aplicar una `F` nueva.

Funciones del horizonte:

- `drawHorizonBezel()`;
- `drawHorizonPitchLadder()`;
- `drawHorizonGroundPerspective()`;
- `drawHorizonMovingSphere()`;
- `drawHorizonBankScale()`;
- `drawHorizonAircraftSymbol()`;
- `drawHorizonKnob()`;
- `drawHorizonFrame()`.

## 15. Estadísticas RTOS mediante BOOT

BOOT está conectado a `GPIO0`. `boot_button_task()` realiza antirrebote y una
sola impresión por pulsación. `print_rtos_statistics()` usa
`vTaskGetRunTimeStats()` para mostrar tiempo y porcentaje de CPU de cada tarea.

Requiere:

```text
CONFIG_FREERTOS_USE_TRACE_FACILITY=y
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
```

## 16. Datalogger opcional

Se incluye solo si `DATALOGGER_ENABLED` está activo:

- `DataLogger_start()` monta SPIFFS y crea `datalogger_task()`.
- `DataLogger_begin_recording()` crea el CSV si IMU, FIX y UTC son válidos.
- `datalogger_task()` registra UTC, aceleración Z y presión.
- `DataLogger_stop_recording()` vacía y cierra el fichero.
- `DataLogger_get_status()` informa de estado, muestras y tamaño.
- `download_log_get_handler()` descarga el CSV si no se está grabando.

Se ejecuta `fflush()` tras cada muestra para minimizar pérdidas ante apagado.

## 17. Robustez

- Mutex para el I2C compartido.
- Recuperación del bus y del BNO086.
- Lectura por interrupción, no por polling.
- Cola IMU de longitud uno.
- Un solo trabajo WebSocket rápido pendiente.
- `TCP_NODELAY` en cada socket WebSocket.
- Purga LRU de sockets HTTP.
- Verificación del checksum NMEA.
- Rechazo de valores no finitos en los filtros.
- No se recuperan periodos de navegación atrasados tras una desconexión.

## 18. Módulos y funciones principales

| Archivo | Responsabilidad | Funciones destacadas |
|---|---|---|
| `main.c` | Arranque y BOOT | `app_main()`, `boot_button_start()`, `print_rtos_statistics()` |
| `EFIS_I2C.c` | Bus compartido | `efis_i2c_init()`, `efis_i2c_lock()`, `efis_i2c_reset_bus()` |
| `BNO086_driver.c` | SHTP/SH‑2 | `bno086_driver_init()`, `bno086_driver_receive_packet()`, `bno086_driver_recover()` |
| `BNO086.c` | Actitud y magnitudes derivadas | `BNO086Task()`, `process_packet()`, `compute_vertical_turn_rate_dps()`, `compute_slip_ball_deg()` |
| `BMP280.c` | Presión, altitud y VSI | `BMP280Task()`, `bmp280_altitude_m()`, `bmp280_vertical_speed_mps()` |
| `GPS.c` | UBX/NMEA y navegación | `GPS_start()`, `gps_task()`, `gps_parse_gga()`, `gps_parse_rmc()` |
| `wifi_ap.c` | Access Point | `wifi_ap_start()`, `wifi_event_handler()` |
| `webserver.c` | HTTP | `webserver_start()`, `root_get_handler()` |
| `websocket.c` | Órdenes, NVS y telemetría | `websocket_handler()`, `telemetry_task()`, `websocket_queue_fast_json()` |
| `index.html` | Interfaz gráfica | `connect()`, `animateAltimeter()`, `updateFast()`, `drawHorizonFrame()` |
| `DataLogger.c` | CSV opcional | `DataLogger_start()`, `DataLogger_begin_recording()`, `DataLogger_stop_recording()` |
| `config.h` | Parámetros | Periodos, filtros, pines, potencia, rangos y buffers |

## 19. API pública

### BNO086

```c
esp_err_t BNO086_start(void);
bno086_data_t BNO086_get_data(void);
void BNO086_set_output_queue(QueueHandle_t queue);
void BNO086_reset_accel_peaks(void);
esp_err_t BNO086_set_mount_mode(bno086_mount_mode_t mode);
bno086_mount_mode_t BNO086_get_mount_mode(void);
```

### GPS y WebSocket

```c
esp_err_t GPS_start(void);
gps_data_t GPS_get_data(void);
bool GPS_is_connected(void);

esp_err_t websocket_register_uri(httpd_handle_t server);
esp_err_t websocket_start_dummy_stream(httpd_handle_t server);
float websocket_get_qnh(void);
```

## 20. Parámetros relevantes de `config.h`

- `BNO086_REPORT_INTERVAL_US`: periodo SH‑2.
- `TURN_RATE_FILTER_TAU_S`: filtro del bastón.
- `SLIP_BALL_FILTER_TAU_S`: filtro de la bola.
- `VSI_FILTER_TAU_S`: filtro del VSI.
- `GPS_TARGET_RATE_MS`: periodo GPS.
- `GPS_HEADING_MIN_SPEED_KT`: umbral de track.
- `QNH_STEP_HPA`: paso del QNH.
- `WIFI_TX_POWER_QDBM`: potencia en cuartos de dBm.
- `MAX_WS_CLIENTS`: clientes WebSocket.
- `JSON_BUFFER_SIZE`: longitud máxima de trama.
- `DATALOGGER_ENABLED`: datalogger opcional.

Una `TAU` mayor proporciona más filtrado y menor frecuencia de corte, a costa
de mayor retraso.

## 21. Consideraciones operativas

- El girodireccional muestra *Ground Track*, no heading magnético.
- `GAME_RV` evita el magnetómetro, pero su yaw no es una referencia absoluta.
- Los signos del coordinador deben validarse físicamente en montajes V y H.
- USB presenta normalmente menor latencia y variabilidad que Wi‑Fi.
- En Wi‑Fi también influyen el ahorro de energía, navegador y capacidad gráfica
  de la tableta.
- Se descartan muestras rápidas antiguas deliberadamente: es preferible mostrar
  el estado actual que reproducir datos retrasados.

## 22. Requisitos de compilación

- ESP32‑S3 y ESP‑IDF.
- WebSocket HTTP habilitado:

```text
CONFIG_HTTPD_WS_SUPPORT=y
```

- Las opciones FreeRTOS de la sección 15 para estadísticas de CPU.
- Partición SPIFFS únicamente si se habilita el datalogger.

