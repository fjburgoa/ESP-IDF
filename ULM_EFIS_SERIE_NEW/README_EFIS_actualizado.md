<!-- VS Code: Ctrl+Shift+V para vista previa Markdown/KaTeX -->

# ESP32-S3 Electronic Flight Instrument System (EFIS)

## 1. Descripción general

Este proyecto implementa un EFIS de bajo coste basado en ESP32-S3. El
microcontrolador adquiere los sensores, realiza el procesamiento,
filtrado y cálculo de las variables de vuelo y transmite la telemetría
ya procesada mediante JSON/WebSocket a un navegador en tablet, teléfono
o PC.

Sensores principales:

- **BMP280/BME280**: presión, temperatura, altitud barométrica y VSI.
- **BNO055**: acelerómetro, giróscopo y magnetómetro.
- **u-blox NEO-M10/M10M**: posición, UTC, altitud GPS, Ground Speed y
  Ground Track. La comunicación GNSS se realiza por UART.

La interfaz presenta actualmente **seis instrumentos principales**. Las
G mínima, actual y máxima se muestran como texto.

## 2. Arquitectura

```text
BMP280/BME280 -----------+
                         |
BNO055 ------------------+--> ESP32-S3 --> JSON / WebSocket --> index.html
                         |
NEO-M10/M10M -- UART -----+
```

El ESP32-S3 calcula o acondiciona: altitud barométrica, velocidad
vertical, pitch, roll, régimen de giro, bola, G, Ground Speed, Ground
Track y configuración persistente.

## 3. BNO055: NDOF y AMG

La selección se realiza con `BNO055_USE_INTERNAL_FUSION`.

### NDOF

Con:

```c
#define BNO055_USE_INTERNAL_FUSION 1
```

se utiliza la fusión interna del BNO055. Están disponibles Euler,
cuaternión, gravedad y aceleración lineal:

```text
heading_deg
roll_deg
pitch_deg
quaternion
gravity_ms2
linear_acceleration_ms2
```

Se conserva esta opción para compatibilidad y pruebas.

### AMG --- configuración preferida actual

Con:

```c
#define BNO055_USE_INTERNAL_FUSION 0
```

se leen acelerómetro, giróscopo y magnetómetro sin utilizar la solución
NDOF. En este modo:

- pitch y roll se estiman mediante un filtro complementario propio;
- el régimen de giro se calcula a partir de los tres ejes del
  giróscopo;
- la componente gravitatoria lateral se elimina por software para
  calcular la bola;
- el heading principal de la interfaz procede del GPS, no del
  magnetómetro/BNO055.

## 4. Ejes y orientación física

Sistema lógico:

```text
X = transversal
Y = longitudinal
Z = vertical
```

Se soportan dos montajes:

- **V**: P1/default.
- **H**: P0, giro de 90° en XY.

Para H/P0:

```text
X_aircraft = -Y_sensor
Y_aircraft =  X_sensor
Z_aircraft =  Z_sensor
```

La selección V/H se realiza desde la web, es persistente en NVS y por
defecto se utiliza V. Al cambiar de orientación se reinician los estados
del estimador y de los filtros para no mezclar sistemas de referencia.

## 5. Horizonte artificial: ecuaciones de pitch y roll

### 5.1 Estimación con acelerómetro

En AMG se calcula:

$$
\theta_a =
\operatorname{atan2}
\left(
a_y,\sqrt{a_x^2+a_z^2}
\right)
$$

$$
\phi_a =
\operatorname{atan2}
\left(
a_x,\sqrt{a_y^2+a_z^2}
\right)
$$

donde:

- $\theta_a$ es el pitch obtenido del acelerómetro.
- $\phi_a$ es el roll obtenido del acelerómetro.

En C:

```c
pitch_acc_deg = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
roll_acc_deg  = atan2f(ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
```

### 5.2 Integración del giróscopo

La predicción de pitch mediante el giróscopo es:

$$
\theta_g[k]
=
\theta[k-1]
+
S_\theta\,\omega_x\,\Delta t
$$

La predicción de roll es:

$$
\phi_g[k]
=
\phi[k-1]
+
S_\phi\,\omega_y\,\Delta t
$$

donde:

- $\omega_x$ es la velocidad angular alrededor del eje X.
- $\omega_y$ es la velocidad angular alrededor del eje Y.
- $S_\theta$ y $S_\phi$ permiten adaptar el signo a nuestro convenio de ejes.
- $\Delta t$ es el periodo de muestreo.

Actualmente se ha probado:

```c
#define BNO055_ROLL_GYRO_SIGN -1.0f
```

### 5.3 Filtro complementario

El coeficiente del filtro es:

$$
\alpha =
\frac{\tau_{att}}
{\tau_{att}+\Delta t}
$$

La estimación final de pitch es:

$$
\theta[k]
=
\alpha\,\theta_g[k]
+
(1-\alpha)\,\theta_a[k]
$$

La estimación final de roll es:

$$
\phi[k]
=
\alpha\,\phi_g[k]
+
(1-\alpha)\,\phi_a[k]
$$

Valor actual de ensayo:

```c
#define BNO055_ATTITUDE_TAU_S 0.3f
```

El giróscopo proporciona la respuesta rápida y el acelerómetro aporta la referencia de vertical a largo plazo.

## 6. Indicador de giro

Usar únicamente `gyro_z_dps` solo es correcto cuando el eje Z coincide con la vertical local. Con roll y/o pitch, el giro alrededor de la vertical se reparte entre $\omega_x$, $\omega_y$ y $\omega_z$.

### 6.1 Filtrado previo de los tres giróscopos

Se aplica el mismo filtro paso bajo de primer orden a los tres ejes.

El coeficiente del filtro es:

$$
\alpha_g
=
1-e^{-\Delta t/\tau_g}
$$

Para cada eje $i \in \{x,y,z\}$:

$$
\omega_{i,f}[k]
=
\omega_{i,f}[k-1]
+
\alpha_g
\left(
\omega_i[k]
-
\omega_{i,f}[k-1]
\right)
$$

Configuración actual:

```c
#define GYRO_FILTER_TAU_S 0.6f
```

Filtrar X, Y y Z con la misma dinámica evita introducir diferencias de fase antes de la proyección.

### 6.2 Reconstrucción de la vertical

A partir de las ecuaciones actuales de actitud:

$$
\hat g_x = \sin\phi
$$

$$
\hat g_y = \sin\theta
$$

Como $\hat{\mathbf g}$ es un vector unitario:

$$
\hat g_x^2 + \hat g_y^2 + \hat g_z^2 = 1
$$

por lo que:

$$
\hat g_z
=
\sqrt{
1-\hat g_x^2-\hat g_y^2
}
$$

y finalmente:

$$
\hat{\mathbf g}
=
\begin{bmatrix}
\sin\phi \\
\sin\theta \\
\sqrt{1-\sin^2\phi-\sin^2\theta}
\end{bmatrix}
$$

Esta expresión se utiliza en el rango normal de funcionamiento con el instrumento aproximadamente erguido.

### 6.3 Proyección de la velocidad angular

El vector de velocidades angulares filtradas es:

$$
\boldsymbol{\omega}_f
=
\begin{bmatrix}
\omega_{x,f} \\
\omega_{y,f} \\
\omega_{z,f}
\end{bmatrix}
$$

La componente alrededor de la vertical local es el producto escalar:

$$
\omega_v
=
\boldsymbol{\omega}_f
\cdot
\hat{\mathbf g}
$$

Desarrollando:

$$
\boxed{
\omega_v
=
\omega_{x,f}\sin\phi
+
\omega_{y,f}\sin\theta
+
\omega_{z,f}
\sqrt{
1-\sin^2\phi-\sin^2\theta
}
}
$$

El signo final se adapta únicamente a la convención gráfica del indicador.

### 6.4 Filtrado final del régimen de giro

Después de la proyección se aplica un segundo filtro paso bajo.

Su coeficiente es:

$$
\alpha_t
=
1-e^{-\Delta t/\tau_t}
$$

y la salida filtrada es:

$$
\omega_{turn}[k]
=
\omega_{turn}[k-1]
+
\alpha_t
\left(
\omega_v[k]
-
\omega_{turn}[k-1]
\right)
$$

Configuración actual:

```c
#define TURN_RATE_FILTER_TAU_S 0.75f
#define TURN_RATE_DEADBAND_DPS 0.10f
```

Cadena completa:

```text
gyro X/Y/Z
    |
    v
LPF XYZ, tau = 0.6 s
    |
    v
proyección sobre la vertical estimada
    |
    v
deadband
    |
    v
LPF turn-rate, tau = 0.75 s
    |
    v
indicador de giro
```

## 7. Bola de resbale/deslizamiento

En AMG no disponemos directamente de `linear_acceleration_ms2` calculada por la fusión NDOF. La bola debe responder a la **aceleración lateral no gravitatoria**, no a `accel_x_g` total.

El acelerómetro mide, en el eje transversal X:

$$
a_{x,\mathrm{meas}}
=
a_{x,\mathrm{linear}}
+
g_x
$$

Con X transversal y la convención actual de roll:

$$
\frac{g_x}{g}
=
\sin\phi
$$

Si expresamos la aceleración en unidades de G:

$$
g_{x,g}
=
\sin\phi
$$

Por tanto, la aceleración lateral no gravitatoria es:

$$
\boxed{
a_{\mathrm{lat},g}
=
a_{x,g}
-
\sin\phi
}
$$

Esta magnitud reconstruye por software el equivalente de la componente X de `linear_acceleration_ms2`, expresada en G.

El ángulo equivalente de bola se obtiene mediante:

$$
\boxed{
\beta
=
\arctan
\left(
a_{\mathrm{lat},g}
\right)
}
$$

Ejemplos:

```text
0.0 G ->  0.00 deg
0.1 G ->  5.71 deg
0.2 G -> 11.31 deg
```

Después se aplican signo gráfico, deadband, filtro paso bajo y saturación:

```c
#define SLIP_BALL_FILTER_TAU_S 1.5f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f
```

La llamada actual es conceptualmente:

```c
bno055_compute_slip_ball_deg(
    data.accel_x_g,
    data.roll_deg,
    dt_s);
```

## 8. G-meter

La G actual se calcula como el módulo del vector de aceleración:

$$
\boxed{
G
=
\sqrt{
G_x^2+G_y^2+G_z^2
}
}
$$

`Gmin` y `Gmax` son los mínimos y máximos absolutos acumulados desde el último reset.

En la interfaz se muestran en una única línea:

```text
Gmin (verde) - G actual (blanco) - Gmax (azul)
```

El antiguo instrumento analógico de G fue sustituido por el Ground Speed.

## 9. BMP280: altitud y VSI

El BMP280 proporciona presión y temperatura. La altitud se obtiene a partir de presión y QNH:

$$
h=f(P,QNH)
$$

El VSI se calcula en `BMP280.c`:

$$
v_z
=
\frac{dh}{dt}
$$

y se filtra antes de enviarlo al navegador.

## 10. GPS/GNSS

El receptor actualmente utilizado es un **u-blox NEO-M10/M10M**. La comunicación con el ESP32-S3 se realiza por **UART1**:

```text
ESP32-S3                 GNSS
--------                  ----
GPIO7  UART1 RX  <------- TX
GPIO5  UART1 TX  -------> RX
GND              -------- GND
5 V              -------- VCC
```

Configuración actual:

```c
#define GPS_INITIAL_BAUD_RATE 115200U
#define GPS_TARGET_BAUD_RATE  115200U
#define GPS_TARGET_RATE_MS    200U
```

El receptor trabaja a **115200 baud** y la navegación se configura a **5 Hz**. Se mantienen activas las sentencias NMEA GGA y RMC. El driver conserva la lógica de cambio de baud rate para mantener compatibilidad con receptores anteriores, como el NEO-6M, que pueden arrancar a 9600 baud.

El GPS proporciona latitud, longitud, altitud GPS, Ground Speed, Ground Track / Course Over Ground, FIX, satélites, HDOP y fecha/hora UTC.

La fecha y hora UTC se almacenan de forma compacta como:

```c
uint32_t utc_timestamp;
```

El navegador realiza la conversión a fecha y hora legibles.

### Nota sobre módulos NEO-M10 comerciales

En el módulo ensayado aparecen los pines `VCC`, `GND`, `TX`, `RX`, `SDA` y `SCL`. Los pines `SDA/SCL` corresponden a un magnetómetro auxiliar en I2C (`0x0D`), mientras que el receptor GNSS M10 se comunica por UART.

En el bus I2C del prototipo se han observado:

```text
0x29 -> BNO055
0x76 -> BMP280/BME280
0x0D -> magnetómetro auxiliar del módulo GNSS
```

El magnetómetro auxiliar del módulo GNSS no se utiliza actualmente.

### Ground Speed

Existe un instrumento analógico dedicado **GS**, en knots, con leyenda `KNOTS`.

### Girodireccional / Ground Track

El girodireccional utiliza el **Ground Track del GPS**, no el heading de la IMU. Solo se acepta una nueva muestra cuando existe FIX válido y la velocidad supera el umbral:

```c
#define GPS_HEADING_MIN_SPEED_KT 5.0f
```

Si se pierde temporalmente el heading GPS, se conserva el último Ground Track válido; si nunca se recibió uno válido, se muestra 0°. Técnicamente el GPS proporciona track/course over ground, no heading aerodinámico.

### Triángulo amarillo / course bug

El triángulo amarillo es un heading/course manual seleccionado por el usuario mediante `+` y `-` y almacenado de forma persistente.

$$
\boxed{
\Delta\psi = \psi_{\mathrm{manual}} - \psi_{\mathrm{GPS}}
}
$$

Cuando ambos coinciden, el triángulo queda a las 12. Por compatibilidad, la variable interna puede conservar el nombre histórico `heading_offset_deg`.

## 11. Seis instrumentos principales actuales

1.  **Altímetro**: altitud barométrica ft/m y altitud GPS.
2.  **VSI**: velocidad vertical barométrica filtrada.
3.  **Horizonte artificial**: pitch y roll calculados en AMG mediante
    filtro complementario.
4.  **Girodireccional**: Ground Track GPS + course bug amarillo manual.
5.  **Bola y bastón / Turn Coordinator**: régimen de giro y bola.
6.  **Ground Speed**: velocidad GPS en knots.

Además se muestran Gmin/G/Gmax, UTC, latitud/longitud y otra información
de diagnóstico/configuración.

## 12. Diseño de la interfaz para tablet

La zona prioritaria de la página incluye los seis instrumentos y los
datos de navegación hasta latitud/longitud.

Información menos prioritaria, como temperatura, diagnóstico y selección
de orientación, puede quedar por debajo del viewport y accederse
mediante scroll. Esto evita reducir excesivamente el tamaño de los
instrumentos en tablets con barra de navegación visible.

El contador, mensajes/s y tiempo se agrupan de forma compacta en el
borde de la interfaz.

## 13. Configuración persistente

Actualmente se contemplan, según la build:

- QNH;
- pitch offset;
- heading/course manual;
- orientación BNO055 V/H.

Los parámetros se almacenan en NVS.

## 14. Datalogger

La infraestructura del datalogger se conserva, pero su funcionalidad C
puede desactivarse mediante una opción de compilación en `config.h`.

Las partes visuales de `index.html` y campos JSON asociados pueden
permanecer aunque el logger esté desactivado. De esta forma no es
necesario eliminar código para realizar ensayos sin adquisición de
datos.

## 15. Variables principales

### BMP280

```text
pressure_hPa
temperature_C
altitude_m
verticalSpeed_ms
```

### BNO055 / inerciales

```text
accelX_g
accelY_g
accelZ_g
accelTotal_g

gyroX_dps
gyroY_dps
gyroZ_dps

magX_uT
magY_uT
magZ_uT

pitch_deg
roll_deg
heading_deg

turnRate_dps
slip_deg

gCurrent
gMax
gMin
```

En NDOF también pueden existir:

```text
quaternion
gravity_ms2
linear_acceleration_ms2
```

En AMG esos datos fusionados no se utilizan; las magnitudes necesarias
se reconstruyen localmente.

### GPS

```text
gpsConnected
gpsFixValid
gpsLatitude_deg
gpsLongitude_deg
gpsAltitude_m
gpsGroundSpeed_knots
gpsGroundTrack_deg
gpsUtcTimestamp
```

## 16. Módulos de software

- `BMP280.c`: presión, temperatura, altitud y VSI.
- `BNO055.c`: tarea periódica, adquisición, estado compartido y API pública.
- `BNO055_driver.c`: I2C, registros, detección, modos de operación, unidades, remapeo V/H y conversión a unidades físicas.
- `BNO055_processing.c`: filtro complementario, filtrado del giróscopo, régimen de giro, bola y G-meter.
- `BNO055.h`: interfaz pública y tipos comunes.
- `BNO055_driver.h` / `BNO055_processing.h`: interfaces internas.
- `GPS.c`: UART, UBX/NMEA, posición, UTC, Ground Speed y Ground Track.
- `wifi_ap.c`: Access Point ESP32-S3.
- `webserver.c`: servidor HTTP.
- `websocket.c`: telemetría JSON, comandos, persistencia y lógica de heading GPS.
- `index.html`: representación gráfica del EFIS.
- `main.c`: inicialización.
- `config.h`: opciones de compilación.

Las magnitudes vectoriales del BNO055 se representan mediante `bno055_vector3f_t`, incluyendo aceleración, gravedad, campo magnético y velocidad angular (`gyro_dps`).

## 17. Resumen del procesamiento

```text
ACCEL + GYRO X/Y
       |
       v
filtro complementario
       |
       +--------------------> pitch / roll

GYRO X/Y/Z
       |
       v
LPF tau=0.6 s
       |
       v
proyección sobre vertical estimada
       |
       v
deadband + LPF tau=0.75 s
       |
       +--------------------> indicador de giro

ACCEL X + roll
       |
       v
accel_x_g - sin(roll)
       |
       v
atan()
       |
       v
deadband + LPF + saturación
       |
       +--------------------> bola

GPS Ground Speed ----------> GS
GPS Ground Track ----------> girodireccional
Heading manual ------------> triángulo amarillo

BMP280 P --> altitud --> derivada --> LPF --> VSI
```

## 18. Estado de validación

Pruebas realizadas en estático y en automóvil:

- altitud barométrica: buen comportamiento;
- VSI: buen comportamiento;
- GS GPS: muy buen comportamiento;
- Ground Track GPS en girodireccional: buen comportamiento;
- indicador de giro: funcional, actualmente en ajuste de filtrado;
- horizonte artificial: estimador AMG propio basado en acelerómetro + giróscopo, actualmente con `BNO055_ATTITUDE_TAU_S = 0.3 s`;
- bola: revisada para eliminar la componente lateral de gravedad;
- NEO-M10/M10M: comunicación UART validada a 115200 baud y configuración a 5 Hz;
- telemetría WebSocket: aproximadamente 15 Hz.

### Incidencia de montaje detectada en el BNO055

Durante el montaje de una nueva placa se observó un fallo aparente del acelerómetro:

```text
ACC_ID = 0x00
ACC RAW = 00 00 00 00 00 00
```

mientras giróscopo y magnetómetro continuaban funcionando. La causa fue mecánica: los tornillos M3 que sujetaban el módulo a una base de PVC estaban demasiado apretados y producían flexión del PCB.

Al reducir la precarga de los tornillos se recuperó inmediatamente:

```text
ACC_ID = 0xFB
ACC RAW != 0
```

Los módulos de sensores deben fijarse sin flexionar la PCB. Se recomienda utilizar separadores y una precarga moderada, preferiblemente con tornillería o arandelas de nylon.

Las pruebas en automóvil son útiles para desarrollo, pero sus dinámicas no son equivalentes a las de una aeronave.

## 19. Pendiente

- validar en carretera el filtro complementario AMG con `BNO055_ATTITUDE_TAU_S = 0.3 s`;
- ajustar definitivamente `GYRO_FILTER_TAU_S` y `TURN_RATE_FILTER_TAU_S`;
- estudiar la configuración explícita del giróscopo BNO055: rango ±500 deg/s frente a ±2000 deg/s y bandwidth;
- validar físicamente el signo y dinámica de la bola;
- validar la reconstrucción de aceleración lateral lineal;
- probar el NEO-M10/M10M durante recorridos dinámicos;
- futura entrada pitot/estática para airspeed;
- density altitude;
- alarmas;
- diagnóstico/calibración;
- terrain awareness;
- reactivar/ampliar el datalogger cuando vuelva a ser necesario.

## 20. Configuración de build y depuración

El servidor WebSocket de ESP-IDF requiere habilitar:

```text
CONFIG_HTTPD_WS_SUPPORT=y
```

Conviene mantener esta opción también en `sdkconfig.defaults` para que no se pierda al regenerar `sdkconfig` o realizar una limpieza completa del proyecto.

La ESP32-S3-DevKitC-1 permite utilizar **USB Serial/JTAG** como alternativa al convertidor USB-UART de la placa para programación, monitorización y depuración.

## 21. Herramienta útil


ESPConnect:

https://thelastoutpostworkshop.github.io/ESPConnect/
