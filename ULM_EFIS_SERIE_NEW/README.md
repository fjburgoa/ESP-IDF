# ESP32-S3 Electronic Flight Instrument System (EFIS)

## 1. Descripción general

Este proyecto implementa un EFIS de bajo coste basado en ESP32-S3. El
microcontrolador adquiere los sensores, realiza el procesamiento,
filtrado y cálculo de las variables de vuelo y transmite la telemetría
ya procesada mediante JSON/WebSocket a un navegador en tablet, teléfono
o PC.

Sensores principales:

-   **BMP280**: presión, temperatura, altitud barométrica y VSI.
-   **BNO055**: acelerómetro, giróscopo y magnetómetro.
-   **GPS/GNSS**: posición, UTC, altitud GPS, Ground Speed y Ground
    Track.

La interfaz presenta actualmente **seis instrumentos principales**. Las
G mínima, actual y máxima se muestran como texto.

## 2. Arquitectura

``` text
BMP280 ------------------+
                         |
BNO055 ------------------+--> ESP32-S3 --> JSON / WebSocket --> index.html
                         |
GPS / GNSS --------------+
```

El ESP32-S3 calcula o acondiciona: altitud barométrica, velocidad
vertical, pitch, roll, régimen de giro, bola, G, Ground Speed, Ground
Track y configuración persistente.

## 3. BNO055: NDOF y AMG

La selección se realiza con `BNO055_USE_INTERNAL_FUSION`.

### NDOF

Con:

``` c
#define BNO055_USE_INTERNAL_FUSION 1
```

se utiliza la fusión interna del BNO055. Están disponibles Euler,
cuaternión, gravedad y aceleración lineal:

``` text
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

``` c
#define BNO055_USE_INTERNAL_FUSION 0
```

se leen acelerómetro, giróscopo y magnetómetro sin utilizar la solución
NDOF. En este modo:

-   pitch y roll se estiman mediante un filtro complementario propio;
-   el régimen de giro se calcula a partir de los tres ejes del
    giróscopo;
-   la componente gravitatoria lateral se elimina por software para
    calcular la bola;
-   el heading principal de la interfaz procede del GPS, no del
    magnetómetro/BNO055.

## 4. Ejes y orientación física

Sistema lógico:

``` text
X = transversal
Y = longitudinal
Z = vertical
```

Se soportan dos montajes:

-   **V**: P1/default.
-   **H**: P0, giro de 90° en XY.

Para H/P0:

``` text
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
\theta\_a =
\operatorname{atan2}\left(a_y,\sqrt{a_x^2+a_z^2}\right)
$$

$$
\phi\_a =
\operatorname{atan2}\left(a_x,\sqrt{a_y^2+a_z^2}\right)
$$

donde (\theta) es pitch y (\phi) es roll.

En C:

``` c
pitch_acc_deg = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG;
roll_acc_deg  = atan2f(ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
```

### 5.2 Integración del giróscopo

$$
\theta*g\[k
$$ = \theta[k-1] +
S*\theta,\omega\_x,\Delta t \]

$$
\phi*g\[k
$$ = \phi[k-1] +
S*\phi,\omega\_y,\Delta t \]

Actualmente se ha probado:

``` c
#define BNO055_ROLL_GYRO_SIGN -1.0f
```

### 5.3 Filtro complementario

$$
\alpha = \frac{\tau_{att}}{\tau_{att}+\Delta t}
$$

$$
\theta[k]=\alpha\theta\_g\[k
$$+(1-\alpha)\theta\_a$$
k
$$
\]

$$
\phi[k]=\alpha\phi\_g\[k
$$+(1-\alpha)\phi\_a$$
k
$$
\]

Valor actual de ensayo:

``` c
#define BNO055_ATTITUDE_TAU_S 1.0f
```

El giróscopo proporciona la respuesta rápida y el acelerómetro aporta la
referencia de vertical a largo plazo.

## 6. Indicador de giro

Usar únicamente `gyro_z_dps` solo es correcto cuando Z coincide con la
vertical local. Con roll/pitch, el giro alrededor de la vertical se
reparte entre (\omega\_x,\omega\_y,\omega\_z).

### 6.1 Filtrado previo de los tres giróscopos

Se aplica el mismo LPF de primer orden a X, Y y Z:

$$
\alpha\_g=1-e\^{-\Delta t/\tau\_g}
$$

$$
\omega*{i,f}\[k
$$= \omega*{i,f}$$
k-1
$$+
\alpha\_g\left(\omega*i$$
k
$$-\omega*{i,f}$$
k-1
$$\right)
\]

para (i=x,y,z).

Configuración actual:

``` c
#define GYRO_FILTER_TAU_S 0.15f
```

Filtrar los tres ejes con la misma dinámica evita introducir diferencias
de fase antes de la proyección.

### 6.2 Reconstrucción de la vertical

Las ecuaciones actuales de actitud permiten escribir:

$$
\hat g_x=\sin\phi
$$

$$
\hat g_y=\sin\theta
$$

y, al ser un vector unitario:

$$
\hat g_z= \sqrt{1-\hat g_x^2-\hat g_y^2}
$$

Por tanto:

$$
\hat{\mathbf g}=
```
\begin{bmatrix}
\sin\phi\\
\sin\theta\\
\sqrt{1-\sin^2\phi-\sin^2\theta}
\end{bmatrix}
```
$$

en el rango normal de operación con el equipo aproximadamente erguido.

### 6.3 Proyección de la velocidad angular

$$
\boldsymbol{\omega}\_f=
```
\begin{bmatrix}
\omega_{x,f}\\
\omega_{y,f}\\
\omega_{z,f}
\end{bmatrix}
```
$$

La componente alrededor de la vertical local es:

$$
\omega\_v=
\boldsymbol{\omega}\_f\cdot\hat{\mathbf g}
$$

es decir:

$$
`\boxed{
\omega_v=
\omega_{x,f}\sin\phi+
\omega_{y,f}\sin\theta+
\omega_{z,f}
\sqrt{1-\sin^2\phi-\sin^2\theta}
}`
$$

El signo final se adapta únicamente a la convención gráfica del
instrumento.

### 6.4 Filtrado final del régimen de giro

Después de la proyección se mantiene un segundo LPF:

$$
\alpha\_t=1-e\^{-\Delta t/\tau\_t}
$$

$$
\omega*{turn}\[k
$$= \omega*{turn}$$
k-1
$$+
\alpha\_t(\omega*v$$
k
$$-\omega*{turn}$$
k-1
$$) \]

con:

``` c
#define TURN_RATE_FILTER_TAU_S 0.75f
#define TURN_RATE_DEADBAND_DPS 0.10f
```

Cadena completa:

``` text
gyro XYZ
   |
LPF XYZ, tau = 0.15 s
   |
proyección sobre vertical estimada
   |
deadband
   |
LPF turn-rate, tau = 0.75 s
   |
indicador de giro
```

## 7. Bola de resbale/deslizamiento

En AMG ya no disponemos directamente de `linear_acceleration_ms2`
calculada por la fusión NDOF. La bola debe responder a la **aceleración
lateral no gravitatoria**, no a `accel_x_g` total.

El acelerómetro mide:

$$
a\_{x,meas}=a\_{x,linear}+g_x
$$

Con X transversal y la convención actual de roll:

$$
g\_{x,g}=\sin\phi
$$

por lo que:

$$
`\boxed{
a_{lat,g}=a_{x,g}-\sin\phi
}`
$$

Esta magnitud reconstruye por software el equivalente de la componente X
de `linear_acceleration_ms2`, expresada en G.

El ángulo equivalente de bola se obtiene mediante:

$$
`\boxed{
\beta=\arctan(a_{lat,g})
}`
$$

Ejemplos:

``` text
0.0 G ->  0.00 deg
0.1 G ->  5.71 deg
0.2 G -> 11.31 deg
```

Después se aplican signo gráfico, deadband, LPF y saturación:

``` c
#define SLIP_BALL_FILTER_TAU_S 1.5f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f
```

La llamada actual conceptualmente es:

``` c
bno055_compute_slip_ball_deg(
    data.accel_x_g,
    data.roll_deg,
    dt_s);
```

## 8. G-meter

La G actual se calcula como módulo del vector del acelerómetro:

$$
`\boxed{
G=\sqrt{G_x^2+G_y^2+G_z^2}
}`
$$

`Gmin` y `Gmax` son los mínimos y máximos absolutos acumulados desde el
último reset.

En la interfaz se muestran en una única línea:

``` text
Gmin (verde) - G actual (blanco) - Gmax (azul)
```

El antiguo instrumento analógico de G fue sustituido por el Ground
Speed.

## 9. BMP280: altitud y VSI

El BMP280 proporciona presión y temperatura. La altitud se obtiene a
partir de presión y QNH:

$$
h=f(P,QNH)
$$

El VSI se calcula en `BMP280.c`:

$$
v_z=\frac{dh}{dt}
$$

y se filtra antes de enviarlo al navegador.

## 10. GPS/GNSS

El GPS proporciona:

-   latitud y longitud;
-   altitud GPS;
-   Ground Speed;
-   Ground Track / Course Over Ground;
-   FIX;
-   satélites y HDOP;
-   UTC fecha/hora.

La validez de comunicaciones y la validez del FIX se tratan por
separado.

### Ground Speed

Existe un instrumento analógico dedicado **GS**, en knots, con leyenda
`KNOTS`. Funciona a partir de la velocidad GPS.

### Girodireccional / Ground Track

El girodireccional utiliza ahora el **Ground Track del GPS**, no el
heading de la IMU.

Una muestra nueva se acepta únicamente cuando el GPS tiene información
válida y la velocidad es suficientemente alta para que el
course-over-ground sea significativo. Si se pierde temporalmente el
heading GPS:

1.  se conserva el último Ground Track válido;
2.  si nunca se recibió uno válido, se muestra 0°.

Técnicamente el GPS proporciona track/course over ground, no el heading
aerodinámico del vehículo.

### Triángulo amarillo / course bug

El triángulo amarillo es un heading/course manual seleccionado por el
usuario mediante `+` y `-` y almacenado de forma persistente.

Su posición relativa es:

$$
`\boxed{
\Delta\psi=
\psi_{manual}-\psi_{GPS}
}`
$$

Cuando ambos coinciden, el triángulo queda a las 12.

Por compatibilidad, la variable interna puede conservar el nombre
histórico `heading_offset_deg`, aunque actualmente representa un valor
manual absoluto.

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

-   QNH;
-   pitch offset;
-   heading/course manual;
-   orientación BNO055 V/H.

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

``` text
pressure_hPa
temperature_C
altitude_m
verticalSpeed_ms
```

### BNO055 / inerciales

``` text
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

``` text
quaternion
gravity_ms2
linear_acceleration_ms2
```

En AMG esos datos fusionados no se utilizan; las magnitudes necesarias
se reconstruyen localmente.

### GPS

``` text
gpsConnected
gpsFixValid
gpsLatitude_deg
gpsLongitude_deg
gpsAltitude_m
gpsGroundSpeed_knots
gpsGroundTrack_deg
gpsUtcHour
gpsUtcMinute
gpsUtcSecond
gpsUtcDay
gpsUtcMonth
gpsUtcYear
```

## 16. Módulos de software

-   `BMP280.c`: presión, temperatura, altitud y VSI.
-   `BNO055.c`: configuración/remapeo, AMG/NDOF, adquisición inercial,
    filtro complementario, filtrado gyro XYZ, turn-rate, bola y G-meter.
-   `GPS.c`: UART/NMEA, posición, UTC, Ground Speed y Ground Track.
-   `wifi_ap.c`: Access Point ESP32-S3.
-   `webserver.c`: servidor HTTP.
-   `websocket.c`: telemetría JSON, comandos, persistencia y lógica de
    heading GPS.
-   `index.html`: representación gráfica del EFIS.
-   `main.c`: inicialización.
-   `config.h`: opciones de compilación, incluyendo fusión BNO055 y
    datalogger.

## 17. Resumen del procesamiento

``` text
ACCEL + GYRO X/Y
       |
       v
filtro complementario
       |
       +--------------------> pitch / roll

GYRO X/Y/Z
       |
       v
LPF tau=0.15 s
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

-   altitud barométrica: buen comportamiento;
-   VSI: buen comportamiento;
-   GS GPS: muy buen comportamiento;
-   Ground Track GPS en girodireccional: buen comportamiento;
-   indicador de giro: funcional, actualmente en ajuste de filtrado;
-   horizonte: se ha abandonado provisionalmente la dependencia de Euler
    NDOF y se está validando el estimador AMG propio;
-   bola: revisada para eliminar la componente lateral de gravedad antes
    de representar aceleración lateral;
-   se prevé probar un GPS de mayor frecuencia de actualización.

Las pruebas en automóvil son útiles para desarrollo, pero las dinámicas
de un coche no son equivalentes a las de una aeronave, especialmente en
régimen de giro, peralte, pitch/roll y aceleración lateral.

## 19. Pendiente

-   validar en carretera el filtro complementario AMG con `tau = 1.0 s`;
-   ajustar definitivamente `GYRO_FILTER_TAU_S` y
    `TURN_RATE_FILTER_TAU_S`;
-   validar físicamente el signo y dinámica de la bola;
-   validar la reconstrucción de aceleración lateral lineal;
-   probar el GPS de mayor frecuencia;
-   futura entrada pitot/estática para airspeed;
-   density altitude;
-   alarmas;
-   diagnóstico/calibración;
-   terrain awareness;
-   reactivar/ampliar el datalogger cuando vuelva a ser necesario.

## 20. Herramienta útil

ESPConnect:

https://thelastoutpostworkshop.github.io/ESPConnect/
