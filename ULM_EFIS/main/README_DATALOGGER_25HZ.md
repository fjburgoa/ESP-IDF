# Datalogger circular a 25 Hz

## PSRAM necesaria

El buffer se reserva expresamente con `MALLOC_CAP_SPIRAM`. En una
ESP32-S3-DevKitC-1-N16R8 debe habilitarse la PSRAM desde `idf.py menuconfig`:

1. `Component config -> ESP PSRAM`.
2. Activar soporte de RAM SPI externa (`CONFIG_SPIRAM`).
3. Seleccionar PSRAM **Octal** (`CONFIG_SPIRAM_MODE_OCT`).
4. Inicializar la PSRAM durante el arranque y añadirla al heap.

Los GPIO35, GPIO36 y GPIO37 quedan reservados para la comunicación interna con
la memoria octal y no pueden emplearse como E/S externas.

## Capacidad

La configuración predeterminada es:

```c
#define DATALOGGER_PERIOD_MS 40U
#define DATALOGGER_MAX_SAMPLES 30000U
```

A 25 Hz conserva 20 minutos. Cada muestra ocupa 88 bytes, por lo que el anillo
reserva aproximadamente 2,64 MB de PSRAM. Al llenarse, la siguiente muestra
sobrescribe la más antigua.

## Funcionamiento

- `INICIAR REGISTRO` vacía lógicamente el anillo y comienza una sesión nueva.
- `PARAR Y DESCARGAR CSV` detiene la adquisición y descarga automáticamente
  `efis_datalogger.csv`.
- El CSV se genera desde la muestra más antigua conservada hasta la más nueva,
  incluso si el anillo ha dado una o más vueltas.
- El buffer está en RAM: se pierde al reiniciar o quitar alimentación.

## Columnas CSV

```text
date_utc,time_utc,latitude_deg,longitude_deg,
accel_x_ms2,accel_y_ms2,accel_z_ms2,
linear_x_ms2,linear_y_ms2,linear_z_ms2,
gravity_x_ms2,gravity_y_ms2,gravity_z_ms2,
gyro_x_dps,gyro_y_dps,gyro_z_dps,
pitch_deg,roll_deg,slip_ball_deg,turn_rate_dps
```

Cada muestra ocupa una única línea. `accel_*` es la aceleración total escalada
en m/s², antes de eliminar la gravedad; `linear_*` es la aceleración lineal
proporcionada por SH-2. Las coordenadas se expresan en grados decimales con
ocho cifras decimales; si el GNSS no dispone de FIX, ambas columnas contienen
`nan`.
