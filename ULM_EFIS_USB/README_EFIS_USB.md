# EFIS USB para ESP32-S3

Versión reducida del EFIS centrada en el horizonte artificial y el coordinador
de giro. La comunicación Wi-Fi/JSON se ha sustituido por USB HID binario.

## Contenido

- `efis-usb/`: proyecto ESP-IDF 6.1 para el ESP32-S3.
- `efis-usb-host/`: página WebHID separada en HTML, CSS y JavaScript.

## Firmware

Desde la carpeta `efis-usb`:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Durante el arranque, el MPU6050 debe permanecer inmóvil y horizontal mientras
se calculan los offsets. El bus I2C utiliza SDA=GPIO8, SCL=GPIO9 y 400 kHz.

El firmware envía un informe HID cada 20 ms, alternando sensores:

- Report ID 1: MPU6050, 25 informes/s.
- Report ID 2: BNO055, 25 informes/s.
- Report ID 3: salida del host para guardar el offset de pitch en NVS.

Cada informe de entrada contiene 44 bytes little-endian:

| Offset | Tipo | Campo |
|---:|---|---|
| 0 | `uint32_t` | Secuencia |
| 4 | `uint32_t` | Tiempo desde arranque, ms |
| 8 | `float` | Pitch, grados |
| 12 | `float` | Roll, grados |
| 16, 20, 24 | `float[3]` | Aceleraciones X/Y/Z, m/s² |
| 28, 32, 36 | `float[3]` | Velocidades angulares X/Y/Z, °/s |
| 40 | `float` | Offset de pitch, grados |

Pitch y roll se calculan únicamente a partir del acelerómetro. No se emplea
filtro complementario ni la fusión interna del BNO055.

## Página del host

Desde la carpeta `efis-usb-host`:

```bash
python -m http.server 8000
```

Abrir `http://localhost:8000` con Chrome o Edge y pulsar **Conectar USB**.
Los dos instrumentos usan exclusivamente los datos del MPU6050. Las tarjetas
inferiores permiten comparar los valores recibidos de ambos sensores.

El avión del coordinador usa `gyro_z`. La bola se obtiene directamente de la
aceleración lateral `ax` mediante `asin(ax/g)`, limitada a ±25°; este cálculo
deliberadamente simple permite estudiar el problema antes de añadir filtros o
compensaciones.
