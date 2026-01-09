| ESP32-S3 |

#### ANTES DE EMPEZAR ####

"Arreglo" o Ajuste del Vref (Paso a Paso) El "arreglo" consiste en el procedimiento de calibración. Sigue estos pasos para ajustar el Vref correctamente: 
1- Desconecta el motor paso a paso de la placa del driver.

2- Asegura las conexiones de alimentación: Conecta la alimentación lógica (si es necesaria, aunque el DRV8825 tiene un regulador interno de 3.3V) y la alimentación del motor (VMOT, 8.2V a 45V) a la placa del driver. Importante: Nunca conectes ni desconectes el motor mientras el driver está encendido, ya que esto puede destruirlo.

3- Habilita el driver: Asegúrate de que los pines RST (Reset) y SLP (Sleep) estén conectados a un nivel lógico alto (por ejemplo, a 5V o 3.3V si se usa una placa como la RAMPS, o un puente si tu placa específica lo requiere para funcionar). El pin ENBL (Enable) suele estar habilitado por defecto si se deja desconectado.

4- Calcula el voltaje de referencia (Vref) objetivo: La fórmula para la mayoría de las placas DRV8825 (con resistencias sensoras de 0.1 ohmios) es: VRef=Imax/2,  Donde Imax es la corriente nominal máxima por fase de tu motor paso a paso (consulta la hoja de datos de tu motor).

5- Mide el Vref: Con un multímetro configurado para medir voltaje DC, coloca la punta de prueba negativa (negra) en un pin GND de la placa y la punta de prueba positiva (roja) en el centro metálico del potenciómetro de ajuste (o en el pin AVREF/BVREF si es accesible).

6- Ajusta el potenciómetro: Con un destornillador de plástico (para evitar cortocircuitos accidentales), gira suavemente el potenciómetro mientras mides el voltaje para alcanzar el valor de Vref calculado. Girar en sentido horario generalmente aumenta el voltaje, y en sentido antihorario lo disminuye (puede variar según el fabricante de la placa, así que procede con cuidado).

7- Desconecta la alimentación una vez ajustado el Vref, conecta el motor y ya puedes usar el driver. Este ajuste es fundamental para un funcionamiento seguro y eficiente del sistema. 


#### #### #### #### #### #### #### #### #### #### #### #### #### #### 












# RMT Based Stepper Motor Smooth Controller

(See the README.md file in the upper level 'examples' directory for more information about examples.)

One RMT TX channel can use different encoders in sequence, which is useful to generate waveforms that have obvious multiple stages.

This example shows how to drive a stepper motor with a **STEP/DIR** interfaced controller (e.g. [DRV8825](https://www.ti.com/lit/ds/symlink/drv8825.pdf)) in a [smooth](https://en.wikipedia.org/wiki/Smoothstep) way. To smoothly drive a stepper motor, there're three phases: **Acceleration**, **Uniform** and **Deceleration**. Accordingly, this example implements two encoders so that RMT channel can generate the waveforms with different characteristics:

* `curve_encoder` is to encode the **Acceleration** and **Deceleration** phase
* `uniform_encoder` is to encode the ***Uniform** phase

## How to Use Example

### Hardware Required

* A development board with any supported Espressif SOC chip (see `Supported Targets` table above)
* A USB cable for Power supply and programming
* A two-phase four-wire stepper motor
* A DRV8825 stepper motor controller

Connection :

```
+---------------------------+             +--------------------+      +--------------+
|          ESP Board        |             |       DRV8825      |      |    4-wire    |
|                       GND +-------------+ GND                |      |     Step     |
|                           |             |                    |      |     Motor    |
|                       3V3 +-------------+ VDD             A+ +------+ A+           |
|                           |             |                    |      |              |
|       STEP_MOTOR_GPIO_DIR +------------>+ DIR             A- +------+ A-           |
|                           |             |                    |      |              |
|      STEP_MOTOR_GPIO_STEP +------------>+ STEP            B- +------+ B-           |
|                           |             |                    |      |              |
|                           |      3V3----+ nSLEEP          B+ +------+ B+           |
|                           |             |                    |      +--------------+
|                           |      3V3----+ nRST            VM +-------------------+
|                           |             |                    |                   |
|                           |  3V3|GND----+ M2             GND +----------+        |
|                           |             |                    |          |        |
|                           |  3V3|GND----+ M1                 |          |        |
|                           |             |                    |          |        |
|                           |  3V3|GND----+ M0                 |      +---+--------+-----+
|                           |             |                    |      |  GND     +12V    |
|        STEP_MOTOR_GPIO_EN +------------>+ nEN                |      |   POWER SUPPLY   |
+---------------------------+             +--------------------+      +------------------+
```

The GPIO number used in this example can be changed according to your board, by the macro `STEP_MOTOR_GPIO_EN`, `STEP_MOTOR_GPIO_DIR` and `STEP_MOTOR_GPIO_STEP` defined in the [source file](main/stepper_motor_example_main.c).

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.


## Example Output

```
I (0) cpu_start: Starting scheduler on APP CPU.
I (325) example: Initialize EN + DIR GPIO
I (325) gpio: GPIO[16]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (335) gpio: GPIO[17]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (345) example: Create RMT TX channel
I (365) example: Set spin direction
I (365) example: Enable step motor
I (375) example: Create motor encoders
I (405) example: Start RMT channel
I (405) example: Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel
```

## Troubleshooting

For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
