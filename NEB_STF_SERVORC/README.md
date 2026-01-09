# MCPWM RC Servo Control Example

This example illustrates how to drive a typical [RC Servo] https://en.wikipedia.org/wiki/Servo_%28radio_control%29 by sending a PCM signal using the MCPWM driver. 

The PCM pulse has a frequency of 50Hz (period of 20ms), and the active-high time (which controls the rotation) ranges from 0.5s (500us) to 2.5ms (2500us) with 1.5ms (1500us) always being center of range.

## How to Use Example

### Hardware Required

USAR ALIMENTACIÓN EXTERNA

Connection :


      ESP Board              Servo Motor      5V
+-------------------+     +---------------+    ^
|  SERVO_PULSE_GPIO +-----+PWM        VCC +----+
|                   |     |               |
|               GND +-----+GND            |
+-------------------+     +---------------+


