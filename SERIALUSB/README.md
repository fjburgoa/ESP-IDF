 ESP32-S3 |


# TinyUSB Serial Device Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example shows how to set up ESP chip to work as a USB Serial Device.

As a USB stack, a TinyUSB component is used.


After the flashing you should see this output (for single CDC channel):

```
I (285) example: USB initialization
I (285) tusb_desc:
┌─────────────────────────────────┐
│  USB Device Descriptor Summary  │
├───────────────────┬─────────────┤
│bDeviceClass       │ 239         │
├───────────────────┼─────────────┤
│bDeviceSubClass    │ 2           │
├───────────────────┼─────────────┤
│bDeviceProtocol    │ 1           │
├───────────────────┼─────────────┤
│bMaxPacketSize0    │ 64          │
├───────────────────┼─────────────┤
│idVendor           │ 0x303a      │
├───────────────────┼─────────────┤
│idProduct          │ 0x4001      │
├───────────────────┼─────────────┤
│bcdDevice          │ 0x100       │
├───────────────────┼─────────────┤
│iManufacturer      │ 0x1         │
├───────────────────┼─────────────┤
│iProduct           │ 0x2         │
├───────────────────┼─────────────┤
│iSerialNumber      │ 0x3         │
├───────────────────┼─────────────┤
│bNumConfigurations │ 0x1         │
└───────────────────┴─────────────┘
I (455) TinyUSB: TinyUSB Driver installed
I (465) example: USB initialization DONE
```

Connect to the serial port (e.g. on Linux, it should be `/dev/ttyACM0`) by any terminal application (e.g. `picocom /dev/ttyACM0`).
Now you can send data strings to the device, the device will echo back the same data string.

The monitor tool will also print the communication process:

```
I (12438) example: Data from channel 0:
I (12438) example: 0x3ffbfea0   45 73 70 72 65 73 73 69  66 0d                    |Espressif.|
```
