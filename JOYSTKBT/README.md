Estos dos programas están basados en el ejemplo ..\bluetooth\esp_hid_device\

Se debe activar además la propiedad nimble dentro de la configuración

Si se selecciona en el CMakeList.txt, esp_hid_mouse_main.c, se compila un programa que convierte el ESP32s3 en un mouse a través de nimble. El ESP32 se controla a través del puerto serie. 

vSi se selecciona en el CMakeList.txt, esp_hid_joystick_main.c, se compila un programa que convierte el ESP32s3 en un joystick a través de nimble. El ESP32 genera una secuencia de prueba

Es necesario siempre después de un reset emparejar el dispositivo. La contraseña es 123456

Probarlo con Win+R -> "joy.cpl"

--------------------

Joystick:

Por el terminal:

rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x158c
load:0x403c8700,len:0xd24
load:0x403cb700,len:0x2f34
entry 0x403c8924
I (24) boot: ESP-IDF v5.5.1-dirty 2nd stage bootloader
I (25) boot: compile time Sep 20 2025 16:52:50
I (25) boot: Multicore bootloader
I (25) boot: chip revision: v0.2
I (28) boot: efuse block revision: v1.3
I (32) boot.esp32s3: Boot SPI Speed : 80MHz
I (36) boot.esp32s3: SPI Mode       : DIO
I (39) boot.esp32s3: SPI Flash Size : 2MB
I (43) boot: Enabling RNG early entropy source...
I (48) boot: Partition Table:
I (50) boot: ## Label            Usage          Type ST Offset   Length
I (57) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (63) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (70) boot:  2 factory          factory app      00 00 00010000 00177000
I (76) boot: End of partition table
I (79) esp_image: segment 0: paddr=00010020 vaddr=3c060020 size=152f0h ( 86768) map
I (102) esp_image: segment 1: paddr=00025318 vaddr=3fc99e00 size=04108h ( 16648) load
I (106) esp_image: segment 2: paddr=00029428 vaddr=40374000 size=06bf0h ( 27632) load
I (112) esp_image: segment 3: paddr=00030020 vaddr=42000020 size=59b20h (367392) map
I (178) esp_image: segment 4: paddr=00089b48 vaddr=4037abf0 size=0f15ch ( 61788) load
I (192) esp_image: segment 5: paddr=00098cac vaddr=50000000 size=00020h (    32) load
I (201) boot: Loaded app from partition at offset 0x10000
I (201) boot: Disabling RNG early entropy source...
I (211) cpu_start: Multicore app
I (221) cpu_start: Pro cpu start user code
I (221) cpu_start: cpu freq: 160000000 Hz
I (221) app_init: Application information:
I (221) app_init: Project name:     esp_hid_device
I (225) app_init: App version:      1
I (229) app_init: Compile time:     Sep 20 2025 23:54:15
I (234) app_init: ELF file SHA256:  83629a08c...
I (238) app_init: ESP-IDF:          v5.5.1-dirty
I (242) efuse_init: Min chip rev:     v0.0
I (246) efuse_init: Max chip rev:     v0.99 
I (250) efuse_init: Chip rev:         v0.2
I (254) heap_init: Initializing. RAM available for dynamic allocation:
I (260) heap_init: At 3FCA1A70 len 00047CA0 (287 KiB): RAM
I (265) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (271) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (276) heap_init: At 600FE000 len 00001FE8 (7 KiB): RTCRAM
I (282) spi_flash: detected chip: generic
I (285) spi_flash: flash io: dio
W (288) spi_flash: Detected size(8192k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (300) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (306) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (313) coexist: coex firmware version: b0bcc39
I (331) coexist: coexist rom version e7ae62f
I (332) main_task: Started on CPU0
I (342) main_task: Calling app_main()
I (352) HID_DEV_JOYSTICK: setting hid gap, mode:1
I (352) BLE_INIT: BT controller compile version [2edb0b0]
I (352) BLE_INIT: Using main XTAL as clock source
I (352) BLE_INIT: Feature Config, ADV:1, BLE_50:1, DTM:1, SCAN:1, CCA:0, SMP:1, CONNECT:1
I (362) BLE_INIT: Bluetooth MAC: dc:da:0c:64:75:c6
I (372) phy_init: phy_version 701,f4f1da3a,Mar  3 2025,15:50:10
I (412) HID_DEV_JOYSTICK: setting ble device
I (412) HID_DEV_JOYSTICK: BLE Host Task Started
I (412) NimBLE: GAP procedure initiated: stop advertising.

I (422) HID_DEV_JOYSTICK: START
I (422) main_task: Returned from app_main()
I (422) NimBLE: GAP procedure initiated: advertise; 
I (422) NimBLE: disc_mode=2
I (422) NimBLE:  adv_channel_map=0 own_addr_type=0 adv_filter_policy=0 adv_itvl_min=48 adv_itvl_max=80
I (432) NimBLE: 

I (2202) ESP_HID_GAP: mtu update event; conn_handle=1 cid=4 mtu=256
I (2322) HID_DEV_JOYSTICK: CONNECT
I (2322) ESP_HID_GAP: connection established; status=0
I (2682) ESP_HID_GAP: PASSKEY_ACTION_EVENT started
I (2682) ESP_HID_GAP: Enter passkey 123456on the peer side
I (2682) ESP_HID_GAP: ble_sm_inject_io result: 0
I (3222) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=8 reason=1 prevn=0 curn=0 previ=0 curi=1

I (4002) ESP_HID_GAP: connection updated; status=0
I (4262) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=18 reason=1 prevn=0 curn=1 previ=0 curi=0

I (5982) ESP_HID_GAP: connection updated; status=0
I (11682) NimBLE: GAP procedure initiated: stop advertising.

I (11692) NimBLE: encryption change event; status=0 
I (11692) NimBLE: GATT procedure initiated: notify; 
I (11692) NimBLE: att_handle=39

I (11792) NimBLE: GATT procedure initiated: notify; 
I (11792) NimBLE: att_handle=39
