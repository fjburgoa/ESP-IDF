Ejemplo 26 del Libro - Comunicación a través de UDP


CREADO POR FRANCISCO JAVIER BURGOA el 15/12/2025

1 En menuconfig, actualiza SSID y PASSWORD
2 Copia el directorio.
3 Compila
4 Selecciona ESP32-S3
5 Flashea
6 Instalar de Microsoft Store > UDP - Sender/Receiver de ReddySoftware u otro programa equivalente
7 Toma nota de la dirección IP de tu dispositivo.
8 Indica la dirección remota en el cliente UDP y el puerto (3333)



---- Opened the serial port COM4 ----
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0xb (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x158c
load:0x403c8700,len:0xd24
load:0x403cb700,len:0x2f34
entry 0x403c8924
I (24) boot: ESP-IDF v5.5.1-dirty 2nd stage bootloader
I (25) boot: compile time Dec 16 2025 18:41:44
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
I (70) boot:  2 factory          factory app      00 00 00010000 00100000
I (76) boot: End of partition table
I (79) esp_image: segment 0: paddr=00010020 vaddr=3c090020 size=19eb8h (106168) map
I (106) esp_image: segment 1: paddr=00029ee0 vaddr=3fc9a700 size=04a24h ( 18980) load
I (110) esp_image: segment 2: paddr=0002e90c vaddr=40374000 size=0170ch (  5900) load
I (111) esp_image: segment 3: paddr=00030020 vaddr=42000020 size=87220h (553504) map
I (214) esp_image: segment 4: paddr=000b7248 vaddr=4037570c size=14f54h ( 85844) load
I (233) esp_image: segment 5: paddr=000cc1a4 vaddr=50000000 size=00020h (    32) load
I (243) boot: Loaded app from partition at offset 0x10000
I (243) boot: Disabling RNG early entropy source...
I (253) cpu_start: Multicore app
I (262) cpu_start: Pro cpu start user code
I (262) cpu_start: cpu freq: 160000000 Hz
I (262) app_init: Application information:
I (262) app_init: Project name:     Ejemplo26
I (266) app_init: App version:      86cd50bd-dirty
I (271) app_init: Compile time:     Dec 16 2025 18:46:14
I (276) app_init: ELF file SHA256:  bc0cdfe0e...
I (280) app_init: ESP-IDF:          v5.5.1-dirty
I (284) efuse_init: Min chip rev:     v0.0
I (288) efuse_init: Max chip rev:     v0.99 
I (292) efuse_init: Chip rev:         v0.2
I (296) heap_init: Initializing. RAM available for dynamic allocation:
I (302) heap_init: At 3FCA2E08 len 00046908 (282 KiB): RAM
I (307) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (313) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (318) heap_init: At 600FE000 len 00001FE8 (7 KiB): RTCRAM
I (324) spi_flash: detected chip: boya
I (326) spi_flash: flash io: dio
W (329) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (342) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (348) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (355) main_task: Started on CPU0
I (375) main_task: Calling app_main()
I (385) example_connect: Start example_connect.
I (385) pp: pp rom version: e7ae62f
I (385) net80211: net80211 rom version: e7ae62f
I (395) wifi:wifi driver task: 3fcee9b0, prio:23, stack:6656, core=0
I (395) wifi:wifi firmware version: 14da9b7
I (395) wifi:wifi certification version: v7.0
I (405) wifi:config NVS flash: enabled
I (405) wifi:config nano formatting: disabled
I (405) wifi:Init data frame dynamic rx buffer num: 32
I (415) wifi:Init static rx mgmt buffer num: 5
I (415) wifi:Init management short buffer num: 32
I (425) wifi:Init dynamic tx buffer num: 32
I (425) wifi:Init static tx FG buffer num: 2
I (435) wifi:Init static rx buffer size: 1600
I (435) wifi:Init static rx buffer num: 10
I (435) wifi:Init dynamic rx buffer num: 32
I (445) wifi_init: rx ba win: 6
I (445) wifi_init: accept mbox: 6
I (445) wifi_init: tcpip mbox: 32
I (455) wifi_init: udp mbox: 6
I (455) wifi_init: tcp mbox: 6
I (455) wifi_init: tcp tx win: 5760
I (455) wifi_init: tcp rx win: 5760
I (465) wifi_init: tcp mss: 1440
I (465) wifi_init: WiFi IRAM OP enabled
I (465) wifi_init: WiFi RX IRAM OP enabled
I (475) phy_init: phy_version 701,f4f1da3a,Mar  3 2025,15:50:10
I (515) phy_init: Saving new calibration data due to checksum failure or outdated calibration data, mode(0)
I (535) wifi:mode : sta 
I (535) wifi:enable tsf
I (535) example_connect: Connecting to XXXXXXXXXXXXXXXXXXXX...
W (535) wifi:Password length matches WPA2 standards, authmode threshold changes from OPEN to WPA2
I (545) example_connect: Waiting for IP(s)
I (3035) wifi:new:<1,1>, old:<1,0>, ap:<255,255>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (3035) wifi:state: init -> auth (0xb0)
I (3045) wifi:state: auth -> assoc (0x0)
I (3045) wifi:state: assoc -> run (0x10)
I (3085) wifi:connected with XXXXXXXXXXXXXXXXXXXXXXX, aid = 3, channel 1, 40U, bssid = xxxxxxxxxxxxxxxxxxxxxxx
I (3085) wifi:security: WPA2-PSK, phy: bgn, rssi: -65
I (3085) wifi:pm start, type: 1

I (3095) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (3095) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (3135) wifi:AP's beacon interval = 102400 us, DTIM period = 1
I (4155) wifi:<ba-add>idx:0 (ifx:0, f4:23:9c:0d:d9:81), tid:0, ssn:0, winSize:64
I (4385) example_connect: Got IPv6 event: Interface "example_netif_sta" address: fe80:0000:0000:0000:66e8:33ff:fe57:b558, type: ESP_IP6_ADDR_IS_LINK_LOCAL
I (6185) esp_netif_handlers: example_netif_sta ip: 192.168.0.22, mask: 255.255.255.0, gw: 192.168.0.1
I (6185) example_connect: Got IPv4 event: Interface "example_netif_sta" address: 192.168.0.22
I (6185) example_common: Connected to example_netif_sta
I (6195) example_common: - IPv4 address: 192.168.0.22,
I (6195) example_common: - IPv6 address: fe80:0000:0000:0000:66e8:33ff:fe57:b558, type: ESP_IP6_ADDR_IS_LINK_LOCAL
Socket creado
sendto falló: errno=12


RX desde 192.168.0.15: envió desde el cliente UDP  <- enviado desde el cliente

