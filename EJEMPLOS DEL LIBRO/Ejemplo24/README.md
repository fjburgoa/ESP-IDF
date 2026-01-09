Ejemplo 24 del Libro - Cliente HTTPS


CREADO POR FRANCISCO JAVIER BURGOA el 15/12/2025, Version API: 5.5.1
Actualizado a API: 5.5.2 el 27/12/2025


1 Copia el directorio 
2 Abrir el directorio en VSCode > Open Folder
3 Compila
4 Selecciona ESP32-S3
5 Flashea


---------------------------------------

ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0xb (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x158c
load:0x403c8700,len:0xcdc
load:0x403cb700,len:0x2f10
entry 0x403c8918
I (24) boot: ESP-IDF v5.5.1-dirty 2nd stage bootloader
I (25) boot: compile time Dec 16 2025 13:11:20
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
I (79) esp_image: segment 0: paddr=00010020 vaddr=3c0a0020 size=303b4h (197556) map
I (122) esp_image: segment 1: paddr=000403dc vaddr=3fc9a500 size=04a18h ( 18968) load
I (126) esp_image: segment 2: paddr=00044dfc vaddr=40374000 size=0b21ch ( 45596) load
I (136) esp_image: segment 3: paddr=00050020 vaddr=42000020 size=9eef8h (651000) map
I (251) esp_image: segment 4: paddr=000eef20 vaddr=4037f21c size=0b2c8h ( 45768) load
I (266) esp_image: segment 5: paddr=000fa1f0 vaddr=50000000 size=00020h (    32) load
I (276) boot: Loaded app from partition at offset 0x10000
I (276) boot: Disabling RNG early entropy source...
I (286) cpu_start: Multicore app
I (295) cpu_start: Pro cpu start user code
I (295) cpu_start: cpu freq: 160000000 Hz
I (295) app_init: Application information:
I (295) app_init: Project name:     cliente_https
I (299) app_init: App version:      86cd50bd-dirty
I (304) app_init: Compile time:     Dec 16 2025 13:11:03
I (309) app_init: ELF file SHA256:  42db5c82f...
I (313) app_init: ESP-IDF:          v5.5.1-dirty
I (318) efuse_init: Min chip rev:     v0.0
I (321) efuse_init: Max chip rev:     v0.99 
I (325) efuse_init: Chip rev:         v0.2
I (329) heap_init: Initializing. RAM available for dynamic allocation:
I (336) heap_init: At 3FCA41E8 len 00045528 (277 KiB): RAM
I (341) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (346) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (351) heap_init: At 600FE000 len 00001FE8 (7 KiB): RTCRAM
I (357) spi_flash: detected chip: boya
I (360) spi_flash: flash io: dio
W (363) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (375) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (381) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (388) main_task: Started on CPU0
I (408) main_task: Calling app_main()
I (418) example_connect: Start example_connect.
I (418) pp: pp rom version: e7ae62f
I (418) net80211: net80211 rom version: e7ae62f
I (428) wifi:wifi driver task: 3fcee8d0, prio:23, stack:6656, core=0
I (428) wifi:wifi firmware version: 14da9b7
I (428) wifi:wifi certification version: v7.0
I (438) wifi:config NVS flash: enabled
I (438) wifi:config nano formatting: disabled
I (448) wifi:Init data frame dynamic rx buffer num: 32
I (448) wifi:Init static rx mgmt buffer num: 5
I (448) wifi:Init management short buffer num: 32
I (458) wifi:Init dynamic tx buffer num: 32
I (458) wifi:Init static tx FG buffer num: 2
I (468) wifi:Init static rx buffer size: 1600
I (468) wifi:Init static rx buffer num: 10
I (468) wifi:Init dynamic rx buffer num: 32
I (478) wifi_init: rx ba win: 6
I (478) wifi_init: accept mbox: 6
I (478) wifi_init: tcpip mbox: 32
I (488) wifi_init: udp mbox: 6
I (488) wifi_init: tcp mbox: 6
I (488) wifi_init: tcp tx win: 5760
I (498) wifi_init: tcp rx win: 5760
I (498) wifi_init: tcp mss: 1440
I (498) wifi_init: WiFi IRAM OP enabled
I (508) wifi_init: WiFi RX IRAM OP enabled
I (508) phy_init: phy_version 701,f4f1da3a,Mar  3 2025,15:50:10
I (548) wifi:mode : sta (64:e8:33:57:b5:58)
I (548) wifi:enable tsf
I (548) example_connect: Connecting to XXXXXXXXXXXXX...
W (548) wifi:Password length matches WPA2 standards, authmode threshold changes from OPEN to WPA2
I (558) example_connect: Waiting for IP(s)
I (3058) wifi:new:<1,1>, old:<1,0>, ap:<255,255>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (3058) wifi:state: init -> auth (0xb0)
I (3058) wifi:state: auth -> assoc (0x0)
I (3058) wifi:state: assoc -> run (0x10)
I (3098) wifi:connected with XXXXXXXXXXXXX, aid = 3, channel 1, 40U, bssid = f4:23:9c:0d:d9:81
I (3098) wifi:security: WPA2-PSK, phy: bgn, rssi: -62
I (3098) wifi:pm start, type: 1

I (3098) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (3108) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (3168) wifi:AP's beacon interval = 102400 us, DTIM period = 1
I (3588) wifi:<ba-add>idx:0 (ifx:0, f4:23:9c:0d:d9:81), tid:0, ssn:0, winSize:64
I (4418) example_connect: Got IPv6 event: Interface "example_netif_sta" address: fe80:0000:0000:0000:66e8:33ff:fe57:b558, 

I (6228) esp_netif_handlers: example_netif_sta ip: 192.168.0.22, mask: 255.255.255.0, gw: 192.168.0.1
I (6228) example_connect: Got IPv4 event: Interface "example_netif_sta" address: 192.168.0.22
I (6228) example_common: Connected to example_netif_sta
I (6238) example_common: - IPv4 address: 192.168.0.22,
I (6238) example_common: - IPv6 address: fe80:0000:0000:0000:66e8:33ff:fe57:b558, type: ESP_IP6_ADDR_IS_LINK_LOCAL

>>>> Conectado al Router, comienza el ejemplo
>>>> Pulsa Boot para saber cosas de gatitos
>>>> Pulsador activado
Petición HTTPS a la URL:  => https://catfact.ninja/fact

I (13548) esp-x509-crt-bundle: Certificate validated
HTTP_EVENT_ON_CONNECTED
HTTP_EVENT_HEADER_SENT
HTTP_EVENT_ON_HEADER, key=Date, value=Tue, 16 Dec 2025 12:12:29 GMT
HTTP_EVENT_ON_HEADER, key=Content-Type, value=application/json
HTTP_EVENT_ON_HEADER, key=Content-Length, value=59
HTTP_EVENT_ON_HEADER, key=Connection, value=keep-alive
HTTP_EVENT_ON_HEADER, key=Server, value=cloudflare
HTTP_EVENT_ON_HEADER, key=Cache-Control, value=no-cache, private
HTTP_EVENT_ON_HEADER, key=x-ratelimit-limit, value=100
HTTP_EVENT_ON_HEADER, key=x-ratelimit-remaining, value=99
HTTP_EVENT_ON_HEADER, key=access-control-allow-origin, value=*
HTTP_EVENT_ON_HEADER, key=Set-Cookie,  expires=Tue, 16-Dec-2025 14:12:29 GMT; Max-Age=7200; path=/; secure; httponly; samesite=lax
HTTP_EVENT_ON_HEADER, key=x-frame-options, value=SAMEORIGIN
HTTP_EVENT_ON_HEADER, key=x-xss-protection, value=1; mode=block
HTTP_EVENT_ON_HEADER, key=x-content-type-options, value=nosniff
HTTP_EVENT_ON_HEADER, key=Nel, value={"report_to":"cf-nel","success_fraction":0.0,"max_age":604800}
HTTP_EVENT_ON_HEADER, key=cf-cache-status, value=DYNAMIC

HTTP_EVENT_ON_HEADER, key=CF-RAY, value=9aee16db7e1b0355-MAD
HTTP_EVENT_ON_HEADER, key=alt-svc, value=h3=":443"; ma=86400
HTTP_EVENT_ON_DATA, len=59
HTTP_EVENT_ON_FINISH
HTTPS Status = 200
Content-Length reportado = 59
Bytes recibidos realmente = 0
Codificación: normal

Response:
{"fact":"A group of cats is called a clowder.","length":36}
HTTP_EVENT_DISCONNECTED



>>>>> Pulsador activado
Petición HTTPS a la URL:  => https://catfact.ninja/fact
I (40508) esp-x509-crt-bundle: Certificate validated
HTTP_EVENT_ON_CONNECTED
HTTP_EVENT_HEADER_SENT
HTTP_EVENT_ON_HEADER, key=Date, value=Tue, 16 Dec 2025 12:12:56 GMT
HTTP_EVENT_ON_HEADER, key=Content-Type, value=application/json
HTTP_EVENT_ON_HEADER, key=Content-Length, value=141
HTTP_EVENT_ON_HEADER, key=Connection, value=keep-alive
HTTP_EVENT_ON_HEADER, key=Server, value=cloudflare
HTTP_EVENT_ON_HEADER, key=Cache-Control, value=no-cache, private
HTTP_EVENT_ON_HEADER, key=x-ratelimit-limit, value=100
HTTP_EVENT_ON_HEADER, key=x-ratelimit-remaining, value=98
HTTP_EVENT_ON_HEADER, key=access-control-allow-origin, value=*
 
HTTP_EVENT_ON_HEADER, key=x-frame-options, value=SAMEORIGIN
HTTP_EVENT_ON_HEADER, key=x-xss-protection, value=1; mode=block
HTTP_EVENT_ON_HEADER, key=x-content-type-options, value=nosniff
HTTP_EVENT_ON_HEADER, key=Nel, value={"report_to":"cf-nel","success_fraction":0.0,"max_age":604800}

HTTP_EVENT_ON_HEADER, key=cf-cache-status, value=DYNAMIC

HTTP_EVENT_ON_HEADER, key=CF-RAY, value=9aee1783ea6ccfa6-MAD
HTTP_EVENT_ON_HEADER, key=alt-svc, value=h3=":443"; ma=86400
HTTP_EVENT_ON_DATA, len=141
HTTP_EVENT_ON_FINISH
HTTPS Status = 200
Content-Length reportado = 141
Bytes recibidos realmente = 0
Codificación: normal

Response:
{"fact":"In Ancient Egypt, when a person's house cat passed away, the owner would shave their eyebrows to reflect their grief.","length":117}

HTTP_EVENT_DISCONNECTED
