Basado en "C:\Users\fjbur\esp\v5.5.2\esp-idf\examples\provisioning"

Hay que descargars en el movil ESP BLE Prov


For this example, BLE is chosen as the default mode of transport, over which the provisioning related communication is to take place. NimBLE has been configured as the default host.

In the provisioning process the device is configured as a Wi-Fi station with specified credentials. Once configured, the device will retain the Wi-Fi configuration, until a flash erase is performed.

Right after provisioning is complete, BLE is turned off and disabled to free the memory used by the BLE stack. Though, that is specific to this example, and the user can choose to keep BLE stack intact in their own application.

`wifi_prov_mgr` uses the following components :
* `wifi_provisioning` : provides manager, data structures and protocomm endpoint handlers for Wi-Fi configuration
* `protocomm` : for protocol based communication and secure session establishment
* `protobuf` : Google's protocol buffer library for serialization of protocomm data structures
* `bt` : ESP32 BLE stack for transport of protobuf packets

## Example Output

```
I (445) app: Starting provisioning
I (1035) app: Provisioning started
I (1045) wifi_prov_mgr: Provisioning started with service name : PROV_261FCC
```

Make sure to note down the BLE device name (starting with `PROV_`) displayed in the serial monitor log (eg. PROV_261FCC). This will depend on the MAC ID and will be unique for every device.

conectate con la aplicación de movil a PROV_261FCC e indícale la SSID y PASSWORD de tu red
```

se puede usar
```
python esp_prov.py --transport ble --service_name PROV_261FCC --sec_ver 2 --sec2_username wifiprov --sec2_pwd abcd1234 --ssid myssid --passphrase mypassword
```

Above command will perform the provisioning steps, and the monitor log should display something like this :

```
I (39725) app: Received Wi-Fi credentials
    SSID     : myssid
    Password : mypassword
.
.
.
I (45335) esp_netif_handlers: sta ip: 192.168.43.243, mask: 255.255.255.0, gw: 192.168.43.1
I (45345) app: Provisioning successful
I (45345) app: Connected with IP Address:192.168.43.243
I (46355) app: Hello World!
I (47355) app: Hello World!
I (48355) app: Hello World!
I (49355) app: Hello World!
.
.
.
I (52315) wifi_prov_mgr: Provisioning stopped
.
.
.
I (52355) app: Hello World!
I (53355) app: Hello World!
I (54355) app: Hello World!
I (55355) app: Hello World!
```

**Note:** For generating the credentials for security version 2 (`SRP6a` salt and verifier) for the device-side, the following example command can be used. The output can then directly be used in this example.

The config option `CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE` should be enabled for the example and in `main/app_main.c`, the macro `EXAMPLE_PROV_SEC2_USERNAME` should be set to the same username used in the salt-verifier generation.

```log
$ python esp_prov.py --transport softap --sec_ver 2 --sec2_gen_cred --sec2_username wifiprov --sec2_pwd abcd1234
==== Salt-verifier for security scheme 2 (SRP6a) ====
static const char sec2_salt[] = {
    0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4
};

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
    0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
    .
    .
    .
    0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
};

```
 