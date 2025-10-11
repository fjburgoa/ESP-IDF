// -----------------------------------------------------------------------------
// HID BLE Joystick Example (ESP32-S3, ESP-IDF + NimBLE + esp_hidd)
// se envían en secuencia 
// Envía los comandos por bluetooth-nimble (BLE)
// se descubre el servicio y hay que meter de pass: 123456
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"

static const char *TAG = "HID_DEV_JOYSTICK";

typedef struct {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

// -----------------------------------------------------------------------------
// HID Report Map for Joystick (Gamepad)
// -----------------------------------------------------------------------------
const unsigned char joystickReportMap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)

    // Report ID
    0x85, 0x01,       //   Report ID (1)

    // Buttons (8 bits)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x08,       //   Usage Maximum (Button 8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x95, 0x08,       //   Report Count (8 buttons)
    0x75, 0x01,       //   Report Size (1)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)

    // Padding (to make 1 byte)
    0x95, 0x01,
    0x75, 0x08,
    0x81, 0x03,       //   Input (Constant)

    // X, Y axes (each 8 bits signed)
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x15, 0x81,       //   Logical Minimum (-127)
    0x25, 0x7F,       //   Logical Maximum (127)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)

    0xC0              // End Collection
};

// -----------------------------------------------------------------------------
// Send Joystick Report
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t buttons;  // 8 buttons (bitmask)
    uint8_t padding;
    int8_t x;         // X axis
    int8_t y;         // Y axis
} joystick_report_t;

//------------------------------------------------------------------------------
void send_gamepad(uint8_t buttons, int8_t x, int8_t y) {
    joystick_report_t rpt;
    rpt.buttons = buttons;
    rpt.padding = 0;
    rpt.x = x;
    rpt.y = y;
    
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 1,  (uint8_t*)&rpt, sizeof(rpt));
}

// -----------------------------------------------------------------------------
// Demo Task
// -----------------------------------------------------------------------------
void ble_hid_demo_task_joystick(void *pvParameters) 
{
    int8_t x = -127, y = -127;
    uint8_t buttons = 0;

    while (1) 
    {
        if (s_ble_hid_param.hid_dev) 
        {
            send_gamepad(buttons++, x++, y++);   // botón1 pulsado, eje X a la derecha    
            
            if (x>126)
                x = -127;    

            if (y>126)
                y = -127;

            if (buttons==255)
                buttons = 0;        


        }



        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// -----------------------------------------------------------------------------
// BLE HID config
// -----------------------------------------------------------------------------
static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = joystickReportMap,
        .len = sizeof(joystickReportMap)
    },
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05E1,
    .version            = 0x0100,
    .device_name        = "ESP Joystick",
    .manufacturer_name  = "Espressif",
    .serial_number      = "5678YYYYYYY",
    .report_maps        = ble_report_maps,
    .report_maps_len    = 1
};

//----------------------------------------------------------------------------------------------------
void ble_hid_task_start_up(void)
{
    if (s_ble_hid_param.task_hdl) {
        // Task already exists
        return;
    }
 
    /* Nimble Specific */
    xTaskCreate(ble_hid_demo_task_joystick, "ble_hid_demo_task_mouse", 3 * 1024, NULL, configMAX_PRIORITIES - 3, &s_ble_hid_param.task_hdl);
 
}
//----------------------------------------------------------------------------------------------------
void ble_hid_task_shut_down(void)
{
    if (s_ble_hid_param.task_hdl) 
    {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}

// -----------------------------------------------------------------------------
// BLE HID callbacks and app_main()
// -----------------------------------------------------------------------------
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base,  int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    //esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "CONNECT");
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "DISCONNECT");
        esp_hid_ble_gap_adv_start();
        break;
    default:
        break;
    }
}

//--------------------------------------------------------------------------------
void ble_hid_device_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

//--------------------------------------------------------------------------------
void ble_store_config_init(void);

//--------------------------------------------------------------------------------
void app_main(void) 
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE); 
    ESP_ERROR_CHECK( ret );

    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GAMEPAD,
                                   ble_hid_config.device_name);

    ESP_ERROR_CHECK( ret );                                   

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_config, 
                                     ESP_HID_TRANSPORT_BLE,
                                      ble_hidd_event_callback,
                                      &s_ble_hid_param.hid_dev));

    ble_store_config_init();
    
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Start NimBLE task
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret) 
    {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

 
}
