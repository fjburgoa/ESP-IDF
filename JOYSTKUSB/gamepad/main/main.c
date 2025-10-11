/*
Este ejemplo convierte un esp32s3 en un joystick con 32 botones y 6 ejes analógicos de 8 bits
*/


#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

static const char *TAG = "GAMEPAD";

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

const uint8_t hid_report_descriptor[] = 
{    
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(1))   //descriptor del GAMEPAD
};

const char* hid_string_descriptor[5] = 
{
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "GAMEPAD",             // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

 
static const uint8_t hid_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

 

/********* Application ***************/

void app_main(void)
{
 
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor, // HID configuration descriptor for full-speed and high-speed are the same
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    int8_t x = -100, y = 0;
    uint32_t  buttons = 0x0001; // alterna el botón 1;    

    while (1) 
    {
        if (tud_mounted())  
        {
                tud_hid_gamepad_report(
                1,       // report_id
                0, x,    // ejes X, Y
                x, x,    // Z, Rz
                x, x,    // Rx, Ryx
                y,       // hat (0 = neutral)
                buttons  // botones
            );

            ESP_LOGI(TAG, "x=%d y=%d btn=0x%08lx", x, y, buttons);
          
            
            x++; 
            if (x >= 127)
            {
               x = -127; 
               y++; 
               buttons = buttons << 1 ;
            } 
            if (y >= 127)
            {
               x = -127;  
               y = -127;
            } 
            if (buttons==0x0000)    
                buttons = 0x0001;

        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
