/*
08.09.2025
Este programa, convierte los pedales + esp32s3 + 2xMPUs + LEDS en un controlador de rudder
- el esp32s3 en un joystick (GAMEPAD) con 32 botones y 6 ejes analógicos de 8 bits, la salida está en Rz, todas las demás a 0
- lectura de 2 x MPU6050 por I2C que dan la deplexión de los pedales
- control de 10 Leds en función de la deplexión de los pedales

- se puede probar con Win+R >> joy.cpl
*/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

#include "acond_y_medida.h"

static const char *TAG = "GAMEPAD";

//---------------------------------------------------------------------
#define MPU0 0 
#define MPU1 1 

#define PI 3.141592
#define dT 0.05

//según cableado de la tarjeta
#define LED0 47
#define LED1 48
#define LED2 45
#define LED3 35
#define LED4 36
#define LED5 37
#define LED6 38
#define LED7 7
#define LED8 6
#define LED9 4

#define N    10       //número de leds

#define OFF  1        //LED OFF (es pull-up) 
#define ON   0        //LED ON  (es pull-up)  

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
uint8_t EncienIndexLedOn(float angle) ;
 int8_t OutputJoystick(float angle);
 void envia(uint8_t n, uint8_t n_old);

//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
int DO[N] = {LED0,LED1,LED2,LED3,LED4,LED5,LED6,LED7,LED8,LED9};
uint8_t Led[N] = {0};

//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
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


//---------------------------------------------------------------------------------------------------------------------------------
//---------------------- Application ----------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------
uint8_t IndexLedOn(float angle) 
{
    //esta función devuelve un indce 0..9 en función de la deplexión del pedal

    // Asegurar que angle está dentro del rango [-1, 1]
    if (angle < -1.0f) angle = -1.0f;
    if (angle >  1.0f) angle =  1.0f;

    return (int)((angle + 1.0f) * (N - 1) / 2.0f);       // Mapear angle de [-1,1] a [0,N-1]
}

//---------------------------------------------------------------------------------------------------------------------------------
int8_t OutputJoystick(float angle) 
{
    //esta función devuelve un valor -127..127 en función de la deplexión del pedal

    // Asegurar que angle está dentro del rango [-1, 1]
    if (angle < -1.0f) angle = -1.0f;
    if (angle >  1.0f) angle =  1.0f;

    if ((angle < 0.1f)&&(angle > -0.1f))
       return 0;
    else
       return (int)(angle * 127);       
}

//---------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
 
    // Inicializar I2C
    i2c_master_init();

    // Configurar el MPU6050
    mpu6050_register_write(MPU0, MPU6050_PWR_MGMT_1, 0x00); // Despertar el sensor
    mpu6050_register_write(MPU1, MPU6050_PWR_MGMT_1, 0x00); // Despertar el sensor

    // Configurar la frecuencia de muestreo a 1000 Hz
    mpu6050_set_sample_rate(MPU0, 1000);
    mpu6050_set_sample_rate(MPU1, 1000);

    // Configurar el rango del giroscopio
    mpu6050_set_gyro_scale(MPU0, 250); //  ±250 °/s
    mpu6050_set_gyro_scale(MPU1, 250); //  ±250 °/s

    // Configurar el rango del acelerómetro
    mpu6050_set_accel_scale(MPU0, 2);   // ±4g
    mpu6050_set_accel_scale(MPU1, 2);   // ±4g

    // Configurar el filtro de paso bajo (DLPF)
    mpu6050_set_dlpf(MPU0, 6); // Ancho de banda de 21 Hz
    mpu6050_set_dlpf(MPU1, 6); // Ancho de banda de 21 Hz

    // Variables para almacenar los offsets
    int16_t accel_offsets_MPU0[3] = {0, 0, 0};
    int16_t gyro_offsets_MPU0[3]  = {0, 0, 0};
    int16_t accel_offsets_MPU1[3] = {0, 0, 0};
    int16_t gyro_offsets_MPU1[3]  = {0, 0, 0};

    // Realizar la autocalibración
    ESP_LOGI(TAG, "Calibrando el sensor MPU0...");
    mpu6050_calibrate(MPU0, accel_offsets_MPU0, gyro_offsets_MPU0);

    ESP_LOGI(TAG, "Calibrando el sensor MPU1...");
    mpu6050_calibrate(MPU1, accel_offsets_MPU1, gyro_offsets_MPU1);

    // Variables para almacenar los datos
    int16_t accel_x_MPU0, accel_y_MPU0, accel_z_MPU0;
    int16_t accel_x_MPU1, accel_y_MPU1, accel_z_MPU1;    
    int16_t gyro_x_MPU0, gyro_y_MPU0, gyro_z_MPU0;
    int16_t gyro_x_MPU1, gyro_y_MPU1, gyro_z_MPU1;

    //los 10 leds son salidas
    for (int i=0; i<N; i++)
       gpio_set_direction(DO[i],GPIO_MODE_OUTPUT);
 
    
    //------------USB INIT------------------------------------------------------------

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

    uint32_t  buttons = 0x0000;  
    uint8_t index;    
    int8_t rudder;

    while (1) 
    {

        // Leer los datos del sensor
        mpu6050_read_accel_gyro(MPU0, &accel_x_MPU0, &accel_y_MPU0, &accel_z_MPU0, &gyro_x_MPU0, &gyro_y_MPU0, &gyro_z_MPU0);
        mpu6050_read_accel_gyro(MPU1, &accel_x_MPU1, &accel_y_MPU1, &accel_z_MPU1, &gyro_x_MPU1, &gyro_y_MPU1, &gyro_z_MPU1);

        //calcular ángulo MPU0        
        float pitch_MPU0 = atan2(-accel_x_MPU0, sqrt(accel_y_MPU0 * accel_y_MPU0 + accel_z_MPU0* accel_z_MPU0)) * 180.0 / PI;

        //calcular ángulo MPU1
        float pitch_MPU1 = atan2(-accel_x_MPU1, sqrt(accel_y_MPU1 * accel_y_MPU1 + accel_z_MPU1* accel_z_MPU1)) * 180.0 / PI;

        //calcular ángulo total MPU1-MPU0
        float angle = (pitch_MPU1-pitch_MPU0)/19.0f;
        
        //Led Index
        index = IndexLedOn(angle);

        //Saca por pantalla
        //printf("%03d s, Pitch-0: %.2f deg, Pitch1: %.2f deg, dPtitch %.2f deg, %d \n", j++%100, pitch_MPU0, pitch_MPU1, pitch_MPU1-pitch_MPU0,index); 

        //LED control---------------
        for (uint8_t i = 0; i < N; i++)
        {             
            if (i == index)
                gpio_set_level(DO[i],ON);     //solo 1 estará encendido a la vez
            else
                gpio_set_level(DO[i],OFF);
        }        
        
        //Transmit USB--------------
        if (tud_mounted())  
        {
            rudder = OutputJoystick(angle);
            tud_hid_gamepad_report(
               1,                 // report_id
               0,         0, 0,   // ejes X, Y, Z
               -rudder  , 0, 0,   // Rz, Rx, Ry
               0,                 // hat (0 = neutral)
               buttons);          // botones

            ESP_LOGI(TAG, "rudder =%d ", rudder);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
