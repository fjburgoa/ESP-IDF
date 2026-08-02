/*
Este código, imprime "Hola Mundo" por el OLED
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "SH1106.h"
#include "BM280.h"
#include "HCM588L3L.h"
#include "MPU6050.h"

#include "esp_log.h"
#include "esp_check.h"
#include <math.h>

#define PULS_ISR 0 // GPIO0 -> BOOT

#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

int flag_int_ext = false;
float f_qnh = 1013.0;

//-------------ISR---------------------
static void IRAM_ATTR ExtPinO_ISR_handler(void *args)
{
    flag_int_ext = true;
}

//------------------------------------
void app_main(void)
{
    //------------------------------------

    gpio_config_t myGPIOconfig; // estructura de configuración input

    // Se configura la estructura gpio_config_t
    myGPIOconfig.pin_bit_mask = 1ULL << PULS_ISR; // GPIO PULS_ISR asociado a la
                                                  // interrupción
    myGPIOconfig.mode = GPIO_MODE_INPUT;          // GPIO es entrada
    myGPIOconfig.pull_up_en = true;               // pull-up habilitada
    myGPIOconfig.pull_down_en = false;            // pull-down deshabilitada
    myGPIOconfig.intr_type = GPIO_INTR_NEGEDGE;   // Flanco de bajada

    gpio_config(&myGPIOconfig); // registra el pin y su configuración
    printf("Registra ISR\n");
    gpio_install_isr_service(0); // registra la rutina la ISR
    gpio_isr_handler_add(PULS_ISR, ExtPinO_ISR_handler, NULL);

    //------------------------------------

    ESP_ERROR_CHECK(i2c_master_init());
    sh1106_init();
    ESP_ERROR_CHECK(bmp280_init());
    ESP_ERROR_CHECK(hmc5883l_init());

    sh1106_clear();

    //-----------------------------------
    init_MPU6050();

    // Variables para almacenar los offsets
    int16_t accel_offsets_MPU0[3] = {0, 0, 0};
    int16_t gyro_offsets_MPU0[3] = {0, 0, 0};

// Realizar la autocalibración
#define MPU0 0
    mpu6050_calibrate(MPU0, accel_offsets_MPU0, gyro_offsets_MPU0);

    // Variables para almacenar los datos
    int16_t accel_x_MPU0, accel_y_MPU0, accel_z_MPU0 = 0.0f;
    int16_t gyro_x_MPU0, gyro_y_MPU0, gyro_z_MPU0 = 0.0f;

    //-----------------------------------

#define TEXT_LEN 25

    float temp = 0.0f;
    float pressure = 0.0f;
    float heading = 0.0f;
    float altitude = 0.0f;

    bool HCM588L3L_ready = false;

    char texto[TEXT_LEN] = {0};

    float acc_accum = 0.0f;

    while (1)
    {
        // Leer los datos del sensor
        mpu6050_read_accel_gyro(MPU0, &accel_x_MPU0, &accel_y_MPU0, &accel_z_MPU0, &gyro_x_MPU0, &gyro_y_MPU0, &gyro_z_MPU0);

        float accel_x = (float)accel_x_MPU0 / 16535.0f;
        float accel_y = (float)accel_y_MPU0 / 16535.0f;
        float accel_z = (float)accel_z_MPU0 / 16535.0f;

        float acc = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

        acc_accum = (acc > acc_accum) ? acc : acc_accum;

        memset(texto, 0, TEXT_LEN);
        sprintf(texto, "Gs:  %.2f G ", acc_accum);
        sh1106_draw_text(2, 4, texto); // abajo

        if (bmp280_read_measurement(&temp, &pressure) == ESP_OK)
        {
            altitude = bmp280_altitude_m(pressure, f_qnh);

            memset(texto, 0, TEXT_LEN);
            sprintf(texto, "QNH: %d hPa ", (int)f_qnh);
            sh1106_draw_text(2, 0, texto); // arriba

            memset(texto, 0, TEXT_LEN);
            sprintf(texto, "ALT: %.1f m ", altitude);
            sh1106_draw_text(2, 1, texto); // centro

            memset(texto, 0, TEXT_LEN);
            sprintf(texto, "P:   %.1f hPa ", pressure);
            sh1106_draw_text(2, 2, texto); // centro

            memset(texto, 0, TEXT_LEN);
            sprintf(texto, "T:   %.2f^C ", temp);
            sh1106_draw_text(2, 3, texto); // abajo

            /*
                        float ft_min = 123.4;
                        memset(texto, 0, TEXT_LEN);
                        sprintf(texto, "Var: %.1f ft.min ", ft_min);
                        sh1106_draw_text(2, 7, texto); // abajo
            */
        }
        else
        {
            pressure = -1;
        }

        if (hmc5883l_data_ready(&HCM588L3L_ready) == ESP_OK && HCM588L3L_ready)
        {
            heading = hmc5883l_calculate_heading();

            float normalized_heading = fmodf(heading, 360.0f);

            if (normalized_heading < 0.0f)
                normalized_heading += 360.0f;

            const char *direction = heading_to_direction(normalized_heading);

            memset(texto, 0, TEXT_LEN);

            snprintf(texto, TEXT_LEN, "HDG: %.0f^%s    ", normalized_heading, direction);

            sh1106_draw_text(2, 5, texto);
        }
        else
        {
            heading = -1.0;
        }

        // printf("T = %.2f ºC | P = %.2f hPa | h = %.1f m | HDG = %.2f deg | a = %.2f m/s2 \n", temp, pressure, altitude, heading, acc);
        // printf("ax = %.2f G's  ay = %.2f G's  az= %.2f G's \n", accel_x, accel_y, accel_z);

        vTaskDelay(pdMS_TO_TICKS(50));

        if (flag_int_ext)
        {
            acc_accum = 0;
            flag_int_ext = false;
            f_qnh = f_qnh + 1.0;
            if (f_qnh > 1030)
                f_qnh = 980;
        }
    }
}
