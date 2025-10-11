/*
Este código,junto con acond_y_medida.c y procesado.c leen por I2C el MPU6050
e imprimen por pantalla los ángulos de Euler
*/

#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "procesado.h"
#include "acond_y_medida.h"

EulerAngles angles1 = {0.0, 0.0, 0.0}; // Ángulos de Euler iniciales - método 1
EulerAngles angles2 = {0.0, 0.0, 0.0}; // Ángulos de Euler iniciales - método 2
EulerAngles angles3 = {0.0, 0.0, 0.0}; // Ángulos de Euler iniciales - método 2

// Tag para logging
static const char *TAG = "MPU6050";


/************************************************************************************************************************************ */
void app_main() {
    // Inicializar I2C
    i2c_master_init();

    // Configurar el MPU6050
    mpu6050_register_write(MPU6050_PWR_MGMT_1, 0x00); // Despertar el sensor

    // Configurar la frecuencia de muestreo a 1000 Hz
    mpu6050_set_sample_rate(1000);

    // Configurar el rango del giroscopio y acelerómetro
    mpu6050_set_gyro_scale(1000); //  ±250 °/s
    mpu6050_set_accel_scale(4);   // ±4g

    // Configurar el filtro de paso bajo (DLPF)
    mpu6050_set_dlpf(4); // Ancho de banda de 21 Hz

    // Variables para almacenar los offsets
    int16_t accel_offsets[3] = {0, 0, 0};
    int16_t gyro_offsets[3] = {0, 0, 0};

    // Realizar la autocalibración
    ESP_LOGI(TAG, "Calibrando el sensor...");
    mpu6050_calibrate(accel_offsets, gyro_offsets);

    // Aplicar los offsets
    //mpu6050_set_offsets(accel_offsets, gyro_offsets);
    ESP_LOGI(TAG, "Calibración completada.");

    // Variables para almacenar los datos
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    //tiempo en cada medida
    float dT = 0.05;

    // Inicialización de q
    float Q[4] = {0};
    initial_conditions_quaternions(Q, angles3);   

    int j = 0;

    while (1) {
        // Leer los datos del sensor
        mpu6050_read_accel_gyro(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

        gyro_x -= gyro_offsets[0];
        gyro_y -= gyro_offsets[1];
        gyro_z -= gyro_offsets[2];

        // Mostrar los datos en bruto de aceleracion y velocidad angular por el puerto serie
        //ESP_LOGI(TAG, "Acc: AX=%6d, AY=%6d, AZ=%6d // Giro: WX=%6d, WY=%6d, WZ=%6d", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        float angular_velocity_deg_1 = raw_to_angular_velocity_deg(gyro_x);
        float angular_velocity_deg_2 = raw_to_angular_velocity_deg(gyro_y);
        float angular_velocity_deg_3 = raw_to_angular_velocity_deg(gyro_z);
    
        // Convertir de °/s a rad/s
        float p = deg_to_rad(angular_velocity_deg_1);
        float q = deg_to_rad(angular_velocity_deg_2);
        float r = deg_to_rad(angular_velocity_deg_3);

        // Mostrar los datos de velocidad angular bruto y procesados por el puerto serie 
        //ESP_LOGI(TAG, "RAW giro: WX=%6d, WY=%6d, WZ=%6d; Giro: p=%.1f, q=%.1f, r=%.1f rad/s", gyro_x, gyro_y, gyro_z,p,q,r);

        //angles1 = update_euler_angles(p, q, r, dT, angles1);
        
        //angles2 =  calcularT(p, q, r, dT); 

        angles3 = calcularQ(p, q, r, Q, dT);

        //printf("%03d s, Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, p=%.1f, q=%.1f, r=%.1f rad/s \n", j++%100, (180/M_PI)*angles1.roll, (180/M_PI)*angles1.pitch, (180/M_PI)*angles1.yaw, p,q,r);
        //printf("%03d s, Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, p=%.1f, q=%.1f, r=%.1f rad/s \n", j++%100, (180/M_PI)*angles2.roll, (180/M_PI)*angles2.pitch, (180/M_PI)*angles2.yaw, p,q,r);
        printf("%03d s, Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, p=%.1f, q=%.1f, r=%.1f rad/s \n", j++%100, (180/M_PI)*angles3.roll, (180/M_PI)*angles3.pitch, (180/M_PI)*angles3.yaw, p,q,r);

        // Esperar T milisegundos
        vTaskDelay(dT*1000 / portTICK_PERIOD_MS);
    }
}