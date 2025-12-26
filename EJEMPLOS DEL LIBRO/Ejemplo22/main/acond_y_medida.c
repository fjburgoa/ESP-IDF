#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "acond_y_medida.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "acond_y_medida.h"


// Inicializar I2C
void i2c_master_init() 
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// Escribir un byte en un registro del MPU6050 ****************************************************************************************
esp_err_t mpu6050_register_write(uint8_t mpuid, uint8_t reg_addr, uint8_t data) 
{    
    uint8_t write_buf[2] = {reg_addr, data};    
    return i2c_master_write_to_device(I2C_MASTER_PORT, MPU6050_ADDR+mpuid, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// Leer datos del MPU6050 **************************************************************************************************************
esp_err_t mpu6050_register_read(uint8_t mpuid, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_PORT, MPU6050_ADDR+mpuid, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Leer los valores del acelerómetro y giroscopio **************************************************************************************
void mpu6050_read_accel_gyro(uint8_t mpuid, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[14];
    mpu6050_register_read(mpuid,MPU6050_ACCEL_XOUT_H, data, sizeof(data));

    *accel_x = (data[0] << 8) | data[1];
    *accel_y = (data[2] << 8) | data[3];
    *accel_z = (data[4] << 8) | data[5];
    *gyro_x  = (data[8] << 8) | data[9];
    *gyro_y  = (data[10] << 8) | data[11];
    *gyro_z  = (data[12] << 8) | data[13];
}
// Establce  la escala del acelerometro ************************************************************************************************
void mpu6050_set_accel_scale(uint8_t mpuid,uint8_t scale) 
{
    uint8_t value;
    switch (scale) {
        case 2:
            value = 0x00; // AFS_SEL = 00 ±2g
            break;
        case 4:
            value = 0x08; // AFS_SEL = 01 ±4g
            break;
        case 8:
            value = 0x10; // AFS_SEL = 10 ±8g
            break;
        case 16:
            value = 0x18; // AFS_SEL = 11 ±16g   
            break;
        default:
            value = 0x00; // Por defecto, ±2g
            break;
    }
    mpu6050_register_write(mpuid, 0x1C, value); // Escribir en ACCEL_CONFIG
}
// Establce la escala del giroscopo ************************************************************************************************
void mpu6050_set_gyro_scale(uint8_t mpuid, uint16_t scale) {
    uint8_t value;
    switch (scale) {
        case 250:
            value = 0x00; // FS_SEL = 00
            break;
        case 500:
            value = 0x08; // FS_SEL = 01
            break;
        case 1000:
            value = 0x10; // FS_SEL = 10
            break;
        case 2000:
            value = 0x18; // FS_SEL = 11
            break;
        default:
            value = 0x00; // Por defecto, ±250 °/s
            break;
    }
    mpu6050_register_write(mpuid,0x1B, value); // Escribir en GYRO_CONFIG
}
// Establce el filtro digital LP  ************************************************************************************************
void mpu6050_set_dlpf(uint8_t mpuid, uint8_t bandwidth) 
{
 //   DLPF_CFG	Ancho de banda (Hz)	Retardo (ms)	Frecuencia de muestreo (kHz)
 //   0	        260	                0	            8
 //   1	        184	                2.0	            1
 //   2	        94	                3.0	            1
 //   3	        44	                4.9	            1
 //   4	        21	                8.5	            1
 //   5	        10	                13.8	        1
 //   6	        5	                19.0            1
 //   7	        Reservado	        -	            -

    if (bandwidth > 6) 
    {
        bandwidth = 6; // Evitar valores no válidos
    }
    mpu6050_register_write(mpuid, 0x1A, bandwidth); // Escribir en el registro CONFIG
}


// Calibración  ************************************************************************************************

#define CALIBRATION_SAMPLES 100  // Número de muestras para la calibración

void mpu6050_calibrate(uint8_t mpuid, int16_t *accel_offsets, int16_t *gyro_offsets) {
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};

    // Leer y sumar múltiples muestras
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;

        // Leer los datos del sensor
        mpu6050_read_accel_gyro(mpuid, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

        // Sumar los valores
        accel_sum[0] += accel_x;
        accel_sum[1] += accel_y;
        accel_sum[2] += accel_z;
        gyro_sum[0] += gyro_x;
        gyro_sum[1] += gyro_y;
        gyro_sum[2] += gyro_z;

        vTaskDelay(1 / portTICK_PERIOD_MS); // Pequeño retardo entre lecturas
    }

    // Calcular el promedio (offset)
    for (int i = 0; i < 3; i++) {
        accel_offsets[i] = accel_sum[i] / CALIBRATION_SAMPLES;
        gyro_offsets[i] = gyro_sum[i] / CALIBRATION_SAMPLES;
    }
}

// Set Offsets  ************************************************************************************************

void mpu6050_set_offsets(uint8_t mpuid, int16_t *accel_offsets, int16_t *gyro_offsets) {
    // Escribir offsets del acelerómetro
    mpu6050_register_write(mpuid, 0x06, (accel_offsets[0] >> 8) & 0xFF); // XA_OFFS_H
    mpu6050_register_write(mpuid, 0x07, accel_offsets[0] & 0xFF);        // XA_OFFS_L_TC
    mpu6050_register_write(mpuid, 0x08, (accel_offsets[1] >> 8) & 0xFF); // YA_OFFS_H
    mpu6050_register_write(mpuid, 0x09, accel_offsets[1] & 0xFF);        // YA_OFFS_L_TC
    mpu6050_register_write(mpuid, 0x0A, (accel_offsets[2] >> 8) & 0xFF); // ZA_OFFS_H
    mpu6050_register_write(mpuid, 0x0B, accel_offsets[2] & 0xFF);        // ZA_OFFS_L_TC
    
    // Escribir offsets del giroscopio
    mpu6050_register_write(mpuid, 0x13, (gyro_offsets[0] >> 8) & 0xFF);  // XG_OFFS_USRH
    mpu6050_register_write(mpuid, 0x14, gyro_offsets[0] & 0xFF);         // XG_OFFS_USRL
    mpu6050_register_write(mpuid, 0x15, (gyro_offsets[1] >> 8) & 0xFF);  // YG_OFFS_USRH
    mpu6050_register_write(mpuid, 0x16, gyro_offsets[1] & 0xFF);         // YG_OFFS_USRL
    mpu6050_register_write(mpuid, 0x17, (gyro_offsets[2] >> 8) & 0xFF);  // ZG_OFFS_USRH
    mpu6050_register_write(mpuid, 0x18, gyro_offsets[2] & 0xFF);         // ZG_OFFS_USRL
}

// Set Offsets  ************************************************************************************************

void mpu6050_set_sample_rate(uint8_t mpuid, uint16_t rate_hz) 
{
    if (rate_hz < 4 || rate_hz > 1000) {
        rate_hz = 1000; // Valor por defecto si está fuera de rango
    }
    uint8_t smplrt_div = (1000 / rate_hz) - 1;
    mpu6050_register_write(mpuid, 0x19, smplrt_div); // Escribir en SMPLRT_DIV
}