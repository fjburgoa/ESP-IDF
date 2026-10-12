
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define MPU6050_ADDR 		0x68	// Dirección I2C del MPU6050

#define MPU6050_ACCEL_XOUT_H	0x3B	// Registros del MPU6050
#define MPU6050_PWR_MGMT_1  	0x6B

#define I2C_MASTER_SCL_IO    9      	// Pin SCL
#define I2C_MASTER_SDA_IO    8      	// Pin SDA
#define I2C_MASTER_FREQ_HZ   400000 	// Frecuencia de I2C (400 kHz)
#define I2C_MASTER_PORT      I2C_NUM_0  	// 0

//---------Escribe registro de MPU6050------------------------------
esp_err_t mpu6050_register_write(uint8_t reg_addr, uint8_t data) 
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_PORT, 
                                      MPU6050_ADDR, 
                                      write_buf, 
                                      sizeof(write_buf), 
                                      pdMS_TO_TICKS(1000) );
}

//---------Lee registro de MPU6050------------------------------
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len) 
{
    return i2c_master_write_read_device(I2C_MASTER_PORT, 
                                        MPU6050_ADDR, 
                                        &reg_addr, 
                                        1, 
                                        data, 
                                        len, 
                                        pdMS_TO_TICKS (1000) );
}
//---------Inicializa comunicación I2C en el ESP32-S3-------------
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = 
    {
        .mode 			    = I2C_MODE_MASTER,
        .sda_io_num 		= I2C_MASTER_SDA_IO,
        .scl_io_num 		= I2C_MASTER_SCL_IO,
        .sda_pullup_en 	    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en 	    = GPIO_PULLUP_ENABLE,
        .master.clk_speed 	= I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
 
    return ESP_OK;
}

//--------------lectura datos MPU6050 ----------------------------------
void mpu6050_read_accel_gyro(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,  
                             int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) 
{
    uint8_t data[14];
    mpu6050_register_read(MPU6050_ACCEL_XOUT_H, data, sizeof(data));

    *accel_x = (data[0] << 8) | data[1];
    *accel_y = (data[2] << 8) | data[3];
    *accel_z = (data[4] << 8) | data[5];
    *gyro_x  = (data[8] << 8) | data[9];
    *gyro_y  = (data[10] << 8) | data[11];
    *gyro_z  = (data[12] << 8) | data[13];
}

//----------------app_main-----------------------------------------
void app_main() 
{
    i2c_master_init();            	   	        // Inicializar I2C

    mpu6050_register_write(MPU6050_PWR_MGMT_1, 0x00); 	// Despertar el sensor

    // Variables para almacenar los datos
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;

    while (1) 
    {
        // Leer los datos del sensor
        mpu6050_read_accel_gyro(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

        // Muestra a través del terminal serie
        printf("A_x: %d, A_y: %d, A_z: %d; W_x: %d, W_y: %d, W_z: %d\n", 
                        acc_x, acc_y, acc_z,
                        gyro_x, gyro_y, gyro_z);

        vTaskDelay(pdMS_TO_TICKS(200));  // Esperar 200 ms.
    }
}

