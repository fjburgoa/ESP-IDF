#include <stdio.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_ADDR 0x68 // Dirección I2C del MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B // Registros del MPU6050
#define MPU6050_PWR_MGMT_1 0x6B

#define I2C_MASTER_SCL_IO 9       // Pin SCL
#define I2C_MASTER_SDA_IO 8       // Pin SDA
#define I2C_MASTER_FREQ_HZ 400000 // Frecuencia de I2C (400 kHz)
#define I2C_MASTER_PORT I2C_NUM_0

// Handles de la nueva API I2C
static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t mpu6050_handle;

//---------Escribe registro de MPU6050------------------------------
esp_err_t mpu6050_register_write(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_transmit(mpu6050_handle,
                               write_buf,
                               sizeof(write_buf),
                               1000); // timeout en ms
}

//---------Lee registro de MPU6050------------------------------
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    /*
     * Envía primero la dirección del registro y, sin generar STOP
     * intermedio, realiza la lectura mediante repeated START.
     */
    return i2c_master_transmit_receive(mpu6050_handle,
                                       &reg_addr,
                                       1,
                                       data,
                                       len,
                                       1000); // timeout en ms
}

//---------Inicializa comunicación I2C en el ESP32-S3-------------
static esp_err_t i2c_master_init(void)
{
    // Configuración del BUS I2C
    i2c_master_bus_config_t bus_config =
        {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_PORT,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Configuración específica del MPU6050 conectado al bus
    i2c_device_config_t dev_config =
        {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = MPU6050_ADDR,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };

    ret = i2c_master_bus_add_device(i2c_bus_handle,
                                    &dev_config,
                                    &mpu6050_handle);

    if (ret != ESP_OK)
    {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
        return ret;
    }

    return ESP_OK;
}

//--------------lectura datos MPU6050 ----------------------------------
void mpu6050_read_accel_gyro(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                             int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t data[14];

    esp_err_t ret = mpu6050_register_read(MPU6050_ACCEL_XOUT_H,
                                          data,
                                          sizeof(data));

    if (ret != ESP_OK)
    {
        printf("Error leyendo MPU6050: %s\n", esp_err_to_name(ret));
        return;
    }

    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);

    *gyro_x = (int16_t)((data[8] << 8) | data[9]);
    *gyro_y = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z = (int16_t)((data[12] << 8) | data[13]);
}

//----------------app_main-----------------------------------------
void app_main(void)
{
    esp_err_t ret;

    ret = i2c_master_init();
    if (ret != ESP_OK)
    {
        printf("Error inicializando I2C: %s\n", esp_err_to_name(ret));
        return;
    }

    // Opcional: comprobar que el MPU6050 responde en 0x68
    ret = i2c_master_probe(i2c_bus_handle, MPU6050_ADDR, 1000);
    if (ret != ESP_OK)
    {
        printf("MPU6050 no encontrado en 0x%02X: %s\n",
               MPU6050_ADDR, esp_err_to_name(ret));
        return;
    }

    ret = mpu6050_register_write(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK)
    {
        printf("Error despertando MPU6050: %s\n", esp_err_to_name(ret));
        return;
    }

    // Variables para almacenar los datos
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;

    while (1)
    {
        // Leer los datos del sensor
        mpu6050_read_accel_gyro(&acc_x, &acc_y, &acc_z,
                                &gyro_x, &gyro_y, &gyro_z);

        // Muestra a través del terminal serie
        printf("A_x: %d, A_y: %d, A_z: %d; W_x: %d, W_y: %d, W_z: %d\n",
               acc_x, acc_y, acc_z,
               gyro_x, gyro_y, gyro_z);

        vTaskDelay(pdMS_TO_TICKS(200)); // Esperar 200 ms
    }
}
