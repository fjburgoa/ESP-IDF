#ifndef _ACOND_Y_MEDIDA_H
#define _ACOND_Y_MEDIDA_H

// Dirección I2C del MPU6050
#define MPU6050_ADDR 0x68

// Registros del MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

// Configuración del I2C
#define I2C_MASTER_SCL_IO    9        // Pin SCL
#define I2C_MASTER_SDA_IO    8        // Pin SDA
#define I2C_MASTER_FREQ_HZ   400000   // Frecuencia de I2C (400 kHz)
#define I2C_MASTER_PORT      I2C_NUM_0


void i2c_master_init(void) ;
esp_err_t mpu6050_register_write(uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
void mpu6050_read_accel_gyro(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void mpu6050_set_accel_scale(uint8_t scale) ;
void mpu6050_set_gyro_scale(uint16_t scale);
void mpu6050_set_dlpf(uint8_t bandwidth);
void mpu6050_calibrate(int16_t *accel_offsets, int16_t *gyro_offsets);
void mpu6050_set_offsets(int16_t *accel_offsets, int16_t *gyro_offsets);
void mpu6050_set_sample_rate(uint16_t rate_hz) ;

#endif