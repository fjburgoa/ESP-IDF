#ifndef _HCM588L3L_H
#define _HCM588L3L_H

// Magnetometer data structure
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} hmc5883l_data_t;

esp_err_t hmc5883l_write_register(uint8_t reg_addr, uint8_t data);
esp_err_t hmc5883l_read_registers(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t hmc5883l_init(void);
esp_err_t hmc5883l_read_raw(hmc5883l_data_t *data);
esp_err_t hmc5883l_data_ready(bool *ready);


#endif
