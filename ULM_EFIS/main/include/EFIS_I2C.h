#ifndef EFIS_I2C_H
#define EFIS_I2C_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inicializa una única vez el bus I2C compartido del EFIS. */
esp_err_t efis_i2c_init(void);

/* Devuelve el handle del bus ya inicializado, o NULL si aún no existe. */
i2c_master_bus_handle_t efis_i2c_get_bus(void);

/* Exclusión mutua para transacciones compuestas sobre el bus compartido. */
esp_err_t efis_i2c_lock(uint32_t timeout_ms);
void efis_i2c_unlock(void);

/* Añade un dispositivo usando los parámetros generales del bus. */
esp_err_t efis_i2c_add_device(uint16_t address,
                              i2c_master_dev_handle_t *device_handle);

/* Añade un dispositivo con frecuencia y tolerancia de clock-stretch propias. */
esp_err_t efis_i2c_add_device_ex(uint16_t address,
                                 uint32_t scl_speed_hz,
                                 uint32_t scl_wait_us,
                                 i2c_master_dev_handle_t *device_handle);

/* Comprueba si una dirección responde con ACK. */
esp_err_t efis_i2c_probe(uint16_t address);

/* Recupera el controlador/bus tras un timeout. */
esp_err_t efis_i2c_reset_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* EFIS_I2C_H */
