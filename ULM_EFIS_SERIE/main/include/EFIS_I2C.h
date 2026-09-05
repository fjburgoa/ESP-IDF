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

/* Añade un dispositivo de 7 bits al bus compartido. */
esp_err_t efis_i2c_add_device(uint16_t address,
                              i2c_master_dev_handle_t *device_handle);

/* Comprueba si una dirección responde con ACK. */
esp_err_t efis_i2c_probe(uint16_t address);

#ifdef __cplusplus
}
#endif

#endif /* EFIS_I2C_H */
