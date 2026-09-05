#ifndef BMP280_H
#define BMP280_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t bmp280_start(void);
    void bmp280_set_test_altitude(float altitude_m, bool enable);

    /* Últimas medidas válidas calculadas por la tarea BMP280. */
    extern float altitude;
    extern float temperature;
    extern float pressure_hpa;
    extern float vertical_speed;

    /*
     * El QNH sigue siendo propiedad del módulo WebSocket en la arquitectura
     * actual. Esta declaración se conserva por compatibilidad con el proyecto.
     */
    extern float s_qnh_hpa;

#ifdef __cplusplus
}
#endif

#endif
