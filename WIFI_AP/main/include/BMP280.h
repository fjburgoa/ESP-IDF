#ifndef BMP280_H
#define BMP280_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t bmp280_start(void);

    /* Últimas medidas válidas calculadas por la tarea BMP280. */
    extern float altitude;
    extern float temperature;
    extern float pressure_hpa;

    /*
     * El QNH sigue siendo propiedad del módulo WebSocket en la arquitectura
     * actual. Esta declaración se conserva por compatibilidad con el proyecto.
     */
    extern float s_qnh_hpa;

#ifdef __cplusplus
}
#endif

#endif
