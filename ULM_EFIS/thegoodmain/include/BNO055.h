/**
 * @file BNO055.h
 * @brief Interfaz pública del módulo BNO055 del ULM-EFIS.
 *
 * Este es el único header que deben incluir los módulos externos.
 * La implementación se divide internamente en:
 *
 *   BNO055.c             -> tarea, estado y API pública.
 *   BNO055_driver.c      -> acceso al hardware BNO055.
 *   BNO055_processing.c  -> filtrado y procesado de señales.
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    /*
     * V: orientación original del proyecto (P1 del BNO055).
     *    X -> transversal (+ izquierda)
     *    Y -> longitudinal (+ morro)
     *    Z -> vertical (+ arriba)
     *
     * H: equipo girado 90 grados en el plano horizontal (P0).
     *    El propio BNO055 remapea los ejes de los sensores.
     */
    BNO055_MOUNT_VERTICAL = 0,
    BNO055_MOUNT_HORIZONTAL = 1
} bno055_mount_mode_t;

typedef enum
{
    BNO055_OPERATION_MODE_AMG = 0,
    BNO055_OPERATION_MODE_NDOF = 1
} bno055_operation_mode_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno055_vector3f_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} bno055_quaternionf_t;

typedef struct
{
    /*
     * Aceleración total en los ejes lógicos del avión:
     *
     *   +X -> ala izquierda
     *   +Y -> morro
     *   +Z -> arriba
     */
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float accel_total_g;

    /* Aceleración específica total medida por el BNO055 [m/s²]. */
    bno055_vector3f_t acceleration_ms2;

    /*
     * Aceleración lineal calculada por la fusión interna [m/s²].
     * Sólo válida cuando el modo de operación actual es NDOF.
     */
    bno055_vector3f_t linear_acceleration_ms2;

    /*
     * Vector gravedad calculado por la fusión interna [m/s²].
     * Sólo válido cuando el modo de operación actual es NDOF.
     */
    bno055_vector3f_t gravity_ms2;

    /* Campo magnético [µT]. */
    bno055_vector3f_t magnetic_field_ut;

    /* Velocidad angular del cuerpo [deg/s]. */
    bno055_vector3f_t gyro_dps;

    /*
     * Actitud publicada por el módulo:
     *
     *   heading_deg -> rumbo/yaw alrededor de Z.
     *   roll_deg    -> alabeo.
     *   pitch_deg   -> cabeceo.
     *
     * En NDOF proceden de la fusión interna del BNO055.
     * En AMG pitch/roll se estiman en BNO055_processing.c.
     * En AMG heading_deg puede contener el yaw integrado de demostración.
     */
    float heading_deg;
    float roll_deg;
    float pitch_deg;

    /* Cuaternión proporcionado por la fusión interna del BNO055. */
    bno055_quaternionf_t quaternion;

    /*
     * Régimen de giro alrededor de la vertical local [deg/s].
     * Se obtiene proyectando la velocidad angular sobre la vertical y
     * aplicando el filtrado correspondiente.
     */
    float yaw_rate_dps;

    /* Posición calculada de la bola del coordinador [deg]. */
    float slip_ball_deg;

    /*
     * Magnitud total de aceleración específica:
     *
     *   g_current = sqrt(gx² + gy² + gz²)
     *
     * g_max y g_min retienen los extremos desde el último RESET.
     */
    float g_current;
    float g_max;
    float g_min;

    /* Temperatura interna del BNO055 [°C]. */
    int8_t temperature_c;

    /* Estado de calibración 0..3. */
    uint8_t calibration_system;
    uint8_t calibration_gyro;
    uint8_t calibration_accel;
    uint8_t calibration_mag;

    bool valid;

} bno055_data_t;

/**
 * @brief Inicializa el BNO055 y crea su tarea periódica.
 *
 * El bus I2C debe estar previamente inicializado por main.c.
 */
esp_err_t BNO055_start(void);

/**
 * @brief Devuelve una instantánea atómica de los últimos datos válidos.
 */
bno055_data_t BNO055_get_data(void);

/**
 * @brief Reinicia G mínima/máxima al valor instantáneo actual.
 */
void BNO055_reset_accel_peaks(void);

/**
 * @brief Cambia la orientación física del módulo.
 *
 * La conmutación se realiza mediante AXIS_MAP_CONFIG/AXIS_MAP_SIGN del
 * propio BNO055.
 */
esp_err_t BNO055_set_mount_mode(bno055_mount_mode_t mode);

/**
 * @brief Devuelve el modo de montaje actualmente configurado.
 */
bno055_mount_mode_t BNO055_get_mount_mode(void);

/**
 * @brief Cambia en ejecución el modo de operación AMG/NDOF.
 *
 * La tarea periódica se suspende durante la transición. Tras el cambio se
 * invalidan la muestra anterior y los estados de filtros/estimadores.
 */
esp_err_t BNO055_set_operation_mode(bno055_operation_mode_t mode);

/**
 * @brief Devuelve el modo de operación actualmente activo.
 */
bno055_operation_mode_t BNO055_get_operation_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
