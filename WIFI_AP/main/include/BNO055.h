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
         * Aceleración total en los ejes lógicos del avión, ya corregidos según
         * el modo de montaje seleccionado:
         *
         *   +X -> ala izquierda
         *   +Y -> morro
         *   +Z -> arriba
         *
         * Se publica en G para mantener compatibilidad con el EFIS.
         */
        float accel_x_g;
        float accel_y_g;
        float accel_z_g;
        float accel_total_g;

        /* Aceleración total original del BNO055 [m/s²].
        ES la aceleración específica total medida. Contiene el efecto de la gravedad y de las aceleraciones debidas al movimiento del dispositivo.
        */
        bno055_vector3f_t acceleration_ms2;

        /* Aceleración lineal calculada por el motor de fusión [m/s²]. */
        bno055_vector3f_t linear_acceleration_ms2;

        /* Vector de gravedad calculado por el motor de fusión [m/s²]. */
        bno055_vector3f_t gravity_ms2;

        /* Campo magnético [µT]. */
        bno055_vector3f_t magnetic_field_ut;

        /* Velocidades angulares del cuerpo [deg/s]. */
        float gyro_x_dps;
        float gyro_y_dps;
        float gyro_z_dps;

        /*
         * Actitud publicada por el driver:
         *
         *   heading_deg -> rumbo alrededor de Z
         *   roll_deg    -> alabeo alrededor de Y (longitudinal)
         *   pitch_deg   -> cabeceo alrededor de X (transversal)
         */
        float heading_deg;
        float roll_deg;
        float pitch_deg;

        /* Cuaternión de orientación proporcionado por el BNO055. */
        bno055_quaternionf_t quaternion;

        /*
         * Velocidad de cambio de rumbo [deg/s].
         * Se calcula derivando heading_deg, con unwrap y filtro paso bajo.
         * Se utiliza en el coordinador de giro.
         */
        float yaw_rate_dps;

        /*
         * Posición de la bola del coordinador.
         * Se obtiene de la aceleración específica transversal medida en X
         * (+X hacia el ala izquierda) y se filtra.
         */
        float slip_ball_deg;

        /*
         * Magnitud total de aceleración específica:
         *   g_current = sqrt(gx^2 + gy^2 + gz^2)
         *   g_max / g_min retienen los extremos desde el último RESET.
         */
        float g_current;
        float g_max;
        float g_min;

        /* Temperatura interna del BNO055 [°C]. */
        int8_t temperature_c;

        /* Estado de calibración, 0..3. */
        uint8_t calibration_system;
        uint8_t calibration_gyro;
        uint8_t calibration_accel;
        uint8_t calibration_mag;

        bool valid;

    } bno055_data_t;

    /**
     * @brief Inicializa el BNO055 en el modo seleccionado en config.h y crea su tarea periódica.
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
     * propio BNO055, por lo que acelerómetro, giróscopo, magnetómetro,
     * gravedad, aceleración lineal, Euler y cuaternión quedan expresados
     * en el mismo sistema lógico del avión.
     */
    esp_err_t BNO055_set_mount_mode(bno055_mount_mode_t mode);

    /**
     * @brief Devuelve el modo de montaje actualmente configurado.
     */
    bno055_mount_mode_t BNO055_get_mount_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
