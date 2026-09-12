/**
 * @file Attitude_modes.c
 * @brief Selector de los algoritmos experimentales de pitch/roll del EFIS.
 *
 * Este módulo no adquiere sensores. Recibe las últimas muestras publicadas por
 * BNO055 y MPU6050, selecciona la fuente de actitud y devuelve un único par
 * roll/pitch para el horizonte artificial.
 */

#include "Attitude_modes.h"

#include <math.h>
#include <stddef.h>

#include "Hybrid_pitch_roll.h"

#define RAD_TO_DEG 57.29577951308232f
#define MIN_VECTOR_NORM 0.1f

static attitude_mode_t s_mode = ATTITUDE_MODE_BNO_AMG_ACCEL;

/* -------------------------------------------------------------------------- */

static bool vector_to_attitude(float x,
                               float y,
                               float z,
                               float *roll_deg,
                               float *pitch_deg)
{
    if ((roll_deg == NULL) || (pitch_deg == NULL))
    {
        return false;
    }

    const float norm = sqrtf((x * x) + (y * y) + (z * z));

    if ((!isfinite(norm)) || (norm < MIN_VECTOR_NORM))
    {
        return false;
    }

    /*
     * Convención histórica del proyecto. No sustituir por las fórmulas
     * genéricas habituales: los ejes del EFIS son X transversal, Y
     * longitudinal y Z vertical.
     */
    const float z_squared = z * z;

    *pitch_deg = atan2f(y, sqrtf((x * x) + z_squared)) * RAD_TO_DEG;
    *roll_deg = atan2f(x, sqrtf((y * y) + z_squared)) * RAD_TO_DEG;

    return true;
}

/* -------------------------------------------------------------------------- */

static void get_mpu_aircraft_acceleration(const mpu6050_data_t *mpu,
                                          float *x,
                                          float *y,
                                          float *z)
{
    float aircraft_x = mpu->accel_x_g;
    float aircraft_y = mpu->accel_y_g;
    const float aircraft_z = mpu->accel_z_g;

    /*
     * El BNO055 realiza internamente el remapeo V/H. El MPU6050 no dispone de
     * ese remapeo, por lo que se aplica aquí la misma transformación:
     *
     *      X_aircraft = -Y_sensor
     *      Y_aircraft =  X_sensor
     *      Z_aircraft =  Z_sensor
     */
    if (BNO055_get_mount_mode() == BNO055_MOUNT_HORIZONTAL)
    {
        const float sensor_x = aircraft_x;
        const float sensor_y = aircraft_y;

        aircraft_x = -sensor_y;
        aircraft_y = sensor_x;
    }

    *x = aircraft_x;
    *y = aircraft_y;
    *z = aircraft_z;
}

/* -------------------------------------------------------------------------- */

esp_err_t Attitude_set_mode(attitude_mode_t mode)
{
    bno055_operation_mode_t bno_mode;

    switch (mode)
    {
    case ATTITUDE_MODE_BNO_AMG_ACCEL:
        bno_mode = BNO055_OPERATION_MODE_AMG;
        break;

    case ATTITUDE_MODE_MPU_ACCEL:
    case ATTITUDE_MODE_BNO_IMUPLUS_GRAVITY:
    case ATTITUDE_MODE_FUSION_NORM:
    case ATTITUDE_MODE_FUSION_DOUBLE:
        /*
         * El modo MPU puro no necesita al BNO para calcular el horizonte, pero
         * se deja el BNO en IMUPLUS para disponer inmediatamente de GRAVITY al
         * cambiar a los modos 3, 5 o 6.
         */
        bno_mode = BNO055_OPERATION_MODE_IMUPLUS;
        break;

    case ATTITUDE_MODE_BNO_NDOF_GRAVITY:
        bno_mode = BNO055_OPERATION_MODE_NDOF;
        break;

    default:
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t err = BNO055_set_operation_mode(bno_mode);

    if (err != ESP_OK)
    {
        return err;
    }

    s_mode = mode;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

attitude_mode_t Attitude_get_mode(void)
{
    return s_mode;
}

/* -------------------------------------------------------------------------- */

const char *Attitude_get_mode_name(attitude_mode_t mode)
{
    switch (mode)
    {
    case ATTITUDE_MODE_BNO_AMG_ACCEL:
        return "BNO AMG / ACC";

    case ATTITUDE_MODE_MPU_ACCEL:
        return "MPU6050 / ACC";

    case ATTITUDE_MODE_BNO_IMUPLUS_GRAVITY:
        return "IMUPLUS / GRAVITY";

    case ATTITUDE_MODE_BNO_NDOF_GRAVITY:
        return "NDOF / GRAVITY";

    case ATTITUDE_MODE_FUSION_NORM:
        return "FUSION |a|";

    case ATTITUDE_MODE_FUSION_DOUBLE:
        return "FUSION |a| + DIR";

    default:
        return "UNKNOWN";
    }
}

/* -------------------------------------------------------------------------- */

attitude_output_t Attitude_compute(const bno055_data_t *bno,
                                   const mpu6050_data_t *mpu)
{
    attitude_output_t output = {0};

    if ((bno == NULL) || (mpu == NULL))
    {
        return output;
    }

    switch (s_mode)
    {
    case ATTITUDE_MODE_BNO_AMG_ACCEL:
        if (!bno->valid)
        {
            return output;
        }

        /*
         * En AMG, BNO055_processing calcula roll/pitch a 25 Hz mediante el
         * mismo filtro complementario utilizado históricamente por el EFIS:
         * acelerómetro + giróscopo, con BNO055_ATTITUDE_TAU_S.
         */
        output.roll_deg = bno->roll_deg;
        output.pitch_deg = bno->pitch_deg;
        output.valid = true;
        break;

    case ATTITUDE_MODE_MPU_ACCEL:
        if (!mpu->valid)
        {
            return output;
        }

        /*
         * El MPU6050 mantiene su propio filtro complementario independiente,
         * actualizado dentro de la tarea del sensor al mismo periodo nominal
         * de 40 ms. Así no dependemos de la frecuencia del WebSocket.
         */
        output.roll_deg = mpu->roll_deg;
        output.pitch_deg = mpu->pitch_deg;
        output.valid = true;
        break;

    case ATTITUDE_MODE_BNO_IMUPLUS_GRAVITY:
    case ATTITUDE_MODE_BNO_NDOF_GRAVITY:
        if (!bno->valid)
        {
            return output;
        }

        output.valid = vector_to_attitude(bno->gravity_ms2.x,
                                          bno->gravity_ms2.y,
                                          bno->gravity_ms2.z,
                                          &output.roll_deg,
                                          &output.pitch_deg);
        break;

    case ATTITUDE_MODE_FUSION_NORM:
    case ATTITUDE_MODE_FUSION_DOUBLE:
    {
        if ((!bno->valid) || (!mpu->valid))
        {
            return output;
        }

        float accel_x;
        float accel_y;
        float accel_z;

        get_mpu_aircraft_acceleration(mpu,
                                      &accel_x,
                                      &accel_y,
                                      &accel_z);

        hybrid_pitch_roll_data_t hybrid;

        if (s_mode == ATTITUDE_MODE_FUSION_DOUBLE)
        {
            hybrid = Hybrid_pitch_roll_update_double(
                accel_x,
                accel_y,
                accel_z,
                bno->gravity_ms2.x,
                bno->gravity_ms2.y,
                bno->gravity_ms2.z);
        }
        else
        {
            hybrid = Hybrid_pitch_roll_update(
                accel_x,
                accel_y,
                accel_z,
                bno->gravity_ms2.x,
                bno->gravity_ms2.y,
                bno->gravity_ms2.z);
        }

        output.roll_deg = hybrid.roll_deg;
        output.pitch_deg = hybrid.pitch_deg;
        output.accel_weight = hybrid.accel_weight;
        output.accel_norm_ms2 = hybrid.accel_norm_ms2;
        output.gravity_norm_ms2 = hybrid.gravity_norm_ms2;
        output.accel_error_ms2 = hybrid.accel_error_ms2;
        output.angle_error_deg = hybrid.angle_error_deg;
        output.accel_mod_weight = hybrid.accel_mod_weight;
        output.accel_angle_weight = hybrid.accel_angle_weight;
        output.valid = hybrid.valid;
        break;
    }

    default:
        break;
    }

    return output;
}
