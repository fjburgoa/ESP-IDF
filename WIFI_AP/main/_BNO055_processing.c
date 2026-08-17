/**
 * @file BNO055_processing.c
 * @brief Filtrado, estimación de actitud y magnitudes derivadas del BNO055.
 *
 * Responsabilidades:
 *   - Conversión de aceleraciones a G.
 *   - Cálculo del módulo total de G.
 *   - Pitch/roll a partir del acelerómetro.
 *   - Filtro complementario acelerómetro + giróscopo.
 *   - Filtro LPF de los tres ejes del giróscopo.
 *   - Proyección del giro sobre la vertical local.
 *   - Filtrado del régimen de giro.
 *   - Integración de yaw en modo AMG para demostración.
 *   - Reconstrucción de aceleración lateral y bola.
 *
 * Este fichero no accede al bus I2C ni a registros del BNO055.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "BNO055_processing.h"
#include "config.h"

/* -------------------------------------------------------------------------- */
/* Constantes físicas                                                         */
/* -------------------------------------------------------------------------- */

#define STANDARD_GRAVITY_MS2 9.80665f
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f

/* -------------------------------------------------------------------------- */
/* Filtro del régimen de giro                                                 */
/* -------------------------------------------------------------------------- */

#define TURN_RATE_FILTER_TAU_S 0.75f
#define TURN_RATE_DEADBAND_DPS 0.10f

/* -------------------------------------------------------------------------- */
/* Filtro previo de giróscopos                                                */
/* -------------------------------------------------------------------------- */

#define GYRO_FILTER_TAU_S 0.15f

/* -------------------------------------------------------------------------- */
/* Bola                                                                       */
/* -------------------------------------------------------------------------- */

#define SLIP_BALL_FILTER_TAU_S 1.5f
#define SLIP_BALL_LIMIT_DEG 25.0f
#define SLIP_BALL_DEADBAND_DEG 0.8f
#define SLIP_BALL_GAIN 5.0f

/* -------------------------------------------------------------------------- */

static void bno055_compute_accel_attitude(
    float ax,
    float ay,
    float az,
    float *roll_deg,
    float *pitch_deg)
{
    const float az2 = az * az;

    *pitch_deg = atan2f(ay, sqrtf(ax * ax + az2)) * RAD_TO_DEG;
    *roll_deg = atan2f(ax, sqrtf(ay * ay + az2)) * RAD_TO_DEG;
}

/* -------------------------------------------------------------------------- */
/* ESTIMACIÓN DE ROLL Y PITCH MEDIANTE FILTRO COMPLEMENTARIO                  */
/* -------------------------------------------------------------------------- */

/*
 * Combina dos estimaciones de la actitud:
 *
 *   1. Acelerómetro:
 *      Proporciona una referencia absoluta de roll y pitch respecto a la
 *      gravedad. Es estable a largo plazo, pero sensible a las aceleraciones
 *      producidas por el movimiento del vehículo.
 *
 *   2. Giróscopo:
 *      Proporciona las velocidades angulares wx y wy. Integrándolas durante
 *      dt_s obtenemos la variación de pitch y roll. La respuesta es rápida,
 *      pero la integración acumula error (drift).
 *
 * El filtro complementario combina ambas:
 *
 *   pitch[k] = alpha * (pitch[k-1] + S_pitch * wx * dt) + (1-alpha) * pitch_acc
 *
 *   roll[k]  = alpha * (roll[k-1]  + S_roll  * wy * dt) + (1-alpha) * roll_acc
 *
 * donde:
 *
 *   wx, wy     = velocidades angulares medidas por el giróscopo [deg/s]
 *   dt         = periodo entre muestras [s]
 *   pitch_acc  = pitch obtenido exclusivamente del acelerómetro [deg]
 *   roll_acc   = roll obtenido exclusivamente del acelerómetro [deg]
 *   S_pitch    = BNO055_PITCH_GYRO_SIGN
 *   S_roll     = BNO055_ROLL_GYRO_SIGN
 *
 * El coeficiente del filtro es:
 *
 *                  tau
 *   alpha = -------------------
 *              tau + dt
 *
 * siendo tau = BNO055_ATTITUDE_TAU_S.
 *
 * Un alpha próximo a 1 da más peso al giróscopo:
 *   - respuesta rápida
 *   - menor sensibilidad a aceleraciones lineales
 *   - mayor posibilidad de drift
 *
 * Un alpha menor da más peso al acelerómetro:
 *   - corrige antes el drift
 *   - sigue mejor cambios lentos de actitud
 *   - es más sensible a aceleraciones del vehículo
 *
 * En la primera muestra no se integra el giróscopo. El estado inicial de
 * pitch y roll se toma directamente de la actitud calculada mediante el
 * acelerómetro.
 */

static void bno055_calcula_pitch_roll_filtro_complementario(
    bno055_processing_state_t *state,
    const bno055_vector3f_t *accel_ms2,
    float gyro_x_dps,
    float gyro_y_dps,
    float dt_s,
    float *roll_deg,
    float *pitch_deg)
{
    float roll_acc_deg = 0.0f;
    float pitch_acc_deg = 0.0f;

    /*
     * Calcula una estimación absoluta de roll y pitch a partir de la
     * dirección aparente de la gravedad medida por el acelerómetro.
     */
    bno055_compute_accel_attitude(accel_ms2->x, accel_ms2->y, accel_ms2->z, &roll_acc_deg, &pitch_acc_deg);

    /*
     * Primera muestra.
     *
     * Todavía no existe una actitud anterior sobre la que integrar las
     * velocidades angulares, por lo que inicializamos el filtro con la
     * actitud obtenida exclusivamente del acelerómetro.
     */
    if (!state->attitude_initialized)
    {
        state->roll_est_deg = roll_acc_deg;
        state->pitch_est_deg = pitch_acc_deg;
        state->attitude_initialized = true;
    }
    else
    {
        /*
         * Predicción mediante el giróscopo.
         *
         * Integramos la velocidad angular durante dt_s:
         *
         *   pitch_gyro[k] = pitch[k-1] + S_pitch * wx * dt
         *   roll_gyro[k]  = roll[k-1]  + S_roll  * wy * dt
         *
         * Los factores SIGN permiten adaptar el signo del giróscopo a
         * nuestro sistema de referencia.
         */
        const float pitch_gyro_deg = state->pitch_est_deg + BNO055_PITCH_GYRO_SIGN * gyro_x_dps * dt_s;

        const float roll_gyro_deg = state->roll_est_deg + BNO055_ROLL_GYRO_SIGN * gyro_y_dps * dt_s;

        /*
         * Coeficiente del filtro complementario:
         *
         *               tau
         *   alpha = ------------
         *             tau + dt
         *
         * La parte procedente del acelerómetro recibe un peso (1-alpha).
         */
        float alpha = BNO055_ATTITUDE_TAU_S / (BNO055_ATTITUDE_TAU_S + dt_s);

        /*
         * Protección numérica. En funcionamiento normal alpha debe estar
         * siempre comprendido entre 0 y 1.
         */
        if (!isfinite(alpha))
            alpha = 0.98f;

        if (alpha < 0.0f)
            alpha = 0.0f;
        else if (alpha > 1.0f)
            alpha = 1.0f;

        /*
         * Fusión complementaria:
         *
         *   estimación = alpha * giróscopo + (1-alpha) * acelerómetro
         *
         * El giróscopo aporta principalmente la dinámica rápida y el
         * acelerómetro proporciona la referencia absoluta que corrige
         * progresivamente el drift de la integración.
         */
        state->pitch_est_deg = alpha * pitch_gyro_deg + (1.0f - alpha) * pitch_acc_deg;

        state->roll_est_deg = alpha * roll_gyro_deg + (1.0f - alpha) * roll_acc_deg;
    }

    /*
     * Publicamos la última estimación del filtro.
     */
    *pitch_deg = state->pitch_est_deg;
    *roll_deg = state->roll_est_deg;
}

/* -------------------------------------------------------------------------- */

static void bno055_filter_gyro_dps(
    bno055_processing_state_t *state,
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    float dt_s,
    float *gyro_x_filtered_dps,
    float *gyro_y_filtered_dps,
    float *gyro_z_filtered_dps)
{
    if (!isfinite(gyro_x_dps) ||
        !isfinite(gyro_y_dps) ||
        !isfinite(gyro_z_dps))
    {
        *gyro_x_filtered_dps = state->gyro_x_filtered_dps;
        *gyro_y_filtered_dps = state->gyro_y_filtered_dps;
        *gyro_z_filtered_dps = state->gyro_z_filtered_dps;
        return;
    }

    if (!state->gyro_filter_initialized)
    {
        state->gyro_x_filtered_dps = gyro_x_dps;
        state->gyro_y_filtered_dps = gyro_y_dps;
        state->gyro_z_filtered_dps = gyro_z_dps;

        state->gyro_filter_initialized = true;
    }
    else
    {
        /*
         * LPF de primer orden:
         *
         *     y[k] = y[k-1] + alpha * (x[k] - y[k-1])
         *
         *     alpha = 1 - exp(-dt/tau)
         */
        float alpha = 1.0f - expf(-dt_s / GYRO_FILTER_TAU_S);

        if (!isfinite(alpha) || (alpha < 0.0f))
            alpha = 0.0f;
        else if (alpha > 1.0f)
            alpha = 1.0f;

        state->gyro_x_filtered_dps +=
            alpha * (gyro_x_dps - state->gyro_x_filtered_dps);

        state->gyro_y_filtered_dps +=
            alpha * (gyro_y_dps - state->gyro_y_filtered_dps);

        state->gyro_z_filtered_dps +=
            alpha * (gyro_z_dps - state->gyro_z_filtered_dps);
    }

    *gyro_x_filtered_dps = state->gyro_x_filtered_dps;
    *gyro_y_filtered_dps = state->gyro_y_filtered_dps;
    *gyro_z_filtered_dps = state->gyro_z_filtered_dps;
}

/* -------------------------------------------------------------------------- */

static float bno055_compute_turn_rate_from_attitude_dps(
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    float roll_deg,
    float pitch_deg)
{
    /*
     * En AMG no existe gravity_ms2 fusionada. Reconstruimos la dirección de
     * la vertical local a partir del roll/pitch estimados:
     *
     *     g_hat_x = sin(roll)
     *     g_hat_y = sin(pitch)
     *     g_hat_z = sqrt(1 - g_hat_x² - g_hat_y²)
     *
     * La velocidad angular alrededor de la vertical local es:
     *
     *     omega_vertical = omega · g_hat
     */
    if (!isfinite(gyro_x_dps) ||
        !isfinite(gyro_y_dps) ||
        !isfinite(gyro_z_dps) ||
        !isfinite(roll_deg) ||
        !isfinite(pitch_deg))
    {
        return 0.0f;
    }

    const float roll_rad = roll_deg * DEG_TO_RAD;
    const float pitch_rad = pitch_deg * DEG_TO_RAD;

    const float gx = sinf(roll_rad);
    const float gy = sinf(pitch_rad);

    float gz2 = 1.0f - gx * gx - gy * gy;

    if (gz2 < 0.0f)
        gz2 = 0.0f;

    const float gz = sqrtf(gz2);

    float turn_rate_dps =
        gyro_x_dps * gx +
        gyro_y_dps * gy +
        gyro_z_dps * gz;

    /*
     * Convención gráfica del proyecto:
     * giro a derechas -> bastón hacia la derecha.
     */
    turn_rate_dps = -turn_rate_dps;

    if (fabsf(turn_rate_dps) < TURN_RATE_DEADBAND_DPS)
        turn_rate_dps = 0.0f;

    return turn_rate_dps;
}

/* -------------------------------------------------------------------------- */

static float bno055_compute_vertical_turn_rate_dps(
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps,
    const bno055_vector3f_t *gravity_ms2)
{
    if (gravity_ms2 == NULL)
    {
        return 0.0f;
    }

    const float gx = gravity_ms2->x;
    const float gy = gravity_ms2->y;
    const float gz = gravity_ms2->z;

    const float gravity_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    if (!isfinite(gravity_norm) || (gravity_norm < 1.0f))
    {
        return 0.0f;
    }

    float turn_rate_dps =
        (gyro_x_dps * gx +
         gyro_y_dps * gy +
         gyro_z_dps * gz) /
        gravity_norm;

    turn_rate_dps = -turn_rate_dps;

    if (fabsf(turn_rate_dps) < TURN_RATE_DEADBAND_DPS)
        turn_rate_dps = 0.0f;

    return turn_rate_dps;
}

/* -------------------------------------------------------------------------- */

static float bno055_filter_yaw_rate_dps(
    bno055_processing_state_t *state,
    float yaw_rate_dps,
    float dt_s)
{
    float alpha = 1.0f - expf(-dt_s / TURN_RATE_FILTER_TAU_S);

    if (!isfinite(alpha) || (alpha < 0.0f))
        alpha = 0.0f;
    else if (alpha > 1.0f)
        alpha = 1.0f;

    state->yaw_rate_filtered_dps +=
        alpha * (yaw_rate_dps - state->yaw_rate_filtered_dps);

    return state->yaw_rate_filtered_dps;
}

/* -------------------------------------------------------------------------- */

static float bno055_compute_slip_ball_deg(
    bno055_processing_state_t *state,
    float accel_x_g,
    float roll_deg,
    float dt_s)
{
    /*
     * El acelerómetro mide:
     *
     *     a_meas = g + a_linear
     *
     * La componente lateral gravitatoria, con nuestro convenio, es:
     *
     *     g_x / g = sin(roll)
     *
     * Por tanto:
     *
     *     a_lateral_g = accel_x_g - sin(roll)
     */
    if (!isfinite(accel_x_g) ||
        !isfinite(roll_deg))
    {
        return state->slip_ball_filtered_deg;
    }

    const float roll_rad = roll_deg * DEG_TO_RAD;
    const float gravity_x_g = sinf(roll_rad);
    const float lateral_accel_g = accel_x_g - gravity_x_g;

    float raw_ball_deg = atanf(lateral_accel_g) * RAD_TO_DEG;

    /*
     * Convención gráfica actual.
     */
    raw_ball_deg = -raw_ball_deg;

    if (fabsf(raw_ball_deg) < SLIP_BALL_DEADBAND_DEG)
    {
        raw_ball_deg = 0.0f;
    }

    float alpha = 1.0f - expf(-dt_s / SLIP_BALL_FILTER_TAU_S);

    if (!isfinite(alpha) || (alpha < 0.0f))
        alpha = 0.0f;
    else if (alpha > 1.0f)
        alpha = 1.0f;

    state->slip_ball_filtered_deg +=
        alpha * (raw_ball_deg - state->slip_ball_filtered_deg);

    if (state->slip_ball_filtered_deg >
        SLIP_BALL_LIMIT_DEG)
    {
        state->slip_ball_filtered_deg = SLIP_BALL_LIMIT_DEG;
    }
    else if (state->slip_ball_filtered_deg <
             -SLIP_BALL_LIMIT_DEG)
    {
        state->slip_ball_filtered_deg = -SLIP_BALL_LIMIT_DEG;
    }

    return state->slip_ball_filtered_deg;
}

/* -------------------------------------------------------------------------- */

static float bno055_integrate_yaw_deg(
    bno055_processing_state_t *state, float yaw_rate_dps, float dt_s)
{
    state->yaw_demo_deg += yaw_rate_dps * dt_s;

    while (state->yaw_demo_deg >= 360.0f)
        state->yaw_demo_deg -= 360.0f;

    while (state->yaw_demo_deg < 0.0f)
        state->yaw_demo_deg += 360.0f;

    return state->yaw_demo_deg;
}

/* -------------------------------------------------------------------------- */
/* API interna de procesamiento                                               */
/* -------------------------------------------------------------------------- */

void bno055_processing_reset(bno055_processing_state_t *state)
{
    if (state == NULL)
        return;

    *state = (bno055_processing_state_t){0};
}

/* -------------------------------------------------------------------------- */

void bno055_processing_reset_g_peaks(
    bno055_processing_state_t *state,
    float g_current)
{
    if ((state == NULL) || !isfinite(g_current))
        return;

    state->g_min = g_current;
    state->g_max = g_current;
    state->g_peaks_initialized = true;
}

/* -------------------------------------------------------------------------- */

void bno055_processing_process_sample(
    bno055_processing_state_t *state,
    bno055_data_t *data,
    float dt_s,
    bool use_internal_fusion)
{
    if ((state == NULL) || (data == NULL))
        return;

    /* ---------------------------------------------------------------------- */
    /* Aceleración en G y módulo total                                        */
    /* ---------------------------------------------------------------------- */

    data->accel_x_g = data->acceleration_ms2.x / STANDARD_GRAVITY_MS2;
    data->accel_y_g = data->acceleration_ms2.y / STANDARD_GRAVITY_MS2;
    data->accel_z_g = data->acceleration_ms2.z / STANDARD_GRAVITY_MS2;

    data->accel_total_g =
        sqrtf(data->accel_x_g * data->accel_x_g +
              data->accel_y_g * data->accel_y_g +
              data->accel_z_g * data->accel_z_g);

    /* ------------------------------------------------------------------------------ */
    /* Calcula Pitch y Roll sin utilizar la fusión interna NDOF del BNO055            */
    /* -------------------------------------------------------------------------------*/

    if (!use_internal_fusion)
    {
        bno055_calcula_pitch_roll_filtro_complementario(
            state,
            &data->acceleration_ms2,
            data->gyro_dps.x,
            data->gyro_dps.y,
            dt_s,
            &data->roll_deg,
            &data->pitch_deg);
    }

    /* ---------------------------------------------------------------------- */
    /* Régimen de giro                                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * El indicador de giro debe representar la velocidad angular alrededor
     * de la vertical local, y no simplemente la velocidad medida en el eje Z
     * del BNO055.
     *
     * Cuando el vehículo presenta roll o pitch, la vertical local deja de
     * coincidir con el eje Z del sensor. Por tanto, es necesario proyectar
     * el vector de velocidad angular:
     *
     *      omega = [wx  wy  wz]
     *
     * sobre la dirección de la vertical local.
     *
     * El procedimiento para obtener dicha vertical depende del modo de
     * funcionamiento del BNO055:
     *
     *   - NDOF: se utiliza directamente gravity_ms2 proporcionado por la
     *           fusión interna del BNO055.
     *
     *   - AMG:  como no disponemos de gravity_ms2, la dirección de la
     *           vertical se reconstruye a partir de los valores estimados
     *           de roll y pitch.
     */

    float yaw_rate_raw_dps = 0.0f;

    if (use_internal_fusion)
    {
        /*
         * Modo NDOF.
         *
         * El BNO055 proporciona una estimación del vector gravedad. Se proyecta
         * directamente el vector de velocidad angular sobre dicho vector:
         *
         *                     omega · g
         *      yaw_rate = -----------------
         *                       |g|
         *
         * De esta forma el régimen de giro queda referido a la vertical local
         * aunque el dispositivo tenga roll o pitch.
         */
        yaw_rate_raw_dps = bno055_compute_vertical_turn_rate_dps(
            data->gyro_dps.x,
            data->gyro_dps.y,
            data->gyro_dps.z,
            &data->gravity_ms2);
    }
    else
    {
        /*
         * Modo AMG.
         *
         * Antes de calcular el régimen de giro se filtran independientemente
         * las tres componentes del giróscopo mediante un filtro paso bajo.
         * Esto reduce ruido y pequeñas oscilaciones que, después de realizar
         * la proyección sobre la vertical, producirían movimiento innecesario
         * en el indicador de giro.
         */
        float gyro_x_filtered_dps = 0.0f;
        float gyro_y_filtered_dps = 0.0f;
        float gyro_z_filtered_dps = 0.0f;

        bno055_filter_gyro_dps(
            state,
            data->gyro_dps.x,
            data->gyro_dps.y,
            data->gyro_dps.z,
            dt_s,
            &gyro_x_filtered_dps,
            &gyro_y_filtered_dps,
            &gyro_z_filtered_dps);

        /*
         * En AMG no disponemos del vector gravity_ms2 calculado por la fusión
         * interna. La función reconstruye la dirección de la vertical local
         * utilizando roll y pitch y proyecta sobre ella el vector de velocidad
         * angular filtrado.
         *
         * Conceptualmente:
         *
         *      g_hat = f(roll, pitch)
         *
         *      yaw_rate = omega_filtered · g_hat
         *
         * siendo:
         *
         *      omega_filtered = [wx_f  wy_f  wz_f]
         */
        yaw_rate_raw_dps = bno055_compute_turn_rate_from_attitude_dps(
            gyro_x_filtered_dps,
            gyro_y_filtered_dps,
            gyro_z_filtered_dps,
            data->roll_deg,
            data->pitch_deg);
    }

    /*
     * Finalmente se aplica un segundo filtro paso bajo al régimen de giro ya
     * proyectado.
     *
     * Por tanto, en AMG existen dos niveles de filtrado:
     *
     *   1. Filtrado de wx, wy y wz antes de realizar la proyección.
     *   2. Filtrado del yaw_rate resultante después de la proyección.
     *
     * El primer filtro limpia las medidas del giróscopo; el segundo determina
     * principalmente la suavidad visual y la rapidez de respuesta del
     * indicador de giro.
     */
    data->yaw_rate_dps = bno055_filter_yaw_rate_dps(state, yaw_rate_raw_dps, dt_s);

    /* ---------------------------------------------------------------------- */
    /* Yaw integrado de demostración en AMG o del GPS                         */
    /* ---------------------------------------------------------------------- */

#if !HEADING_GPS
    data->heading_deg = bno055_integrate_yaw_deg(state, data->yaw_rate_dps, dt_s);
#endif

    /* ---------------------------------------------------------------------- */
    /* Bola                                                                   */
    /* ---------------------------------------------------------------------- */

    data->slip_ball_deg = SLIP_BALL_GAIN * bno055_compute_slip_ball_deg(state, data->accel_x_g, data->roll_deg, dt_s);

    /* ---------------------------------------------------------------------- */
    /* G-meter: valor actual y extremos desde el último RESET                  */
    /* ---------------------------------------------------------------------- */

    data->g_current = data->accel_total_g;

    if (!state->g_peaks_initialized)
    {
        bno055_processing_reset_g_peaks(state, data->g_current);
    }
    else
    {
        if (data->g_current < state->g_min)
            state->g_min = data->g_current;

        if (data->g_current > state->g_max)
            state->g_max = data->g_current;
    }

    data->g_min = state->g_min;
    data->g_max = state->g_max;
}
