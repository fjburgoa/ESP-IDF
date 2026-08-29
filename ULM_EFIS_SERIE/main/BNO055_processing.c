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

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

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

/* -------------------------------------------------------------------------- */
/* Filtro previo de giróscopos                                                */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Bola                                                                       */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

static void bno055_compute_accel_attitude(float ax, float ay, float az, float *roll_deg, float *pitch_deg)
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
static void bno055_update_complementary_attitude(
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
    /*
        ESP_LOGI(
            "ATT",
            "ACC=[%+.3f %+.3f %+.3f]  "
            "ACC_ATT=[P=%+.2f R=%+.2f]  "
            "GYRO=[%+.2f %+.2f]  "
            "EST=[P=%+.2f R=%+.2f]",
            accel_ms2->x,
            accel_ms2->y,
            accel_ms2->z,
            pitch_acc_deg,
            roll_acc_deg,
            gyro_x_dps,
            gyro_y_dps,
            state->pitch_est_deg,
            state->roll_est_deg);
    */
}

/*******************************************************/
/*     NUEVA FORMULACIÓN DEL CÁLCULO DE ROLL/PITCH     */
/*******************************************************/
static void bno055_update_adaptive_attitude(
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

    bno055_compute_accel_attitude(
        accel_ms2->x,
        accel_ms2->y,
        accel_ms2->z,
        &roll_acc_deg,
        &pitch_acc_deg);

    if (!state->attitude_initialized)
    {
        state->roll_est_deg = roll_acc_deg;
        state->pitch_est_deg = pitch_acc_deg;
        state->attitude_initialized = true;

        *roll_deg = state->roll_est_deg;
        *pitch_deg = state->pitch_est_deg;
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* Predicción mediante giróscopo                                          */
    /* ---------------------------------------------------------------------- */

    const float pitch_gyro_deg = state->pitch_est_deg + BNO055_PITCH_GYRO_SIGN * gyro_x_dps * dt_s;

    const float roll_gyro_deg = state->roll_est_deg + BNO055_ROLL_GYRO_SIGN * gyro_y_dps * dt_s;

    /* ---------------------------------------------------------------------- */
    /* Calidad instantánea de la referencia del acelerómetro                  */
    /* ---------------------------------------------------------------------- */

    const float accel_norm =
        sqrtf(
            accel_ms2->x * accel_ms2->x +
            accel_ms2->y * accel_ms2->y +
            accel_ms2->z * accel_ms2->z);

    /*
     * Error relativo respecto a 1 g.
     *
     *     e = ||a| - g| / g
     *
     * e = 0      -> módulo exactamente igual a g
     * e grande   -> probable aceleración dinámica
     */
    float accel_error = fabsf(accel_norm - STANDARD_GRAVITY_MS2) / STANDARD_GRAVITY_MS2;

    if (!isfinite(accel_error))
        accel_error = 1.0f;

    /*
     * Peso de confianza del acelerómetro.
     *
     *     confidence = 1 / (1 + K*e²)
     *
     * confidence ~= 1 : acelerómetro fiable
     * confidence ~= 0 : acelerómetro contaminado por dinámica
     */
    const float k = 100.0f;

    float accel_confidence = 1.0f / (1.0f + k * accel_error * accel_error);

    if (accel_confidence < 0.0f)
        accel_confidence = 0.0f;
    else if (accel_confidence > 1.0f)
        accel_confidence = 1.0f;

    /* ---------------------------------------------------------------------- */
    /* Peso base del filtro complementario                                    */
    /* ---------------------------------------------------------------------- */

    float alpha_base = BNO055_ATTITUDE_TAU_S / (BNO055_ATTITUDE_TAU_S + dt_s);

    if (!isfinite(alpha_base))
        alpha_base = 0.98f;

    if (alpha_base < 0.0f)
        alpha_base = 0.0f;
    else if (alpha_base > 1.0f)
        alpha_base = 1.0f;

    /*
     * Cuando accel_confidence = 1:
     *
     *     alpha = alpha_base
     *
     * Cuando accel_confidence -> 0:
     *
     *     alpha -> 1
     *
     * es decir, progresivamente dejamos de corregir con el acelerómetro.
     */
    const float alpha = 1.0f - accel_confidence * (1.0f - alpha_base);

    /* ---------------------------------------------------------------------- */
    /* Fusión                                                                 */
    /* ---------------------------------------------------------------------- */

    state->pitch_est_deg = alpha * pitch_gyro_deg + (1.0f - alpha) * pitch_acc_deg;

    state->roll_est_deg = alpha * roll_gyro_deg + (1.0f - alpha) * roll_acc_deg;

    *pitch_deg = state->pitch_est_deg;
    *roll_deg = state->roll_est_deg;
}

/* -------------------------------------------------------------------------- */
/* FILTRO PASO BAJO DE LAS TRES COMPONENTES DEL GIRÓSCOPO                     */
/* -------------------------------------------------------------------------- */

/*
 * Aplica un filtro paso bajo (LPF) de primer orden, de forma independiente,
 * a las tres componentes del vector de velocidad angular medido por el BNO055:
 *
 *      gyro_dps = [wx  wy  wz]
 *
 * donde las tres componentes están expresadas en [deg/s].
 *
 * El objetivo es reducir el ruido y las pequeñas oscilaciones de alta
 * frecuencia antes de utilizar el vector de velocidad angular en otros
 * cálculos, principalmente en la estimación del régimen de giro alrededor
 * de la vertical local.
 *
 * Para cada componente del vector se aplica:
 *
 *      y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 *
 * donde:
 *
 *      x[k]   = nueva medida del giróscopo
 *      y[k]   = nueva salida filtrada
 *      y[k-1] = salida filtrada de la muestra anterior
 *
 * El coeficiente alpha se calcula a partir del periodo de muestreo y de la
 * constante de tiempo del filtro:
 *
 *      alpha = 1 - exp(-dt/tau)
 *
 * siendo:
 *
 *      dt  = periodo entre muestras [s]
 *      tau = GYRO_FILTER_TAU_S [s]
 *
 * Una tau mayor produce mayor filtrado, pero también una respuesta más lenta.
 * Una tau menor produce una respuesta más rápida, pero deja pasar más ruido.
 *
 * El estado interno del filtro se conserva entre llamadas dentro de
 * bno055_processing_state_t.
 *
 * La primera muestra válida se copia directamente al estado del filtro para
 * evitar el transitorio que aparecería si el filtro comenzase desde cero.
 *
 * Si alguna de las tres componentes recibidas no es finita (NaN o infinito),
 * el estado del filtro no se actualiza y se devuelve la última salida válida.
 */

static void bno055_filter_gyro_dps(
    bno055_processing_state_t *state,
    const bno055_vector3f_t *gyro_dps,
    float dt_s,
    bno055_vector3f_t *gyro_filtered_dps)
{
    /*
     * Comprobación defensiva de los punteros de entrada y salida.
     */
    if ((state == NULL) || (gyro_dps == NULL) || (gyro_filtered_dps == NULL))
        return;

    /*
     * Comprobamos que las tres componentes de la medida sean válidas.
     *
     * Si alguna contiene NaN o infinito, no actualizamos el filtro y
     * devolvemos la última medida filtrada disponible.
     */
    if (!isfinite(gyro_dps->x) ||
        !isfinite(gyro_dps->y) ||
        !isfinite(gyro_dps->z))
    {
        gyro_filtered_dps->x = state->gyro_x_filtered_dps;
        gyro_filtered_dps->y = state->gyro_y_filtered_dps;
        gyro_filtered_dps->z = state->gyro_z_filtered_dps;
        return;
    }

    /*
     * Primera muestra válida.
     *
     * Inicializamos directamente el estado del filtro con la medida actual.
     * De esta forma evitamos un transitorio artificial desde cero hasta el
     * valor real del giróscopo.
     */
    if (!state->gyro_filter_initialized)
    {
        state->gyro_x_filtered_dps = gyro_dps->x;
        state->gyro_y_filtered_dps = gyro_dps->y;
        state->gyro_z_filtered_dps = gyro_dps->z;

        state->gyro_filter_initialized = true;
    }
    else
    {
        /*
         * Calculamos el coeficiente del LPF de primer orden:
         *
         *      alpha = 1 - exp(-dt/tau)
         *
         * El uso de dt_s en el cálculo permite mantener aproximadamente la
         * misma constante de tiempo aunque cambie el periodo de muestreo.
         */
        float alpha = 1.0f - expf(-dt_s / GYRO_FILTER_TAU_S);

        /*
         * Protección numérica.
         *
         * En condiciones normales alpha debe estar siempre comprendido
         * entre 0 y 1.
         */
        if (!isfinite(alpha) || (alpha < 0.0f))
            alpha = 0.0f;
        else if (alpha > 1.0f)
            alpha = 1.0f;

        /*
         * Aplicamos independientemente el mismo LPF a las tres componentes:
         *
         *      wx_f[k] = wx_f[k-1] + alpha * (wx[k] - wx_f[k-1])
         *      wy_f[k] = wy_f[k-1] + alpha * (wy[k] - wy_f[k-1])
         *      wz_f[k] = wz_f[k-1] + alpha * (wz[k] - wz_f[k-1])
         */
        state->gyro_x_filtered_dps += alpha * (gyro_dps->x - state->gyro_x_filtered_dps);

        state->gyro_y_filtered_dps += alpha * (gyro_dps->y - state->gyro_y_filtered_dps);

        state->gyro_z_filtered_dps += alpha * (gyro_dps->z - state->gyro_z_filtered_dps);
    }

    /*
     * Copiamos el estado actualizado del filtro al vector de salida.
     */
    gyro_filtered_dps->x = state->gyro_x_filtered_dps;
    gyro_filtered_dps->y = state->gyro_y_filtered_dps;
    gyro_filtered_dps->z = state->gyro_z_filtered_dps;
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
/* CONVERSIÓN DE CUATERNIÓN A ÁNGULOS DE EULER                               */
/* -------------------------------------------------------------------------- */

/*
 * Convierte el cuaternión unitario proporcionado por la fusión interna del
 * BNO055 a heading, roll y pitch.
 *
 * Se utiliza en NDOF en lugar de leer los registros Euler del sensor. De esta
 * forma el horizonte utiliza directamente la representación de actitud
 * fundamental entregada por el algoritmo de fusión y se elimina una
 * transacción I2C por muestra.
 */
static void bno055_quaternion_to_euler_deg(
    const bno055_quaternionf_t *q,
    float *heading_deg,
    float *roll_deg,
    float *pitch_deg)
{
    if ((q == NULL) || (heading_deg == NULL) || (roll_deg == NULL) || (pitch_deg == NULL))
        return;

    const float w = q->w;
    const float x = q->x;
    const float y = q->y;
    const float z = q->z;

    const float norm2 = w * w + x * x + y * y + z * z;

    if (!isfinite(norm2) || (norm2 < 1.0e-8f))
    {
        *heading_deg = 0.0f;
        *roll_deg = 0.0f;
        *pitch_deg = 0.0f;
        return;
    }

    /*
     * Normalizamos defensivamente. El BNO055 entrega normalmente un cuaternión
     * prácticamente unitario, pero esta normalización evita que pequeños
     * errores numéricos afecten a asin().
     */
    const float inv_norm = 1.0f / sqrtf(norm2);

    const float q0 = w * inv_norm;
    const float q1 = x * inv_norm;
    const float q2 = y * inv_norm;
    const float q3 = z * inv_norm;

    const float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);

    const float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    float sinp = 2.0f * (q0 * q2 - q3 * q1);

    if (sinp > 1.0f)
        sinp = 1.0f;
    else if (sinp < -1.0f)
        sinp = -1.0f;

    const float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);

    const float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    *pitch_deg = asinf(sinp) * RAD_TO_DEG;

    float heading = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    if (heading < 0.0f)
        heading += 360.0f;

    *heading_deg = heading;
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

void bno055_processing_reset_g_peaks(bno055_processing_state_t *state, float g_current)
{
    if ((state == NULL) || !isfinite(g_current))
        return;

    state->g_min = g_current;
    state->g_max = g_current;
    state->g_peaks_initialized = true;
}

/* -------------------------------------------------------------------------- */

static void bno055_process_acceleration(
    bno055_data_t *data)
{
    data->accel_x_g = data->acceleration_ms2.x / STANDARD_GRAVITY_MS2;

    data->accel_y_g = data->acceleration_ms2.y / STANDARD_GRAVITY_MS2;

    data->accel_z_g = data->acceleration_ms2.z / STANDARD_GRAVITY_MS2;

    data->accel_total_g = sqrtf(data->accel_x_g * data->accel_x_g +
                                data->accel_y_g * data->accel_y_g +
                                data->accel_z_g * data->accel_z_g);
}

/* -------------------------------------------------------------------------- */

static void bno055_process_g_meter(
    bno055_processing_state_t *state,
    bno055_data_t *data)
{
    data->g_current = data->accel_total_g;

    if (!state->g_peaks_initialized)
    {
        bno055_processing_reset_g_peaks(
            state,
            data->g_current);
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

/* -------------------------------------------------------------------------- */

static void bno055_process_slip_ball(bno055_processing_state_t *state, bno055_data_t *data, float dt_s)
{
    data->slip_ball_deg = SLIP_BALL_GAIN * bno055_compute_slip_ball_deg(state, data->accel_x_g, data->roll_deg, dt_s);
}

/* -------------------------------------------------------------------------- */

void bno055_process_sample_amg(bno055_processing_state_t *state, bno055_data_t *data, float dt_s)
{
    if ((state == NULL) || (data == NULL))
        return;

    /* ---------------------------------------------------------------------- */
    /* Aceleración                                                            */
    /* ---------------------------------------------------------------------- */

    bno055_process_acceleration(data);

    /* ---------------------------------------------------------------------- */
    /* Actitud AMG: acelerómetro + giróscopo                                  */
    /* ---------------------------------------------------------------------- */

    bno055_update_complementary_attitude(
        state,
        &data->acceleration_ms2,
        data->gyro_dps.x,
        data->gyro_dps.y,
        dt_s,
        &data->roll_deg,
        &data->pitch_deg);

    /*
     * Alternativa experimental:
     *
     * bno055_update_adaptive_attitude(
     *     state,
     *     &data->acceleration_ms2,
     *     data->gyro_dps.x,
     *     data->gyro_dps.y,
     *     dt_s,
     *     &data->roll_deg,
     *     &data->pitch_deg);
     */

    /* ---------------------------------------------------------------------- */
    /* Régimen de giro AMG                                                    */
    /* ---------------------------------------------------------------------- */

    bno055_vector3f_t gyro_filtered_dps = {0};

    bno055_filter_gyro_dps(
        state,
        &data->gyro_dps,
        dt_s,
        &gyro_filtered_dps);

    const float yaw_rate_raw_dps =
        bno055_compute_turn_rate_from_attitude_dps(
            gyro_filtered_dps.x,
            gyro_filtered_dps.y,
            gyro_filtered_dps.z,
            data->roll_deg,
            data->pitch_deg);

    data->yaw_rate_dps =
        bno055_filter_yaw_rate_dps(
            state,
            yaw_rate_raw_dps,
            dt_s);

    /* ---------------------------------------------------------------------- */
    /* Heading AMG                                                            */
    /* ---------------------------------------------------------------------- */

#if !HEADING_GPS
    data->heading_deg =
        bno055_integrate_yaw_deg(
            state,
            data->yaw_rate_dps,
            dt_s);
#endif

    /* ---------------------------------------------------------------------- */
    /* Bola y G-meter                                                         */
    /* ---------------------------------------------------------------------- */

    bno055_process_slip_ball(state, data, dt_s);

    bno055_process_g_meter(state, data);
}

/* -------------------------------------------------------------------------- */

void bno055_process_sample_ndof(bno055_processing_state_t *state, bno055_data_t *data, float dt_s)
{
    if ((state == NULL) || (data == NULL))
        return;

    /*
     * En NDOF el cuaternión, gravity y linear_acceleration proceden de la
     * fusión interna del BNO055.
     *
     * La actitud Euler utilizada por el EFIS se obtiene localmente a partir
     * del cuaternión; no se leen los registros Euler del sensor.
     */
    /*
        bno055_quaternion_to_euler_deg(
            &data->quaternion,
            &data->heading_deg,
            &data->roll_deg,
            &data->pitch_deg);
    */

    float gx = data->gravity_ms2.x;
    float gy = data->gravity_ms2.y;
    float gz = data->gravity_ms2.z;

    bno055_compute_accel_attitude(gx, gy, gz, &data->roll_deg, &data->pitch_deg);

    /* ---------------------------------------------------------------------- */
    /* Aceleración                                                            */
    /* ---------------------------------------------------------------------- */

    bno055_process_acceleration(data);

    /* ---------------------------------------------------------------------- */
    /* Régimen de giro NDOF                                                   */
    /* ---------------------------------------------------------------------- */

    const float yaw_rate_raw_dps =
        bno055_compute_vertical_turn_rate_dps(
            data->gyro_dps.x,
            data->gyro_dps.y,
            data->gyro_dps.z,
            &data->gravity_ms2);

    data->yaw_rate_dps = bno055_filter_yaw_rate_dps(state, yaw_rate_raw_dps, dt_s);

    /* ---------------------------------------------------------------------- */
    /* Bola y G-meter                                                         */
    /* ---------------------------------------------------------------------- */

    bno055_process_slip_ball(state, data, dt_s);

    bno055_process_g_meter(state, data);
}
