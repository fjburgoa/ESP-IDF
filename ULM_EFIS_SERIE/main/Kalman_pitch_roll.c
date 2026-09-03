#include "Kalman_pitch_roll.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG       57.29577951308232f

/* Initial tuning parameters */
#define KALMAN_Q_ANGLE   0.001f
#define KALMAN_Q_BIAS    0.003f
#define KALMAN_R_ANGLE   0.030f

typedef struct
{
    float angle_deg;
    float bias_dps;

    float P00;
    float P01;
    float P10;
    float P11;
} kalman_axis_t;

static kalman_axis_t s_pitch = {0};
static kalman_axis_t s_roll  = {0};
static bool s_initialized = false;


static float kalman_axis_update(kalman_axis_t *kf,
                                float angle_acc_deg,
                                float gyro_rate_dps,
                                float dt_s)
{
    /* State prediction:
       angle- = angle + dt * (gyro - bias)
       bias-  = bias
    */
    kf->angle_deg += dt_s * (gyro_rate_dps - kf->bias_dps);

    /* Covariance prediction: P- = F P F' + Q */
    const float P00 = kf->P00;
    const float P01 = kf->P01;
    const float P10 = kf->P10;
    const float P11 = kf->P11;

    kf->P00 = P00 - dt_s * (P10 + P01)
            + dt_s * dt_s * P11
            + KALMAN_Q_ANGLE;

    kf->P01 = P01 - dt_s * P11;
    kf->P10 = P10 - dt_s * P11;
    kf->P11 = P11 + KALMAN_Q_BIAS;

    /* Innovation */
    const float innovation = angle_acc_deg - kf->angle_deg;

    /* Innovation covariance */
    const float S = kf->P00 + KALMAN_R_ANGLE;

    /* Kalman gain */
    const float K0 = kf->P00 / S;
    const float K1 = kf->P10 / S;

    /* State correction */
    kf->angle_deg += K0 * innovation;
    kf->bias_dps  += K1 * innovation;

    /* Covariance correction: P = (I-KH)P- */
    const float P00_pred = kf->P00;
    const float P01_pred = kf->P01;

    kf->P00 -= K0 * P00_pred;
    kf->P01 -= K0 * P01_pred;
    kf->P10 -= K1 * P00_pred;
    kf->P11 -= K1 * P01_pred;

    return kf->angle_deg;
}


void Kalman_pitch_roll_init(void)
{
    memset(&s_pitch, 0, sizeof(s_pitch));
    memset(&s_roll, 0, sizeof(s_roll));
    s_initialized = false;
}


kalman_pitch_roll_data_t Kalman_pitch_roll_update(
    float ax, float ay, float az,
    float wx, float wy, float wz,
    float dt_s)
{
    kalman_pitch_roll_data_t out = {0};

    /* wz is not used by these two independent pitch/roll filters */
    (void)wz;

    if (dt_s <= 0.0f)
        return out;

    /*
     * Same accelerometer convention already used in the project:
     *
     * pitch = atan2(ay, sqrt(ax^2 + az^2))
     * roll  = atan2(ax, sqrt(ay^2 + az^2))
     */
    const float az2 = az * az;

    const float pitch_acc_deg =
        atan2f(ay, sqrtf(ax * ax + az2)) * RAD_TO_DEG;

    const float roll_acc_deg =
        atan2f(ax, sqrtf(ay * ay + az2)) * RAD_TO_DEG;

    /* First sample: initialize attitude from accelerometer */
    if (!s_initialized)
    {
        s_pitch.angle_deg = pitch_acc_deg;
        s_roll.angle_deg  = roll_acc_deg;
        s_initialized = true;
    }

    /*
     * Current assumed gyro mapping:
     *   wy -> pitch
     *   wx -> roll
     */
    const float pitch_deg =
        kalman_axis_update(&s_pitch, pitch_acc_deg, wy, dt_s);

    const float roll_deg =
        kalman_axis_update(&s_roll, roll_acc_deg, wx, dt_s);

    out.pitch_deg = pitch_deg;
    out.roll_deg = roll_deg;
    out.pitch_acc_deg = pitch_acc_deg;
    out.roll_acc_deg = roll_acc_deg;
    out.pitch_bias_dps = s_pitch.bias_dps;
    out.roll_bias_dps = s_roll.bias_dps;
    out.valid = true;

    return out;
}
