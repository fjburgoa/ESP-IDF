#include "Hybrid_pitch_roll.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f
#define GRAVITY_MS2 9.81f

/* Weighting based on ||a|-g| */
#define ACCEL_ERROR_LOW_MS2   0.20f
#define ACCEL_ERROR_HIGH_MS2  1.50f

/* Sanity check for BNO055 gravity in IMUPLUS */
#define GRAVITY_VALID_MIN_MS2  8.0f
#define GRAVITY_VALID_MAX_MS2 11.5f

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

hybrid_pitch_roll_data_t Hybrid_pitch_roll_update(
    float ax, float ay, float az,
    float gx, float gy, float gz)
{
    hybrid_pitch_roll_data_t out = {0};

    const float accel_norm =
        sqrtf(ax*ax + ay*ay + az*az);

    const float gravity_norm =
        sqrtf(gx*gx + gy*gy + gz*gz);

    const float accel_error =
        fabsf(accel_norm - GRAVITY_MS2);

    float alpha;

    if (accel_error <= ACCEL_ERROR_LOW_MS2)
    {
        alpha = 1.0f;
    }
    else if (accel_error >= ACCEL_ERROR_HIGH_MS2)
    {
        alpha = 0.0f;
    }
    else
    {
        alpha =
            (ACCEL_ERROR_HIGH_MS2 - accel_error) /
            (ACCEL_ERROR_HIGH_MS2 - ACCEL_ERROR_LOW_MS2);

        alpha = clampf_local(alpha, 0.0f, 1.0f);
    }

    const bool gravity_valid =
        (gravity_norm >= GRAVITY_VALID_MIN_MS2) &&
        (gravity_norm <= GRAVITY_VALID_MAX_MS2);

    /* In AMG, gravity is zero: fall back to MPU6050 */
    if (!gravity_valid)
    {
        alpha = 1.0f;
    }

    const float hx = alpha*ax + (1.0f-alpha)*gx;
    const float hy = alpha*ay + (1.0f-alpha)*gy;
    const float hz = alpha*az + (1.0f-alpha)*gz;

    const float hybrid_norm =
        sqrtf(hx*hx + hy*hy + hz*hz);

    if (hybrid_norm < 0.1f)
    {
        return out;
    }

    /* Same convention already used in the project */
    out.pitch_deg =
        atan2f(hy, sqrtf(hx*hx + hz*hz)) * RAD_TO_DEG;

    out.roll_deg =
        atan2f(hx, sqrtf(hy*hy + hz*hz)) * RAD_TO_DEG;

    out.accel_weight = alpha;
    out.accel_norm_ms2 = accel_norm;
    out.gravity_norm_ms2 = gravity_norm;
    out.accel_error_ms2 = accel_error;
    out.valid = true;

    return out;
}
