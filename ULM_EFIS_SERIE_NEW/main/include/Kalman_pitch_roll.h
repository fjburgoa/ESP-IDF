#ifndef KALMAN_PITCH_ROLL_H
#define KALMAN_PITCH_ROLL_H

#include <stdbool.h>

typedef struct {
    float pitch_deg;
    float roll_deg;
    float pitch_acc_deg;
    float roll_acc_deg;
    float pitch_bias_dps;
    float roll_bias_dps;
    bool valid;
} kalman_pitch_roll_data_t;

void Kalman_pitch_roll_init(void);

kalman_pitch_roll_data_t Kalman_pitch_roll_update(
    float ax, float ay, float az,
    float wx, float wy, float wz,
    float dt_s);

#endif
