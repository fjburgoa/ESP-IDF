#ifndef _PROCESADO_H
#define _PROCESADO_H

    // Estructura para almacenar los ángulos de Euler
    typedef struct {
        float roll;    // Ángulo de roll (phi)
        float pitch;   // Ángulo de pitch (theta)
        float yaw;     // Ángulo de yaw (psi)
    } EulerAngles;

    /*********************************************************/
    void calculate_euler_derivatives(float p, float q, float r, float phi, float theta, float *phi_dot, float *theta_dot, float *psi_dot);
    EulerAngles update_euler_angles(float p, float q, float r, float dT, EulerAngles previous_angles);

    float raw_to_angular_velocity_deg(int16_t raw_value); 
    float deg_to_rad(float angular_velocity_deg);

    float sign(float x);

    /*********************************************************/
    EulerAngles calcularT(float p, float q, float r, float dT);

    /*********************************************************/
    void initial_conditions_quaternions(float Q[4], EulerAngles A);
    EulerAngles calcularQ(float p, float q, float r, float Q[4], float dT);

#endif