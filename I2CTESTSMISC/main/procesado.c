#include <stdio.h>
#include <math.h>
#include "procesado.h"

/*****************************************************************************************************************/

// Función para calcular las derivadas de los ángulos de Euler
void calculate_euler_derivatives(float p, float q, float r, float phi, float theta, float *phi_dot, float *theta_dot, float *psi_dot) 
{
    // Matriz de transformación para las derivadas de los ángulos de Euler
    *phi_dot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
    *theta_dot = cos(phi) * q - sin(phi) * r;
    *psi_dot = (sin(phi) / cos(theta)) * q + (cos(phi) / cos(theta)) * r;
}

// Función principal para actualizar los ángulos de Euler
EulerAngles update_euler_angles(float p, float q, float r, float dT, EulerAngles previous_angles) 
{
    EulerAngles new_angles;
    float phi_dot, theta_dot, psi_dot;

    // Calcular las derivadas de los ángulos de Euler
    calculate_euler_derivatives(p, q, r, previous_angles.roll, previous_angles.pitch, &phi_dot, &theta_dot, &psi_dot);

    // Integrar para obtener los nuevos ángulos de Euler
    new_angles.roll  = previous_angles.roll + phi_dot * dT;
    new_angles.pitch = previous_angles.pitch + theta_dot * dT;
    new_angles.yaw   = previous_angles.yaw + psi_dot * dT;

    return new_angles;
}

float raw_to_angular_velocity_deg(int16_t raw_value) {
    float range = 1000.0; // Rango de ±250°/s
    float resolution = 32768.0; // Resolución de 16 bits (2^15)
    return (((float)raw_value * range) / resolution);
}

// Función para convertir de °/s a rad/s
float deg_to_rad(float angular_velocity_deg) {
    return angular_velocity_deg * (M_PI / 180.0);
}
//Funcion signo de x
float sign(float x)
{
    if(x > 0) 
        return 1.0;
    
    if(x < 0) 
        return -1.0;
    
    return 0.0;
}

/*****************************************************************************************************************/

// Inicialización de T(0)
float t11 = 1.0, t12 = 0.0, t13 = 0.0;
float t21 = 0.0, t22 = 1.0, t23 = 0.0;
float t31 = 0.0, t32 = 0.0, t33 = 1.0;

EulerAngles calcularT(float p, float q, float r, float dT) 
{
    EulerAngles new_angles2;
 
    // Variables para las derivadas
    float dt11, dt12, dt13, dt21, dt22, dt23, dt31, dt32, dt33 ;

    // Iteración en el tiempo
    
    // Calcular las derivadas
    dt11 = r * t21 - q * t31;         dt12 = r * t22 - q * t32;        dt13 = r * t23 - q * t33;
    dt21 = -r * t11 + p * t31;        dt22 = -r * t12 + p * t32;       dt23 = -r * t13 + p * t33;
    dt31 = q * t11 - p * t21;         dt32 = q * t12 - p * t22;        dt33 = q * t13 - p * t23;

    // Actualizar los elementos de T
    t11 += dt11 * dT;        t12 += dt12 * dT;        t13 += dt13 * dT;
    t21 += dt21 * dT;        t22 += dt22 * dT;        t23 += dt23 * dT;        
    t31 += dt31 * dT;        t32 += dt32 * dT;        t33 += dt33 * dT;

    // Calcular los elementos restantes usando ortogonalidad
    //t32 = -(t12 * t31 + t22 * t21) / t11; // Relación de ortogonalidad
    //t33 = sqrt(1 - t31 * t31 - t32 * t32); // Normalización

    new_angles2.pitch = asin(-t31);
    new_angles2.roll  = atan2(t32,t33);//acos(t33/cos(new_angles2.pitch))*sign(t23);
    new_angles2.yaw   = atan2(t21,t11);//acos(t11/cos(new_angles2.pitch))*sign(t12);
     
    printf("T = [%f, %f, %f;\n     %f, %f, %f;\n     %f, %f, %f] Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg \n\n",   t11, t12, t13, t21, t22, t23, t31, t32, t33,(180/M_PI)*new_angles2.roll, (180/M_PI)*new_angles2.pitch, (180/M_PI)*new_angles2.yaw);

    return new_angles2; 
  
};



/*****************************************************************************************************************/
void initial_conditions_quaternions(float Q[4], EulerAngles A)
{
       Q[0] = cos(A.yaw/2)*cos(A.pitch/2)*cos(A.roll/2) + sin(A.yaw/2)*sin(A.pitch/2)*sin(A.roll/2);
       Q[1] = cos(A.yaw/2)*cos(A.pitch/2)*sin(A.roll/2) - sin(A.yaw/2)*sin(A.pitch/2)*cos(A.roll/2);
       Q[2] = cos(A.yaw/2)*sin(A.pitch/2)*cos(A.roll/2) + sin(A.yaw/2)*cos(A.pitch/2)*sin(A.roll/2);
       Q[3] = sin(A.yaw/2)*cos(A.pitch/2)*cos(A.roll/2) - cos(A.yaw/2)*sin(A.pitch/2)*sin(A.roll/2);  

      // for (int k=0;k<4;k++)
      //      printf("Q[%d] = %f\n",k,Q[k]); 

}



EulerAngles calcularQ(float p, float q, float r, float Q[4], float dT) 
{
    EulerAngles new_angles3;
 
    // Variables para las derivadas
    float dtq[4] = {0};

    // Calcular las derivadas
    dtq[0] =  0*Q[0] -p*Q[1] -q*Q[2] -r*Q[3];
    dtq[1] =  p*Q[0] +0*Q[1] +r*Q[2] -q*Q[3];
    dtq[2] =  q*Q[0] -r*Q[1] +0*Q[2] +p*Q[3];
    dtq[3] =  r*Q[0] +q*Q[1] -p*Q[2] -0*Q[3];    

    // Actualizar los elementos de Q
    for (int k=0 ; k<4 ; k++)
        Q[k] += 0.5*dtq[k]*dT;


    float T[3][3] = {0};
    
    T[0][0] = Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3];
    T[0][1] = 2*(Q[1]*Q[2] +  Q[0]*Q[3]);
    T[0][2] = 2*(Q[1]*Q[3] -  Q[0]*Q[2]);

    T[1][0] = 2*(Q[1]*Q[2] -  Q[0]*Q[3]);
    T[1][1] = Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] - Q[3]*Q[3];
    T[1][2] = 2*(Q[2]*Q[3] +  Q[0]*Q[1]);

    T[2][0] = 2*(Q[1]*Q[3] +  Q[0]*Q[2]);
    T[2][1] = 2*(Q[2]*Q[3] -  Q[0]*Q[1]);
    T[2][2] = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];


    new_angles3.pitch = asin(-T[2][0]);
    new_angles3.roll  = atan2(T[2][1],T[2][2]); //acos(t33/cos(new_angles2.pitch))*sign(t23);
    new_angles3.yaw   = atan2(T[1][0],T[0][0]); //acos(t11/cos(new_angles2.pitch))*sign(t12);
     
    //printf("T = [%f, %f, %f;\n     %f, %f, %f;\n     %f, %f, %f] Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg \n\n",   t11, t12, t13, t21, t22, t23, t31, t32, t33,(180/M_PI)*new_angles2.roll, (180/M_PI)*new_angles2.pitch, (180/M_PI)*new_angles2.yaw);

    return new_angles3; 
  
};


