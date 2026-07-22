#ifndef KALMAN_2D_H
#define KALMAN_2D_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

class Kalman2D {
private:
    // El estado del sistema consiste en la altitud y la velocidad vertical
    Matrix<2, 1> S; 
    
    // Matriz de incertidumbre (covarianza)
    Matrix<2, 2> P; 
    
    // Matrices del sistema
    Matrix<2, 2> A; // Matriz de transición de estado 
    Matrix<2, 1> B; // Matriz de control 
    Matrix<1, 2> H; // Matriz de observación 
    
    // Matrices de ruido
    Matrix<2, 2> Q; // Incertidumbre del proceso 
    Matrix<1, 1> R; // Incertidumbre de la medición 
    Matrix<2, 2> I; // Matriz Identidad 

    float var_accel; // Varianza del acelerómetro (ruido)
    float var_baro;  // Varianza del barómetro (ruido)

public:
    Kalman2D();
    
    // Inicializa el filtro con varianzas y estado inicial
    void init(float init_alt, float init_vel, float varianza_accel, float varianza_baro);
    
    // Actualiza el filtro con el dt dinámico, la aceleración Z inercial (sin gravedad) y la altitud barométrica
    void update(float dt, float accel_z_inertial, float baro_alt);
    
    // Getters para el vector de estado S
    float getAltitude() { return S(0, 0); }
    float getVelocity() { return S(1, 0); }
};

#endif // KALMAN_2D_H