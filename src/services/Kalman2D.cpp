#include "Kalman2D.h"

Kalman2D::Kalman2D() {
    S.Fill(0); // La altitud y velocidad vertical iniciales son iguales a cero
    P.Fill(0); // La matriz de incertidumbre P también se inicia en 0
    
    // La matriz de observación consiste solo en 1 y 0, 
    // porque el barómetro solo mide la altitud (posición 0 de S).
    H = {1, 0}; 
    
    // Matriz Identidad 2x2
    I = {1, 0, 
         0, 1}; 
}

void Kalman2D::init(float init_alt, float init_vel, float varianza_accel, float varianza_baro) {
    S(0, 0) = init_alt;
    S(1, 0) = init_vel;
    
    // La varianza es esencialmente el cuadrado de la desviación estándar 
    var_accel = varianza_accel; 
    var_baro = varianza_baro;
}

void Kalman2D::update(float dt, float accel_z_inertial, float baro_alt) {
    
    // 1. DEFINICIÓN DINÁMICA DE MATRICES (Dependientes de dt)
    // La matriz de transición de estado F (o A) se rellena con 1, dt, 0 y 1 
    A = {1, dt, 
         0, 1};
         
    // La matriz de control G (o B) 
    B = {0.5f * dt * dt, 
         dt};
         
    // Matriz Q: Varianza de la incertidumbre del proceso 
    Q = {B(0,0)*B(0,0)*var_accel, B(0,0)*B(1,0)*var_accel,
         B(1,0)*B(0,0)*var_accel, B(1,0)*B(1,0)*var_accel};

    R = {var_baro};

    // --- FASE 1: PREDICCIÓN ---
    // Predecir el estado actual del sistema usando la matriz de espacio de estado
    Matrix<2, 1> S_pred = A * S + B * accel_z_inertial;
    
    // Calcular la incertidumbre de la predicción
    Matrix<2, 2> P_pred = A * P * ~A + Q;

    // --- FASE 2: CORRECCIÓN ---
    // Calcula la transpuesta primero para evitar ambigüedades
    Matrix<2, 1> H_trans = ~H;

    // Define la matriz que contendrá el resultado de la inversión
    Matrix<1, 1> Inv;
    Matrix<1, 1> Inv_resultado;

    // Calculamos la parte interna de la ganancia
    Inv = H * P_pred * H_trans + R;

    // Inverte y verifica
    // La función Invert(Matriz_Original, Matriz_Destino) devuelve true si tuvo éxito
    if (Invert(Inv, Inv_resultado)) {
        // Si la inversión fue exitosa, calculamos K
        Matrix<2, 1> K = P_pred * H_trans * Inv_resultado;
        
        // Y procedemos a la actualización del estado S y la covarianza P
        Matrix<1, 1> Y = {baro_alt};
        S = S_pred + K * (Y - H * S_pred);
        P = (I - K * H) * P_pred;
    } else {
        // Manejo de error: Si la matriz no es invertible, no podemos corregir.
        // En un cohete, lo mejor es saltar la corrección y mantener la predicción.
        S = S_pred;
    }
}