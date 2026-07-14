#include "Kalman1D.h"
#include <cmath>

Kalman1D::Kalman1D(float initial_var_gyro, float initial_var_accel_rad) {
    angle_rad = 0.0f;
    P = 2.0f;
    var_gyro_rad_s = initial_var_gyro;
    var_accel_g = initial_var_accel_rad;
}

float Kalman1D::update(float gyro_rate, float accel_angle_rad, float dt, float accel_z_g) {
    // --- 1. PREDICCIÓN ---
    float angle_pred = angle_rad + (gyro_rate * dt);
    float P_pred = P + (var_gyro_rad_s * dt * dt);

    // --- 2. LÓGICA DINÁMICA DE COHETERÍA ---
    // Si la aceleración en Z está fuera de la ventana de reposo (0.8G a 1.2G),
    // el cohete está acelerando o en caída libre. Elevamos la varianza al infinito (1000.0).
    float current_var_accel = var_accel_g;
    
    // Asumimos que accel_z_g viene en múltiplos de G (ej. 1.0 = 9.81 m/s^2)
    // El valor absoluto cubre si el sensor está invertido.
    if (std::abs(accel_z_g) > 1.2f || std::abs(accel_z_g) < 0.8f) {
        current_var_accel = 1000.0f; 
    }

    // --- 3. CORRECCIÓN ---
    float K = P_pred / (P_pred + current_var_accel);
    angle_rad = angle_pred + K * (accel_angle_rad - angle_pred);
    P = (1.0f - K) * P_pred;

    return angle_rad;
}