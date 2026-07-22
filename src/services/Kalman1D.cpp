#include "Kalman1D.h"
#include <cmath>

Kalman1D::Kalman1D(float initial_var_gyro, float initial_var_accel_rad) {
    angle_rad = 0.0f;
    P = 2.0f;
    var_gyro_rad_s = initial_var_gyro;
    var_accel_angle = initial_var_accel_rad;
    var_dinamica_accel_angle = false; 
}

float Kalman1D::update(float gyro_rate, float accel_angle_rad, float dt, float accel_z_m_s2) {
    // --- 1. PREDICCIÓN ---
    float angle_pred = angle_rad + (gyro_rate * dt);
    float P_pred = P + (var_gyro_rad_s * dt * dt);

    // --- 2. LÓGICA DINÁMICA DE COHETERÍA ---
    float current_var_accel = var_accel_angle;
    
    // Convertimos m/s^2 a unidades G para validar contra la ventana estática
    // 9.80665 m/s^2 pasará a ser 1.0G. El valor absoluto cubre si el sensor está invertido.
    float accel_z_g = std::abs(accel_z_m_s2) / 9.80665f;

    // Si la aceleración Z sale de la ventana de reposo (0.8G a 1.2G), significa que el 
    // motor encendió o estamos en caída libre. Elevamos la varianza para confiar solo en el gyro.
    if (var_dinamica_accel_angle == true && (accel_z_g > 1.2f || accel_z_g < 0.8f)) {
        current_var_accel = 1000.0f; 
    }

    // --- 3. CORRECCIÓN ---
    float K = P_pred / (P_pred + current_var_accel);
    angle_rad = angle_pred + K * (accel_angle_rad - angle_pred);
    P = (1.0f - K) * P_pred;

    return angle_rad;
}