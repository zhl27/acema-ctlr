#ifndef KALMAN1D_H
#define KALMAN1D_H

class Kalman1D {
private:
    float angle_rad; // Ángulo filtrado
    float P;     // Incertidumbre actual

    float var_gyro_rad_s;  // Varianza base del giroscopio
    float var_accel_g; // Varianza base del acelerómetro (estático)

public:
    Kalman1D(float initial_var_gyro, float initial_var_accel_rad);

    // Actualizamos la firma para recibir la aceleración en Z
    float update(float gyro_rate, float accel_angle_rad, float dt, float accel_z_g);
    
    float getAngleRad() const { return angle_rad; }
};

#endif