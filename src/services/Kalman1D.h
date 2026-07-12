#ifndef KALMAN1D_H
#define KALMAN1D_H

class Kalman1D {
private:
    float angle; // Ángulo filtrado
    float P;     // Incertidumbre actual

    float var_gyro;  // Varianza base del giroscopio
    float var_accel; // Varianza base del acelerómetro (estático)

public:
    Kalman1D(float initial_var_gyro, float initial_var_accel);

    // Actualizamos la firma para recibir la aceleración en Z
    float update(float gyro_rate, float accel_angle, float dt, float accel_z_g);
    
    float getAngle() const { return angle; }
};

#endif