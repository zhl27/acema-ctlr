#ifndef KALMAN1D_H
#define KALMAN1D_H

class Kalman1D {
private:
    float angle_rad{0.0f};  // Ángulo filtrado
    float P{1.0f};          // Incertidumbre de la estimación

    float var_gyro_rad_s;          // Varianza del proceso (ruido del giroscopio)
    float var_accel_angle;         // Varianza base de la medición del acelerómetro (rad^2)
    bool var_dinamica_accel_angle{true}; // Flag para ajuste dinámico de R durante el empuje

public:
    Kalman1D(float initial_var_gyro, float initial_var_accel_rad);

    /**
     * @brief Actualiza la estimación del ángulo.
     * @param gyro_rate Velocidad angular actual (rad/s).
     * @param accel_angle_rad Ángulo medido por acelerómetro (rad).
     * @param dt Delta de tiempo entre iteraciones (s).
     * @param accel_z_m_s2 Aceleración bruta en el eje Z (m/s^2).
     * @return float Ángulo estimado y filtrado en radianes.
     */
    float update(float gyro_rate, float accel_angle_rad, float dt, float accel_z_m_s2);
    
    [[nodiscard]] float getAngleRad() const { return angle_rad; }

    inline void habilitarVarianzaDinamica() { var_dinamica_accel_angle = true; }
    inline void deshabilitarVarianzaDinamica() { var_dinamica_accel_angle = false; }
    [[nodiscard]] inline bool esVarianzaDinamicaHabilitada() const { return var_dinamica_accel_angle; }
};

#endif // KALMAN1D_H