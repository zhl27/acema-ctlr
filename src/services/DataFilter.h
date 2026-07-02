//
// Created by zhl on 6/25/26.
//

#ifndef ACEMA_CTLR_DATAFILTER_H
#define ACEMA_CTLR_DATAFILTER_H
#include "data.h"

class IFilter {

    /*valor de inicio */
    void virtual inicializar(float) = 0;

    /* Actualización */
    float virtual actualizar(float) = 0; 

    /* Reseteo */
    void virtual resetear() = 0;
};



// TODO: por ahora lo modelamos como un util (static), pero idealmente debemos hacerlo clase, para permitir flexibilidad y desacople. Queremos poder pasarle filtros (sea Filtro complementario, sea filtro de kalman, etc.), y crear una instancia DataFilter con distintas combinaciones de estas.
class DataFilter {
public:
    // Settear en el setup() ANTES de despegar
    static void init(float masa_cohete_kg = 15.0f, float altitud_cero_pad_m = 0.0f);

    // Función principal de transformación estática
    static data_all_t process(const data_raw_t& raw);

private:
    // Ganancias cinemáticas Alpha-Beta (Ajustadas para ráfagas de 150Hz)
    static constexpr float ALPHA_Z = 0.15f;  // Peso de la medición del barómetro
    static constexpr float BETA_Z  = 0.005f; // Peso de la corrección de velocidad

    // Ganancia del Filtro Complementario MPU6050
    static constexpr float ALPHA_COMP = 0.98f;

    // Límite sónico amateur para corte de picos de presión espurios
    static constexpr float MAX_VELOCIDAD_FISICA_M_S = 343.0f; // Mach 1

    // Memoria estática interna (Variables de estado del ciclo k-1)
    static float _h_est;
    static float _v_est;
    static float _pitch;
    static float _roll;
    static uint64_t _t_prev_us;
    static float _masa_kg;
    static float _h_pad_offset;
    static bool _iniciado;
};

#endif //ACEMA_CTLR_DATAFILTER_H
