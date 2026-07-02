//
// Created by zhl on 6/25/26.
//

#ifndef ACEMA_CTLR_DATAFILTER_H
#define ACEMA_CTLR_DATAFILTER_H
#include "data.h"
#include "EmaFilter.h"


// TODO: por ahora lo modelamos como un util (static), pero idealmente debemos hacerlo clase, para permitir flexibilidad y desacople. Queremos poder pasarle filtros (sea Filtro complementario, sea filtro de kalman, etc.), y crear una instancia DataFilter con distintas combinaciones de estas.
class DataFilter {
public:
    // Settear en el setup() ANTES de despegar
    static void init(float masa_cohete_kg = 15.0f, float altitud_cero_pad_m = 0.0f);

    // Función principal de transformación estática
    static data_all_t process(const data_raw_t& raw);

private:
    // Variables de configuración de entorno (se setean en init)
    static float _masa_cohete_kg;
    static float _altitud_cero_pad_m;
    static float _ultima_altura_m;
    static uint64_t _ultimo_tiempo_us;
    static bool _es_primer_ciclo;

    // Instancias de filtrado EMA para las señales crudas del MPU6050
    static EmaFilter filter_accel_x;
    static EmaFilter filter_accel_y;
    static EmaFilter filter_accel_z;
    static EmaFilter filter_gyro_x;
    static EmaFilter filter_gyro_y;
    static EmaFilter filter_gyro_z;

    // Instancias de filtrado EMA para las señales crudas del BMP280
    static EmaFilter filter_bmp_presion;
    static EmaFilter filter_bmp_temp;
};

#endif //ACEMA_CTLR_DATAFILTER_H
