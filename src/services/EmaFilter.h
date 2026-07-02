#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#include "DataFilter.h"
#include <cmath>

/**
 * @
 */
class EmaFilter: public IFilter {
public:
    /**
     * @brief Constructor por default
     */
    EmaFilter();

    /**
     * @brief Contructor parametrizado por alfa
     */
    EmaFilter(float alfa);

    /**
     * @brief Contructor parametrizado por frecuencias, calcula alfa según las frecuencias
     */
    EmaFilter(float FS, float fc);

    void inicializar(float muestraInicial = 0) override;

    /* procesa la muestra */
    float actualizar(float) override; 

    /* Reseteo */
    void resetear() override;

    /**
     * @details valor entre 0.0 (mucho filtrado) y 1.0 (Bajo filtrado)
     */
    bool setAlfa(float);

    /**
     * @details setea alfa de acuerdo a la frecuencia de muestreo y corte 
     */
    bool setFrecuenciaCorte(float FS, float fc);


private:
    double _alfa;
    double _salidaPrev;
    bool _estaInicializado;
};
#endif