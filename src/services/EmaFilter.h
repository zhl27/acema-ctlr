#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#include <cmath>
/**
 * @
 */
class EmaFilter {
public:
    /**
     * @brief Constructor por default
     */
    EmaFilter();

    /**
     * @brief Contructor parametrizado por alfa
     */
    explicit EmaFilter(float alfa);

    /**
     * @brief Contructor parametrizado por frecuencias, calcula alfa según las frecuencias
     */
    EmaFilter(float FS, float fc);

    void inicializar(float muestraInicial = 0) ;

    /* procesa la muestra */
    float filtrar(float) ;

    /* Reseteo */
    void resetear() ;

    /**
     * @details valor entre 0.0 (mucho filtrado) y 1.0 (Bajo filtrado)
     */
    bool setAlfa(float);

    /**
     * @details setea alfa de acuerdo a la frecuencia de muestreo y corte 
     */
    bool setFrecuenciaCorte(float FS, float fc);

    ~EmaFilter() = default;
private:
    double _alfa;
    double _salidaPrev;
    bool _estaInicializado;
};
#endif