#include "EmaFilter.h"

EmaFilter::EmaFilter()
{
    _alfa = 1.0;
    _salidaPrev = 0.0;
    _estaInicializado = false;
}

EmaFilter::EmaFilter(float alfa)
{
    setAlfa(alfa);
    _salidaPrev = 0.0;
    _estaInicializado = false; 
}

EmaFilter::EmaFilter(float FS, float fc)
{
    setFrecuenciaCorte(FS, fc);
    _salidaPrev = 0.0;
    _estaInicializado = false;
}

void EmaFilter::inicializar(float muestraInicial)
{
    _salidaPrev = muestraInicial;
    _estaInicializado = true;
}

float EmaFilter::actualizar(float muestra)
{
// Corrección de transitorio inicial (para evitar arrancar desde 0 si la señal empieza alta)
    if (!_estaInicializado) {
        _salidaPrev = muestra;
        muestra = true;
        return _salidaPrev;
    }

    // Ecuación en diferencias del EMA
    double salidaActual = _alfa * muestra + (1.0 - _alfa) * _salidaPrev;
    _salidaPrev = salidaActual;
    
    return salidaActual;
}

void EmaFilter::resetear()
{
    _estaInicializado = false;
    _salidaPrev = 0.0;
}

bool EmaFilter::setAlfa(float nuevoAlfa)
{
    if (nuevoAlfa > 0.0 && nuevoAlfa <= 1.0) {
        _alfa = nuevoAlfa;
        return true;
    }
    return false;
}

bool EmaFilter::setFrecuenciaCorte(float FS, float fc)
{
    if (fc <= 0.0 || FS <= 0.0) return false;
        
    // Ecuación de correspondencia temporal: alfaa = dt / (tau + dt)
    double dt = 1.0 / FS;
    double tau = 1.0 / (2.0 * M_PI * fc);
    _alfa = dt / (tau + dt);
    return true;
}
