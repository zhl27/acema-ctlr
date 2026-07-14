#include "EmaFilter.h"
#include <cmath>

EmaFilter::EmaFilter()
{
    _alfa = 1.0f;
    _salidaPrev = 0.0f;
    _estaInicializado = false;
    _fc = 1.0f;
}

EmaFilter::EmaFilter(float alfa)
{
    setAlfa(alfa);
    _salidaPrev = 0.0f;
    _estaInicializado = false;
    _fc = 1.0f;
}

EmaFilter::EmaFilter(float FS, float fc)
{
    configurarFrecuenciaCorte(FS, fc);
    _salidaPrev = 0.0f;
    _estaInicializado = false;
}

void EmaFilter::inicializar(float muestraInicial)
{
    _salidaPrev = muestraInicial;
    _estaInicializado = true;
}

float EmaFilter::actualizar(float muestra)
{
    // Primera muestra: evita el transitorio inicial arrancando
    // desde el valor medido y no desde cero.
    if (!_estaInicializado)
    {
        _salidaPrev = muestra;
        _estaInicializado = true;
        return _salidaPrev;
    }

    // Filtro EMA:
    // y[k] = α·x[k] + (1-α)·y[k-1]
    _salidaPrev =
        _alfa * muestra +
        (1.0f - _alfa) * _salidaPrev;

    return _salidaPrev;
}

float EmaFilter::actualizar(float muestra, float dt)
{
    if (!_estaInicializado)
    {
        _salidaPrev = muestra;
        _estaInicializado = true;
        return _salidaPrev;
    }

    if (dt <= 0.0f) return _salidaPrev;

    const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * _fc);
    float alpha = dt / (tau + dt);
    if (alpha > 1.0f) alpha = 1.0f;

    _salidaPrev = alpha * muestra + (1.0f - alpha) * _salidaPrev;

    return _salidaPrev;
}

void EmaFilter::resetear()
{
    _estaInicializado = false;
    _salidaPrev = 0.0f;
}

bool EmaFilter::setAlfa(float nuevoAlfa)
{
    if (nuevoAlfa <= 0.0f || nuevoAlfa > 1.0f)
        return false;

    _alfa = nuevoAlfa;
    return true;
}

bool EmaFilter::configurarFrecuenciaCorte(float FS, float fc)
{
    if (FS <= 0.0f || fc <= 0.0f)
        return false;

    _fc = fc;
    // α = dt / (τ + dt)
    // τ = 1 / (2πfc)
    const float dt  = 1.0f / FS;
    const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * fc);

    _alfa = dt / (tau + dt);

    return true;
}
