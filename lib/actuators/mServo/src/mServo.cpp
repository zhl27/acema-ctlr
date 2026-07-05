//
// Created by zhl on 6/4/26.
//

#include "mServo.h"

#include <math.h>

mServo::mServo(mServoConfig_t* cfg)
  : _cfg(cfg),
    angActual(0.0f),
    angTarget(0.0f),
    intervaloPaso(0),
    ultimoPasoMicros(0),
    estaMoviendo(false),
    tieneError(false),
    ID(0)
{

}

bool mServo::init() {

  if (_cfg == nullptr) return false;

  // Attach del servo
  _servo.attach(_cfg->pinServo);

  // Inicialización encapsulada: leer ángulo bruto y establecer estado
  const float raw = leerAnguloBruto();
  angActual = raw;
  angTarget = raw;

  ultimoPasoMicros = micros();
  estaMoviendo = false;
  tieneError = false;
  ID = _cfg->id;

  // Posicionar el servo a la posición inicial calculada
  const uint16_t pulse = static_cast<uint16_t>(round(_cfg->minPulse +
                                               (_cfg->maxPulse - _cfg->minPulse) * (angActual / 180.0f)));
  _servo.writeMicroseconds(pulse);

  return true;
}

float mServo::leerAnguloBruto() const {
  // Lectura del potenciómetro (0..4095 en ESP32 típicamente)
  float adcVal = analogRead(_cfg->pinPot);
  return _cfg->pendiente * adcVal;
}

float mServo::leerAngulo() {
  if (_cfg == nullptr) return angActual;

  // Lectura y filtro EMA
  float nuevoAngulo = leerAnguloBruto();
  angActual = (_cfg->alpha * nuevoAngulo) + ((1.0f - _cfg->alpha) * angActual);

  // Asegurar límites
  if (angActual < 0.0f) angActual = 0.0f;
  if (angActual > 180.0f) angActual = 180.0f;
  return angActual;
}

void mServo::setAngulo(float ang, uint16_t degPerSec) {
  if (_cfg == nullptr) return;

  // Evita cambios triviales y velocidades no razonables
  if (fabs(ang - angActual) < _cfg->deadband) return;
  if (degPerSec < 1) degPerSec = 1;

  angTarget = constrain(ang, 0.0f, 180.0f);
  intervaloPaso = 1000000UL / degPerSec; // µs por grado
}

bool mServo::enAnguloTarget() const {
  if (_cfg == nullptr) return true;
  return (fabs(angTarget - angActual) < _cfg->deadband);
}

void mServo::step(const bool autoActualizacion) {
  if (enAnguloTarget()) {
    estaMoviendo = false;
    return;
  }

  const uint32_t now = micros();
  if (now - ultimoPasoMicros >= intervaloPaso) {
    ultimoPasoMicros = now;
    estaMoviendo = true;

    // Dirección según la diferencia
    const float dir = (angTarget > angActual) ? 1.0f : -1.0f;
    angActual += dir * 1.0f; // un grado por paso (ajusta si necesitas otro step)

    if (angActual > 180.0f) angActual = 180.0f;
    if (angActual < 0.0f)   angActual = 0.0f;

    // Actualizar pulso del servo
    const uint16_t pulse = static_cast<uint16_t>(round(_cfg->minPulse +
                                                       (_cfg->maxPulse - _cfg->minPulse) * (angActual / 180.0f)));
    _servo.writeMicroseconds(pulse);

    if (autoActualizacion) {
      angActual = leerAngulo();
      const uint16_t pulse2 = static_cast<uint16_t>(round(_cfg->minPulse +
                                                    (_cfg->maxPulse - _cfg->minPulse) * (angActual / 180.0f)));
      _servo.writeMicroseconds(pulse2);
    }
  }
}

bool mServo::enAngulo(float pos) const {
  return fabs(pos - angActual) < _cfg->deadband;
}
