#include "mServo.h"
#include <cmath>

mServo::mServo(mServoConfig_t* cfg)
  : _cfg(cfg),
    angActual(0.0f),
    ID(0)
{
}

bool mServo::init() {
  if (_cfg == nullptr) return false;

  ID = _cfg->id;

  // Asignamos el pin al periférico PWM del ESP32 especificando los anchos de pulso
  _servo.attach(_cfg->pinServo, _cfg->minPulse, _cfg->maxPulse);

  // Inicializamos la posición en 0 grados a máxima velocidad
  angActual = 0.0f;
  setAngulo(angActual);

  return true;
}

void mServo::setAngulo(const float ang) {
  if (_cfg == nullptr) return;

  // Restringimos el valor entre los límites de operación del servo (0° a 180°)
  angActual = constrain(ang, 0.0f, 180.0f);

  // Mapeo directo del ángulo al ancho de pulso en microsegundos
  const uint16_t pulse = static_cast<uint16_t>(round(_cfg->minPulse + 
                         (_cfg->maxPulse - _cfg->minPulse) * (angActual / 180.0f)));
  
  // Enviamos la orden inmediatamente al hardware para movimiento a máxima velocidad
  _servo.writeMicroseconds(pulse);
}

float mServo::getAngulo() const {
  return angActual;
}

uint8_t mServo::getID() const {
  return ID;
}