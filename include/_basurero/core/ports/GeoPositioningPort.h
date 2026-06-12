//
// Created by lucaz on 31/1/2026.
//

#ifndef GEOPOSITIONINGPORT_H
#define GEOPOSITIONINGPORT_H

#include <Arduino.h>

struct Position : public Printable {
  double latitude;
  double longitude;
};
// puede ser printeado
// ejemplo de uso --> Position p1 {40.7128, -74.0060};
// Serial.println(p1);

// GPS
class GeoPositioningPort {
public:
  GeoPositioningPort(); // acá se hacen cosas como .begin()
  bool getPosition(Position& position);
  bool getAltitude(double &altitude);

};


#endif //GEOPOSITIONINGPORT_H
