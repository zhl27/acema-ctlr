//
// Created by lucaz on 31/1/2026.
//

#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <fases.h>

#include "ports/LcdPort.h"
#include "ports/LoggingPort.h"
#include "ports/TelemetryPort.h"
#include "ports/GeoPositioningPort.h"


class FlightController {
    FaseDeVuelo *faseActual;

    LoggingPort& logging_;
    TelemetryPort& telemetry_;
    GeoPositioningPort& geoPositioning_;
public:
    FlightController(
      LoggingPort& logging,
      TelemetryPort& telemetry,
      GeoPositioningPort& geoPositioning
      ) : faseActual(new PREVUELO),
          logging_(logging),
          telemetry_(telemetry),
          geoPositioning_(geoPositioning) {
    };

    void update();
};

/*


         [ Outside World ]
   sensors | buttons | UART | WiFi | flash | OS | drivers
                     ↓
               ADAPTERS
                     ↓
                 PORTS
                     ↓
               CORE LOGIC
 */



#endif //FLIGHTCONTROLLER_H
