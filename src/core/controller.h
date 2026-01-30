//
// Created by lucaz on 30/1/2026.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ports/LcdPort.h"
#include "ports/LoggingPort.h"
#include "ports/TelemetryPort.h"


class Controller {
  LoggingPort& logging;
  TelemetryPort& telemetryPort;
public:
  Controller(LoggingPort& logging, TelemetryPort& telemetry) : logging(logging), telemetry(telemetry) {};
};



#endif //CONTROLLER_H
