//
// Created by lucaz on 9/7/2026.
//

#ifndef ACEMA_CTLR_MAIN_H
#define ACEMA_CTLR_MAIN_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#include <cstring>
#include <cstdio>

#include "LoraWrapped.h"
#include "core/mde_cohete/mde_cohete.h"
#include "SerialPrint.h"
#include "data.h"
#include "mBuzzer.h"
// #include "services/DataFilter.h"
#include "services/GSE.h"
#include "services/Sensors.h"
#include "config.h"
#include "services/EmaFilter.h"
#include <cmath> // Para atan2
#include "services/Kalman1D.h"
#include "services/Kalman2D.h"
#include "services/CmdDispatcher/CmdDispatcher.h"
#include "services/EnlaceGSE/EnlaceGSE.h"
#include "services/Actuators.h"
#include "mPyro.h"
#include "mFlash.h"

#endif //ACEMA_CTLR_MAIN_H
