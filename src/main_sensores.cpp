//
// Created by lucaz on 3/7/2026.
//

#include <Arduino.h>
#include <SerialPrint.h>

#include "services/DataFilter.h"
#include "services/Sensors.h"


void setup() {
    Serial.begin(115200);

    while (!Serial)
        delay(10); // will pause mcu until serial console opens

    Sensors::init();
    DataFilter::init();
}

long long counter = 0;

void loop() {

    const data_raw_t raw = Sensors::get_raw_data();

    const data_all_t data = DataFilter::process(raw);

    print_data(&data);

    // SerialPrint::plot("rand1", static_cast<float>(esp_random()));
    // SerialPrint::plot("rand2", static_cast<float>(esp_random()));
    delay(10);
}