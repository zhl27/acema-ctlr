#include <Arduino.h>
#include <../lib/SerialPrint/SerialPrint.h>


void setup() {
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause mcu until serial console opens
}

long long counter = 0;

void loop() {

  counter++;
  SerialPrint::plot("counter", static_cast<float>(counter));

  // SerialPrint::plot("rand1", static_cast<float>(esp_random()));
  // SerialPrint::plot("rand2", static_cast<float>(esp_random()));
  delay(10);
}