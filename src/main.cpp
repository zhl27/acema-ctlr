#include <Arduino.h>
#include <../lib/SerialPrint/SerialPrint.h>


void setup() {
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause mcu until serial console opens
}

void loop() {
  SerialPrint::plot("dummy", 10);
  delay(10);
}