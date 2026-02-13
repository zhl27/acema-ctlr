#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>   // BMP180
#include <Adafruit_BMP280.h>
#include <Wire.h>

#include <../lib/SerialPrint/SerialPrint.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp180;
Adafruit_BMP280 bmp280;



void setup() {
  Serial.begin(921600);

  Wire.begin(21, 22);
  Wire.setClock(400000); // I2C a 400kHz (Fast Mode)

  while (!Serial)
    delay(10); // will pause mcu until serial console opens

  if (!mpu.begin()) {
    SerialPrint::err("Failed to find MPU6050 chip.");
    while (1) delay(10);
  }
  else {
    SerialPrint::msg("MPU6050 Found!");
    // Configurar MPU para máxima velocidad de respuesta
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  }

  if (!bmp180.begin()) {
    SerialPrint::err("Could not find a valid BMP180 sensor, check wiring!");
    while (1) delay(10);
  }
  else {
    SerialPrint::msg("BMP180 Found!");
  }

  if (!bmp280.begin(0x76)) {
    SerialPrint::err("Could not find a valid BMP180 sensor, check wiring!");
    // If you don't have a BMP180 connected, you should comment out the BMP180 lines in loop()
    while (1) delay(10);
  }
  else {
    SerialPrint::msg("BMP180 Found!");
    // Config BMP280
    bmp280.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_250
    );
  }

  delay(100);
}



void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // // Calculate altitude assuming 'standard' barometric
  // // pressure of 1013.25 millibar = 101325 Pascal
  // Serial.print(bmp180.readAltitude()); // meters
  // Serial.print(bmp180.readSealevelPressure()); // Pa
  // // you can get a more precise measurement of altitude
  // // if you know the current sea level pressure which will
  // // vary with weather and such. If it is 1015 millibars
  // // that is equal to 101500 Pascals.
  // Serial.print(bmp180.readAltitude(102000));


   // Accelerometer
  SerialPrint::plot("ax", a.acceleration.x);
  SerialPrint::plot("ay", a.acceleration.y);
  SerialPrint::plot("az", a.acceleration.z);

  // Gyroscope
  SerialPrint::plot("gx", g.gyro.x);
  SerialPrint::plot("gy", g.gyro.y);
  SerialPrint::plot("gz", g.gyro.z);

  // Temperature (MPU)
  SerialPrint::plot("mt", temp.temperature);

  // BMP180 Data
  SerialPrint::plot("bt", bmp180.readTemperature());
  SerialPrint::plot("bp", bmp180.readSealevelPressure());
  SerialPrint::plot("ba", bmp180.readAltitude(102000));


  delay(10);
}