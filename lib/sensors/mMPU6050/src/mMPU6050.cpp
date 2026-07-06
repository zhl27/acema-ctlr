//
// Created by zhl on 6/3/26.
//

#include "mMPU6050.h"

#include "SerialPrint.h"


// TODO: Falta revisión general de mMPU6050, especialmente en la lógica de actualización y detección de movimiento. Se recomienda implementar un Unit Test que simule movimientos significativos y menores para confirmar que el método update() solo actualiza y envía datos al plot cuando se detecta movimiento real, evitando falsos positivos por ruido o vibraciones menores.


mMPU6050::mMPU6050()
    : accelX(0.0f), accelY(0.0f), accelZ(0.0f),
      gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f), temp(0.0f) {}

bool mMPU6050::init(uint8_t addr) {
    if (!_mpu.begin(addr)) {
        return false;
    }

    // 1. Maximize Sensitivity
    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    // 2. Maximize Speed (1kHz internal sampling)
    _mpu.setFilterBandwidth(MPU6050_BAND_184_HZ); // Sets DLPF to ~188Hz base
    _mpu.setSampleRateDivisor(0);                 // Keeps the sample rate at 1kHz

    // 3. Configure the Interrupt Pin Behavior
    _mpu.setInterruptPinPolarity(false);          // Active High pulse
    _mpu.setInterruptPinLatch(false);             // 50us pulse instead of latching
    _mpu.setMotionInterrupt(false);               // Disable motion gating

    // 4. Manually Enable the Data Ready Interrupt
    // (Because the Adafruit library doesn't have a native method for this specific register)
    // Wire.beginTransmission(0x68);
    // Wire.write(0x38);                            // Target the INT_ENABLE register
    // Wire.write(0x01);                            // Write 1 to the DATA_RDY_EN bit
    // Wire.endTransmission();

    return true;
}

data_raw_mpu_t mMPU6050::get_mpu_raw_data() {

    data_raw_mpu_t raw_mpu = {};

    // Read and plot MPU6050 only if motion is detected
    // if (mpu.getMotionInterruptStatus())
    // {
    sensors_event_t a, g, t;
    _mpu.getEvent(&a, &g, &t);

    raw_mpu.accel_x = a.acceleration.x;
    raw_mpu.accel_y = a.acceleration.y;
    raw_mpu.accel_z = a.acceleration.z;
    raw_mpu.gyro_x  = g.gyro.x;
    raw_mpu.gyro_y  = g.gyro.y;
    raw_mpu.gyro_z  = g.gyro.z;
    // raw_mpu.temp    = t.temperature;
    // }

    return raw_mpu;
}

int calibrar() {
    // TODO: CALIBRAR MPU6050
    return 0;
}
