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
    if (!mpu.begin(addr)) {
        return false;
    }

    // Motion detection and interrupt setup as configured in original test code
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    return true;
}

data_raw_mpu_t mMPU6050::get_raw_mpu() {

    data_raw_mpu_t raw_mpu = {};

    // Read and plot MPU6050 only if motion is detected
    if (mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        raw_mpu.accel_x = a.acceleration.x;
        raw_mpu.accel_y = a.acceleration.y;
        raw_mpu.accel_z = a.acceleration.z;
        raw_mpu.gyro_x  = g.gyro.x;
        raw_mpu.gyro_y  = g.gyro.y;
        raw_mpu.gyro_z  = g.gyro.z;
        raw_mpu.temp    = t.temperature;
    }

    return raw_mpu;
}