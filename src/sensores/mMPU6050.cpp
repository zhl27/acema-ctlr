//
// Created by zhl on 6/3/26.
//

#include "mMPU6050.h"

#include "actuadores/SerialPrint.h"


mMPU6050::mMPU6050()
    : accelX(0.0f), accelY(0.0f), accelZ(0.0f),
      gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f), temp(0.0f) {}

bool mMPU6050::init(uint8_t addr) {
    if (!mpu.begin(addr)) { // una guarda para salir de la función
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

void mMPU6050::update() {
    // Read and plot MPU6050 only if motion is detected
    if (mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        accelX = a.acceleration.x;
        accelY = a.acceleration.y;
        accelZ = a.acceleration.z;
        gyroX  = g.gyro.x;
        gyroY  = g.gyro.y;
        gyroZ  = g.gyro.z;
        temp   = t.temperature;

        SerialPrint::plot("a.X", static_cast<float>(accelX));
        SerialPrint::plot("a.y", static_cast<float>(accelY));
        SerialPrint::plot("a.z", static_cast<float>(accelZ));
        SerialPrint::plot("g.x", static_cast<float>(gyroX));
        SerialPrint::plot("g.y", static_cast<float>(gyroY));
        SerialPrint::plot("g.z", static_cast<float>(gyroZ));
    }
}