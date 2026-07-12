//
// Created by zhl on 6/3/26.
//

#include "mMPU6050.h"

#include "SerialPrint.h"


// TODO: Falta revisión general de mMPU6050, especialmente en la lógica de actualización y detección de movimiento. Se recomienda implementar un Unit Test que simule movimientos significativos y menores para confirmar que el método update() solo actualiza y envía datos al plot cuando se detecta movimiento real, evitando falsos positivos por ruido o vibraciones menores.


mMPU6050::mMPU6050()
    : accelX(0.0f), accelY(0.0f), accelZ(0.0f),
      gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f), temp(0.0f),
      CalAccelX(0.0f), CalAccelY(0.0f), CalAccelZ(0.0f),
      CalGyroX(0.0f), CalGyroY(0.0f), CalGyroZ(0.0f) {}

bool mMPU6050::init(uint8_t addr) {
    if (!_mpu.begin(addr)) {
        return false;
    }

    // Maximiza el rango para soportar la dinámica del cohete
    _mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // 16 G de fuerza
    _mpu.setGyroRange(MPU6050_RANGE_2000_DEG); // 2000 °/seg de aceleración angular

    // Filtro Pasa Bajos (DLPF) para mitigar vibración del motor
    _mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // Lo configuro a fc = 10 hz, pero si hay ruido bajarlo a 5 hz
    _mpu.setSampleRateDivisor(0); // Mantiene el muestreo a 1kHz internamente

    // Configuración de Interrupciones (Opcional, según tu arquitectura actual)
    // Sirve para leer el sensor con el pin de interrupciones, no usado en la pcb
    _mpu.setInterruptPinPolarity(false);          
    _mpu.setInterruptPinLatch(false);             
    _mpu.setMotionInterrupt(false);               

    // Calibra el sensor asumiendo que está en la rampa de lanzamiento
    calibrar();

    return true;
}

int mMPU6050::calibrar() {
    const int num_muestras = 1000;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    sensors_event_t a, g, t;

    // Descartar las primeras lecturas (estabilización)
    for (int i = 0; i < 100; i++) {
        _mpu.getEvent(&a, &g, &t);
        delay(2);
    }

    // Tomar N muestras
    for (int i = 0; i < num_muestras; i++) {
        _mpu.getEvent(&a, &g, &t);
        sum_ax += a.acceleration.x;
        sum_ay += a.acceleration.y;
        sum_az += a.acceleration.z;
        sum_gx += g.gyro.x;
        sum_gy += g.gyro.y;
        sum_gz += g.gyro.z;
        delay(2);
    }

    // Promediar giroscopios (Deberían ser 0 en reposo)
    CalGyroX = sum_gx / num_muestras;
    CalGyroY = sum_gy / num_muestras;
    CalGyroZ = sum_gz / num_muestras;

    // Promediar acelerómetros
    // NOTA: El eje X apunta hacia arriba (+Z del cohete). 
    // Debe medir 1G (9.81 m/s^2) positivo o negativo dependiendo de la convención física.
    // Asumimos que la gravedad empuja hacia abajo, por lo que el sensor siente una aceleración normal hacia arriba de +9.81 m/s^2.
    CalAccelX = (sum_ax / num_muestras) - 9.80665f; 
    CalAccelY = (sum_ay / num_muestras); // Debería ser 0
    CalAccelZ = (sum_az / num_muestras); // Debería ser 0

    return 0; // Éxito
}

data_raw_mpu_t mMPU6050::get_mpu_raw_data() {
    data_raw_mpu_t raw_mpu = {};
    sensors_event_t a, g, t;
    
    _mpu.getEvent(&a, &g, &t);

    // Aplicamos el offset calculado en la calibración
    raw_mpu.accel_x_g = a.acceleration.x - CalAccelX;
    raw_mpu.accel_y_g = a.acceleration.y - CalAccelY;
    raw_mpu.accel_z_g = a.acceleration.z - CalAccelZ;
    
    raw_mpu.gyro_x_rad_s  = g.gyro.x - CalGyroX;
    raw_mpu.gyro_y_rad_s  = g.gyro.y - CalGyroY;
    raw_mpu.gyro_z_rad_s  = g.gyro.z - CalGyroZ;

    return raw_mpu;
}

