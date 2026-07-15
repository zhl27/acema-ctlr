#include <Arduino.h>

#include <unity.h>
#include "mServo.h"

// arregla el error de SPI
#include "LoraWrapped.h"
#include <SPI.h>

// Definición de un pin PWM válido en el ESP32 para las pruebas
#define PIN_SERVO_TEST 27

// Variables globales para la suite de pruebas
mServoConfig_t configServo;
mServo* servoTest = nullptr;

// Unity llama a setUp() antes de ejecutar cada test individual
void setUp(void) {
    configServo.pinServo = PIN_SERVO_TEST;
    configServo.minPulse = 500;   // 500 µs para 0°
    configServo.maxPulse = 2500;  // 2500 µs para 180°
    configServo.id = 1;

    servoTest = new mServo(&configServo);
}

// Unity llama a tearDown() después de ejecutar cada test individual
void tearDown(void) {
    if (servoTest != nullptr) {
        delete servoTest;
        servoTest = nullptr;
    }
}

// Test 1: Verificar el comportamiento frente a una configuración nula (Null Pointer)
void test_mservo_init_con_config_nula(void) {
    mServo servoNulo(nullptr);
    TEST_ASSERT_FALSE_MESSAGE(servoNulo.init(), "init() debería retornar false si la configuración es nullptr");
}

// Test 2: Verificar la inicialización correcta del hardware y valores por defecto
void test_mservo_inicializacion_exitosa(void) {
    TEST_ASSERT_TRUE_MESSAGE(servoTest->init(), "init() debería retornar true con una configuración válida");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, servoTest->getID(), "El ID del servo no coincide con la configuración");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest->getAngulo(), "El ángulo inicial al hacer init() debe ser 0.0°");
}

// Test 3: Verificar movimientos a ángulos válidos dentro del rango de operación
void test_mservo_set_angulo_rango_valido(void) {
    servoTest->init();

    // Mover a 45 grados
    servoTest->setAngulo(45.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 45.0f, servoTest->getAngulo(), "El servo no registró el ángulo de 45.0°");

    // Mover a 90 grados (punto medio)
    servoTest->setAngulo(90.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 90.0f, servoTest->getAngulo(), "El servo no registró el ángulo de 90.0°");

    // Mover a 180 grados (máximo)
    servoTest->setAngulo(180.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest->getAngulo(), "El servo no registró el ángulo de 180.0°");
}

// Test 4: Verificar saturación y truncamiento en límites inferiores (Underflow)
void test_mservo_saturacion_limite_inferior(void) {
    servoTest->init();

    // Intentar mover a un ángulo negativo (ej. -25.0°)
    servoTest->setAngulo(-25.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest->getAngulo(), "El ángulo debería saturar en 0.0° para comandos negativos");
}

// Test 5: Verificar saturación y truncamiento en límites superiores (Overflow)
void test_mservo_saturacion_limite_superior(void) {
    servoTest->init();

    // Intentar mover a un ángulo superior al máximo (ej. 250.0°)
    servoTest->setAngulo(250.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest->getAngulo(), "El ángulo debería saturar en 180.0° para comandos > 180.0°");
}

// En el framework Arduino sobre ESP32, el punto de entrada para Unity es setup()
void setup() {
    // Es recomendable una pequeña pausa para permitir que el monitor serial se conecte
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_mservo_init_con_config_nula);
    RUN_TEST(test_mservo_inicializacion_exitosa);
    RUN_TEST(test_mservo_set_angulo_rango_valido);
    RUN_TEST(test_mservo_saturacion_limite_inferior);
    RUN_TEST(test_mservo_saturacion_limite_superior);

    UNITY_END();
}

void loop() {
    // No es necesario hacer nada en el loop tras ejecutar las pruebas unitarias
    vTaskDelete(NULL);
}