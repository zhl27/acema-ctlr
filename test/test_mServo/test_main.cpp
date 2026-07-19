#include <Arduino.h>
#include <unity.h>
#include "mServo.h"

#include <SPI.h>
#include "LoraWrapped.h"

// Definición del GPIO real donde está conectado el cable de señal del servo
#define PIN_SERVO_TEST 27

// Configuración pre-asignada en memoria estática
mServoConfig_t configServo = {
    .pinServo = PIN_SERVO_TEST,
    .minPulse = 500,   // 500 µs para 0°
    .maxPulse = 2500,  // 2500 µs para 180°
    .id = 1
};

// Instanciación directa del objeto sin punteros ni memoria heap (cero 'new')
mServo servoTest(&configServo);

// setUp se ejecuta antes de cada test individual
void setUp(void) {
    // Ya no requerimos inicializar punteros ni asignar memoria con 'new'
}

// tearDown se ejecuta después de cada test
void tearDown(void) {
    // Ya no requerimos liberar memoria con 'delete'
}

// -------------------------------------------------------------------------
// CASOS DE PRUEBA
// -------------------------------------------------------------------------

// Test 1: Verificar el comportamiento frente a una configuración nula en el stack local
void test_mservo_init_con_config_nula(void) {
    // Instanciación temporal en el stack de la función
    mServo servoNulo(nullptr);
    TEST_ASSERT_FALSE_MESSAGE(servoNulo.init(), "init() deberia retornar false si la configuracion es nullptr");
}

// Test 2: Verificar la inicialización correcta del hardware y valores por defecto
void test_mservo_inicializacion_exitosa(void) {
    TEST_ASSERT_TRUE_MESSAGE(servoTest.init(), "init() deberia retornar true con una configuracion valida");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, servoTest.getID(), "El ID del servo no coincide con la configuracion");

    // Al inicializar, el servo debe mandarse a 0.0°
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest.getAngulo(), "El angulo inicial al hacer init() debe ser 0.0");

    // Pausa para que el servo viaje físicamente a su posición 0° inicial
    delay(600);
}

// Test 3: Verificar movimientos físicos a ángulos dentro del rango
void test_mservo_movimiento_rango_valido(void) {
    // Mover a 45 grados
    servoTest.setAngulo(45.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 45.0f, servoTest.getAngulo(), "El servo no registro el comando de 45.0");
    delay(400); // Tiempo para movimiento físico

    // Mover a 90 grados (centro)
    servoTest.setAngulo(90.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 90.0f, servoTest.getAngulo(), "El servo no registro el comando de 90.0");
    delay(400);

    // Mover a 180 grados (apertura máxima)
    servoTest.setAngulo(180.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest.getAngulo(), "El servo no registro el comando de 180.0");
    delay(600); // Recorrido más largo, damos un poco más de tiempo
}

// Test 4: Verificar saturación y truncamiento en límites inferiores (Underflow)
void test_mservo_saturacion_limite_inferior(void) {
    // Intentar mover a un ángulo negativo (ej. -30.0°)
    servoTest.setAngulo(-30.0f);

    // El software debe truncar a 0.0°, y físicamente el servo debe ir a la posición 0°
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest.getAngulo(), "El angulo deberia saturar en 0.0 para comandos negativos");
    delay(600);
}

// Test 5: Verificar saturación y truncamiento en límites superiores (Overflow)
void test_mservo_saturacion_limite_superior(void) {
    // Intentar mover a un ángulo superior al máximo (ej. 250.0°)
    servoTest.setAngulo(250.0f);

    // El software debe truncar a 180.0°, y físicamente el motor va a su tope máximo
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest.getAngulo(), "El angulo deberia saturar en 180.0 para comandos mayores a 180");
    delay(600);

    // Regresamos el servo a 0° al finalizar todas las pruebas por seguridad mecánica
    servoTest.setAngulo(0.0f);
    delay(500);
}

// -------------------------------------------------------------------------
// PUNTO DE ENTRADA ARDUINO / PLATFORMIO
// -------------------------------------------------------------------------
void setup() {
    // Pausa de seguridad para estabilizar la alimentación y permitir conectar el monitor serial
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_mservo_init_con_config_nula);
    RUN_TEST(test_mservo_inicializacion_exitosa);
    RUN_TEST(test_mservo_movimiento_rango_valido);
    RUN_TEST(test_mservo_saturacion_limite_inferior);
    RUN_TEST(test_mservo_saturacion_limite_superior);

    UNITY_END();
}

void loop() {
    // Eliminamos la tarea actual de FreeRTOS para detener la ejecución del loop
    vTaskDelete(NULL);
}