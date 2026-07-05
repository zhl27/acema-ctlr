//
// Created by lucaz on 5/7/2026.
//

#include <Arduino.h>
#include <unity.h>
#include "Actuators.h"

// ====================================================================
// Funciones requeridas por el framework Unity
// ====================================================================

void setUp(void) {
    // Esta función se ejecuta ANTES de cada RUN_TEST.
    // Como Actuators es estática, su estado persiste entre tests,
    // pero podemos asegurarnos de desarmar los pirotécnicos por seguridad.
    Actuators::getPyroDrogue().desarmar();
    Actuators::getPyroPpal().desarmar();
    Actuators::getBuzzer().off();
}

void tearDown(void) {
    // Esta función se ejecuta DESPUÉS de cada RUN_TEST.
    // Ideal para apagar cosas y dejar el hardware en estado seguro.
    Actuators::getPyroDrogue().desarmar();
    Actuators::getPyroPpal().desarmar();
    Actuators::getBuzzer().off();
}

// ====================================================================
// Casos de Prueba (Test Cases)
// ====================================================================

void test_actuators_init(void) {
    // Probamos que la inicialización general devuelva true
    bool init_result = Actuators::init();
    TEST_ASSERT_TRUE_MESSAGE(init_result, "Actuators::init() deberia retornar true");
}

void test_buzzer_access_and_state(void) {
    // Obtenemos la referencia
    mBuzzer& buzzer = Actuators::getBuzzer();

    // Verificamos que por defecto esté apagado (según lógica del setUp)
    TEST_ASSERT_FALSE_MESSAGE(buzzer.isOn(), "El buzzer deberia estar apagado inicialmente");

    // Probamos cambiar el estado
    buzzer.on();
    TEST_ASSERT_TRUE_MESSAGE(buzzer.isOn(), "El buzzer deberia estar encendido tras llamar a on()");
}

void test_pyro_access_and_default_state(void) {
    mPyro& drogue = Actuators::getPyroDrogue();
    mPyro& ppal = Actuators::getPyroPpal();

    // Verificamos que por seguridad arranquen desarmados
    TEST_ASSERT_FALSE_MESSAGE(drogue.estaArmado(), "El piro drogue deberia estar desarmado por seguridad");
    TEST_ASSERT_FALSE_MESSAGE(ppal.estaArmado(), "El piro ppal deberia estar desarmado por seguridad");
}

void test_pyro_arm_disarm_logic(void) {
    mPyro& drogue = Actuators::getPyroDrogue();

    // Probamos lógica de armado
    drogue.armar();
    TEST_ASSERT_TRUE_MESSAGE(drogue.estaArmado(), "El piro drogue deberia estar armado tras llamar a armar()");

    // Probamos lógica de desarmado
    drogue.desarmar();
    TEST_ASSERT_FALSE_MESSAGE(drogue.estaArmado(), "El piro drogue deberia estar desarmado tras llamar a desarmar()");
}

void test_servo_access(void) {
    mServo& servo = Actuators::getServo();

    // No podemos testear mucho estado interno del servo sin mockear hardware,
    // pero podemos asegurar que la referencia existe y el objeto es accesible.
    TEST_ASSERT_NOT_NULL_MESSAGE(&servo, "La referencia al Servo no deberia ser nula");
}

// ====================================================================
// Entry Point para Arduino + PlatformIO
// ====================================================================

void setup() {
    // Esperamos 2 segundos para que se abra el Monitor Serial antes de imprimir resultados
    delay(2000);

    // Iniciamos el framework de testing
    UNITY_BEGIN();

    // Ejecutamos las pruebas
    RUN_TEST(test_actuators_init);
    RUN_TEST(test_buzzer_access_and_state);
    RUN_TEST(test_pyro_access_and_default_state);
    RUN_TEST(test_pyro_arm_disarm_logic);
    RUN_TEST(test_servo_access);

    // Finalizamos y mostramos el resumen
    UNITY_END();
}

void loop() {
    // No se necesita nada en el loop para Unity.
    // Hacemos parpadear un LED o simplemente lo dejamos en delay para no saturar el procesador.
    delay(1000);
}