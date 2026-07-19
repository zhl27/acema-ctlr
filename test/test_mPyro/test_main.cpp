#include <Arduino.h>
#include <unity.h>
#include "mPyro.h"

#include <LoraWrapped.h>
#include <SPI.h>

// Pines de prueba (ajustar según el hardware conectado durante el test)
const uint8_t TEST_PIN_ACTIVAR = 12; 
const uint8_t TEST_PIN_CONT = 35;    

mPyro piro(TEST_PIN_ACTIVAR, TEST_PIN_CONT, 500); // Umbral de 500 mV

void setUp(void) {
    piro.init();
    piro.desarmar(); // Asegurar estado inicial conocido
}

void tearDown(void) {
    piro.desarmar();
    digitalWrite(TEST_PIN_ACTIVAR, LOW);
}

// ==========================================
// PRUEBAS DE FUNCIONAMIENTO IDEAL
// ==========================================

void test_pyro_initial_state(void) {
    TEST_ASSERT_FALSE(piro.estaArmado());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_PIN_ACTIVAR));
}

void test_pyro_armar_y_desarmar(void) {
    piro.armar();
    TEST_ASSERT_TRUE(piro.estaArmado());
    
    piro.desarmar();
    TEST_ASSERT_FALSE(piro.estaArmado());
}

void test_pyro_disparo_exitoso(void) {
    piro.armar();
    
    // Disparamos con un pulso corto de 50ms para el test
    bool aceptado = piro.disparar(50);
    
    TEST_ASSERT_TRUE(aceptado);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(TEST_PIN_ACTIVAR));
    TEST_ASSERT_FALSE(piro.estaArmado()); // El seguro se debe poner automáticamente
}

// ==========================================
// PRUEBAS DE ROTURA / SEGURIDAD
// ==========================================

void test_pyro_rechaza_disparo_si_desarmado(void) {
    piro.desarmar(); // Estado explícito
    
    bool aceptado = piro.disparar(100);
    
    TEST_ASSERT_FALSE(aceptado);
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_PIN_ACTIVAR)); // El pin no debió subir
}

void test_pyro_timer_freertos_corta_corriente(void) {
    piro.armar();
    piro.disparar(50); // Tiempo de ignición: 50ms
    
    // Inmediatamente el pin debe estar en HIGH
    TEST_ASSERT_EQUAL(HIGH, digitalRead(TEST_PIN_ACTIVAR));

    // Bloqueamos la tarea de test actual para permitir que FreeRTOS 
    // procese el timer en segundo plano. Esperamos el doble del tiempo (100ms)
    vTaskDelay(pdMS_TO_TICKS(100));

    // El callback _timerCallback debió ejecutarse y bajar el pin
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_PIN_ACTIVAR));
}

void test_pyro_continuidad_rechaza_falsos_positivos(void) {
    // NOTA: Para esta prueba se asume que el pin TEST_PIN_CONT no tiene
    // voltaje aplicado físicamente durante el unit test (está en 0V).
    
    bool continuidad = piro.tieneContinuidad();
    
    // Como está en 0V y el umbral es 500mV, debe dar false (circuito abierto)
    TEST_ASSERT_FALSE(continuidad);
}

// ==========================================
// ENTRY POINT
// ==========================================

void setup() {
    delay(2000); 
    UNITY_BEGIN();
    
    RUN_TEST(test_pyro_initial_state);
    RUN_TEST(test_pyro_armar_y_desarmar);
    RUN_TEST(test_pyro_rechaza_disparo_si_desarmado);
    
    RUN_TEST(test_pyro_disparo_exitoso);
    RUN_TEST(test_pyro_timer_freertos_corta_corriente);
    
    RUN_TEST(test_pyro_continuidad_rechaza_falsos_positivos);
    
    UNITY_END();
}

void loop() {}