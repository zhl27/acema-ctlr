//
// Created by zhl on 6/7/26.
//

#include <Arduino.h>
#include <unity.h>
#include "mFlash.h"

// Instanciamos el objeto mFlash usando el pin CS habitual (GPIO 4)
mFlash flashTest(4);

// Se ejecuta ANTES de cada test individual
void setUp(void) {
    // Código de preparación si fuera necesario
}

// Se ejecuta DESPUÉS de cada test individual
void tearDown(void) {
    // Código de limpieza si fuera necesario
}

// TEST 1: Verificar que la inicialización no rompa nada y configure los pines
void test_mflash_begin(void) {
    // En este caso begin() no retorna nada, pero nos aseguramos de que se ejecute sin colgar el micro
    flashTest.begin();

    // Verificamos de forma indirecta asegurando que el pin CS quedó en HIGH (estado en reposo de SPI)
    TEST_ASSERT_EQUAL(HIGH, digitalRead(4));
}

// TEST 2: Verificar la conexión real con el chip físico
void test_mflash_connection(void) {
    // Como testConnection retorna un booleano (true si el ID del fabricante es 0xEF)
    // podemos usarlo directamente para validar el estado del hardware.

    // Pasamos 'Serial' como argumento para que imprima el debug mientras corre el test
    bool conexionExitosa = flashTest.testConnection(Serial);

    TEST_ASSERT_TRUE_MESSAGE(conexionExitosa, "La memoria Flash no respondio con el ID esperado (0xEF)");
}

void setup() {
    // NOTA: Es crucial un delay inicial para que Unity y el monitor serie de PlatformIO se sincronicen
    delay(2000);

    UNITY_BEGIN(); // Inicializa el framework Unity

    // Ejecutamos la lista de tests
    RUN_TEST(test_mflash_begin);
    RUN_TEST(test_mflash_connection);

    UNITY_END(); // Finaliza el reporte de tests
}

void loop() {
    // El flujo de test se ejecuta una sola vez en el setup
}