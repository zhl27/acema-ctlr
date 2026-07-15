//
// Created by zhl on 6/10/26.
//

#include <Arduino.h>
#include <unity.h>
#include "Vector3D.h" // Incluimos la lógica a testear

// arregla el error de SPI
#include "LoraWrapped.h"
#include <SPI.h>

// Unity requiere estas dos funciones obligatoriamente
void setUp(void) {
    // Se ejecuta ANTES de cada test (ideal para resetear variables, pines I2C/SPI, etc.)
}

void tearDown(void) {
    // Se ejecuta DESPUÉS de cada test
}

// --- Casos de Prueba ---

void test_vector_suma(void) {
    Vector3D<float> v1{1.0f, 2.0f, 3.0f};
    Vector3D<float> v2{4.0f, 5.0f, 6.0f};
    Vector3D<float> res = v1 + v2;

    // Usamos ASSERT_EQUAL_FLOAT para evitar problemas de precisión con decimales
    TEST_ASSERT_EQUAL_FLOAT(5.0f, res.x);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, res.y);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, res.z);
}

void test_vector_producto_punto(void) {
    Vector3D<float> v1{1.0f, 2.0f, 3.0f};
    Vector3D<float> v2{4.0f, 5.0f, 6.0f};
    float res = v1 * v2;

    TEST_ASSERT_EQUAL_FLOAT(32.0f, res);
}

// --- Configuración de Arduino ---

void setup() {
    // Pequeño delay para dar tiempo a que se estabilice el puerto serie del ESP32
    delay(2000);

    UNITY_BEGIN(); // Inicializa el framework

    // Ejecutamos las pruebas
    RUN_TEST(test_vector_suma);
    RUN_TEST(test_vector_producto_punto);

    UNITY_END(); // Finaliza y envía los resultados por el puerto serie
}

void loop() {
    // No necesitamos que haga nada después de correr los tests
}