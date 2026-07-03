#include <Arduino.h>
#include <SPI.h>
#include "LoraWrapped.h"
#include <unity.h>
// Asegúrate de incluir el path correcto a tu header.
// Dependiendo de tu estructura, puede ser algo como "../include/data.h"
#include "data.h" 

// ==========================================
// Funciones de configuración de Unity
// ==========================================
void setUp(void) {
    // Esta función se ejecuta ANTES de cada test.
    // Podemos dejarla vacía para este caso.
}

void tearDown(void) {
    // Esta función se ejecuta DESPUÉS de cada test.
    // Podemos limpiar variables aquí si fuera necesario.
}

// ==========================================
// Tests para print_data_raw
// ==========================================

void test_print_data_raw_null_pointer(void) {
    // Probamos el guard de seguridad. No debería crashear.
    print_data_raw(NULL);
    
    // Si la ejecución llega hasta aquí sin reiniciar el microcontrolador, el guard funciona.
    TEST_ASSERT_TRUE_MESSAGE(true, "Manejo de puntero nulo en print_data_raw exitoso");
}

void test_print_data_raw_valid_data(void) {
    // Instanciamos e inicializamos en 0
    data_raw_t dummy_raw = {}; 
    
    // Poblamos con algunos datos de prueba
    dummy_raw.elapsed_time_micros = 1500000;
    dummy_raw.bmp.presion = 1013.25f;
    dummy_raw.bmp.temp = 22.5f;
    
    dummy_raw.mpu.accel_x = 0.1f;
    dummy_raw.mpu.accel_y = 0.05f;
    dummy_raw.mpu.accel_z = 9.81f;
    dummy_raw.mpu.gyro_x = 0.0f;
    dummy_raw.mpu.gyro_y = 0.0f;
    dummy_raw.mpu.gyro_z = 0.0f;
    dummy_raw.mpu.temp = 24.0f;

    // Nota: Como 'nav_pvt_t' viene de "UbxProtocols.h", asumimos que
    // el struct se inicializó correctamente con el '= {}' superior.

    print_data_raw(&dummy_raw);
    
    TEST_ASSERT_TRUE_MESSAGE(true, "Ejecución de print_data_raw con datos válidos completada sin errores");
}

// ==========================================
// Tests para print_data
// ==========================================

void test_print_data_null_pointer(void) {
    print_data(NULL);
    TEST_ASSERT_TRUE_MESSAGE(true, "Manejo de puntero nulo en print_data exitoso");
}

void test_print_data_valid_data(void) {
    data_all_t dummy_all = {};
    
    // Simulamos un estado del sistema (ej. en ascenso)
    dummy_all.altura_m = 150.5f;
    dummy_all.velocidad_z_m_s = 35.2f;
    dummy_all.aceleracion_z_m_s2 = 12.5f;
    
    dummy_all.momentum_kg_m_s = 45.0f;
    
    dummy_all.vel_angular_x = 2.5f;
    dummy_all.vel_angular_y = 1.2f;
    dummy_all.vel_angular_z = 0.5f;
    dummy_all.vel_rotacional_rpm = 15.0f;
    
    dummy_all.pitch_deg = 85.0f;
    dummy_all.roll_deg = 2.0f;
    
    dummy_all.temperatura_amb_c = 18.5f;
    dummy_all.densidad_aire_kg_m3 = 1.20f;
    
    // Telemetría empaquetada
    dummy_all.posicion_relativa = 150;
    dummy_all.velocidad = 35;
    dummy_all.momentum = 45;

    print_data(&dummy_all);
    
    TEST_ASSERT_TRUE_MESSAGE(true, "Ejecución de print_data con datos procesados completada sin errores");
}

// ==========================================
// Setup principal (Requerido por Arduino/PIO)
// ==========================================
void setup() {
    // Damos un pequeño retraso para que el monitor serie se conecte
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_print_data_raw_null_pointer);
    RUN_TEST(test_print_data_raw_valid_data);
    
    RUN_TEST(test_print_data_null_pointer);
    RUN_TEST(test_print_data_valid_data);
    
    UNITY_END();
}

void loop() {
    // Vacío en los tests de Unity
}