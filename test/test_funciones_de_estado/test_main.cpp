//
// Created by lucaz on 24/7/2026.
//

#include "LoraWrapped.h"
#include "SPI.h"

#include <unity.h>

#include "cstdint"
#include "core/mde_cohete/funciones_de_estado.h"
#include "data.h"

Cohete::system_data_t SYSTEM;

// setUp se ejecuta antes de cada test individual
void setUp(void) {
    SYSTEM = {
        .estado = Cohete::ST_INIT,
        .estado_anterior = Cohete::ST_NULL,
        ._error = Cohete::ERR_NINGUNO,
        ._entrando_estado = false,
        .procesos ={
            .xTaskReadSensorsHandle = NULL,
            .xTaskStateMachineHandle = NULL,
            .xTaskFlashHandle = NULL,
            .xTaskLoraHandle = NULL,
            .xTaskDataFilterHandle = NULL,
            .flujos = {
                .Sensors_a_StateMachine_enabled = true,
                .Sensors_a_Flash_enabled = true,
                .Sensors_a_Lora_enabled = true
            }
        },

        .gse_configs = {
            .gps_override_skip = false,
        },

        .timestamp_millis_entrada_estado = 0,
        .timestamp_millis_inicio_pico_g = 0,
        .timestamp_micros_apertura_drogue = 0,

        .ctx_fisico = {
            .altura_m_max_historica = 0.0f,
            .masa_g_cohete = 0, // TODO: masa_cohete_kg debe ser configurable a traves de comando desde GSE: "set_masa_cohete_kg" o similar
            .masa_g_combustible = 0, // TODO: masa_combustible_kg debe ser configurable a traves de comando desde GSE: "set_masa_combustible_kg" o similar
            .altitud_m_pad = 0.0f, // TODO: altitud_m_pad toma el valor actual de la altitud_bmp --> cuando comando desde GSE: "tara_altitud" o similar
            .altitud_m_relativa_al_pad = 0.0f // se actualiza utilizando SYSTEM.ctx_fisico.altitud_m_cero_pad
        },

        .flags = {
            // .gse_conectado = false,
            .flash_log_borrado = false, // Se debe borrar el log de datos basura
            // .gps_preciso = false,
            .drogue_disparado = false,
            .paracaidas_principal_disparado = false,
            .emergencia_fatal = false,
            // .borrar_log = false,
            .volcar_ram_a_flash = false,
        },
    };
}

// tearDown se ejecuta después de cada test
void tearDown(void) {
    // Ya no requerimos liberar memoria con 'delete'
}

// -------------------------------------------------------------------------
// CASOS DE PRUEBA
// -------------------------------------------------------------------------

void test_evento_en_codiciones_para_volar(void) {
    data_all_t datos_sensores = {};

    bool res = Cohete::Eventos::en_codiciones_para_volar(&datos_sensores);

    TEST_ASSERT_TRUE_MESSAGE(true, "mensaje loco");
}

void test_evento_hay_boost(void) {
    bool loopeamos = true;
    int elapsed_ms = 0;

    data_all_t datos_sensores = {};

    while (loopeamos) {
        datos_sensores.aceleracion_z_m_s2 = 2 * 10;
        bool res = Cohete::Eventos::hay_boost(&datos_sensores);
    }

    TEST_ASSERT_TRUE_MESSAGE(false, "mensaje loco");

    // TEST_ASSERT_TRUE_MESSAGE(servoTest.init(), "init() deberia retornar true con una configuracion valida");
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, servoTest.getID(), "El ID del servo no coincide con la configuracion");
    //
    // // Al inicializar, el servo debe mandarse a 0.0°
    // TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest.getAngulo(), "El angulo inicial al hacer init() debe ser 0.0");
    //
    // // Pausa para que el servo viaje físicamente a su posición 0° inicial
    // delay(600);
}

// // Test 3: Verificar movimientos físicos a ángulos dentro del rango
// void test_mservo_movimiento_rango_valido(void) {
//     // Mover a 45 grados
//     servoTest.setAngulo(45.0f);
//     TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 45.0f, servoTest.getAngulo(), "El servo no registro el comando de 45.0");
//     delay(400); // Tiempo para movimiento físico
//
//     // Mover a 90 grados (centro)
//     servoTest.setAngulo(90.0f);
//     TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 90.0f, servoTest.getAngulo(), "El servo no registro el comando de 90.0");
//     delay(400);
//
//     // Mover a 180 grados (apertura máxima)
//     servoTest.setAngulo(180.0f);
//     TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest.getAngulo(), "El servo no registro el comando de 180.0");
//     delay(600); // Recorrido más largo, damos un poco más de tiempo
// }

// // Test 4: Verificar saturación y truncamiento en límites inferiores (Underflow)
// void test_mservo_saturacion_limite_inferior(void) {
//     // Intentar mover a un ángulo negativo (ej. -30.0°)
//     servoTest.setAngulo(-30.0f);
//
//     // El software debe truncar a 0.0°, y físicamente el servo debe ir a la posición 0°
//     TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 0.0f, servoTest.getAngulo(), "El angulo deberia saturar en 0.0 para comandos negativos");
//     delay(600);
// }
//
// // Test 5: Verificar saturación y truncamiento en límites superiores (Overflow)
// void test_mservo_saturacion_limite_superior(void) {
//     // Intentar mover a un ángulo superior al máximo (ej. 250.0°)
//     servoTest.setAngulo(250.0f);
//
//     // El software debe truncar a 180.0°, y físicamente el motor va a su tope máximo
//     TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 180.0f, servoTest.getAngulo(), "El angulo deberia saturar en 180.0 para comandos mayores a 180");
//     delay(600);
//
//     // Regresamos el servo a 0° al finalizar todas las pruebas por seguridad mecánica
//     servoTest.setAngulo(0.0f);
//     delay(500);
// }

// -------------------------------------------------------------------------
// PUNTO DE ENTRADA ARDUINO / PLATFORMIO
// -------------------------------------------------------------------------
void setup() {
    // Pausa de seguridad para estabilizar la alimentación y permitir conectar el monitor serial
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_evento_en_codiciones_para_volar);
    RUN_TEST(test_evento_hay_boost);

    UNITY_END();
}

void loop() {
    // Eliminamos la tarea actual de FreeRTOS para detener la ejecución del loop
    vTaskDelete(NULL);
}