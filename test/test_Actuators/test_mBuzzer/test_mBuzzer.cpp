#include <Arduino.h>
#include <unity.h>
#include "mBuzzer.h"

// Usamos el pin 25 por defecto como en la clase
const int TEST_BUZZER_PIN = 25; 
mBuzzer buzzer(TEST_BUZZER_PIN);

// setUp() y tearDown() son nativos de Unity y se ejecutan antes y después de cada test
void setUp(void) {
    buzzer.init();
}

void tearDown(void) {
    buzzer.off();
}

// --------------------------------------------
// PRUEBAS DE FUNCIONAMIENTO IDEAL
// --------------------------------------------

void test_buzzer_initial_state(void) {
    TEST_ASSERT_FALSE(buzzer.isOn());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_BUZZER_PIN));
}

void test_buzzer_manual_on_off(void) {
    buzzer.on();
    TEST_ASSERT_TRUE(buzzer.isOn());
    TEST_ASSERT_EQUAL(HIGH, digitalRead(TEST_BUZZER_PIN));

    buzzer.off();
    TEST_ASSERT_FALSE(buzzer.isOn());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_BUZZER_PIN));
}

void test_buzzer_toggle(void) {
    // Partimos de apagado
    buzzer.toggle();
    TEST_ASSERT_TRUE(buzzer.isOn());
    TEST_ASSERT_EQUAL(HIGH, digitalRead(TEST_BUZZER_PIN));

    // Volvemos a presionar
    buzzer.toggle();
    TEST_ASSERT_FALSE(buzzer.isOn());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_BUZZER_PIN));
}

void test_buzzer_sequence_beep_ideal(void) {
    uint32_t beepDuration = 50; // 50 ms
    
    buzzer.beep(beepDuration);
    buzzer.runBuzzer();
    
    // Inmediatamente después de iniciar, debe estar ON
    TEST_ASSERT_TRUE(buzzer.isOn());
    
    // Esperamos un tiempo menor a la duración
    delay(20);
    buzzer.runBuzzer();
    TEST_ASSERT_TRUE(buzzer.isOn()); // Aún debería estar sonando

    // Esperamos a que venza el timer
    delay(40); // 20 + 40 = 60ms (> 50ms)
    buzzer.runBuzzer();
    
    // El timer soft debió apagarlo
    TEST_ASSERT_FALSE(buzzer.isOn());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_BUZZER_PIN));
}

// --------------------------------------------
// PRUEBAS DE ROTURA (EDGE CASES)
// --------------------------------------------

void test_buzzer_beep_zero_duration(void) {
    buzzer.beep(0);
    
    // Al setearlo, arranca en ON inmediatamente
    TEST_ASSERT_TRUE(buzzer.isOn());
    
    // Forzamos un delay mínimo para que millis() avance al menos 1ms
    delay(2); 
    buzzer.runBuzzer();
    
    // Al ejecutar la FSM, debe detectar que (millis() - last > 0) y apagarse
    TEST_ASSERT_FALSE(buzzer.isOn());
}

void test_buzzer_override_sequence_with_manual_off(void) {
    // Iniciamos una secuencia larga
    buzzer.playError(); // 500ms ON[cite: 3]
    buzzer.runBuzzer();
    TEST_ASSERT_TRUE(buzzer.isOn());

    // Interrumpimos a la mitad
    delay(100);
    buzzer.off();
    buzzer.runBuzzer();

    // Verificamos que el apagado manual canceló la secuencia exitosamente
    TEST_ASSERT_FALSE(buzzer.isOn());
    TEST_ASSERT_EQUAL(LOW, digitalRead(TEST_BUZZER_PIN));
}

// --------------------------------------------
// ENTRY POINT PARA PLATFORMIO
// --------------------------------------------

void setup() {
    // Pequeño retardo para que el monitor serie se estabilice
    delay(2000); 
    
    UNITY_BEGIN();
    
    // Tests normales
    RUN_TEST(test_buzzer_initial_state);
    RUN_TEST(test_buzzer_manual_on_off);
    RUN_TEST(test_buzzer_toggle);
    RUN_TEST(test_buzzer_sequence_beep_ideal);
    
    // Tests de estrés / edge cases
    RUN_TEST(test_buzzer_beep_zero_duration);
    RUN_TEST(test_buzzer_override_sequence_with_manual_off);
    
    UNITY_END();
}

void loop() {
    // No se necesita nada en el loop para los tests
}