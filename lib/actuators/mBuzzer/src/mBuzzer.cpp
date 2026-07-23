//
// Created by zhl on 6/4/26.
//

#include "mBuzzer.h"

// --------------------------------------------
// MACROS 
// --------------------------------------------

#define ON_BUZZER(pin) digitalWrite(pin, HIGH);
#define OFF_BUZZER(pin) digitalWrite(pin, LOW)

mBuzzer::mBuzzer(const int pin) : buzzerPin(pin)
{
    _last_ms = 0;
    _onTime_ms = 0;
    _offTime_ms = 0;
    _beepsRemaining = 0;
    state = BUZZER_ST_IDLE;
    _actualState = &mBuzzer::stIdle;
}

void mBuzzer::init() {
    pinMode(buzzerPin, OUTPUT);
    OFF_BUZZER(buzzerPin);
    
    state = BUZZER_ST_IDLE;
    _actualState = &mBuzzer::stIdle;
    _beepsRemaining = 0;
}



// ------------------------------------------
// MÁQUINA DE ESTADOS
// ------------------------------------------


void mBuzzer::stIdle(void) {
    // En reposo no hace nada, espera a que se cargue una secuencia.
}

void mBuzzer::stOn (void){


    // Timer soft de ON. Vencido el timer, apaga el buzzer. 
    // Sólo activa la lógica si entró al estado por secuencia y no forzado 
    if(_reloadedSequence && (millis() - _last_ms > _onTime_ms)){
        OFF_BUZZER(buzzerPin);

        _last_ms = millis();    // Actualzación del timer de wait
        _beepsRemaining--;      // Descuenta el beep que acaba de sonar

        // Aún quedan repeticiones, pasa al estado de pausa WAIT
        if (_beepsRemaining > 0) {
            state = BUZZER_ST_WAIT;
            _actualState = &mBuzzer::stWait;
        } 
        // Terminó la secuencia
        else {
            state = BUZZER_ST_IDLE;
            _actualState = &mBuzzer::stIdle;

            _reloadedSequence = false; // Resetea la secuencia
        }
    }
}

void mBuzzer::stWait(void) {
    
    // Timer soft de WAIT. Vencido el timer, prende el buzzer
    // No revisa la reloadSequence pues sólo entra a este estado por secuencia 
    if (millis() - _last_ms >= _offTime_ms) {
        ON_BUZZER(buzzerPin);
        _last_ms = millis(); // Actualización del timer para ON
        
        // Vuelve a prender para el siguiente beep
        state = BUZZER_ST_ON;
        _actualState = &mBuzzer::stOn;
    }
}





// ------------------------------------------
// CONTROL MANUAL Y SECUENCIAS
// ------------------------------------------

void mBuzzer::startSequence(uint8_t repetitions, uint32_t onTime, uint32_t offTime) {
    // Se podria agregar un if(_reloadedSequence) {return;}
    // o para garantizar la urgencia, que sólo si la secuencia es del tipo ERROR, o por prioridad si usamos valores numéricos
    // (cambiar la firma del método) 
    // Para sobrescribir las secuencias en caso de que haga múltiples llamadas
    // Para más complejida, se podría agregar una pequeña cola de secuencias. (cola de eventos)
    
    _beepsRemaining = repetitions;
    _onTime_ms = onTime;
    _offTime_ms = offTime;
    _last_ms = millis();
    
    ON_BUZZER(buzzerPin);
    state = BUZZER_ST_ON;
    _reloadedSequence = true; // Secuencia activada

    _actualState = &mBuzzer::stOn;
}

void mBuzzer::on() {
    _beepsRemaining = 0; // Cancela cualquier secuencia activa
    ON_BUZZER(buzzerPin);
    _reloadedSequence = false; // Fuerza la anulación de cualquier secuencia

    state = BUZZER_ST_ON; 
    _actualState = &mBuzzer::stOn; 
}

void mBuzzer::off() {
    _beepsRemaining = 0; 
    OFF_BUZZER(buzzerPin);

    state = BUZZER_ST_IDLE;
    _actualState = &mBuzzer::stIdle;
}

void mBuzzer::toggle() {
    if (digitalRead(buzzerPin) == LOW) {
        on();
    } else {
        off();
    }
}

void mBuzzer::beep(const uint32_t durationMs) {
    // 1 repetición, tiempo de ON dinámico, 0 ms de pausa (irrelevante para N=1)
    startSequence(1, durationMs, 0);
}

// Podria haber un bool para verificar si se realizó la carga de la secuencia
void mBuzzer::playSuccess() {
    // N = 2 | ON = 100ms | OFF = 50ms
    startSequence(2, 100, 50); // tampoco funciona en N=4
}

void mBuzzer::playError() {
    // N = 2 | ON = 500ms | OFF = 100ms
    startSequence(2, 500, 100);
}