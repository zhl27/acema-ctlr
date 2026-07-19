#ifndef CMD_DISPATCHER_H
#define CMD_DISPATCHER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* MUDADO A DATOS.H
// Estructura del payload que viajará por la cola RTOS
struct CommandPayload {
    uint32_t opCode;
    float value;
};
*/

// Estructura de respuesta de la operación
struct CmdResult {
    int8_t status;  // 1: Éxito, 0: Fallo, -1: Rechazado/No válido
    float data;     // Dato útil (opcional) de respuesta
};

// Firma del callback
typedef CmdResult (*CmdCallback_t)(float value, void* context);

// Entrada de la Lookup Table
struct CmdEntry {
    uint32_t opCode;
    CmdCallback_t callback;
    void* context; // Puntero a la instancia (ej. &sensorBmp, &piro)
};

class CmdDispatcher {
private:
    static const size_t MAX_COMMANDS = 20; // Tamaño máximo de la tabla de registro
    static const size_t QUEUE_SIZE = 10;   // Comandos máximos encolados al mismo tiempo

    CmdEntry _lookupTable[MAX_COMMANDS];
    size_t _registeredCmds;

    QueueHandle_t _cmdQueue;
    TaskHandle_t _taskHandle;

    // Tarea RTOS estática y método run interno
    static void _taskWrapper(void* pvParameters);
    void run();

public:
    explicit CmdDispatcher();

    // Inicializa la cola y lanza la tarea
    void init();

    // Registra una operación en la Lookup Table. 
    // Se llama en el setup() de tu aplicación principal.
    bool registerCommand(uint32_t opCode, CmdCallback_t callback, void* context = nullptr);

    // Encola un comando. Este es el método que usa la tarea de LoRa.
    bool enqueueCommand(uint32_t opCode, float value);
};

#endif // CMD_DISPATCHER_H
