#include "CmdDispatcher.h"
#include "../EnlaceGSE/EnlaceGSE.h"

CmdDispatcher::CmdDispatcher() : _registeredCmds(0), _cmdQueue(nullptr), _taskHandle(nullptr) {}

void CmdDispatcher::init() {
    // Crear la cola RTOS
    _cmdQueue = xQueueCreate(QUEUE_SIZE, sizeof(CommandPayload));
    
    // Disparar la tarea. Ajustar prioridad según necesidad (normalmente media/alta)
    if (_cmdQueue != nullptr) {
        xTaskCreate(
            _taskWrapper, 
            "CmdTask", 
            4096,        // Stack size 
            this,        // Pasamos la instancia como parámetro
            2,           // Prioridad
            &_taskHandle
        );
    }
}

bool CmdDispatcher::registerCommand(uint32_t opCode, CmdCallback_t callback, void* context) {
    if (_registeredCmds >= MAX_COMMANDS) {
        return false; // Tabla llena
    }
    
    _lookupTable[_registeredCmds].opCode = opCode;
    _lookupTable[_registeredCmds].callback = callback;
    _lookupTable[_registeredCmds].context = context;
    _registeredCmds++;
    
    return true;
}

bool CmdDispatcher::enqueueCommand(uint32_t opCode, float value) {
    if (_cmdQueue == nullptr) return false;
    
    CommandPayload cmd = {opCode, value};
    
    // pxHigherPriorityTaskWoken no es necesario si la tarea LoRa no es una interrupción (ISR).
    // Esperamos máximo 10 ticks si la cola está llena.
    return (xQueueSend(_cmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS); 
}

void CmdDispatcher::_taskWrapper(void* pvParameters) {
    CmdDispatcher* dispatcher = static_cast<CmdDispatcher*>(pvParameters);
    dispatcher->run();
}

void CmdDispatcher::run() {
    CommandPayload receivedCmd;

    while (true) {
        if (xQueueReceive(_cmdQueue, &receivedCmd, portMAX_DELAY) == pdTRUE) {
            CmdResult result{.status = 0, .data = 0.0f};

            for (size_t i = 0; i < _registeredCmds; i++) {
                if (_lookupTable[i].opCode == receivedCmd.opCode) {
                    
                    // Ejecutar la operación y capturar el resultado
                    if(_lookupTable[i].callback){
                        CmdResult result = _lookupTable[i].callback(receivedCmd.value, _lookupTable[i].context);
                    }
                    // Enviar los resultados a la estación terrena
                    EnlaceGSE::enviarAcuseComando(receivedCmd.opCode, result.status, result.data);
                }
            }
        }
    }
}

