#include "mFlash.h"
#include "data.h"
#include <freertos/ringbuf.h>



// ============================================================
//DEFINICIONES A NO IMPLEMENTAR PARA QUE NO HAYA ERRORES

// SI IMPLEMNTAR EN LAS STRUCTS REALES LAS DIRECTIVAS DE COMPLAODR
// Datos de prevuelo / Estación terrena (Máx 4096 bytes para el Sector 0)
struct __attribute__((__packed__)) ConfigDatos {
    uint32_t id_mision;
    float sensor_offset_x;
    float sensor_offset_y;
    float alfa_filtro;              // Coeficiente para filtrado
    float altitud_base_referencia;  // CRÍTICO: Presión o altitud nivel del suelo
    bool mision_activa;             // false = En tierra/Test, true = Vuelo armado/En curso
    char estado_calibracion[10]; 
};

// Estructura de los datos filtrados que vas a loggear en ráfaga
struct __attribute__((__packed__)) LogDatos {
    uint32_t timestamp;
    float valor_filtrado;
    float valor_crudo;
};

enum estados {ESTADO_VUELO, ESTADO_PREVUELO};
int estadoActual;
float leerSensorBarometrico();
// NO IMPLEMENTAR

// DECLARACIÓN REPETIDA, NOMAS PARA QUE NO SALTE EL ERROR. YA EXISTE EN OTRA RAMA
struct CmdResult{
    uint8_t status;
    float data;
};


enum comandosGlobale {
    CMD_ON_PIRO,
    CMD_OFF_PIRO,
    CMD_DISPARAR_PIRO,
    CMD_DEPLEGAR_DROGE,
    CMD_CLEAR_LOG,         // <----- AGREGAR
    CMD_DUMP_DATA          // <----- AGREGAR
};

// ============================================================

//TODO: IMPLEMENTAR EN MAIN u DONDE CORRESPONDA



// TODO: En la implementación real CAMBIAR LogDatos por la estructura utilizada
CmdResult cmd_clear_log(float value, void* context){
    CmdResult res = {};
    bool ok = false;
    mFlash* f = static_cast<mFlash*>(context);
    f->resetearLog();
    res.data = 0.0; res.status = 1;
    return res;
}

CmdResult cmd_dupm_data(float value, void* context){
    bool ok = false;
    mFlash* f = static_cast<mFlash*>(context);
    uint32_t total = f->getCantidadRegistros(sizeof(LogDatos));
    CmdResult res;

    Serial.printf("START_DUMP");
    
    for (uint32_t i = 0; i < total; i++) {
        LogDatos punto;
        ok = f->leerPuntoLog(i, (void*)&punto, sizeof(LogDatos) );
        if(ok) {
            // Imprime en formato CSV listo para copiar, pegar y graficar
            Serial.printf("%d,%.2f,%.2f\n", punto.timestamp, punto.valor_filtrado, punto.valor_crudo);
            // MODIFICAR SEGÚN LA STRUC
            // O ENCAPSULAR PARA QUE LA ESP DEL GSE PUEDA USARLO TAMBIEN
        }
    }
    Serial.printf("END_DUMP");

    

    res.status = ok ? 1 : 0;
    res.data = 0.0f; // No hay dato que devolver
    
    return res;
}

// ===================================================
// SUGERENCIA PARA GUARDAR CAMBIOS EN LA MEMORIA

ConfigDatos configActual; // <--- DEBE SER GLOBA U EN UNA STRUCT GLOBAL (SYSTEM)

// Handlers que SOLO modifican la RAM
CmdResult cmd_set_offset_x(float value, void* context) {
    configActual.sensor_offset_x = value;
    return {1, value};
}

CmdResult cmd_set_offset_y(float value, void* context) {
    configActual.sensor_offset_y = value;
    return {1, value};
}

// Handler dedicado exclusivamente a sincronizar RAM -> Flash
CmdResult cmd_commit_config(float value, void* context) {
    mFlash* flash = static_cast<mFlash*>(context);
    
    // Guardamos toda la estructura de una sola vez
    flash->guardarConfig(&configActual, sizeof(configActual));
    
    return {1, 0.0f}; // Retornamos OK a la estación terrena
}

void setup() {
    // ... inicialización ...
//    cmdDispatcher.registerCommand(CMD_SET_OFFSET_X, cmd_set_offset_x, nullptr);
//    cmdDispatcher.registerCommand(CMD_SET_OFFSET_Y, cmd_set_offset_y, nullptr);
//    cmdDispatcher.registerCommand(CMD_COMMIT_CONFIG, cmd_commit_config, &memFlash);
}
/* Secuencia en tierra: El operador ajusta todo lo que necesita.
 * Cuando el software de la PC muestra que todo está en orden, 
 * se aprieta un botón "Guardar en Cohete" 
 * que envía el CMD_COMMIT_CONFIG.
 */
// =======================================


// Instancia del manager
mFlash memFlash(5); 

// EJEMPLO DE LÓGICA DE INICIALIZACIÓN PARA LA MÁQUINA DE STADOS
void setup() {
    memFlash.begin(sizeof(LogDatos));

    ConfigDatos config;
    memFlash.cargarConfig(&config, sizeof(config));


    // cmdDispatcher.registerCommand(CMD_CLEAR_LOG, cmd_clear_log, &memFlash);
    // cmdDispatcher.registerCommand(CMD_DUMP_DATA, cmd_dump_dat, &memFlash);

    // Se esperaría en algun momento, limpiar el logger antes de despegar,
    // mas no, que sea condición para el depegue
    // LOGICA DE MAQUINA DE ESTADOS: Esto puede ir en la FSM para prevenir los reinicios
    // INTERLOCK 1: ¿La misión había sido armada/iniciada antes del reinicio?
    if (config.mision_activa == true) {
        
        // INTERLOCK 2: Validación por sensores
        // Comparamos la altitud actual con la altitud base que guardamos en prevuelo
        float altitud_actual = leerSensorBarometrico(); 
        
        if ((altitud_actual - config.altitud_base_referencia) > 20.0) {
            // ¡ESTAMOS EN EL AIRE REALMENTE! Recuperando vuelo.
            estadoActual = ESTADO_VUELO;
        } else {
            // Falsa alarma. Se armó, pero nunca despegó (o ya aterrizó).
            estadoActual = ESTADO_PREVUELO;
        }
        
    } else {
        // No hay misión activa. Arranque normal.
        estadoActual = ESTADO_PREVUELO;
    }
}

void take(){
    memFlash.begin(sizeof(LogDatos));

    // --- Ejemplo de Escritura en Prevuelo ---
    ConfigDatos miseteo = { 101, -0.02, 0.05, 0.15};
    memFlash.guardarConfig((void*)&miseteo, sizeof(miseteo));

    // --- Ejemplo de Loggeo en Bucle de Vuelo ---
    LogDatos nuevoPunto = { millis(), 12.45, 12.80 };
    memFlash.guardarPuntoLog(&nuevoPunto, sizeof(nuevoPunto));

    // --- Ejemplo de volcado (Dump) post-aterrizaje ---
    uint32_t total = memFlash.getCantidadRegistros(sizeof(LogDatos));
    for (uint32_t i = 0; i < total; i++) {
        LogDatos puntoLeido;
        if (memFlash.leerPuntoLog(i, &puntoLeido, sizeof(LogDatos))) {
            Serial.printf("%d,%.2f,%.2f\n", puntoLeido.timestamp, puntoLeido.valor_filtrado, puntoLeido.valor_crudo);
        }

    }

}

// no implementar para el error
RingbufHandle_t xFlashRingbuf;

// SI MODIFICAR ESTO
/*
xTaskCreatePinnedToCore(
    vTaskLora, 
    "Lora", 
    4096, 
    &memFlash, // <<<------ AGREGAR ESTOOOOOOO 
    4, 
    &(Cohete::SYSTEM.procesos.xTaskLoraHandle),
    0
);
*/

void vTaskFlash(void *pvParameters) {

    mFlash* f = static_cast<mFlash*>(pvParameters);
    while (true) {

        ESP_LOGD(TAG_TASK_FLASH, "Core ID: %d", xPortGetCoreID());

        size_t item_size = 0;
        void *item = xRingbufferReceive(xFlashRingbuf, &item_size, pdMS_TO_TICKS(10));

        if (item != NULL) {
            if (item_size == sizeof(data_all_t)) {

                data_all_t *datos_sensores = static_cast<data_all_t *>(item);
                f->guardarPuntoLog((void*)datos_sensores,sizeof(data_all_t));

                // Print a specific member of the struct (like elapsed_time) instead of %s
                // Serial.printf("[Flash] Persisting (%d bytes). Time: %lu\n", static_cast<int>(item_size), micros());
                // SerialPrint::plot("contadorFlash", contadorFlash);
                // contadorFlash++;
                // TODO: In a real implementation, write 'datos' to SD/flash.
            }
            vRingbufferReturnItem(xFlashRingbuf, item);
        } else {
            ESP_LOGI(TAG_TASK_FLASH, "No items to persist (timeout)");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}