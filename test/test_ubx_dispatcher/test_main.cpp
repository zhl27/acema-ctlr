#include <unity.h>
#include <cstring>
#include <cstddef>
#include "../../lib/sensores/mGPS/src/UbxDispatcher.h"
#include "UbxProtocols.h"

// ==========================================
// Mocks y Variables Globales para Tests
// ==========================================
nav_pvt_t navPvtDestino;
bool callbackPvtEjecutado = false;

// Callback de prueba
void onNavPvtReceived() {
    callbackPvtEjecutado = true;
}

// Tabla de registros
const UbxRegMsg_t msgNavPvt = {
    UBX_CLASS::NAV, UBX_ID_NAV::PVT, (uint8_t*)&navPvtDestino, sizeof(nav_pvt_t), onNavPvtReceived
};

const UbxRegMsg_t* tablaMensajes[] = { &msgNavPvt };
const size_t sizeTabla = sizeof(tablaMensajes) / sizeof(tablaMensajes[0]);

UbxDispatcher dispatcher(tablaMensajes, sizeTabla);

// Helper para calcular e inyectar el checksum Fletcher-8 en un buffer de prueba
void calcularEInyectarChecksum(uint8_t* buffer, size_t totalLen) {
    uint8_t ckA = 0, ckB = 0;
    // Empieza desde el byte 2 (saltea 0xB5 0x62) hasta antes de los 2 bytes de CRC
    for(size_t i = 2; i < totalLen - 2; i++) {
        ckA += buffer[i];
        ckB += ckA;
    }
    buffer[totalLen - 2] = ckA;
    buffer[totalLen - 1] = ckB;
}

// ==========================================
// Setup y Teardown
// ==========================================
void setUp(void) {
    // Se ejecuta antes de cada test
    callbackPvtEjecutado = false;
    memset(&navPvtDestino, 0, sizeof(nav_pvt_t));
}

void tearDown(void) {
    // Se ejecuta después de cada test
}

// ==========================================
// Test 1: Funcionamiento Normal (Mensaje Válido)
// ==========================================
void test_dispatcher_procesa_nav_pvt_correctamente() {
    // Armamos un paquete NAV-PVT válido
    const size_t payloadLen = 84;
    const size_t packetLen = 2 + 4 + payloadLen + 2; // Sync + Header + Payload + CRC
    uint8_t bufferPaquete[packetLen];
    memset(bufferPaquete, 0, packetLen);

    // Sync & Header
    bufferPaquete[0] = 0xB5;
    bufferPaquete[1] = 0x62;
    bufferPaquete[2] = UBX_CLASS::NAV;
    bufferPaquete[3] = UBX_ID_NAV::PVT;
    bufferPaquete[4] = payloadLen & 0xFF;        // LSB
    bufferPaquete[5] = (payloadLen >> 8) & 0xFF; // MSB

    // Simulamos datos en el Payload
    nav_pvt_t payloadSimulado;
    memset(&payloadSimulado, 0, sizeof(nav_pvt_t));
    payloadSimulado.iTOW = 123456789;
    payloadSimulado.lat = -346037000; // Ej: Latitud en Buenos Aires
    payloadSimulado.lon = -583816000;
    
    // Copiamos el struct simulado al buffer de transmisión
    memcpy(&bufferPaquete[6], &payloadSimulado, sizeof(nav_pvt_t));

    // Calculamos el CRC válido
    calcularEInyectarChecksum(bufferPaquete, packetLen);

    // Simulamos la llegada de bytes por UART alimentando la FSM
    for(size_t i = 0; i < packetLen; i++) {
        dispatcher.handleFSM(bufferPaquete[i]);
    }

    // Aserciones (Verificaciones)
    TEST_ASSERT_TRUE_MESSAGE(callbackPvtEjecutado, "El callback no se ejecuto. FSM fallo o CRC invalido.");
    TEST_ASSERT_EQUAL_UINT32(123456789, navPvtDestino.iTOW);
    TEST_ASSERT_EQUAL_INT32(-346037000, navPvtDestino.lat);
    TEST_ASSERT_EQUAL_INT32(-583816000, navPvtDestino.lon);
}

// ==========================================
// Test 2: Ignorar Mensaje Desconocido
// ==========================================
void test_dispatcher_ignora_mensaje_desconocido() {
    // Armamos un paquete inventado (Ej: Class 0x99, ID 0x99)
    const size_t payloadLen = 4; // Un payload cortito de 4 bytes
    const size_t packetLen = 2 + 4 + payloadLen + 2; 
    uint8_t bufferPaquete[packetLen];

    bufferPaquete[0] = 0xB5;
    bufferPaquete[1] = 0x62;
    bufferPaquete[2] = 0x99; // Clase Inexistente
    bufferPaquete[3] = 0x99; // ID Inexistente
    bufferPaquete[4] = payloadLen & 0xFF;
    bufferPaquete[5] = (payloadLen >> 8) & 0xFF;
    
    // Payload basura
    bufferPaquete[6] = 0xAA;
    bufferPaquete[7] = 0xBB;
    bufferPaquete[8] = 0xCC;
    bufferPaquete[9] = 0xDD;

    calcularEInyectarChecksum(bufferPaquete, packetLen);

    // Alimentamos la FSM
    for(size_t i = 0; i < packetLen; i++) {
        dispatcher.handleFSM(bufferPaquete[i]);
    }

    // Aserciones
    TEST_ASSERT_FALSE_MESSAGE(callbackPvtEjecutado, "El callback se ejecuto erróneamente para un mensaje no registrado.");
    
    // Para probar que no tocó los datos de nuestra estructura válida
    TEST_ASSERT_EQUAL_UINT32(0, navPvtDestino.iTOW);
}

// ==========================================
// Main de Tests
// ==========================================
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_dispatcher_procesa_nav_pvt_correctamente);
    RUN_TEST(test_dispatcher_ignora_mensaje_desconocido);
    UNITY_END();

    return 0;
}