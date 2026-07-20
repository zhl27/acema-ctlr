// Created by zhl on 6/7/26.
// Modificado y ampliado para cobertura completa: 19/7/2026
//

#include <Arduino.h>
#include <unity.h>
#include "mFlash.h"
#include <SPI.h>
#include "config.h"
#include "LoraWrapped.h"
LoraWrapped _lora(ConfigInit::LORA_CS, ConfigInit::LORA_RST, ConfigInit::LORA_DIO0, ConfigInit::LORA_DIO1, SPI);

// --- ESTRUCTURAS DE PRUEBA (Representan los datos reales del cohete) ---
struct ConfigPreVuelo {
    uint32_t idCohete;
    float presionNivelMar;
    uint8_t loraCanal;
    char claveCifrado[4];
};

struct RegistroTelemetria {
    uint32_t timestamp; // El token de 4 bytes que usa Etapa 1 y 2
    float altitud;
    float aceleracionZ;
    uint8_t estadoFSM;
};


// Pines SPI compartidos de la placa
constexpr uint8_t SPI_SCK   = 18;
constexpr uint8_t SPI_MISO  = 19;
constexpr uint8_t SPI_MOSI  = 23;
constexpr uint8_t SD_CS     = 15; // Para silenciar la SD durante el test

void setUp(void) {
    // Preparación antes de cada test si fuera necesario
}

void tearDown(void) {
    // Limpieza después de cada test si fuera necesario
}

// Lo declaramos como puntero para controlar EXACTAMENTE cuándo se inicializa
mFlash* flashTest = nullptr;




// =========================================================================
// TEST 1: Inicialización básica y verificación del bus
// =========================================================================
void test_mflash_inicializacion(void) {
    // Inicializamos el objeto dinámicamente aquí, con el hardware ya despierto
    flashTest = new mFlash(ConfigInit::FLASH_CS);
    Serial.printf("-> flash diamico creado en %x",flashTest );
    bool resultado = flashTest->begin(sizeof(RegistroTelemetria));
    TEST_ASSERT_TRUE_MESSAGE(resultado, "El chip físico W25Q128 no respondió al begin()");
    TEST_ASSERT_EQUAL(HIGH, digitalRead(ConfigInit::FLASH_CS));
}

// =========================================================================
// TEST 2: Almacenamiento y Recuperación de Configuración (Sector 0)
// =========================================================================
void test_mflash_configuracion_prevuelo(void) {
    ConfigPreVuelo configOriginal = {1042, 1013.25f, 7, {'A', 'C', 'E', 'M'}};
    ConfigPreVuelo configLeida;
    memset(&configLeida, 0, sizeof(ConfigPreVuelo)); // Limpiamos buffer

    // Guardar en el Sector 0[cite: 7, 8]
    flashTest->guardarConfig(&configOriginal, sizeof(ConfigPreVuelo));

    // Cargar del Sector 0[cite: 7, 8]
    bool lecturaOk = flashTest->cargarConfig(&configLeida, sizeof(ConfigPreVuelo));

    TEST_ASSERT_TRUE_MESSAGE(lecturaOk, "Fallo al leer la sección de configuración");
    TEST_ASSERT_EQUAL_UINT32(configOriginal.idCohete, configLeida.idCohete);
    TEST_ASSERT_EQUAL_FLOAT(configOriginal.presionNivelMar, configLeida.presionNivelMar);
    TEST_ASSERT_EQUAL_UINT8(configOriginal.loraCanal, configLeida.loraCanal);
    TEST_ASSERT_EQUAL_INT8_ARRAY(configOriginal.claveCifrado, configLeida.claveCifrado, 4);
}

// =========================================================================
// TEST 3: Flujo de Telemetría (Escritura, conteo y lectura secuencial)
// =========================================================================
void test_mflash_ciclo_telemetria(void) {
    // Aseguramos que el log empiece limpio en este test[cite: 7]
    flashTest->resetearLog();
    TEST_ASSERT_EQUAL_UINT32(0, flashTest->getCantidadRegistros(sizeof(RegistroTelemetria)));

    // Creamos 3 puntos de datos distintos
    RegistroTelemetria p1 = {1000, 0.0f, 1.0f, 0};   // En rampa
    RegistroTelemetria p2 = {2000, 150.5f, 4.2f, 1}; // Propulsado
    RegistroTelemetria p3 = {3000, 850.2f, -0.5f, 2};// Apogeo

    // Escribimos secuencialmente[cite: 7]
    TEST_ASSERT_TRUE(flashTest->guardarPuntoLog(&p1, sizeof(RegistroTelemetria)));
    TEST_ASSERT_TRUE(flashTest->guardarPuntoLog(&p2, sizeof(RegistroTelemetria)));
    TEST_ASSERT_TRUE(flashTest->guardarPuntoLog(&p3, sizeof(RegistroTelemetria)));

    // Validamos que el contador dinámico marque exactamente 3 registros[cite: 7, 8]
    TEST_ASSERT_EQUAL_UINT32(3, flashTest->getCantidadRegistros(sizeof(RegistroTelemetria)));

    // Leemos y comparamos el registro intermedio (Índice 1 -> p2)[cite: 7, 8]
    RegistroTelemetria clonP2;
    bool lecturaOk = flashTest->leerPuntoLog(1, &clonP2, sizeof(RegistroTelemetria));
    
    TEST_ASSERT_TRUE_MESSAGE(lecturaOk, "No se pudo leer el índice 1 del log");
    TEST_ASSERT_EQUAL_UINT32(p2.timestamp, clonP2.timestamp);
    TEST_ASSERT_EQUAL_FLOAT(p2.altitud, clonP2.altitud); // Ajustado a la propiedad de tu struct
    TEST_ASSERT_EQUAL_UINT8(p2.estadoFSM, clonP2.estadoFSM);
}

// =========================================================================
// TEST 4: Protecciones contra lectura fuera de límites (Out of Bounds)
// =========================================================================
void test_mflash_limites_lectura(void) {
    RegistroTelemetria bufferBasura;
    
    // Intentar leer el índice 99 cuando solo guardamos 3 registros en el test anterior[cite: 7, 8]
    // El método leerPuntoLog debe retornar false al validar targetAddr + len > _currentLogAddr[cite: 8]
    bool resultadoOB = flashTest->leerPuntoLog(99, &bufferBasura, sizeof(RegistroTelemetria));
    
    TEST_ASSERT_FALSE_MESSAGE(resultadoOB, "Error de seguridad: La clase permitió leer memoria no escrita");
}

// =========================================================================
// TEST 5: Borrado parcial y selectivo del log
// =========================================================================
void test_mflash_reset_log(void) {
    // El log tiene datos del test anterior. Ejecutamos el resetearLog[cite: 7]
    flashTest->resetearLog();

    // Verificamos que la dirección haya vuelto a ADDR_LOG[cite: 8]
    // y por ende la cantidad de registros calculada sea 0[cite: 7, 8]
    uint32_t registrosPostReset = flashTest->getCantidadRegistros(sizeof(RegistroTelemetria));
    TEST_ASSERT_EQUAL_UINT32(0, registrosPostReset);

    // Intentar leer el índice 0 ahora debe fallar inmediatamente[cite: 7, 8]
    RegistroTelemetria buffer;
    TEST_ASSERT_FALSE(flashTest->leerPuntoLog(0, &buffer, sizeof(RegistroTelemetria)));
}

// =========================================================================
// TEST 6: Simulación de recuperación post-cuelgue (Búsqueda Coarse/Fine)
// =========================================================================
void test_mflash_recuperacion_memoria(void) {
    // 1. Escribimos un registro de telemetría válido para simular datos previos en la Flash
    RegistroTelemetria puntoPreCuelgue = {5555, 1200.0f, 0.0f, 3};
    flashTest->guardarPuntoLog(&puntoPreCuelgue, sizeof(RegistroTelemetria));

    // 2. Destruimos el estado actual en RAM volviendo a correr el `begin` 
    // Esto fuerza a las etapas 1 y 2 a escanear los sectores físicos desde cero[cite: 8]
    bool inicializacionPostCuelgue = flashTest->begin(sizeof(RegistroTelemetria));
    TEST_ASSERT_TRUE(inicializacionPostCuelgue);

    // 3. Tu lógica de begin() ante un reinicio con datos redondea la dirección al siguiente sector de 4KB[cite: 8]
    // Validemos que la clase sigue operativa y permite escribir en el nuevo sector alineado sin romperse[cite: 7, 8]
    RegistroTelemetria puntoNuevo = {6666, 1205.0f, -0.1f, 3};
    bool escrituraPostCuelgueOk = flashTest->guardarPuntoLog(&puntoNuevo, sizeof(RegistroTelemetria));
    
    TEST_ASSERT_TRUE_MESSAGE(escrituraPostCuelgueOk, "La alineación por software posterior al cuelgue bloqueó la escritura");
}

// =========================================================================
// Configuración y Secuencia de Ejecución de PlatformIO
// =========================================================================
void setup() {
// 1. ARRANCAR SERIAL DE INMEDIATO
    Serial.begin(115200);
    delay(2000); 
    Serial.println("--- [ESP32 DESPIERTA] Iniciando entorno de pruebas ---");

    // 2. Limpiar el bus SPI antes de tocar cualquier librería
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH); // Silenciamos la SD

    pinMode(ConfigInit::FLASH_CS, OUTPUT);
    digitalWrite(ConfigInit::FLASH_CS, HIGH); // Aseguramos reposo en la Flash

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
    Serial.println("-> Bus SPI configurado con fix -1");

    // 3. Arrancar Unity
    UNITY_BEGIN(); 
   Serial.println("-> UNITI INICIADO");
    RUN_TEST(test_mflash_inicializacion);
    RUN_TEST(test_mflash_configuracion_prevuelo);
    RUN_TEST(test_mflash_ciclo_telemetria);
    RUN_TEST(test_mflash_limites_lectura);
    RUN_TEST(test_mflash_reset_log);
    RUN_TEST(test_mflash_recuperacion_memoria);

    UNITY_END(); 
}

void loop() {
    // Sin acción, Unity termina en el setup()[cite: 9]
}