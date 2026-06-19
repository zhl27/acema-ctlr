#ifndef UBX_PROTOCOLS_H
#define UBX_PROTOCOLS_H

#include <cstdint>


// ==========================================
// Protocolos class
// ==========================================
typedef enum UBX_CLASS: uint8_t {
    NAV = 0X01,
    RXM = 0X02,
    INF = 0X04,
    ACK = 0X05,
    CFG = 0X06,
    MON = 0X0A,
    AID = 0X0B,
    TIM = 0X0D,
    LOG = 0X21
}ubx_class_e;


// ==========================================
// ID's de la clase NAV (Navigation)
// ==========================================
typedef enum UBX_ID_NAV: uint8_t{
    POSLLH = 0X02,
    PVT = 0x07,
    SBAS = 0X32
}ubx_id_nav_e;

//Fuerza al compilador a empaquetar estructuras estrechamente con una alineación de 1 byte
#pragma pack(push, 1)
// ==========================================
// Estructura NAV-PVT Payload (84 Bytes)
// ==========================================
/** 
 * @details Conversion a datos reales:
 *  -   float true_lon = packet.lon * 1e-7f;
 *  -   float true_lat = packet.lat * 1e-7f;
 *  -   float true_heading = packet.heading * 1e-5f;
 *  -   float true_pdop = packet.pDOP * 0.01f;
 *  -   float meters = packet.hMSL / 1000.0f;
*/
typedef struct NAV_PVT {
    // Byte Offset: 0
    uint32_t iTOW;          // Tiempo GPS de la semana (ms)
    uint16_t year;          // Año (UTC)
    uint8_t  month;         // Mes, Rango 1..12 (UTC)
    uint8_t  day;           // Dia del mes, rango 1..31 (UTC)
    uint8_t  hour;          // Hora del dia, rango 0..23 (UTC)
    uint8_t  min;           // Minuto de la hora, rango 0..59 (UTC)
    uint8_t  sec;           // Segundos del minuto, rango 0..60 (UTC)
    
    // Byte Offset: 11 (Campo de bits Valid)
    union {
        struct {
            uint8_t validDate:1;        // 1 = Fecha UTC válida
            uint8_t validTime:1;        // 1 = Hora del día UTC válida
            uint8_t fullyResolved:1;    // 1 = Hora del día UTC completamente resuelta
            uint8_t reserved:5;
        } bits;
        uint8_t word;
    } valid;
    
    uint32_t tAcc;          // Estimación de precisión de tiempo (ns)
    int32_t  nano;          // Fracción de segundo (ns), rango -1e9 .. 1e9
    uint8_t  fixType;       // Tipo de arreglo GNSS (0..5)
    
    // Byte Offset: 21 (Campo de bits Flags)
    union {
        struct {
            uint8_t gnssFixOK:1;        // 1 = Arreglo Válido
            uint8_t diffSoln:1;         // 1 = Correcciones diferenciales aplicadas
            uint8_t psmState:3;         // Estado del modo de ahorro de energía
            uint8_t reserved:3;
        } bits;
        uint8_t word;
    } flags;
    
    uint8_t  reserved1;     // Byte Offset: 22
    uint8_t  numSV;         // Byte Offset: 23 - Número de satelites usados
    int32_t  lon;           // Byte Offset: 24 - Longitud (Escala: 1e-7 grados)
    int32_t  lat;           // Byte Offset: 28 - Latitud (Escala: 1e-7 grados)
    int32_t  height;        // Byte Offset: 32 - Altura sobre el elipsoide (mm)
    int32_t  hMSL;          // Byte Offset: 36 - Altura sobre el nivel medio del mar (mm)
    uint32_t hAcc;          // Byte Offset: 40 - Estimación de precisión horizontal (mm)
    uint32_t vAcc;          // Byte Offset: 44 - Estimación de precisión vertical (mm)
    int32_t  velN;          // Byte Offset: 48 - Velocidad norte NED (mm/s)
    int32_t  velE;          // Byte Offset: 52 - Velocidad este NED (mm/s)
    int32_t  velD;          // Byte Offset: 56 - Velocidad de descenso NED (mm/s)
    int32_t  gSpeed;        // Byte Offset: 60 - Rapidez de avance 2D (mm/s)
    int32_t  heading;       // Byte Offset: 64 - Rumbo del movimiento 2D (Escala: 1e-5 grados)
    uint32_t sAcc;          // Byte Offset: 68 - Estimación de precisión de rapidez (mm/s)
    uint32_t headingAcc;    // Byte Offset: 72 - Estimación de la precisión del rumbo (escala: 1e-5 grados)
    uint16_t pDOP;          // Byte Offset: 76 - Posición DOP (Escala: 0,01)
    uint16_t reserved2;     // Byte Offset: 78
    uint32_t reserved3;     // Byte Offset: 80
} nav_pvt_t;

#pragma pack(pop)

// Comprobación de integridad en tiempo de compilación para verificar que la carga útil sea exactamente de 84 bytes por especificación
static_assert(sizeof(nav_pvt_t) == 84, "Size of nav_pvt_t must be exactly 84 bytes!");




/**
 * @struct UbxRegistroMensajes
 */
typedef struct UBX_REGISTRO_MENSAJES
{
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t* payloadBuffer;
    uint16_t length;
    void (*onReceive)();
} UbxRegMsg_t;

#endif