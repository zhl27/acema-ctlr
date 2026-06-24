#ifndef UBX_PROTOCOLS_H
#define UBX_PROTOCOLS_H


/**
 * SE DEBE CAMBIAR LA ARQUITECTURA DE LAS DECLARACIONES. ESTÁ TODO SOLUCIONADO CON ALAMBRE 
 * Y CASTEOS ESTÁTICOS. EL CONFICTO VIENE QUE AL SER SÓLO ENUM, HAY "NOMBRES" QUE SE REPITEN Y
 * NO COMPILA. USÉ ENNUM CLASS PARA QUE ESTÉN FUERTEMENTE TIPADOS PERO LAS FIRMAS DE LOS MÉTODOS
 * ESPERABAN UN UINT8_T, DE ALLÍ LOS CASTEOS. 
 * 
 * EL GROSO PROBLEMA ES CON EL ID. AHÍ CREO QUE SE DEBE USAR SPACENAME PARA PODER DECLARAR
 * OTROS ID. POR AHORA SÓLO USO CASTEO ESTÁTICOS
 *   
 */

#include <cstdint>

/**
 * @struct UbxRegistroMensajes
 */
typedef struct UBX_REGISTRO_MENSAJES
{
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t* payloadBuffer;
    uint16_t length;
    void (*onReceive)(void*);
} UbxRegMsg_t;


// ==========================================
// Protocolos class
// ==========================================
typedef enum class UBX_SYNC: uint8_t {
    SYNC_1 = 0xB5,
    SYNC_2 = 0x62
}ubx_sync_e;


typedef enum class UBX_CLASS: uint8_t {
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
typedef enum class UBX_ID_NAV: uint8_t{
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


#pragma pack(push, 1)


// ==========================================
// ID's de la clase CGF (CONFIGURATION)
// ==========================================
typedef enum class UBX_ID_CFG: uint8_t{
    ANT =   0X13,
    CFG =   0X09,
    DAT =   0X06,
    GNSS =  0X3E,
    INF =   0X02,
    ITFM =  0X39,
    LOGFILTER = 0X47,
    MSG =   0X01,
    NAV5 =  0X24,
    NAVX5 = 0X23,
    NMEA =  0X17,
    PM2 =   0X3B,
    PRT =   0X00,
    RATE =  0X08,
    RINV =  0X34,
    RST =   0X04,
    RXM =   0X11,
    SBAS =  0X16,
    TP5 =   0X31,
    USB =   0X1B
}ubx_id_cfg_e;

// ==========================================
// Payload para UBX-CFG-PRT (Configuración de Puerto UART)
// ==========================================
typedef struct {
    uint8_t  portID;         // Usualmente 1 para UART1
    uint8_t  reserved1;
    
    union 
    {
        struct
        {
            uint16_t en:1;
            uint16_t pol:1;
            uint16_t pin:5;
            uint16_t thres:9;
        }bitfield;
        uint16_t txReady;
    }txReady;                // Configuración del pin TX Ready (0 = inactivo)
    
    union{
        struct
        {
            uint32_t reserved0:4;
            //
            uint32_t reserved1:2;
            uint32_t charLen:2;
            //
            uint32_t reserved2:1;
            uint32_t parity:3;
            //
            uint32_t nStopBits:2;
            uint32_t reserved3:18;
        }bitfield;
        uint32_t mode;           // Configuración UART (Ej: 0x08D0 = 8N1)
    }mode;

    uint32_t baudRate;       // Baudios (Ej: 115200)

    union {
        struct 
        {
            uint16_t inUbx:1;
            uint16_t inNmea:1;
            uint16_t inRtcm:1;
            uint16_t reserved0:13;
        }bitfield;
        uint16_t inProtoMask;    // Máscara entrada (Bit 0 = UBX, Bit 1 = NMEA)
    } inProtoMask;

    union {
        struct 
        {
            uint16_t outUbx:1;
            uint16_t outNmea:1;
            uint16_t reserved0:14;
        }bitfield;
        uint16_t outProtoMask;   // Máscara salida  (Bit 0 = UBX, Bit 1 = NMEA)
    } outProtoMask;

    union{
        struct 
        {
            uint16_t reserved0:1;
            uint16_t extendedTxTimeout:1;
            uint16_t reserved1:14;
        }bitfield;
        uint16_t flags;          // Flags de timeout
    } flags;
    uint16_t reserved2;
} cfg_prt_uart_t;

// ==========================================
// Payload para UBX-CFG-RATE (Tasa de actualización)
// ==========================================
typedef struct {
    uint16_t measRate;       // Periodo de medición en milisegundos (Ej: 100ms = 10Hz)
    uint16_t navRate;        // Ciclos de medición por ciclo de navegación (usualmente 1)
    uint16_t timeRef;        // 0 = UTC, 1 = GPS Time
} cfg_rate_t;

// ==========================================
// Payload para UBX-CFG-MSG (Habilitar mensajes)
// ==========================================
typedef struct {
    uint8_t msgClass;        // Clase del mensaje a habilitar (Ej: 0x01 para NAV)
    uint8_t msgID;           // ID del mensaje (Ej: 0x07 para PVT)
    uint8_t rate;            // Tasa de envío (1 = una vez por ciclo de navegación)
} cfg_msg_t;

// ==========================================
// Payload para UBX-CFG-NAV5 (Filtro de Kalman / Modelo Dinámico)
// ==========================================
// Modelos dinámicos útiles: 
//     Dynamic Platform model:
typedef enum class NAV5_DYN_MODEL: uint8_t{
    Portable =      0,
    Stationary =    2,
    Pedestrian =    3,
    Automotive =    4,
    Sea = 5,
    Airborne_1G =      6,      // with <1g Acceleration
    Airborne_2G =      7,      // with <2g Acceleration
    Airborne_4G =      8       // with <4g Acceleration
}nav_dyn_model_e;
// 0 = Portable, 2 = Estacionario, 3 = Peatón, 4 = Automotriz (Robot de piso), 
// 6 = Airborne < 1G, 7 = Airborne < 2G, 8 = Airborne < 4G (Cohetes/Alta dinámica)
typedef struct {
    uint16_t mask;           // Máscara de parámetros a aplicar (0x0001 = cambiar dynModel)
    uint8_t  dynModel;       // Modelo dinámico seleccionado
    uint8_t  fixMode;        // 1=2D only, 2=3D only, 3=Auto 2D/3D
    int32_t  fixedAlt;       // Altitud fija para modo 2D
    uint32_t fixedAltVar;    // Varianza de altitud
    int8_t   minElev;        // Elevación mínima del satélite (grados)
    uint8_t  drLimit;        // Límite de Dead Reckoning
    uint16_t pDop;           // Máscara de PDOP
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t  staticHoldThresh;
    uint8_t  dgpsTimeOut;
    uint8_t  cnoThreshNumSVs;
    uint8_t  cnoThresh;
    uint16_t pAccExt;
    uint16_t tAccExt;
    uint8_t  reserved[16];
} cfg_nav5_t;

#pragma pack(pop)



#endif