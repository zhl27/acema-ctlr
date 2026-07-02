/*

* @details: c_: funciones o comandos que utiliza el Cohete/CPU/Copm...
			g_: Funciones o comandos que utiliza el GSE
*/ 
#ifndef LORAWRAPPED_H
#define LORAWRAPPED_H

#include <cstdint>

#include "LoraConfig.h"
#include <RadioLib.h>

#include "data.h"


#define DEFAULT_SPI_LORA SPI



/* Estructuras provicionales*/
typedef struct PACKET {
    uint8_t len;
    uint8_t protocole;
    void* payload;
} pkt_t;


/* PROVI VA EN GLOALS.H*/
typedef struct {
    int datoX;
    float giroX;
    float giroY;
    float altitud;
} data_plot_t;

union pay_u {
    char msg[SIZE_BUFFER_MSG];
    data_plot_t data;
};

enum lora_protocol: uint8_t {
    // CPU --> GSE
    C_PLOT    = 0X01,
    C_MGS     = 0X02,
    C_ERR     = 0X03,
    PING    = 0X04,

    // GSE --> CPU
    G_START   = 0X10,
    G_END     = 0X20,
    PONG    = 0X30
};

/* estados de la conexion, para mejorar la reconexion, proximamente*/
typedef enum CONNECTION_STATUS: uint8_t {DISCONNECTED = 0x00, CONNECTED} connSts_t; /*CONNECTION_LOST */

class LoraWrapped
{
private:
    
    // Punteros dinámicos de RadioLib según el chip
    Module* _mod;
    #if defined(MODULE_SX1278)
        SX1278* _radio;
    #elif defined(MODULE_SX1262)
        SX1262* _radio;
    #endif

    /* Palabra de sincronización con el módulo a comunicar, evita interferencias de otro módulo*/
    int _syncWord;

    /* caracter de encriptación, en caso de intersección espía XD*/
    char _encryptWord;

    /* Buffer interno para enviar los datos de manera segura*/
    pay_u _internalPayload_rx;

    /* Estado de la conexion*/
    connSts_t _st;

    /* Funciones de bajo nivel, des/encriptación y envio*/
    int _send_packet(pkt_t *ptrPkt) const;
    int _read_packet(pkt_t *ptrPkt);

    /* Cifrado de un byte con el método XOR*/
    inline char _encrypt_byte(const char dat) const {return static_cast<char>(dat)^_encryptWord ;};
    int _pinPacketReady; // Pin DIO0 o Busy de acuerdo a modelo
public:
    LoraWrapped(uint32_t nss, uint32_t rst, uint32_t pin3, uint32_t pin4, SPIClass& spi = SPI);
    ~LoraWrapped();
    int begin(int sw = DEFAULT_SYNC_WORD, char ew = DEFAULT_ENCRY_WORD, float frequency = DEFAULT_FREC);
    
    /**
     * @brief Envia una petición de conexion del cohete al GSE
     */
    int c_connect_to_GSE() const;

    /**
     * @brief Verifica la conexion estable con el GSE
     */
    int c_connection_accepted();
    
    /** 
     *   @brief Acepta la petición de conexion del cohete
     */
    int g_accept_connection();

    /**
     * @brief Envía datos de telemetría (struct dataPlot_t)
     */
    int send_data(data_all_t dato) const;

    /**
     * @brief Envía un mensaje de texto genérico (MSG)
     */
    int send_msg(const char* texto) const;

    /**
     * @brief Envía un mensaje de error (ERR)
     */
    int send_error(const char* error) const;

    /**
     * @brief Envía un PONG(ERR)
     */
    int send_pong();

    int read_packet(pkt_t* pPkt);
};


#endif /* LORAWRAPPED_H **/


/*
[ msg / plot
X Y >
x Y >
x Y >
x Y >
x   >
x   >
]


main:
------
muy rapido 5 ms no resolucion -> fiabilidad 
sensor-> cola(muuuuuuuuy) [prol, lon, dato]
------
lora.ocupado?
    NO:
	enviar segun protocolo: plot, msg <-- paquete <-- cola
end
-------
mde:
----
mde_envio
---
--
*/
