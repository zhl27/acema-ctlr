/*

* @details: c_: funciones o comandos que utiliza el Cohete/CPU/Copm...
			g_: Funciones o comandos que utiliza el GSE
*/ 
#ifndef LORAWRAPPED_H
#define LORAWRAPPED_H

#include <LoRa.h>


#define DEFAULT_SYNC_WORD 'J'
#define DEFAULT_ENCRY_WORD '%' 
#define DEFAULT_FREC 433E6
#define DEFAULT_SPI_LORA SPI
#define SIZE_BUFFER_MSG 128


/* Estructuras provicionales*/
typedef struct PAQUETE {
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
}dataPlot_t;

union pay_u {
    char msg[SIZE_BUFFER_MSG];
    dataPlot_t data;
};

enum Protocolo: uint8_t {
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


class LoraWrapped
{
private:
    /* estados de la conexcion, para mejorar la reconeccion, proximamente*/
    typedef enum CONNECTION_STATUS: uint8_t {DISCONNECTED = 0x00, CONNECTED} connSts_t; /*CONNECTION_LOST */
    /* Palabra de sincronización con el modulo a comunicar, evita interferencias de otro módulo*/
    int _syncWord;

    /* caracter de encriptación, en caso de interseeción espía XD*/
    char _encryptWord;

    /* Buffer interno para enviar los datos de manera segura*/
    pay_u _internalPayload_tx;
    pay_u _internalPayload_rx;

    /* Estado de la conexion*/
    connSts_t _st;

    /* Funciones de bajo nivel, des/encriptación y envio*/
    bool send_package(pkt_t *ptrPkt);
    bool read_package(pkt_t *ptrPkt);

    /* Cifrado de un byte con el método XOR*/
    inline char encrypt_byte(char dat) {return  ((char)dat)^_encryptWord ;};
public:
    LoraWrapped(int ss, int reset, int dio0, SPIClass& spi = DEFAULT_SPI_LORA);
    ~LoraWrapped();
    bool begin(int sw = DEFAULT_SYNC_WORD, char ew = DEFAULT_ENCRY_WORD, long frequency = DEFAULT_FREC);
    
    /**
     * @brief Envia una petición de conexion del cohete al GSE
     */
    bool c_connect_to_GSE();

    /**
     * @brief Verifica la conexion estable con el GSE
     */
    bool c_connection_accepted();   
    
    /** 
     *   @brief Acepta la petición de conexion del cohete
     */
    bool g_accept_connection();



    /**
     * @brief Envía datos de telemetría (struct dataPlot_t)
     */
    bool send_datos(dataPlot_t dato);

    /**
     * @brief Envía un mensaje de texto genérico (MSG)
     */
    bool send_mensaje(const char* texto);

    /**
     * @brief Envía un mensaje de error (ERR)
     */
    bool send_mensaje_error(const char* error);

    bool read_paquete(pkt_t* pPkt);
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
