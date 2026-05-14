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

typedef struct {
    int dato[8];
}dataPlot_t;

union pay_u {
    char msg[SIZE_BUFFER_MSG];
    dataPlot_t data;
};

enum Protocolo: uint8_t {
    // CPU --> GSE
    PLOT    = 0X01,
    MGS     = 0X02,
    ERR     = 0X03,
    PING    = 0X04,

    // GSE --> CPU
    START   = 0X10,
    END     = 0X20,
    PONG    = 0X30
};


class LoraWrapped
{
private:
    int _syncWord;
    char _encryptWord;
    pay_u _internalPayload;

    bool send_package(pkt_t *ptrPkt);
    bool read_package(pkt_t *ptrPkt);
    /* Cifrado de un byte con el método XOR*/
    inline char encrypt_byte(char dat) {return  ((char)dat)^_encryptWord ;};
public:
    LoraWrapped(int ss, int reset, int dio0, SPIClass& spi = DEFAULT_SPI_LORA);
    ~LoraWrapped();
    bool begin(int sw = DEFAULT_SYNC_WORD, char ew = DEFAULT_ENCRY_WORD, long frequency = DEFAULT_FREC);
    bool connect_to_GSE();
    bool connection_isAccept();
    bool accept_connection();

    bool connection_isAccepted();

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
};


#endif /* LORAWRAPPED_H */
