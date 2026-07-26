//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_MSD_H
#define ACEMA_CTLR_MSD_H


class mSD {
private:
    eEsp32Cam _cam; // Instancia de la clase de comunicación

public:
    // Pasa los pines al constructor del driver eEsp32Cam
    mSD(int rxPin = 34, int txPin = 26, uint32_t baudRate = 115200);

    // Métodos públicos que el usuario final utilizará
    bool init();
    bool write(const String& filename, const String& data);
    String read(const String& filename);
};


#endif //ACEMA_CTLR_MSD_H
