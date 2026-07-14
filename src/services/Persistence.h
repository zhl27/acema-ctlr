//
// Created by lucaz on 11/7/2026.
//

#ifndef ACEMA_CTLR_PERSISTENCE_H
#define ACEMA_CTLR_PERSISTENCE_H

#include "mSD.h"
#include "mFlash.h"

class Persistence {
private:
    Persistence() = delete;

    static mSD* _sd;
    static mFlash _flash;

public:
    static bool init(mSD& sd);

    static mSD& getSD() { return *_sd; }
    static mFlash& getFlash() { return _flash; }
};


#endif //ACEMA_CTLR_PERSISTENCE_H
