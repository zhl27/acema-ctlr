//
// Created by lucaz on 11/7/2026.
//

#include "Persistence.h"


// TODO: PERSISTENCE INCLUYE A mSD y mFLash

// TODO: Tanto mSD como Camera utilizan a eEsp32Cam. Debo inicializar a eEsp32Cam por fuera de estos services para que puedan utilizar la misma instancia. Ninguna de las dos debería instanciarla.

mFlash Persistence::_flash; // esto podría recibir como constructor del mFlash los pines que utiliza

bool Persistence::init(mSD& sd) { // el ampersand evita que se pasen nullptrs
    _sd = &sd; // espCam es la instancia única de eEsp32Cam que service Camera también utiliza

    // TODO: tengo un pedo mental ahora mismo con respecto a la diferencia entre usar punteros o no para estas variables. Entiendo que las variables que guardan punteros son referencias, simplemente eso. Pero las asignaciones "a secas" qué son? preguntas que no me dejan dormir por las noches
    _sd->init();
    _flash.init(); // entiendo que está bien que sea una asignacion "a secas", ya que se inicializa EN la variable Persistence::_flash.

    return true;
}