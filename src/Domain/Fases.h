//
// Created by lucaz on 20/1/2026.
//

#ifndef FASES_H
#define FASES_H

#include "GestorDeFases.h"

class Prevuelo;
class FaseImpulso;
class FaseBalistica;
class FaseFrenoAerodinamico;
class Apogeo;
class AperturaDrogue;
class DescensoRapido;
class AperturaParacaidasPrincipal;
class DescensoLento;
class Aterrizaje;

// static allocation de las fases

class Fase {
public:
    virtual void actuar(GestorDeFases& contexto) = 0;
    virtual const char* nombre() const = 0; // para el logging
};


#endif //FASES_H
