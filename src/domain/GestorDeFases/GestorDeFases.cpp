//
// Created by lucaz on 20/1/2026.
//

#include "GestorDeFases.h"



// static allocation de las fases
static Prevuelo prevuelo;
static FaseImpulso faseImpulso;
static FaseBalistica faseBalistica;
static FaseFrenoAerodinamico faseFrenoAerodinamico;
static Apogeo apogeo;
static AperturaDrogue aperturaDrogue;
static DescensoRapido descensoRapido;
static AperturaParacaidasPrincipal aperturaParacaidasPrincipal;
static DescensoLento descensoLento;
static Aterrizaje aterrizaje;

void GestorDeFases::ejecutarCiclo() {
    this.estadoActual->actuar(*this);
}
