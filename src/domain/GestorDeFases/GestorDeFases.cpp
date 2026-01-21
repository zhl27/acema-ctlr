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

class GestorDeFases {
public:
    void setEstado(Estado& nuevo) {
        estadoActual = &nuevo;
    }

    void ejecutarCiclo() {
        estadoActual->actuar(*this);
    }

    // ===== DATOS DEL SISTEMA =====
    float altitud;
    float velocidad;
    float aceleracion;
    bool frenoAerodinamicoAbierto;
    bool drogueAbierto;
    bool paracaidasPrincipalAbierto;

    // Interfaces a hardware
    void abrirFreno()   { /* GPIO / PWM */ }
    void cerrarFreno()  { /* GPIO / PWM */ }
    void abrirDrogue()  { /* piro / servo */ }
    void abrirParacaidasPrincipal() { /* piro / servo */ }

    void logSD()        { /* escribir en SD */ }
    void enviarTelemetria() { /* radio */ }

private:
    Estado* estadoActual;
};
