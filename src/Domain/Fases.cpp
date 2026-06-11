//
// Created by lucaz on 20/1/2026.
//

#include "Fases.h"
#define UMBRAL_LANZAMIENTO 10.0


class Prevuelo : public Fase {
public:
    const char* nombre() const override { return "Prevuelo"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class FaseImpulso : public Fase {
public:
    const char* nombre() const override { return "FaseImpulso"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class FaseBalistica : public Fase {
public:
    const char* nombre() const override { return "FaseBalistica"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class FaseFrenoAerodinamico : public Fase {
public:
    const char* nombre() const override { return "FaseFrenoAerodinamico"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class Apogeo : public Fase {
public:
    const char* nombre() const override { return "Apogeo"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class AperturaDrogue : public Fase {
public:
    const char* nombre() const override { return "AperturaDrogue"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};


class DescensoRapido : public Fase {
public:
    const char* nombre() const override { return "DescensoRapido"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class AperturaParacaidasPrincipal : public Fase {
public:
    const char* nombre() const override { return "AperturaParacaidasPrincipal"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class DescensoLento : public Fase {
public:
    const char* nombre() const override { return "DescensoLento"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

class Aterrizaje : public Fase {
public:
    const char* nombre() const override { return "Aterrizaje"; }
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setFase(faseImpulso);
        }
    }
};

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
