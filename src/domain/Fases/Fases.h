//
// Created by lucaz on 20/1/2026.
//

#ifndef FASES_H
#define FASES_H

class Fase {
public:
    virtual void actuar(GestorDeVuelo& contexto) = 0;
    virtual const char* nombre() const = 0; // para el logging
};

class Prevuelo : public Fase {
public:
    void actuar(GestorDeFases& ctx) override {
        ctx.logSD();
        ctx.enviarTelemetria();

        // Condición de lanzamiento: pico de aceleración
        if (ctx.aceleracion > UMBRAL_LANZAMIENTO) {
            ctx.setEstado(faseImpulso);
        }
    }
};


#endif //FASES_H
