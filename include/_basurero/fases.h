//
// Created by lucaz on 30/1/2026.
//

#ifndef FASES_H
#define FASES_H
#include <HardwareSerial.h>

// enum FaseDeVuelo {
//     PREVUELO,
//     FASE_IMPULSO,
//     FASE_BALISTICA,
//     FASE_FRENO_AERODINAMICO,
//     APOGEO,
//     APERTURA_DROGUE,
//     DESCENSO_RAPIDO,
//     APERTURA_PARACAIDAS_PRINCIPAL,
//     DESCENSO_LENTO,
//     ATERRIZAJE
// };

// Base abstract class
class FaseDeVuelo {
  public:
    virtual ~FaseDeVuelo() {}
    virtual void ejecutar() = 0;     // pure virtual
    virtual const char* nombre() = 0;
};

// Concrete subclasses
class PREVUELO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando PREVUELO: chequeo inicial.");
    }
    const char* nombre() override { return "PREVUELO"; }
};

class FASE_IMPULSO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando FASE_IMPULSO: motor encendido.");
    }
    const char* nombre() override { return "FASE_IMPULSO"; }
};

class FASE_BALISTICA : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando FASE_BALISTICA: vuelo libre balístico.");
    }
    const char* nombre() override { return "FASE_BALISTICA"; }
};

class FASE_FRENO_AERODINAMICO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando FASE_FRENO_AERODINAMICO: reducción de velocidad.");
    }
    const char* nombre() override { return "FASE_FRENO_AERODINAMICO"; }
};

class APOGEO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando APOGEO: punto más alto alcanzado.");
    }
    const char* nombre() override { return "APOGEO"; }
};

class APERTURA_DROGUE : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando APERTURA_DROGUE: despliegue del paracaídas de estabilización.");
    }
    const char* nombre() override { return "APERTURA_DROGUE"; }
};

class DESCENSO_RAPIDO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando DESCENSO_RAPIDO: caída controlada rápida.");
    }
    const char* nombre() override { return "DESCENSO_RAPIDO"; }
};

class APERTURA_PARACAIDAS_PRINCIPAL : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando APERTURA_PARACAIDAS_PRINCIPAL: despliegue del paracaídas principal.");
    }
    const char* nombre() override { return "APERTURA_PARACAIDAS_PRINCIPAL"; }
};

class DESCENSO_LENTO : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando DESCENSO_LENTO: descenso suave bajo paracaídas.");
    }
    const char* nombre() override { return "DESCENSO_LENTO"; }
};

class ATERRIZAJE : public FaseDeVuelo {
  public:
    void ejecutar() override {
      Serial.println("Ejecutando ATERRIZAJE: contacto con el suelo.");
    }
    const char* nombre() override { return "ATERRIZAJE"; }
};









#endif //FASES_H
