//
// Created by lucaz on 20/1/2026.
//

#ifndef GESTORDEFASES_H
#define GESTORDEFASES_H


class Estado; // forward declaration --> para que class GestorDeFases pueda conocerlo

class GestorDeFases {
public:
    // Meyers singleton accessor
    static GestorDeFases& getInstance() {
        static GestorDeFases instance;
        return instance;
    }

    void setEstado(Estado& nuevo) {
        estadoActual = &nuevo;
    }

    void ejecutarCiclo() {
        if (estadoActual) {
            estadoActual->actuar(*this);
        }
    }

    float altitud;
    float velocidad;
    float aceleracion;
    bool frenoAerodinamicoAbierto;
    bool drogueAbierto;
    bool paracaidasPrincipalAbierto;

    // Interfaces a hardware
    void abrirFreno();
    void cerrarFreno();
    void abrirDrogue();
    void abrirParacaidasPrincipal();

    void logSD();
    void enviarTelemetria();

private:
    // Private constructor to enforce singleton
    GestorDeFases()
        : estadoActual(nullptr),
          altitud(0.0f),
          velocidad(0.0f),
          aceleracion(0.0f),
          frenoAerodinamicoAbierto(false),
          drogueAbierto(false),
          paracaidasPrincipalAbierto(false) {}

    ~GestorDeFases() = default;

    // Non-copyable, non-movable
    GestorDeFases(const GestorDeFases&) = delete;
    GestorDeFases& operator=(const GestorDeFases&) = delete;
    GestorDeFases(GestorDeFases&&) = delete;
    GestorDeFases& operator=(GestorDeFases&&) = delete;

    Estado* estadoActual;
};



#endif //GESTORDEFASES_H
