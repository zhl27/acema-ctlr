#include <Arduino.h>
#include "../lib/Domain/GestorDeFases/GestorDeFases.h"

void setup() {
    GestorDeFases gestor;

    gestor.setEstado(prevuelo);

    while (1) {
        gestor.ejecutarCiclo();
    }

}

void loop() {


}