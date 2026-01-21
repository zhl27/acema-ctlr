int main() {
    GestorDeVuelo gestor;

    gestor.setEstado(prevuelo);

    while (1) {
        gestor.ejecutarCiclo();
    }
}
