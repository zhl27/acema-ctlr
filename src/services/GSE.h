//
// Created by zhl on 6/27/26.
//

#ifndef ACEMA_CTLR_GSE_H
#define ACEMA_CTLR_GSE_H
#include <cstdint>

#include "data.h"
#include "LoraConfig.h"
#include "LoraWrapped.h"

#include "EnlaceGSE/EnlaceGSE.h"

enum EstadoConexionGSE : std::uint8_t {
    ROCKET_INIT,
    ROCKET_DISCONNECTED,
    ROCKET_WAITING_PONG,
    ROCKET_CONNECTED
};

class GSE {
    static constexpr long INTERVALO_TELEMETRIA = 1000; // 5s de frecuencia de envío

    static EstadoConexionGSE _currentState;

    static unsigned long _previousMillis;

    static int _cicloContador;
    static float _simuladorAltitud;

    // Instanciación única y genérica usando los alias de los macros
    static LoraWrapped _lora;

public:
    GSE() = default; // TODO: Revisar constructor de GSE
    static void init();

    static bool actualizar_graficas(const data_all_t *data);
    static bool enviar_mensaje(const char* mensaje);
    static bool enviar_error(const char* error);
    static bool enviar_ack(CmdAck_t acuse);
    static bool enviar_pong();
    static bool leer_paquete(pkt_t* paquete);
    static void set_estado_conexion(EstadoConexionGSE estado);

    //static void actualizar(data_all_t *data);

    static EstadoConexionGSE estado_conexion_gse() {
        return GSE::_currentState;
    }

};


#endif //ACEMA_CTLR_GSE_H
