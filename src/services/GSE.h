//
// Created by zhl on 6/27/26.
//

#ifndef ACEMA_CTLR_GSE_H
#define ACEMA_CTLR_GSE_H
#include <cstdint>

#include "data.h"
#include "LoraConfig.h"
#include "LoraWrapped.h"

// TODO: DEBEMOS DECLARAR ESTAS MACROS DENTRO DE LoraWrapped
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_DIO1 4

class GSE {
    static constexpr long INTERVALO_TELEMETRIA = 1; // 5s de frecuencia de envío

    enum RocketState : std::uint8_t {
        ROCKET_INIT,
        ROCKET_DISCONNECTED,
        ROCKET_WAITING_PONG,
        ROCKET_CONNECTED
    };

    static RocketState _currentState;

    static unsigned long _previousMillis;

    static int _cicloContador;
    static float _simuladorAltitud;

    // Instanciación única y genérica usando los alias de los macros
    static LoraWrapped _lora;

public:
    GSE() = default; // TODO: Revisar constructor de GSE
    static void init();

    static int actualizar_graficas(const data_all_t *data);
    static int enviar_mensaje(const char* mensaje);
    static int enviar_error(const char* error);

    static void actualizar();

    static void _printear_struct_y_hex(data_plot_t* datos);

    static RocketState estado_mde();
};


#endif //ACEMA_CTLR_GSE_H
