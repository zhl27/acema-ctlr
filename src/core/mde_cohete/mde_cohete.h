//
// Created by zhl on 6/11/26.
// Refactorizado para misión ACEMA (Vuelo real con redundancia)
//

#ifndef ACEMA_CTLR_MDE_COHETE_H
#define ACEMA_CTLR_MDE_COHETE_H

#include "data.h"
#include "funciones_de_estado.h"
#include "include.h"

/* =========================================================================
 * CONSULTAS DE DISEÑO / INCERTIDUMBRES (Para revisar con el equipo)
 * =========================================================================
 *
 * 1. SOBRE EL EJE VERTICAL:
 * En tu pseudocódigo usaste 'ay' y 'vy'. Estoy asumiendo que tu software
 * trata el EJE Y como el vector normal al suelo (típico de motores gráficos).
 * En aeronáutica el estándar inercial suele ser el eje Z (Z-Down o Z-Up).
 * Si su vertical física calibrada es Z, hay que cambiar los '.y' por '.z'.
 *
 * Solución: Se configura en código que eje del sensor representa el eje +Z del cohete. 
 * Un cambio de sistemas de referencia. 
 * 
 * 2. SOBRE EL SHOCK ESTRUCTURAL DEL PARACAÍDAS PRINCIPAL:
 * Si el drogue falla y venimos a -35 m/s (o peor), abrir el principal de
 * golpe genera un "Opening Shock" brutal. ¿La cuerda de retención (shock cord)
 * y los cáncamos de la bahía de recuperación soportan los Newtons de
 * desacelerar esa masa a esa velocidad, o corremos riesgo de arrancar la bahía?
 *
 * 3. SOBRE LA SD DE LA CÁMARA POR UART:
 * Enviar datos por TX (Compu) a RX (Cámara) asume que la cámara tiene un
 * microcontrolador propio ejecutando un firmware que sabe agarrar lo que entra
 * por su UART y anexarlo a un archivo .txt en su SD. Una tarjeta SD física
 * habla protocolo SPI o SDIO, no entiende UART nativo. Verificar este puente.
 *
 * 4. SOBRE EL TIMEOUT DE CONEXIÓN GSE:
 * Definí un flag 'vuelo_en_silencio_radio'. Si salimos a volar sin enlace,
 * ¿queremos que el cohete intente reconectar continuamente en segundo plano
 * durante el ascenso, o apagamos el módulo de radio para ahorrar batería?
 * ========================================================================= */

namespace Cohete {

    // bool entrando_a_estado();
    // void transicionar_hacia(estado_cohete_t nuevo_estado);

    /** @brief Tabla de punteros a función estrictamente mapeada a estado_vuelo_t */
    extern const f_st_t MDE_COHETE[];
    extern const char* estado_cohete_string[];

    void mde_cohete_actualizar(data_all_t* datos_sensores);
    // void transicion_error(error_cohete_t error, data_all_t* datos_sensores);


}




#endif //ACEMA_CTLR_MDE_COHETE_H