//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

void mde_cohete_actualizar(data_all_t* datos_sensores) {
    if (COHETE.estado < ST_NULL) {
        // if (SISTEMA.baro.presionBar > SISTEMA.limPresion.max || SISTEMA.flags.emergencia == 1) {
        //     transicionError(self, datosEnsayo);
        // }
        MDE_COHETE[COHETE.estado](datos_sensores); // Ejecuta la función que corresponde al estado actual
    } else {
        manejar_error(datos_sensores);
    }
}

void manejar_error(data_all_t* datos_sensores) {
    COHETE.estado = estado_vuelo_t::ST_ERROR;
    // Acción de seguridad
    // TODO: Definir qué se hace en cada tipo de error.
    // definimos en alguna función de estado el error que surge tal que así:
        // COHETE.error = ERR_TRAYECTORIA_PELIGROSA;

}

