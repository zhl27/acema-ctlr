//
// Created by zhl on 6/11/26.
//

#include "mde_cohete.h"

void mde_cohete_actualizar(data_all_t* datos_sensores) {
    MDE_COHETE[COHETE.estado_actual](datos_sensores);

}


