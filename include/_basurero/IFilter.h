//
// Created by lucaz on 2/7/2026.
//

#ifndef ACEMA_CTLR_IFILTER_H
#define ACEMA_CTLR_IFILTER_H
class IFilter {

    /*valor de inicio */
    void virtual inicializar(float) = 0;

    /* Actualización */
    float virtual filtrar(float) = 0;

    /* Reseteo */
    void virtual resetear() = 0;
};
#endif //ACEMA_CTLR_IFILTER_H
