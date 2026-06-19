#ifndef ACEMA_CTLR_TELEMETRIA_H
#define ACEMA_CTLR_TELEMETRIA_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Envía datos de telemetría a la estación terrena
 * @param datos Puntero a los datos a enviar
 * @return 0 si fue exitoso, != 0 si hubo error
 */
int telemetria_enviar(void);

/**
 * @brief Inicializa el módulo de telemetría
 * @return 0 si fue exitoso, != 0 si hubo error
 */
int telemetria_init(void);

#ifdef __cplusplus
}
#endif

#endif // ACEMA_CTLR_TELEMETRIA_H

