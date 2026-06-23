#ifndef ACEMA_CTLR_ACTUADORES_H
#define ACEMA_CTLR_ACTUADORES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Activa el actuador de pirotecnia
 * @param canal Número del canal de pirotecnia (0-7)
 * @return 0 si fue exitoso, != 0 si hubo error
 */
int actuador_piro_activar(int canal);

/**
 * @brief Desactiva el actuador de pirotecnia
 * @param canal Número del canal de pirotecnia (0-7)
 * @return 0 si fue exitoso, != 0 si hubo error
 */
int actuador_piro_desactivar(int canal);

/**
 * @brief Inicializa los actuadores del sistema
 * @return 0 si fue exitoso, != 0 si hubo error
 */
int actuadores_init(void);

#ifdef __cplusplus
}
#endif

#endif // ACEMA_CTLR_ACTUADORES_H

