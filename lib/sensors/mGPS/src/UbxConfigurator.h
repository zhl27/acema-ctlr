#ifndef UBX_CONFIGURATOR_H
#define UBX_CONFIGURATOR_H

#include <cstdint>
#include <cstddef>
#include <UbxProtocols.h>
#include "UbxDispatcher.h" // Para conocer UbxRegMsg_t

class UbxConfigurator {
public:
    /**
     * @brief Punteros a funciones provistos por el usuario (Desacople)
     */
    typedef void (*TxCallback)(const uint8_t* data, size_t len);
    typedef bool (*WaitAckCallback)(uint8_t cls, uint8_t id, uint32_t timeoutMs);

    /**
     * @brief Constructor
     * @param txFunc Función que escribirá físicamente en la UART.
     * @param waitFunc Función bloqueante (RTOS) que espera la llegada de un ACK.
     */
    UbxConfigurator(TxCallback txFunc, WaitAckCallback waitFunc);

    ~UbxConfigurator() = default;

    // --- Métodos de Configuración Específicos ---

    /**
     * @brief Configura la UART. Desactiva NMEA y activa solo UBX en la salida.
     * @param baudrate Velocidad deseada (ej. 115200).
     */
    bool setPortUart(uint32_t baudrate) const;

    /**
     * @brief Ajusta la frecuencia de los cálculos del GPS.
     * @param rateHz Frecuencia en Hertz (ej. 5 para 5Hz / 200ms).
     */
    bool setNavigationRate(uint8_t rateHz) const;

    /**
     * @brief Ajusta el modelo dinámico del filtro de Kalman.
     * @param model 4=Auto (Robots), 8=Airborne<4G (Cohetes), etc.
     */
    bool setDynamicModel(nav_dyn_model_e model) const;

    /**
     * @brief Lee la tabla del Dispatcher y pide al GPS que envíe todo.
     * @param ptrTablaMsg Puntero a la tabla usada en UbxDispatcher.
     * @param tamanioTabla Cantidad de elementos.
     */
    bool enableRegisteredMessages(const UbxRegMsg_t **ptrTablaMsg, size_t tamanioTabla) const;


private:
    TxCallback _txFunc;
    WaitAckCallback _waitFunc;

    /**
     * @brief Método interno genérico. 
     * Arma el paquete completo (Header + Payload + Checksum) y lo envía.
     */
    bool _buildAndSend(ubx_class_e msgClass, uint8_t msgID, const uint8_t* payload, size_t payloadSize) const;
};

#endif