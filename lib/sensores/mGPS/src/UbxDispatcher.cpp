#include "UbxDispatcher.h"
#include <cstring>
#include <cstddef>
/**
 * @file UbxDispatcher
 * @brief
 * @author Joe Cruz (jocruz@frba.edu.ar)
 * @date 19-06-2026
 */

#define getByte() 

UbxDispatcher::UbxDispatcher(const UbxRegMsg_t **ptrTablaMsg, const size_t tamanioTabla):
    _tablaMsg(ptrTablaMsg), _tablaMsgSize(tamanioTabla)
{
    this->stActual = &UbxDispatcher::waitSync1;
    _payloadCounter = 0;
    _ptrPayloadActual=nullptr;
    _offsetActual = 0;
    _checksumModulo[0] = _checksumModulo[1] = 0;
}

UbxDispatcher::~UbxDispatcher()
{
}

void UbxDispatcher::waitSync1(uint8_t byte)
{
    if(byte == SYNC::_1){
        stActual = &UbxDispatcher::waitSync2;
        return;
    }
    return;
}

void UbxDispatcher::waitSync2(uint8_t byte)
{
    if(byte == SYNC::_2){
        // Inicializamos el cálculo del checksum local justo antes de procesar el subencabezado
        _header.ckA = 0;
        _header.ckB = 0;
        _payloadCounter = 0;
        stActual = &UbxDispatcher::loadSubencabezado;
    } else {
        stActual = &UbxDispatcher::waitSync1; // Si falla, resetea la FSM
    }
}

void UbxDispatcher::loadSubencabezado(uint8_t byte)
{
    bool mensajeEncontrado = false;
    // Acumulamos el byte en el subencabezado (Class, ID, LenL, LenH)
    _header.subEncabezado[_payloadCounter++] = byte;

    // El byte actual forma parte del cálculo del checksum
    _header.ckA += byte;
    _header.ckB += _header.ckA;

    if(_payloadCounter == 4)
    {
        _payloadCounter = 0; // Resetea el contador para usarlo en el payload
        
        // Búsqueda lineal en la tabla de registros. Lookup Table
        for (size_t i = 0; i < _tablaMsgSize; i++)
        {
            if((_header.msgClass == _tablaMsg[i]->msgClass) && 
                (_header.msgID == _tablaMsg[i]->msgID))
            {
                _offsetActual = i;
                _ptrPayloadActual = _tablaMsg[i]->payloadBuffer;
//               (_tablaMsg[i]->onReceive)();
                mensajeEncontrado = true;
                break;
            }
            
        }

        if (mensajeEncontrado)
        {
            // Si el mensaje existe pero su longitud declarada es cero, saltamos directo al checksum
            if(_tablaMsg[_offsetActual]->length == 0) {
                stActual = &UbxDispatcher::checksum;
            } else {
                stActual = &UbxDispatcher::loadPayload;
            }
        }
        else{
            stActual =&UbxDispatcher::ignorePayload; 
        }       
    }
}

void UbxDispatcher::loadPayload(uint8_t byte)
{
    // Almacena el byte directamente indexando con el contador de instancia
    *(_ptrPayloadActual + _payloadCounter++) = byte; 

    // Actualiza el checksum acumulativo
    _header.ckA += byte;
    _header.ckB += _header.ckA;

    // Evalua si terminó de llenar la estructura de datos asignada
    if(_payloadCounter == _tablaMsg[_offsetActual]->length)
    {
        _payloadCounter = 0; // Lo dejamos en cero listo para que el estado CHECKSUM lo use como índice
        stActual = &UbxDispatcher::checksum;
    }
}

void UbxDispatcher::ignorePayload(uint8_t byte)
{
    // Seguimos calculando el checksum por si quisiéramos validar paquetes ignorados,
    // pero operativamente solo contamos bytes hasta vaciar el payload no deseado del buffer serial.
    _payloadCounter++;

    if(_payloadCounter == _header.length) 
    {
        _payloadCounter = 0; 
        stActual = &UbxDispatcher::checksum; // Saltamos a consumir los 2 bytes de CRC sobrantes
    }
}

void UbxDispatcher::checksum(uint8_t byte)
{
    // _payloadCounter actúa aquí como índice: 0 para CK_A entrante, 1 para CK_B entrante
    if(_payloadCounter == 0)
    {
        // Guardamos el primer byte de checksum enviado por el módulo para comparar al final
        _checksumModulo[0] = byte;
        _payloadCounter = 1;
    }
    else if(_payloadCounter == 1)
    {
        _checksumModulo[1] = byte;

        // ¡Momento de la verdad! Validamos nuestro checksum calculado contra el del módulo
        if((_header.ckA == _checksumModulo[0]) && (_header.ckB == _checksumModulo[1]))
        {
            // El paquete es íntegro y legítimo. Ejecutamos el callback correspondiente.
            // (Si entramos desde ignorePayload, _offsetActual no será válido, validamos protección)
            if(stActual != &UbxDispatcher::ignorePayload && _tablaMsg[_offsetActual]->onReceive != nullptr)
            {
                _tablaMsg[_offsetActual]->onReceive();
            }
        }
        else
        {
            // TODO: Se podria implementar aquí un contador de errores de CRC globales si lo deseas
        }

        // Reseteo total de la FSM para la siguiente trama limpia
        _payloadCounter = 0;
        memset(&_header, 0, sizeof(ubx_header_t));
        stActual = &UbxDispatcher::waitSync1;
    }
}
