#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

// ============================================================================
// SELECCIÓN DE MÓDULO (Descomenta SOLO UNO)
// ============================================================================
#define MODULE_SX1278
// #define MODULE_SX1262

// ============================================================================
// CONFIGURACIÓN DE PINES SEGÚN TU HARDWARE
// ============================================================================
#if defined(MODULE_SX1278)
    const int LORA_NSS  = 10;
    const int LORA_RST  = 9;
    const int LORA_DIO0 = 2; // Pin de interrupción principal para SX1278
    const int LORA_DIO1 = 3; // Opcional en SX1278

#elif defined(MODULE_SX1262)
    const int LORA_NSS  = 10;
    const int LORA_RST  = 9;
    const int LORA_DIO1 = 2; // El SX1262 usa DIO1 para indicar paquetes listos
    const int LORA_BUSY = 3; // OBLIGATORIO para SX1262
#endif


#define SIZE_BUFFER_MSG 128
#define DEFAULT_SYNC_WORD 0x12 // RadioLib suele usar bytes numéricos (ej: 0x12)
#define DEFAULT_ENCRY_WORD '%' 
#define DEFAULT_FREC 433.0     // RadioLib trabaja la frecuencia en MHz (float)
#define SIZE_BUFFER_MSG 128

#endif // LORA_CONFIG_H