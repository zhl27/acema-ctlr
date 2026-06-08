#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

// ============================================================================
// SELECCIÓN DE MÓDULO (Descomenta SOLO UNO)
// ============================================================================
#define MODULE_SX1278
// #define MODULE_SX1262



#define SIZE_BUFFER_MSG 128
#define DEFAULT_SYNC_WORD 0x12 // RadioLib suele usar bytes numéricos (ej: 0x12)
#define DEFAULT_ENCRY_WORD '%' 
#define DEFAULT_FREC 433.0     // RadioLib trabaja la frecuencia en MHz (float)
#define SIZE_BUFFER_MSG 128

#endif // LORA_CONFIG_H