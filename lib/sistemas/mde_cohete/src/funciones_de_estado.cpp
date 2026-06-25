#include "funciones_de_estado.h"

#include "mde_cohete.h"
#include "SerialPrint.h"
//#include <Arduino.h>

// TODO: Integrar con la variable global COHETE para transiciones de estado

void f_st_inicializar_sistema(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [INIT] Inicializando buses I2C/SPI y SD...");
    // Lógica:
    // 1. Configurar BMP280, MPU6050, módulo LoRa y SD.
    // 2. Si la SD falla, setear bandera COHETE.usar_uart_camara_como_sd = true
    //    y reconfigurar los pines UART correspondientes.
    // 3. Transición a ST_WARMUP_MPU.
}

void f_st_ejecutar_warmup_mpu(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [WARMUP] Esperando estabilización térmica del MPU6050 (5 min).");
    // Lógica:
    // 1. Monitorear millis().
    // 2. Una vez superado el tiempo (300,000 ms), capturar offsets en reposo.
    // 3. Transición a ST_BUSCANDO_CONEXION_GSE.
}

void f_st_buscar_enlace_gse_timeout(data_all_t* datos_sensores) {
    static unsigned long t_inicio_busqueda = 0; // Usar millis() o el timestamp del sistema

    // Lógica:
    // 1. Intentar recibir configuración por LoRa (GSE).
    // 2. Si llegan los datos, configurar y pasar a ST_ESPERA_DESPEGUE.
    // 3. Si (millis() - t_inicio_busqueda > TIMEOUT_LORA), entonces:
    //    COHETE.vuelo_en_silencio_radio = true;
    //    Transición forzada a ST_ESPERA_DESPEGUE (el despegue es prioridad).
}

void f_st_esperar_gps_fix(data_all_t* datos_sensores) {
    // Actualizamos las variables globales para que el GSE sepa qué pasa
    // COHETE.contexto_fisico.satelites_visibles = datos_sensores->gps.satellites;
    // COHETE.contexto_fisico.gps_hdop = datos_sensores->gps.hdop;

    if (COHETE.contexto_fisico.satelites_visibles < 4) {
        SerialPrint::msg(" -> [GPS] Esperando satélites. Visibles: ");
        // Imprimir cantidad de satélites. El cohete está bloqueado aquí.
    }
    // Un FIX 3D requiere mínimo 4 satélites.
    // Un HDOP menor a 2.0 o 2.5 significa que la precisión es lo suficientemente buena (los satélites no están todos agrupados en un rincón del cielo).
    else if (COHETE.contexto_fisico.satelites_visibles >= 5 && COHETE.contexto_fisico.gps_hdop < 2.5f) {
        SerialPrint::msg(" -> [GPS] Fix 3D asegurado. Listo para rampa.");
        COHETE.contexto_fisico.gps_3d_fix_obtenido = true;

        // Transición de estado:
        // COHETE.estado = ST_ESPERA_DESPEGUE;
    }

    // Opcional: Agregar un mecanismo de override (timeout largo o comando desde GSE)
    // por si el clima está muy nublado y deciden lanzar igual bajo su propio riesgo.
}

void f_st_esperar_impulso(data_all_t* datos_sensores) {
    // Aquí el cohete está pasivo en la rampa. Ignición es externa.
    // Lógica del filtro anti-zarandeo:
    // 1. Transformar aceleración a vector inercial.
    // 2. if (aceleracion_vertical >= 2.0g) {
    //      iniciar temporizador (timestamp_inicio_pico_g)
    //    }
    // 3. if (tiempo_con_2g >= 150ms AND delta_altura > 4.0m) {
    //      Transición a ST_PROPULSION.
    //    }
}

void f_st_gestionar_ascenso_propulsado(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [PROPULSION] ¡Despegue detectado! Motor encendido.");
    // Lógica:
    // 1. Seguir integrando velocidad y altitud a 150 Hz.
    // 2. Cuando la aceleración vertical decaiga bruscamente (Burn-out / Fin de combustión),
    //    Transición a ST_FASE_BALISTICA.
}

void f_st_gestionar_vuelo_balistico_y_freno(data_all_t* datos_sensores) {
    // Lógica:
    // 1. Activar servomotores de frenado aerodinámico si están integrados.
    // 2. Monitorear constantemente las condiciones de Apogeo:
    //    if (velocidad_vertical <= 0 && aceleracion_vertical < 0 && altura_actual == altura_maxima) {
    //      Transición a ST_APOGEO.
    //    }
}

void f_st_disparar_apogeo_y_detener_cam(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [APOGEO] Apogeo detectado. Deteniendo cámara y abriendo Drogue.");
    // Lógica:
    // 1. Enviar señal de STOP a la cámara por pin/UART.
    // 2. Ignición pirotécnica del Drogue.
    // 3. Guardar tiempo: COHETE.timestamp_apertura_drogue = millis();
    // 4. Transición inmediata a ST_DESCENSO_EVALUACION.
}

void f_st_evaluar_supervivencia_drogue(data_all_t* datos_sensores) {
    // Este estado DEBE durar exactamente 3 segundos.
    // Lógica:
    // if (millis() - COHETE.timestamp_apertura_drogue >= 3000) {
    //      if (velocidad_vertical <= -35 m/s) {
    //          // Falló el drogue
    //          Transición a ST_DESCENSO_EMERGENCIA;
    //      } else if (aceleracion_vertical ~= -1g) {
    //          // Caída libre balística. Muerte inminente.
    //          Transición a ST_CAIDA_CATASTROFICA;
    //      } else {
    //          // Todo en orden. El drogue frenó el cohete.
    //          Transición a ST_DESCENSO_NOMINAL;
    //      }
    // }
}

void f_st_descenso_controlado_drogue(data_all_t* datos_sensores) {
    // Bajando con Drogue.
    // Lógica:
    // if (altura_actual_filtrada <= 250.0m) { // ¡Cuidado de chequear contra cota_suelo_rampa!
    //      Ignición pirotécnica Paracaídas Principal.
    //      SerialPrint::msg(" -> [NOMINAL] Paracaídas Principal desplegado.");
    //      Transición a ST_ATERRIZAJE (o estado intermedio de espera).
    // }
}

void f_st_desplegar_principal_emergencia(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [EMERGENCIA] Drogue fallido. Disparando Principal de inmediato!");
    // Lógica:
    // 1. Disparo inmediato del paracaídas principal.
    // 2. Transición a ST_ATERRIZAJE (esperando el suelo).
}

void f_st_ejecutar_panico_flash_dump(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [FATAL] Caída libre detectada. Volcando RAM a Flash!");
    // Lógica:
    // 1. Las tarjetas SD mecánicas pueden corromperse en impactos duros.
    // 2. Escribir el buffer circular de últimos 5 segundos en la Flash de la ESP32
    //    (NVS o SPIFFS/LittleFS) para análisis post-mortem.
    // 3. Este estado no tiene salida, es terminal antes del impacto.
}

void f_st_transmitir_baliza_aterrizaje(data_all_t* datos_sensores) {
    // Lógica:
    // 1. Detectamos reposo en el suelo (posicion_world cerca de 0 relativa, acel == 0).
    // 2. Detener logs de alta frecuencia para salvar batería.
    // 3. Cerrar archivos en la SD (flush y close).
    // 4. Emitir un "beep" continuo y transmitir coordenadas Lat/Lon por LoRa cada X segundos.
}

void f_st_procesar_falla_de_sistema(data_all_t* datos_sensores) {
    SerialPrint::msg(" -> [ERROR] Error crítico de hardware detectado.");
    // Manejo genérico de fallas.
}