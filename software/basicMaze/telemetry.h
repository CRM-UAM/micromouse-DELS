/********************/
/* TELEMETRY MODULE */
/********************/
/* Genera un servidor Telnet conectandose a una red WiFi y 
 *  permite mandar mensajes a los clientes que esten 
 *  escuchando en su Ip y puerto 23 dentro de la misma red WiFi */

#ifndef _TELEMETRY_H
#define _TELEMETRY_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//inlcuir estas declaraciones en los ficheros que usen la telemetría para poder acceder directamente a la memoria de este modulo
//extern float **telemetria;
//extern int p_telemetria=0;

/**
 * Inicializa la conexion wifi para mandar la telemetria
 * Genera un servidor telnet en el que se puede escuchar desde un ordenador con el comando: $> nc <IP_ESP> 23
 */
void init_telnet();

/**
 * Envia una cadena de texto por telnet
 */
void print_tl(String str);

/**
 * Si la conexión esta up envia el array de valores **telemetria
 */
void check_send_telnet_telemetry();


#endif
