/*
 * WM8960.h
 *
 *  Created on: Jun 16, 2022
 *      Author: carlo
 */

#ifndef SRC_WM8960_H_
#define SRC_WM8960_H_

#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

/* Definicio del tipo bool para el driver*/
typedef bool bool_t;

/** definición de tipos puntero a función **/
typedef int8_t (*sdFunctionI2C_t)(uint8_t, uint16_t, uint16_t*);
typedef void (*rdFunctionI2C_t)(uint8_t, uint16_t*, uint16_t*);
typedef int8_t (*sdFunctionI2S_t)(uint16_t*, uint32_t);
typedef int8_t (*paFunctionI2S_t)(void);
typedef int8_t (*reFunctionI2S_t)(void);
typedef int8_t (*stFunctionI2S_t)(void);
//typedef int8_t (*rdFunctionI2S_t)(uint16_t*, uint32_t);

/**
 * Estructura puntero a función con las  funciones requeridas en el archivo port
 *
 * 			Periféricos I2C e I2S se encuentran en modo maestro
 *
 * 			i2c_sendDat_port	Envia datos a traves del periférico I2C
 * 			i2c_recvDat_port	Recibe datos a traves del periférico I2C
 * 								Nota: el codec WM8960 no tiene la fución de lectura por I2C por tanto
 * 								se obtiene el "valor actual" de un registro utilizando un vector de datos,
 * 								los cuales se actualizan luego de realizar la función de escritura I2C
 *
 * 			i2s_sendDat_port	Envia datos a traves del periférico I2S
 * 			i2s_pausDat_port	Pone en pausa la tranmisión de datos por DMA del periférico I2S
 * 			i2s_resuDat_port	Resume la tranmisión de datos por DMA del periférico I2S
 * 			i2s_stopDat_port	Finaliza la tranmisión de datos por DMA del periférico I2S
 *
 */
struct _wm8960 {
	sdFunctionI2C_t i2c_sendDat_port;
	rdFunctionI2C_t i2c_recvDat_port;
	sdFunctionI2S_t i2s_sendDat_port;
	paFunctionI2S_t	i2s_pausDat_port;
	reFunctionI2S_t	i2s_resuDat_port;
	stFunctionI2S_t i2s_stopDat_port;
//	rdFunctionI2S_t i2s_recvDat_port;

};
typedef struct _wm8960 wm8960_t;

/**
 * Estructura de datos de la maquina de estados finita para la reproducción de una variable de audio
 * Se utiliza a fin de configurar y leer el estado actual de la maquina de estados
 */
typedef enum _WM8960_pstatesFSM {
	pr_start = 0, pr_pending, pr_paused, pr_stopped
} WM8960_prStates;

/**
 * Inicializa el driver WM8960
 * Se pasa como parametro una estructura que apunta a funciones
 * Se configura el codec WM8960 en modo de reproducción
 *
 * @param configWM
 */
void WM8960_InitDriver(wm8960_t configWM);

/**
 * La función permite configurar la ganancia del DAC del codec
 *
 * @param vol: Parametro de ganancia desde 0db hasta 79 dB con resolución de 0.5dB
 */
void WM8960_setGainDAC(float vol);

/**
 * @func WM8960_setGainHPs
 * Se utiliza para configurar la ganacia del periferico "headphone" del codec wm8960
 *
 * @param hp_vol: parametro de entrada con rango 0-79, pasos de 1dB de ganancia
 */
void WM8960_setGainHPs(int8_t hp_vol);

/**
 * @function WM8960_setGainSPKs	Configura la ganancia de la salida "speaker"
 *
 * @param spk_vol: parametro de entrada con rango 0-79, pasos de 1dB de ganancia
 */
void WM8960_setGainSPKs(uint8_t spk_vol);

/**
 *
 * @function WM8960_dtcHPjack : Habilita o deshabilita la detección del jack, el cual
 * 								permite la predominancia del conector jack sobre los parlantes
 * 								("speaker") deshabilitando este último
 *
 * @param detectFlag: Valor "true" para habilitar y "false" para deshabilitar"
 */
void WM8960_dtcHPjack(bool_t detectFlag);


/**
 * @function WM8960_audioPlayVar:	Función para la reproducción sonora de una variable, ejecuta internamente
 * 									una maquina de estados finita. Esta función es llamada a fin de que se realize
 * 									el envío de paquetes de datos por el periférico I2S
 *
 * @param pBuffer: Entrada de la variable de sonido (vector) con tipo de dato uint16_t
 * @param audSize: Entrada del tamaño de vector de datos de sonido en tipo de dato uint32_t
 */
void WM8960_audioPlayVar(uint16_t *pBuffer, uint32_t audSize);


/**
 * @function WM8960_getStatePlayer:	Permite conocer el estado actual en el que se encuentra la reproducción
 * 									de la variable de sonido: estado "inicio", "pendiente", "pausa", "finalizado",
 *
 * @return	Devuelve el tipo de dato WM8960_prStates con el estado correspondiente
 */
WM8960_prStates WM8960_getStatePlayer(void);

/**
 * @function WM8960_setStatePlayer	Establece el estado actual de la reproducción del audio.
 * 									Se utiliza a fin de iniciar la reproducción configurando en
 * 									el estado "inicio".*
 *
 * @param newPlayState 	Recibe como parametro de entrada el valor del estado del tipo estructura WM8960_prStates
 * 						pr_start ("inicio"), pr_pending ("pendiente"), pr_paused ("pausa"), pr_stopped ("finalizado")
 */
void WM8960_setStatePlayer(WM8960_prStates newPlayState);

/**
 * @function WM8960_setIRQplay	La función es llamada con una interrupción desde el contexto principal
 * 								Establece un flag global interno del driver en estado de REPRODUCCION (Play)
 * 								Su uso es la detección de interrupciones generadas por fin de transmisión I2S con DMA
 * 								a fin de que se vuelva a enviar el siguiente paquete de datos al codec
 */
void WM8960_setIRQplay(void);

/**
 * @function	WM8960_setIRQpause	Permite realizar una pausa en la reproducción de la variable de audio al ejecutar
 * 									internamente una pausa en la transmisión DMA y establece "pausa" en la FSM interna
 * 									del driver
 */
void WM8960_setIRQpause(void);

/**
 * @function	WM8960_setIRQstop	Realiza una finalización en la comunicación  DMA del I2S, y establece los valores
 *									respectivos de inicio para una siguiente reproducción de cualquier otra variable
 */
void WM8960_setIRQstop(void);

//void WM8960_setIRQrec(void);
//void WM8960_audioRecVar(uint16_t *pBuffer, uint32_t audSize) ;
//WM8960_prStates WM8960_getStateRecorder(void);
//void WM8960_setStateRecorder(WM8960_prStates newRecState);

#endif /* SRC_WM8960_H_ */
