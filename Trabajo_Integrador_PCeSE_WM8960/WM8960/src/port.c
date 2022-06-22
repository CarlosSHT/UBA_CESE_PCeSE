/*
 * port.c
 *
 *  Created on: Jun 16, 2022
 *      Author: carlo
 */
#include "i2c.h"
#include "usart.h"
#include "i2s.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * Definición de instancias globales Punteros para los handlers de los periféricos I2C e I2S
 */
static I2C_HandleTypeDef *i2cportHandle;
static I2S_HandleTypeDef *i2sportHandle;

#define WM8960_ADDRESS	0x34	// Dirección I2C del codec de Audio
#define TOUT			10		// Valor de 10 para el timeout de las funciones HAL

/**
 *
 * Función de inicialización del port.c a fin de que el usuario tenga la capacidad de
 * configurar la instancia de los periféricos I2C e I2S disponibles en la plataforma STM32
 *
 * @param i2c_inst	Instancia I2C definidia por el usuario
 * @param i2s_inst	Instancia I2S definidia por el usuario
 * @return			Valor de retorno de error 0 (sin error), -1 (error)
 */
int8_t Init_wm8960_port(I2C_HandleTypeDef *i2c_inst,
		I2S_HandleTypeDef *i2s_inst) {
	if (HAL_I2C_Init(i2c_inst) != HAL_OK) {
		return -1;
	}
	i2cportHandle = i2c_inst;

	if (HAL_I2S_Init(i2s_inst) != HAL_OK) {
		return -1;
	}
	i2sportHandle = i2s_inst;

	return 0;
}


/**
 *
 * Función de "lectura" de registros del codec, emula la lectura de registros al leer el valor del vector
 * de registros almacenado en la variable del programa
 *
 * @param reg_dir		Valor de la dirección del registro
 * @param reg_val		Valor del registro consultado (2 bytes)
 * @param array_vals	Puntero a vector con valores de registros almacenados
 */
void I2C_read_wm8960_port(uint8_t reg_dir, uint16_t *reg_val,
		uint16_t *array_vals) {
	*reg_val = *(array_vals + reg_dir);
}

/**
 *
 * Realiza la función de escritura I2C en 2 envios de datos, el primer envio consta
 * de la dirección del registro y el último bit más significativo del valor de datos del registro
 * El segundo envío realiza una transmisión de los 8 bits menos significativos del registro
 *
 * @param regdir		Valor dirección del regisro
 * @param value			Valor del registro expresado en 16bits
 * @param storageVals	Vector de almacenamiento de valores de registro
 * @return
 */
int8_t I2C_write_wm8960_port(uint8_t regdir, uint16_t value,
		uint16_t *storageVals) {
	HAL_StatusTypeDef res;
	uint8_t dataTX[2];

	*dataTX = ((value >> 8) | 0x01) | (regdir << 1);
	*(dataTX + 1) = (uint8_t) (value & 0x00FF);

//	res = HAL_I2C_Mem_Write(i2cPortHandle, DeviceAdressWM8960, regdir, I2C_MEMADD_SIZE_8BIT,  val, I2C_DATSIZE, TOUT); // No funciona
	res = HAL_I2C_Master_Transmit(i2cportHandle, WM8960_ADDRESS, dataTX,
			sizeof(dataTX) / sizeof(uint8_t), TOUT);

	if (res != HAL_OK) {
		return -1;
	}
	*(storageVals + regdir) = value;

	return 0;
}

/**
 *
 * Función de escritura del periférico I2S utilizando DMA e interrupciones habilitadas
 *
 * @param pBuffer	Vector de audio con posición inicial actualizada
 * @param audSize	Cantidad de elementos a enviar del vector de audio
 * @return			Estado de la transmisión, si es HAL_OK se realizó con éxito, caso contrario hay error
 */
int8_t I2S_write_w8960_port(uint16_t *pBuffer, uint32_t audSize) {
	HAL_StatusTypeDef res;

	res = HAL_I2S_Transmit_DMA(i2sportHandle, pBuffer, audSize);

	if (res != HAL_OK) {
		return -1;
	}

	return 0;
}


/********** Funciones de la capa HAL utilizadas luego de las Interrupciones del driver **********/

/**
 *
 * Realiza una pausa en la transmisión DMA del periferico del STM32, utilizado para
 * realizar una pausa de la música
 *
 * @return	Estado de la ejecución del comando "pause" del DMA
 */
int8_t I2S_pauseDMA_wm8960_port(void)
{
	HAL_StatusTypeDef res;
	res = HAL_I2S_DMAPause(i2sportHandle);

	if (res != HAL_OK) {
		return -1;
	}

	return 0;
}

/**
 *
 * Se reanuda el proceso de transmisión por DMA a través del I2S a fin de continuar escuchando el audio
 *
 * @return	Estado de la ejecución del comando "resume" del DMA
 */
int8_t I2S_resumeDMA_wm8960_port(void)
{
	HAL_StatusTypeDef res;
	res = HAL_I2S_DMAResume(i2sportHandle);

	if (res != HAL_OK) {
		return -1;
	}

	return 0;
}

/**
 * Se realiza una finalización del envio de datos  DMA por un comando del usuario
 *
 * @return
 */
int8_t I2S_stopDMA_wm8960_port(void)
{
	HAL_StatusTypeDef res;
	res = HAL_I2S_DMAStop(i2sportHandle);

	if (res != HAL_OK) {
		return -1;
	}

	return 0;
}
/*int8_t I2S_read_w8960_port(uint16_t *pBuffer, uint32_t audSize) {
	HAL_StatusTypeDef res;

	res = HAL_I2S_Receive_DMA(i2sportHandle, pBuffer, audSize);

	if (res != HAL_OK) {
		return -1;
	}

	return 0;
}*/

/**
 * Función que permite el envío de texto a traves de la función "printf" desde el STM32 a través del periférico serial
 *
 * @param PUTCHAR_PROTOTYPE
 * @return
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART3 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}
