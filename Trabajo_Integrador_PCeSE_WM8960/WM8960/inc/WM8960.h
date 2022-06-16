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

typedef enum _WM8960_i2sWL
{	AUDIO_WL_16bits=0,
	AUDIO_WL_20bits,
	AUDIO_WL_24bits,
	AUDIO_WL_32bits,
}wmWordLen_t;


typedef enum _WM8960_i2sFormat
{	FORMAT_RLEFT=0,
	FORMAT_JLEFT,
	FORMAT_I2S,
	FORMAT_DSP,
}wmAFormat_t;

typedef enum _i2cHandleSTM
{
	i2cHandle1=0,
	i2cHandle2,
}i2cHandleSTM;

typedef int8_t (*spFunctionI2C_t)(uint8_t,uint8_t);
typedef int8_t (*sdFunctionI2C_t)(uint8_t, uint16_t, uint16_t *);
typedef int8_t (*rdFunctionI2C_t)(uint8_t, uint16_t *);
typedef int8_t (*spFunctionI2S_t)(uint8_t);
typedef int8_t (*sdFunctionI2S_t)(uint16_t *, uint32_t );

typedef struct _wm_i2cRegDat  {
	uint8_t i2c_Reg;
	uint8_t i2c_Val;

} wm_i2cRegDat;



struct _wm8960  {
	spFunctionI2C_t		i2c_select_port;
	sdFunctionI2C_t		i2c_sendDat_port;
	rdFunctionI2C_t		i2c_recvDat_port;
	spFunctionI2S_t		i2s_select_port;
	sdFunctionI2S_t		i2s_sendDat_port;
//	spiRead_t spi_read_fnc;
//	delay1ms_t delay_1ms_func;
};

typedef struct _wm8960 wm8960_t;


typedef enum _ena_ClassD
{	CLASS_D_OFF=0,
	CLASS_D_LSPK,
	CLASS_D_RSPK,
	CLASS_D_BOTH,
}ena_ClassD;


void WM8960_InitDriver(wm8960_t configWM);
uint32_t WM8960_audioPlay(uint16_t *pBuffer, uint32_t audSize) ;

/**
 * IQR to continue Playing music
 */
void WM8960_setFlagplay(void);

void WM8960_stopFlagplay(void);


#endif /* SRC_WM8960_H_ */
