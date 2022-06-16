/*
 * port.c
 *
 *  Created on: Jun 16, 2022
 *      Author: carlo
 */
#include "port.h"
//#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "i2s.h"

I2C_HandleTypeDef 	*i2cPortHandle;
I2S_HandleTypeDef	*i2sportHandle;
uint8_t DeviceAdressWM8960=-1;

int8_t i2cInstDev_WM8960_port(uint8_t numI2Cport,uint8_t WM8960_addr) {
	int8_t res = 0;
	DeviceAdressWM8960=WM8960_addr;

	switch (numI2Cport) {
		case 1:
			i2cPortHandle = &hi2c1;
			res = 0;
			break;
//		case 2:
//			i2cPortHandle=&hi2c2;
//			break;
		default:
			res = -1;
			break;
	}
	return res;
}


int8_t i2sInst_WM8960_port(uint8_t numI2Sport) {
	int8_t res = 0;

	switch (numI2Sport) {
		case 2:
			i2sportHandle = &hi2s2;
			res = 0;
			break;
//		case 2:
//			i2cPortHandle=&hi2c2;
//			break;
		default:
			res = -1;
			break;
	}
	return res;
}


int8_t i2cwrite_WM8960_port(uint8_t regdir, uint16_t value, uint16_t *storageVals) {

	HAL_StatusTypeDef res;
	uint8_t dataTX[2],val=0;

	*dataTX= ((value>>8)|0x01) | (regdir<<1);
	*(dataTX+1)= (uint8_t) (value & 0x00FF);

//	res = HAL_I2C_Mem_Write(i2cPortHandle, DeviceAdressWM8960, regdir, I2C_MEMADD_SIZE_8BIT,  val, I2C_DATSIZE, TOUT);
	res = HAL_I2C_Master_Transmit(i2cPortHandle,DeviceAdressWM8960,dataTX,sizeof(dataTX)/sizeof(uint8_t),TOUT);

	if (res != HAL_OK) {
		return -1;
	}
	*(storageVals+regdir)=value;

	return 0;
}


uint16_t i2cread_WM8960_port(uint8_t regdir, uint16_t *storageVals) {

	uint16_t val;
	val=*(storageVals+regdir);

	return val;
}

int8_t i2swrite_WM8960_port(uint16_t *pBuffer, uint32_t audSize) {

	HAL_StatusTypeDef res;

	res = HAL_I2S_Transmit_DMA(i2sportHandle, pBuffer, audSize );

	if (res != HAL_OK) {
		return -1;
	}

	return 0;

}

