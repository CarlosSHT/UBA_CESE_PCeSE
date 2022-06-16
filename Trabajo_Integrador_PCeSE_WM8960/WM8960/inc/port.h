/*
 * port.h
 *
 *  Created on: Jun 16, 2022
 *      Author: carlo
 */

#ifndef INC_PORT_H_
#define INC_PORT_H_

#include "stdint.h"

#define TOUT	10
#define	I2C_DATSIZE		1

int8_t i2cInstDev_WM8960_port(uint8_t numI2Cport,uint8_t WM8960_addr);
int8_t i2sInst_WM8960_port(uint8_t numI2Sport);

int8_t i2cwrite_WM8960_port(uint8_t regdir, uint16_t value, uint16_t *storageVals);
uint16_t i2cread_WM8960_port(uint8_t regdir, uint16_t *storageVals);

int8_t i2swrite_WM8960_port(uint16_t *pBuffer, uint32_t audSize);
#endif /* INC_PORT_H_ */
