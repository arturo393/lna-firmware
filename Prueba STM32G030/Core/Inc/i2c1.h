/*
 * i2c.h
 *
 *  Created on: Aug 16, 2022
 *      Author: sigmadev
 */

#include "stm32g0xx.h"
#include "main.h"

#ifndef INC_I2C1_H_
#define INC_I2C1_H_

#define WRITE 0
#define READ 1

void i2c1_init();
void  i2c1_start(char,uint8_t,uint8_t);
char i2c1_byteReceive(char ,uint8_t);
void i2c1_byteTransmit(char,char*,uint8_t);

#endif /* INC_I2C1_H_ */
