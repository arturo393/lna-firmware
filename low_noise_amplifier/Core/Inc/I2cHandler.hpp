/*
 * I2cHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_I2cHandler_HPP_
#define INC_I2cHandler_HPP_

#include "main.h"
#include "CommandMessage.hpp"

#define WRITE 0
#define READ 1

class I2cHandler {
public:
	char byteReceive(char, uint8_t);
	void byteTransmit(char, char*, uint8_t);
	void i2c1_init();
protected:
	virtual ~I2cHandler()=0;
	void startComunication(char, uint8_t, uint8_t); // solo del baremetal
	void scanner(uint8_t *addr); // solo del baremetal
};

inline I2cHandler::~I2cHandler() {
}

#endif /* INC_I2cHandler_HPP_ */
