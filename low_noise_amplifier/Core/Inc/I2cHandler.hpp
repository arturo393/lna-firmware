/*
 * I2cHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_I2cHandler_HPP_
#define INC_I2cHandler_HPP_

#include "main.h"

#define WRITE 0
#define READ 1

class I2cHandler {
	public:
		virtual uint8_t byteReceive(char address, uint8_t bytes_to_read)=0;
		virtual void byteTransmit(char address, char* buffer, uint8_t bytes_to_write)=0;
		virtual void i2c1_init()=0;
	protected:
		virtual ~I2cHandler()=0;
};

inline I2cHandler::~I2cHandler() {
}

#endif /* INC_I2cHandler_HPP_ */
