/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLER_HPP_
#define INC_UARTHANDLER_HPP_

#include "main.h"

class UartHandler {
protected:
	virtual ~UartHandler()=0;
	virtual bool transmitMessage(const char *message)=0;
	virtual bool transmitData(uint8_t *data, uint8_t data_bytes)=0;
	virtual void wait_for_it_byte()=0;
};

inline UartHandler::~UartHandler() {}

#endif /* INC_UARTHANDLER_HPP_ */
