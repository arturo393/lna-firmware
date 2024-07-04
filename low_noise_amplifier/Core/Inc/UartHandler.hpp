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
public:
	UartHandler();
	~UartHandler();
	virtual bool transmitMessage(const char *message);
	virtual bool transmitData(uint8_t *data, uint8_t data_bytes);
	virtual void wait_for_it_byte();
};

#endif /* INC_UARTHANDLER_HPP_ */
