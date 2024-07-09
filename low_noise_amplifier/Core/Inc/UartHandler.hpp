/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLER_HPP_
#define INC_UARTHANDLER_HPP_

#include "main.h"
#include "CommandMessage.hpp"

class UartHandler {
public:
	virtual bool transmitMessage(char* message);
	virtual bool transmitData(uint8_t *data, uint8_t data_bytes);
	virtual bool transmitCommand(CommandMessage command);
	virtual void readCommand(CommandMessage& c)=0;
protected:
	virtual ~UartHandler()=0;
	virtual void wait_for_it_byte()=0;
	virtual void uart_gpio_init()=0;
	virtual void uart_init(uint32_t pclk, uint32_t baud_rate)=0;
	virtual void uart_write(char* ch)=0;
};

inline UartHandler::~UartHandler() {}

#endif /* INC_UARTHANDLER_HPP_ */
