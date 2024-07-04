/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLER_HPP_
#define INC_UARTHANDLER_HPP_

#include <Command.hpp>
#include <cstring>
#include "main.h"

class UartHandler {
public:
	UartHandler(UART_HandleTypeDef *_huart,
			GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin);
	~UartHandler();

	virtual void handleRxData(uint8_t data);
	virtual bool transmitMessage(const char *message);
	virtual bool transmitData(uint8_t *data, uint8_t data_bytes);
	virtual void wait_for_it_byte();
	virtual uint8_t getByte();
};

#endif /* INC_UARTHANDLER_HPP_ */
