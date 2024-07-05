/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLERHAL_HPP_
#define INC_UARTHANDLERHAL_HPP_

#ifdef UART_HAL
#include <Command.hpp>
#include <cstring>
#include <UartHandler.hpp>
#include "main.h"


class UartHandlerHAL : UartHandler {
public:
	void init(UART_HandleTypeDef *_huart,
				GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin);
	bool transmitMessage(const char *message);
	bool transmitData(uint8_t *data, uint8_t data_bytes);
	void wait_for_it_byte();
	uint8_t getByte();
private:
	UART_HandleTypeDef* huart;
	GPIO_TypeDef* data_enable_port;
	uint16_t data_enable_pin;
	uint8_t rx_byte;
};

#endif

#endif /* INC_UARTHANDLER_HPP_ */
