/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLERHAL_HPP_
#define INC_UARTHANDLERHAL_HPP_

#include <cstring>
#include <UartHandler.hpp>
#include "main.h"

class UartHandlerBareMetal : public UartHandler{
public:
	UartHandlerBareMetal(){rxdata = false;}
	~UartHandlerBareMetal(){}
	void init(USART_TypeDef * USART,GPIO_TypeDef *_data_enable_port,
	uint16_t _data_enable_pin);
	void wait_for_it_byte() override;
	uint8_t getByte();
	void readCommand(CommandMessage& c) override;

private:
	USART_TypeDef * usart;
	uint8_t rxData;
	GPIO_TypeDef *data_enable_port;
	uint16_t data_enable_pin;
	uint32_t rxfne;
	bool rxdata;
	void uart_gpio_init() override;
	void uart_init(uint32_t pclk, uint32_t baud_rate) override;
	void uart_write(char* ch) override;
};

#endif /* INC_UARTHANDLER_HPP_ */
