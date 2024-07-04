/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLERHAL_HPP_
#define INC_UARTHANDLERHAL_HPP_

#include <Command.hpp>
#include <cstring>
#include <UartHandler.hpp>
#include "main.h"

class UartHandlerBareMetal : UartHandler{
public:
	UartHandlerBareMetal(){rxdata = false;}
	~UartHandlerBareMetal(){}
	void init(USART_TypeDef * USART,GPIO_TypeDef *_data_enable_port,
	uint16_t _data_enable_pin);
	bool transmitMessage(const char *message) override;
	bool transmitData(uint8_t *data, uint8_t data_bytes) override;
	void wait_for_it_byte() override;
	uint8_t getByte();
	void uart1_send_str(const char *str);

private:
	USART_TypeDef * _USART;
	uint8_t rxData;
	GPIO_TypeDef *data_enable_port;
	uint16_t data_enable_pin;
	uint32_t rxfne;
	bool rxdata;
	void uart1_gpio_init();
	void uart1_init(uint32_t pclk, uint32_t baud_rate);
	void uart1_write(char ch);
	void uart1_read(char *data, uint8_t size);
	char uart1_1byte_read(void);
	void uart1_write_frame(char *str, uint8_t len);

	//void uart1_send_str(volatile char *str);
};

#endif /* INC_UARTHANDLER_HPP_ */
