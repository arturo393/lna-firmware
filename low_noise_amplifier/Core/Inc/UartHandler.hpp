/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#ifndef INC_UARTHANDLER_HPP_
#define INC_UARTHANDLER_HPP_

#include <cstring>
#include <stdbool.h>
#include "main.h"
#include <stdio.h>
#include <string.h>

class UartHandler {
public:
	UartHandler(UART_HandleTypeDef *_huart, GPIO_TypeDef *_data_enable_port,
			uint16_t _data_enable_pin, uint8_t _max_rx_buffer_size,
			uint8_t _max_tx_buffer_size);
	~UartHandler();

	void clearBuffers();
	void handleRxData(uint8_t data);
	bool transmitMessage(const char *message);
	bool transmitData(uint8_t *data, uint8_t data_bytes);
	bool receive_it();

	bool isDataReady;
	uint8_t command;

private:
	UART_HandleTypeDef *huart;
	uint8_t max_rx_buffer_size;
	uint8_t max_tx_buffer_size;
	uint8_t *rx_buffer;
	uint8_t uart1_rcv_counter;
	uint8_t *tx_buffer;
	uint8_t tx_buffer_size;
	uint8_t rxData;
	GPIO_TypeDef *data_enable_port;
	uint16_t data_enable_pin;
	bool start_byte;

	static constexpr uint8_t LTEL_START_MARK = 0x7e;
	static constexpr uint8_t LTEL_END_MARK = 0x7f;
    static constexpr uint8_t MODULE_ADDR = 0x08;
    static constexpr uint8_t MODULE_FUNCTION = 0x09;



	bool prepareTxData(const char *message);
};

#endif /* INC_UARTHANDLER_HPP_ */
