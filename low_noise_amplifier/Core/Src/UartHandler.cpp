/*
 * UartHandler.cpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "UartHandler.hpp"

UartHandler::UartHandler(const Command &_command, UART_HandleTypeDef *_huart,
		GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin) {

	command = _command;
	huart = _huart;
	data_enable_port = _data_enable_port;
	data_enable_pin = _data_enable_pin; // Initialize with passed value
	clearBuffers();
}

UartHandler::~UartHandler() {
}

void UartHandler::clearBuffers() {

	uart1_rcv_counter = 0;

	rxData = 0;
	isDataReady = false;
	start_byte = false;
}



bool UartHandler::transmitData(uint8_t *data, uint8_t data_bytes) {
	HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_SET);
	if (HAL_UART_Transmit(huart, data, data_bytes,
	HAL_MAX_DELAY) == HAL_OK) {
		this->clearBuffers();  // Clear buffers after successful transmission
		return (true);
		HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_SET);
	return (false);
}

bool UartHandler::transmitMessage(const char *message) {
	this->prepareTxData(message);
	return (this->transmitData(tx_buffer, tx_buffer_size));
}

/* Read received data from UART1 */
bool UartHandler::receive_it() {

	if (uart1_rcv_counter >= max_rx_buffer_size) {
		memset(rx_buffer, 0, max_rx_buffer_size);
		uart1_rcv_counter = 0;
	}

	HAL_UART_Receive_IT(huart, &rxData, 1);
	if (rxData == 0x7E)
		start_byte = true;

	if (start_byte) {
		rx_buffer[uart1_rcv_counter++] = rxData;
		if (rxData == 0x7F) {
			isDataReady = true;
		}
	}

	return (isDataReady);
}

