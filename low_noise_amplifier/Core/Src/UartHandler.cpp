/*
 * UartHandler.cpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "UartHandler.hpp"

UartHandler::UartHandler(UART_HandleTypeDef *_huart,
		GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin,
		uint8_t _max_rx_buffer_size, uint8_t _max_tx_buffer_size) :
		huart(_huart), max_rx_buffer_size(_max_rx_buffer_size), // Enforce max size
		max_tx_buffer_size(_max_tx_buffer_size), // No validation assumed for TX
		rx_buffer(new uint8_t[max_rx_buffer_size]), tx_buffer(
				new uint8_t[max_tx_buffer_size]), data_enable_port(
				_data_enable_port), data_enable_pin(_data_enable_pin) // Initialize with passed value
{

	clearBuffers();
}

UartHandler::~UartHandler() {
	delete[] rx_buffer;
	delete[] tx_buffer;
}

void UartHandler::clearBuffers() {
	memset(rx_buffer, 0, max_rx_buffer_size);
	memset(tx_buffer, 0, max_tx_buffer_size);
	uart1_rcv_counter = 0;
	tx_buffer_size = 0;
	rxData = 0;
	isDataReady = false;
	command = 0;
}

void UartHandler::handleRxData(uint8_t data) {
	if (uart1_rcv_counter < max_rx_buffer_size) {
		rx_buffer[uart1_rcv_counter++] = data;
	}
}

bool UartHandler::prepareTxData(const char *message) {
	// 1. Calculate message length (avoid using sprintf for this)
	size_t message_length = std::strlen(message);

	// 2. Check if message fits within the buffer size
	if (message_length >= max_tx_buffer_size) {
		// Handle error: message too long
		// (e.g., return false, set an error flag)
		return (false);
	}

	// 3. Use safer string copy function (consider snprintf if needed)
	std::strncpy(reinterpret_cast<char*>(tx_buffer), message,
			max_tx_buffer_size);
	// Ensure null termination even if message length reaches max_tx_buffer_size
	tx_buffer[message_length] = '\0';

	// 4. Update tx_buffer_size with actual copied message length
	tx_buffer_size = message_length;

	return (true);
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
	rx_buffer[uart1_rcv_counter++] = rxData;
	if (rxData == 0x7F){
		isDataReady = true;
	}
	return (isDataReady);
}

