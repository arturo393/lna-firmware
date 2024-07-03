/*
 * UartHandler.cpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "UartHandler.hpp"

UartHandler::UartHandler(UART_HandleTypeDef *_huart,
		GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin) {

	huart = _huart;
	data_enable_port = _data_enable_port;
	data_enable_pin = _data_enable_pin; // Initialize with passed value
	clearBuffers();
}

UartHandler::~UartHandler() {
}

bool UartHandler::transmitData(uint8_t *data, uint8_t data_bytes) {
	HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_SET);
	if (HAL_UART_Transmit(huart, data, data_bytes,
	HAL_MAX_DELAY) == HAL_OK) {
		return (true);
		HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(data_enable_port, data_enable_pin, GPIO_PIN_RESET);
	return (false);
}

/* Read received data from UART1 */
void UartHandler::wait_for_it_byte() {
	HAL_UART_Receive_IT(huart, &rxData, 1);
}

uint8_t UartHandler::getByte() {
	return (rxData);
}

