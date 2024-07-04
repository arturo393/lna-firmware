/*
 * GpioController.hpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#ifndef SRC_GPIOCONTROLLER_HPP_
#define SRC_GPIOCONTROLLER_HPP_

#include "main.h"
#include "Gpio.hpp"

class GpioHandler {
public:

	GpioHandler(uint8_t _ports = 0, uint8_t _pins = 0);
	virtual ~GpioHandler();
	void turnOn(Gpio gpio);
	void turnOff(Gpio gpio);
	void turnOnWaitOff(Gpio gpio, int wait_ms);

private:
	void switch_state(Gpio gpio, GPIO_PinState state);
	uint8_t ports;
	uint8_t pins;
};

#endif /* SRC_GPIOCONTROLLER_HPP_ */
