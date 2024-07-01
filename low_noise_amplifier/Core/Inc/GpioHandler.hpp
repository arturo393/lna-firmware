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
	GpioHandler(int _ports, int _pins);
	virtual ~GpioHandler();
	void turnOn(Gpio gpio);
	void turnOff(Gpio gpio);
	void turnOnWaitOff(Gpio gpio, int wait_ms);

private:
	void switch_state(Gpio gpio, GPIO_PinState state);
	int ports;
	int pins;
};

#endif /* SRC_GPIOCONTROLLER_HPP_ */
