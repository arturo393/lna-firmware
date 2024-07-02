/*
 * RFAttenuator.hpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#ifndef SRC_RFATTENUATOR_HPP_
#define SRC_RFATTENUATOR_HPP_

#include <Gpio.hpp>
#include <GpioHandler.hpp>

class RFAttenuator {
public:
	RFAttenuator(GpioHandler _gpioHandler, Gpio _data_pin, Gpio _clock_pin, Gpio _le_pin);
	//RFAttenuator(GpioHandler _gpioHandler);
	virtual ~RFAttenuator();

	void attenuate(int value);
private:
	GpioHandler gpioHandler;
	Gpio data_pin;
	Gpio clock_pin;
	Gpio le_pin;
};

#endif /* SRC_RFATTENUATOR_HPP_ */
