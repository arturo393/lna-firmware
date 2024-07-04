/*
 * RFAttenuator.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include "RFAttenuator.hpp"

RFAttenuator::RFAttenuator(GpioHandler* _gpioHandler, Gpio* _data_pin, Gpio* _clock_pin, Gpio* _le_pin) {
	gpioHandler = _gpioHandler;
	data_pin = _data_pin;
	clock_pin = _clock_pin;
	le_pin = _le_pin;
}

RFAttenuator::~RFAttenuator() {
	// TODO Auto-generated destructor stub
}

void RFAttenuator::attenuate(int value) {
	const uint8_t MAX_ATTENUATION = 31;
	const uint8_t BITS_TO_SEND = 6;

	// Validate and adjust the attenuation value if necessary
	value = (value > MAX_ATTENUATION) ? 0 : value * 2;

	for (uint8_t i = 0; i < 3; i++) {
		uint8_t mask = 0b00100000;

		// Send the 6 configuration bits
		for (uint8_t j = 0; j < BITS_TO_SEND; j++) {
			bool is_bit_set = mask & value;
			// Set the data pin based on the current bit
			if (is_bit_set) {
				gpioHandler->turnOn(*data_pin);
			} else {
				gpioHandler->turnOff(*data_pin);
			}
			// Set the clock pin high and then low
			gpioHandler->turnOnWaitOff(*clock_pin, 1);

			mask >>= 1; // Shift the mask one position to the right
		}

		// Set the LE pin high and then low
		gpioHandler->turnOnWaitOff(*le_pin, 1);
	}
}
