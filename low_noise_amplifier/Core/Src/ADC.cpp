/*
 * ADC.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <main.h>

class ADC{
public:
	ADC();
	virtual ~ADC();
	void read();
	void setUp();

private:
	uint16_t voltage;
	uint16_t current;
	uint8_t rf_gain;
	uint8_t rf_power;
};

