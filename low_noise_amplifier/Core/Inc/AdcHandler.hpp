/*
 * AdcHandler.h
 *
 *  Created on: Jul 2, 2024
 *      Author: artur
 */

#ifndef INC_ADCHANDLER_HPP_
#define INC_ADCHANDLER_HPP_
#include "main.h"

class AdcHandler {
public:
	AdcHandler(ADC_HandleTypeDef*_hadc, uint32_t _channel);
	virtual ~AdcHandler();
	uint16_t getChannelValue();

private:
	void setChannel();
	ADC_HandleTypeDef *hadc;
	uint32_t channel;
};

#endif /* INC_ADCHANDLER_HPP_ */
