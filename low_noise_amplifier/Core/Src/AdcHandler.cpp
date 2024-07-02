/*
 * AdcHandler.cpp
 *
 *  Created on: Jul 2, 2024
 *      Author: artur
 */

#include <AdcHandler.hpp>

AdcHandler::AdcHandler(ADC_HandleTypeDef* _hadc,uint32_t _channel) {
	hadc = _hadc;
	channel = _channel;
}

AdcHandler::~AdcHandler() {
	// TODO Auto-generated destructor stub
}

uint16_t AdcHandler::getChannelValue(){

	setChannel();
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	uint16_t value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);
	return (value);
}

void AdcHandler::setChannel() {
	/** Configure Regular Channel
	 */ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

