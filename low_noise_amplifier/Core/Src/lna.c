/*
 * lna.c
 *
 *  Created on: Jul 26, 2023
 *      Author: artur
 */
#include "lna.h"

Lna_t* lna_init() {

	Lna_t *lna;

	lna = malloc(sizeof(Lna_t));

	if (lna == NULL)
		Error_Handler();

	lna->adcDataReady = false;
	lna->isPrintEnable = false;
	lna->led_counter = 0;
	lna->attenuation = 0;
	lna->gain = 0;
	lna->pout = 0;
	lna->current = 0;
	lna->pin = 0;
	lna->voltage = 0;

	return (lna);
}

int8_t arduino_map(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t arduino_map16(uint16_t value, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint8_t get_dbm_pout_old(uint16_t pout_adc) {
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;

//	do {
	adc_max = eeprom_2byte_read(POUT_ADC_MAX_ADDR);
	HAL_Delay(2);
	adc_min = eeprom_2byte_read(POUT_ADC_MIN_ADDR);
//	} while (adc_max == adc_min);

	float m = (POUT_DBM_MAX - POUT_DBM_MIN) / (adc_max - adc_min);
	float b = POUT_DBM_MAX - adc_max * m;

	if (pout_adc > adc_max) {
		return (POUT_DBM_MAX);
	} else if (pout_adc < adc_min) {
		return (POUT_DBM_MIN);
	}

	return ((int8_t) (m * pout_adc + b));
}

void packet_lna_for_ltel_protocol(uint8_t *frame, struct Lna lna) {
	uint8_t crc_frame[2];
	uint16_t crc;
//	frame[0] = LTEL_START_MARK;
//	frame[1] = MODULE_FUNCTION;
//	frame[2] = MODULE_ADDR;
//	frame[3] = QUERY_PARAMETER_LTEL;
	frame[4] = 0x00;
	frame[5] = 0x05;
	frame[6] = 0x00;
	frame[7] = lna.attenuation;
	frame[8] = lna.gain;
	frame[9] = lna.pout;
	frame[10] = lna.voltage;
	crc = crc_get(&(frame[1]), 10);
	memcpy(crc_frame, &crc, 2);
	frame[11] = crc_frame[0];
	frame[12] = crc_frame[1];
//	frame[13] = LTEL_END_MARK;
}

void sigma_set_parameter_frame(uint8_t *frame, struct Lna lna) {
	uint8_t crc_frame[2];
	uint16_t crc;
//	frame[0] = LTEL_START_MARK;
//	frame[1] = MODULE_FUNCTION;
//	frame[2] = MODULE_ADDR;
//	frame[3] = QUERY_PARAMETER_SIGMA;
	frame[4] = 0x06;
	frame[5] = lna.pout;
	frame[6] = lna.attenuation;
	frame[7] = lna.gain;
	frame[8] = lna.current;
	frame[9] = lna.voltage;
	frame[10] = lna.pin;
	crc = crc_get(&(frame[1]), 10);
	memcpy(crc_frame, &crc, 2);
	frame[11] = crc_frame[0];
	frame[12] = crc_frame[1];
//	frame[13] = LTEL_END_MARK;
}

uint8_t get_db_gain_old(uint16_t adc_gain) {

	if (adc_gain >= 3781)
		return 45;
	else if (adc_gain < 3781 && adc_gain >= 1515)
		return 0.0022f * adc_gain + 36.6571f;
	else if (adc_gain < 1515 && adc_gain >= 1188)
		return (0.0153f * adc_gain + 16.8349f);
	else if (adc_gain < 1188 && adc_gain >= 1005)
		return (0.0273f * adc_gain + 2.540f);
	else if (adc_gain < 1005 && adc_gain >= 897)
		return (0.0463f * adc_gain - 16.5278f);
	else if (adc_gain < 897 && adc_gain >= 825)
		return (0.0694f * adc_gain - 37.2917f);
	else if (adc_gain < 825 && adc_gain >= 776)
		return (0.1020f * adc_gain - 64.1837f);
	else if (adc_gain < 776 && adc_gain >= 746)
		return (0.1667f * adc_gain - 114.333f);
	else if (adc_gain < 746 && adc_gain >= 733)
		return (0.3846f * adc_gain - 276.9231f);
	else if (adc_gain < 733 && adc_gain >= 725)
		return (0.625f * adc_gain - 453.125f);
	else if (adc_gain < 725)
		return 0;
	return 0;
}

uint8_t get_db_gain(uint16_t adc, uint8_t fix) {

	uint16_t adc_max = 3781;
	uint16_t adc_min = 725;
	uint8_t gain_min = 0;
	uint8_t gain_max = 45;
	uint8_t gain;

	gain = arduino_map(adc, adc_min, adc_max, gain_min, gain_max);
	return (gain);
}

uint8_t get_dbm_pout(uint16_t value, uint8_t fix) {
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;
	uint8_t pout;

//	do {
	adc_max = eeprom_2byte_read(POUT_ADC_MAX_ADDR);
	HAL_Delay(2);
	adc_min = eeprom_2byte_read(POUT_ADC_MIN_ADDR);
//	} while (adc_max == adc_min);
	pout = arduino_map(value, adc_min, adc_max, POUT_DBM_MIN, POUT_DBM_MAX);
	return (pout + fix);
}
uint8_t get_voltage(uint16_t value, uint8_t fix) {
	uint16_t adc_max = 2362;
	uint16_t adc_min = 1635;
	uint16_t voltage_min = 120;
	uint16_t voltage_max = 240;
	uint8_t voltage;

	voltage = arduino_map16(value, adc_min, adc_max, voltage_min, voltage_max);
	return (voltage);
}

uint8_t get_current(uint16_t value, uint8_t fix) {
	uint16_t adc_min = 2310;
	uint16_t adc_max = 2340;
	uint8_t current_max = 190;
	uint8_t current_min = 170;
	uint8_t current;

	current = arduino_map16(value, adc_min, adc_max, current_min, current_max);
	return (current);
}

struct Lna calulate_lna_real_values(volatile uint16_t *adc) {

	struct Lna lna;
    // Early return if ADC pointer is NULL
    if (!adc) {
        return (lna); // Return an empty struct
    }
	uint8_t pout_fix = 1;
	uint8_t gain_fix = -10;
	lna.pout = get_dbm_pout(adc[POUT_INDEX], pout_fix);
	lna.current = get_current(adc[CURRENT_INDEX], 0);
	lna.gain = get_db_gain(adc[GAIN_INDEX], gain_fix);
	lna.voltage = get_voltage(adc[VOLTAGE_INDEX], 0) / 10;
	lna.attenuation = eeprom_1byte_read(LNA_ATT_ADDR);
	lna.pin = lna.pout - lna.gain + lna.attenuation;
	return (lna);
}

void lna_uart_read(uint8_t *buffer) {
	uint8_t rcvcount = 0;
	uint32_t tickstart = HAL_GetTick();
	bool timeout = false;
	uint8_t timeout_value = 10;

	while (rcvcount < RX_UART1_BUFFLEN && timeout == false) {
		if (((HAL_GetTick() - tickstart) > timeout_value)
				|| (timeout_value == 0U)) {
			timeout = true;
			if (READ_BIT(USART1->ISR, USART_ISR_ORE))
				SET_BIT(USART1->ICR, USART_ICR_ORECF);
			if (READ_BIT(USART1->ISR, USART_ISR_IDLE))
				SET_BIT(USART1->ICR, USART_ICR_IDLECF);
			if (READ_BIT(USART1->ISR, USART_ISR_FE))
				SET_BIT(USART1->ICR, USART_ICR_FECF);
		}

		if (READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE))
			buffer[rcvcount++] = USART1->RDR;
	}
}

bool lna_check_valid_str(uint8_t *buffer) {
	if (!buffer)
		return (false); // Early return if buffer is NULL

//	if (buffer[0] != LTEL_START_MARK)
		return (false); // First byte check

//	if (buffer[1] != MODULE_FUNCTION)
		return (false); // Second byte check

	//if (buffer[2] != MODULE_ADDR)
		return (false); // Third byte check

	// Search for end mark efficiently using memchr
	//if (memchr(buffer + 3, LTEL_END_MARK, RX_UART1_BUFFLEN - 3) == NULL)
		return (false);

	return (true);
}

void set_attenuation_to_bda4601(uint8_t attenuation_value, uint8_t repetitions) {
	const uint8_t MAX_ATTENUATION = 31;
	const uint8_t BITS_TO_SEND = 6;

	// Validate and adjust the attenuation value if necessary
	attenuation_value =
			(attenuation_value > MAX_ATTENUATION) ? 0 : attenuation_value * 2;

	for (uint8_t i = 0; i < repetitions; i++) {
		uint8_t mask = 0b00100000;

		// Send the 6 configuration bits
		for (uint8_t j = 0; j < BITS_TO_SEND; j++) {
			bool is_bit_set = mask & attenuation_value;

			// Set the data pin based on the current bit
			HAL_GPIO_WritePin(GPIOA, DATA_ATTENUATOR_Pin,
					is_bit_set ? GPIO_PIN_SET : GPIO_PIN_RESET);

			// Set the clock pin high and then low
			HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_RESET);

			mask >>= 1; // Shift the mask one position to the right
		}

		// Set the LE pin high and then low
		HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_RESET);
	}
}

