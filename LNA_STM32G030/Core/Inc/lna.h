/*
 * lna.h
 *
 *  Created on: Jul 26, 2023
 *      Author: artur
 */

#ifndef INC_LNA_H_
#define INC_LNA_H_

#include "main.h"
#include "eeprom.h"
#include "uart1.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "utils.h"

#define SYS_FREQ 64000000
#define APB_FREQ SYS_FREQ
#define HS16_CLK 16000000
#define BAUD_RATE 115200
#define LED_PIN PB1
#define DE_PIN PA15

#define led_off() CLEAR_BIT(GPIOB->ODR,GPIO_ODR_OD1)
#define led_on() SET_BIT(GPIOB->ODR,GPIO_ODR_OD1)

#define RX_UART1_BUFFLEN  25
#define TX_UART1_BUFFLEN  100
#define LTEL_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14
#define MEDIA_NUM 20

#define MODULE_ADDR  0x08
#define MODULE_FUNCTION  0x09

#define QUERY_PARAMETER_LTEL  0x11
#define QUERY_PARAMETER_SIGMA  0x12
#define QUERY_PARAMETER_STR  0x15
#define QUERY_ADC  0x16

#define SET_ATT_LTEL  0x20
#define SET_POUT_MAX  0x24
#define SET_POUT_MIN  0x23

#define ADC_CURRENT_FACTOR  298.1818182f
#define ADC_VOLTAGE_FACTOR  0.007404330f

#define LTEL_START_MARK  0x7e
#define LTEL_END_MARK  0x7f

#define POUT_DBM_MAX  -2
#define POUT_DBM_MIN  -30
#define POUT_ADC_MAX  1833
#define POUT_ADC_MIN  488

#define LNA_ATT_ADDR  0x00
#define POUT_ADC_MAX_ADDR  0x03
#define POUT_ADC_MIN_ADDR  0x05
#define POUT_ISCALIBRATED_ADDR  0x07

#define POUT_ISCALIBRATED  0xAA

#define LNA_PRINT_TIMEOUT 2000
#define UART_TRANSMIT_TIMEOUT 1000
#define LED_STATE_TIMEOUT  1000
#define LED_ON_TIMEOUT  50

typedef enum {
	POUT_INDEX, CURRENT_INDEX, GAIN_INDEX, VOLTAGE_INDEX, ADC_READINGS
} ADC_INDEX;

typedef struct Lna {
	uint8_t attenuation;
	uint8_t gain;
	int8_t pout;
	uint8_t current;
	int8_t pin;
	uint8_t voltage;

	volatile uint16_t adcResultsDMA[ADC_READINGS];
	bool adcDataReady;
	bool isPrintEnable;

	uint32_t led_counter;

} Lna_t;

void print_lna_to_uart1();
int8_t arduino_map(uint16_t value, uint16_t in_min, uint16_t in_max,
		int8_t out_min, int8_t out_max);
uint8_t get_dbm_pout_old(uint16_t pout_adc);

// 1. Frame creation functions:
void packet_lna_for_ltel_protocol(uint8_t *frame, struct Lna lna);
void sigma_set_parameter_frame(uint8_t *frame, struct Lna lna);
// 2. Value calculation functions:
uint8_t get_db_gain(uint16_t adc_gain, uint8_t fix);
uint8_t get_dbm_pout(uint16_t value, uint8_t fix);
struct Lna calulate_lna_real_values(volatile uint16_t *adc);
uint8_t get_voltage(uint16_t value, uint8_t fix);
uint8_t get_current(uint16_t value, uint8_t fix);
// 3. Command handling functions:
void lna_cmd_action();
//4. Communication functions:
void lna_uart_read();
bool lna_check_valid_str();
//Hardware interaction functions:
void set_attenuation_to_bda4601(uint8_t attenuation, uint8_t times);


#endif /* INC_LNA_H_ */
