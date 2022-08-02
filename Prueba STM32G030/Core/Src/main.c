/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "eeprom.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

#define RX_UART1_BUFFLEN  40

static const uint8_t MINIMUN_FRAME_SIZE = 7;
static const uint8_t MODULE_ADDR = 0x08;
static const uint8_t MODULE_FUNCTION = 0x09;

static const uint8_t QUERY_PARAMETER_LTEL = 0x11;
static const uint8_t QUERY_PARAMETER_STR = 0x15;
static const uint8_t QUERY_ADC = 0x16;

static const uint8_t SET_ATT_LTEL = 0x20;
static const uint8_t SET_POUT_MAX = 0x24;
static const uint8_t SET_POUT_MIN = 0x23;

//static const float LNA_VREF = 3.28f; // volts * 1000
//static const float LNA_CURRENT_FACTOR = 2.2f/200.0f;  // 2,2 [v] / 20static uint8_t0 [A]
//static const float LNA_CURRENT_MULTIPLIER = (LNA_VREF)/(LNA_CURRENT_FACTOR);
static const float ADC_CURRENT_FACTOR = 298.1818182f;
static const float ADC_VOLTAGE_FACTOR = 0.007404330f;

static const uint8_t LTEL_START_MARK = 0x7e;
static const uint8_t LTEL_END_MARK = 0x7f;



//static const uint8_t UART_DATA_ARRIVED = 0x20;

static const uint8_t POUT_DBM_MAX = 0;
static const uint8_t POUT_DBM_MIN = -30;
static const uint16_t POUT_ADC_MAX = 1833;
static const uint16_t POUT_ADC_MIN = 488;

static uint8_t LNA_ATT_ADDR = 0x00;
//static uint8_t POUT_DBM_MIN_ADDR = 0x01;
//static uint8_t POUT_DBM_MAX_ADDR = 0x02;
static uint8_t POUT_ADC_MAX_ADDR = 0x03;
static uint8_t POUT_ADC_MIN_ADDR = 0x05;
static uint8_t POUT_ISCALIBRATED_ADDR = 0x07;
//static uint8_t POUT_DBM_ADDR = 0x08;

static uint8_t POUT_ISCALIBRATED = 0xAA;

struct Lna {
	uint8_t attenuation;
	uint8_t gain;
	int8_t pout;
	uint8_t current;
	int8_t pin;
	float voltage;
};


volatile uint16_t adcResultsDMA[4];
bool IS_CONVERSION_COMPLETE = false;
uint8_t uart_readed_bytes;
uint8_t UART1_rxBuffer[RX_UART1_BUFFLEN] = { 0 };

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);
	IS_CONVERSION_COMPLETE = true;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

uint8_t ltel_get_start_index(uint8_t *rxBuffer);
void ltel_set_parameter_frame(uint8_t *frame, struct Lna lna);
uint8_t get_db_gain(uint16_t adc_gain);
uint8_t get_dbm_pout(uint16_t pout_adc);
struct Lna get_lna();
void set_attenuation(uint8_t attenuation, uint8_t times);
void uart_send_frame(uint8_t *str, uint8_t len);
void uart_send_str(uint8_t *str);
void uart_reset_reading(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	MX_DMA_Init();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	// MX_IWDG_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */

	// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1);
	uart_send_str((uint8_t*) "LNA init\n\r");
	set_attenuation(EEPROM_Read(LNA_ATT_ADDR), 3);

	if (EEPROM_Read(POUT_ISCALIBRATED_ADDR) != POUT_ISCALIBRATED) {
		EEPROM_2byte_Write(POUT_ADC_MIN_ADDR, POUT_ADC_MIN);
		EEPROM_2byte_Write(POUT_ADC_MAX_ADDR, POUT_ADC_MAX);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);

	while (1) {

		//Fin function 1 second
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, RX_UART1_BUFFLEN);
		uart_readed_bytes = RX_UART1_BUFFLEN - (huart1.hdmarx->Instance->CNDTR);

		if (uart_readed_bytes > MINIMUN_FRAME_SIZE) {

			HAL_UART_DMAPause(&huart1);
			uart_reset_reading(&huart1);

			uint8_t start_index = ltel_get_start_index(UART1_rxBuffer);

			if (start_index != (uint8_t) -1) {

				uint8_t module_function_index = UART1_rxBuffer[start_index + 1];
				uint8_t module_address_index = UART1_rxBuffer[start_index + 2];

				if (module_function_index == MODULE_FUNCTION
						&& module_address_index == MODULE_ADDR) {
					uint8_t lna_cmd = UART1_rxBuffer[start_index + 3];

					if (lna_cmd == QUERY_PARAMETER_LTEL) {
						struct Lna lna;
						uint8_t frame[14];
						lna = get_lna();
						ltel_set_parameter_frame(frame, lna);
						uart_send_frame(frame, 14);

					} else if (lna_cmd == SET_ATT_LTEL) {
						uint8_t attenuation_value = UART1_rxBuffer[start_index
								+ 6];
						set_attenuation(attenuation_value, 3);
						EEPROM_Write(LNA_ATT_ADDR, attenuation_value);
					} else if (lna_cmd == SET_POUT_MAX) {

						EEPROM_2byte_Write(POUT_ADC_MAX_ADDR,
								adcResultsDMA[0]);
						HAL_Delay(5);
						EEPROM_Write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
						uart_send_str((uint8_t*) "Saved Pout max value\n\r");

					} else if (lna_cmd == SET_POUT_MIN) {

						EEPROM_2byte_Write(POUT_ADC_MIN_ADDR,
								adcResultsDMA[0]);
						HAL_Delay(5);
						EEPROM_Write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
						uart_send_str((uint8_t*) "Saved Pout min value\n\r");

					} else if (lna_cmd == QUERY_PARAMETER_STR) {

						struct Lna lna = get_lna();
						uint8_t str[100];
						sprintf(str,
								"Pout %d [dBm] \t Att %u [dB] \t Gain %u [dB] \t Pin %d [dBm] \t  \t Curent %d [mA] \t Voltage %u [V]\r\n",
								lna.pout, lna.attenuation, lna.gain, lna.pin,
								lna.current, (uint8_t) lna.voltage);
						uart_send_str(str);

					} else if (lna_cmd == QUERY_ADC) {

						uint8_t str[80];
						sprintf(str,
								"Pout %d \t Att %d \t Gain %u \t Curent %d \t Voltage %d\r\n",
								adcResultsDMA[0], adcResultsDMA[1],
								adcResultsDMA[2], adcResultsDMA[3]);
						uart_send_str(str);

					}

				}
				start_index = -1;
			}
			HAL_UART_DMAResume(&huart1);
		}

		if (IS_CONVERSION_COMPLETE) {
			// TODO: AGREGAR LECTURAS PROMEDIOS
			IS_CONVERSION_COMPLETE = false;
		}
		//HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(5); // for UART1_rx buffer complete readings

	}   //Fin while
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_12CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00602173;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 800;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 19200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* DMA1_Ch4_5_DMAMUX1_OVR_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Ch4_5_DMAMUX1_OVR_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	LE_ATTENUATOR_Pin | CLK_ATTENUATOR_Pin | DATA_ATTENUATOR_Pin | DE_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LE_ATTENUATOR_Pin CLK_ATTENUATOR_Pin DATA_ATTENUATOR_Pin DE_Pin */
	GPIO_InitStruct.Pin = LE_ATTENUATOR_Pin | CLK_ATTENUATOR_Pin
			| DATA_ATTENUATOR_Pin | DE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t ltel_get_start_index(uint8_t *rxBuffer) {
	uint8_t start_index = -1;
	for (uint8_t i = 0; i < RX_UART1_BUFFLEN - MINIMUN_FRAME_SIZE; i++) {
		if (start_index == (uint8_t) -1) {
			if (rxBuffer[i] == (uint8_t) LTEL_START_MARK)
				start_index = rxBuffer[i + 1] == MODULE_FUNCTION ? i : -1;
			else
				start_index = -1;
		} else {
			if (rxBuffer[i] == (uint8_t) LTEL_END_MARK) {
				if (i > MINIMUN_FRAME_SIZE) {
					uint16_t crc_rcv;
					uint16_t crc_calc;
					uint8_t data_start_index = start_index + 1;
					crc_calc = crc_get(&(rxBuffer[data_start_index]),
							i - data_start_index - 2);
					crc_rcv = rxBuffer[i - 1] << 8 | rxBuffer[i - 2];
					return crc_calc == crc_rcv ? start_index : -1;
				}
			}
		}
	}
	return -1;
}

void ltel_set_parameter_frame(uint8_t *frame, struct Lna lna) {
	uint8_t crc_frame[2];
	uint16_t crc;
	frame[0] = LTEL_START_MARK;
	frame[1] = MODULE_FUNCTION;
	frame[2] = MODULE_ADDR;
	frame[3] = QUERY_PARAMETER_LTEL;
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
	frame[13] = LTEL_END_MARK;
}

uint8_t get_db_gain(uint16_t adc_gain) {

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

uint8_t get_dbm_pout(uint16_t pout_adc) {
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;
	float m, b;

	do {
		adc_max = EEPROM_2byte_Read(POUT_ADC_MAX_ADDR);
		adc_min = EEPROM_2byte_Read(POUT_ADC_MIN_ADDR);
	} while (adc_max == adc_min);

	m = (POUT_DBM_MAX - POUT_DBM_MIN) / (float) (adc_max - adc_min);
	b = POUT_DBM_MAX - adc_max * m;

	if (pout_adc > adc_max) {
		return POUT_DBM_MAX;
	} else if (pout_adc < adc_min) {
		return POUT_DBM_MIN;
	}

	return (int8_t) (m * (float) pout_adc + b);
}

struct Lna get_lna() {

	struct Lna lna;
	lna.pout = get_dbm_pout(adcResultsDMA[0]);
	lna.current = ADC_CURRENT_FACTOR * adcResultsDMA[1] / 4096.0f;
	lna.gain = get_db_gain(adcResultsDMA[2]);
	lna.voltage = ADC_VOLTAGE_FACTOR * (float) adcResultsDMA[3];
	lna.attenuation = EEPROM_Read(LNA_ATT_ADDR);
	lna.pin = lna.pout - lna.gain + lna.attenuation;
	return lna;
}

void set_attenuation(uint8_t attenuation, uint8_t times) {

	if (attenuation < 0 || attenuation > 31) {
		attenuation = 0;
	}
	attenuation *= 2;
	for (uint8_t i = 0; i < times; i++) {
		uint8_t mask = 0b00100000;
		for (uint8_t j = 0; j < 6; j++) {
			//Ciclo for de 6 vueltas para enviar los 6bits de configuración
			if (mask & attenuation) {
				//Si el bit de la mascara en 1 coincide con el bit del valor, entonces
				HAL_GPIO_WritePin(GPIOA, DATA_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin data en alto
			} else {
				HAL_GPIO_WritePin(GPIOA, DATA_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin data en bajo
			}
			HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin clock en alto
			HAL_Delay(1); //Delay de 1mS
			HAL_GPIO_WritePin(GPIOA, CLK_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin clock en bajo
			mask = mask >> 1; //Muevo la máscara una posición
		}
		HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_SET); //Pin LE en alto
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, LE_ATTENUATOR_Pin, GPIO_PIN_RESET); //Pin LE en bajo
	}

}

void uart_send_frame(uint8_t *str, uint8_t len) {
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, str, len, 100);
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_RESET);
}

void uart_send_str(uint8_t *str) {
	uint8_t i;
	for (i = 0; str[i] != '\0'; i++)
		;
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, str, i, 100);
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_RESET);
}

void uart_reset_reading(UART_HandleTypeDef *huart) {
	huart->hdmarx->Instance->CCR &= ~DMA_CCR_EN; // disable
	huart->hdmarx->Instance->CNDTR = RX_UART1_BUFFLEN; // reset counter
	huart->hdmarx->Instance->CCR |= DMA_CCR_EN; // re-enable
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
