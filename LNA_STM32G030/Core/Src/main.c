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
#include <uart1.h>
#include "eeprom.h"
#include "utils.h"
#include "i2c1.h"
#include "stdbool.h"
#include "lna.h"
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

volatile uint16_t adcResultsDMA[4];
bool adcDataReady = false;
bool isPrintEnable = false;

uint32_t led_counter = 0;

volatile uint8_t UART1_rxBuffer[RX_UART1_BUFFLEN] = { 0 };
uint8_t uart1_rcv_counter = 0;
uint8_t rxData;
uint8_t UART1_txBuffer[TX_UART1_BUFFLEN] = { 0 };
uint8_t tx_buffer_size;
bool isDataReady = false;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

void uart_clean_buffer();

//void uart_reset_reading(UART_HandleTypeDef *huart);
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
	//__disable_irq();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* enable clock access ro GPIOA and GPIOB */
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

	/* PBA15 as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE15_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE15_1);

	/* PB1 as output */
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE1_0);
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE1_1);

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
//	MX_IWDG_Init();
	i2c1_init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */

// Calibrate The ADC On Power-Up For Better Accuracy
	//uint8_t addrs[5];
	//i2c1_scanner(addrs);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_UART_Receive_IT(&huart1, &rxData, 1);

	tx_buffer_size = sprintf((char*) UART1_txBuffer, "LNA init\n\r");
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
	UART_TRANSMIT_TIMEOUT);

	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
	uint8_t tries = 3;
	set_attenuation_to_bda4601(eeprom_1byte_read(LNA_ATT_ADDR), tries);

	if (eeprom_1byte_read(POUT_ISCALIBRATED_ADDR) != POUT_ISCALIBRATED) {
		eeprom_2byte_write(POUT_ADC_MIN_ADDR, POUT_ADC_MIN);
		eeprom_2byte_write(POUT_ADC_MAX_ADDR, POUT_ADC_MAX);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);
	led_counter = HAL_GetTick();
	uint32_t lna_print_counter = HAL_GetTick();
	set_attenuation_to_bda4601(eeprom_1byte_read(LNA_ATT_ADDR), 5);
	while (1) {

//Fin function 1 second
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (isDataReady) {
			struct Lna lna;
			uint8_t LTEL_FRAME_LENGTH = 14;
			uint8_t frame[LTEL_FRAME_LENGTH];
			switch (UART1_rxBuffer[3]) {
			case QUERY_PARAMETER_LTEL:
				lna = calulate_lna_real_values(adcResultsDMA);
				packet_lna_for_ltel_protocol(frame, lna);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, frame, LTEL_FRAME_SIZE,
				UART_TRANSMIT_TIMEOUT);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
				break;
			case SET_ATT_LTEL:
				uint8_t attenuation_value = UART1_rxBuffer[6];
				uint8_t tries = 2;
				set_attenuation_to_bda4601(attenuation_value, tries);
				eeprom_1byte_write(LNA_ATT_ADDR, attenuation_value);
				tx_buffer_size = sprintf((char*) UART1_txBuffer,
						"Attenuation %u\r\n", attenuation_value);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
				UART_TRANSMIT_TIMEOUT);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
				break;
			case SET_POUT_MAX:
				eeprom_2byte_write(POUT_ADC_MAX_ADDR,
						adcResultsDMA[POUT_INDEX]);
				HAL_Delay(5);
				eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
				tx_buffer_size = sprintf((char*) UART1_txBuffer,
						"Saved adc = %d as Pout 0 [dBm]\n\r",
						adcResultsDMA[POUT_INDEX]);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
				UART_TRANSMIT_TIMEOUT);
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

				break;
			case SET_POUT_MIN:
				eeprom_2byte_write(POUT_ADC_MIN_ADDR,
						adcResultsDMA[POUT_INDEX]);
				HAL_Delay(5);
				eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
				tx_buffer_size = sprintf((char*) UART1_txBuffer,
						"Saved adc = %d as Pout -30 [dBm]\n\r",
						adcResultsDMA[POUT_INDEX]);
				uart1_write_frame((char*) UART1_txBuffer, tx_buffer_size);
				break;
			case QUERY_PARAMETER_STR:
				isPrintEnable = !isPrintEnable;
				break;
			case QUERY_ADC:
				sprintf((char*) UART1_txBuffer,
						"Pout %d  \t Gain %u \t Curent %u \t Voltage %u\r\n",
						adcResultsDMA[POUT_INDEX], adcResultsDMA[GAIN_INDEX],
						adcResultsDMA[CURRENT_INDEX],
						adcResultsDMA[VOLTAGE_INDEX]);
				uart1_write_frame((char*) UART1_txBuffer, TX_UART1_BUFFLEN);
				break;
			case QUERY_PARAMETER_SIGMA:
				lna = calulate_lna_real_values(adcResultsDMA);
				sigma_set_parameter_frame(frame, lna);
				uart1_write_frame((char*) frame, LTEL_FRAME_SIZE);
				break;
			default:
				break;
			}
			isDataReady = false;
			uart_clean_buffer();
		} else {
			//lna_uart_read();
			isDataReady = lna_check_valid_str(UART1_rxBuffer);
		}

		if (isPrintEnable)
			if (HAL_GetTick() - lna_print_counter > LNA_PRINT_TIMEOUT) {
				struct Lna lna;
				lna = calulate_lna_real_values(adcResultsDMA);
				char *buffer = (char*) UART1_txBuffer;
				tx_buffer_size =
						sprintf(buffer,
								"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
								lna.pout, lna.attenuation, lna.gain, lna.pin,
								lna.current, (uint8_t) lna.voltage);

				HAL_GPIO_WritePin(DE_GPIO_Port,
				DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer, tx_buffer_size,
				UART_TRANSMIT_TIMEOUT);
				HAL_GPIO_WritePin(DE_GPIO_Port,
				DE_Pin, GPIO_PIN_RESET);
				lna_print_counter = HAL_GetTick();

			}

		if (HAL_GetTick() - led_counter > LED_STATE_TIMEOUT)
			led_counter = HAL_GetTick();
		else {
			if (HAL_GetTick() - led_counter > LED_ON_TIMEOUT)
				led_off();
			else
				led_on();
		}
		//HAL_IWDG_Refresh(&hiwdg);

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
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 2000;
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
	huart1.Init.BaudRate = 9600;
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
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			LE_ATTENUATOR_Pin | CLK_ATTENUATOR_Pin | DATA_ATTENUATOR_Pin
					| DE_Pin, GPIO_PIN_RESET);

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

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void uart_clean_buffer() {
	memset((uint8_t*) UART1_rxBuffer, 0, RX_UART1_BUFFLEN);

	memset(UART1_txBuffer, 0, TX_UART1_BUFFLEN);
	tx_buffer_size = 0;
	uart1_rcv_counter = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
// TODO : se puede reemplazar leyendo el flag del registro

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// Read received data from UART1

	/* Read received data from UART1 */
	if (uart1_rcv_counter >= RX_UART1_BUFFLEN) {
		memset((uint8_t*) UART1_rxBuffer, 0, RX_UART1_BUFFLEN);
		uart1_rcv_counter = 0;
	}
	HAL_UART_Receive_IT(&huart1, &rxData, 1);
	UART1_rxBuffer[uart1_rcv_counter++] = rxData;
	if (rxData == 0x7F)
		isDataReady = true;

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
