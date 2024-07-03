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
#include "utils.h"
#include "i2c1.h"
#include "stdbool.h"
#include "lna.h"
#include <UartHandler.hpp>
#include <Command.hpp>
#include <Gpio.hpp>
#include <GpioHandler.hpp>
#include <RfAttenuator.hpp>
#include <AdcHandler.hpp>
#include <Memory.hpp>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD
 #define LNA_ATT_KEY 1
 #define POUT_ADC_MAX_KEY 2
 #define POUT_ADC_MIN_KEY 3
 #define POUT_ISCALIBRATED_KEY 4
 USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bool adcDataReady = false;
bool isPrintEnable = false;

uint32_t led_counter = 0;

#define MODULE_FUNCTION 0x09
#define MODULE_ADDRESS 0x08

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct Lna lna;
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
//	MX_IWDG_Init();
	/* memory ok
	 * attenuator ok
	 * gpio ok
	 * gpio handler ok
	 * adcHandler
	 */

	Command command = Command(MODULE_FUNCTION,MODULE_ADDRESS);
	UartHandler myUart(&huart1, DE_GPIO_Port, DE_Pin);

	Memory eeeprom = Memory(&hi2c1);
	uint8_t lna_att_key = eeeprom.createKey(LNA_ATT_ADDR, sizeof(uint8_t));
	uint8_t pout_adc_max_key = eeeprom.createKey(POUT_ADC_MAX_ADDR,
			sizeof(uint16_t));
	uint8_t pout_adc_min_key = eeeprom.createKey(POUT_ADC_MIN_ADDR,
			sizeof(uint16_t));
	uint8_t pout_iscalibrated_key = eeeprom.createKey(POUT_ISCALIBRATED_ADDR,
			sizeof(uint8_t));
	uint8_t att_flag_key = eeeprom.createKey(ATT_FLAG_ADDR, sizeof(uint8_t));

	Gpio data_pin(DATA_ATTENUATOR_GPIO_Port, DATA_ATTENUATOR_Pin);
	Gpio le_pin(LE_ATTENUATOR_GPIO_Port, LE_ATTENUATOR_Pin);
	Gpio clock_pin(CLK_ATTENUATOR_GPIO_Port, CLK_ATTENUATOR_Pin);
	Gpio led_pin(LED_GPIO_Port, LED_Pin);

	HAL_ADCEx_Calibration_Start(&hadc1);

	AdcHandler rf_power_out(&hadc1, ADC_CHANNEL_0);
	AdcHandler current_consumption(&hadc1, ADC_CHANNEL_6);
	AdcHandler voltage_input(&hadc1, ADC_CHANNEL_7);
	AdcHandler agc_level(&hadc1, ADC_CHANNEL_8);
	GpioHandler gpioHandler(4, 4);
	RFAttenuator rfAttenuator(gpioHandler, data_pin, clock_pin, le_pin);

	uint8_t attenuation_flag = eeeprom.getValue<uint8_t>(att_flag_key);

	if (attenuation_flag == ATT_FLAG) {
		uint8_t attenuation = eeeprom.getValue<uint8_t>(lna_att_key);
		rfAttenuator.attenuate(attenuation);
	} else {
		eeeprom.setValue(att_flag_key, ATT_FLAG);
		eeeprom.setValue(lna_att_key, (uint8_t) 0);
	}

	uint8_t pout_calibration_flag = eeeprom.getValue<uint8_t>(
	POUT_ISCALIBRATED_ADDR);
	if (pout_calibration_flag != POUT_ISCALIBRATED) {
		eeeprom.setValue(POUT_ADC_MIN_ADDR, POUT_ADC_MIN);
		eeeprom.setValue(POUT_ADC_MAX_ADDR, POUT_ADC_MAX);
	}

	myUart.transmitMessage("LNA init\n\r");
	myUart.wait_for_it_byte();

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Start ADC and enable interrupt (optional, for more efficient background reading)
	led_counter = HAL_GetTick();
	uint32_t lna_print_counter = HAL_GetTick();
//	set_attenuation_to_bda4601(eeprom_1byte_read(LNA_ATT_ADDR), 5);
//
	uint16_t adcRredings[4];

	while (1) {

		/* USER CODE END WHILE */

		adcRredings[0] = rf_power_out.getChannelValue();
		adcRredings[1] = current_consumption.getChannelValue();
		adcRredings[2] = voltage_input.getChannelValue();
		adcRredings[3] = agc_level.getChannelValue();

		/* USER CODE BEGIN 3 */
		if (myUart.isDataReady){

		}
		/*
		 if (myUart.command == command.getQueryParameterLTEL()) {
		 //lna = calulate_lna_real_values(adcResultsDMA);
		 packet_lna_for_ltel_protocol(frame, lna);
		 myUart.transmitData(frame, LTEL_FRAME_SIZE);
		 } else if (myUart.command == command.getSetAttLTEL()) {
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

		 } else if (myUart.command == command.getSetPoutMax()) {
		 eeprom_2byte_write(POUT_ADC_MAX_ADDR, adcResultsDMA[POUT_INDEX]);
		 HAL_Delay(5);
		 eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
		 tx_buffer_size = sprintf((char*) UART1_txBuffer,
		 "Saved adc = %d as Pout 0 [dBm]\n\r",
		 adcResultsDMA[POUT_INDEX]);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
		 HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
		 UART_TRANSMIT_TIMEOUT);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
		 } else if (myUart.command == command.getSetPoutMin()) {
		 eeprom_2byte_write(POUT_ADC_MIN_ADDR, adcResultsDMA[POUT_INDEX]);
		 HAL_Delay(5);
		 eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
		 tx_buffer_size = sprintf((char*) UART1_txBuffer,
		 "Saved adc = %d as Pout -30 [dBm]\n\r",
		 adcResultsDMA[POUT_INDEX]);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
		 HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
		 UART_TRANSMIT_TIMEOUT);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
		 } else if (myUart.command == command.getQueryParameterStr()) {
		 isPrintEnable = !isPrintEnable;
		 } else if (myUart.command == command.getQueryADC()) {
		 tx_buffer_size = sprintf((char*) UART1_txBuffer,
		 "Pout %d  \t Gain %u \t Curent %u \t Voltage %u\r\n",
		 adcResultsDMA[POUT_INDEX], adcResultsDMA[GAIN_INDEX],
		 adcResultsDMA[CURRENT_INDEX], adcResultsDMA[VOLTAGE_INDEX]);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
		 HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
		 UART_TRANSMIT_TIMEOUT);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
		 } else if (myUart.command == command.getQueryParameterSigma()) {
		 //lna = calulate_lna_real_values(adcResultsDMA);
		 sigma_set_parameter_frame(frame, lna);
		 tx_buffer_size = LTEL_FRAME_SIZE;
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
		 HAL_UART_Transmit(&huart1, UART1_txBuffer, tx_buffer_size,
		 UART_TRANSMIT_TIMEOUT);
		 HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
		 }

		 if (isPrintEnable)
		 if (HAL_GetTick() - lna_print_counter > LNA_PRINT_TIMEOUT) {
		 struct Lna lna;
		 lna = calulate_lna_real_values(adcResultsDMA);
		 char *buffer = (char*) UART1_txBuffer;
		 tx_buffer_size =
		 sprintf(buffer,
		 "Pout %d"
		 "[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
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

		 */

		if (HAL_GetTick() - led_counter > LED_STATE_TIMEOUT)
			led_counter = HAL_GetTick();
		else {
			if (HAL_GetTick() - led_counter > LED_ON_TIMEOUT)
				gpioHandler.turnOn(led_pin);
			else
				gpioHandler.turnOff(led_pin);
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
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
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
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

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// Read received data from UART1
	uint8_t myBute = myUart.getByte();
	command.checkByte(myBute);
	myUart.wait_for_it_byte();


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
	/* USER CODE END Error_Handler_Debug */}

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
