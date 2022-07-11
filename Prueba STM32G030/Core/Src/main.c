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
#include <string.h>
#include <stdio.h>
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

#define MINIMUN_FRAME_SIZE 7
#define LNA_MODULE_ADDR 0x08
#define LNA_MODULE_FUNCTION 0x09
#define PARAM_QUERY_ID 0x11
#define SET_ATT_ID 0x20
#define LNA_VREF 3.28f // volts * 1000
#define LNA_CURRENT_FACTOR  2.2f/200.0f  // 2,2 [v] / 200 [A]
#define LNA_CURRENT_MULTIPLIER (LNA_VREF)/(LNA_CURRENT_FACTOR)

#define LNA_MULTIPLIER 298
#define TXBUFLEN 40
#define RX_UART1_BUFFLEN 20

#define  START_MARK  0x7e
#define  END_MARK  0x7f

#define EEPROM_ADDR 0x50
#define LNA_ATT_ADDR 0x00
#define LNA_GAIN_ADDR 0x01
#define LNA_CURRENT_ADDR 0x02
#define LNA_POUT_ADDR 0x03
#define LNA_PIN_ADDR 0x04
#define LNA_MAX_POUT_ADDR 0x05
#define LNA_MIN_POUT_ADDR 0x06

struct Lna {
	uint8_t attenuation;
	float gain;
	float pout;
	uint8_t current;
	uint8_t pin;
};

static uint8_t data_arrive = 0;
volatile uint16_t adcResultsDMA[3];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
volatile int adcConversionComplete = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// Conversion Complete & DMA Transfer Complete As Well
	// So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
	// Update The PWM Duty Cycle With Latest ADC Conversion Result
	adcConversionComplete = 1;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t get_crc_calc(uint8_t buffer[], uint8_t buff_len) {
	uint8_t b;
	uint8_t i;
	uint16_t generator = 0x1021; //divisor is 16bit
	uint16_t crc = 0;			 // CRC value is 16bit

	for (b = 0; b < buff_len; b++) {
		crc ^= ((uint16_t) (buffer[b] << 8)); // move byte into MSB of 16bit CRC
		for (i = 0; i < 8; i++) {
			if ((crc & 0x8000) != 0) // test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			else
				crc <<= 1;
		}
	}
	return crc;
}

//---------[ UART Data Reception Completion CallBackFunc. ]---------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	data_arrive = 1;
//HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 20);
}

uint8_t EEPROM_Read(uint8_t address) {
	uint8_t buff[2];
	buff[0] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR << 1, buff, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR << 1 | 1, &buff[1], 1, 100);
	return buff[1];
}

void EEPROM_Write(uint8_t address, uint8_t data) {
	uint8_t buff[2];
	uint8_t stored_data;
	buff[0] = address;
	buff[1] = data;

	stored_data = EEPROM_Read(address);
	if (stored_data != data)
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR << 1, buff, 2, 100);
}

uint8_t check_crc(uint8_t *frame, uint8_t len, uint8_t crc_frame[2]) {

	uint16_t crc;
	uint8_t crc_valid = 1;
	uint8_t testframe[2];
	crc = get_crc_calc(frame, len);
	memcpy(testframe, &crc, 2);
	if (testframe[1] == crc_frame[1] && testframe[0] == crc_frame[0]) {
		crc_valid = 1;
	} else {
		crc_valid = 0;
	}
	return crc_valid;;
}

void set_lna_attenuation(uint8_t attenuation, uint8_t times) {
	attenuation *= 2;
	for (int i = 0; i < times; i++) {
		uint8_t mask = 0b00100000;
		for (uint8_t i = 0; i < 6; i++) {
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

void create_lna_frame(uint8_t *frame, struct Lna lna) {
	uint8_t crc_frame[2];
	uint16_t crc;
	frame[0] = START_MARK;
	frame[1] = LNA_MODULE_FUNCTION;
	frame[2] = LNA_MODULE_ADDR;
	frame[3] = PARAM_QUERY_ID;
	frame[4] = 0x00;
	frame[5] = 0x05;
	frame[6] = 0x00;
	frame[7] = lna.attenuation;
	frame[8] = lna.gain;
	frame[9] = lna.pout;
	frame[10] = lna.pout;
	crc = get_crc_calc(&(frame[1]), 10);
	memcpy(crc_frame, &crc, 2);
	frame[11] = crc_frame[0];
	frame[12] = crc_frame[1];
	frame[13] = END_MARK;
}

void uart_send_frame(uint8_t *frame, uint8_t len) {
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_SET);
	//HAL_Delay(10);
	HAL_UART_Transmit(&huart1, frame, len, 100);
	//HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, DE_Pin, GPIO_PIN_RESET);

}

uint8_t get_valid_start_index(uint8_t UART1_rxBuffer[20]) {
	uint8_t start_index = -1;
	for (uint8_t i = 0; i < RX_UART1_BUFFLEN - MINIMUN_FRAME_SIZE; i++) {
		if (start_index == (uint8_t) -1) {
			if (UART1_rxBuffer[i] == (uint8_t) START_MARK)
				start_index =
						UART1_rxBuffer[i + 1] == LNA_MODULE_FUNCTION ? i : -1;
			else
				start_index = -1;
		} else {
			// if(start_index > 0 )
			if (UART1_rxBuffer[i] == (uint8_t) END_MARK) {
				if (i > MINIMUN_FRAME_SIZE) {
					uint16_t crc_rcv;
					uint16_t crc_calc;
					uint8_t data_start_index = start_index + 1;
					crc_calc = get_crc_calc(&(UART1_rxBuffer[data_start_index]),
							i - data_start_index - 2);
					crc_rcv = UART1_rxBuffer[i - 1] << 8
							| UART1_rxBuffer[i - 2];
					return crc_calc == crc_rcv ? start_index : -1;
				}
			}
			// end if endmark}
			// end if startmark}

		}
	}
	return -1;
}

uint8_t init_lna_value() {
	uint8_t lna_attenuation = 0;
	lna_attenuation = EEPROM_Read(LNA_ATT_ADDR);
	if (lna_attenuation < 0 || lna_attenuation > 31) {
		lna_attenuation = 0;
	}
	set_lna_attenuation(lna_attenuation, 3);

	return lna_attenuation;
}

float get_lna_pout(uint16_t pout_adc) {
	if (pout_adc > 1828)
		return 0;
	else if (pout_adc <= 1828 && pout_adc > 1763)
		return (0.0262f * pout_adc - 47.71f);
	else if (pout_adc <= 1763 && pout_adc > 1705)
		return (0.0138f * pout_adc - 25.92f);
	else if (pout_adc <= 1705 && pout_adc > 1607)
		return (0.020f * pout_adc - 36.5f);
	else if (pout_adc <= 1607 && pout_adc > 1519)
		return (0.0226f * pout_adc - 40.9f);
	else if (pout_adc <= 1519 && pout_adc > 1369)
		return 0.02f * pout_adc - 36.98f;
	else if (pout_adc <= 1369 && pout_adc > 1106)
		return 0.0194f * pout_adc - 36.15f;
	else if (pout_adc <= 1106 && pout_adc > 950)
		return 0.0199f * pout_adc - 36.68f;
	else if (pout_adc <= 1106 && pout_adc >= 864)
		return 0.0207f * pout_adc - 37.55f;
	else if (pout_adc <= 864 && pout_adc > 725)
		return 0.0209f * pout_adc - 37.64f;
	else if (pout_adc <= 864 && pout_adc > 611)
		return 0.0209f * pout_adc - 37.79f;
	else if (pout_adc <= 611 && pout_adc >= 569)
		return 0.0357f * pout_adc - 46.82f;
	else if (pout_adc <= 569 && pout_adc > 475)
		return 0.036f * pout_adc - 47.01f;
	else if (pout_adc <= 475 && pout_adc > 419)
		return 0.0911f * pout_adc - 73.16f;
	else if (pout_adc < 419)
		return 1;
	return 1;
}

uint8_t get_lna_current(uint16_t adc_current) {
	return (LNA_CURRENT_MULTIPLIER * adc_current / 4096.0f);
}

float get_lna_gain(uint16_t gain) {

	if (gain >= 3781)
		return 45.0f;
	else if (gain < 3781 && gain >= 1515)
		return 0.0022f * gain + 36.6571f;
	else if (gain < 1515 && gain >= 1188)
		return (0.0153f * gain + 16.8349f);
	else if (gain < 1188 && gain >= 1005)
		return (0.0273f * gain + 2.540f);
	else if (gain < 1005 && gain >= 897)
		return (0.0463f * gain - 16.5278f);
	else if (gain < 897 && gain >= 825)
		return (0.0694f * gain - 37.2917f);
	else if (gain < 825 && gain >= 776)
		return (0.1020f * gain - 64.1837f);
	else if (gain < 776 && gain >= 746)
		return (0.1667f * gain - 114.333f);
	else if (gain < 746 && gain >= 733)
		return (0.3846f * gain - 276.9231f);
	else if (gain < 733 && gain >= 725)
		return (0.625f * gain - 453.125f);
	else if (gain < 725)
		return 0;
	else
		return 0;
}

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
	MX_USART2_UART_Init();
	MX_IWDG_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */

	// Calibrate The ADC On Power-Up For Better Accuracy
	//HAL_ADCEx_Calibration_Start(&hadc1);
	//uart_send_frame("LNA init\n\r",11);
	struct Lna lna;
	uint8_t UART1_rxBuffer[20] = { 0 };
	uint8_t bytes_readed;
	lna.attenuation = init_lna_value();
//	uint8_t isPotCalibrated = 0;

//	EEPROM_Read(POT_MAX_ADDR);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 3);

	while (1) {

		//Fin function 1 second
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, RX_UART1_BUFFLEN);
		bytes_readed = RX_UART1_BUFFLEN - (huart1.hdmarx->Instance->CNDTR);

		if (data_arrive || bytes_readed > MINIMUN_FRAME_SIZE) {

			HAL_UART_DMAPause(&huart1);
			huart1.hdmarx->Instance->CCR &= ~DMA_CCR_EN; // disable
			huart1.hdmarx->Instance->CNDTR = 20; // reset lna_attenuation
			huart1.hdmarx->Instance->CCR |= DMA_CCR_EN; // re-enable
			data_arrive = 0;
			uint8_t start_index = get_valid_start_index(UART1_rxBuffer);

			if (start_index != (uint8_t) -1) {
				if (UART1_rxBuffer[start_index + 1] == LNA_MODULE_FUNCTION) {
					if (UART1_rxBuffer[start_index + 3] == PARAM_QUERY_ID) {

						uint8_t ltel_frame[14];
						create_lna_frame(ltel_frame, lna);
						uart_send_frame(ltel_frame, 14);
					} else if (UART1_rxBuffer[start_index + 3] == SET_ATT_ID) {
						lna.attenuation = UART1_rxBuffer[start_index + 6];
						HAL_Delay(1); //Delay de 1mS
						set_lna_attenuation(lna.attenuation, 3);
						EEPROM_Write(LNA_ATT_ADDR, lna.attenuation);
					}
				}
				start_index = -1;
			}

			HAL_UART_DMAResume(&huart1);
		}

		if (adcConversionComplete == 1) {
			lna.pout = get_lna_pout(adcResultsDMA[0]);
			lna.current = get_lna_current(adcResultsDMA[1]);
			lna.gain = get_lna_gain(adcResultsDMA[2]);
			lna.pin = lna.pout - lna.gain + lna.attenuation;
			adcConversionComplete = 0;
		}

		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 3);
		HAL_Delay(5); // for UART1_rx buffer complete readings
		HAL_IWDG_Refresh(&hiwdg);

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
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DiscontinuousConvMode = ENABLE;
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
