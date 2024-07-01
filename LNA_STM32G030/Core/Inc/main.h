/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pout_Volt_Pin GPIO_PIN_0
#define Pout_Volt_GPIO_Port GPIOA
#define LE_ATTENUATOR_Pin GPIO_PIN_1
#define LE_ATTENUATOR_GPIO_Port GPIOA
#define CLK_ATTENUATOR_Pin GPIO_PIN_4
#define CLK_ATTENUATOR_GPIO_Port GPIOA
#define DATA_ATTENUATOR_Pin GPIO_PIN_5
#define DATA_ATTENUATOR_GPIO_Port GPIOA
#define Current_Volt_Pin GPIO_PIN_6
#define Current_Volt_GPIO_Port GPIOA
#define Volt_Volt_Pin GPIO_PIN_7
#define Volt_Volt_GPIO_Port GPIOA
#define AGC_Volt_Pin GPIO_PIN_0
#define AGC_Volt_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define DE_Pin GPIO_PIN_15
#define DE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
