/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Header for eeprom.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c1.h"


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
uint8_t EEPROM_Read(uint8_t address);
void EEPROM_Write(uint8_t address, uint8_t data);
void EEPROM_2byte_Write(uint8_t addr, uint16_t data);
uint16_t EEPROM_2byte_Read(uint8_t address);

void eeprom_2byte_write(uint8_t addr, uint16_t data);
uint16_t eeprom_2byte_read(uint8_t address);
uint8_t eeprom_1byte_read(uint8_t eaddress);
void eeprom_1byte_write(uint8_t address, uint8_t data);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EEPROM_CHIP_ADDR 0x50
#define EEPROM_PAGE_SIZE 8
#define EEPrOM_PAGE_NUM 32
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
