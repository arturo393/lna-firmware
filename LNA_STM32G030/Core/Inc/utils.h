/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __utils_H
#define __utils_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <string.h>


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */
/* USER CODE BEGIN Global variables */


/* USER CODE END Global variables */

/* USER CODE BEGIN Prototypes */
uint16_t crc_get(uint8_t buffer[], uint8_t buff_len);
bool crc_check(uint8_t *frame, uint8_t len, uint8_t *crc_frame);
uint8_t gencrc(uint8_t *data, size_t len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */
