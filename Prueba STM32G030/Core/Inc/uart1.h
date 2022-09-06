/*
 * uart1.h
 *
 *  Created on: Aug 29, 2022
 *      Author: sigmadev
 */

#ifndef INC_UART1_H_
#define INC_UART1_H_

#include "stm32g0xx_hal.h"
#include "stdbool.h"

extern bool rxdata;
extern uint32_t rxfne;

void uart1_init(uint32_t , uint32_t );
void uart1_write(volatile char);
void uart1_send_str(volatile char *);
void uart1_send_frame(char*, uint8_t );
void uart1_read(char *,uint8_t);
void uart1_read_test(char *,uint8_t);
char uart1_1byte_read(void);

#endif /* INC_UART1_H_ */
