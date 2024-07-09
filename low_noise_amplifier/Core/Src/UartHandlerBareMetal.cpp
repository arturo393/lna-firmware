/*
 * UartHandler.cpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "UartHandlerBareMetal.hpp"


void UartHandlerBareMetal::init(USART_TypeDef *USART,
		GPIO_TypeDef *_data_enable_port, uint16_t _data_enable_pin) {
	_data_enable_port = data_enable_port;
	_data_enable_pin = data_enable_pin;
	usart = USART;
	uart_gpio_init();
	uart_init(16000000, 9600);
}


/* Read received data from UART1 */
void UartHandlerBareMetal::wait_for_it_byte() {
	//HAL_UART_Receive_IT(huart, &rxData, 1);
}

uint8_t UartHandlerBareMetal::getByte() {
	return (rxData);
}


void UartHandlerBareMetal::uart_gpio_init() {
	/**USART1 GPIO Configuration
	 PA9     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */
	/* PA9 TX as alter */
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE9_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE9_1);
	/* PA9 TX as alter */
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE10_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE10_1);
	/* PA9 Tx open drain */
	CLEAR_BIT(GPIOA->MODER, GPIO_OTYPER_OT9);
	/* PA9 Tx open drain */
	CLEAR_BIT(GPIOA->MODER, GPIO_OTYPER_OT10);
	/* PA9 Tx pull up */
	CLEAR_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD9_0);
	SET_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD9_1);
	/* PA10 RX pull up */
	CLEAR_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD10_0);
	SET_BIT(GPIOA->MODER, GPIO_PUPDR_PUPD10_1);
	/*  PA9 Tx low speed */
	CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED9_0);
	CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED9_1);
	/* PA10 RX low speed */
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED10_0);
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED10_1);
	/*PA9 TX   AF1 as alter   */
	SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_0);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_1);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_2);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_3);
	/* PA10 RX  AF1 as alter   */
	SET_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_0);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_1);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_2);
	CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_3);
}

void UartHandlerBareMetal::uart_init(uint32_t pclk, uint32_t baud_rate) {
	uint32_t br_value = 0;

	uart_gpio_init();

	/*enable clock access to USART1 */
	SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);
	if(pclk == 16000000){
	/*set HSI 16 CLK */
	CLEAR_BIT(RCC->CCIPR,RCC_CCIPR_USART1SEL_0);
	SET_BIT(RCC->CCIPR,RCC_CCIPR_USART1SEL_1);
	}
	//MODIFY_REG(usart->PRESC,USART_PRESC_PRESCALER,0x0010);
	/* set baud rate */
	br_value = (pclk) / baud_rate;
	usart->BRR = (uint16_t) br_value;
	/* transmitter enable*/
	usart->CR1 = USART_CR1_TE | USART_CR1_RE;

	SET_BIT(usart->CR1, USART_CR1_RXNEIE_RXFNEIE);
	//NVIC_EnableIRQ(USART1_IRQn);
	SET_BIT(usart->CR1, USART_CR1_UE);
}


void UartHandlerBareMetal::uart_write(char* ch) {
	SET_BIT(GPIOA->ODR, GPIO_ODR_OD15);

	while (!READ_BIT(usart->ISR, USART_ISR_TXE_TXFNF));
	usart->TDR = (uint8_t) (*ch & 0xFFU);

	while (!READ_BIT(usart->ISR, USART_ISR_TC));

	CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD15);
}

void UartHandlerBareMetal::readCommand(CommandMessage& c) {
	  // Check for overrun error (optional)
	uint32_t MAX_TIMEOUT= 10000;
	  bool override = READ_BIT(usart->ISR, USART_ISR_ORE);
	  if (override) {
	    // Handle overrun error (e.g., clear flag)
	    SET_BIT(usart->ICR, USART_ICR_ORECF);
	  }

	  // Loop to read data byte-by-byte
	  while(!c.isReady()) {
	    // Set timeout counter
	    uint32_t timeout_counter = 0;

	    // Wait for RXNE flag with timeout
	    while (!READ_BIT(usart->ISR, USART_ISR_RXNE_RXFNE) && timeout_counter < MAX_TIMEOUT) {
	      timeout_counter++;
	    }

	    // Check for timeout
	    if (timeout_counter >= MAX_TIMEOUT) {
	      // Handle timeout (e.g., return error code)
	      return; // Replace with your timeout handling logic
	    }

	    // Read the received byte from the Data Register
	    c.checkByte(usart->RDR);
	  }

}
