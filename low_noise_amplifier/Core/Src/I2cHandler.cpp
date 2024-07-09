#include <I2cHandler.hpp>



void I2cHandler::i2c1_init() {

	/* SCL PB6 as alternate */

	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE6_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE6_1);
	/* SDC PB7 as alternate */
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE7_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE7_1);
	/* SCL PB6 as open-drain */
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT6);
	/* SDC PB7 as open-drain */
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT7);
	/* SCL PB6 High Speed output */
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED6_0);
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED6_1);
	/* SDC PB7  High Speed output */
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7_0);
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7_1);
	/* SCL PB6 as pull-up */
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD6_0);
	SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD6_1);
	/* SDC PB7 as pull-up */
	CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD7_0);
	SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD7_1);
	/*  PB6 as i2c SCL */
	CLEAR_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_0);
	SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_1);
	SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_2);
	CLEAR_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_3);
	/*  PB7 as i2c SDL */
	CLEAR_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_0);
	SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_1);
	SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_2);
	CLEAR_BIT(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_3);

	/* select normal speed */
	SET_BIT(RCC->APBENR1, RCC_APBENR1_I2C1EN);

	/* i2c disable */
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);

	// TODO revisar porque funciona
	//	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_PRESC, 1);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLDEL, 0x7);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SDADEL, 0x0);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLH, 0x7D);
// 	ATOMIC_MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLL, 0xBC);

	MODIFY_REG(I2C1->TIMINGR, 0X10111111U, 0X10707DBCU);
	/*i2c Rx interrupt enable */
	SET_BIT(I2C1->CR1, I2C_CR1_RXIE);
	SET_BIT(I2C1->CR1, I2C_CR1_TXIE);

	/* i2c enable */
	SET_BIT(I2C1->CR1, I2C_CR1_PE);
}

char I2cHandler::byteReceive(char saddr, uint8_t N) {

	startComunication(saddr, READ, N);

	char data = 0;
	for (int i = 0; i < N; i++) {
		while (!READ_BIT(I2C1->ISR, I2C_ISR_RXNE)
				& !READ_BIT(I2C1->ISR, I2C_ISR_NACKF)) {
		}
		data = READ_REG(I2C1->RXDR);

	}
	while (!(READ_BIT(I2C1->ISR, I2C_ISR_STOPF)
			| READ_BIT(I2C1->ISR, I2C_ISR_NACKF))) {
	}
	SET_BIT(I2C1->ISR, I2C_ICR_STOPCF);
	SET_BIT(I2C1->ISR, I2C_ICR_NACKCF);

	return data;
}

void I2cHandler::byteTransmit(char saddr, char *data, uint8_t N) {
	startComunication(saddr, WRITE, N);

	for (int i = 0; i < N; i++) {
		while (!READ_BIT(I2C1->ISR, I2C_ISR_TXIS)
				& !READ_BIT(I2C1->ISR, I2C_ISR_NACKF)) {
		}
		MODIFY_REG(I2C1->TXDR, I2C_TXDR_TXDATA, data[i]);
	}

	while (!READ_BIT(I2C1->ISR, I2C_ISR_STOPF)) {
	}
	SET_BIT(I2C1->ISR, I2C_ICR_STOPCF);

}

void I2cHandler::startComunication(char saddr, uint8_t transfer_request, uint8_t N) {
	/*master 7 bit addressing mode */
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ADD10);
	/* set Slave address */
	MODIFY_REG(I2C1->CR2, I2C_CR2_SADD, saddr << I2C_CR2_SADD_Pos);
	/* read 1 byte */
	MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES, N << I2C_CR2_NBYTES_Pos);
	/* stops when NBytes are transferred */
	SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND);
	/* set START condition  automatically changes to master */

	if (transfer_request == 1) {
		/* request a read transfer */
		SET_BIT(I2C1->CR2, I2C_CR2_RD_WRN);
	} else if (transfer_request == 0) {
		/* request a write transfer */
		CLEAR_BIT(I2C1->CR2, I2C_CR2_RD_WRN);
	}

	SET_BIT(I2C1->CR2, I2C_CR2_START);

}

void I2cHandler::scanner(uint8_t *addr) {
	uint32_t counter = HAL_GetTick();
	uint8_t j = 0;
	uint8_t timeout = 0;

	for (int i = 1; i < 128; i++) {
		startComunication(i << 1 | 1, READ, 1);
		timeout = 0;

		while (!READ_BIT(I2C1->ISR, I2C_ISR_RXNE) & !timeout) {

			if (HAL_GetTick() - counter > 500) {
				counter = HAL_GetTick();
				timeout = 1;
			}
		}
		if (!timeout) {
			addr[j++] = i;
			READ_REG(I2C1->RXDR);
		}
	}
}

