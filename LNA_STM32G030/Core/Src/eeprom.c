#include "eeprom.h"

//extern I2C_HandleTypeDef hi2c1;
/*
uint8_t EEPROM_Read(uint8_t address) {
	uint8_t buff[2];
	buff[0] = address;
	HAL_I2C_Master_Transmit(&hi2c1, EEPROM_CHIP_ADDR << 1, buff, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, EEPROM_CHIP_ADDR << 1 | 1, &buff[1], 1, 100);
	return buff[1];
}

void EEPROM_Write(uint8_t address, uint8_t data) {
	uint8_t buff[2];
	uint8_t stored_data;
	buff[0] = address;
	buff[1] = data;

	stored_data = EEPROM_Read(address);

	if (stored_data != data)
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_CHIP_ADDR << 1, buff, 2, 100);
}

void EEPROM_2byte_Write(uint8_t addr, uint16_t data) {
	EEPROM_Write(addr, data & 0xff);
	HAL_Delay(5);
	EEPROM_Write(addr + 1, data >> 8);
}

uint16_t EEPROM_2byte_Read(uint8_t address) {
	uint16_t data = 0;
	data = EEPROM_Read(address + 1) << 8;
	HAL_Delay(5);
	data |= EEPROM_Read(address);

	return data;
}
*/
uint8_t eeprom_1byte_read(uint8_t eaddress) {
	char  buff[2];
	buff[0] = eaddress;
	i2c1_byteTransmit(EEPROM_CHIP_ADDR << 1, buff,1);
	buff[1] = i2c1_byteReceive(EEPROM_CHIP_ADDR << 1 | 1,1);
	return buff[1];
}

void eeprom_1byte_write(uint8_t address, uint8_t data) {
	char  buff[2];
	uint8_t stored_data;
	buff[0] = address;
	buff[1] = data;

	stored_data = eeprom_1byte_read(address);
	if (stored_data != data)
		i2c1_byteTransmit(EEPROM_CHIP_ADDR << 1, buff,2);
}

void eeprom_2byte_write(uint8_t addr, uint16_t data) {
	eeprom_1byte_write(addr, data & 0xff);
	HAL_Delay(5);
	eeprom_1byte_write(addr + 1, data >> 8);
}

uint16_t eeprom_2byte_read(uint8_t address) {
	uint16_t data = 0;
	data = eeprom_1byte_read(address + 1) << 8;
	HAL_Delay(5);
	data |= eeprom_1byte_read(address);

	return data;
}


