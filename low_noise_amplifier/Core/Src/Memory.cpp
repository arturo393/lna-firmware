/*
 * Memory.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <Memory.hpp>

// Implementations of the member functions (would be placed outside the class)
MemLocation::MemLocation(uint8_t _address, int _size) {
	address = _address;
	size = _size;
}

Memory::Memory(I2C_HandleTypeDef *_hi2c) {
	hi2c = _hi2c;
	EEPROM_CHIP_ADDR = 0x50;
	EEPROM_PAGE_SIZE = 8;
	EEPrOM_PAGE_NUM = 32;

}

Memory::~Memory() {
}

void Memory::createKey(std::string name, MemLocation address) {
	// Add the name-address pair to the map
	value_addr[name] = address;
}


uint8_t Memory::EEPROM_Read(uint8_t address) {
	uint8_t buff[2];
	buff[0] = address;
	HAL_I2C_Master_Transmit(hi2c, EEPROM_CHIP_ADDR << 1, buff, 1, 100);
	HAL_I2C_Master_Receive(hi2c, EEPROM_CHIP_ADDR << 1 | 1, &buff[1], 1, 100);
	return (buff[1]);
}



void Memory::EEPROM_Write(uint8_t address, uint8_t data) {
	uint8_t buff[2];
	uint8_t stored_data;
	buff[0] = address;
	buff[1] = data;

	stored_data = EEPROM_Read(address);

	if (stored_data != data)
		HAL_I2C_Master_Transmit(hi2c, EEPROM_CHIP_ADDR << 1, buff, 2, 100);
}

