/*
 * Memory.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <Memory.hpp>


// Implementations of the member functions (would be placed outside the class)

Memory::Memory(I2C_HandleTypeDef *_hi2c) {
	hi2c = _hi2c;
}

Memory::~Memory() {}

void Memory::createKey(std::string name, MemLocation address) {
  // Add the name-address pair to the map
  value_addr[name] = address;
}

uint8_t* Memory::getValue(std::string name) {
}

void Memory::setValue(std::string name, uint8_t value) {

}

uint8_t Memory::EEPROM_Read(uint8_t address) {
	uint8_t buff[2];
	buff[0] = address;
	HAL_I2C_Master_Transmit(hi2c, EEPROM_CHIP_ADDR << 1, buff, 1, 100);
	HAL_I2C_Master_Receive(hi2c, EEPROM_CHIP_ADDR << 1 | 1, &buff[1], 1, 100);
	return buff[1];
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

void Memory::EEPROM_2byte_Write(uint8_t addr, uint16_t data) {
	EEPROM_Write(addr, data & 0xff);
	HAL_Delay(5);
	EEPROM_Write(addr + 1, data >> 8);
}

uint16_t Memory::EEPROM_2byte_Read(uint8_t address) {
	uint16_t data = 0;
	data = EEPROM_Read(address + 1) << 8;
	HAL_Delay(5);
	data |= EEPROM_Read(address);

	return data;
}
