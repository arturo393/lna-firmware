/*
 * Memory.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <Memory.hpp>

// Implementations of the member functions (would be placed outside the class)

Memory::Memory(I2cHandler *_hi2c) {
	hi2c = _hi2c;
	EEPROM_CHIP_ADDR = 0x50;
	EEPROM_PAGE_SIZE = 8;
	EEPrOM_PAGE_NUM = 32;

}

Memory::~Memory() {
}

uint8_t Memory::createKey(uint8_t address, uint8_t size) {
	// Add the name-address pair to the vector
	MemoryLocation mem_location;
	mem_location.address = address;
	mem_location.size = size;
	value_addr.push_back(mem_location);
  return (static_cast<uint8_t>(value_addr.size() - 1));
}


uint8_t Memory::EEPROM_Read(uint8_t address) {
	char  buff[2];
	buff[0] = address;
	hi2c->byteTransmit(EEPROM_CHIP_ADDR << 1, buff,1);
	buff[1] = hi2c->byteReceive(EEPROM_CHIP_ADDR << 1 | 1,1);
	return buff[1];
}

void Memory::EEPROM_Write(uint8_t address, uint8_t data) {
	char  buff[2];
	uint8_t stored_data;
	buff[0] = address;
	buff[1] = data;

	stored_data = EEPROM_byte_Read<uint8_t>(address);
	if (stored_data != data)
		hi2c->byteTransmit(EEPROM_CHIP_ADDR << 1, buff,2);
}




