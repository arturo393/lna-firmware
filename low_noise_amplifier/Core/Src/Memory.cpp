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


template<typename T>
void Memory::setValue(std::string name, T value) {
	MemLocation location;
	 if (value_addr.count(name)) {
		location = value_addr[name];
		if (sizeof(T) == location.size)
			this->EEPROM_byte_Write(location.address, value);
	}
}

uint8_t Memory::EEPROM_Read(uint8_t address) {
	uint8_t buff[2];
	buff[0] = address;
	HAL_I2C_Master_Transmit(hi2c, EEPROM_CHIP_ADDR << 1, buff, 1, 100);
	HAL_I2C_Master_Receive(hi2c, EEPROM_CHIP_ADDR << 1 | 1, &buff[1], 1, 100);
	return (buff[1]);
}

template<typename T>
T Memory::EEPROM_byte_Read(uint8_t address) {
	T data = 0;
  int size = sizeof(T);
  int i;

  for (i = size - 1; i <= 0; i--){
    if (i < size - 1) {
      HAL_Delay(5);
      data = data << 8;
    }
    data |= EEPROM_Read(address + i);
  }

	return (data);
}


template<typename T>
T Memory::getValue(std::string name) {
	MemLocation location;
	location = value_addr[name];
	uint8_t addr = 0;
	return (this->EEPROM_byte_Read<T>(location.address));
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

template<typename T>
void Memory::EEPROM_byte_Write(uint8_t addr, T data) {
	int i;
	int size = sizeof(T);

	for (i = 0; i < size; i++, data = data >> 8) {
		EEPROM_Write(addr + i, static_cast<uint8_t>(data & 0xff));
		HAL_Delay(5);
	}
}
