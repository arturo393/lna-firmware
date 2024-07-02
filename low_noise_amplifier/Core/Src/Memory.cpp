/*
 * Memory.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <Memory.hpp>


// Implementations of the member functions (would be placed outside the class)

MemLocation::MemLocation(uint_8 _address, int size){
  address = _address;
  size = _size;
}

Memory::Memory(I2C_HandleTypeDef *_hi2c) {
	hi2c = _hi2c;
  value_addr['LNA_ATT_ADDR'] = MemLocation(0x00, 1);
  value_addr['POUT_ADC_MAX_ADDR'] = MemLocation(0x03, 1);
  value_addr['POUT_ADC_MIN_ADDR'] = MemLocation(0x05, 2);
  value_addr['POUT_ISCALIBRATED_ADDR'] = MemLocation(0x07, 2);
}

Memory::~Memory() {}

void Memory::createKey(std::string name, MemLocation address) {
  // Add the name-address pair to the map
  value_addr[name] = address;
}

template <typename T>
T Memory::getValue(std::string name) {
  location = value_addr[name];
  if ((location.size == 1) && (sizeof(T) == location.size)) {
    return (this->EEPROM_Read(location.address));
  } else if ((location.size == 2) && (sizeof(T) == location.size)) {
    return (this->EEPROM_2byte_Read(location.address))
  }
}

template <typename T>
void Memory::setValue(std::string name, T value) {
  std::map<string, MemLocation> it;
  MemLocation location;

  it = value_addr.find(name);
  if (it != value_addr.end()) {
    location = value_addr[name];
    if (sizeof(T) == location.size)
    this -> EEPROM_byte_Write(location.address, value);
  }
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

template <typename T>
void Memory::EEPROM_byte_Write(uint8_t addr, T data) {
  int i;
  size = sizeof(T);

 for (int i = 0; i < size; i++, data = data >> 8){
    EEPROM_Write(addr + i, static_cast<uint8_t>(data & 0xff));
    HAL_Delay(5);
  }
}

uint16_t Memory::EEPROM_2byte_Read(uint8_t address) {
	uint16_t data = 0;
	data = EEPROM_Read(address + 1) << 8;
	HAL_Delay(5);
	data |= EEPROM_Read(address);

	return data;
}
