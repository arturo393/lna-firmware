/*
 * Memory.hpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#ifndef INC_MEMORY_HPP_
#define INC_MEMORY_HPP_

#include <main.h>
#include <map>
#include <string>

class MemLocation {
public:
	MemLocation(uint8_t _address, int _size);
	uint8_t address;
	int size;
};


template <typename T>
class Memory {
public:
	Memory(I2C_HandleTypeDef* _hi2c);
	virtual ~Memory();
	void createKey(std::string name,MemLocation address);
	T getValue(std::string name);
	void setValue(std::string name, T value);
private:
	I2C_HandleTypeDef* hi2c;
	uint8_t EEPROM_CHIP_ADDR = 0x50;
	uint8_t EEPROM_PAGE_SIZE = 8;
	uint8_t EEPrOM_PAGE_NUM = 32;
	std::map<std::string, MemLocation> value_addr;
	uint8_t EEPROM_Read(uint8_t address);
	void EEPROM_Write(uint8_t address, uint8_t data);
	void EEPROM_byte_Write(uint8_t addr, T data);
	T EEPROM_byte_Read(uint8_t address);
};


#endif /* INC_MEMORY_HPP_ */
