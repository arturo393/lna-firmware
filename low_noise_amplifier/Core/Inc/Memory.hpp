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
	MemLocation(uint_8 _address, int size);
	uint8_t address;
	int size;
};

class Memory {
public:
	Memory(I2C_HandleTypeDef* _hi2c);
	virtual ~Memory();
	void createKey(std::string name,MemLocation address);
	template <typename T>
	T getValue(std::string name);
	void setValue(std::string name,uint8_t value);
private:
	I2C_HandleTypeDef* hi2c;
	uint8_t EEPROM_CHIP_ADDR = 0x50;
	uint8_t EEPROM_PAGE_SIZE = 8;
	uint8_t EEPrOM_PAGE_NUM = 32;
	std::map<std::string, MemLocation> value_addr;
	uint8_t EEPROM_Read(uint8_t address);
	void EEPROM_Write(uint8_t address, uint8_t data);
	uint16_t EEPROM_2byte_Write(uint8_t addr, uint16_t data);
	void EEPROM_2byte_Read(uint8_t address);

};


#endif /* INC_MEMORY_HPP_ */
