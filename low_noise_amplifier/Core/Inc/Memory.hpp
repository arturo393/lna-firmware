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
	uint8_t address;
	int size;
};

class Memory {
public:
	Memory();
	virtual ~Memory();
	void createKey(std::string name,MemLocation address);

	uint8_t* getValue(std::string name);
	void setValue(std::string name,uint8_t value);
private:
	std::map<std::string, MemLocation> value_addr;

};


#endif /* INC_MEMORY_HPP_ */
