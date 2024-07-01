/*
 * Memory.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: artur
 */

#include <Memory.hpp>


// Implementations of the member functions (would be placed outside the class)

Memory::Memory() {}

Memory::~Memory() {}

void Memory::createKey(std::string name, MemLocation address) {
  // Add the name-address pair to the map
  value_addr[name] = address;
}

uint8_t* Memory::getValue(std::string name) {
}

void Memory::setValue(std::string name, uint8_t value) {

}
