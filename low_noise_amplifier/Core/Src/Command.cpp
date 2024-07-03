/*
 * Function.cpp
 *
 *  Created on: Jun
 *   19, 2024
 *      Author: artur
 */

#include <main.h>

#include "Command.hpp" // Include the header file

Command::Command(uint8_t _module_function, uint8_t _module_id) {

	module_function = _module_function;
	module_id = _module_id;
	max_rx_buffer_size = _max_rx_buffer_size; // Enforce max size
	max_tx_buffer_size = _max_tx_buffer_size; // No validation assumed for TX
	rx_buffer = new uint8_t[max_rx_buffer_size];
	tx_buffer = new uint8_t[max_tx_buffer_size];
}

Command::~Command() {

} // Empty destructor (optional)


// Implementations of the virtual functions (specific to your commands)
void Command::encode() {
	// Implementation of encoding logic
}

void Command::decode() {
	// Implementation of decoding logic
}
