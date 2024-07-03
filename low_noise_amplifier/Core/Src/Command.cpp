/*
 * Function.cpp
 *
 *  Created on: Jun
 *   19, 2024
 *      Author: artur
 */

#include "Command.hpp" // Include the header file

Command::Command(uint8_t _module_function, uint8_t _module_id) {
    module_function = _module_function;
    module_id = _module_id;
    max_message_size = 255; // Enforce max size
    listening = false;
}

Command::~Command() {

} // Empty destructor (optional)

void Command::checkByte(uint8_t number) {
  if (listening) {
    message.push_back(number);
    if (number == getLTELEndMark()) {
      listening = false;
    }
    if (message.size() >= max_rx_buffer_size) {
      message.clear();
      listening = false;
    }
  } else {
    if (number == getLTELStartMark()) {
      message.clear();
      message.push_back(number);
      listening = true;
    }
  }
}

bool Command::prepareTxData(const char *message) {
	// 1. Calculate message length (avoid using sprintf for this)
	size_t message_length = std::strlen(message);

	// 2. Check if message fits within the buffer size
	if (message_length >= max_tx_buffer_size) {
		// Handle error: message too long
		// (e.g., return false, set an error flag)
		return (false);
	}

	// 3. Use safer string copy function (consider snprintf if needed)
	std::strncpy(reinterpret_cast<char*>(tx_buffer), message,
			max_tx_buffer_size);
	// Ensure null termination even if message length reaches max_tx_buffer_size
	tx_buffer[message_length] = '\0';

	// 4. Update tx_buffer_size with actual copied message length
	tx_counter = message_length;

	return (true);
}

void Command::handleRxData(uint8_t data) {
	if (rx_counter < max_rx_buffer_size) {
		rx_buffer[rx_counter++] = data;
	}
}

// Implementations of the virtual functions (specific to your commands)
void Command::encode() {
	// Implementation of encoding logic
}

void Command::decode() {
	// Implementation of decoding logic
}
