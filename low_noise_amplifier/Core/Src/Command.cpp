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
    command_id = 0;
}

Command::~Command() {

} // Empty destructor (optional)

void Command::checkByte(uint8_t number) {
  if (listening) {
    message.push_back(number);
    if (number == getLTELEndMark()) {
      listening = false;
    }
    if (message.size() >= max_message_size) {
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

