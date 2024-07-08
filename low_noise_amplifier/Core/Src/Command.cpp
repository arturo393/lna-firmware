/*
 * Function.cpp
 *
 *  Created on: Jun
 *   19, 2024
 *      Author: artur
 */

#include "Command.hpp" // Include the header file

Command::Command(uint8_t _module_function, uint8_t _module_id, uint8_t max_size)
    : module_function(_module_function), module_id(_module_id), max_message_size(max_size) {
    reset(true);
}

Command::Command(uint8_t _module_function, uint8_t _module_id)
    : Command(_module_function, _module_id, 255) {}

Command::~Command() {}

void Command::setVars() {
  #define MODULE_FUNCTION_BYTE 1
  #define MODULE_ID_BYTE 2
  #define COMMAND_BYTE 3

  if (!ready) return;

  command_id = message[COMMAND_BYTE];
  module_id = message[MODULE_ID_BYTE];
  module_function = message[MODULE_FUNCTION_BYTE];
}

std::vector<uint8_t> Command::getData(){
  #define DATA_LENGTH_INDEX 5
  #define DATA_INDEX 6

  if (!ready) return std::vector<uint8_t>();

  uint8_t length = message[5];
  uint8_t end_index = DATA_INDEX + length;
  return std::vector<uint8_t>(message.begin() + DATA_INDEX, message.begin() + end_index);
}

void Command::reset(bool init) {
  if (!init) {
  module_function = 0;
  module_id = 0;
  }

  command_id = 0;
  ready = false;
  listening = false;
  message.clear();
}

void Command::reset() {
  reset(false);
}

void Command::checkByte(uint8_t number) {
  if (listening) {
    message.push_back(number);
    if (number == getLTELEndMark()) {
      listening = false;
      ready = checkCRC();
      if (ready) {
        setVars();
      }
    }
    if (message.size() >= max_message_size) {
      reset();
    }
  } else {
    if (number == getLTELStartMark()) {
      message.clear();
      message.push_back(number);
      listening = true;
    }
  }
}

bool Command::checkCRC() {
  #define CRC_BYTE_1_BACKWARD 3
  #define CRC_BYTE_2_BACKWARD 2


	uint16_t crc;
	uint8_t testframe[2];
  uint8_t crc_frame[2] = { message[message.size() - CRC_BYTE_1_BACKWARD], message[message.size() - CRC_BYTE_2_BACKWARD]};

	crc = calculateCRC(1, 3);
	memcpy(testframe, &crc, 2);
	if (testframe[1] == crc_frame[1] && testframe[0] == crc_frame[0]) {
		return true;
	}
	return false;
}

uint16_t Command::calculateCRC(uint8_t start, uint8_t end) {
	uint8_t b;
	uint8_t i;
	uint16_t generator = 0x1021; //divisor is 16bit
	uint16_t crc = 0;			 // CRC value is 16bit
  uint8_t end_byte = static_cast<uint8_t>(message.size()) - end;

	for (b = start; 1 < end_byte; b++) {
		crc ^= ((uint16_t) (message[b] << 8)); // move byte into MSB of 16bit CRC
		for (i = 0; i < 8; i++) {
			if ((crc & 0x8000) != 0) // test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			else
				crc <<= 1;
		}
	}
	return (crc);
}
