#include <UartHandler.hpp>

bool UartHandler::transmitMessage(char* message){
	uint8_t i;
	for (i = 0; message[i] != '\0'; i++)
		uart_write(&message[i]);

	return (true);
}

bool UartHandler::transmitData(uint8_t *data, uint8_t data_bytes){
	for (int i = 0; i < data_bytes; i++)
		uart_write(reinterpret_cast<char*>(&data[i]));
	return (false);
}

bool UartHandler::transmitCommand(CommandMessage command) {
	uint8_t* data = command.getMessage().data();
	uint8_t size = command.getMessage().size();
	return this->transmitData(data, size);
}
