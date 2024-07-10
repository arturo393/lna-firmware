#include "I2cHandler.hpp"

class I2cBareMetal :public I2cHandler {
	public:
		~I2cBareMetal() {}
		uint8_t byteReceive(char address, uint8_t bytes_to_read) override;
		void byteTransmit(char address, char* buffer, uint8_t bytes_to_write) override;
		void i2c1_init() override;
	protected:
		void startComunication(char, uint8_t, uint8_t); // solo del baremetal
		void scanner(uint8_t *addr); // solo del baremetal
	private:
		void i2c1_start(char saddr, uint8_t transfer_request, uint8_t N);
};
