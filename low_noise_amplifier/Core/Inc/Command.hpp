/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "main.h"
// Abstract class Function
class Command {

public:
	Command(uint8_t _module_function, uint8_t _module_id,
			uint8_t _max_rx_buffer_size, uint8_t _max_tx_buffer_size);
	virtual ~Command();

	// Virtual destructor (optional)
	virtual void encode();
	virtual void decode();
	// Getters for private members
	uint8_t getModuleFunction() {
		return(module_function);
	}
	void setModuleFunction(uint8_t _module_function){
		module_function = _module_function;
	}
	uint8_t getModuleId()  {
		return(module_function);
	}
	void setModuleId(uint8_t _module_id){
		module_id = _module_id;
	}
	uint8_t getLTELStartMark()  {
		return (LTEL_START_MARK);
	}
	uint8_t getLTELEndMark() const {
		return (LTEL_END_MARK);
	}
	uint8_t getMinFrameHeaderSize() const {
		return (MIN_FRAME_HEADER_SIZE);
	}

	uint8_t getQueryParameterLTEL() const {
		return (QUERY_PARAMETER_LTEL);
	}
	uint8_t getQueryParameterSigma() const {
		return (QUERY_PARAMETER_SIGMA);
	}
	uint8_t getQueryParameterStr() const {
		return (QUERY_PARAMETER_STR);
	}
	uint8_t getQueryADC() const {
		return (QUERY_PARAMETER_ADC);
	}

	uint8_t getSetAttLTEL() const {
		return (SET_ATT_LTEL);
	}
	uint8_t getSetPoutMax() const {
		return (SET_POUT_MAX);
	}
	uint8_t getSetPoutMin() const {
		return (SET_POUT_MIN);
	}

private:

	uint8_t max_rx_buffer_size;
	uint8_t max_tx_buffer_size;
	uint8_t *rx_buffer;
	uint8_t *tx_buffer;
	uint8_t tx_buffer_size;

	uint8_t module_function;
	uint8_t module_id;
	uint8_t LTEL_START_MARK = 0x7e;
	uint8_t LTEL_END_MARK = 0x7f;
	uint8_t MIN_FRAME_HEADER_SIZE = 10;

	uint8_t QUERY_PARAMETER_LTEL = 0x11;
	uint8_t QUERY_PARAMETER_SIGMA = 0x12;
	uint8_t QUERY_PARAMETER_STR = 0x15;
	uint8_t QUERY_PARAMETER_ADC = 0x16;

	uint8_t SET_ATT_LTEL = 0x20;
	uint8_t SET_POUT_MAX = 0x24;
	uint8_t SET_POUT_MIN = 0x23;
};

