/*
 * UartHandler.hpp
 *
 *  Created on: Jun 18, 2024
 *      Author: artur
 */

#include "main.h"
#include <cstring>
#include <vector>
// Abstract class Function


class Command {
public:
    Command(uint8_t _module_function, uint8_t _module_id, uint8_t max_size);
    Command(uint8_t _module_function, uint8_t _module_id); // Constructor que delega
    virtual ~Command();

    virtual void encode();

    // Getters y Setters
    uint8_t getModuleFunction() const {
        return module_function;
    }
    void setModuleFunction(uint8_t _module_function) {
        module_function = _module_function;
    }

    uint8_t getModuleId() const {
        return module_id;
    }
    void setModuleId(uint8_t _module_id) {
        module_id = _module_id;
    }

    uint8_t getCommandId() const {
        return command_id;
    }
    void setCommandId(uint8_t _command_id) {
        command_id = _command_id;
    }

    bool isListening() const {
        return listening;
    }

    bool isReady() const {
        return ready;
    }

    void setMaxSize(uint8_t max_size) {
        max_message_size = max_size;
    }

    uint8_t getMaxSize() const {
        return max_message_size;
    }

    uint8_t getLTELStartMark() const {
        return LTEL_START_MARK;
    }
    uint8_t getLTELEndMark() const {
        return LTEL_END_MARK;
    }
    uint8_t getMinFrameHeaderSize() const {
        return MIN_FRAME_HEADER_SIZE;
    }

    uint8_t getQueryParameterLTEL() const {
        return QUERY_PARAMETER_LTEL;
    }
    uint8_t getQueryParameterSigma() const {
        return QUERY_PARAMETER_SIGMA;
    }
    uint8_t getQueryParameterStr() const {
        return QUERY_PARAMETER_STR;
    }
    uint8_t getQueryADC() const {
        return QUERY_PARAMETER_ADC;
    }

    uint8_t getSetAttLTEL() const {
        return SET_ATT_LTEL;
    }
    uint8_t getSetPoutMax() const {
        return SET_POUT_MAX;
    }
    uint8_t getSetPoutMin() const {
        return SET_POUT_MIN;
    }

    void checkByte(uint8_t number);

private:
    void setVars();
    void reset(bool init);
    void reset();
    bool prepareTxData(const char *message);
    void handleRxData(uint8_t data);
    bool validateChecksum();
    bool checkCRC();
    uint16_t calculateCRC(uint8_t start, uint8_t end);
    std::vector<uint8_t> getData();

    uint8_t max_message_size;
    uint8_t module_function;
    uint8_t module_id;
    uint8_t command_id;
    const uint8_t LTEL_START_MARK = 0x7e;
    const uint8_t LTEL_END_MARK = 0x7f;
    const uint8_t MIN_FRAME_HEADER_SIZE = 10;

    const uint8_t QUERY_PARAMETER_LTEL = 0x11;
    const uint8_t QUERY_PARAMETER_SIGMA = 0x12;
    const uint8_t QUERY_PARAMETER_STR = 0x15;
    const uint8_t QUERY_PARAMETER_ADC = 0x16;

    const uint8_t SET_ATT_LTEL = 0x20;
    const uint8_t SET_POUT_MAX = 0x24;
    const uint8_t SET_POUT_MIN = 0x23;

    std::vector<uint8_t> message;
    bool listening;
    bool ready;
};
