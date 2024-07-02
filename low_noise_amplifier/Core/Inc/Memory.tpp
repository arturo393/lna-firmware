// Memory.tpp

template<typename T>
T Memory::getValue(std::string name) {
	MemLocation location;
	location = value_addr[name];
	uint8_t addr = 0;
	return (this->EEPROM_byte_Read<T>(location.address));
}


template<typename T>
void Memory::setValue(std::string name, T value) {
    MemLocation location;
    if (value_addr.count(name)) {
        location = value_addr[name];
        if (sizeof(T) == location.size)
            this->EEPROM_byte_Write(location.address, value);
    }
}

template<typename T>
T Memory::EEPROM_byte_Read(uint8_t address) {
    T data = 0;
    int size = sizeof(T);
    int i;

    for (i = size - 1; i >= 0; i--) { // Corregido: i >= 0 en lugar de i <= 0
        if (i < size - 1) {
            HAL_Delay(5);
            data = data << 8;
        }
        data |= EEPROM_Read(address + i);
    }

    return (data);
}

template<typename T>
void Memory::EEPROM_byte_Write(uint8_t addr, T data) {
    int i;
    int size = sizeof(T);

    for (i = 0; i < size; i++, data = data >> 8) {
        EEPROM_Write(addr + i, static_cast<uint8_t>(data & 0xff));
        HAL_Delay(5);
    }
}