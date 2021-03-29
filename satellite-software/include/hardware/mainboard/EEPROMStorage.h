#ifndef __EEPROM_STORAGE_H__
#define __EEPROM_STORAGE_H__

#include <stdint.h>

class EEPROMStorage
{
    public:
        uint8_t readUInt8(uint32_t offset);
        uint16_t readUInt16(uint32_t offset);
        uint32_t readUInt32(uint32_t offset);
        void readArrayOfBytes(uint32_t offset, uint8_t* buffer, uint8_t bufferSize);

        void writeUInt8(uint32_t offset, uint8_t valueToWrite);
        void writeUInt16(uint32_t offset, uint16_t valueToWrite);
        void writeUInt32(uint32_t offset, uint32_t valueToWrite);
};

#endif // __EEPROM_STORAGE_H__