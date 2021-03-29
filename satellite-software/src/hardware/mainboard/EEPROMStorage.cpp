#include <eeprom.h>
#include "EEPROMStorage.h"
#include "KorellianP1SatConfig.h"

uint8_t EEPROMStorage::readUInt8(uint32_t offset)
{
    return eeprom_read_byte((const uint8_t*)(EEPROM_BASE_ADDRESS + offset));
}

uint16_t EEPROMStorage::readUInt16(uint32_t offset)
{
    return eeprom_read_word((const uint16_t*)(EEPROM_BASE_ADDRESS + offset));
}

uint32_t EEPROMStorage::readUInt32(uint32_t offset)
{
    return eeprom_read_dword((const uint32_t*)(EEPROM_BASE_ADDRESS + offset));
}

void EEPROMStorage::readArrayOfBytes(uint32_t offset, uint8_t* buffer, uint8_t bufferSize)
{
    eeprom_read_block(buffer, (const void*)(EEPROM_BASE_ADDRESS + offset), bufferSize);
}

void EEPROMStorage::writeUInt8(uint32_t offset, uint8_t valueToWrite)
{
    eeprom_write_byte((uint8_t*)(EEPROM_BASE_ADDRESS + offset), valueToWrite);
}

void EEPROMStorage::writeUInt16(uint32_t offset, uint16_t valueToWrite)
{
    eeprom_write_word((uint16_t*)(EEPROM_BASE_ADDRESS + offset), valueToWrite);
}

void EEPROMStorage::writeUInt32(uint32_t offset, uint32_t valueToWrite)
{
    eeprom_write_dword((uint32_t*)(EEPROM_BASE_ADDRESS + offset), valueToWrite);
}
