#include <eeprom.h>
#include "EEPROMStorage.h"
#include "KorellianP1SatConfig.h"

uint8_t EEPROMStorage::readUInt8(uint32_t offset)
{
    uint8_t uniqueValuesReadFromPartitions[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };
    uint8_t uniqueValuesNumberOfApprearences[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };

    // Loop over all paritions and read the values
    // There we determine the frequence of each value's occurance
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        // Read value from the current partition
        uint8_t readValue = eeprom_read_byte(
            (const uint8_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)));
        
        // Loop over all unique values we have found during previous loop iterations
        // and check if the new value is unique or not
        // If the value is unique then store it
        // Otherwise increase number of apperences for an already existing unique value
        uint8_t uniqueNumberIndex = 0;
        for (;uniqueNumberIndex < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; uniqueNumberIndex++)
        {
            if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == 0) 
            {
                // We have found a new unique value
                // Save the number and set number of appearences for this value to 1
                uniqueValuesReadFromPartitions[uniqueNumberIndex] = readValue;
                uniqueValuesNumberOfApprearences[uniqueNumberIndex] = 1;
                break;
            }
            else 
            {
                // This is a previously found unique value
                if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == readValue) 
                {
                    // This value was previously read from another partition. Good!
                    // Increase number of appearences for this value
                    uniqueValuesNumberOfApprearences[uniqueNumberIndex] = uniqueValuesNumberOfApprearences[uniqueNumberIndex] + 1;
                    break;
                }

                // The value that was read from the current partition may still be unique, keep looking
            }
        }
    }
            
    // Now find the value with most appearences
    uint8_t indexOfValueWithMostAppearencens = 0;
    uint8_t maxNumberOfAppearences = 0;
    for (uint8_t i = 0; i < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; i ++)
    {
        if (uniqueValuesNumberOfApprearences[i] > maxNumberOfAppearences)
        {
            indexOfValueWithMostAppearencens = i;
            maxNumberOfAppearences = uniqueValuesNumberOfApprearences[i];
        }
    }

    // Get the value with most numbers of appearences
    uint8_t valueWithMostAppearences = uniqueValuesReadFromPartitions[indexOfValueWithMostAppearencens];

    // If there is some data inconsistency then write it back to ensure consistency
    if (maxNumberOfAppearences != EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS)
    {
        writeUInt8(offset, valueWithMostAppearences);
    }

    return valueWithMostAppearences;
}

uint16_t EEPROMStorage::readUInt16(uint32_t offset)
{
    uint16_t uniqueValuesReadFromPartitions[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };
    uint8_t uniqueValuesNumberOfApprearences[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };

    // Loop over all paritions and read the values
    // There we determine the frequence of each value's occurance
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        // Read value from the current partition
        uint8_t readValue = eeprom_read_word(
            (const uint16_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)));
        
        // Loop over all unique values we have found during previous loop iterations
        // and check if the new value is unique or not
        // If the value is unique then store it
        // Otherwise increase number of apperences for an already existing unique value
        uint8_t uniqueNumberIndex = 0;
        for (;uniqueNumberIndex < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; uniqueNumberIndex++)
        {
            if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == 0) 
            {
                // We have found a new unique value
                // Save the number and set number of appearences for this value to 1
                uniqueValuesReadFromPartitions[uniqueNumberIndex] = readValue;
                uniqueValuesNumberOfApprearences[uniqueNumberIndex] = 1;
                break;
            }
            else 
            {
                // This is a previously found unique value
                if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == readValue) 
                {
                    // This value was previously read from another partition. Good!
                    // Increase number of appearences for this value
                    uniqueValuesNumberOfApprearences[uniqueNumberIndex] = uniqueValuesNumberOfApprearences[uniqueNumberIndex] + 1;
                    break;
                }

                // The value that was read from the current partition may still be unique, keep looking
            }
        }
    }
            
    // Now find the value with most appearences
    uint8_t indexOfValueWithMostAppearencens = 0;
    uint8_t maxNumberOfAppearences = 0;
    for (uint8_t i = 0; i < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; i ++)
    {
        if (uniqueValuesNumberOfApprearences[i] > maxNumberOfAppearences)
        {
            indexOfValueWithMostAppearencens = i;
            maxNumberOfAppearences = uniqueValuesNumberOfApprearences[i];
        }
    }

    // Get the value with most numbers of appearences
    uint16_t valueWithMostAppearences = uniqueValuesReadFromPartitions[indexOfValueWithMostAppearencens];

    // If there is some data inconsistency then write it back to ensure consistency
    if (maxNumberOfAppearences != EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS)
    {
        writeUInt16(offset, valueWithMostAppearences);
    }

    return valueWithMostAppearences;
}

uint32_t EEPROMStorage::readUInt32(uint32_t offset)
{
    uint32_t uniqueValuesReadFromPartitions[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };
    uint8_t uniqueValuesNumberOfApprearences[EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS] = { 0 };

    // Loop over all paritions and read the values
    // There we determine the frequence of each value's occurance
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        // Read value from the current partition
        uint8_t readValue = eeprom_read_dword(
            (const uint32_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)));
        
        // Loop over all unique values we have found during previous loop iterations
        // and check if the new value is unique or not
        // If the value is unique then store it
        // Otherwise increase number of apperences for an already existing unique value
        uint8_t uniqueNumberIndex = 0;
        for (;uniqueNumberIndex < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; uniqueNumberIndex++)
        {
            if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == 0) 
            {
                // We have found a new unique value
                // Save the number and set number of appearences for this value to 1
                uniqueValuesReadFromPartitions[uniqueNumberIndex] = readValue;
                uniqueValuesNumberOfApprearences[uniqueNumberIndex] = 1;
                break;
            }
            else 
            {
                // This is a previously found unique value
                if (uniqueValuesNumberOfApprearences[uniqueNumberIndex] == readValue) 
                {
                    // This value was previously read from another partition. Good!
                    // Increase number of appearences for this value
                    uniqueValuesNumberOfApprearences[uniqueNumberIndex] = uniqueValuesNumberOfApprearences[uniqueNumberIndex] + 1;
                    break;
                }

                // The value that was read from the current partition may still be unique, keep looking
            }
        }
    }
            
    // Now find the value with most appearences
    uint8_t indexOfValueWithMostAppearencens = 0;
    uint8_t maxNumberOfAppearences = 0;
    for (uint8_t i = 0; i < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; i ++)
    {
        if (uniqueValuesNumberOfApprearences[i] > maxNumberOfAppearences)
        {
            indexOfValueWithMostAppearencens = i;
            maxNumberOfAppearences = uniqueValuesNumberOfApprearences[i];
        }
    }

    // Get the value with most numbers of appearences
    uint32_t valueWithMostAppearences = uniqueValuesReadFromPartitions[indexOfValueWithMostAppearencens];

    // If there is some data inconsistency then write it back to ensure consistency
    if (maxNumberOfAppearences != EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS)
    {
        writeUInt32(offset, valueWithMostAppearences);
    }

    return valueWithMostAppearences;
}

void EEPROMStorage::readArrayOfBytes(uint32_t offset, uint8_t* buffer, uint8_t bufferSize)
{
    for (uint8_t i = 0; i < bufferSize; i ++)
    {
        *(buffer + i) = this->readUInt8(offset + i);
    }
}

void EEPROMStorage::writeUInt8(uint32_t offset, uint8_t valueToWrite)
{
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        eeprom_write_byte((uint8_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)), valueToWrite);
    }
}

void EEPROMStorage::writeUInt16(uint32_t offset, uint16_t valueToWrite)
{
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        eeprom_write_word((uint16_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)), valueToWrite);
    }
}

void EEPROMStorage::writeUInt32(uint32_t offset, uint32_t valueToWrite)
{
    for (int partition = 0; partition < EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS; partition ++)
    {
        eeprom_write_dword((uint32_t*)(EEPROM_GET_OFFSET_FOR_USER_DATA(partition, offset)), valueToWrite);
    }
}
