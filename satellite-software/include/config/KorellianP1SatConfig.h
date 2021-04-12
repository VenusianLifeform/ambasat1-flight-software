#ifndef __KORELLIAN_P1_SAT_CONFIG_H__
#define __KORELLIAN_P1_SAT_CONFIG_H__

#include <Arduino.h>
#include <lmic.h>

// IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT - IMPORTANT
// 
// Set the following three values to match your unique AmbaSat-1 satellite.
// DO NOT commit this file with your private device identifiers into source control.
// KEEP your device identifiers private.
//

// The Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000 ;  //<< CHANGE


//
// Mission Sensor
//
// The AmbaSat comes with multiple main sensor options. Set the AMBASAT_MISSION_SENSOR
// macro to on the of SENSOR_XXXX values to compile for that mission
// sensor.
//
#define SENSOR_SHT30        (1)
#define SENSOR_STS21        (2)
#define SENSOR_BME680       (3)
#define SENSOR_OPT3001DNPT  (4)
#define SENSOR_ZMOD4410AI1V (5)
#define SENSOR_SI1132       (6)
#define SENSOR_CCS811B      (7)
#define SENSOR_TESEO_LIV3R  (8)

#define AMBASAT_MISSION_SENSOR (SENSOR_SI1132)


//
// LED configutation
//
#define LED_PIN ((uint8_t)(9))


///
/// EEPROM Configuration
///
#define EEPROM_BASE_ADDRESS                     (0x0000)
#define EEPROM_TOTAL_SIZE                       (1024)
#define EEPROM_USED_BY_STORAGE_DATA_SIZE        (1000)
#define EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS   (5)
#define EEPROM_MAX_USER_DATA_SIZE               (EEPROM_USED_BY_STORAGE_DATA_SIZE/EEPROM_NUMBER_OF_REDUNTANT_PARTITIONS)
#define EEPROM_USER_DATA_OFFET                  (0x0000)
#define EEPROM_PARTITION_SIZE                   (EEPROM_MAX_USER_DATA_SIZE)
#define EEPROM_GET_OFFSET_FOR_USER_DATA(partition, userDataOffert) \
                                                (EEPROM_USER_DATA_OFFET + partition * EEPROM_PARTITION_SIZE + userDataOffert)




//
// Volage reader configuration
// 
#define VOLTAGE_READER_VREF_SETTLE_DELAY_MILLISECONDS (10)
#define VOLTAGE_READER_MAX_BIT_CHECK_ATTEMPTS (1000)


//
// Main Control Board telemetry configuration
//
#define MAINBOARD_TELEMETRY_PORT_NUMBER                     (1)

#define MAINBOARD_TELEMETRY_BUFFER_REBOOT_COUNT_OFFSET      (0)
#define MAINBOARD_TELEMETRY_BUFFER_VOLTAGE_OFFSET           (4)
#define MAINBOARD_TELEMETRY_BUFFER_SENSOR_STATUS_OFFSET     (6)
#define MAINBOARD_TELEMETRY_SATELLITE_STATUS_BUFFER_SIZE    (7)

#define MAINBOARD_TELEMETRY_SENSOR_STATUS_LSM9DS1_FOUND     0b00000001
#define MAINBOARD_TELEMETRY_SENSOR_STATUS_LSM9DS1_ACTIVE    0b00000010
#define MAINBOARD_TELEMETRY_SENSOR_STATUS_MISSION_FOUND     0b00010000
#define MAINBOARD_TELEMETRY_SENSOR_STATUS_MISSION_ACTIVE    0b00100000


#endif // __KORELLIAN_P1_SAT_CONFIG_H__