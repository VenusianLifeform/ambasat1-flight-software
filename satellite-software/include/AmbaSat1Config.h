#ifndef __AmbaSat1Config__
#define __AmbaSat1Config__
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
#define SENSOR_SHT30        1
#define SENSOR_STS21        2
#define SENSOR_BME680       3
#define SENSOR_OPT3001DNPT  4
#define SENSOR_ZMOD4410AI1V 5
#define SENSOR_SI1132       6
#define SENSOR_CCS811B      7
#define SENSOR_TESEO_LIV3R  8

#define AMBASAT_MISSION_SENSOR SENSOR_SI1132


//
// LED configutation
//
#define LED_PIN ((uint8_t)(9))


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


#endif // __AmbaSat1Config__