#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "PersistedConfiguration.h"
#include "VoltageReader.h"
#include "LEDController.h"
#include "LSM9DS1Sensor.h"
#include "LoraWANRadio.h"

#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    #include "Si1132Sensor.h"
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
    #include "SHT30Sensor.h"
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
    #include "STS21Sensor.h"
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
    #include "BME680Sensor.h"
#endif  // AMBASAT_MISSION_SENSOR

class Hardware
{
    private:
        PersistedConfiguration _config;
        LoraWANRadio _loraWANRadio;
        VoltageReader _voltageReader;
        LEDController _ledController;
        LSM9DS1Sensor _lsm9DS1Sensor;
#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
        Si1132Sensor _Si1132Sensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
        SHT30Sensor _SHT30Sensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
        STS21Sensor _STS21Sensor;
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
        BME680Sensor _BME680Sensor;
#endif  // AMBASAT_MISSION_SENSOR

    public:
        Hardware();
        ~Hardware();

        static Hardware* g_hardware;

        PersistedConfiguration& getPersistedConfiguration() const;
        LoraWANRadio& getLoraWANRadio() const;
        VoltageReader& getVoltageReader() const;
        LEDController& getLEDController() const;
        LSM9DS1Sensor& getLSM9DS1Sensor() const;
        SensorBase& getMissionSensor() const;
#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
        Si1132Sensor& getSi1132Sensor() const;
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
        SHT30Sensor& getSHT30Sensor() const;
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
        STS21Sensor& getSTS21Sensor() const;
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
        BME680Sensor& getBME680Sensor() const;
#endif  // AMBASAT_MISSION_SENSOR

};

#endif
