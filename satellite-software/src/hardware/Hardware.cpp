#include "Hardware.h"
#include "AmbaSat1Config.h"
#include "Logging.h"

// Static instance
Hardware* Hardware::g_hardware = nullptr;

Hardware::Hardware() : 
    _config(),
    _loraWANRadio(_config),
    _voltageReader(VOLTAGE_READER_VREF_SETTLE_DELAY_MILLISECONDS, VOLTAGE_READER_MAX_BIT_CHECK_ATTEMPTS),
    _ledController(LED_PIN),
    _lsm9DS1Sensor(_config),
#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    _Si1132Sensor(_config)
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
    _SHT30Sensor(_config)
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
    _STS21Sensor(_config)
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
    BME680Sensor(_config)
#endif  // AMBASAT_MISSION_SENSOR
{
    if (Hardware::g_hardware != nullptr) {
        // complain loudly. Only one app object should be created.
        PRINT_ERROR(F("ERROR multiple Hardware objs"));
    }
    else {
        Hardware::g_hardware = this;
    }
}

Hardware::~Hardware()
{
    Hardware::g_hardware = nullptr;
}

PersistedConfiguration& Hardware::getPersistedConfiguration() const
{
    return (PersistedConfiguration&)_config;
}

LoraWANRadio& Hardware::getLoraWANRadio() const
{
    return (LoraWANRadio&)_loraWANRadio;
}

VoltageReader& Hardware::getVoltageReader() const
{
    return (VoltageReader&)_voltageReader;
}

LEDController& Hardware::getLEDController() const
{
    return (LEDController&)_ledController;
}

LSM9DS1Sensor& Hardware::getLSM9DS1Sensor() const
{
    return (LSM9DS1Sensor&)_lsm9DS1Sensor;
}

#if AMBASAT_MISSION_SENSOR == SENSOR_SI1132
Si1132Sensor& Hardware::getSi1132Sensor() const
{
    return (Si1132Sensor&)_Si1132Sensor;
}

SensorBase& Hardware::getMissionSensor() const
{
    return this->getSi1132Sensor();
}
#elif AMBASAT_MISSION_SENSOR == SENSOR_SHT30
SHT30Sensor& Hardware::getSHT30Sensor() const
{
    return (SHT30Sensor&)_SHT30Sensor;
}

SensorBase& Hardware::getMissionSensor() const
{
    return this->getSHT30Sensor();
}
#elif AMBASAT_MISSION_SENSOR == SENSOR_STS21
STS21Sensor& Hardware::getSTS21Sensor() const
{
    return (STS21Sensor&)_STS21Sensor;
}

SensorBase& Hardware::getMissionSensor() const
{
    return this->getSTS21Sensor();
}
#elif AMBASAT_MISSION_SENSOR == SENSOR_BME680
BME680Sensor& Hardware::getBME680Sensor() const
{
    return (BME680Sensor&)_BME680Sensor;
}

SensorBase& Hardware::getMissionSensor() const
{
    return this->getBME680Sensor();
}
#endif  // AMBASAT_MISSION_SENSOR
