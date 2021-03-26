#include "MainControlBoardTelemetryPayload.h"
#include "Utilities.h"

MainControlBoardTelemetryPayload::MainControlBoardTelemetryPayload(Hardware& hardware) : 
    _hardware(hardware)
{
}

uint8_t MainControlBoardTelemetryPayload::getMeasurementBufferSize() const 
{
    return MAINBOARD_TELEMETRY_SATELLITE_STATUS_BUFFER_SIZE;
}

uint8_t MainControlBoardTelemetryPayload::getPort() const 
{
    return MAINBOARD_TELEMETRY_PORT_NUMBER;
}

const uint8_t* MainControlBoardTelemetryPayload::getCurrentMeasurementBuffer(void)
{
    // Buffer format
    //
    //      uint32_t    reboot count
    //      uint16_t    voltage
    //      uint8_t     sensor status
    //
    //  TOTAL BUFFER SIZE --> 7 bytes
    //

    hton_int32(
        this->_hardware.getPersistedConfiguration().getRebootCount(), 
        &(this->_buffer[MAINBOARD_TELEMETRY_BUFFER_REBOOT_COUNT_OFFSET]));
    hton_int16(
        this->_hardware.getVoltageReader().readVoltage(), 
        &(_buffer[MAINBOARD_TELEMETRY_BUFFER_VOLTAGE_OFFSET]));

    // calculate the sensor status byte
    uint8_t sensorStatus = 0x00;
    if (this->_hardware.getLSM9DS1Sensor().isFound()) {
        sensorStatus |= MAINBOARD_TELEMETRY_SENSOR_STATUS_LSM9DS1_FOUND;
    }

    if (this->_hardware.getLSM9DS1Sensor().isActive()) {
        sensorStatus |= MAINBOARD_TELEMETRY_SENSOR_STATUS_LSM9DS1_ACTIVE;
    }

    if (this->_hardware.getMissionSensor().isFound()) {
        sensorStatus |= MAINBOARD_TELEMETRY_SENSOR_STATUS_MISSION_FOUND;
    }

    if (this->_hardware.getMissionSensor().isActive()) {
        sensorStatus |= MAINBOARD_TELEMETRY_SENSOR_STATUS_MISSION_ACTIVE;
    }

    _buffer[MAINBOARD_TELEMETRY_BUFFER_SENSOR_STATUS_OFFSET] = sensorStatus;
    return _buffer;
}