#include "AmbaSat1App.h"
#include <LowPower.h>
#include "Utilities.h"
#include "Logging.h"
#include "AmbaSat1Config.h"


//
// Global Variable so LMIC functions can access app object
//
AmbaSat1App* AmbaSat1App::gApp = nullptr;

AmbaSat1App::AmbaSat1App() : 
    _hardware(),
    _mainControlBoardTelemetryPayload(_hardware)
#ifdef ENABLE_AMBASAT_COMMANDS
        ,
        _queuedCommandPort(0xFF)
#endif
{
    if (AmbaSat1App::gApp != nullptr) {
        // complain loudly. Only one app object should be created.
        PRINT_ERROR(F("ERROR multiple app objs"));
        return;
    }
    
    AmbaSat1App::gApp = this;
    
    _hardware.getPersistedConfiguration().setSensorConfigDelegates(
        &(_hardware.getLSM9DS1Sensor()), &(_hardware.getMissionSensor()));
}

AmbaSat1App::~AmbaSat1App()
{
    AmbaSat1App::gApp = nullptr;
}

void AmbaSat1App::setup()
{
    // Turn on LED during setup
    this->_hardware.getLEDController().switchOn();

    this->_hardware.getPersistedConfiguration().init();
    this->_hardware.getLSM9DS1Sensor().setup();
    this->_hardware.getMissionSensor().setup();
    this->_hardware.getLoraWANRadio().setup(transmissionReceivedDelegate);

    // Finished Setting up. Tun LED off
    this->_hardware.getLEDController().switchOff();
}

void AmbaSat1App::loop()
{
    uint8_t pattern = this->_hardware.getPersistedConfiguration().getUplinkPattern();
    UplinkPayloadType lastPayload = this->_hardware.getPersistedConfiguration().getLastPayloadUplinked();
    PRINT_INFO(F("Transmit pattern "));
    PRINT_INFO(pattern);
    PRINT_INFO(F(", last payload "));
    PRINT_INFO(lastPayload);
    PRINT_INFO(F("\n"));

    int8_t transmitQueue[3] = {-1, -1, -1};

    switch (pattern) {
        default:
        case 0:
            // transmit all sensors in sequence
            transmitQueue[0] = SATTELITE_PAYLOAD;
            transmitQueue[1] = LSM9DS1_PAYLOAD;
            transmitQueue[2] = MISSION_SENSOR_PAYLOAD;
            break;
        case 1:
            // transmit all sensors in sequence but rotate starting point
            transmitQueue[0] = (lastPayload+2)%3;
            transmitQueue[1] = (lastPayload+3)%3;
            transmitQueue[2] = (lastPayload+4)%3;
            break;
        case 2:
            // transmit oly one sensor
            transmitQueue[0] = (lastPayload+1)%3;
            break;
        case 3:
            // transmit satellite data and alternate between LSM9DS1 and mission
            transmitQueue[0] = SATTELITE_PAYLOAD;
            transmitQueue[1] = lastPayload == LSM9DS1_PAYLOAD ? MISSION_SENSOR_PAYLOAD : LSM9DS1_PAYLOAD;
            break;
    }

    for (int8_t i = 0; i < 3; i++) {
        if (transmitQueue[i] >= 0) {
            switch (transmitQueue[i]) {
                case SATTELITE_PAYLOAD:
                    PRINTLN_INFO(F("Sending Satellite Status"));
                    sendSensorPayload(this->_mainControlBoardTelemetryPayload);
                    break;
                case LSM9DS1_PAYLOAD:
                    if (this->_hardware.getLSM9DS1Sensor().isActive()) {
                        PRINTLN_INFO(F("Sending LSM9DS1 sensor"));
                        sendSensorPayload(this->_hardware.getLSM9DS1Sensor());
                    }
                    break;
                case MISSION_SENSOR_PAYLOAD:
                    if (this->_hardware.getMissionSensor().isActive()) {
                        PRINTLN_INFO(F("Sending mission sensor"));
                        sendSensorPayload(this->_hardware.getMissionSensor());
                    }
                    break;
                default:
                    // this should never happen
                    break;
            }
            lastPayload = transmitQueue[i];
        } else {
            break;
        }
    }

    // defer CRC calculation until subsequent frame count config being set
    this->_hardware.getPersistedConfiguration().setLastPayloadUplinked(lastPayload);
    //
    // technically there is some risk that the satellite will loose power between
    // the first transmission above and the last one, and in such case we will not
    // capture the uplink frame count. We are accepting that risk in order to reduce
    // number of time we write to EEPROM.
    //
    this->_hardware.getPersistedConfiguration().setUplinkFrameCount(LMIC.seqnoUp);
    this->_hardware.getPersistedConfiguration().updateCRC();

    #ifdef ENABLE_AMBASAT_COMMANDS
    // handle any command that got queued
    processQueuedCommand();
    #endif

    // flush serial before going to sleep
    Serial.flush();

    // sleep device for designated sleep cycles
    for (int i=0; i < this->_hardware.getPersistedConfiguration().getUplinkSleepCycles(); i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds * sleepcycles
    }
}

void AmbaSat1App::incomingTransmission(uint8_t port, const uint8_t* receivedData, uint8_t receivedDataLen)
{
    // TODO: handle commands
}

extern void transmissionReceivedDelegate(uint8_t port, const uint8_t* receivedData, uint8_t receivedDataLen)
{
    if (AmbaSat1App::gApp != nullptr)
    {
        AmbaSat1App::gApp->incomingTransmission(port, receivedData, receivedDataLen);
    }
}

void AmbaSat1App::sendSensorPayload(LoRaPayloadBase& sensor)
{
    const uint8_t* data_ptr = sensor.getCurrentMeasurementBuffer();
    if (data_ptr == nullptr) {
        PRINTLN_INFO(F("  Sensor data is NULL"));
        return;
    }

#if LOG_LEVEL >= LOG_LEVEL_INFO
    PRINT_INFO(F("  Sending payload = "));
    print_buffer(data_ptr, sensor.getMeasurementBufferSize());
    Serial.flush();
#endif

    _hardware.getLoraWANRadio().transmit(
        sensor.getPort(), 
        data_ptr, 
        sensor.getMeasurementBufferSize());
}

#ifdef ENABLE_AMBASAT_COMMANDS
void AmbaSat1App::queueCommand(uint8_t port, uint8_t* receivedData, uint8_t receivedDataLen)
{
    if ((_queuedCommandPort == 0xFF) && (receivedDataLen <= QUEUED_COMMAND_BUFFER_SIZE )) {
        _queuedCommandPort = port;
        memcpy(_queuedCommandDataBuffer, receivedData, receivedDataLen);
        _queuedCommandDataLength = receivedDataLen;
        PRINT_DEBUG(F("  queued cmd on port "));
        PRINT_DEBUG(port);
        PRINT_DEBUG(F(" with data len = "));
        PRINT_DEBUG(receivedDataLen);
        PRINT_DEBUG(F("\n"));
    }
}

void AmbaSat1App::processQueuedCommand(void)
{
    if (_queuedCommandPort == 0xFF) {
        PRINTLN_DEBUG(F("No queued cmds to process"));
        return;
    }

    // this method decodes the command header and routes the data to the appropriate handler
    PRINT_DEBUG(F("received cmd on port "));
    PRINT_DEBUG(_queuedCommandPort);
    PRINT_DEBUG(F(", payload = "));
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    print_buffer(_queuedCommandDataBuffer, _queuedCommandDataLength);
#endif

    // The port is used to determine which handler the command should be routed to:
    //      port 2 = satellite commands
    //      port 3 = LSM9DS1 commands
    //      port 4 = Mission sensor comands

    // The command payload is:
    //      uint16_t - command sequence ID: ground software uses this value to identify the command sent
    //      uint8_t  - command: the command ID, to be interpreted by the command handler.
    //      uint8_t* - comannd data: a variable length of data that provides parameters to the command.
    //                 The size of this data blob is determined by the command.
    //

    uint16_t cmdSeuqenceID = (uint16_t)ntoh_int16(&_queuedCommandDataBuffer[0]);
    uint8_t cmd = _queuedCommandDataBuffer[2];
    uint8_t* cmdData = _queuedCommandDataLength > 3 ? &_queuedCommandDataBuffer[3] : nullptr;
    uint8_t cmdDataSize = _queuedCommandDataLength >= 3 ? (_queuedCommandDataLength-3) : 0;

    uint8_t status = CMD_STATUS_UNIMPLEMENTED;
    switch (_queuedCommandPort) {
        case 2:
            status = this->handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        case 3:
            status = _lsm9DS1Sensor.handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        case 4:
            status = _missionSensor.handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        default:
            PRINT_ERROR(F("ERROR received cmd on port "));
            PRINT_ERROR(_queuedCommandPort);
            PRINT_ERROR(F("\n"));
            break;
    }

    // reset port back to no command
    _queuedCommandPort = 0xFF;

    // responde with command status
    uint8_t replyBuffer[3];
    hton_int16(cmdSeuqenceID, &replyBuffer[0]);
    replyBuffer[2] = status;

#if LOG_LEVEL >= LOG_LEVEL_INFO
    PRINT_INFO(F("  Sending cmd response payload = "));
    print_buffer(replyBuffer, 3);
#endif

    _hardware.getLoraWANRadio().transmit(
        PORT_CMD_STATUS, 
        replyBuffer, 
        3);

}

uint8_t AmbaSat1App::handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* receivedData, uint8_t receivedDataLen)
{
    // Commands are identified in the first byte. Commands that the satellite supports:
    //
    //  0x01 - Blink LED. The second byte is is split into two nibbles. The uppper 2 bits indicate the
    //         blink period as follows:
    //              00 = 0.1 second blinks
    //              01 = 0.5 second blinks
    //              10 = 1 second blinks
    //              11 = 2 second blinks.
    //         The lower 6 bits are used to indicate the number of blinks.
    //

    if (command == 0x01) {
        return executeBlinkCmd(receivedData, receivedDataLen);
    } else if (command == 0x02) {
        return executeUplinkPatternCmd(receivedData, receivedDataLen);
    } else if (command == 0x03) {
        return executeUplinkRateCmd(receivedData, receivedDataLen);
    } else if (command == 0x04) {
        return executeSetFrameCountCmd(receivedData, receivedDataLen);
    }

    return CMD_STATUS_UNKNOWN_CMD;
}

uint8_t AmbaSat1App::executeBlinkCmd(uint8_t* receivedData, uint8_t receivedDataLen)
{
    if (receivedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint8_t blinkCount = receivedData[0]&0x3F;
    uint16_t blinkDurationMillis = 100;
    switch (receivedData[0]&0xC0) {
        default:
        case 0x00:
            blinkDurationMillis = 100;
            break;
        case 0x40:
            blinkDurationMillis = 500;
            break;
        case 0x80:
            blinkDurationMillis = 1000;
            break;
        case 0xC0:
            blinkDurationMillis = 2000;
            break;
    }
    PRINT_DEBUG(F("  BLINK! "));
    PRINT_DEBUG(blinkCount);
    PRINT_DEBUG(F("x, dur = "));
    PRINT_DEBUG(blinkDurationMillis);
    PRINT_DEBUG(F(" ms\n"));

    for (int16_t i = 0; i <blinkCount; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(blinkDurationMillis);
        digitalWrite(LED_PIN, LOW);
        if (i < blinkCount-1) {
            delay(blinkDurationMillis);
        }
    }
    delay(100);
    return CMD_STATUS_SUCCESS;
}

uint8_t AmbaSat1App::executeUplinkPatternCmd(uint8_t* receivedData, uint8_t receivedDataLen)
{
    if (receivedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }
    UplinkPayloadType pattern = static_cast<UplinkPayloadType>(receivedData[0]);
    _config.setUplinkPattern(pattern);
    _config.updateCRC();

    PRINT_DEBUG(F("  set uplink pattern: "));
    PRINT_DEBUG(pattern);
    PRINT_DEBUG(F("\n"));
    return CMD_STATUS_SUCCESS;
}

uint8_t AmbaSat1App::executeUplinkRateCmd(uint8_t* receivedData, uint8_t receivedDataLen)
{
    if (receivedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint8_t rateValue = receivedData[0];
    _config.setUplinkSleepCycles(rateValue);
    _config.updateCRC();
    PRINT_DEBUG(F("  set uplink rate: "));
    PRINT_DEBUG(rateValue);
    PRINT_DEBUG(F("\n"));
    return CMD_STATUS_SUCCESS;
}

uint8_t AmbaSat1App::executeSetFrameCountCmd(uint8_t* receivedData, uint8_t receivedDataLen)
{
    if (receivedDataLen != 2 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint16_t frameCount = (uint16_t)ntoh_int16(receivedData);
    _config.setUplinkFrameCount(frameCount);
    _config.updateCRC();
    LMIC.seqnoUp = frameCount;

    PRINT_DEBUG(F("  reset uplink frame count: "));
    PRINT_DEBUG(frameCount);
    PRINT_DEBUG(F("\n"));

    return CMD_STATUS_SUCCESS;
}

#endif
