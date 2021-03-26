#ifndef __AmbaSat1App__
#define __AmbaSat1App__
#include <lmic.h>
#include "AmbaSat1Config.h"
#include "LoRaPayloadBase.h"
#include "Sensors.h"
#include "PersistedConfiguration.h"
#include "LEDController.h"
#include "VoltageReader.h"
#include "MainControlBoardTelemetryPayload.h"

extern void transmissionReceivedDelegate(uint8_t port, const uint8_t* receivedData, uint8_t receivedDataLen);

class AmbaSat1App {
private:
    Hardware _hardware;
    MainControlBoardTelemetryPayload _mainControlBoardTelemetryPayload;

    void sendSensorPayload(LoRaPayloadBase& sensor);

    friend void onEvent (ev_t ev);
public:
    static AmbaSat1App*  gApp;

    AmbaSat1App();
    virtual ~AmbaSat1App();

    // standard Arduino functions
    void setup();
    void loop();
    void incomingTransmission(uint8_t port, const uint8_t* receivedData, uint8_t receivedDataLen);

 
    //
    // Command Handling (if enabled)
    //
#ifdef ENABLE_AMBASAT_COMMANDS
    void queueCommand(uint8_t port, uint8_t* receivedData, uint8_t receivedDataLen);
    void processQueuedCommand(void);
    virtual uint8_t handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* receivedData, uint8_t receivedDataLen);

    uint8_t executeBlinkCmd(uint8_t* receivedData, uint8_t receivedDataLen);
    uint8_t executeUplinkPatternCmd(uint8_t* receivedData, uint8_t receivedDataLen);
    uint8_t executeUplinkRateCmd(uint8_t* receivedData, uint8_t receivedDataLen);
    uint8_t executeSetFrameCountCmd(uint8_t* receivedData, uint8_t receivedDataLen);

private:
    #define QUEUED_COMMAND_BUFFER_SIZE MAX_LEN_FRAME
    uint8_t _queuedCommandPort;
    uint8_t _queuedCommandDataBuffer[QUEUED_COMMAND_BUFFER_SIZE];
    uint8_t _queuedCommandDataLength;
#endif

};

#ifdef __cplusplus
extern "C"{
#endif

void initfunc (osjob_t* j);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //__AmbaSat1App__