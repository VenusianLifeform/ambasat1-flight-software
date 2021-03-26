#ifndef __KORELLIAN_P1_SAT_APP_H__
#define __KORELLIAN_P1_SAT_APP_H__

#include "LoRaPayloadBase.h"
#include "MainControlBoardTelemetryPayload.h"

extern void transmissionReceivedDelegate(uint8_t port, const uint8_t* receivedData, uint8_t receivedDataLen);

class KorellianP1SatApp {
private:
    Hardware _hardware;
    MainControlBoardTelemetryPayload _mainControlBoardTelemetryPayload;

    void sendSensorPayload(LoRaPayloadBase& sensor);
public:
    static KorellianP1SatApp*  gApp;

    KorellianP1SatApp();
    virtual ~KorellianP1SatApp();

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

#endif // __KORELLIAN_P1_SAT_APP_H__