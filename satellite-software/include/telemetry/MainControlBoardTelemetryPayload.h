#ifndef __MAIN_CONTROL_BOARD_TELEMETRY_PAYLOAD_H__
#define __MAIN_CONTROL_BOARD_TELEMETRY_PAYLOAD_H__

#include "Hardware.h"
#include "KorellianP1SatConfig.h"
#include "LoRaPayloadBase.h"

class MainControlBoardTelemetryPayload : public LoRaPayloadBase
{
    private:
        Hardware& _hardware;
        uint8_t _buffer[MAINBOARD_TELEMETRY_SATELLITE_STATUS_BUFFER_SIZE];

    public:
        MainControlBoardTelemetryPayload(Hardware& hardware);

        virtual const uint8_t* getCurrentMeasurementBuffer(void);
        virtual uint8_t getMeasurementBufferSize() const;
        virtual uint8_t getPort() const;
};

#endif
