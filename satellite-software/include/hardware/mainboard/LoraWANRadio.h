#ifndef __LORAWANRADION_H__
#define __LORAWANRADION_H__

#include <stdint.h>
#include <lmic.h>
#include "PersistedConfiguration.h"

typedef void(*IncomingTransmissionDelegate)(uint8_t port, const uint8_t* incomingData, uint8_t dataSize);

class LoraWANRadio
{
    public:
        LoraWANRadio(PersistedConfiguration& config);

        void setup(IncomingTransmissionDelegate incomingTransmissionDelegate);
        void transmit(uint8_t port, const uint8_t* dataToSend, uint8_t dataSize);

        void onTransmissionComplete();
        void onIncomingTransmision(uint8_t port, const uint8_t* incomingData, uint8_t dataSize);
        void onLinkDead();
        void onLinkReset();

    private:
        PersistedConfiguration _config;
        IncomingTransmissionDelegate _incomingTransmissionDelegate;

        volatile bool _isWaitingForTransmissionToFinish;

        friend void onEvent(ev_t event);
};

#endif
