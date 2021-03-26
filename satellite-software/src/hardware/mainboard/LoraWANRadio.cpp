#include "LoraWANRadio.h"
#include <lmic.h>
#include <hal/hal.h>
#include "Logging.h"
#include "AmbaSat1Config.h"
#include "Hardware.h"

//
// Satellite Physical Setup
//
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {2, A2, LMIC_UNUSED_PIN},
};


LoraWANRadio::LoraWANRadio(PersistedConfiguration& config) 
{
    this->_config = config;
    this->_isWaitingForTransmissionToFinish = false;
}

void LoraWANRadio::setup(IncomingTransmissionDelegate incomingTransmissionDelegate)
{
    // Save delegate that will be called when a new transmission is received
    this->_incomingTransmissionDelegate = incomingTransmissionDelegate;

    //
    // Set up LoRaWAN radio
    //
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    // LMIC_setDrTxpow(DR_SF9, KEEP_TXPOW);

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

    // network ID 0x01 = Expiremental
    // network ID 0x13 = The Things Network
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
    PRINTLN_INFO(F("Using eu868 frequency plan"));
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    PRINTLN_INFO(F("Using us915 frequency plan"));
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    //
    // set the LMIC uplink frame count
    //
    LMIC.seqnoUp = this->_config.getUplinkFrameCount();

    this->_isWaitingForTransmissionToFinish = false;
}

void LoraWANRadio::transmit(uint8_t port, const uint8_t* dataToSend, uint8_t dataSize) 
{
    // wait for any in process transmission to end. This really shouldn't happen, but can
    // if the prior transmission received an downlink requiring an ACK.
    if ((LMIC.opmode&0x00FF) != OP_NONE) {
        PRINTLN_INFO(F("  Waiting on prior transmission ..."));
        while((LMIC.opmode&0x00FF) != OP_NONE) {
            os_runloop_once();
        }
        PRINTLN_INFO(F("   Complete"));
        // delay a little for radio to reset
        delay(2000);
    }

    // LMIC seems to crash here if we previously just received a downlink AND
    // there are any pending data in the Serial queue. So flush the Serial queue.
    Serial.flush();
    this->_isWaitingForTransmissionToFinish = true;
    LMIC_setTxData2(
        port,
        (xref2u1_t)dataToSend,
        dataSize,
        0
    );

    // Again, LMIC seems to flake out after recived downlinks without this delay.
    // Seems to be some interaction with the serial prints that follow. Since
    // LMIC is a black box and 50 milliseconds isn't all that much, so just wait.
    delay(50);

    // Must wait for the TX operations of LMIC.opmode to go to zero in order to handle
    // any downlink ACK that was requested.
     while((this->_isWaitingForTransmissionToFinish) || (LMIC.opmode&0x00FF) != OP_NONE) {
        os_runloop_once();
    }

    this->_isWaitingForTransmissionToFinish = false;
}

void LoraWANRadio::onTransmissionComplete()
{
    this->_isWaitingForTransmissionToFinish = false;
}

void LoraWANRadio::onIncomingTransmision(uint8_t port, const uint8_t* incomingData, uint8_t dataSize)
{
    PRINTLN_INFO(F("Incoming transmission"));

    // If delegate is present then call it
    if (this->_incomingTransmissionDelegate != nullptr)
    {
        (*(this->_incomingTransmissionDelegate))(port, incomingData, dataSize);
    }
}

void LoraWANRadio::onLinkDead()
{
    this->_isWaitingForTransmissionToFinish = false;
}

void LoraWANRadio::onLinkReset()
{
    this->_isWaitingForTransmissionToFinish = false;
}

// =========================================================================================================================================
// onEvent
// =========================================================================================================================================
void onEvent(ev_t ev)
{
    PRINTLN_INFO(F("OnEvent"));

    if (ev == EV_TXCOMPLETE) {
        // Notify radio class that sending transmission is complete
        Hardware::g_hardware->getLoraWANRadio().onTransmissionComplete();

        // Check for incoming transmission
        if (LMIC.dataLen > 0)
        {
            // Notify radio class that a new transmission is received
            Hardware::g_hardware->getLoraWANRadio().onIncomingTransmision(
                LMIC.frame[LMIC.dataBeg-1],
                &LMIC.frame[LMIC.dataBeg],
                LMIC.dataLen
            );

    #ifdef ENABLE_AMBASAT_COMMANDS
            AmbaSat1App::gApp->queueCommand(
                LMIC.frame[LMIC.dataBeg-1],
                &LMIC.frame[LMIC.dataBeg],
                LMIC.dataLen
            );
    #else
            PRINTLN_DEBUG(F("WARNING received a downlink but code is not enabled to process it."));
    #endif
        }

        PRINTLN_INFO(F("EV_TXCOMPLETE (includes RX windows)"));
    } else if (ev == EV_LINK_DEAD) {
        // Notify radio class that link is dead
        Hardware::g_hardware->getLoraWANRadio().onLinkDead();
    } else if (ev == EV_RESET) {
        // Notify radio class that link is reset
        Hardware::g_hardware->getLoraWANRadio().onLinkReset();
    }

    Serial.flush();
}
