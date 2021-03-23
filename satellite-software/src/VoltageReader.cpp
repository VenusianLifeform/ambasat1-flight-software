#include "VoltageReader.h"
#include "Logging.h"

#define MAX_INT16_VALUE (32767)


VoltageReader::VoltageReader(const long vrefSettleDelayInMilliseconds, const int numberOfAttemptsToCheckForADSCBit) :
    _vrefSettleDelayInMilliseconds(vrefSettleDelayInMilliseconds),
    _numberOfAttemptsToCheckForADSCBit(numberOfAttemptsToCheckForADSCBit)
{
    if (vrefSettleDelayInMilliseconds < 0)
    {
        PRINTLN_ERROR("VoltageReader: vrefSettleDelayInMilliseconds must be positive");
    }

    if (numberOfAttemptsToCheckForADSCBit < 1)
    {
        PRINTLN_ERROR("VoltageReader: numberOfAttemptsToCheckForADSCBit must be positive");
    }
}

int16_t VoltageReader::readVoltage() const
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    // Wait for Vref to settle
    delay(this->_vrefSettleDelayInMilliseconds);

    // Start conversion
    ADCSRA |= _BV(ADSC);

    // measuring    
    int attempts = 0;
    while (bit_is_set(ADCSRA, ADSC)) 
    {
        attempts++;
        if (attempts >= this->_numberOfAttemptsToCheckForADSCBit) 
        {
            PRINTLN_ERROR("VoltageReader: reached maximum number of ADSC bit checks");
            return 0;
        }
    };

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    int32_t result = (high<<8) | low;

    result = 1125300L / result;

    // Check for overflow
    if (result >= MAX_INT16_VALUE) 
    {
            PRINTLN_ERROR("VoltageReader: voltage is above maximum value for int16");
            return MAX_INT16_VALUE;
    }

    // Vcc in millivolts
    return result;
}