#ifndef __VOLTAGE_READER_H__
#define __VOLTAGE_READER_H__

#include "Arduino.h"

class VoltageReader 
{
    private:
        const long _vrefSettleDelayInMilliseconds;
        const int _numberOfAttemptsToCheckForADSCBit;

    public:
        VoltageReader(const long vrefSettleDelayInMilliseconds, const int numberOfAttemptsToCheckForADSCBit);
        
        int16_t readVoltage() const;
};

#endif // __VOLTAGE_READER_H__