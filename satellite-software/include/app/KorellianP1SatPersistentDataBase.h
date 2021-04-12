#ifndef __KORELLIAN_P1_SAT_PERSISTENTDATABASE_H__
#define __KORELLIAN_P1_SAT_PERSISTENTDATABASE_H__

#include <stdint.h>

class KorellianP1SatPersistentDataBase
{
    public:
        uint16_t readRebootCount() const;
        uint16_t writeRebootCount(uint16_t rebootCount) const;

};

#endif // __KORELLIAN_P1_SAT_PERSISTENTDATABASE_H__
