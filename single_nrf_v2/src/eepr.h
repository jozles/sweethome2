#ifndef _EEPR_H_
#define _EEPR_H_

#include "Arduino.h"

#define EEPRCRCLENGTH 4
#define EEPRCRC    0
#define EEPRLENGTHLENGTH 2
#define EEPRLENGTH EEPRCRC+EEPRCRCLENGTH
#define EEPRVERSLENGTH 2
#define EEPRVERS   EEPRLENGTH+EEPRLENGTHLENGTH
#define EEPRHEADERLENGTH EEPRCRCLENGTH+EEPRLENGTHLENGTH+EEPRVERSLENGTH

class Eepr
{
    public:
        Eepr();
        uint32_t calcCrc32(byte* data,uint16_t len);
        bool checkCrc32(byte* data,uint16_t len,uint32_t crc32);
        void eeread(byte* data,uint16_t len,uint16_t addr);
        void eewrite(byte* data,uint16_t len,uint16_t addr);
        void store(byte* data,uint16_t length); // longueur totale avec header
        bool load(byte*data,uint16_t length);   // longueur totale avec header
};

#endif // __EEPR_H
