#ifndef DS18X20_H_INCLUDED
#define DS18X20_H_INCLUDED


/*
    Interrupt-free, small memory wasting ds1820 interface (less than 1K)
    this is working with any pin who allow digitalRead and digitalWrite.
    Interrupt suspend should not be longer than 85 uSec at each written bit and
    less than 10uSec for the other occurencys.

    2 steps : first converting (could be as long as 750 mSec) then get value.

    convertDs(uint8_t pin) returns 1 or error codes (see below)
    readDs(uint8_t pin) returns float value between -55 and +125 or error codes (see below)

    getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein)
    return a status (codes below) and fill framein with data provided by Ds in accordance
    with the frameout data transmitted to Ds.
    (see datasheet)

    pin is the Arduino pin number used to connect Ds unit
*/


#include "Arduino.h"

#define MODEL_S 0x10
#define MODEL_B 0x28



class Ds1820
{
    public:
        Ds1820();
        int getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein);
        byte calcBitCrc(byte shiftReg, byte data_bit);
        float readDs(uint8_t pin);
        int romDs(uint8_t pin,uint8_t* framein);
        int setDs(uint8_t pin,uint8_t* frameout,uint8_t* framein);
        int convertDs(uint8_t pin);

        byte dsmodel=0x00;
};

#endif // DS18X20_H_INCLUDED


