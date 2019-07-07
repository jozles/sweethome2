#ifndef _DS3231_H_INCLUDED
#define _DS3231_H_INCLUDED

#define LNOW 16             // len chargée par alphanow

class Ds3231
{
    public:
        Ds3231();
        void setTime(byte second, byte minute, byte hour,byte dayOfWeek,byte dayOfMonth, byte month, byte year);
        void readTime(byte *second,byte *minute,byte *hour,byte *dow,byte *day,byte *month,byte *year);
        void readTemp(float* th);
        void getDate(uint32_t* hms2,uint32_t* amj2,byte* js,char* strdate);
        void alphaNow(char* buff);  // charge LNOW (16) car YYYYMMDDHHMMSSd\0
        int  i2cAddr;

};




#endif // _DS3231_H_INCLUDED
