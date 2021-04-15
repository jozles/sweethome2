
#include <EEPROM.h>
#include "eepr.h"
#include <shutil2.h>




Eepr::Eepr()
{
}

uint32_t Eepr::calcCrc32(byte* data,uint16_t len)
{
  uint32_t crc32=0;
  uint32_t reg;
  uint16_t i;
  uint8_t j,k;

  for(i=0;i<len;i++){
    //memcpy(&reg,data+i,4);
    reg=(uint32_t)data[i];

    for(j=0;j<8;j++){
        k=(crc32^reg)&0x00000001;
        crc32=crc32>>1;
        if(k==1){crc32=crc32^0x82608EDB;} //0x04C11DB7 rigth rotated polynom
        reg=reg>>1;
    }
  }
  return crc32;
}

bool Eepr::checkCrc32(byte* data,uint16_t len,uint32_t crc32)
{
    return calcCrc32(data,len)==crc32;
}

void Eepr::eeread(byte* data,uint16_t length,uint16_t addr)
{
  //Serial.print("eeread ");Serial.print(length);Serial.print(" at ");Serial.println(addr);
    length&=0x00ff; // maxi 256
    for(uint8_t i=0;i<length;i++){
        data[i] = EEPROM.read(addr+i);}
  //dumpstr((char*)data,length);
}

void Eepr::eewrite(byte* data,uint16_t length,uint16_t addr)
{
    length&=0x00ff; // maxi 256
    for(uint8_t i=0;i<length;i++){
        EEPROM.update(addr+i,data[i]);}
}

void Eepr::store(byte* data,uint16_t length)
{
    memcpy(data+EEPRCRCLENGTH,&length,EEPRLENGTHLENGTH);
    uint32_t crc=calcCrc32(data+EEPRCRCLENGTH,length-EEPRCRCLENGTH);
    memcpy(data,&crc,4);

    eewrite(data,length,0);
}

bool Eepr::load(byte* data,uint16_t length)
{
  byte header[EEPRHEADERLENGTH];
    if(length<EEPRHEADERLENGTH) {return false;}
    eeread(header,EEPRHEADERLENGTH,0);                              // lit header
    uint16_t usefullLength;
    memcpy(&usefullLength,header+EEPRCRCLENGTH,EEPRLENGTHLENGTH);   // recup longueur-crc
    usefullLength &= 0x00ff; // 256 bytes max
//    Serial.print("usefullLength=");Serial.print(usefullLength);Serial.print(" data length=");Serial.println(length);
    if(length<usefullLength) {return false;}

    eeread(data,usefullLength,0);                                   // charge tout
    uint32_t crc32;
    memcpy(&crc32,data,EEPRCRCLENGTH);
    return checkCrc32(data+EEPRCRCLENGTH,usefullLength-EEPRCRCLENGTH,crc32);
}
