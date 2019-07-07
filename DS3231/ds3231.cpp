#include <Arduino.h>
#include <Wire.h>

#include "ds3231.h"



Ds3231::Ds3231()  // constructeur
{
}

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void Ds3231::setTime(byte second, byte minute, byte hour,
    byte dayOfWeek,byte dayOfMonth, byte month, byte year)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}


void Ds3231::readTime(byte *second,byte *minute,byte *hour,
    byte *dow,byte *day,byte *month,byte *year)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddr, 7);

  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dow = bcdToDec(Wire.read());
  *day = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void Ds3231::readTemp(float* th)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(17); // set DS3231 register pointer to 11h
  Wire.endTransmission();
  Wire.requestFrom(i2cAddr, 2);
  // request two bytes of data from DS3231 starting from register 11h

  byte msb = Wire.read();
  byte lsb = Wire.read();
  //Serial.print(*msb);Serial.print(":");Serial.println(*lsb);
  switch(lsb)
  {
    case 0:break;
    case 64: lsb=25;break;
    case 128:lsb=50;break;
    case 192:lsb=75;break;
    default: msb=99,lsb=99;break; // error
  }
  *th=(float)msb+(float)lsb/100;
}

void Ds3231::getDate(uint32_t* hms2,uint32_t* amj2,byte* js,char* strdate)
{
  char* days={"SunMonTueWedThuFriSat"};
  char* months={"JanFebMarAprMayJunJulAugSepOctNovDec"};
  int i=0;
  byte seconde,minute,heure,jour,mois,annee,msb,lsb; // numérique DS3231   // ,joursemaine
  char buf[8];for(byte i=0;i<8;i++){buf[i]=0;}
  readTime(&seconde,&minute,&heure,js,&jour,&mois,&annee);
  *hms2=(long)(heure)*10000+(long)minute*100+(long)seconde;*amj2=(long)(annee+2000)*10000+(long)mois*100+(long)jour;
  memset(strdate,0x00,sizeof(strdate));
  strncpy(strdate,days+(*(js)-1)*3,3);strcat(strdate,", ");sprintf(buf,"%u",(byte)jour);strcat(strdate,buf);
  strcat(strdate," ");strncat(strdate,months+(mois-1)*3,3);strcat(strdate," ");sprintf(buf,"%u",annee+2000);strcat(strdate,buf);
  strcat(strdate," ");sprintf(buf,"%.2u",heure);strcat(strdate,buf);strcat(strdate,":");sprintf(buf,"%.2u",minute);
  strcat(strdate,buf);strcat(strdate,":");sprintf(buf,"%02u",seconde);strcat(strdate,buf);strcat(strdate," GMT");
}

void Ds3231::alphaNow(char* buff)
{

  byte second,minute,hour,day,month,year,dow;
  readTime(&second,&minute,&hour,&dow,&day,&month,&year);

  sprintf(buff,"%.8lu",(long)(year+2000)*10000+(long)month*100+(long)day);
  sprintf((buff+8),"%.2u",hour);sprintf((buff+10),"%.2u",minute);sprintf((buff+12),"%.2u",second);
  buff[14]=dow;
  buff[15]='\0';
  //Serial.print("alphaNow ");Serial.print(buff);Serial.print(" ");Serial.print(ndt.hour);Serial.print(" ");Serial.print(ndt.minute);Serial.print(" ");Serial.println(ndt.second);
  //Serial.println();
}

