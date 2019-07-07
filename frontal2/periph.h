#ifndef _PERIPH_H_
#define _PERIPH_H_

/*
typedef struct Ymdhms Ymdhms;

struct Ymdhms
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t dow;
};

Ymdhms now();


#define LNOW 16             // len chargée par alphanow 
void alphaNow(char* buff);  // charge LNOW (16) car YYYYMMDDHHMMSSd\0
void setDS3231time(byte second, byte minute, byte hour,byte dayOfWeek,byte dayOfMonth, byte month, byte year);
void readDS3231time(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,byte *month,byte *year);
void readDS3231temp(float* th);
//*/



void ledblink(uint8_t nbre);


#define PERISAVESD    VRAI                     // copie du cache sur le disque lors de periSave
#define PERISAVELOCAL FAUX                     // pas de copie du cache sur le disque lors de periSave

int   periLoad(uint16_t num);
int   periSave(uint16_t num,bool sd);
int   periRemove(uint16_t num);
void  periModif();
void  periConvert();
void  periInit();
void  periInitVar();  // le contenu de periRec seul
void  periInitVar0(); // pulses et inputs 
void  periCheck(uint16_t num,char* text);
void  periPrint(uint16_t num);

void  configInit();
int   configLoad();
int   configSave();
void  configInit();
void  configInitVar(); 
void  configPrint();

void remotePrint();
void remInit();
int  remSave(char* remF,uint16_t remL,char* remA);
void remoteLoad();
void remoteSave();

void timersPrint();
void timersInit();
int  timersLoad();
int  timersSave();

void memDetPrint();
void memDetInit();
int  memDetLoad();
int  memDetSave();

#endif // _PERIPH_
