#ifndef _PERIPH_H_
#define _PERIPH_H_

#define PERISAVESD    VRAI                     // copie du cache sur le disque lors de periSave
#define PERISAVELOCAL FAUX                     // pas de copie du cache sur le disque lors de periSave

void  sdCardGen();

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
void  periTableLoad();
void  periMaintenance();

void  configInit();
int   configLoad();
int   configSave();
void  configInitVar(); 
void  configPrint();

void  remotePrint();
void  remInit();
int   remSave(char* remF,uint16_t remL,char* remA);
void  remoteLoad();
void  remoteSave();

void  timersPrint();
void  timersInit();
int   timersLoad();
int   timersSave();

void  thermosPrint();
void  thermosInit();
int   thermosLoad();
int   thermosSave();

void  memDetPrint();
void  memDetInit();
int   memDetLoad();
int   memDetSave();

int   memosLoad(int m);
int   memosSave(int m);
void  memosPrint();
void  memosInit();


#endif // _PERIPH_
