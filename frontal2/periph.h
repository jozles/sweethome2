#ifndef _PERIPH_H_
#define _PERIPH_H_

#define PERISAVESD    VRAI                     // copie du cache sur le disque lors de periSave
#define PERISAVELOCAL FAUX                     // pas de copie du cache sur le disque lors de periSave
                                               // periCacheStatus[num] vaut CACHEISFILE si fichier == cache ;

#define NBRULOP 7
#define LENRULOP 5

void  sdCardGen();

int   periLoad(uint16_t num);
int   periSave(uint16_t num,bool sd);
int   periCacheSave(uint16_t num);
int   periCacheLoad(uint16_t num);
//int   peri (uint16_t num);
int   periRaz(uint16_t num);
void  periModif();
void  periConvert();
void  periInit();
void  periInitVar();  // le contenu de periRec seul
void  periInitVar0(); // pulses et inputs 
byte  periSwLev(uint8_t sw);                    // Lev Sw bit value
byte  periSwCde(uint8_t sw);                    // Cd  Sw bit value
void  periSwCdUpdate(uint8_t sw,uint8_t stat);  // update Cd Sw bit
void  periSwLevUpdate(uint8_t sw,uint8_t stat); // update Lev Sw bit
void  periSwSync();                             // synchro disjoncteurs switchs sur remotes au démarrage
void  periCheck(uint16_t num,char* text);
void  periPrint(uint16_t num);
void  periTableLoad();
void  periTableSave();
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
void  remMemDetUpdate(uint8_t rem,uint8_t endet);
void  remoteNPlus(int plus);

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
int   memosFind();


#endif // _PERIPH_
