#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "radio_const.h"

#if MACHINE_CONCENTRATEUR
#define CONFIGRECLEN 200
#endif // MACHINE == 'C'

#if MACHINE_DET328
#define CONFIGRECLEN 76                      // len maxi paramètres de config en Eeprom (37 v01 ; 38 v02 ; 75 v03)
#endif // MACHINE == 'C'

#define CONCRECADDR 0

uint16_t getServerConfig();
bool syncServerConfig(char* bf,char* syncMess,uint16_t* rcvl);
void configInit();
bool configLoad();
void configSave();
void configPrint();
void configCreate();

#endif // CONFIG INCLUDED