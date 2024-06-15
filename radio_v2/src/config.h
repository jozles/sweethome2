#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "radio_const.h"

#if NRF_MODE == 'C'
#define CONFIGRECLEN 200
#endif // NRF_MODE == 'C'

#if NRF_MODE == 'P'
#define CONFIGRECLEN 76                      // len maxi paramètres de config en Eeprom (37 v01 ; 38 v02 ; 75 v03)
#endif // NRF_MODE == 'C'

#define CONCRECADDR 0

uint16_t getServerConfig();
bool syncServerConfig(char* bf,char* syncMess,uint16_t* rcvl);
void configInit();
bool configLoad();
void configSave();
void configPrint();
void configCreate();

#endif // CONFIG INCLUDED