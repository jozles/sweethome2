#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "nrf24l01s_const.h"

#ifdef DUE

#define CONCRECLEN 200
#define CONCRECADDR 0

uint16_t getServerConfig();
void configInit();
bool configLoad();
void configSave();
void configPrint();

#endif // DUE

#endif // CONFIG INCLUDED