#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#define CONCRECADDR 0
#define CONCRECLEN 200

uint16_t getServerConfig();
void configInit();
bool configLoad();
void configSave();
void configPrint();


#endif // CONFIG INCLUDED