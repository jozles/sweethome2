#ifndef _UTILETHER_H_
#define _UTILETHER_H_

#include <Ethernet.h>
#include "SdFat.h"

int  ethWrite(EthernetClient* cli,char* buf);
void mail(char* a,char* mm);
int  sdOpen(char* fname,File32* file32);
void sdRemove(char* fname,File32* file32);
void sdInit();

void histoStore_textdh(char* val1,char* val2,char* val3);  // getdate()
void histoStore_textdh0(char* val1,char* val2,char* val3);
unsigned long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde);
void convertNTP(unsigned long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second);
void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss);

void initDate();
char* alphaDate();
//void initMess();

#ifdef UDPUSAGE
int  getUDPdate(uint32_t* hms,uint32_t* amj,byte* js);
#endif UDPUSAGE

#endif // _UTILETHER_H_
