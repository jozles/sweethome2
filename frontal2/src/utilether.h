#ifndef _UTILETHER_H_
#define _UTILETHER_H_

#include <Ethernet.h>
#include "SdFat.h"

int  ethWrite(EthernetClient* cli,char* buf);
int  ethWrite(EthernetClient* cli,char* buf,uint16_t* lb);
void mail(const char* a, const char* mm);
int  sdOpen(const char* fname,File32* file32);
int  sdOpen(const char* fname,File32* file32,const char* txt);
void sdRemove(const char* fname,File32* file32);
void sdInit();

void histoStore_textdh(const char* val1,const char* val2,const char* val3);  // getdate()
void histoStore_textdh0(const char* val1,const char* val2,const char* val3);
unsigned long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde);
void convertNTP(unsigned long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second);
void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss);

void initDate();
char* alphaDate();
//void initMess();

#ifdef UDPUSAGE
int  getUDPdate(uint32_t* hms,uint32_t* amj,byte* js);
#endif // UDPUSAGE

#endif // _UTILETHER_H_
