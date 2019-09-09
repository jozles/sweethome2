#ifndef _UTILETHER_H_
#define _UTILETHER_H_

#include <SD.h>
#include <Ethernet.h>

int  sdOpen(char mode,File* fileS,char* fname);
void sdInit();

int  htmlPrint(EthernetClient* cli,File* fhtml,char* fname);
int  htmlSave(File* fhtml,char* fname,char* buff);

void sdstore_textdh(File* fhisto,char* val1,char* val2,char* val3);  // getdate()
void sdstore_textdh0(File* fhisto,char* val1,char* val2,char* val3);
unsigned long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde);
void convertNTP(unsigned long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second);
void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss);

void initDate();

#ifdef UDPUSAGE
int  getUDPdate(uint32_t* hms,uint32_t* amj,byte* js);
#endif UDPUSAGE

#endif // _UTILETHER_H_
