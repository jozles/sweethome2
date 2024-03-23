#ifndef _UTILETHER_H_
#define _UTILETHER_H_

#include <Ethernet.h>
#include "SdFat.h"

int  ethWrite(EthernetClient* cli,char* buf);
int  ethWrite(EthernetClient* cli,char* buf,long len);
int  ethWrite(EthernetClient* cli,char* buf,uint16_t* lb);
int  ethWrite(EthernetClient* cli,char* buf,uint16_t* lb,long len);
void mail(const char* a, const char* mm);
void mailInit(char* login,char* pass);
//int  sdOpen(const char* fname,File32* file32);
int  sdOpen(const char* fname,SdFile* file32);
//int  sdOpen(const char* fname,File32* file32,const char* txt);
int  sdOpen(const char* fname,SdFile* file32,const char* txt);
//void sdRemove(const char* fname,File32* file32);
void sdRemove(const char* fname,SdFile* file);
void sdInit();
void sdExfatFormat();

void histoStore_textdh(const char* val1,const char* val2,const char* val3);  // getdate()
void histoStore_textdh0(const char* val1,const char* val2,const char* val3);
unsigned long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde);
void convertNTP(unsigned long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second);
void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss);
void unixDateToStr(unsigned long uxd,char* date);
unsigned long alphaDateToUnix(const char* tim,bool onlyHours);
unsigned long alphaDateToUnix(const char* tim,bool onlyHours,bool print);
void dateToStr(char* buff,int year,int month,int day,int hour,int minute,int second);
void addTime(char* recep,const char* tim1,const char* tim2,bool onlyHours);
void subTime(char* recep,const char* endtime,const char* time,bool onlyHours);

void initDate();
char* alphaDate();
//void initMess();
bool ctlpass(char* data,char* model);
int  searchusr(char* usrname);

#ifdef UDPUSAGE
int  getUDPdate(uint32_t* hms,uint32_t* amj,byte* js);
void sendUdpData(uint8_t udpChannel,IPAddress host,uint16_t hostPort,char* data);
#endif // UDPUSAGE

//#define SOCK_DEBUG          // WARNING : le traitement de fermeture des sockets ouverts est dans showSocketStatus()

void printSocketStatus(bool nolf);
void printSocketStatus(bool nolf,const char* mess);
void showSocketsStatus(bool close);
void showSocketsStatus(bool close,bool nolf);
void showSocketsStatus(bool close,bool nolf,bool print);
void showSocketsStatus(bool close,bool nolf,bool print,const char* mess);

#endif // _UTILETHER_H_
