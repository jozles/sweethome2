#ifndef MESSAGES_H_INCLUDED
#define MESSAGES_H_INCLUDED

#include "Arduino.h"
#include "shconst2.h"

//#define PERIF

//void assySet(char* message,int pericur,char* diag,char* date14);
int buildMess(char* fonction,char* data,char* sep);
int checkData(char* data);
int checkHttpData(char* data,uint8_t* fonction);
char* periDiag(int8_t diag);

#ifndef PERIF
void purgeServer(EthernetClient* cli);
int waitRefCli(EthernetClient* cli,char* ref,int lref,char* buf,int lbuf);           // attente d'un chaine spécifique dans le flot
int messToServer(EthernetClient* cli,const char* host,int port,char* data);    // connecte au serveur et transfère la data
int getHttpResponse(EthernetClient* cli,char* data,int lmax,uint8_t* fonction);
#endif // PERIF

#ifdef PERIF
void purgeServer(WiFiClient* cli);
int waitRefCli(WiFiClient* cli,char* ref,int lref,char* buf,int lbuf);            // attente d'un chaine spécifique dans le flot
int messToServer(WiFiClient* cli,const char* host,int port,char* data);    // connecte au serveur et transfère la data
int getHttpResponse(WiFiClient* cli,char* data,int lmax,uint8_t* fonction);
#endif // PERIF

#endif // MESSAGES_H_INCLUDED
