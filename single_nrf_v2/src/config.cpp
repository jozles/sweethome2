
#include <Arduino.h>
#include <shconst2.h>
#include <shmess2.h>
#include <shutil2.h>
#include <eepr.h>
#include "config.h"
#include "nrf24l01s_const.h"
#include "nrf24l01s.h"

extern Eepr eeprom;

#ifdef DUE
#define SERIALX Serial1
#define SERNB 1
#endif
#ifndef DUE
#define SERIALX Serial
#define SERNB 0
#endif


byte  configRec[CONFIGRECLEN];

bool configLoad()                    // INUTILISE 
{
    if(!eeprom.load((byte*)configRec,(uint16_t)CONFIGRECLEN)){Serial.println("***EEPROM KO***");ledblink(BCODESDCARDKO);} // ledblink bloque
    Serial.println("eeprom ok");
    return 1;    
}

void configSave()
{
    eeprom.store((byte*)configRec,CONFIGRECLEN);
}

bool syncServerConfig(char* bf,char* syncMess,uint16_t* rcvl)
{
  Serial.println("\nsync1");delay(10);
  
  serPurge(SERNB);
  
  Serial.println("\nsync2");delay(10);

  for(uint8_t i=0;i<=TSCNB;i++){SERIALX.print(RCVSYNCHAR);}
  SERIALX.print(syncMess);
  if(*bf!=0x00){SERIALX.print(bf);memset(bf,0x00,MAXSER);}

  Serial.println("\nsync3");delay(10);
  
  while(*rcvl==0){*rcvl=serialRcv(bf,MAXSER,SERNB);blink(1);}  // attente réponse sans time out

  if(*rcvl>5){
    Serial.print(*rcvl);Serial.print(" ");Serial.println(bf);
    Serial.print("checkData=");
    uint16_t ll=0;
    int cd=checkData(bf,&ll);               // longueur stockée dans le message
  
    if(cd!=1){Serial.println(" ko");return 0;}      // renvoie mess = MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3)  
    else {Serial.println(" ok");return 1;}
  }
  return 0;
}

#if NRF_MODE == 'C'

/* >>>> config concentrateur <<<<<< */

/* pointeurs dans l'enregitrement de config */

  uint16_t* cfgLen;           // cfg record length

  byte*     serverIp;         // server ip addr
  uint16_t* serverTcpPort;    // server tcp port
  uint16_t* serverUdpPort;    // server udp port

  char*     peripass;         // mot de passe périphériques

  uint8_t*  concMac;          // macaddr concentrateur (5 premiers caractères valides le 6ème est le numéro dans la table)
  byte*     concIp;           // adresse IP concentrateur
  uint16_t* concPort;         // port concentrateur
  uint8_t*  concRx;           // radio addr nrf
  uint16_t* concChannel;      // n° channel nrf utilisé par le concentrateur
  uint16_t* concRfSpeed;      // RF_Speed concentrateur
  uint8_t*  concNb;           // numéro concentrateur

  uint32_t* cfgCrc;

  byte* configBegOfRecord;
  byte* configEndOfRecord;

  uint16_t hostPort=0;             // server port (udp/tcp selon TXRX_MODE) 

void configInitVar()
{
  *concNb=0;
  memset(serverIp,0x00,4);
  *serverTcpPort=0;
  *serverUdpPort=0;
  memset(concMac,0x00,MACADDRLENGTH);
  *concChannel=0;
  *concRfSpeed=0;
  memset(concIp,0x00,4);
  *concPort=0;
  memset(peripass,0x00,LPWD+1);
}

void configInit()
{
byte* temp=(byte*)configRec;

  configBegOfRecord=(byte*)temp;         // doit être le premier !!!
 
  cfgLen=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  temp+=2;                      // adresse multiple de 4 pour début lecture/écriture

  temp+=4;
  serverIp=(byte*)temp;
  temp+=4;
  serverTcpPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  serverUdpPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);

  peripass=(char*)temp;
  temp+=(LPWD+1);

  concMac=(uint8_t*)temp;
  temp+=MACADDRLENGTH;
  concIp=(byte*)temp;
  temp+=4;
  concPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  concRx=(uint8_t*)temp;
  temp+=RADIO_ADDR_LENGTH;
  concChannel=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concRfSpeed=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concNb=(uint8_t*)temp;
  temp+=sizeof(uint8_t);

  temp+=73;                          // dispo 
  cfgCrc=(uint32_t*)temp;
  temp+=sizeof(uint32_t);

  configEndOfRecord=(byte*)temp;     // doit être le dernier !!!

  configInitVar();
}

void configPrint()
{
  Serial.print("concNb=");Serial.println(*concNb);
  Serial.print(" serverIP=");serialPrintIp(serverIp);Serial.print("/");Serial.print(*serverTcpPort);Serial.print("/");Serial.println(*serverUdpPort);
  Serial.print(" concIP =");serialPrintIp(concIp);Serial.print("/");Serial.println(*concPort);
  Serial.print(" concMac=");serialPrintMac(concMac,1);
  Serial.print(" RX Addr=");for(uint8_t i=0;i<RADIO_ADDR_LENGTH;i++){Serial.print((char)*(concRx+i));}
  Serial.print(" channel=");Serial.print(*concChannel);Serial.print(" speed=");Serial.println(*concRfSpeed);
  Serial.print("peripass=");Serial.println(peripass);
}

bool nextpv(char* b,char** newb,int maxl)
{
  while(*b++!=';' && 0<maxl--){}
  if(maxl<=0){return 0;}
  
  *newb=b;return 1;
}

uint16_t getServerConfig()
{
    char bf[MAXSER];memset(bf,0x00,MAXSER);
    uint16_t rcvl=0;

    if(!syncServerConfig(bf,(char*)CONCCFG,&rcvl)){return 0;};

    char a=' ';
    char* b=bf;
    char* newb;
    uint8_t cntpv=0;
    uint16_t temp=0;

    while(cntpv<2 && a!='\0' && b<(bf+rcvl)){a=*b++;if(a==';'){cntpv++;}}                // skip len+name+version
    
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;serverIp[i]=temp;}           // serverIp  
    temp=0;conv_atob(b,&temp);b+=6;*serverTcpPort=temp;                                  // server tcp Port
    if(!nextpv(b,&newb,6)){return 0;}b=newb;
    temp=0;conv_atob(b,&temp);b+=6;*serverUdpPort=temp;                                  // server udp Port 
    temp=0;a=' ';while((a=*b++)!=';' && a!='\0' && b<(bf+rcvl) && temp<LPWD){peripass[temp]=a;temp++;}  // peripass
  
    packMac((byte*)concMac,b);b+=(MACADDRLENGTH*3-1+1);                          // concMac               
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;concIp[i]=temp;}     // concIp  
    temp=0;conv_atob(b,&temp);b+=6;*concPort=temp;                               // concPort
    memcpy(concRx,b,RADIO_ADDR_LENGTH);b+=RADIO_ADDR_LENGTH+1;                   // radio addr
    temp=0;conv_atob(b,&temp);b+=4;*concChannel=temp;                            // Channel
    temp=0;conv_atob(b,&temp);b+=2;*concRfSpeed=temp;                            // Speed
    *concNb=(uint8_t)(*b-PMFNCVAL);                                              
  }
  return rcvl;
}
#endif // NRF_MODE == 'C'

#if NRF_MODE == 'P'

byte*     configVers;
float*    thFactor;
float*    thOffset;
float*    vFactor;
float*    vOffset;
byte*     macAddr;
byte*     concAddr;
uint8_t*  concNb;
uint8_t*  concChannel;
uint8_t*  concSpeed;
uint8_t*  concPeriParams;   // provenance des params de calibrage (0 périf ; 1 saisie serveur)

void configInit()
{
  byte* temp=(byte*)configRec;

  byte* configBegOfRecord=(byte*)temp;         // doit être le premier !!!

  configVers=temp+EEPRVERS;
  temp += EEPRHEADERLENGTH;
  thFactor=(float*)temp;                           
  temp +=sizeof(float);
  thOffset=(float*)temp;
  temp +=sizeof(float);
  vFactor=(float*)temp;                           
  temp +=sizeof(float);
  vOffset=(float*)temp;
  temp +=sizeof(float);
  macAddr=(byte*)temp;
  temp +=6;
  concAddr=(byte*)temp;
  temp +=6;
  concNb=(uint8_t*)temp;
  temp +=sizeof(uint8_t);
  concChannel=(uint8_t*)temp;
  temp+=sizeof(uint8_t);
  concSpeed=(uint8_t*)temp;
  temp+=sizeof(uint8_t);
  concPeriParams=(uint8_t*)temp;
  temp+=sizeof(uint8_t);

  temp+=31;                   // dispo

  byte* configEndOfRecord=(byte*)temp;      // doit être le dernier !!!

  long configLength=(long)configEndOfRecord-(long)configBegOfRecord+1;  
  Serial.print("CONFIGRECLEN=");Serial.print(CONFIGRECLEN);Serial.print("/");Serial.print(configLength);
  delay(10);if(configLength>CONFIGRECLEN) {ledblink(BCODECONFIGRECLEN);}
/*
  memcpy(configVers,VERSION,2);
  memcpy(macAddr,DEF_ADDR,6);
  strncpy((char*)concAddr,CONC,5);
  *thFactor=0.1071;
  *thOffset=50;
  *vFactor=0.0057;
  *vOffset=0;
*/
}

void configPrint()
{
    uint16_t configLen;memcpy(&configLen,configRec+EEPRLENGTH,2);
    char configVers[3];memcpy(configVers,configRec+EEPRVERS,2);configVers[3]='\0';
    Serial.print("crc     ");dumpfield((char*)configRec,4);Serial.print(" len ");Serial.print(configLen);Serial.print(" V ");Serial.print(configVers[0]);Serial.println(configVers[1]);
    char buf[7];memcpy(buf,concAddr,5);buf[5]='\0';
    Serial.print("MAC  ");dumpstr((char*)macAddr,6);Serial.print("CONC ");dumpstr((char*)concAddr,6);
    if(memcmp(configVers,"01",2)!=0){
      Serial.print("concNb ");Serial.print(*concNb);
      Serial.print("  channel ");Serial.print(*concChannel);
      Serial.print("  speed ");Serial.print(*concSpeed);
      Serial.print("  source(0 peri ; 1 server) ");Serial.println(*concPeriParams);
    }
    Serial.print("thFactor=");Serial.print(*thFactor*10000);Serial.print("  thOffset=");Serial.print(*thOffset);   
    Serial.print("   vFactor=");Serial.print(*vFactor*10000);Serial.print("   vOffset=");Serial.println(*vOffset);   
    delay(10);
}

uint16_t getServerConfig()
{
  #define MAXS 50
    uint16_t rcvl=0;
    char xf[MAXS];
    byte* bf=(byte*)xf;
    memset(bf,0x00,MAXS);
    strcat(xf,"0123;1234567890");                                    // len ascii

dumpstr((char*)bf,128);
Serial.println();
dumpstr((char*)xf,128);
Serial.println();
//dumpstr((char*)&bf[64],128);
//    bf+=64;
//Serial.println();    
//dumpstr((char*)bf,128);
//return 1;

    for(uint8_t i=5;i<15;i++){Serial.print(xf[i]);}
    Serial.println();
return 1;

    sprintf((char*)bf,"%04d",(int)(*vFactor*10000));bf+=5;        // vFactor
    *bf=';';bf++;
    sprintf((char*)bf,"%04d",(int)*vOffset);bf+=5;                // vOffset
    *bf=';';bf++;
    sprintf((char*)bf,"%04d",(int)(*thFactor*10000));bf+=5;       // thFactor
    *bf=';';bf++;
    sprintf((char*)bf,"%04d",(int)*thOffset);bf+=5;               // thOffset
    *bf=';';bf++;
    *bf=*concPeriParams+PMFNCVAL;bf+=1;                  // provenance periParams (0=périf 1=saisie server)
    *bf=';';bf++;
    
    dumpstr((char*)xf,MAXS);while(1){delay(1000);blink(1);}
    
    memcpy(bf,macAddr,RADIO_ADDR_LENGTH);                  // perif Rx addr
    bf+=RADIO_ADDR_LENGTH;
    *bf=';';bf++;
    *bf='\0';

dumpstr((char*)bf,MAXS);    
delay(1000);
    Serial.println("\nsync0");delay(10);while(1){delay(1000);blink(1);}

    setExpEnd((char*)xf);                                            // len + crc

    delay(1000);
    Serial.println("\nsync0");delay(10);while(1){delay(1000);blink(1);}
    if(!syncServerConfig(xf,(char*)PERICFG,&rcvl)){return 0;}

Serial.println();
configPrint();
dumpstr((char*)xf,200);

while(1){delay(1000);blink(1);}

    char a=' ';
    uint16_t temp=0;
    uint8_t cntpv=0;
    int tmp=0;
    bf=(byte*)xf;

    while(cntpv<2 && a!='\0' && bf<((byte*)xf+rcvl)){a=*bf++;if(a==';'){cntpv++;}}                // skip len+name+version
    
    memcpy(concAddr,bf,RADIO_ADDR_LENGTH);bf+=RADIO_ADDR_LENGTH+1;                  
    temp=0;conv_atob((char*)bf,&temp);bf+=4;*concChannel=temp;                           // Channel
    temp=0;conv_atob((char*)bf,&temp);bf+=2;*concSpeed=temp;                             // Speed
    *concNb=(uint8_t)(*bf-PMFNCVAL);bf+=2;                                             
    
    *vFactor=convStrToNum((char*)bf,&tmp);bf+=tmp+1;
    *vOffset=convStrToNum((char*)bf,&tmp);bf+=tmp+1;
    *thFactor=convStrToNum((char*)bf,&tmp);bf+=tmp+1;
    *thOffset=convStrToNum((char*)bf,&tmp);bf+=tmp+1;
    *concPeriParams=*bf-PMFNCVAL;bf+=2;
    memcpy(macAddr,bf,RADIO_ADDR_LENGTH);bf+=RADIO_ADDR_LENGTH+1;                                                            

  return rcvl;
}

#endif // NRF_MODE == 'P'