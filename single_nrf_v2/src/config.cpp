
#include <Arduino.h>
#include <shconst2.h>
#include <shmess2.h>
#include <shutil2.h>
#include <eepr.h>
#include "config.h"
#include "nrf24l01s_const.h"
#include "nrf24l01s.h"
//#include <Arduino.h>

extern Eepr eeprom;

/* >>>> config concentrateur <<<<<< */

char configRec[CONCRECLEN];       // enregistrement de config  

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

  //uint8_t  channelTable[]={CHANNEL0,CHANNEL1,CHANNEL2,CHANNEL3};   // canal bvs N° conc 
  //const char  concMacTable[] = {CC_ADDRX};
  //uint16_t portTable[MAXCONC] = {CC_UDP0,CC_UDP1,CC_UDP2,CC_UDP3};

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

bool configLoad()                    // INUTILISE 
{
    if(!eeprom.load((byte*)configRec,(uint16_t)CONCRECLEN)){Serial.println("***EEPROM KO***");ledblink(BCODESDCARDKO);return 0;}
    Serial.println("eeprom ok");
    return 1;    
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

void configSave()
{
    eeprom.store((byte*)configRec,CONCRECLEN);
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
  serPurge(1);
  
  for(uint8_t i=0;i<=TSCNB;i++){Serial1.print(RCVSYNCHAR);}
  Serial1.print(CONCCFG);
  
  while(rcvl==0){rcvl=serialRcv(bf,MAXSER,1);}  // attente réponse sans time out

  if(rcvl>5){
    Serial.print(rcvl);Serial.print(" ");Serial.println(bf);
  
    Serial.print("checkData=");
    uint16_t ll=0;
    int cd=checkData(bf,&ll);               // longueur stockée dans le message
    Serial.print(cd);
    if(cd!=1){                              // renvoie mess = MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3) 
      Serial.println(" ko");return 0;}     
    Serial.println(" ok");

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
