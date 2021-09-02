
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
  uint16_t* serverPort;       // server port

  char*     peripass;         // mot de passe périphériques

  uint8_t*  concMac;          // (table concentrateurs) macaddr concentrateur (5 premiers caractères valides le 6ème est le numéro dans la table)
  uint16_t* concChannel;      // (table concentrateurs) n° channel nrf utilisé par le concentrateur
  uint16_t* concRfSpeed;      // (table concentrateurs) RF_Speed concentrateur
  byte*     concIp;           // (table concentrateurs) adresse IP concentrateur
  uint16_t* concPort;         // (table concentrateurs) port concentrateur
  uint8_t*  concNb;

  uint32_t* cfgCrc;

  byte* configBegOfRecord;
  byte* configEndOfRecord;

  uint8_t  channelTable[]={CHANNEL0,CHANNEL1,CHANNEL2,CHANNEL3};   // canal bvs N° conc 
  byte*    concAddrTable[MAXCONC] = {CC_ADDR0,CC_ADDR1,CC_ADDR2,CC_ADDR3};
  uint16_t portTable[MAXCONC] = {CC_UDP0,CC_UDP1,CC_UDP2,CC_UDP3};
  #define DEFCONC 0

extern uint8_t numConc;   

void configInitVar()
{
  numConc=0;
  memset(serverIp,0x00,4);
  *serverPort=0;
  memcpy(concMac,concAddrTable+numConc*MACADDRLENGTH,MACADDRLENGTH);
  *concChannel=channelTable[numConc];
  *concRfSpeed=RF_SPEED;
  memset(concIp,0x00,4);
  *concPort=portTable[numConc];
}

void configInit()
{
byte* temp=(byte*)configRec;

  configBegOfRecord=(byte*)temp;         // doit être le premier !!!
 
  cfgLen=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  temp+=2;                      // adresse multiple de 4 pour début lecture/écriture

  serverIp=(byte*)temp;
  temp+=4;
  serverPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);

  peripass=(char*)temp;
  temp+=(LPWD+1);

  concMac=(uint8_t*)temp;
  temp+=(MAXCONC*MACADDRLENGTH);
  concIp=(byte*)temp;
  temp+=4;
  concPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  concChannel=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concRfSpeed=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concNb=(uint8_t*)temp;
  temp+=sizeof(uint8_t);

  temp+=64;                          // dispo 
  cfgCrc=(uint32_t*)temp;
  temp+=sizeof(uint32_t);

  configEndOfRecord=(byte*)temp;      // doit être le dernier !!!

  configInitVar();
}

bool configLoad()
{
    if(!eeprom.load((byte*)configRec,(uint16_t)CONCRECLEN)){Serial.println("***EEPROM KO***");return 0;}
    Serial.println("eeprom ok");
    return 1;    
}

void configPrint()
{
  Serial.print("numConc= ");Serial.println(numConc);
  Serial.print("serverIP=");serialPrintIp(serverIp);Serial.print("/");Serial.println(*serverPort);
  Serial.print("  concIP=");serialPrintIp(concIp);Serial.print("/");Serial.println(*concPort);
  Serial.print(" concMac=");serialPrintMac(concMac,1);
  Serial.print(" channel=");Serial.print(*concChannel);Serial.print(" speed=");Serial.println(*concRfSpeed);
}

void configSave()
{
    eeprom.store((byte*)configRec,CONCRECLEN);
}

bool nextpv(char* b,int l)
{
    while(*b++!=';' && --l>0){}
    return l;
}

uint16_t getServerConfig()
{
  char bf[MAXSER];
  
  for(uint8_t i=0;i<=RSCNB;i++){Serial1.print(RCVSYNCHAR);}
  Serial1.print(CONCCFG);
  delay(10);                                 // tx time (14*100uS) + response time (100uS)
  uint16_t rcvl=serialRcv(bf,MAXSER,1);     // longueur effectivement reçue (strlen(bf))
  
  if(rcvl>0){
    Serial.println(bf);
  
    Serial.print("checkData=");
    uint16_t ll=0;
    int cd=checkData(bf,&ll);               // longueur stockée dans le message
    Serial.print(cd);
    if(cd!=1){                              // renvoie mess = MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3) 
      Serial.println(" ko");return 0;}     
    Serial.println(" ok");

    char a=' ';
    char* b=bf;
    uint8_t cntpv=0;

    while(cntpv<2 && a!='\0' && b<(bf+rcvl)){a=*b++;if(a==';'){cntpv++;}}                // skip len+name+version
    uint16_t temp=0;
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;serverIp[i]=temp;}           // serverIp  
    temp=0;conv_atob(b,&temp);b+=6;*serverPort=temp;                                     // serverPort
    if(!nextpv(b,1000)){return 0;}                                                       // skip remote port
    if(!nextpv(b,1000)){return 0;}                                                       // skip server concPort    
    for(uint8_t nc=0;nc<MAXCONC;nc++){                                                   // get 4 conc
        if(nc==numConc){                                                                 // mac,ip,port,channel,speed
#define NBCF 5                                                                           // nbre champs dans table conc
            packMac((byte*)concMac,b);b+=(MACADDRLENGTH*2+1);                            // concMac               
            for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;concIp[i]=temp;}     // concIp  
            temp=0;conv_atob(b,&temp);b+=6;*concPort=temp;                               // concPort
            temp=0;conv_atob(b,&temp);b+=6;*concChannel=temp;                            // Channel
            temp=0;conv_atob(b,&temp);b+=6;*concRfSpeed=temp;                            // Speed
        }
        else {
          for(uint8_t i=0;i<NBCF;i++){if(!nextpv(b,1000)){return 0;}}}                   // skip other
    }
  }
  return rcvl;
}
