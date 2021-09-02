
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

uint16_t hostPort=0;             // server port (udp/tcp selon TXRX_MODE)
extern uint8_t numConc;   

void configInitVar()
{
  numConc=0;
  memset(serverIp,0x00,4);
  *serverTcpPort=0;
  *serverUdpPort=0;
  memcpy(concMac,concAddrTable+numConc*MACADDRLENGTH,MACADDRLENGTH);
  *concChannel=channelTable[numConc];
  *concRfSpeed=RF_SPEED;
  memset(concIp,0x00,4);
  *concPort=portTable[numConc];
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
  concChannel=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concRfSpeed=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concNb=(uint8_t*)temp;
  temp+=sizeof(uint8_t);

  temp+=78;                          // dispo 
  cfgCrc=(uint32_t*)temp;
  temp+=sizeof(uint32_t);

  configEndOfRecord=(byte*)temp;      // doit être le dernier !!!

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
  Serial.print("numConc =");Serial.println(numConc);
  Serial.print("serverIP=");serialPrintIp(serverIp);Serial.print("/");Serial.print(*serverTcpPort);Serial.print("/");Serial.println(*serverUdpPort);
  Serial.print("concIP  =");serialPrintIp(concIp);Serial.print("/");Serial.println(*concPort);
  Serial.print("concMac =");serialPrintMac(concMac,1);
  Serial.print("channel =");Serial.print(*concChannel);Serial.print(" speed=");Serial.println(*concRfSpeed);
  Serial.print("peripass=");Serial.println(peripass);
}

void configSave()
{
    eeprom.store((byte*)configRec,CONCRECLEN);
}

uint16_t nextpv(char* b,uint16_t l)
{
    while(*b!=';' && l>0){b++;l--;}
    return l;
}

uint16_t getServerConfig()
{
  char bf[MAXSER];memset(bf,0x00,MAXSER);
  serPurge(1);
  
  for(uint8_t i=0;i<=TSCNB;i++){Serial1.print(RCVSYNCHAR);}
  Serial1.print(CONCCFG);
  //delay(1000);                              // tx time (14*100uS) + response time (100uS)
  uint16_t rcvl=serialRcv(bf,MAXSER,1);     // longueur effectivement reçue (strlen(bf))
  
  if(rcvl>0){
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
    uint8_t cntpv=0;
    uint16_t temp=0;
    uint16_t lpv,ppv;

    while(cntpv<2 && a!='\0' && b<(bf+rcvl)){a=*b++;if(a==';'){cntpv++;}}                // skip len+name+version
    
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;serverIp[i]=temp;}           // serverIp  
    temp=0;conv_atob(b,&temp);b+=6;*serverTcpPort=temp;                                  // server tcp Port
    lpv=6;ppv=nextpv(b,lpv);if(ppv==0){return 0;}                                        // skip remote port
    b+=lpv-ppv+1;
    temp=0;conv_atob(b,&temp);b+=6;*serverUdpPort=temp;                                  // server udp Port 
    temp=0;a=' ';while((a=*b++)!=';' && a!='\0' && b<(bf+rcvl) && temp<LPWD){peripass[temp]=a;temp++;}  // peripass

    for(uint8_t nc=0;nc<MAXCONC;nc++){                                                   // get 4 conc
        if(nc==numConc){                                                                 // mac,ip,port,channel,speed
#define NBCF 5                                                                           // nbre champs dans table conc
            packMac((byte*)concMac,b);b+=(MACADDRLENGTH*3-1+1);                          // concMac               
            for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;concIp[i]=temp;}     // concIp  
            temp=0;conv_atob(b,&temp);b+=6;*concPort=temp;                               // concPort
            temp=0;conv_atob(b,&temp);b+=4;*concChannel=temp;                            // Channel
            temp=0;conv_atob(b,&temp);b+=2;*concRfSpeed=temp;                            // Speed
        }
        else {
          for(uint8_t i=0;i<NBCF;i++){
            lpv=(MACADDRLENGTH*3-1+1)+15+6+4+2;ppv=nextpv(b,lpv);
            if(ppv==0){return 0;}}}                   
            b+=lpv-ppv+1;                                                                // skip other
    }
  }
  return rcvl;
}
