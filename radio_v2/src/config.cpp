
#include <Arduino.h>
#include <shconst2.h>
#include <shutil2.h>
#include "eepr.h"
#include "config.h"
#include "radio_const.h"

#define CONFMESSLEN 70

extern Eepr eeprom;

byte  configRec[CONFIGRECLEN];
byte*     configVers;

#if MACHINE_CONCENTRATEUR

/* >>>> config concentrateur <<<<<< */

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

  long configLength=(long)configEndOfRecord-(long)configBegOfRecord+1;  
  Serial.print("CONFIGRECLEN=");Serial.print(CONFIGRECLEN);Serial.print("/");Serial.print(configLength);
  delay(10);if(configLength>CONFIGRECLEN) {ledblink(BCODECONFIGRECLEN);}

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
    char message[MAXSER];memset(message,0x00,MAXSER);
    uint16_t rcvl=0;

    if(!syncServerConfig(message,(char*)CONCCFG,&rcvl)){return 0;};

    char a=' ';
    char* b=message;
    char* newb;
    uint8_t cntpv=0;
    uint16_t temp=0;

    while(cntpv<2 && a!='\0' && b<(message+rcvl)){a=*b++;if(a==';'){cntpv++;}}                // skip len+name+version
    
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;serverIp[i]=temp;}           // serverIp  
    temp=0;conv_atob(b,&temp);b+=6;*serverTcpPort=temp;                                  // server tcp Port
    if(!nextpv(b,&newb,6)){return 0;}b=newb;
    temp=0;conv_atob(b,&temp);b+=6;*serverUdpPort=temp;                                  // server udp Port 
    temp=0;a=' ';while((a=*b++)!=';' && a!='\0' && b<(message+rcvl) && temp<LPWD){peripass[temp]=a;temp++;}  // peripass
  
    packMac((byte*)concMac,b);b+=(MACADDRLENGTH*3-1+1);                          // concMac               
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;concIp[i]=temp;}     // concIp  
    temp=0;conv_atob(b,&temp);b+=6;*concPort=temp;                               // concPort
    memcpy(concRx,b,RADIO_ADDR_LENGTH);b+=RADIO_ADDR_LENGTH+1;                   // radio addr
    temp=0;conv_atob(b,&temp);b+=4;*concChannel=temp;                            // Channel
    temp=0;conv_atob(b,&temp);b+=2;*concRfSpeed=temp;                            // Speed
    *concNb=(uint8_t)(*b-PMFNCVAL);                                              
    
    return rcvl;
}

void configCreate()       // valeurs pour concentrateur de test 
{
  /////// !!!!!!!!!!! CONC2 !!!!!!!!!!! //////////
  serverIp[0]=192;serverIp[1]=168;serverIp[2]=0;serverIp[3]=36;
  *serverTcpPort=1787;
  *serverUdpPort=8885;        // CONC1 8886
  memcpy(peripass,"17515A\0\0",8);
  //memcpy(concMac,"\x72\x37\x68\x30\xFD\xFD",6);
  memcpy(concMac,"\x74\x65\x73\x74\x78\x32",6);            
  concIp[0]=192;concIp[1]=168;concIp[2]=0;concIp[3]=11;//concIp[3]=216;
  *concPort=55558;
  memcpy(concRx,"SHCO2",RADIO_ADDR_LENGTH);
  *concChannel=100;
  *concRfSpeed=0;
  *concNb=2;

  configSave();
}

#endif // MACHINE == 'C'

#if MACHINE_DET328

extern byte message[];

float*    thFactor;
float*    thOffset;
float*    vFactor;
float*    vOffset;
byte*     periRxAddr;
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
  periRxAddr=(byte*)temp;
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
  memcpy(periRxAddr,DEF_ADDR,6);
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
    Serial.print("crc  ");dumpfield((char*)configRec,4);Serial.print(" len ");Serial.print(configLen);Serial.print(" V ");Serial.print(configVers[0]);Serial.println(configVers[1]);
    char buf[7];memcpy(buf,concAddr,5);buf[5]='\0';
    Serial.print("Peri ");dumpstr((char*)periRxAddr,6);Serial.print("Conc ");dumpstr((char*)concAddr,6);
    if(memcmp(configVers,"01",2)!=0){
      Serial.print("concNb ");Serial.print(*concNb);
      Serial.print("  ch ");Serial.print(*concChannel);
      Serial.print("  sp ");Serial.print(*concSpeed);
      Serial.print("  sce(0 peri ; 1 serv) ");Serial.println(*concPeriParams);
    }
    Serial.print("thFactor=");Serial.print(*thFactor*10000);Serial.print("  thOffset=");Serial.print(*thOffset);   
    Serial.print("   vFactor=");Serial.print(*vFactor*10000);Serial.print("   vOffset=");Serial.println(*vOffset);   
    delay(10);
}

uint16_t getServerConfig()
{
#ifndef NOCONFSER

    char pv=';';
    char spf[]={"%04d"};

    char message[CONFMESSLEN];
    message[0]=0x00;strcat((char*)message,"1234;");
    
    uint8_t mm=5;
    sprintf((char*)(message+mm),spf,(int)(*vFactor*10000));mm+=4;        // vFactor
    message[mm]=pv;mm++;
    sprintf((char*)(message+mm),spf,(int)(*vOffset));mm+=4;              // vOffset
    message[mm]=pv;mm++;
    sprintf((char*)(message+mm),spf,(int)(*thFactor*10000));mm+=4;       // thFactor
    message[mm]=pv;mm++;
    sprintf((char*)(message+mm),spf,(int)*thOffset);mm+=4;               // thOffset
    message[mm]=pv;mm++;
    message[mm]=0x00;
    
    memcpy(&message[mm],periRxAddr,RADIO_ADDR_LENGTH);                     // perif Rx addr
    mm+=RADIO_ADDR_LENGTH;
    message[mm]=';';mm++;
    message[mm]='\0';

    setExpEnd((char*)message);                                           // len + crc
    uint16_t rcvl=0;
    if(!syncServerConfig((char*)message,(char*)PERICFG,&rcvl)){return 0;}

    uint16_t cnt=0;
    uint16_t tmp=0;
    int cntint;

    cnt=5;  // skip len
    memcpy(concAddr,message+cnt,RADIO_ADDR_LENGTH);cnt+=RADIO_ADDR_LENGTH+1;                  
    tmp=0;conv_atob((char*)(message+cnt),&tmp);cnt+=4;*concChannel=tmp;                           // Channel
    tmp=0;conv_atob((char*)(message+cnt),&tmp);cnt+=2;*concSpeed=tmp;                             // Speed
    *concNb=(uint8_t)(*(message+cnt)-PMFNCVAL);cnt+=2;                                             
    
    *vFactor=convStrToNum((char*)(message+cnt),&cntint)/10000;cnt+=5;
    *vOffset=convStrToNum((char*)(message+cnt),&cntint);cnt+=5;
    *thFactor=convStrToNum((char*)(message+cnt),&cntint)/10000;cnt+=5;
    *thOffset=convStrToNum((char*)(message+cnt),&cntint);cnt+=5;
    *concPeriParams=*(message+cnt)-PMFNCVAL;cnt+=2;
    memcpy(periRxAddr,(message+cnt),RADIO_ADDR_LENGTH);cnt+=RADIO_ADDR_LENGTH;

  return rcvl;

  #endif // NOCONFSER
}

#endif // MACHINE == 'P'

bool configLoad()
{
    if(!eeprom.load((byte*)configRec,(uint16_t)CONFIGRECLEN)){
      dumpstr((char*)configRec,200);
      Serial.println("**EEPROM KO**");ledblink(BCODESDCARDKO);} // ledblink bloque
    Serial.println(" eeprom ok");
    return 1;    
}

void configSave()
{
  Serial.println("configSave mofifié");delay(100);
  dumpstr((char*)configRec,200);
    eeprom.store((byte*)configRec,CONFIGRECLEN);
  trigwd(0);    
  dumpstr((char*)configRec,200);  
  memset(configRec,0xff,200);
  dumpstr((char*)configRec,200);
  trigwd(0);
    if(eeprom.load(configRec,200)){Serial.println("true");}
  dumpstr((char*)configRec,200);
  trigwd(0);
}

bool syncServerConfig(char* message,char* syncMess,uint16_t* rcvl)
{  
#ifndef NOCONFSER

  serPurge(SERNB);
  for(uint8_t i=0;i<=TSCNB;i++){SERIALX.print(RCVSYNCHAR);}
  SERIALX.print(syncMess);
  if(*message!=0x00){SERIALX.print(message);}

  while(*rcvl==0){*rcvl=serialRcv(message,CONFMESSLEN-1,SERNB);blink(1);}  // attente réponse sans time out
  *(message+*rcvl)=0x00;
  if(*rcvl>5){
    Serial.print(" ");Serial.println(message);
    Serial.print("checkData");
    uint16_t ll=0;
    if(checkData(message,&ll)==MESSOK){Serial.println(" ok");return 1;}
  }
  Serial.println(" ko");return 0;

#endif // NOCONFSER
}
