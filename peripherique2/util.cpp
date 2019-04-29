
#include "Arduino.h"
#include "const.h"
#include <ESP8266WiFi.h>
#include "ds1820.h"
#include "shconst2.h"
#include "shutil2.h"
#if CONSTANT==EEPROMSAVED
#include <EEPROM.h>
#endif

extern "C" {                  
#include <user_interface.h>     // pour struct rst_info, system_deep_sleep_set_option(), rtc_mem
}

extern Ds1820 ds1820;

extern WiFiClient cli;                 // client local du serveur externe
extern WiFiClient cliext;              // client externe du serveur local

extern constantValues cstRec;

extern char model[LENMODEL];

extern const char* host;

extern char bufServer[LBUFSERVER];

extern byte     mac[6];
extern byte     staPulse[MAXSW];            // état clock pulses
extern uint8_t  pinSw[MAXSW];
extern uint8_t  pinDet[MAXDET];
extern long     detTime[MAXDET+MAXDSP+MAXDEX];

extern          constantValues cstRec;
extern char*    cstRecA;
extern long     dateon;

extern long     tempTime;
extern uint16_t tempPeriod;

extern float    voltage;

void checkVoltage()
{
      voltage=(float)ESP.getVcc()/(float)1024;
      Serial.print(" ");Serial.print(voltage);Serial.print("V ");
#if POWER_MODE != NO_MODE
      if(voltage<3.2){
#if POWER_MODE == PO_MODE
       digitalWrite(PINPOFF,HIGH);        // power down
       pinMode(PINPOFF,OUTPUT);}
#endif PO_MODE
#if POWER_MODE == DS_MODE
       deepsleep(0);}
#endif DS_MODE        
#endif NO_MODE
}

void trigTemp(){startto(&tempTime,&tempPeriod,cstRec.tempPer);}
bool chkTrigTemp(){return ctlto(tempTime,tempPeriod);}
void forceTrigTemp(){tempPeriod=0;}     // -------------> utilisé pour forcer une communication avec le serveur
                                        //                après une éventuelle comm en cours

bool readConstant()
{

#if CONSTANT==RTCSAVED
  int temp=CONSTANTADDR;
  byte buf[4];

//  system_rtc_mem_read(temp,buf,1); // charge la longueur telle qu'enregistrée
//  cstRec.cstlen=buf[0];
  system_rtc_mem_read(temp,cstRecA,cstRec.cstlen);  
#endif

#if CONSTANT==EEPROMSAVED
  cstRec.cstlen=EEPROM.read((int)(CONSTANTADDR));  // charge la longueur telle qu'enregistrée
  for(int temp=0;temp<cstRec.cstlen;temp++){        
    *(cstRecA+temp)=EEPROM.read((int)(temp+CONSTANTADDR));
  }
#endif

for(int k=1;k<=LENVERSION;k++){Serial.print(cstRecA[k]);}          // affichage version ; cstRec.cstVers[k]);}
Serial.print(" readConstant ");Serial.print((long)cstRecA,HEX);Serial.print(" len=");Serial.print(cstRec.cstlen);
Serial.print("/");Serial.print(sizeof(cstRec));
Serial.print(" crc=");Serial.print(*(cstRecA+cstRec.cstlen-1),HEX);Serial.print(" calc_crc=");
byte calc_crc=calcCrc(cstRecA,(uint8_t)cstRec.cstlen-1);Serial.println(calc_crc,HEX);
if(*(cstRecA+cstRec.cstlen-1)==calc_crc){return 1;}
return 0;
}

void writeConstant()
{
memcpy(cstRec.cstVers,VERSION,LENVERSION);
cstRec.cxDurat=millis()-dateon;
cstRec.cstcrc=calcCrc(cstRecA,(uint8_t)cstRec.cstlen-1); 
  
#if CONSTANT==RTCSAVED
  int temp=CONSTANTADDR;
  system_rtc_mem_write(temp,cstRecA,256);
#endif

#if CONSTANT==EEPROMSAVED
  for(int temp=0;temp<cstRec.cstlen;temp++){
    if(*(cstRecA+temp)!=EEPROM.read((int)(temp+CONSTANTADDR))){
      EEPROM.write((int)(temp+CONSTANTADDR),*(cstRecA+temp));
    }    
  }
  EEPROM.commit();
#endif
Serial.print("writeConstant ");for(int h=0;h<4;h++){Serial.print(cstRec.cstVers[h]);};Serial.print(" ");
Serial.print((long)cstRecA,HEX);
Serial.print(" len=");Serial.print((char*)&cstRec.cstcrc-cstRecA+1);
Serial.print("/");Serial.print(cstRec.cstlen);
Serial.print(" crc=");Serial.println(cstRec.cstcrc,HEX);
}

void initConstant()  // inits mise sous tension
{
  cstRec.cstlen=sizeof(cstRec);
  memcpy(cstRec.numPeriph,"00",2);
  cstRec.serverTime=PERSERV+1;            // forçage talkserver à l'init
  cstRec.serverPer=PERSERV;
  cstRec.oldtemp=0;
  cstRec.tempPer=PERTEMP;
  cstRec.tempPitch=0;
  cstRec.talkStep=0;                       // pointeur pour l'automate talkServer()
  cstRec.swCde='\0';                       // cdes inter (4*2bits (periIntVal) ici x0 x=état demandé par le serveur pour le switch)
  
  memset(cstRec.pulseMode,0x00,PCTLLEN);      // ctle pulse
  for(int i=0;i<MAXSW;i++){
  cstRec.durPulseOne[i]=0;       // durée pulses (MAXSW=4*4)
  cstRec.durPulseTwo[i]=0;       // durée pulses (MAXSW=4*4)
  cstRec.cntPulseOne[i]=0;       // compt pulses (MAXSW=4*4)
  cstRec.cntPulseTwo[i]=0;}      // compt pulses (MAXSW=4*4)
  cstRec.IpLocal=IPAddress(0,0,0,0);
  char detDis=DETDIS<<DETBITST_PB;
  memset(cstRec.memDetec,detDis,MAXDET+MAXDSP+MAXDEX);
  memcpy(cstRec.cstVers,VERSION,LENVERSION);
  memcpy(cstRec.cstModel,model,LENMODEL);
  memset(cstRec.swInput,0x00,MAXSW*NBSWINPUT*SWINPLEN);
  cstRec.extDetEn=0;
  cstRec.extDetLev=0;
  cstRec.cxDurat=0;
  memset(cstRec.swToggle,0x00,MAXSW);
  cstRec.portServer=9999;
  Serial.println("Init Constant done");
  writeConstant();
}

void subprintConstant(byte swmode,char a)
{
  Serial.print("     ");Serial.print(a);Serial.print("  ");
//  Serial.print((swmode>>SWMDLNULS_PB));Serial.print("  ");
//  for(int w=5;w>=0;w-=2){Serial.print((swmode>>w)&0x01);Serial.print((swmode>>(w-1))&0x01);Serial.print("  ");}
  Serial.println();
}

void printConstant()
{
  uint64_t swctl=0; 
  char buf[3],buff[32];memcpy(buf,cstRec.numPeriph,2);buf[2]='\0';
  Serial.print("\nnumPeriph=");Serial.print(buf);Serial.print(" IpLocal=");Serial.print(IPAddress(cstRec.IpLocal));
  Serial.print(" serverTime=");Serial.print(cstRec.serverTime);Serial.print(" serverPer=");Serial.println(cstRec.serverPer);
  Serial.print("oldtemp=");Serial.print(cstRec.oldtemp);Serial.print(" tempPer=");Serial.print(cstRec.tempPer);
  Serial.print(" tempPitch=");Serial.print(cstRec.tempPitch);Serial.print("  last durat=");Serial.println(cstRec.cxDurat);
  Serial.print("staPulse=");for(int s=0;s<MAXSW;s++){Serial.print(s);Serial.print("-");Serial.print(staPulse[s],HEX);
  Serial.print(" ");}Serial.println("  C=DIS 0=IDLE 5=RUN1 7=RUN2 4=END1 6=END2");
  Serial.print("memDetec (0-n)=");for(int s=0;s<(MAXDET+MAXDSP+MAXDEX);s++){Serial.print(s);Serial.print("-");
  Serial.print((cstRec.memDetec[s]>>DETBITST_PB)&0x03,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITUD_PB)&0x01,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITLH_PB)&0x01,HEX);Serial.print("  ");}
  Serial.println(" 3-TRIG 2-WAIT 1-IDLE 0-DIS");
  Serial.print("detTime =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(detTime[s]);Serial.print("  -  ");}Serial.println();
  Serial.print("detect  =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(digitalRead(pinDet[s]));Serial.print("   -   ");}Serial.println();  
  Serial.print("ext det =    ");for(int s=NBDSRV-1;s>=0;s--){Serial.print((cstRec.extDetec>>s)&0x01);Serial.print(" ");}Serial.println();
  for(int ns=0;ns<NBSW;ns++){
    Serial.print("sw=");Serial.print(ns+1);Serial.print("-");Serial.print(digitalRead(pinSw[ns]),HEX);
    Serial.print(" F/O=");Serial.print(((byte)(swctl+ns*PCTLLEN)>>PMFRO_PB)&0x01);
    Serial.print("  dur1(");Serial.print(((byte)(swctl+ns*PCTLLEN)>>PMTOE_PB)&0x01);Serial.print(")=");Serial.print(cstRec.cntPulseOne[ns]);Serial.print("/");Serial.print(cstRec.durPulseOne[ns]);
    Serial.print("  dur2(");Serial.print(((byte)(swctl+ns*PCTLLEN)>>PMTTE_PB)&0x01);Serial.print(")=");Serial.print(cstRec.cntPulseTwo[ns]);Serial.print("/");Serial.print(cstRec.durPulseTwo[ns]);
    Serial.println("     codes actions 0=reset 1=raz 2=stop 3=start 4=short 5=end 6=imp");
    for(int ninp=0;ninp<NBSWINPUT;ninp++){  
      Serial.print((*(uint16_t*)(&cstRec.swInput+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>SWINPEN_PB)&0x01); sp(" ",0);             // en input
      Serial.print(*(uint8_t*)(&cstRec.swInput+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)&SWINPNT_MS); sp(" ",0);                      // type détec
      Serial.print(*(uint8_t*)(&cstRec.swInput+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>SWINPNVLS_PB);sp(" ",0);                    // n° détec
      Serial.print(((*(uint16_t*)(&cstRec.swInput+1+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)&SWINPACT_MS)>>SWINPACTLS_PB)&0x01); sp(" ",0);   // act input
      for(int binp=7;binp>=0;binp--){
        Serial.print((*(uint16_t*)(&cstRec.swInput+1+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>(SWINPALLL_PB+binp))&0x01);sp(" ",0);}    // règle
      sp(" ",1);
    }
  }
}



