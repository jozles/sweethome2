
#include "Arduino.h"
#include "const.h"
#include <ESP8266WiFi.h>
#include "ds1820.h"
#include "shconst.h"
#include "shutil.h"
#if CONSTANT==EEPROMSAVED
#include <EEPROM.h>
#endif

extern "C" {                  
#include <user_interface.h>     // pour struct rst_info, system_deep_sleep_set_option(), rtc_mem
}

extern Ds1820 ds1820;

extern WiFiClient cli;                 // client local du serveur externe
extern WiFiClient cliext;              // client externe du serveur local

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
//  memcpy(cstRec.onCdeI,"\0\0\0\0",4);      // mode sw
//  memcpy(cstRec.offCdeI,"\0\0\0\0",4);      // mode sw
//  memcpy(cstRec.onCdeO,"\0\0\0\0",4);       // mode sw
//  memcpy(cstRec.offCdeO,"\0\0\0\0",4);      // mode sw  
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
  
void spc(int det,byte* ctl0){
  uint64_t ctl=0;
/*  memcpy(&ctl,ctl0,DLSWLEN);

    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLNLS_PB))&0x07);
    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLENA_PB))&0x01);
    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLEL_PB))&0x01);
    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLMFE_PB))&0x01);
    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLMHL_PB))&0x01);
    Serial.print("  ");Serial.print((uint8_t)(ctl>>(det*DLBITLEN+DLACLS_PB))&0x07);
*/
}  

void subPC(int i)
{
/*  int nuli=0;
  Serial.println(" Nd  e  x  f  h  Ac      Nd  dd  ss  pp");
  for(nuli=0;nuli<4;nuli++){
    spc(nuli,&cstRec.pulseCtl[i*DLSWLEN]);
    switch(nuli){
      case 0:subprintConstant(cstRec.onCdeI[i],'A');break;
      case 1:subprintConstant(cstRec.offCdeI[i],'D');break;
      case 2:subprintConstant(cstRec.onCdeO[i],'I');break;
      case 3:subprintConstant(cstRec.offCdeO[i],'O');break;
      default:break;
    }
  }
*/  
}

void printConstant()
{
/*  uint64_t swctl=0; 
  char buf[3],buff[32];memcpy(buf,cstRec.numPeriph,2);buf[2]='\0';
  Serial.print("\nnumPeriph=");Serial.print(buf);Serial.print(" IpLocal=");Serial.print(IPAddress(cstRec.IpLocal));
  Serial.print(" serverTime=");Serial.print(cstRec.serverTime);Serial.print(" serverPer=");Serial.println(cstRec.serverPer);
  Serial.print("oldtemp=");Serial.print(cstRec.oldtemp);Serial.print(" tempPer=");Serial.print(cstRec.tempPer);
  Serial.print(" tempPitch=");Serial.print(cstRec.tempPitch);Serial.print("  last durat=");Serial.println(cstRec.cxDurat);
  Serial.print("swCde=");for(int sc=3;sc>=0;sc--){Serial.print((cstRec.swCde>>(sc*2+1))&01);Serial.print((cstRec.swCde>>(sc*2))&01);Serial.print("(");Serial.print(digitalRead(pinSw[sc]));Serial.print(")");
  Serial.print(" ");}Serial.println(" 3/2/1/0 cde/état(pin)");
  Serial.print("staPulse=");for(int s=0;s<MAXSW;s++){Serial.print(s);Serial.print("-");Serial.print(staPulse[s],HEX);
  Serial.print(" ");}Serial.println("  C=DIS 0=IDLE 5=RUN1 7=RUN2 4=END1 6=END2");
  Serial.print("memDetec (0-n)=");for(int s=0;s<(MAXDET+MAXDSP+MAXDEX);s++){Serial.print(s);Serial.print("-");
  Serial.print((cstRec.memDetec[s]>>DETBITST_PB)&0x03,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITUD_PB)&0x01,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITLH_PB)&0x01,HEX);Serial.print("  ");}
  Serial.println(" 3-TRIG 2-WAIT 1-IDLE 0-DIS");
  Serial.print("detTime =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(detTime[s]);Serial.print("  -  ");}Serial.println();
  Serial.print("detect  =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(digitalRead(pinDet[s]));Serial.print("   -   ");}Serial.println();  
  Serial.print("ext detec (en/lev)=  ");for(int s=MAXDEX-1;s>=0;s--){Serial.print((cstRec.extDetEn>>s)&0x01);Serial.print((cstRec.extDetLev>>s)&0x01);Serial.print(" ");}Serial.println();
  for(int i=0;i<NBSW;i++){
    Serial.print("sw=");Serial.print(i+1);Serial.print("-");Serial.print(digitalRead(pinSw[i]),HEX);
    Serial.print(" Cde=");Serial.print((cstRec.swCde>>(i*2+1))&0x01);
    //memcpy(&swctl,&cstRec.pulseCtl[i*DLSWLEN],DLSWLEN);
    Serial.print(" F/O=");Serial.print((byte)(swctl>>PMFRO_PB)&0x01);
    Serial.print("  dur1(");Serial.print((byte)(swctl>>PMTOE_PB)&0x01);Serial.print(")=");Serial.print(cstRec.cntPulseOne[i]);Serial.print("/");Serial.print(cstRec.durPulseOne[i]);
    Serial.print("  dur2(");Serial.print((byte)(swctl>>PMTTE_PB)&0x01);Serial.print(")=");Serial.print(cstRec.cntPulseTwo[i]);Serial.print("/");Serial.print(cstRec.durPulseTwo[i]);
    Serial.println("     codes actions 0=reset 1=raz 2=stop 3=start 4=short 5=end 6=imp");
    subPC(i);
  }
*/
}



