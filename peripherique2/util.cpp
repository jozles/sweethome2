
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
extern long     detTime[MAXDET];

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
Serial.print("/");Serial.print((uint8_t)((long)&cstRec.cstcrc-(long)cstRecA)+1);
Serial.print(" crc=");Serial.print(*(cstRecA+cstRec.cstlen-1),HEX);Serial.print(" calc_crc=");
byte calc_crc=calcCrc(cstRecA,(uint8_t)cstRec.cstlen-1);Serial.println(calc_crc,HEX);
dumpstr((char*)cstRecA,256);
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
Serial.print(" crc=");Serial.print(cstRec.cstcrc,HEX);Serial.print(" ");Serial.println((long)&cstRec.cstcrc,HEX);
}

void initConstant()  // inits mise sous tension
{
  cstRec.cstlen=(uint8_t)((long)&cstRec.cstcrc-(long)cstRecA)+1;    //sizeof(cstRec);
  memcpy(cstRec.numPeriph,"00",2);
  cstRec.serverTime=PERSERV+1;             // forçage talkserver à l'init
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
  memset(cstRec.memDetec,detDis,MAXDET);
  memcpy(cstRec.cstVers,VERSION,LENVERSION);
  memcpy(cstRec.cstModel,model,LENMODEL);
  memset(cstRec.swInput,0x00,MAXSW*NBSWINPUT*SWINPLEN);
  cstRec.extDetec=0;
  cstRec.cxDurat=0;
  memset(cstRec.swToggle,0x00,MAXSW);
  cstRec.portServer=9999;
  memcpy(cstRec.filler,"AA550123456755AA557654321055A",31);
  Serial.println("Init Constant done");
  writeConstant();
  dumpstr((char*)cstRecA,256);
}

void periInputPrint(byte* input,int nbns)
{
  Serial.println("switchs/inputs       codes actions 0=reset 1=raz 2=stop 3=start 4=short 5=end 6=imp");

#define LBINP 12   
  char binput[LBINP];
  byte a;
  char typ[]="__exlopu??";
  for(int ninp=0;ninp<NBSWINPUT;ninp++){  
    for(int ns=0;ns<nbns;ns++){      
      memset(binput,0x20,LBINP-1);binput[LBINP-1]=0x00;
      binput[0]=((*(uint8_t*)(input+2+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>SWINPEN_PB)&0x01)+48;                        // en input
      a=*(uint8_t*)(input+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)&SWINPNT_MS;
      if(a>3){a=4;}binput[2]=typ[a*2];binput[3]=typ[a*2+1];                                                              // type détec
      a=*(uint8_t*)(input+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>SWINPNVLS_PB;conv_htoa(binput+5,&a);                     // n° détec
      a=(*(uint8_t*)(input+2+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)&SWINPACT_MS)>>SWINPACTLS_PB;conv_htoa(binput+8,&a);    // act input
      Serial.print(binput);
      
      for(int binp=7;binp>=0;binp--){
        Serial.print((*(uint8_t*)(input+1+ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN)>>(SWINPRULESLS_PB+binp))&0x01);    // règle
      }

/*      for(int binp=7;binp>=0;binp--){               // exemple plantage alignement
        Serial.print(*(uint16_t*)(input+1));   
      }*/
      
      sp("   ",0);
    }
    Serial.println();
  }
}

void periPulsePrint(uint16_t* pulseCtl,uint32_t* pulseOne,uint32_t* pulseTwo)
{
  Serial.print("pulses(f-e 1 e 2)  ");
  for(int pu=0;pu<NBPULSE;pu++){
    Serial.print((*(uint16_t*)pulseCtl>>(PMFRO_VB+pu*PCTLBIT))&0x01);sp("-",0);         // fr bit
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTOE_VB+pu*PCTLBIT))&0x01);sp(" ",0);       // time one en
    Serial.print(*(uint32_t*)(pulseOne+pu));sp(" ",0);                                  // time one
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTTE_VB+pu*PCTLBIT)&0x01));sp(" ",0);       // time two en
    Serial.print(*(uint32_t*)(pulseTwo+pu));if(pu<NBPULSE-1){sp("  |  ",0);}            // time two
  }Serial.println();
}

void periDetServPrint(uint32_t* detserv)
{
  Serial.print("det serv=");
  for(int d=NBDSRV-1;d>=0;d--){Serial.print((char)(((*detserv>>d)&0x01)+48));Serial.print(" ");} 
  Serial.println();
}  

void printConstant()
{
  uint64_t swctl=0; 
  char buf[3],buff[32];memcpy(buf,cstRec.numPeriph,2);buf[2]='\0';
  Serial.print("\nnumPeriph=");Serial.print(buf);Serial.print(" IpLocal=");Serial.print(IPAddress(cstRec.IpLocal));
  Serial.print("  port=");Serial.print(cstRec.portServer);Serial.print("  sw=");Serial.print(NBSW);Serial.print("  det=");Serial.print(NBDET);
  Serial.print("  ");Serial.println(VERSION);
  Serial.print("SWcde=(");if((cstRec.swCde&0xF0)==0){Serial.print("0");}Serial.print(cstRec.swCde,HEX);Serial.print(") ");
  for(int s=MAXSW;s>=1;s--){Serial.print((char)(((cstRec.swCde>>(2*s-1))&0x01)+48));}
  Serial.print(" serverTime=");Serial.print(cstRec.serverTime);Serial.print(" serverPer=");Serial.print(cstRec.serverPer);
  Serial.print(" oldtemp=");Serial.print(cstRec.oldtemp);Serial.print(" tempPer=");Serial.print(cstRec.tempPer);
  Serial.print(" tempPitch=");Serial.print(cstRec.tempPitch);Serial.print("  last durat=");Serial.print(cstRec.cxDurat);
  Serial.print(" staPulse=");for(int s=0;s<MAXSW;s++){Serial.print(s);Serial.print("-");Serial.print(staPulse[s],HEX);
  Serial.print(" ");}Serial.println("  C=DIS 0=IDLE 5=RUN1 7=RUN2 4=END1 6=END2");
  Serial.print("memDetec (0-n)=");for(int s=0;s<(MAXDET);s++){Serial.print(s);Serial.print("-");
  Serial.print((cstRec.memDetec[s]>>DETBITST_PB)&0x03,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITUD_PB)&0x01,HEX);Serial.print(" ");
  Serial.print((cstRec.memDetec[s]>>DETBITLH_PB)&0x01,HEX);Serial.print("  ");}
  Serial.println(" 3-TRIG 2-WAIT 1-IDLE 0-DIS");
  Serial.print("detTime =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(detTime[s]);Serial.print("  -  ");}Serial.println();
  Serial.print("detect  =    ");for(int s=MAXDET-1;s>=0;s--){Serial.print(digitalRead(pinDet[s]));Serial.print("   -   ");}Serial.println();  
  periDetServPrint(&cstRec.extDetec);  
  periPulsePrint((uint16_t*)&cstRec.pulseMode,(uint32_t*)&cstRec.durPulseOne,(uint32_t*)&cstRec.durPulseTwo);
  periInputPrint((byte*)&cstRec.swInput,NBSW);
}




