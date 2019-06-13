
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

char inptyps[]="meexphpu??";                  // libellés types sources inputs
char inptypd[]="meexswpu??";                  // libellés types destinations inputs
char inpact[]={"     RAZ  STOP STARTSHORTEND  IMP  RESETXOR  OR   AND  NOR                      "};  // libellés actions
char psps[]=  {"____IDLEEND1END2RUN1RUN2DISA"};                                                      // libellés staPulse

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

Serial.print("readConstant ");Serial.print((long)cstRecA,HEX);Serial.print(" len=");Serial.print(cstRec.cstlen);
Serial.print("/");Serial.print((uint8_t)((long)&cstRec.cstcrc-(long)cstRecA)+1);
Serial.print(" crc=");Serial.print(*(cstRecA+cstRec.cstlen-1),HEX);Serial.print(" calc_crc=");
byte calc_crc=calcCrc(cstRecA,(uint8_t)cstRec.cstlen-1);Serial.println(calc_crc,HEX);
//dumpstr((char*)cstRecA,256);
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
Serial.print(" crc=");Serial.print(cstRec.cstcrc,HEX);Serial.print(" ");Serial.print((long)&cstRec.cstcrc,HEX);
Serial.print(" numperiph=");Serial.print((char)cstRec.numPeriph[0]);Serial.println((char)cstRec.numPeriph[1]);
}

void initConstant()  // inits mise sous tension
{
  //cstRec.cstlen=(uint8_t)((long)&cstRec.cstcrc-(long)cstRecA)+1;    //sizeof(cstRec);
  cstRec.cstlen=(sizeof(constantValues));  // si le crc est faux, retour à la valeur par défaut
  memcpy(cstRec.numPeriph,"00",2);
  cstRec.serverTime=PERSERV+1;             // forçage talkserver à l'init
  cstRec.serverPer=PERSERV;
  cstRec.oldtemp=0;
  cstRec.tempPer=PERTEMP;
  cstRec.tempPitch=0;
  cstRec.talkStep=0;                       // pointeur pour l'automate talkServer()
  cstRec.swCde='\0';                       // cdes inter (4*2bits (periIntVal) ici x0 x=état demandé par le serveur pour le switch)
  
  memset(cstRec.pulseMode,0x00,PCTLLEN);      // ctle pulse
  for(int i=0;i<NBPULSE;i++){
    cstRec.durPulseOne[i]=0;
    cstRec.durPulseTwo[i]=0;
    cstRec.cntPulseOne[i]=0;
    cstRec.cntPulseTwo[i]=0;}
    cstRec.IpLocal=IPAddress(0,0,0,0);
  char detDis=DETDIS<<DETBITST_PB;
  memset(cstRec.memDetec,detDis,MAXDET);
  memcpy(cstRec.cstVers,VERSION,LENVERSION);
  memcpy(cstRec.cstModel,model,LENMODEL);
  memset(cstRec.perInput,0x00,NBPERINPUT*PERINPLEN);
  cstRec.extDetec=0;
  cstRec.cxDurat=0;
  cstRec.portServer=9999;
  memcpy(cstRec.filler,"AA550123456755AA557654321055A",LENFILLERCST);
  Serial.println("Init Constant done");
  writeConstant();
  //dumpstr((char*)cstRecA,256);
}

void periInputPrint(byte* input)
{
  Serial.print("inputs ");
#define LBINP 29
  char binput[LBINP];
  byte inp[3];
  byte a;
  char ed[]="de",es[]="es";  
  
  for(int ninp=0;ninp<NBPERINPUT;ninp++){  

      memset(binput,0x20,LBINP-1);binput[LBINP-1]=0x00;
      binput[0]=ed[((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPEN_PB)&0x01)];                         // en input
      binput[2]=es[((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPDETES_PB)&0x01)];                      // edge/static input
      binput[4]=((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPOLDLEV_PB)&0x01)+48;                      // prev level
      binput[6]=((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPVALID_PB)&0x01)+48;                       // valid level
      a=*(uint8_t*)(input+ninp*PERINPLEN)&PERINPNT_MS;
      if(a>3){a=4;}binput[8]=inptyps[a*2];binput[9]=inptyps[a*2+1];                                    // type détec src
      a=*(uint8_t*)(input+ninp*PERINPLEN)>>PERINPNVLS_PB;conv_htoa(binput+11,&a);                      // n° détec src
      a=*(uint8_t*)(input+ninp*PERINPLEN+3)&PERINPNT_MS;
      if(a>3){a=4;}binput[14]=inptypd[a*2];binput[15]=inptypd[a*2+1];                                  // type détec dest
      a=*(uint8_t*)(input+ninp*PERINPLEN+3)>>PERINPNVLS_PB;conv_htoa(binput+17,&a);                    // n° détec dest
      a=(*((uint8_t*)(input+2+ninp*PERINPLEN))&PERINPACT_MS)>>PERINPACTLS_PB;conv_htoa(binput+20,&a);  // act input
      for(int tact=0;tact<LENTACT;tact++){binput[24+tact]=inpact[a*LENTACT+tact];}
      Serial.print(binput);
      sp("/ ",0);
    }
    Serial.println();
}

uint32_t sppp(uint32_t cnt)
{
  uint32_t cur=0;
  if(cnt!=0){cur=(millis()-cnt)/1000;}
  return cur;
}

void periPulsePrint(uint16_t* pulseCtl,uint32_t* pulseOne,uint32_t* pulseTwo,uint32_t* cntOne,uint32_t* cntTwo)
{
  Serial.print("pulses(f-e 1 e 2)  ");
  for(int pu=0;pu<NBPULSE;pu++){
    Serial.print((*(uint16_t*)pulseCtl>>(PMFRO_PB+pu*PCTLBIT))&0x01);sp("-",0);         // fr bit
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTOE_PB+pu*PCTLBIT))&0x01);sp(" ",0);       // time one en
    Serial.print(*(uint32_t*)(pulseOne+pu));sp("/",0);                                  // time one
    Serial.print(sppp((uint32_t)*(cntOne+pu)));sp(" ",0);                               // time one écoulé
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTTE_PB+pu*PCTLBIT)&0x01));sp(" ",0);       // time two en
    Serial.print(*(uint32_t*)(pulseTwo+pu));sp("/",0);                                  // time two
    Serial.print(sppp((uint32_t)*(cntTwo+pu)));                                         // time two écoulé    
    Serial.print("  ");for(int tsp=0;tsp<LENTSP;tsp++){Serial.print(psps[staPulse[pu]*LENTSP+tsp]);} // staPulse
    if(pu<NBPULSE-1){sp("  |  ",0);}
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
  Serial.print("numPeriph=");Serial.print(buf);Serial.print(" IpLocal=");Serial.print(IPAddress(cstRec.IpLocal));
  Serial.print("  port=");Serial.print(cstRec.portServer);Serial.print("  sw=");Serial.print(NBSW);Serial.print("  det=");Serial.print(NBDET);
  Serial.print("  ");Serial.println(VERSION);
  Serial.print("SWcde=(");if((cstRec.swCde&0xF0)==0){Serial.print("0");}Serial.print(cstRec.swCde,HEX);Serial.print(") ");
  for(int s=MAXSW;s>=1;s--){Serial.print((char)(((cstRec.swCde>>(2*s-1))&0x01)+48));}
  Serial.print(" serverTime=");Serial.print(cstRec.serverTime);Serial.print(" serverPer=");Serial.print(cstRec.serverPer);
  Serial.print(" oldtemp=");Serial.print(cstRec.oldtemp);Serial.print(" tempPer=");Serial.print(cstRec.tempPer);
  Serial.print(" tempPitch=");Serial.print(cstRec.tempPitch);Serial.print("  last durat=");Serial.print(cstRec.cxDurat);
  //Serial.print(" staPulse=");for(int s=0;s<MAXSW;s++){Serial.print(s);Serial.print("-");Serial.print(staPulse[s],HEX);
  //Serial.print(" ");}Serial.println("  C=DIS 0=IDLE 5=RUN1 7=RUN2 4=END1 6=END2");
  //Serial.print("memDetec (0-n)=");for(int s=0;s<(MAXDET);s++){Serial.print(s);Serial.print("-");
  //Serial.print((cstRec.memDetec[s]>>DETBITST_PB)&0x03,HEX);Serial.print(" ");
  //Serial.print((cstRec.memDetec[s]>>DETBITUD_PB)&0x01,HEX);Serial.print(" ");
  //Serial.print((cstRec.memDetec[s]>>DETBITLH_PB)&0x01,HEX);Serial.print("  ");}
  Serial.println(); //" 3-TRIG 2-WAIT 1-IDLE 0-DIS");
  Serial.print("detTime         =  ");for(int s=MAXDET-1;s>=0;s--){Serial.print(detTime[s]);if(s!=0){Serial.print("   -   ");}}Serial.println();
  Serial.print("detect (pin/mem)= ");for(int s=MAXDET-1;s>=0;s--){Serial.print(digitalRead(pinDet[s]));Serial.print("/");Serial.print((cstRec.memDetec[s]>>DETBITLH_PB)&0x01,HEX);Serial.print("   -   ");}
  Serial.print("switchs (pins)  = ");for(int s=MAXSW-1;s>=0;s--){Serial.print(digitalRead(pinSw[s]));Serial.print("   -   ");}
  Serial.println();  
#if POWER_MODE==NO_MODE
  periDetServPrint(&cstRec.extDetec);  
  periPulsePrint((uint16_t*)&cstRec.pulseMode,(uint32_t*)&cstRec.durPulseOne,(uint32_t*)&cstRec.durPulseTwo,(uint32_t*)&cstRec.cntPulseOne,(uint32_t*)&cstRec.cntPulseTwo);
  periInputPrint((byte*)&cstRec.perInput);
#endif NO_MODE  
}


