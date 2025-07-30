#include <arduino.h>
#include <shconst2.h>
#include "radio_const.h"

#ifdef  NRF
#include "nrf24l01s.h"
extern Nrfp radio;
#endif

#ifdef LORA
#include "LoRa.h"
#include "LoRa_const.h"
extern LoRaClass radio;
#endif

extern bool diags;

#ifdef MACHINE_CONCENTRATEUR

extern ConTable tableC[];

uint8_t macSearch(char* mac,int* numPer)    // search mac in tableC ; out 1->n ; n<NBPERIF found
{
  int i,j;

  for(i=1;i<NBPERIF;i++){
    for(j=RADIO_ADDR_LENGTH-1;j>=0;j--){
      if(mac[j]!=tableC[i].periMac[j]){j=-2;}
    }
    if(j>-2){*numPer=tableC[i].numPeri;break;}
  }
  return i;
}

int get_radio_message(byte* messageIn,uint8_t* pipe,uint8_t* pldLength)
{
  int sta=radio.read(messageIn,pipe,pldLength,NBPERIF);         // MACHINE_CONCENTRATEUR returns 0:reg_to_do <0:err >0:numPer

      if(sta>=0){
        sta=messageIn[RADIO_ADDR_LENGTH]-'0';                                       // sender numP 0=registration_req
        if(sta!=0 && memcmp(messageIn,tableC[sta].periMac,RADIO_ADDR_LENGTH)!=0){   // macAddr ko ?
            sta=AV_MCADD; 
            radio.rxError();                                                        // if numP==0 registration to do
        }
      }
  return sta;       // returns 0:registration_req <0:err >0:numPer_ok
}

uint8_t cRegister(char* message,uint8_t pldL)      // search free line or existing macAddr
{                                     // retour NBPERIF -> full else numP
          uint8_t i,freeLine=0;
          bool exist=false;

          for(i=1;i<NBPERIF;i++){
            if(memcmp(tableC[i].periMac,message,RADIO_ADDR_LENGTH)==0){exist=true;break;} // already exist at i
            else if(freeLine==0 && tableC[i].periMac[0]=='0'){freeLine=i;}                // store free line nb
          }

          if(!exist && freeLine!=0){
            i=freeLine;                                                                   // i = free line
            exist=true;
            memcpy(tableC[i].periBuf,message,pldL);                                       // record message
            memcpy(tableC[i].periMac,message,RADIO_ADDR_LENGTH);                          // record mac
            tableC[i].periMac[RADIO_ADDR_LENGTH]=i+48;                                    // add numT as 6th char
            tableC[i].periBufLength=pldL;                                                 // record length
          }

          if(exist){
            message[RADIO_ADDR_LENGTH]=i+48;}

          return i;
}

uint8_t extDataStore(uint8_t numPer,uint8_t numT,uint8_t offset,char* data,uint8_t len)
{
  if(numT>NBPERIF){return EDS_STAT_PER;}
  if(len>BUF_SERVER_LENGTH || len>MAX_PAYLOAD_LENGTH){return EDS_STAT_LEN;}

  tableC[numT].numPeri=numPer;
  if((len+offset)>MAX_PAYLOAD_LENGTH-RADIO_ADDR_LENGTH-1){len=MAX_PAYLOAD_LENGTH-RADIO_ADDR_LENGTH-1;}
  if(len!=0){
    tableC[numT].servBufLength=len+offset;
    memcpy(tableC[numT].servBuf+offset,data,len);}
  else{
    tableC[numT].servBufLength=MAX_PAYLOAD_LENGTH-RADIO_ADDR_LENGTH-1;
    memcpy(tableC[numT].servBuf+offset,SBVINIT,SBLINIT);}

  return EDS_STAT_OK;
}

void tableCPrint()
{
  for(int i=0;i<=NBPERIF;i++){                    // entry #NBPERIF for BR_ADDR requester
    if(i<10){Serial.print(" ");}
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");if(tableC[i].numPeri<10){Serial.print(" ");}
    radio.printAddr((char*)tableC[i].periMac,0);Serial.print("  (");
    Serial.print(tableC[i].periBufLength);Serial.print("/");
    Serial.print(tableC[i].periBufSent);Serial.print(") ");
    Serial.print(tableC[i].periBuf);Serial.print(" (");
    
    Serial.print(tableC[i].servBufLength);Serial.print(")");
    Serial.print(tableC[i].servBuf);Serial.print(" ");
    
    Serial.println();
  }
}

void tableCInit()
{
  memcpy(tableC[0].periMac,(char*)radio.locAddr,RADIO_ADDR_LENGTH);
  for(int i=1;i<=NBPERIF;i++){                      // entry #NBPERIF for BR_ADDR requester
    tableC[i].numPeri=0;
    memcpy(tableC[i].periMac,"00000",RADIO_ADDR_LENGTH);
    tableC[i].servBufLength=SBLINIT;
    memcpy(tableC[i].servBuf,SBVINIT,SBLINIT);
    memset(tableC[i].periBuf,'\0',MAX_PAYLOAD_LENGTH+1);
    tableC[i].periBufLength=0;
    tableC[i].periBufSent=false;
  }
}

int tableCLoad()
{
  return 1;
}

int tableCSave()
{
  return 1;
}
#endif // MACHINE_CONCENTRATEUR

#ifdef MACHINE_DET328

#include "radio_const.h"
#include "lpavr_powerSleep.h"

extern int16_t sleepTimings[];
extern int32_t realSleepTimings[];
extern int32_t loopSleepTimings[];

/*
uint8_t sleepDly_old(int32_t dly,int32_t* slpt)                    // should be (nx32)
{
  #define DLYVAL 3500                     // !!!!!! need computed param // loop dly value
  #define SLEEPT 3505                     // !!!!!! need computed param // loop sleep value (micros()/millis() loss)
  // !!!!!!!!!!!!!!!!!!!!! anomalie : le temps de sleep devrait être < temps boucle ?????????????????? 

  //unsigned long tmicros=micros();
  
  //int32_t dly0=dly;
  dly*=100;
  int32_t remainder=dly%DLYVAL;                      
  dly=dly-remainder;
  int32_t slpt0=0;
  uint8_t k=0;
  while(dly>0){
//markerLow(MARKER2);
    sleepNoPwr(T32);                      // sleepNoPwr(T32) vaut 34.46mS
    dly-=DLYVAL;
    slpt0+=SLEEPT;   
    k++;
//markerLow(MARKER2); //34.50ms   
  }

  *slpt+=slpt0/100;

  if(diags){
    unsigned long tmicros2=micros()-tmicros;
    Serial.print("\ndly ");Serial.print(dly0);
    Serial.print("__");Serial.print(tmicros2);
    Serial.print(" + ");Serial.print(slpt0*10);
    Serial.print(" = ");Serial.print(tmicros2+slpt0*10);
    Serial.print(" @ ");Serial.println(k);
    delay(5);
  }
    
  return remainder/100;
}*/

void hardwarePwrUp()
{ 
  //pinMode(REED,INPUT_PULLUP);
}

uint8_t sleepDly(int32_t dly,int32_t* slpt)                  
{
  //#define DLYVAL 3500                     // !!!!!! need computed param // loop dly value
  //#define SLEEPT 3505                     // !!!!!! need computed param // loop sleep value (micros()/millis() loss)
  // !!!!!!!!!!!!!!!!!!!!! anomalie : le temps de sleep devrait être < temps boucle ?????????????????? 

  //unsigned long tmicros=micros();

  dly*=100;
  if(dly>=realSleepTimings[NB_PRESCALER_VALUES-2]){

    int32_t slpt0=0;
    uint8_t k=0;
    while(k<NB_PRESCALER_VALUES-1){
      while(dly>realSleepTimings[k]){
        sleepPwrDown(sleepTimings[k]);        // pas de gestion du matériel qui doit être effectuée en amont/aval
                                              // permet de ne pas éteindre ce qui doit rester en fonctionnement (radio pendant POWONDLY par ex)  
        //sleepNoPwr(sleepTimings[k]);                      
        dly-=realSleepTimings[k];
        slpt0+=loopSleepTimings[k];      
      }
      k++;
    }
    *slpt+=slpt0/100;
  }  
  return (uint8_t)(dly/100);
}

uint8_t sleepDly(int32_t dly) 
{
  int32_t slpt;
  return sleepDly(dly,&slpt);
}

void medSleepDly(int32_t dly)
{
  delay(sleepDly(dly));
}

#endif // MACHINE_DET328



void marker(uint8_t markerPin)      // 100uS
{
  #ifndef MACHINE_DET328
  pinMode(markerPin,OUTPUT);digitalWrite(markerPin,HIGH);delayMicroseconds(250);digitalWrite(markerPin,LOW);
  #endif
  #if MACHINE_DET328
  bitSet(DDR_DIG1,markerPin);bitSet(PORT_DIG1,markerPin);delayMicroseconds(100);bitClear(PORT_DIG1,markerPin);
  #endif
}

void markerL(uint8_t markerPin)     // 500uS
{
  #ifndef MACHINE_DET328
  //pinMode(markerPin,OUTPUT);digitalWrite(markerPin,HIGH);delayMicroseconds(500);digitalWrite(markerPin,LOW);
  #endif
  #if MACHINE_DET328
  bitSet(DDR_DIG1,markerPin);bitSet(PORT_DIG1,markerPin);delayMicroseconds(500);bitClear(PORT_DIG1,markerPin);
  #endif
}