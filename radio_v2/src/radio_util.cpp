#include <shconst2.h>
#include <shutil2.h>
#include "radio_const.h"
#include "nrf24l01s.h"


#if NRF_MODE =='C'

extern Nrfp radio;
extern ConTable tableC[];

int get_radio_message(byte* messageIn,uint8_t* pipe,uint8_t* pldLength,int nbper)
{
  int sta=radio.read(messageIn,pipe,pldLength,nbper);
  if(sta>=0){}

      if(sta>=0){
        sta=messageIn[NRF_ADDR_LENGTH]-'0';                       // sender numP
        if(sta!=0 && memcmp(messageIn,tableC[sta].periMac,NRF_ADDR_LENGTH)!=0){  // macAddr ko ?
            sta=AV_MCADD;radio.rxError();}                              // if numP==0 registration to do
      }
  return sta;
}

uint8_t cRegister(char* message)      // search free line or existing macAddr
{                                           // retour NBPERIF -> full else numP

          uint8_t i,freeLine=0;
          bool exist=false;

          for(i=1;i<NBPERIF;i++){
            if(memcmp(tableC[i].periMac,message,NRF_ADDR_LENGTH)==0){exist=true;break;}   // already exist at i
            else if(freeLine==0 && tableC[i].periMac[0]=='0'){freeLine=i;}                // store free line nb
          }

          if(!exist && freeLine!=0){
            i=freeLine;                                                               // i = free line
            exist=true;
            memcpy(tableC[i].periMac,message,NRF_ADDR_LENGTH);                        // record macAddr
            tableC[i].periMac[NRF_ADDR_LENGTH]=i+48;                                  // add numT as 6th char
          }

          if(exist){
            message[NRF_ADDR_LENGTH]=i+48;}

          return i;
}

uint8_t macSearch(char* mac,int* numPer)    // search mac in tableC ; out 1->n ; n<NBPERIF found
{
  int i,j;

  for(i=1;i<NBPERIF;i++){
    for(j=NRF_ADDR_LENGTH-1;j>=0;j--){
      if(mac[j]!=tableC[i].periMac[j]){j=-2;}
    }
    if(j>-2){*numPer=tableC[i].numPeri;break;}
  }
  return i;
}

uint8_t extDataStore(uint8_t numPer,uint8_t numT,uint8_t offset,char* data,uint8_t len)
{
  if(numT>NBPERIF){return EDS_STAT_PER;}
  if(len>BUF_SERVER_LENGTH || len>MAX_PAYLOAD_LENGTH){return EDS_STAT_LEN;}

  tableC[numT].numPeri=numPer;
  if((len+offset)>MAX_PAYLOAD_LENGTH-NRF_ADDR_LENGTH-1){len=MAX_PAYLOAD_LENGTH-NRF_ADDR_LENGTH-1;}
  if(len!=0){
    tableC[numT].servBufLength=len+offset;
    memcpy(tableC[numT].servBuf+offset,data,len);}
  else{
    tableC[numT].servBufLength=MAX_PAYLOAD_LENGTH-NRF_ADDR_LENGTH-1;
    memcpy(tableC[numT].servBuf+offset,SBVINIT,SBLINIT);}

  return EDS_STAT_OK;
}

void tableCPrint()
{
  for(int i=0;i<=NBPERIF;i++){                    // entry #NBPERIF for BR_ADDR requester
    if(i<10){Serial.print(" ");}
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");
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
  memcpy(tableC[0].periMac,(char*)radio.locAddr,NRF_ADDR_LENGTH);
  for(int i=1;i<=NBPERIF;i++){                      // entry #NBPERIF for BR_ADDR requester
    tableC[i].numPeri=0;
    memcpy(tableC[i].periMac,"00000",NRF_ADDR_LENGTH);
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
#endif // NRF_MODE=='C'