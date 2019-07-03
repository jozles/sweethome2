
#include <Arduino.h>
#include <SD.h>
//#include <nRF24L01.h>
//#include "nrf24l01p.h"
#include "conc_nrf_const.h"

#if NRF_MODE == 'C'

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;

void tableCPrint()
{
  for(int i=0;i<NBPERIF;i++){
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numNrf24);Serial.print(" ");
    Serial.print(tableC[i].numCirc);Serial.print(" ");
    Serial.print(tableC[i].numPipe);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");
    //Serial.println((byte*)tableC[i].periMac);
  }
}

void tableCInit()
{
  for(int i=0;i<NBPI;i++){
    tableC[i].numNrf24=i;
    tableC[i].numCirc=i/PI_C;      // (0->n)
    tableC[i].numPipe=(i%PI_C)+1;  // (1->5)
    memcpy(tableC[i].pipeAddr,nrfp.r1_addr,ADDR_LENGTH-2);
    sprintf(tableC[i].pipeAddr+3,"02d",tableC[i].numCirc*10+tableC[i].numPipe);
    tableC[i].numPeri=0;
    if(NRF_MODE=='C'){
      memcpy(tableC[i].periMac,"00000",5);}
    else{
      memcpy(tableC[i].periMac,R1_ADDR,5);}
  }
}

int tableCLoad()
{
}

int tableCSave()
{
}

#endif NRF_MODE == 'C'
