
#include <Arduino.h>
#include <SD.h>
//#include <nRF24L01.h>
#include "nrf24l01p.h"
#include "conc_nrf_const.h"

#if NRF_MODE == 'C'

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;



void printAddr(char* addr,char n)
{
  for(int j=0;j<ADDR_LENGTH;j++){Serial.print((char)addr[j]);}
  if(n=='n'){Serial.println();}
}

void tableCPrint()
{
  for(int i=0;i<NBPERIF;i++){
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numNrf24);Serial.print(" ");
    Serial.print(tableC[i].numCirc);Serial.print(" ");
    Serial.print(tableC[i].numPipe);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");
    
    printAddr(tableC[i].periMac,' ');Serial.print(" ");
    printAddr(tableC[i].pipeAddr,'n');
  }
}

void tableCInit()
{
  for(int i=0;i<NBPERIF;i++){
    tableC[i].numNrf24=i;
    tableC[i].numCirc=i/NB_PIPE;      // (0->n)
    tableC[i].numPipe=i%NB_PIPE;      // (0->5)
    tableC[i].numPeri=0;
    memcpy(tableC[i].pipeAddr,nrfp.r0_addr,ADDR_LENGTH-1);
    //sprintf(tableC[i].pipeAddr+3,"%02d",(tableC[i].numCirc*10+tableC[i].numPipe));
    tableC[i].pipeAddr[ADDR_LENGTH-1]='0'+(tableC[i].numCirc*NB_PIPE)+tableC[i].numPipe;
    memcpy(tableC[i].periMac,"00000",ADDR_LENGTH);
  }
}

int tableCLoad()
{
}

int tableCSave()
{
}

#endif NRF_MODE == 'C'
