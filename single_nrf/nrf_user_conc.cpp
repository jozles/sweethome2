#include "nrf_user_conc.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern struct NrfConTable tableC[NBPERIF];

#if NRF_MODE == 'C'

/* user includes */

#include <Ethernet.h> //bibliothèque W5100 Ethernet
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"

/* user fields */

char model[]={"D32800"};

char bufServer[LBUFSERVER];
char srvpswd[LPWD];

char* chexa="0123456789ABCDEFabcdef\0";

/* cycle functions */

int exportData(uint8_t numT)
{
  strcpy(bufServer,"GET /cx?\0");
  if(!buildMess("peri_pass_",srvpswd,"?")==MESSOK){
    Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(srvpswd);return MESSDEC;};

  char message[LENVAL];
  int sb=0,i=0,k;
  char x[2]={'\0','\0'};
  
      sprintf(message,"%02d",tableC[numT].numPeri);                 // N° périf                    
      memcpy(message+2,"_\0",2);                                    //                        - 3
      sb=3;
      unpackMac((char*)(message+sb),tableC[numT].periMac);          // macaddr                    - 18
      sb+=16;
      memcpy(message+sb,"_\0",2);
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+7+1+LENVERSION+8,6);   // temp                              
      sb+=6;
      memcpy(message+sb,"__\0",3);                                  // âge 
      sb+=2;
      memcpy(message+sb,tableC[numT].periBuf+7+1+LENVERSION+8+6,4); // volts                              
      sb+=6;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+7+1,LENVERSION);       // VERSION
      sb+=4;
      memcpy(message+sb,tableC[numT].periBuf+7,1);                  // modele DS18x20
      sb+=1;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;      
      
#define NBSW 0
      message[sb]=(char)(NBSW+48);                                    // nombre switchs              - 1   
      //for(i=0;i<NBSW;i++){message[sb+1+(MAXSW-1)-i]=(char)(48+digitalRead(pinSw[i]));}     
      if(NBSW<MAXSW){for(i=NBSW;i<MAXSW;i++){message[sb+1+(MAXSW-1)-i]='x';}}message[sb+MAXSW+1]='_';
      sb+=MAXSW+2;
      
#define NBDET 0
      message[sb]=(char)(NBDET+48);                                   // nombre détecteurs
      //for(i=(NBDET-1);i>=0;i--){message[sb+1+(NBDET-1)-i]=(char)(chexa[cstRec.memDetec[i]]);}
      if(NBDET<MAXDET){for(i=NBDET;i<MAXDET;i++){message[sb+1+i]='x';}}message[sb+MAXDET+1]='_';                              
      sb+=MAXDET+2;

      
      for(i=0;i<NBPULSE;i++){message[sb+i]='0';} // chexa[staPulse[i]];}
      strcpy(message+sb+NBPULSE,"_\0");                               // clock pulse status          - 5

      sb+=NBPULSE+1;
      memcpy(message+sb,model,LENMODEL);
      strcpy(message+sb+LENMODEL,"_\0");                                                     //      - 7
      sb+=LENMODEL+1;
      
      uint32_t currt;
      for(i=0;i<NBPULSE*2;i++){                                                              // loop compteurs (4*2)
        currt=0;
        byte* pcurr=(byte*)&currt;
        //if(cstRec.cntPulseOne[i]!=0){currt=(millis()-cstRec.cntPulseOne[i])/1000;}
        //Serial.print(i);Serial.print(" écoulé=");Serial.print(currt);Serial.print(" ");dumpfield((char*)pcurr,16);Serial.println();     // temps écoulé
        for(int j=0;j<sizeof(uint32_t);j++){conv_htoa((char*)(message+sb+2*(i*sizeof(uint32_t)+j)),(byte*)(pcurr+j));}                        // loop bytes (4/8)
      }
      sb+=NBPULSE*2*sizeof(uint32_t)*2+1;
      strcpy(message+sb-1,"_\0");


if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL);}

char fonctName[]={"data__Read_"};
  if(tableC[numT].numPeri!=0){
    memcpy(fonctName,"data_Save_",8);}
  buildMess(fonctName,message,"");
Serial.println("bufServer=");Serial.println(bufServer);
//  return messToServer(&cli,host,port,bufServer); 
}

/* user functions */


#endif // NRF_MODE == 'C'
