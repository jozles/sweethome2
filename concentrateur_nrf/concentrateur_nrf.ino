
#include <nRF24L01.h>
#include "nrf24l01p.h"
#include "conc_nrf_const.h"


Nrfp nrfp;

#if NRF_MODE == 'C'
#include "table_conc_nrf.h"
struct NrfConTable tableC[NBPI];
#endif NRF_MODE == 'C'

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_beg;
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];
uint8_t pipe;
uint8_t pldLength;
byte    ccPipe[5];         // adresse pipe du concentrateur après inscription
bool    confSta=false;     // pour 'P' true si inscription ok
byte    nulAddr[]={0,0,0,0,0};   

long readTo=0;


/* prototypes */

void timeStat(unsigned long* timeS0,unsigned long* timeS,byte* stat,char* lib);
void hprint(byte* stat);
char getch();

#if NRF_MODE == 'C'
  void pingpong();
  void broadcast();
#endif NRF_MODE == C
 
void setup() {
  
  Serial.begin(115200);

  nrfp.setNum(1,BALISE);    // numéro circuit courant, numéro balise
  
  nrfp.ce_pin[0]=8;
  nrfp.ce_pin[1]=9;
  nrfp.csn_pin[0]=7;
  nrfp.csn_pin[1]=10;    
  
  nrfp.hardInit();
  
  nrfp.setNum(1,BALISE);    // numéro circuit courant, numéro balise
                            // si 'P' sans effet
  
  nrfp.channel=1; 

  nrfp.setMode(NRF_MODE);
 
  nrfp.r1_addr=(byte*)R1_ADDR;  // P->MAC ou C->base 
  nrfp.pi_addr=(byte*)ccPipe;   // adresse pipe après inscription péri
  nrfp.br_addr=(byte*)BR_ADDR;  // adresse commune de broadcast  
  nrfp.cc_addr=(byte*)CC_ADDR;  // adresse commune concentrateur    

  confSta=nrfp.config();

#if NRF_MODE == 'P'
  char* kk[2]; 
  kk[0]="ko";kk[1]="ok";
  Serial.print("start ");Serial.println(kk[confSta]);
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  tableCInit();tableCPrint();

  bool menu=true;
  while(1){
    if(menu){
      Serial.println("start p=pingpong b=broadcast ");
      menu=false;
    }

/* gestion inscriptions */
    
    if(nrfp.available()){
      memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
      nrfp.dataRead(message,pipe,pldLength);
      Serial.print("received ");Serial.print((char*)message);Serial.print(" l=");Serial.print(pldLength);Serial.print(" p=");Serial.println(pipe);
      if(pipe==0){  // demande d'inscription
        int i=0;    // remplacer par la recherche d'emplacement libre ou déjà existant
        memcpy(tableC[i].periMac,message,ADDR_LENGTH);
        nrfp.dataWrite(0,tableC[i].pipeAddr,'A',ADDR_LENGTH,tableC[i].periMac); // envoi à periMac de la pipeAddr qui lui est attribuée
        int trst=1;
        while(trst==1){trst=nrfp.transmitting();}
        if(trst<0){memset(tableC[i].periMac,0x00,ADDR_LENGTH);} // MAX_RT -> effacement table ; la pi_addr sera effacée
                                                                // pour non réponse à la première tentative de TX
      }
    }

/*************************/
    
    char a=getch();
    switch(a){
      case 'p':pingpong();menu=true;break;
      case 'b':broadcast();menu=true;break;
      default:break;
    }
  }
#endif NRF_MODE == 'C'
}

void loop() {
#if NRF_MODE == 'P'

  while(!nrfp.available()){}
  
  Serial.print("received ");delay(1000);
  
  nrfp.dataRead(message,&pipe,&pldLength);
  Serial.print((char*)message);
  Serial.print("p/l:");
  Serial.print(pipe);
  Serial.print("/");
  Serial.print(pldLength);
  
  if(pipe!=BR_PIPE){                              // sinon message de balise
    nrfp.dataWrite(1,message,'A',pldLength,nulAddr);
    Serial.println(" transmit...");
  
    while(nrfp.transmitting()){}
  }
#endif
} 

void pingpong()
{
  while (getch()!=(char)27){
        
    
    cnt++;
    sprintf(message,"%05d",cnt);
    message[5]='*';
    time_beg = micros();

    nrfp.dataWrite(1,message,'A',MAX_PAYLOAD_LENGTH,nulAddr);

    Serial.print((char*)message);

    int trst=1;
    while(trst>0){trst=nrfp.transmitting();

      switch(trst){
        case 0:Serial.print("...transmitted...");
                readTo=0;
                while(!nrfp.available() && (readTo>=0)){
                  readTo=TO_AVAILABLE-micros()+time_beg;}
                if(readTo<0){cntko++;Serial.print(" time out (");Serial.print(cntko);Serial.println(")");}
                else{
                  pldLength=MAX_PAYLOAD_LENGTH;
                  nrfp.dataRead(message,&pipe,&pldLength);  
                  time_end=micros();
                  Serial.print(" received ");  
                  Serial.print((char*)message);
                  Serial.print(" p/l:");Serial.print(pipe);Serial.print("/");Serial.print(pldLength);
                  Serial.print(" in:");
                  Serial.print(time_end - time_beg); 
                  Serial.println("us");
                }break;
        case -1:Serial.println("...MAX_RT...");break;
        default:break;
      }
    }
    delay(2000);
  } 
}


char getch()
{
  if(Serial.available()){
    return Serial.read();
  }
  return 0;
}

void broadcast()
{
  time_beg = micros();
  byte regw;
  
  nrfp.regWrite(TX_ADDR,nrfp.br_addr,ADDR_LENGTH);
  
  cnt++;
  sprintf(message,"%05d",cnt);
  message[strlen(message)+1]='\0';  
  message[strlen(message)]='+';
  
  nrfp.dataWrite(1,message,'N',6,nulAddr);
  
  Serial.print((char*)message);
  
  while(nrfp.transmitting()){}

  time_end=micros();
  Serial.print(" transmitted ");  
  Serial.print((char*)message);Serial.print(" in:");
  Serial.print(time_end - time_beg); 
  Serial.println("us");

}

