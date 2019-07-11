
#include <nRF24L01.h>
#include "nrf24l01p.h"
#include "conc_nrf_const.h"


Nrfp nrfp;

#if NRF_MODE == 'C'
#include "table_conc_nrf.h"
struct NrfConTable tableC[NBPERIF];
#endif NRF_MODE == 'C'

#define LED 5
uint16_t blkdelay=0;
long blktime=0;
uint8_t bcnt=0;
#define TBLK 1
#define DBLK 2000
#define IBLK 80

unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];
uint8_t pipe;
uint8_t pldLength;
byte    ccPipe[]={"00000\0"};   // adresse pipe du concentrateur après inscription
bool    confSta=false;          // pour 'P' true si inscription ok


long readTo=0;

char* kk={"ko\0ok\0"}; 

/* prototypes */

char getch();
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);

#if NRF_MODE == 'C'
  void echo();
  void broadcast();
#endif NRF_MODE == 'C'
 

void setup() {
  
  Serial.begin(115200);

  nrfp.setup(NRF_MODE,CE_PIN,CSN_PIN,NB_CIRCUIT,CHANNEL,(byte*)BR_ADDR,(byte*)CC_ADDR,(byte*)R0_ADDR,(byte*)&ccPipe); // doit être la première fonction

  nrfp.setNum(0,BALISE);  // numéro circuit courant, numéro balise
                          // si 'P' sans effet
  nrfp.start();
  
  pinMode(LED,OUTPUT);
  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}

#if NRF_MODE == 'P'
  
  confSta=false;
  while(!confSta){
    confSta=nrfp.begin();
    Serial.print("start ");
    Serial.print((char*)(kk+confSta*3));
    Serial.print(" ccPipe=");Serial.println((char*)ccPipe);
    
    ledblk(TBLK,DBLK,IBLK,1);
  }

#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'
  
  tableCInit();//tableCPrint();

  nrfp.begin();
  
  Serial.println("start");
  
  bool menu=true;
  while(1){
    if(menu){
      Serial.println("(e)cho (b)roadcast (t)ableC (q)uit");
      menu=false;
    }

  ledblk(TBLK,2000,IBLK,1);

/* gestion inscriptions */
    
      memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
      pldLength=MAX_PAYLOAD_LENGTH;                   // max length
      
      if(nrfp.getData(message,&pipe,&pldLength)==1){  // data ready, no error
        Serial.print("received ");Serial.print((char*)message);Serial.print(" l=");Serial.print(pldLength);Serial.print(" p=");Serial.print(pipe);
        if(message[0]=='p' && pipe==0){               // register request
        //if(numNrf==BALISE && pipe==0){             
     
          uint8_t i,k=0,z=0;    // search free line or existing macAddr
          for(i=1;i<NBPERIF;i++){ 
            if(memcmp(tableC[i].periMac,message,ADDR_LENGTH)==0){k=i;break;}      // already existing
            else if(memcmp(tableC[i].periMac,"0",1)==0 && z==0){z=i;}             // store free line
          }
          
          if(k==0 && z!=0){
            i=z;                                                                  // i = free line 
            memcpy(tableC[i].periMac,message,ADDR_LENGTH);}                       // record macAddr         
          if(k!=0 or z!=0){  
            nrfp.write(tableC[i].pipeAddr,'A',ADDR_LENGTH,tableC[i].periMac);     // send pipeAddr to peri(macAddr)
            int trst=1;
            while(trst==1){trst=nrfp.transmitting();}
            if(trst<0){memset(tableC[i].periMac,'0',ADDR_LENGTH);i=NBPERIF+2;}   // MAX_RT -> effacement table ; la pi_addr sera effacée
                                                                                  // pour non réponse à la première tentative de TX
          }
          if(i!=0 && i<(NBPERIF+2)){
            Serial.print(" registred on addr ");printAddr(tableC[i].pipeAddr,'n');}
          else if(i==(NBPERIF+2)){Serial.print(" MAX_RT ... deleted");}
          else {Serial.print(" full");}
        }
        else{};  // pas demande d'inscription... à traiter
        Serial.println();
      }
      else{}  // no data or error

/*************************/
    
    char a=getch();
    switch(a){
      case 'e':echo();menu=true;break;
      case 'b':broadcast();menu=true;break;
      case 't':tableCPrint();menu=true;break;
      default:break;
    }
  }               // menu loop
#endif NRF_MODE == 'C'
}

void loop() {

#if NRF_MODE == 'P'

  while(nrfp.getData(message,&pipe,&pldLength)!=1){ledblk(TBLK,DBLK,IBLK,2);}
  Serial.print("rx ");
  pldLength=MAX_PAYLOAD_LENGTH;
  nrfp.write(message,'A',pldLength,ccPipe);
  Serial.print((char*)message); 
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.println(" transmit...");
  
  while(nrfp.transmitting()){}
  
#endif
} 

#if NRF_MODE == 'C'

void echo()
{
  int cnt=0;
  int cntko=0;
  
  while (getch()!='q'){
        
    uint8_t numP=1;
    cnt++;
    sprintf(message,"%05d",cnt);
    message[5]='*';
    time_beg = micros();

    Serial.print((char*)message);Serial.print(" to -> ");printAddr(tableC[numP].periMac,' ');
    
    nrfp.write(message,'A',MAX_PAYLOAD_LENGTH,tableC[numP].periMac);
    Serial.print("..w..");

    int gdSta=0;
    int trst=1;
    while(trst>0){trst=nrfp.transmitting();if(getch()=='q'){return;}}

    switch(trst){
      case 0:Serial.print("...transmitted...");
        readTo=millis();
        pldLength=MAX_PAYLOAD_LENGTH;
        while(gdSta!=1 && (millis()-readTo<TO_AVAILABLE)){
          gdSta=nrfp.getData(message,&pipe,&pldLength);if(getch()=='q'){return;}}
        if(millis()-readTo<TO_AVAILABLE){cntko++;Serial.print("...TO...(");Serial.print(cntko);Serial.println(")");}
        else if(gdSta==-1){cntko++;Serial.print("...pipe or length error...(");Serial.print(cntko);Serial.println(")");}
        else{
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
  
  sprintf(message,"%08d",time_beg);
  message[strlen(message)+1]='\0';  
  message[strlen(message)]='+';
  
  nrfp.write(message,'N',9,BR_ADDR);
  
  Serial.print((char*)message);
  
  while(nrfp.transmitting()){}

  time_end=micros();
  Serial.print(" transmitted ");  
  Serial.print((char*)message);Serial.print(" in:");
  Serial.print(time_end - time_beg); 
  Serial.println("us");

}

#endif NRF_MODE == 'C'

void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb)
{
  if((millis()-blktime)>blkdelay){
    if(digitalRead(LED)==LOW){
      digitalWrite(LED,HIGH);blkdelay=dur;}
    else{digitalWrite(LED,LOW);
      if(bcnt<bnb){blkdelay=bint;bcnt++;}
      else{blkdelay=bdelay;bcnt=1;}}
    blktime=millis();
  }
}

