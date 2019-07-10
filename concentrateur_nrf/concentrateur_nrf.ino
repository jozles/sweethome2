
#include <nRF24L01.h>
#include "nrf24l01p.h"
#include "conc_nrf_const.h"


Nrfp nrfp;

#if NRF_MODE == 'C'
#include "table_conc_nrf.h"
struct NrfConTable tableC[NBPERIF];
#endif NRF_MODE == 'C'

#define LED 5
long blkdelay=0;
long blktime=0;
uint8_t bcnt=0;


unsigned long time_beg;
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
void ledblk(long dur,long bdelay,long bint,uint8_t bnb);

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

#if NRF_MODE == 'P'
  
  pinMode(LED,OUTPUT);
  ledblk(30,1000,60,4);
  
  confSta=false;
  while(!confSta){
    confSta=nrfp.begin();
    Serial.print("start ");
    Serial.print((char*)(kk+confSta*3));
    Serial.print(" ccPipe=");Serial.println((char*)ccPipe);
    
    ledblk(30,1000,60,1);
    delay(1950);
  }

#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'
  
  tableCInit();//tableCPrint();

  long blktime=millis();pinMode(LED_BUILTIN,OUTPUT);

  nrfp.begin();
  
  bool menu=true;
  while(1){
    if(menu){
      Serial.println("start (e)cho (b)roadcast (t)ableC (q)uit");
      menu=false;
    }

/* gestion inscriptions */
    
    if(nrfp.available()){
    
      memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
      pldLength=MAX_PAYLOAD_LENGTH;                  // max length
      if(nrfp.read(message,&pipe,&pldLength)){   // true=>no error
        Serial.print("received ");Serial.print((char*)message);Serial.print(" l=");Serial.print(pldLength);Serial.print(" p=");Serial.print(pipe);
        if(message[0]=='p' && pipe==0){              // register request
        //if(numNrf==BALISE && pipe==0){             
     
          uint8_t i,k=NBPERIF;    // search free line or existing macAddr
          for(i=1;i<NBPERIF;i++){ 
            if(memcmp(tableC[i].periMac,message,ADDR_LENGTH)==0){break;}          // already existing
            else if(k>=NBPERIF && tableC[i].periMac[0] == '0'){k=i;}              // store free line
          }
          if(i>=NBPERIF && k<NBPERIF){i=k;}                                       // i = free line 
          if(i<NBPERIF){  
          
            memcpy(tableC[i].periMac,message,ADDR_LENGTH);                        // record macAddr 
            Serial.print(" registred on addr ");printAddr(tableC[i].periMac,'n');        
            nrfp.write(tableC[i].pipeAddr,'A',ADDR_LENGTH,tableC[i].periMac); // send pipeAddr to peri(macAddr)
            int trst=1;
            while(trst==1){trst=nrfp.transmitting();}
            if(trst<0){memset(tableC[i].periMac,0x00,ADDR_LENGTH);} // MAX_RT -> effacement table ; la pi_addr sera effacée
                                                                  // pour non réponse à la première tentative de TX
          }
          else Serial.print(" full");
        }
        else{};  // pas demande d'inscription... à traiter
        Serial.println();
      }          // read error
    }            // unavailable

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

  while(!nrfp.available()){
      ledblk(30,1000,60,2);
  }
  
  Serial.print("received ");

  nrfp.read(message,&pipe,&pldLength);
  Serial.print("rx ");
  //if(pipe!=BR_PIPE){                              // sinon message de balise
    pldLength=MAX_PAYLOAD_LENGTH;
    nrfp.write(message,'A',pldLength,ccPipe);
  Serial.print((char*)message);
  //Serial.print(" c=");Serial.print(numNrf);  
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" l=");Serial.print(pldLength);

    
    Serial.println(" transmit...");

  
    while(nrfp.transmitting()){}
  //}
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
    int trst=1;
    while(trst>0){trst=nrfp.transmitting();}

      switch(trst){
        case 0:Serial.print("...transmitted...");
                readTo=millis();
                while(!nrfp.available() && (millis()-readTo<TO_AVAILABLE)){}
                  if(millis()-readTo<TO_AVAILABLE){cntko++;Serial.print(" time out (");Serial.print(cntko);Serial.println(")");}
                  else{
                    pldLength=MAX_PAYLOAD_LENGTH;
                    nrfp.read(message,&pipe,&pldLength);  
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

void ledblk(long dur,long bdelay,long bint,uint8_t bnb)
{
  if((millis()-blktime)>blkdelay){
    digitalWrite(LED,HIGH);delay(30);digitalWrite(LED,LOW);
    blktime=millis();
    if(blkdelay==bdelay){blkdelay=bint;bcnt=0;}else{bcnt++;}
    if(bcnt>bnb){blkdelay=bdelay;}
  }
}

