
#include "nRF24L01.h"
#include "nrf24l01p.h"
#include "conc_nrf_const.h"
#ifdef DS18X20
#include <ds18x20.h>
#endif DS18X20

Nrfp nrfp;

#if NRF_MODE == 'C'
#include "table_conc_nrf.h"
struct NrfConTable tableC[NBPERIF];
#endif NRF_MODE == 'C'


uint16_t blkdelay=0;
long blktime=0;
uint8_t bcnt=0;
#define TBLK 1
#define DBLK 2000
#define IBLK 80

unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];
uint8_t circuit=CIRCUIT;
uint8_t pipe;
uint8_t pldLength;
byte    ccPipe[]={"00000\0"};   // adresse pipe du concentrateur après inscription
bool    confSta=false;          // pour 'P' true si inscription ok
byte    sta;
int     gdSta=0;
long    readTo=0;
#define TO_GDS 1000

#ifdef DS18X20
Ds1820 ds1820;
float   temp; 
bool    dsSta=false;
byte    setds[]={0,0x7f,0x80,0x3f},readds[8];   // 187mS 10 bits accu 0,25°
#define TCONVDS 200
#endif DS18X20 

char* kk={"rt\0le\0pi\0TO\0ok\0"}; 

/* prototypes */

char getch();
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
void printAddr(char* addr);
int  txMessage(char* message,uint8_t len,byte* addr);
#if NRF_MODE == 'C'
  void echo();
  void broadcast();
#endif NRF_MODE == 'C'
 
#if NRF_MODE == 'P'
void beginP()
{
  confSta=false;
  while(!confSta){
    confSta=nrfp.begin(); // -3 max RT ; -2 len ; -1 pipe ; 0 na ; 1 ok
    Serial.print("start ");
    Serial.print((char*)(kk+(confSta+3)*3));
    Serial.print(" ccPipe=");Serial.println((char*)ccPipe);
    
    delayBlk(TBLK,DBLK,IBLK,1,1000);
  }
}
#endif NRF_MODE == 'P'

void setup() {
  
  Serial.begin(115200);

#ifdef DS18X20  
  dsSta=ds1820.setDs(WPIN,setds,readds);   // setup ds18b20
#endif DS18X20

  nrfp.setup(NRF_MODE,CE_PIN,CSN_PIN,NB_CIRCUIT,CHANNEL,RF_SPEED,(ARD-1)*16+ARC,(byte*)BR_ADDR,(byte*)CC_ADDR,(byte*)R0_ADDR,(byte*)&ccPipe); // doit être la première fonction

  nrfp.setNum(circuit,BALISE);  // numéro circuit courant, numéro balise
                                // si 'P' sans effet
  nrfp.confCircuit();
  
  pinMode(LED,OUTPUT);
  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}

#if NRF_MODE == 'P'
  beginP();
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
      
      if(nrfp.read(message,&pipe,&pldLength)==1){  // data ready, no error
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
            if(trst<0){memset(tableC[i].periMac,'0',ADDR_LENGTH);i=NBPERIF+2;}    // MAX_RT -> effacement table ; la pi_addr sera effacée
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
      case 'e':echo(a);menu=true;break;
      case 'b':broadcast(a);menu=true;break;
      case 't':Serial.println((char)a);tableCPrint();menu=true;break;
      default:break;
    }
  }               // menu loop
#endif NRF_MODE == 'C'
}

void loop() {

#if NRF_MODE == 'P'

#define TR_TO  1000
#define RDY_TO 3600000
pldLength=MAX_PAYLOAD_LENGTH;
long tr_beg;
long timeRdy=0;   // force message
int  trst;
int  gdSt=0;

  if(*ccPipe=='0'){beginP();}

  if((millis()-timeRdy)>RDY_TO){
    
    sprintf(message,"%08ld",millis());
    strcat(message+8,"ready");

#ifdef DS18X20    
    ds1820.convertDs(WPIN);
    delay(TCONVDS);
    temp=ds1820.readDs(WPIN);
    sprintf(message+13,"%+02.2f",temp/100);                                
    if((strstr(message,"nan")!=0) || !dsSta){strcpy(message+13,"+00.00\0");}
#endif DS18X20

    trst=txMessage(message,19,ccPipe);
    if(trst<0){Serial.print((char*)message);Serial.println(" ********** maxRT ");beginP();}
  }
  
  while(gdSt==0){
    gdSt=nrfp.read(message,&pipe,&pldLength);ledblk(TBLK,DBLK,IBLK,2);}

  if(gdSt==1){
    Serial.print(" rx ");
    Serial.print((char*)message);
  
    nrfp.write(message,'A',pldLength,ccPipe);       // le peri parle toujours à ccPipe
                                                    // si ccPipe=='0' begin() est à refaire
    Serial.print(" p=");Serial.print(pipe);
    Serial.print(" l=");Serial.print(pldLength);
    Serial.print(" transmit to ");
    printAddr(ccPipe);

    tr_beg=millis();
    trst=1;
    while((trst>0) && ((millis()-tr_beg)<TR_TO)){trst=nrfp.transmitting();}
    if((millis()-tr_beg)>=TR_TO){Serial.println("erreur TO ACK");}
    if(trst<0){Serial.println("erreur MAX-RT");}
    else{Serial.println("..ok..");}  
  }
  else {Serial.print(" erreur reception ");Serial.println((char*)(kk+(gdSt+3)*3));}
  
#endif
} 

#if NRF_MODE == 'C'

void echo(char a)
{
  int cnt=0;
  int cntko=0;

  Serial.println((char)a);
  
  while (getch()!='q'){

    uint8_t numP=1;
    cnt++;
    sprintf(message,"%05d",cnt);
    echo0(message,5,tableC[numP].periMac);      // le concentrateur parle toujours à periMac
                                                // si periMac ne répond plus ... à voir

    delay(2000);ledblk(TBLK,DBLK,IBLK,2);
  } 
}

char getch()
{
  if(Serial.available()){
    return Serial.read();
  }
  return 0;
}

void showSta(uint8_t nb)
{
  Serial.print(nb);
  Serial.print(" >>> sta ");
  nrfp.regRead(7,&sta);
  Serial.println(sta,HEX);
}

void broadcast(char a)
{           
  Serial.println((char)a);
//showSta(1);
 
  sprintf(message,"%08lu",millis());

  echo0(message,8,BR_ADDR);
//showSta(5);

}


void echo0(char* message,uint8_t len,byte* addr)
{ 
  message[len]='+';
  message[len+1]='\0';

  int trSta=-1;
  trSta=txMessage(message,len,addr);

//showSta(2);
  
  if(trSta<0){Serial.print("********** maxRT ");
#if NRF_MODE == 'P'
    beginP();
#endif NRF_MODE == 'P'
  }
  else{
    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    pldLength=MAX_PAYLOAD_LENGTH;                   // max length
    readTo=0;
    gdSta=0;
    long read_beg=millis();
    while((gdSta==0)&& (readTo>=0)){
//showSta(3);    
      gdSta=nrfp.read(message,&pipe,&pldLength);
//showSta(4);
      readTo=TO_GDS-millis()+read_beg;}    
    time_end=micros();
    if(gdSta==1){  // data ready, no error
      uint8_t numP=circuit*NB_PIPE+pipe;
      Serial.print("received on ");printAddr(tableC[numP].pipeAddr);Serial.print(" ");
      Serial.print((char*)message);Serial.print(" l=");
      Serial.print(pldLength);Serial.print(" p=");Serial.print(pipe);
    }
    else{Serial.print("error ");Serial.print((char*)kk+(gdSta+3)*3);} 
  }
  Serial.print(" in:");Serial.print((long)(time_end - time_beg));Serial.println("us");
}
#endif NRF_MODE == 'C'

int txMessage(char* message,uint8_t len,byte* addr)
{
  time_beg = micros();
  
  nrfp.write(message,'A',len+1,addr);

  Serial.print("!");

  int trSta=1;
  while(trSta==1){
    trSta=nrfp.transmitting();}

  time_end=micros();
  Serial.print((char*)message);  
  Serial.print(" transmitted to ");  
  Serial.print((char*)addr);Serial.print(" in:");
  Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");

  return trSta;
}


void delayBlk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb,long dly)
{
  long tt=millis();
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);}
}

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

void printAddr(char* addr)
{
  char paddr[ADDR_LENGTH+1];
  memcpy(paddr,addr,ADDR_LENGTH);
  paddr[ADDR_LENGTH]='\0';
  Serial.print(paddr);Serial.print(" ");
}
