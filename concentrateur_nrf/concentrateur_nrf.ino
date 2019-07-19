#include "nRF24L01.h"
#include "nrf24l01p.h"
#include "conc_nrf_const.h"

#if  NRF_MODE == 'P'
#include <avr/sleep.h>
#include <avr/power.h>
#ifdef DS18X20
#include <ds18x20.h>
#endif DS18X20
#endif NRF_MODE == 'P'

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
#define TO_GDS 1000
int     gdSta=0;
long    readTo=0;
char*   kk={"rt\0le\0pi\0TO\0ok\0"}; // erreurs en transmission (maxrt, length, pipe)

#if NRF_MODE == 'P'
#define RDY_TO 120000
long    timeRdy=-RDY_TO;        // pour message de présence
bool    reveil=true;

int     awakeCnt=0;
int     awakeMinCnt=0;
int     retryCnt=0;
#define AWAKE_OK_VALUE  5 //15   // 120 sec entre chaque test de temp
#define AWAKE_MIN_VALUE 15 //111  // environ 15 min pour message minimum de présence
#define AWAKE_KO_VALUE  450  // 1 heure avant prochain test si com HS
#define RETRY_VALUE     3    // nbre de retry avant KO

#ifdef DS18X20
Ds1820 ds1820;
float   temp,previousTemp,deltaTemp; 
bool    dsSta=false;
byte    setds[]={0,0x7f,0x80,0x3f},readds[8];   // 187mS 10 bits accu 0,25°
char    dsM;
#define TCONVDS 200     // mS !!
#endif DS18X20 

float   volts=0;

#endif NRF_MODE == 'P'

/* prototypes */

void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
void printAddr(char* addr);
int  txMessage(char* message,char ack,uint8_t len,byte* addr);
#if NRF_MODE == 'C'
  char getch();
  void echo();
  void broadcast();
#endif NRF_MODE == 'C'
 
#if NRF_MODE == 'P'
void wdtSetup();

void hardwarePowerUp()
{
  nrfp.powerUp();
  pinMode(LED,OUTPUT);
}

void hardwarePowerDown()
{
  nrfp.powerDown();
  pinMode(LED,INPUT);
}


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

  nrfp.setup(NRF_MODE,CE_PIN,CSN_PIN,NB_CIRCUIT,CHANNEL,RF_SPEED,(ARD-1)*16+ARC,(byte*)BR_ADDR,(byte*)CC_ADDR,(byte*)R0_ADDR,(byte*)&ccPipe); // doit être la première fonction

  nrfp.setNum(circuit,BALISE);  // numéro circuit courant, numéro balise
                                // si 'P' sans effet

#if NRF_MODE == 'P'
  hardwarePowerUp();                                
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}
  
  nrfp.confCircuit();
  beginP();
  
#ifdef DS18X20  
  dsSta=ds1820.setDs(WPIN,setds,readds);   // setup ds18b20
  dsM='B';if(ds1820.dsmodel==MODEL_S){dsM='S';}
#endif DS18X20
  Serial.println("end setup");delay(1);
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  pinMode(LED,OUTPUT);
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}

  tableCInit();//tableCPrint();
  nrfp.powerUp();
  nrfp.confCircuit();
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
        showRx();
        if(pipe==0){                           // register request
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

  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){
    awakeCnt--;
    awakeMinCnt--;
    pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delayMicroseconds(500);digitalWrite(LED,LOW); // 1mA rc=0,5/8000 -> 62nA
    sleepPwrDown(8000);
    Serial.print("awaken... ");Serial.println(awakeCnt);delay(10);
  }
  Serial.println("awaken...... ");delay(10);
  awakeCnt=AWAKE_OK_VALUE;

  /*****  Here are data acquisition/checking to determine if nfr should be powered up *****
          If not -> sleepPwrDown() 
  */

#ifdef DS18X20
  if(retryCnt==0){
    ds1820.convertDs(WPIN);
    
#if TCONVDS != 200
    tconv // TCONVDS not 200 ... adjust sleep time
#endif
    Serial.println("awaken........ ");delay(10);
    sleepPwrDown(250); 
    Serial.println("awaken......... ");delay(10);

    temp=ds1820.readDs(WPIN);Serial.print(temp);Serial.print(" ");Serial.print(millis());Serial.print(" ");Serial.print(awakeCnt);Serial.print(" ");Serial.print(awakeMinCnt);Serial.print(" ");Serial.println(retryCnt);
  }
#endif DS18X20  
  if((temp>(previousTemp+deltaTemp))||(temp<(previousTemp-deltaTemp))||(awakeMinCnt<0)||(retryCnt!=0)){

    hardwarePowerUp();                   // 5mS delay inside
    Serial.println("awaken............. ");delay(10);
    uint8_t outLength=0;
    char    outMessage[MAX_PAYLOAD_LENGTH+1];
    int     trst;     // status retour txMessage
    int     gdSt=0;   // status retour read

    /* building message VVVVXmmmmmmmm+TT.TTV.VV*/
    memcpy(outMessage,VERSION,LENVERSION);
    outMessage[LENVERSION]=dsM;
    outLength=LENVERSION+1;
    sprintf(outMessage+outLength,"%08ld",millis());
    outLength+=8;
    outMessage[outLength]='+';if(temp<0){outMessage[outLength]='-';}
    outLength+=1;
    dtostrf(temp,5,2,outMessage+outLength);                               
    if((strstr(outMessage,"nan")!=0) || !dsSta){strcpy((char*)(outMessage+outLength),"+00.00\0");}
    outLength+=5;
    // get voltage and store it
    dtostrf(volts,4,2,outMessage+outLength);
    outLength+=4;
    outMessage[outLength]='\0';

    /* sending message */
    trst=txMessage(outMessage,'A',outLength,ccPipe);
    if(trst<0){
      // si le message ne part pas, essai aux (RETRY_VALUE-1) prochains réveils 
      // puis après AWAKE_KO_VALUE réveils
      Serial.print((char*)outMessage);Serial.println(" ********** maxRT ");
      switch(retryCnt){
        case 0:retryCnt=RETRY_VALUE;break;
        case 1:awakeCnt=AWAKE_KO_VALUE;awakeMinCnt=AWAKE_KO_VALUE;awakeCnt=0;break;
        default:retryCnt--;break;
      }
    }
    else{           // transmit ok -> every counters reset
      retryCnt=0;
      awakeCnt=AWAKE_OK_VALUE;
      awakeMinCnt=AWAKE_MIN_VALUE;
      previousTemp=temp;
    }
  }
  delay(1);  // pour serial
/*  
    pldLength=MAX_PAYLOAD_LENGTH;

  gdSt=nrfp.read(message,&pipe,&pldLength);
  ledblk(TBLK,DBLK,IBLK,2);

  switch(gdSt){
    case 0: if((millis()-timeRdy)>RDY_TO){                          // envoi présence ?
              timeRdy=millis();
              sprintf(outMessage,"%08ld",millis());
              outLength=8;
              #ifdef DS18X20    
                ds1820.convertDs(WPIN);
                delay(TCONVDS);
                temp=ds1820.readDs(WPIN);Serial.println(temp);
                outMessage[outLength]='+';if(temp<0){outMessage[outLength]='-';}
                dtostrf(temp,5,2,outMessage+outLength+1);                               
                if((strstr(outMessage,"nan")!=0) || !dsSta){strcpy((char*)(outMessage+outLength),"+00.00\0");}
                outLength+=6;
              #endif DS18X20
              trst=txMessage(outMessage,'A',outLength,ccPipe);
              if(trst<0){Serial.print((char*)outMessage);Serial.println(" ********** maxRT ");}
            }break;
    case 1: Serial.print(" rx ");                                   // reçu message ok
            Serial.print((char*)message);
            Serial.print(" p=");Serial.print(pipe);
            Serial.print(" l=");Serial.print(pldLength);
            Serial.print(" transmit to ");printAddr(ccPipe);
            trst=txMessage(message,'A',pldLength,ccPipe);       // echo - le peri parle toujours à ccPipe
            if(trst<0){Serial.println("erreur MAX-RT");}
            else{Serial.println("..ok..");}  
            break;
    case -1:Serial.print(" erreur reception ");Serial.println((char*)(kk+(gdSt+3)*3));
            break;
    default:break;
  }
  }*/
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
  trSta=txMessage(message,'A',len,addr);

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
      showRx();
    }
    else{Serial.print("error ");Serial.print((char*)kk+(gdSta+3)*3);} 
  }
  Serial.print(" in:");Serial.print((long)(time_end - time_beg));Serial.println("us");
}

void showRx()
{
  uint8_t numP=circuit*NB_PIPE+pipe;
  char pipeAd[ADDR_LENGTH];
  #if NRF_MODE=='C'
  memcpy(pipeAd,tableC[numP].pipeAddr,ADDR_LENGTH);
  #endif NRF_MODE=='C'
  #if NRF_MODE=='P'
  switch(pipe){
    case 1:memcpy(pipeAd,R0_ADDR,ADDR_LENGTH);break;
    case 0:memcpy(pipeAd,BR_ADDR,ADDR_LENGTH);break;
    default:memcpy(pipeAd,"*err*",ADDR_LENGTH);break;
  }
  #endif NRF_MODE=='P'  
  Serial.print((char*)message);
  Serial.print(" received on ");printAddr(pipeAd);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" numP=");Serial.print(numP);
}

#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'

void wdtSetup(uint16_t durat)  // durat=0 for external wdt on INT0
{ 
// datasheet page 54, Watchdog Timer.
   
    noInterrupts();

/*  MCUSR MCU status register (reset sources)(every bit cleared by writing 0 in it)
 *   WDRF reset effectué par WDT
 *   BORF ------------------ brown out detector
 *   EXTRF ----------------- pin reset
 *   PORF ------------------ power ON
*/
  MCUSR &= ~(1<<WDRF);  // pour autoriser WDE=0
   
/*  WDTCSR watchdog timer control
 *   WDIF watchdog interrupt flag (set when int occurs with wdt configured for) (reset byu writing 1 or executing ISR(WDT_vect))
 *   WDIE watchdog interrupt enable 
 *   WDE  watchdog reset enable
 *        WDE  WDIE   Mode
 *         0    0     stop
 *         0    1     interrupt
 *         1    0     reset
 *         1    1     interrupt then reset (WDIE->0 lors de l'interruption, retour au mode reset)
 *       !!!! le fuse WDTON force le mode reset si 0 !!!!
 *   WDCE watchdog change enable (enable writing 0 to WDE and modif prescaler) (auto cleared after 4 cycles)
 *   WDP[3:0] prescaler 2^(0-9)*2048 divisions de l'oscillateur WDT (f=128KHz p*2048=16mS) 
 *   
 *   l'instrction wdr reset le timer (wdt_reset();)
 */

    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3);   // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s
                                    
  if(durat==8000){
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3);}   // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s
  else if(durat==250){
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants                                             
    WDTCSR = (1<<WDIE) | (1<<WDP0);}
  else if(durat==16){
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | (1<<WDP0);}

  interrupts();
Serial.println("wdtsetup");delay(10);

}

void sleepPwrDown(uint16_t durat) {

Serial.println("sleepPowerDown");delay(10); 
    hardwarePowerDown();
Serial.println("hardware down");delay(10);     
    wdtSetup(durat);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    power_adc_disable();
    sleep_enable();
Serial.println("sleep_enable");delay(10); 
    sleep_mode();
Serial.println("awaken");delay(10);   
    sleep_disable();
    power_all_enable();
    //hardwarePowerUp();
}

ISR(WDT_vect)                      // ISR interrupt service pour vecteurs d'IT du MPU (ici vecteur WDT)
{
  reveil = true;
}

#endif NRF_MODE == 'P'

int txMessage(char* message,char ack,uint8_t len,byte* addr)
{
#if NRF_MODE=='P'  
  if(*ccPipe=='0'){beginP();}
#endif NRF_MODE=='P'
  
  time_beg = micros();
  
  nrfp.write(message,ack,len+1,addr);

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
/*

void dumpstr0(char* data,uint8_t len)
{
    char* chexa="0123456789ABCDEFabcdef\0";
    char a[]={0x00,0x00,0x00};
    uint8_t c;
    Serial.print("   ");Serial.print((long)data,HEX);Serial.print("   ");
    for(int k=0;k<len;k++){Serial.print(chexa[(data[k]>>4)&0x0f]);Serial.print(chexa[data[k]&0x0f]);Serial.print(" ");}
    Serial.print("    ");
    for(int k=0;k<len;k++){
            c=data[k];
            if(c<32 || c>127){c='.';}
            Serial.print((char)c);
    }
    Serial.println();
}

void dumpstr(char* data,uint16_t len)
{
    while(len>=16){len-=16;dumpstr0(data,16);data+=16;}
    if(len!=0){dumpstr0(data,len);}
}

void dumpfield(char* fd,uint8_t ll)
{
    for(int ff=ll-1;ff>=0;ff--){if((fd[ff]&0xF0)==0){Serial.print("0");}Serial.print(fd[ff],HEX);}
    Serial.print(" ");
}
*/
