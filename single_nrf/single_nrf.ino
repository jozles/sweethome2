#include "nRF24L01.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "shconst2.h"
#include "shutil2.h"

#if  NRF_MODE == 'P'
#include <avr/sleep.h>
#include <avr/power.h>

#define DS18X20
#ifdef DS18X20
#include <ds18x20.h>
#define WPIN       3          // pin thermomètre
#endif DS18X20
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'
extern struct NrfConTable tableC[NBPERIF];
bool menu=true;
#endif NRF_MODE == 'C'

Nrfp nrfp;

/*** LED ***/
uint16_t blkdelay=0;
long     blktime=0;
uint8_t bcnt=0;
#define TBLK 1
#define DBLK 2000
#define IBLK 80

/*** scratch ***/
#define TO_READ 1000
unsigned long read_beg;
unsigned long readTo;       // compteur TO read()

unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];
int     numP;
uint8_t pipe;
uint8_t pldLength;

int     confSta=0;          // pour 'P' numP si inscription ok
byte    sta;                // status reg 
//int     gdSta=0;            // available() & read() status
#define LMERR 9           
char*   kk={"time out\0tx maxrt\0rx empty\0mac addr\0length  \0pipe nb \0--      \0ok      \0"};         // codes retour et erreur

#if NRF_MODE == 'P'

/*** gestion sleep ***/
bool    reveil; 
int     awakeCnt=0;
int     awakeMinCnt=0;
int     retryCnt=0;
uint32_t nbS=0;                   // nbre sleeps
float   durT=0;                   // temps sleep cumulé (mS/10)
float   tBeg=0;                   // temps total depuis reset (oscillateur local)

#define  STEP_VALUE 8              // nbre sec par step
#define  AWAKE_OK_VALUE  5 //15    // 120 sec entre chaque test de temp
uint16_t aw_ok=AWAKE_OK_VALUE;
#define  AWAKE_MIN_VALUE 15 //111  // environ 15 min pour message minimum de présence
uint16_t aw_min=AWAKE_MIN_VALUE;
#define  AWAKE_KO_VALUE  450       // 1 heure avant prochain test si com HS
uint16_t aw_ko=AWAKE_KO_VALUE;
#define  AWAKE_RETRY_VALUE     3   // nbre de retry avant KO
uint8_t  aw_retry=AWAKE_RETRY_VALUE;

long     perRefr=0;                
uint16_t perTemp=0;
long     pitchTemp=0;

int      sizeRead;
// prescaler WDT
#define T16   0b00000000
#define T32   0b00000001
#define T64   0b00000010
#define T125  0b00000011
#define T250  0b00000100
#define T500  0b00000101
#define T1000 0b00000110
#define T2000 0b00000111
#define T4000 0b00100000
#define T8000 0b00100001

uint16_t wdtTime[]={16,32,64,125,500,1000,2000,4000,8000};   // durées WDT millis

/*** ds18x20 ***/
#ifdef DS18X20
Ds1820 ds1820;
float   temp,previousTemp;
float   deltaTemp=0.25; 
bool    dsSta=false;
byte    setds[]={0,0x7f,0x80,0x3f},readds[8];   // 187mS 10 bits accu 0,25°
char    dsM;
uint32_t nbT=0;         // nbre lectures de temp
#define TCONVDS 200     // mS !!
#endif DS18X20 

/*** volts ***/
float   volts=0;

#endif NRF_MODE == 'P'

/* prototypes */

void showRx();
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
int  txMessage(char* message,char ack,uint8_t len,uint8_t numP);
#if NRF_MODE == 'C'
  char getch();
  void echo();
  void broadcast();
#endif NRF_MODE == 'C'

void hardwarePowerUp()
{
  nrfp.powerUp();
  pinMode(LED,OUTPUT);
}
 
#if NRF_MODE == 'P'
void wdtSetup();

void hardwarePowerDown()
{
  nrfp.powerDown();
  pinMode(LED,INPUT);
}

void beginP()
{
  confSta=-1;
  while(confSta<0){
    confSta=nrfp.pRegister(); // -4 empty -3 max RT ; -2 len ; -1 pipe ; 0 na ; 1 ok
    Serial.print("start ");
    Serial.print((char*)(kk+(confSta+6)*LMERR));//Serial.print((confSta-ER_MAXER)*3);
    Serial.print(" numP=");Serial.println(confSta);
    
    delayBlk(TBLK,DBLK,IBLK,1,1000);
  }
  numP=confSta;
}
#endif NRF_MODE == 'P'

void setup() {
  
  Serial.begin(115200);

#if NRF_MODE == 'P'
  
  hardwarePowerUp();
  nrfp.setup();
                                  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}        // 0,8sec (4 blink)

  beginP();                                 // pRegister()
  
#ifdef DS18X20  
  dsSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
  dsM='B';if(ds1820.dsmodel==MODEL_S){dsM='S';}
#endif DS18X20

#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  hardwarePowerUp();                                
  nrfp.setup();
  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}          // 0,8sec (4 blink)

  nrfp.tableCInit();//nrfp.tableCPrint();
  
#endif NRF_MODE == 'C'

  Serial.println("end setup");delay(1);
}

void loop() {

#if NRF_MODE == 'P'

  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){
    awakeCnt--;
    awakeMinCnt--;
    pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delayMicroseconds(500);digitalWrite(LED,LOW); // 1mA rc=0,5/8000 -> 62nA
    sleepPwrDown(T8000);
  }

  awakeCnt=aw_ok;

  /*****  Here are data acquisition/checking to determine if nfr should be powered up *****
          If not -> sleepPwrDown() 
  */

#ifdef DS18X20
  if(retryCnt==0){            // pas de conversion si retry en cours
    ds1820.convertDs(WPIN);
    
#if TCONVDS != 200
    tconv // TCONVDS not 200 ... should adjust sleep time
#endif

    sleepPwrDown(T250); 

    nbT++;
    temp=ds1820.readDs(WPIN);
    Serial.print(temp);Serial.print(" ");
    Serial.print(nbT);Serial.print("/");
  }
#endif DS18X20  

    tBeg=((float)millis()/1000)+(durT/100);
    Serial.print(nbS);Serial.print("(");Serial.print(tBeg);Serial.print(") ");
    Serial.print(awakeCnt);Serial.print(" ");
    Serial.print(awakeMinCnt);Serial.print(" ");Serial.println(retryCnt);

  // temp change or presence message to send -> send
  if((temp>(previousTemp+deltaTemp))||(temp<(previousTemp-deltaTemp))||(awakeMinCnt<0)||(retryCnt!=0)){

    hardwarePowerUp();                   // 5mS delay inside

    uint8_t outLength=0;
    char    outMessage[MAX_PAYLOAD_LENGTH+1];
    int     trst;     // status retour txMessage
    int     gdSt=0;   // status retour read

    /* building message VVVVXmmmmmmmm+TT.TTV.VV*/
    memcpy(outMessage+outLength,MAC_ADDR,ADDR_LENGTH);                     // macAddr
    outLength+=ADDR_LENGTH;
    outMessage[outLength]=numP+48;                                         // numP du périphérique
    outLength++;
    memcpy(outMessage+outLength,VERSION,LENVERSION);                      // version
    outMessage[outLength+LENVERSION]=dsM;                                 // version ds18x20
    outLength+=LENVERSION+1;
    sprintf(outMessage+outLength,"%08d",(uint32_t)tBeg);                  // seconds since last reset 
    outLength+=8;
    outMessage[outLength]='+';if(temp<0){outMessage[outLength]='-';}      
    outLength+=1;
    dtostrf(temp,5,2,outMessage+outLength);                               // temp
    if((strstr(outMessage,"nan")!=0) || !dsSta){strcpy((char*)(outMessage+outLength),"+00.00\0");}
    outLength+=5;
    // get voltage 
    dtostrf(volts,4,2,outMessage+outLength);
    outLength+=4;                                                         // power voltage
    outMessage[outLength]='\0';

    /* sending message */
    trst=txMessage(outMessage,'N',outLength,0);  
    if(trst<0){
      // si le message ne part pas, essai aux (aw_retry-1) prochains réveils 
      // puis après aw_ko réveils
      Serial.print((char*)outMessage);Serial.println(" ********** maxRT ");
      switch(retryCnt){
        case 0:retryCnt=aw_retry;break;
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;awakeCnt=0;break;
        default:retryCnt--;break;
      }
    }
    else{                 // transmit ok 
      /* every counters reset */
      retryCnt=0;
      awakeCnt=aw_ok;
      awakeMinCnt=aw_min;
      previousTemp=temp;
      /* get response */
      pldLength=MAX_PAYLOAD_LENGTH;                   // max length
      int rdSta=AV_EMPTY;
      read_beg=millis();
      while((rdSta==AV_EMPTY)&& (readTo>=0)){
        rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);
        readTo=TO_READ-millis()+read_beg;}
      Serial.print("rdsta=");Serial.println(rdSta);  
      if(rdSta>=0 && readTo>0){                                    // no error
        showRx();

        perRefr=(long)convStrToNum(message+ADDR_LENGTH+1+MPOSPERREFR-MPOSPERREFR,&sizeRead);     // per refresh server
        aw_ok=perTemp/STEP_VALUE;
        perTemp=(uint16_t)convStrToNum(message+ADDR_LENGTH+1+MPOSTEMPPER-MPOSPERREFR,&sizeRead); // per check température
        aw_min=perRefr/STEP_VALUE;
        deltaTemp=(long)convStrToNum(message+ADDR_LENGTH+1+MPOSPITCH-MPOSPERREFR,&sizeRead);     // pitch mesure
Serial.print("R=");Serial.print(perRefr);Serial.print("/");Serial.print(aw_ok);Serial.print(" T=");Serial.print(perTemp);Serial.print("/");Serial.print(aw_min);Serial.print(" p=");Serial.println(deltaTemp);
      }
      else{
        if(rdSta>=0){rdSta=ER_RDYTO;}
        Serial.print(" response err ");Serial.println((char*)kk+(rdSta+6)*LMERR);}    // error
    }
  }
  delay(1);  // serial

#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'

  if(menu){
    Serial.println("(e)cho (b)roadcast (t)ableC (q)uit");
    menu=false;
  }

  ledblk(TBLK,2000,IBLK,1);

  pldLength=MAX_PAYLOAD_LENGTH;                   // max length
  numP=nrfp.read(message,&pipe,&pldLength,NBPERIF);
  if(numP>=0){                                    // no error
    showRx();
    if(numP==0){                                  // register request
      byte pAd[ADDR_LENGTH];             
      uint8_t numP=nrfp.cRegister(message);
      if(numP<(NBPERIF)){
        Serial.print(" registred on addr (");Serial.print(numP);Serial.print(")");nrfp.printAddr(pAd,'n');}
      else if(numP==(NBPERIF+2)){Serial.print(" MAX_RT ... deleted");}
      else {Serial.print(" full");}
      Serial.println();
    }
        
    else {                                        // pas demande d'inscription... send config
      Serial.println();
      /* store incoming message */
      memcpy(tableC[numP].periBuf,message,pldLength);   
      tableC[numP].periBufLength=pldLength;
      /* build config */
      memcpy(message+ADDR_LENGTH+1,tableC[numP].serverBuf,MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1);
      /* send config message */
      int trst=txMessage(message,'N',MAX_PAYLOAD_LENGTH,numP);
      if(trst==0){tableC[numP].periBufSent=true;}
      // reformater et transmettre au serveur le message reçu 
    }
  }

  else if(numP!=AV_EMPTY){
    showRx();
    Serial.print(" err ");Serial.println((char*)kk+(numP+6)*LMERR);}    // error... à traiter
    
  char a=getch();
  switch(a){
    case 'e':echo(a);menu=true;break;
    case 'b':broadcast(a);menu=true;break;
    case 't':Serial.println((char)a);nrfp.tableCPrint();menu=true;break;
    default:break;
  }
#endif // NRF_MODE == 'C'

} /********** fin loop *********/

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


void echo0(char* message,uint8_t len,uint8_t numP)
{ 
  message[len]='+';
  message[len+1]='\0';

  int trSta=-1;
  trSta=txMessage(message,'N',len,numP);
  
  if(trSta<0){Serial.print("********** maxRT ");
#if NRF_MODE == 'P'
    beginP();
#endif NRF_MODE == 'P'
  }
  else{
    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    pldLength=MAX_PAYLOAD_LENGTH;                   // max length
    readTo=0;
    numP=AV_EMPTY;
    long read_beg=millis();
    while((numP==AV_EMPTY)&& (readTo>=0)){
      numP=nrfp.read(message,&pipe,&pldLength,NBPERIF);
      readTo=TO_READ-millis()+read_beg;}    
    time_end=micros();
    if(numP>=0){         // data ready, no error
      showRx();
    }
    else{Serial.print("error ");Serial.print((char*)kk+(numP+ER_MAXER)*LMERR);} 
  }
  Serial.print(" in:");Serial.print((long)(time_end - time_beg));Serial.println("us");
}

#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'

void wdtSetup(uint8_t durat)  // (0-9) durat>9 for external wdt on INT0 (à traiter)
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
                                  
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | durat;       // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s
//    WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3);       // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s

  interrupts();

}

void sleepPwrDown(uint8_t durat) {

    nbS++;
    durT+=wdtTime[durat]/10;
    hardwarePowerDown();

    wdtSetup(durat);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    power_adc_disable();
    sleep_enable();
//Serial.print("sleep_enable ");Serial.println(durat);delay(10); 
    sleep_mode();
  
    sleep_disable();
    power_all_enable();
    //hardwarePowerUp();
}

ISR(WDT_vect)                      // ISR interrupt service pour vecteurs d'IT du MPU (ici vecteur WDT)
{
  reveil = true;
}

#endif NRF_MODE == 'P'

int txMessage(char* message,char ack,uint8_t len,uint8_t numP)
{
#if NRF_MODE=='P'  
  if(numP==0){beginP();}
#endif NRF_MODE=='P'
  
  Serial.print((char*)message);delay(1);
  
  nrfp.write(message,ack,len,numP);
  int trSta=1;
  time_beg = micros();
  while(trSta==1){
    trSta=nrfp.transmitting();}

  time_end=micros();  
  Serial.print(" tx to ");  
  Serial.print(numP);Serial.print(" in:");
  Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");
delay(1);
  return trSta;
}

void showRx()
{  
  Serial.print((char*)message);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" numP=");Serial.print(numP);
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
