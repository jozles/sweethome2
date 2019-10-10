#include <SPI.h>
#include "nrf24l01s_const.h"
#include "nRF24L01.h"
#include "nrf24l01s.h"

#include "nrf_powerSleep.h"
#include "nrf_user_peri.h"
#include "nrf_user_conc.h"

#ifdef DUE
#include <MemoryFree.h>;
#endif

#if NRF_MODE == 'C'
extern struct NrfConTable tableC[NBPERIF];
bool menu=true;
#endif NRF_MODE == 'C'

Nrfp nrfp;

/*** LED ***/
uint16_t      blkdelay=0;
unsigned long blktime=0;
uint8_t       bcnt=0;
#define TBLK    1
#define DBLK    2000
#define IBLK    50
#define TMINBLK 500     // uS

/*** scratch ***/

#define TO_READ 1000
long readTo;                              // compteur TO read()
unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];    // buffer pour read()/write()
int     rdSta;                            // return status read() / available()
int     trSta;                            // return status write() / transmitting()
uint8_t pipe;
uint8_t pldLength;

uint8_t numT=0;                           // numéro périphérique dans table concentrateur

float   volts=0;                          // tension alim (VCC)

#define NTESTAD '1'                       // numéro testad dans table
byte    testAd[]={'t','e','s','t','x',NTESTAD};    // txaddr pour broadcast

#define LMERR 9           
char*   kk={"time out\0tx maxrt\0rx empty\0mac addr\0length  \0pipe nb \0--      \0ok      \0"};         // codes retour et erreur

#if NRF_MODE == 'C'
char bufServer[BUF_SERVER_LENGTH];     // to/from server buffer

unsigned long timeImport=0;        // timer pour Import (si trop fréquent, buffer pas plein         
#define PERIMPORT 100

#endif

#if NRF_MODE == 'P'

/*** gestion sleep ***/
bool      tfr=false;                // pour DIAG
bool      mustSend; 
int       awakeCnt=0;
int       awakeMinCnt=0;
int       retryCnt=0;
uint32_t  nbS=0;                   // nbre sleeps
float     durT=0;                  // temps sleep cumulé (mS/10)
float     tBeg=0;                  // temps total depuis reset (oscillateur local)

uint16_t  aw_ok=AWAKE_OK_VALUE;
uint16_t  aw_min=AWAKE_MIN_VALUE;
uint16_t  aw_ko=AWAKE_KO_VALUE;
uint8_t   aw_retry=AWAKE_RETRY_VALUE;

float     timer1;
bool      timer1Ovf;
bool      extTimer;
float     period;
#define PRESCALER_RATIO 256           // prescaler ratio clock timer1 clock
#define TCCR1B_PRESCALER_MASK 0xF8    // prescaler bit mask in TCCR1B
#if PRESCALER_RATIO==1024
  #define TCCR1B_PRESCALER_BITS 0x05  // prescaler bit value for ratio 1024 in TCCR1B
#endif  
#if PRESCALER_RATIO==256
  #define TCCR1B_PRESCALER_BITS 0x04  // prescaler bit value for ratio 256 in TCCR1B
#endif  
#define CPU_FREQUENCY 8000000

extern float temp;    // debug

uint8_t beginP();
void    getVolts();
#endif NRF_MODE == 'P'

/* prototypes */

void showErr();
void showRx(bool crlf);
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
void delayBlk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb,unsigned long dly);
int  txMessage(bool ack,uint8_t len,uint8_t numP);
int  rxMessage();
void echo0(char* message,bool ack,uint8_t len,uint8_t numP);
#if NRF_MODE == 'C'
char getch();
void echo();
void broadcast();
#endif NRF_MODE == 'C'

void hardwarePowerUp()
{
  nrfp.powerUp();
  pinMode(LED,OUTPUT);
  pinMode(PP,OUTPUT);
  pinMode(REED,INPUT_PULLUP);
}

void int_ISR()
{
  extTimer=true;
  //Serial.println("int_ISR");
}


void setup() {

  delay(100);
  Serial.begin(115200);

  Serial.print("\nstart setup ");

#if NRF_MODE == 'P'

  /* external timer calibration sequence */

  delayBlk(1,0,250,1,5000);               // 5sec blinking
  sleepPwrDown(0);                        // wait interrupt from external timer

  long beg=millis();
  pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delay(100);digitalWrite(LED,LOW);   // external timer calibration starts
  attachInterrupt(0,int_ISR,FALLING);     // external timer interrupt
  EIFR=bit(INTF0);                        // clr flag
  
  while(!extTimer){delay(1);}             // évite le blocage à la fin ... ???
  
  detachInterrupt(0);
  period=(float)(millis()-beg)/1000;
  pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delay(100);digitalWrite(LED,LOW); 
  Serial.print("period is ");Serial.print(period);Serial.println("sec");delay(1);

  /* ------------------- */
  
  userResetSetup();
  hardwarePowerUp();
  nrfp.setup();

  numT=beginP();                          // registration
  
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  Serial.print(TXRX_MODE);
  delay(100);

  userResetSetup();

  nrfp.tableCInit();//nrfp.tableCPrint();
  memcpy(tableC[1].periMac,testAd,ADDR_LENGTH+1);     // pour broadcast & test

  hardwarePowerUp();                                
  nrfp.setup();
  
#ifdef DUE
  Serial.print("free=");Serial.print(freeMemory(), DEC);Serial.print(" ");
#endif  

#endif NRF_MODE == 'C'

  time_beg=millis();  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}          // 0,8sec (4 blink)

  Serial.println("end setup");delay(1);
}

void loop() {

#if NRF_MODE == 'P'

  /* timing to usefull awake */
  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){           

    pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);
    awakeCnt--;
    awakeMinCnt--;
    delayMicroseconds(TMINBLK);
    digitalWrite(LED,LOW);            // 5+1mA rc=0,5/8000 -> 372nA
    sleepPwrDown(0);
    durT+=period*1000;
  }

  /* usefull awake or retry */
  awakeCnt=aw_ok;
  mustSend=false;           
  mustSend=checkThings(awakeCnt,awakeMinCnt,retryCnt);           // user staff

   delay(200);                    // pour démarrage de l'ADC quand pas de thermo
   getVolts();                  // checkthings laisse le temps à l'ADC de démarrer (si thermo)
   if(volts<VOLTMIN){
    // shutdown...
   }                     

  tBeg=((float)millis()/1000)+(durT/100);
#ifdef DIAG
  Serial.print(" ");
  Serial.print(nbS);Serial.print("(");Serial.print(tBeg);Serial.print(") ");
  Serial.print(awakeCnt);Serial.print(" ");
  Serial.print(awakeMinCnt);Serial.print(" ");Serial.println(retryCnt);
#endif // DIAG

  /* data ready or presence message time or retry -> send */
  if( (mustSend==true) || (awakeMinCnt<0) || (retryCnt!=0)){

    hardwarePowerUp();                   // 5mS delay inside

    /* building message MMMMMPssssssssVVVVU.UU....... MMMMMP should not be changed */
    /* MMMMM mac P periNb ssssssss Seconds VVVV version U.UU volts ....... user data */
    uint8_t outLength=ADDR_LENGTH+1;
    memcpy(message+outLength,VERSION,LENVERSION);                     // version
    outLength+=LENVERSION;
    sprintf((char*)(message+outLength),"%08d",(uint32_t)tBeg);        // seconds since last reset 
    outLength+=8;
                                                                                                          //#define USRDATAPOS ADDR_LENGTH+1+LENVERSION+8+4
    messageBuild((char*)message,&outLength);                          // add user data
    memcpy(message,MAC_ADDR,ADDR_LENGTH);                             // macAddr
    message[ADDR_LENGTH]=numT+48;                                     // numéro du périphérique
    message[outLength]='\0';

    /* send message */
    if(txMessage(NO_ACK,outLength,0)<0){                              // tx error (maxrt)
      // si le message ne part pas, essai aux (aw_retry-1) prochains réveils 
      // puis après aw_ko réveils
#ifdef DIAG
      Serial.print((char*)message);Serial.println(" ********** maxRT ");
#endif // DIAG
      switch(retryCnt){
        case 0:retryCnt=aw_retry;break;
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;awakeCnt=0;break;
        default:retryCnt--;break;
      }
    }
    else{                                  // tx ok 
      
      /* every counters reset */
      retryCnt=0;
      awakeCnt=aw_ok;
      awakeMinCnt=aw_min;
      
      /* get response */
      if(rxMessage()>=0){                  // rx ok
        showRx(true);
        importData(message,pldLength);     // user data
      }
      else{showErr();numT=beginP();}       // rx error : refaire l'inscription
    }
    tfr=true;  // pour DIAG
  }
#ifndef DIAG
        if(tfr==true){Serial.print(" ");Serial.print(rdSta);Serial.print("/");}
        else {Serial.print(" ./");}
        tfr=false;
#endif // DIAG

  delay(1);  // serial
  
#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'

  if(menu){
    nrfp.printAddr((char*)MAC_ADDR,0);
    Serial.println(" (e)cho (b)roadcast (t)ableC (q)uit");
    menu=false;
  }

  ledblk(TBLK,2000,IBLK,1);

  numT=0;                                             // will stay 0 if no registration
  pldLength=MAX_PAYLOAD_LENGTH;                       // max length
  rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);  // get message from perif

  time_beg=micros();
    
  // ====== no error registration request ======
  
  if(rdSta==0){                                 
      showRx(false);                                        
      numT=nrfp.cRegister((char*)message);               
      if(numT<(NBPERIF)){                         // registration ok
        rdSta=numT;                               // entry is valid -> rdSta >0              
        nrfp.printAddr((char*)tableC[numT].periMac,' ');
        Serial.print(" registred as ");Serial.println(numT);}
      else if(numT==(NBPERIF+2)){Serial.println(" MAX_RT ... deleted");}
      else {Serial.println(" full");}
  }

  // ====== no error && valid entry ======
  
  if((rdSta>0) && (memcmp(message,tableC[rdSta].periMac,ADDR_LENGTH)==0)){ 
      if(numT==0){      // if not registration (no valid data), incoming message storage
        memcpy(tableC[rdSta].periBuf,message,pldLength);    
        tableC[rdSta].periBufLength=pldLength;}
      /* build config */
      memcpy(message+ADDR_LENGTH+1,tableC[rdSta].servBuf,MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1);
      /* send it to perif */    
      txMessage(ACK,MAX_PAYLOAD_LENGTH,rdSta);        // end of transaction so auto ACK
      if(trSta==0){tableC[rdSta].periBufSent=true;} 
      
                                                      // ======= formatting & tx to server ======
      if(numT==0){exportData(rdSta);}                 // if not registration (no valid data), tx to server
  }
  
  // ====== error or empty -> ignore ======
  // peripheral must re-do registration so no answer to lead to rx error


#ifdef DIAG                                    
  if(rdSta<0 && rdSta!=AV_EMPTY){                    
    showRx(false);
    showErr();}
#endif // DIAG
#ifndef DIAG                                        // minimal diag - rdSta + durat + trSta
  if(rdSta!=AV_EMPTY){                   
    Serial.print(rdSta);Serial.print(" ");Serial.print(micros()-time_beg);Serial.print(" ");Serial.println(trSta);}
#endif // !DIAG

  // ====== RX from server ? ====
  
  // importData returns MESSOK(ok)/MESSCX(no cx)/MESSLEN(len=0);MESSNUMP(numPeri HS)/MESSMAC(mac not found)

  //if((millis()-(timeImport+PERIMPORT))>0){
    //timeImport=millis();
    int dt=importData();
    if(dt==MESSNUMP){tableC[rdSta].numPeri=0;} 
#ifdef DIAG
  if((dt==MESSMAC)||(dt==MESSNUMP)){Serial.print(" importData=");Serial.print(dt);Serial.print(" bS=");Serial.println(bufServer);}
#endif // DIAG

  //}


  // ====== menu choice ======  
  
  char a=getch();
  switch(a){
    case 'e':echo(a);menu=true;break;
    case 'b':broadcast(a);menu=true;break;
    case 't':Serial.println((char)a);nrfp.tableCPrint();menu=true;break;
    default:break;
  }
  
#endif // NRF_MODE == 'C'

} /********** loop end  *********/

#if NRF_MODE == 'C'

void echo(char a)
{
  int cnt=0;
  int cntko=0;

  Serial.println((char)a);
  
  while (getch()!='q'){

    uint8_t numP=1; // 1er perif de la table
    cnt++;
    sprintf((char*)message,"%05d",cnt);
    echo0((char*)message,NO_ACK,5,numP);
                                                // si periMac ne répond plus ... à traiter
    delay(2000);ledblk(TBLK,DBLK,IBLK,2);
  } 
}

void broadcast(char a)
{
  bool ack=true,titre=true;
  char b;
  Serial.println((char)a);
  while(1){
    if(titre==true){
      titre=false;
      if(!ack){Serial.print('!');}
      Serial.print("ACK ... saisir l'adresse (rien=");
      nrfp.printAddr((char*)tableC[1].periMac,0);Serial.println(" ; !=toogle ack ; q=exit)");}
   
//    while(Serial.available()){Serial.print(getch());}
    b=getch();
    switch (b){
      case 0:break;
      case '!':ack=!ack;titre=true;break;
      case 'q':return;
      default:
        if(b!=' '){
          testAd[0]=b;delay(10);for(int i=1;i<ADDR_LENGTH;i++){testAd[i]=getch();}
          Serial.println((char*)testAd);
          memcpy(tableC[1].periMac,testAd,ADDR_LENGTH);
          nrfp.tableCPrint();}
        memcpy(message,testAd,ADDR_LENGTH);
        message[ADDR_LENGTH]='0';
        sprintf((char*)(message+ADDR_LENGTH+1),"%08lu",millis());
        echo0((char*)message,ack,8,1);
        titre=true;
        break; 
    }
  }
}

void echo0(char* message,bool ack,uint8_t len,uint8_t numP)
{                       // txMessage + read réponse (1er perif de la table)
  message[len]='+';
  message[len+1]='\0';

  int trSta=-1;
  
  if(txMessage(ack,len+1,numP)<0){Serial.println("********* maxRT ");}
  else{
    if(rxMessage()>=0){showRx(true);}         // data ready, no error
    else{showErr();} 
  }
}

char getch()
{
  if(Serial.available()){
    return Serial.read();
  }
  return 0;
}


#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'

uint8_t beginP()
{
  int confSta=-1;
  while(confSta<=0){
    confSta=nrfp.pRegister(message,&pldLength); // -4 empty -3 max RT ; -2 len ; -1 pipe ; 0 na ; >=1 ok numT
    Serial.print(">>> start ");Serial.print(temp);Serial.print(" ");
    getVolts();
    Serial.print(volts);Serial.print(" ");
    Serial.print((char*)(kk+(confSta+6)*LMERR));
    Serial.print(" numT=");Serial.print(confSta);
#ifdef DIAG
    Serial.print("  aw_ok=");Serial.print(aw_ok*STEP_VALUE);
    Serial.print("sec   aw_min=");Serial.print(aw_min*STEP_VALUE);Serial.print("sec ");
    delay(1);
#endif
    Serial.println();
    delay(2);         

    sleepPwrDown(0);
    delayBlk(1,0,250,2,1);         // 2 blinks (hardwarePowerUp() included)
  }
  importData(message,pldLength);   // user data available
  awakeMinCnt=-1;                  // force data upload
  return confSta;                  // le périphérique est inscrit
}

void getVolts()
{
  analogReference(INTERNAL); 
  pinMode(VCHECK,OUTPUT);digitalWrite(VCHECK,LOW);
  volts=analogRead(VINPUT)*VFACTOR;
  pinMode(VCHECK,INPUT);
}
 
#endif NRF_MODE == 'P'

int txMessage(bool ack,uint8_t len,uint8_t numP)
{
#if NRF_MODE=='P'  
  if(numT==0){numT=beginP();}
#endif NRF_MODE=='P'

#ifdef DIAG  
  Serial.print((char*)message);delay(1);
#endif // DIAG
  
  nrfp.write(message,ack,len,numP);
  trSta=1;
  time_beg = micros();
  while(trSta==1){
    trSta=nrfp.transmitting();}

  time_end=micros();
#ifdef DIAG
  Serial.print(" tx to ");Serial.print(numP);
  Serial.print(" trSta=");Serial.print(trSta);
  Serial.print(" in:");Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");
delay(4);
#endif // DIAG
  return trSta;
}

int rxMessage()
{
  memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
  pldLength=MAX_PAYLOAD_LENGTH;                    // max length
  rdSta=AV_EMPTY;
  readTo=0;
  time_beg = micros();
  while((rdSta==AV_EMPTY) && (readTo>=0)){
    rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);
    readTo=TO_READ-(micros()-time_beg)/1000;}
    if(readTo<0){rdSta=ER_RDYTO;}
  time_end=micros();
#ifdef DIAG
  Serial.print((char*)message);  
  Serial.print(" rx from ");Serial.print((int)(message[ADDR_LENGTH]-48));
  Serial.print(" rdsta=");Serial.print(rdSta);  
  Serial.print(" in:");Serial.print((long)(time_end - time_beg)); 
  Serial.print("us ");
delay(4);
#endif // DIAG
  return rdSta;
}

void showRx(bool crlf)
{ 
#ifdef DIAG
  //Serial.print((char*)message);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  //Serial.print(" rdSta=");Serial.print(rdSta);
  if(crlf){Serial.println();}
delay(1);
#endif // DIAG  
}

void showErr()
{
#ifdef DIAG  
  Serial.print(" err ");Serial.println((char*)kk+(rdSta+6)*LMERR);
#endif // DIAG
}

#if NRF_MODE == 'P'
void sleepDly(uint16_t dly)                                                       // should be (nx250)
{
  delay(1);                     // serial
  dly=(dly/250)*250;
  while(dly>=250){durT+=sleepPwrDown(T250);dly-=250;}
}
#endif // NRF_MODE == 'P'

void delayBlk(int dur,int bdelay,int bint,uint8_t bnb,int dly)
/*  dur=on state duration ; bdelay=time between blink sequences ; bint=off state duration ; 
    bnb=(on+off) nb in one sequence ; dly=total delay time   
    
    delays are in sleepPwrDown() mode
    bint, bdelay must be multiple of 250
    at least 1 bdelay is executed (even dly smaller)
    usable if dly > (dur+bint)*bnb+bdelay
    hardwarePowerDown() at beginning of every sleepPwrDown() ; hardwarePowerUp() at end of delayBlk

    exemples :

    delayBlk(1,0,250,1,5000);         // 5sec blinking (1/250)
    delayBlk(1,0,250,3,1);            // 3 blinks (1/250)
    delayBlk(300,0,1,1);              // 1 pulse (300)
    
*/    
{
#if NRF_MODE == 'P'
  
  while(dly>0){
 
    for(int i=0;i<bnb;i++){
      pinMode(LED,OUTPUT);                      // sleepDly() -> sleepPwrDown() -> hardwarePowerDown() -> pinMode(LED,input)
      digitalWrite(LED,HIGH);delay(dur);
      digitalWrite(LED,LOW);sleepDly(bint);
      dly-=(dur+bint);
    }
    sleepDly(bdelay);
    dly-=bdelay;
  }
  hardwarePowerUp();                            // hardwarePowerUp() -> nrfp.powerUp() -> delay(5)

#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'
  unsigned long tt=millis(); 
  blktime=0;bcnt=1;blkdelay=0;
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);dly-=((dur+bint)*bnb+bdelay);}
#endif // NRF_MODE == 'C'
}

void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb)
{   // dur = durée on ; bdelay = delay entre séquences ; bint = intervalle entre blinks ; bnb = nbre blinks
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
 *     utilisation du timer 1        ...... réaction bizarres avec delay()   
 *
ISR(TIMER1_OVF_vect)                     // ISR interrupt service for MPU timer 1 ovf vector
{
  timer1Ovf=true;
  timer1+=0x010000;
  //Serial.println(timer1);
}

 *      
  TCCR1A=0;
  TCCR1B &= TCCR1B_PRESCALER_MASK;        // timer1 prescaler
  TCCR1B |= TCCR1B_PRESCALER_BITS;        // timer1 prescaler value  if /256 -> 8MHz / 256 / 65532 = 2.097Hz
  GTCCR  |= PSRSYNC;                      // reset prescaler
  TCNT1=0;                                // clr counter

  TIMSK1 |= (1<<TOIE1);                   // enable timer1 ovf interrupt  

  TIMSK1 &= ~(1<<TOIE1);                  // disable timer1 ovf interrupt
  */

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
