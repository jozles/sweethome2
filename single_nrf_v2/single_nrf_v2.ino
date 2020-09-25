#include <SPI.h>
#include "nrf24l01s_const.h"
#include "nRF24L01.h"
#include "nrf24l01s.h"

#include "nrf_powerSleep.h"
#include "nrf_user_peri.h"
#include "nrf_user_conc.h"

 /* single signifie 1 seul concentrateur avec une seule adresse de réception
 *  l'option P ou C dans const.h définit si compilation du code pour concentrateur ou périphérique
 *  nRF24L01.h les constantes du circuit ; nrf24l01s.h header pour nrf24l01s.cpp la lib du circuit
 *  nrf24l01s_const.h les constantes pour single_nrf
 */
 
#ifdef DUE
#include <MemoryFree.h>;
#endif

#ifdef DS18X20
#include <ds18x20.h>
Ds1820 ds1820;
byte     setds[]={0,0x7f,0x80,0x3f},readds[8];   // 1f=93mS 9 bits accu 0,5° ; 3f=187mS 10 bits accu 0,25° !!! TCONVDS to adjust
#define  TCONVDS1 T64   // sleep PwrDown mS !!
#define  TCONVDS2 T125  // sleep PwrDown mS !!
#endif DS18X20 

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

#define TO_READ 30                        // mS rxMessage time out 
long readTo;                              // compteur TO read()
uint32_t      tBeg=0;                     // date unix du 1er message reçu
unsigned long t_on;
unsigned long t_on0;
unsigned long t_on1;
unsigned long t_on2;
unsigned long t_on21;
unsigned long t_on3;
unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];    // buffer pour write()
byte    messageIn[MAX_PAYLOAD_LENGTH+1];  // buffer pour read()
int     rdSta;                            // return status read() / available()
int     trSta;                            // return status write() / transmitting()
uint8_t pipe;
uint8_t pldLength;

uint8_t numT=0;                           // numéro périphérique dans table concentrateur

extern float   volts;                          // tension alim (VCC)

#define NTESTAD '1'                       // numéro testad dans table
byte    testAd[]={'t','e','s','t','x',NTESTAD};    // txaddr pour broadcast

#define LDIAGMESS 80
char    diagMessT[LDIAGMESS];             // buffer texte diag Tx
char    diagMessR[LDIAGMESS];             // buffer texte diag Tx
#define LBUFCV 7
char    bufCv[LBUFCV];                    // buffer conversion sprintf

#define LMERR 9           
char*   kk={"time out\0tx maxrt\0rx empty\0mac addr\0length  \0pipe nb \0--      \0ok      \0"};         // codes retour et erreur

#define ECHO_LEN 10                       // echo message len

#define ECHO_MAC_REQ "UUUU0"              // echo req mac
uint8_t echoNb=0;                         // numéro du périf sur lequel demander l'écho
bool    echoOn=false;                     // fonction echo en cours (sur l'entrée de table 1 envoi de messages pour écho - test de portée)
                                          // le périphérique n'écoutant que les réponses à ses messages, il faut attendre une demande pour commencer.
                                          // en attente, echoOn=true ;
                                          // le concentrateur est bloqué pendant la maneuvre

#if NRF_MODE == 'C'

char    bufServer[BUF_SERVER_LENGTH];     // to/from server buffer

unsigned long timeImport=0;        // timer pour Import (si trop fréquent, buffer pas plein         
unsigned long tLast=0;             // date unix dernier message reçu 
#define PERIMPORT 100

#endif // NRF_MODE == 'C'

#if NRF_MODE == 'P'

/*** gestion sleep ***/

bool      mustSend; 
int       awakeCnt=0;
int       awakeMinCnt=0;
int       retryCnt=0;
uint32_t  nbS=0;                   // nbre com
uint32_t  nbL=0;                   // nbre loops
float     durT=0;                  // temps sleep cumulé (mS/10)

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

float temp;
float previousTemp=-99.99;
float deltaTemp=0.25;
bool  thSta=true;                     // temp validity
char  thermo[]={THERMO};              // thermo name text
char  thN;                            // thermo code for version


uint8_t beginP();
void    echo();
#endif NRF_MODE == 'P'

/* prototypes */

void iniTemp();
void readTemp();
void showErr(bool crlf);
void showRx(bool crlf);
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
void delayBlk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb,unsigned long dly);
int  txMessage(bool ack,uint8_t len,uint8_t numP);
int  rxMessage(unsigned long to);
void echo0(char* message,bool ack,uint8_t len,uint8_t numP);
#if NRF_MODE == 'C'
char getch();
void echo();
void broadcast(char a);
#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'
void int_ISR()
{
  extTimer=true;
  //Serial.println("int_ISR");
}
#endif NRF_MODE == 'P'

void setup() {

#if NRF_MODE == 'P'

  /* external timer calibration sequence */
  hardwarePowerUp();
  
  wd();                           // watchdog
  iniTemp();
 
  delay(1000); // pour reset de programmation
  Serial.begin(115200);
  Serial.println();
  Serial.println("+ every wake up ; ! mustSend true ; * force transmit (perRefr or retry) ; $ showerr ");
  Serial.println("£ importData (received to local) ; € diags fin loop");delay(4);
  Serial.println();Serial.print(PER_ADDR);Serial.print(" start setup ");Serial.print(thermo);delay(3);

  delayBlk(1,0,250,1,2000);               // 2sec blinking
  
  sleepPwrDown(0);                        // wait for interrupt from external timer to reach beginning of period

  long beg=millis();
  led(100000);                            // external timer calibration begin
  
  attachInterrupt(0,int_ISR,ISREDGE);     // external timer interrupt
  EIFR=bit(INTF0);                        // clr flag
  while(!extTimer){delay(1);}             // évite le blocage à la fin ... ???
  
  detachInterrupt(0);
  period=(float)(millis()-beg)/1000;
  led(100000);
  Serial.print("period ");Serial.print(period);Serial.print("sec "); // external timer period

  getVolts();getVolts();                  // read voltage and temperature (1ère conversion ADC ko)

  /* ------------------- */
  
  userResetSetup();

  Serial.print(volts);Serial.print("V ");
  Serial.print(temp);Serial.print("°C ");
  
  nrfp.powerOn();
  numT=beginP();                          // registration 
                                          // if radio HS or missing nrfp.lastSta=0xFF
  nrfp.powerOff();                                          
  Serial.print("numT=");Serial.println(numT);
  
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  delay(100);
  Serial.begin(115200);

  Serial.println();Serial.print(" start setup ");

  Serial.print(TXRX_MODE);
  delay(100);

  pinMode(LED,OUTPUT);
  userResetSetup();

  nrfp.tableCInit();//nrfp.tableCPrint();
  memcpy(tableC[1].periMac,testAd,ADDR_LENGTH+1);     // pour broadcast & test

  nrfp.powerUp();
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

#ifdef DIAG  
  Serial.print("€ ");
  ///*
  Serial.print(awakeMinCnt);Serial.print(" / ");Serial.print(awakeCnt);Serial.print(" ; ");
  //Serial.print(nbS);Serial.print("/");Serial.print(nbL);Serial.print(" | ");     // nbS com nb ; nbL loop nb
  Serial.print(volts);Serial.print("V ");Serial.print(temp);Serial.print("/");Serial.print(previousTemp);Serial.print("° t(");
  Serial.print(t_on1-t_on);Serial.print("/");
  Serial.print(t_on2-t_on);Serial.print("/");
  Serial.print(t_on21-t_on);Serial.print("/");
  Serial.print(t_on3-t_on);Serial.print("/");
  //*/
  t_on0=micros();  
  Serial.print(t_on0-t_on);Serial.println(") €");
  delay(1);
#endif DIAG

  /* timing to usefull awake */
  
  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){           

    awakeCnt--;
    awakeMinCnt--;
    sleepPwrDown(0);
    Serial.print("+");delayMicroseconds(100);
    durT+=period*1000;
    nbL++;
  }

  /* usefull awake or retry */
  t_on=micros();
  t_on1=t_on;
  t_on2=t_on;
  t_on21=t_on;  
  t_on3=t_on;
/*
  if(awakeCnt<0){Serial.print('a');}          // cyclic awake
  if(awakeMinCnt<0){Serial.print('m');}       // min awake  
  if(retryCnt!=0){Serial.print('r');}         // retry (no sleep)
*/  
  getVolts();                                 // 1.2 mS include notDS18X20 thermo reading                                     
  readTemp();                                 // only for DS18X20
  awakeCnt=aw_ok;
  mustSend=false;           
  mustSend=checkThings(awakeCnt,awakeMinCnt,retryCnt);           // user staff

  if( (temp>(previousTemp+deltaTemp)) ){
    previousTemp=temp-(deltaTemp/2);
    mustSend=true;}
  if( (temp<(previousTemp-deltaTemp)) ){
    previousTemp=temp+(deltaTemp/2);
    mustSend=true;}

  t_on1=micros();    // end of work... now send or sleep
  if(mustSend){Serial.print("!");}
  
  /* hardware ok, data ready or presence message time or retry -> send */
  if(nrfp.lastSta!=0xFF && ((mustSend==true) || (awakeMinCnt<0) || (retryCnt!=0))){         // anything to send or min or retry

    for(int nb=retryCnt;nb>0;nb--){Serial.print("*");}
    /* building message MMMMMPssssssssVVVVU.UU....... MMMMMP should not be changed */
    /* MMMMM mac P periNb ssssssss Seconds VVVV version U.UU volts ....... user data */
    uint8_t outLength=ADDR_LENGTH+1;
    memcpy(message+outLength,VERSION,LENVERSION);                     // version
    outLength+=LENVERSION;
    memcpy(message+outLength,&thN,1);                                 // modèle thermo ("B"/"S" DS18X20 "M"CP9700  "L"M335  "T"MP36    
    sprintf((char*)(message+outLength+1),"%08d",(uint32_t)tBeg);      // first connection unix time
    outLength+=9;
 
    messageBuild((char*)message,&outLength);                          // add user data
    memcpy(message,MAC_ADDR,ADDR_LENGTH);                             // macAddr
    message[ADDR_LENGTH]=numT+48;                                     // numéro du périphérique
    message[outLength]='\0';

    /* One transaction is tx+rx ; if both ok reset counters else retry management*/  
    t_on2=micros();  // message build ... send
    rdSta=-1;
    nbS++;

    nrfp.powerOn();
    trSta=txMessage(NO_ACK,outLength,0);
    if(trSta>=0){

      t_on21=micros();
      rdSta=rxMessage(0);
      if(rdSta>=0){                                                  // no error

        /* echo request ? (address field is 0x5555555555) */
        if(memcmp(messageIn,ECHO_MAC_REQ,ADDR_LENGTH)==0){echo();}
        else {
          //showRx(false);
          //Serial.println();Serial.print("%");          
          importData(messageIn,pldLength);   // user data
        }
        
        /* tx+rx ok -> every counters reset */
          retryCnt=0;
          awakeCnt=aw_ok;
          awakeMinCnt=aw_min;            
      }
    }
    nrfp.powerOff();
    t_on3=micros();  // message sent / received or error (rdSta)
    if(trSta<0 || rdSta<0){                                           // error
      //Serial.println(diagMessT);delay(2);               // 3,6mS ! + 0,6mS prepa dans txmessage=4,2mS
      //Serial.println(diagMessR);delay(2);               // 3,6mS ! + 0,6mS prepa dans txmessage=4,2mS
    
      showErr(true);
      trSta=0;rdSta=0;
      //numT=0;                                           // tx or rx error : refaire l'inscription au prochain réveil
      switch(retryCnt){
        case 0:retryCnt=aw_retry;break;
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;
                awakeCnt=0;                             // ********************** debug **************************
                retryCnt=0;break;
        default:retryCnt--;break;
      }
    }
  }
  //else Serial.print("-");                               // no transmission done
  
  /* if radio HS or missing ; 1 display every 2 sleeps */
  if(nrfp.lastSta==0xFF){
    retryCnt=0;
    awakeCnt=1;
    awakeMinCnt=1;            
  }
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
  memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
  rdSta=nrfp.read(messageIn,&pipe,&pldLength,NBPERIF);  // get message from perif (<0 err ; 0 reg to do ; >0 entry nb)

  time_beg=micros();  // ******************************* message reçu **********************************
  if(rdSta!=AV_EMPTY){Serial.print(rdSta);Serial.print(" ");}
  if(rdSta>=0){
    //dumpstr((char*)messageIn,24);
    Serial.print((char*)messageIn);Serial.print(" ");
    }    

  if(rdSta==0){
    
  // ====== no error registration request ======
      
      showRx(false);                                        
      numT=nrfp.cRegister((char*)messageIn);               
      if(numT<(NBPERIF)){                             // registration ok
        rdSta=numT;                                   // entry is valid -> rdSta >0              
        nrfp.printAddr((char*)tableC[numT].periMac,' ');
        Serial.print(" registred as ");Serial.println(numT);}                 // numT = 0-(NBPERIF-1) ; rdSta=numT
      else if(numT==(NBPERIF+2)){Serial.println(" MAX_RT ... deleted");}      // numT = NBPERIF+2 ; rdSta=0
      else {Serial.println(" full");}                                         // numT = NBPERIF   ; rdSta=0
  }

  // ====== no error && valid entry ======         
  
  if((rdSta>0) && (memcmp(messageIn,tableC[rdSta].periMac,ADDR_LENGTH)==0)){ 
                                                      // rdSta is table entry nb
      if(numT==0 && (echoNb!=rdSta || !echoOn)){      // numT=0 means that is not a registration, and message is not an echo answer 
                                                      // so -> incoming message storage
        memcpy(tableC[rdSta].periBuf,messageIn,pldLength);    
        tableC[rdSta].periBufLength=pldLength;
      }
      
      if(echoNb!=rdSta){                              // if no echo pending on this table entry
      /* build config */
        memcpy(message,messageIn,ADDR_LENGTH+1);
        memcpy(message+ADDR_LENGTH+1,tableC[rdSta].servBuf,MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1);    // build message to perif with server data
                                                                                                 // server data is MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
                                                                                                 // see importData()
      /* send it to perif */    
        txMessage(ACK,MAX_PAYLOAD_LENGTH,rdSta);      // end of transaction so auto ACK
        // ******************************* réponse passée **********************************
        Serial.println(diagMessT);delay(2);
        if(trSta==0){tableC[rdSta].periBufSent=true;} 
      /* ======= formatting & tx to server ====== */
        if(numT==0){exportData(rdSta);}               // if not registration (no valid data), tx to server
      }
      else {                                    
        
      /* echo pending  :                         // echoOn flag d'attente de réponse 
        if(!echoOn){sendEchoReq();echoOn=true;}  // sendEchoReq gère la tempo
        else {echOn=false; controle temps et réponse ; affichage de la réponse}
      */
        echo();}
  }
  
  // ====== error, full or empty -> ignore ======
  // peripheral must re-do registration so no answer to lead to rx error


#ifdef DIAG                                    
  if(rdSta<0 && rdSta!=AV_EMPTY){                    
    showRx(false);
    showErr(true);}
#endif // DIAG
#ifndef DIAG                                        // minimal diag - rdSta + durat + trSta
  if(rdSta!=AV_EMPTY){                   
    Serial.print(rdSta);Serial.print(" ");Serial.print(micros()-time_beg);Serial.print(" ");Serial.println(trSta);}
#endif // !DIAG

  // ====== RX from server ? ====  
  // importData returns MESSOK(ok)/MESSCX(no cx)/MESSLEN(len=0);MESSNUMP(numPeri HS)/MESSMAC(mac not found)
  //            update tLast (last unix date)


    int dt=importData(&tLast);
    if(dt==MESSNUMP){tableC[rdSta].numPeri=0;} 
#ifdef DIAG
  if((dt==MESSMAC)||(dt==MESSNUMP)){Serial.print(" importData=");Serial.print(dt);Serial.print(" bS=");Serial.println(bufServer);}
#endif // DIAG


  // ====== menu choice ======  
  
  char a=getch();
  switch(a){
    case 'e':getEchoNum();menu=true;break;
    case 'b':broadcast('b');menu=true;break;
    case 't':Serial.println((char)a);nrfp.tableCPrint();menu=true;break;
    case 'q':Serial.print((char)a);echoNb=0;echoOn=false;menu=true;break;
    default:break;
  }
  
#endif // NRF_MODE == 'C'

} /******************** loop end  *******************/

#if NRF_MODE == 'C'

void getEchoNum()
{
  Serial.print("Numéro dans la table ? ");
  char a=0;
  while(a==0){a=getch();}
  if(a>='0' || a<='9'){
    echoNb=a-'0';
    Serial.println(a);
  }
}

void echo()
{ 
  Serial.println("start echo");
  unsigned int cnt=0;
  #define ECHOTO 8000                // uS 
  
  while (getch()!='q'){
    cnt++;
    memcpy(message,ECHO_MAC_REQ,ADDR_LENGTH);
    sprintf((char*)message+ADDR_LENGTH,"%05d",cnt);
    message[ECHO_LEN]='\0';
    Serial.print("sent ");Serial.print((char*)message);

    /* send request */
    time_beg = micros();
    nrfp.write(message,NO_ACK,ECHO_LEN,echoNb);
    trSta=1;
    while(trSta==1 && (micros()-time_beg)<ECHOTO){
      trSta=nrfp.transmitting(NO_ACK);}                 // trsta=0 if data sent ok ; -1 if maxRt
    if(trSta<0){Serial.println("********* maxRT ");break;}
    time_end = micros();
    
    /* wait for answer until receipt ok or TO or err */
    memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
    rdSta=AV_EMPTY; 
           
    while(rdSta==AV_EMPTY && (micros()-time_end)<ECHOTO){
      rdSta=nrfp.read(messageIn,&pipe,&pldLength,NBPERIF);      
      if(rdSta>=0){Serial.print(" rcv ");Serial.print((char*)messageIn);}
      else if(rdSta<0 && rdSta!=AV_EMPTY){
        Serial.print(" err ");Serial.println((char*)kk+(rdSta+6)*LMERR);        // rcv err
        rdSta=AV_EMPTY;}                                                        // permet d'attendre le time out
      if(rdSta==echoNb){break;}                                                 // ok (if not, maybe collision)
    }
    if(rdSta==AV_EMPTY){Serial.print(" time out ");}                            // to
    Serial.print(" rdSta=");Serial.print(rdSta);Serial.print(" in ");
    Serial.print(micros()-time_beg);Serial.println("uS");
    delay(4000);
  }
  Serial.println("stop echo");
  echoNb=0;
}

void echo0(bool ack,uint8_t len,uint8_t numP)
{                       // txMessage + read réponse (1er perif de la table)
  int trSta=-1;
  
  bool waitEcho=true;
  while(waitEcho){
      rdSta=rxMessage(0);
      if(rdSta>=0){
        showRx(true);
        if(memcmp(messageIn,"VVVV",4)==0){waitEcho=false;}
      }
      else {showErr(true);waitEcho=false;}
  }         
  memcpy(message,messageIn,len);
  if(txMessage(ack,len+1,numP)<0){Serial.println("********* maxRT ");}
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
        echo0(ack,8,1);
        titre=true;
        break; 
    }
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
  unsigned long bptime=micros();
  while(1){                                       // confsta>=1 or -5 or wait 
    confSta=nrfp.pRegister(messageIn,&pldLength); // -5 maxRT ; -4 empty ; -3 mac ; -2 len ; -1 pipe ;
                                                  // 0 na ; >=1 ok numT
/*#ifdef DIAG
    int sta=confsta;if(confsta>0){sta=1;)
    Serial.print(">>> start ");
    Serial.print((char*)(kk+(sta-(ER_MAXER))*LMERR));
    Serial.print(" numT=");Serial.print(confSta);
    Serial.print(" reg ");Serial.println(micros()-bptime);
    bptime=micros();
    delay(3);         
#endif    */

    if(confSta>0){
      Serial.println();
      importData(messageIn,pldLength);  // user data available
      awakeMinCnt=-1;                 // force data upload
      
      break;                          // out of while(1)
    }

    if(confSta==-5){
      nrfp.lastSta=0xFF;              // KO mode : radio missing or HS
      break;                          // out of while(1)  
    }
   
    if(confSta=-4){Serial.println();Serial.print("beginP no answer ");}
    Serial.print(confSta);delay(2);

    nrfp.powerOff();
    sleepPwrDown(0);                  // still waiting (about 4,5+2mS @20mA)
    // in order to minimize power wasting, special sleep needed (like aw_ok 1 time then aw_min)
    // + including double blink  
    
    delayBlk(1,0,250,2,1);            // 2 blinks
    nrfp.powerOn();                   // after sleep                   
  }

  return confSta;                     // peripheral registred or radio HS
}

void echo()
{
#define MAXECHOERR 3                 // max consecutive errors to leave
#define ECHOTO  10000                // TO mS

  uint8_t cntErr=0;
  unsigned long to=ECHOTO;
  char echoRef[ADDR_LENGTH+1];
  memcpy(echoRef,PER_ADDR,ADDR_LENGTH);
  echoRef[ADDR_LENGTH]='0'+numT;
  byte echoMess[ECHO_LEN+1];
  
 // Serial.println("start echo");
  
  while(1){

    /* send echo */
    memcpy(echoMess,messageIn,ECHO_LEN);
    memcpy(echoMess,echoRef,ADDR_LENGTH+1);
    echoMess[ECHO_LEN]='\0';
    nrfp.write(echoMess,NO_ACK,ECHO_LEN,0);
    trSta=1;
    while(trSta==1){
      trSta=nrfp.transmitting(NO_ACK);}                     // trsta=0 if data sent ok ; -1 if maxRt

    time_end=micros();

    Serial.print(" rcv ");Serial.print((char*)messageIn);
    Serial.print(" sent ");Serial.print((char*)echoMess);      
    Serial.print(" wait ");Serial.print((time_end-time_beg)/1000);Serial.println("mS");
    delay(4);

    time_beg=micros();

    /* wait for request until recept or max err */
    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    rdSta=AV_EMPTY;        
    while(rdSta<0 && cntErr<MAXECHOERR){
      rdSta=nrfp.read(messageIn,&pipe,&pldLength,NBPERIF);
      if((micros()-time_beg)/1000>to){rdSta=ER_RDYTO;time_beg=micros();}
      if(rdSta<0 && rdSta!=AV_EMPTY){showErr(true);cntErr++;}                
    }

    /* if max err or no req mess leave */
    if(cntErr>=MAXECHOERR || memcmp(messageIn,ECHO_MAC_REQ,4)!=0 ){break;}
    else cntErr=0;
  }
  Serial.println("stop echo");delay(1);
}

#endif NRF_MODE == 'P'

int txMessage(bool ack,uint8_t len,uint8_t numP)
{
#if NRF_MODE=='P'  
  if(numT==0){numT=beginP();}
#endif NRF_MODE=='P'

  nrfp.write(message,ack,len,numP);
  trSta=1;
  time_beg = micros();
  while(trSta==1){
    trSta=nrfp.transmitting(ack);}                 // trsta=0 if data sent ok ; -1 if maxRt

  time_end=micros();

#ifdef DIAG
#if NRF_MODE=='C'
  memset(bufCv,0x00,LBUFCV);
  memcpy(diagMessT,message,len);
  diagMessT[len]='\0';
  strcat(diagMessT," to ");  
  sprintf(bufCv,"%1d",numP);
  strcat(diagMessT,bufCv);
  strcat(diagMessT," trSta=");
  sprintf(bufCv,"%-1d",trSta);
  strcat(diagMessT,bufCv);
  strcat(diagMessT," in ");
  sprintf(bufCv,"%d",(time_end - time_beg));
  strcat(diagMessT,bufCv);
  strcat(diagMessT,"uS");
#endif NRF_MODE=='C'
#endif // DIAG

  return trSta;
}

int rxMessage(unsigned long to) // retour rdSta=ER_RDYTO TO ou sortie de available/read (0=full, pipe err, length err)
{
  time_beg = micros();
  if(to==0){to=TO_READ;}
  //memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
  pldLength=MAX_PAYLOAD_LENGTH;                    // max length
  rdSta=AV_EMPTY;
  readTo=0;
  while((rdSta==AV_EMPTY) && (readTo>=0)){
    rdSta=nrfp.read(messageIn,&pipe,&pldLength,NBPERIF);
    readTo=to-(micros()-time_beg)/1000;}
  PP4_HIGH
  if(readTo<0){rdSta=ER_RDYTO;}
  nrfp.readStop();
  messageIn[pldLength]=0x00;
  time_end=micros();

#ifdef DIAG
#if NRF_MODE=='C'
  memset(bufCv,0x00,LBUFCV);
  memcpy(diagMessR,message,pldLength);
  diagMessR[pldLength]='\0';
  strcat(diagMessR," rdSta=");
  sprintf(bufCv,"%-1d",rdSta);
  strcat(diagMessR,bufCv);
  strcat(diagMessR," in ");
  sprintf(bufCv,"%d",(time_end - time_beg));
  strcat(diagMessR,bufCv);
  strcat(diagMessR,"uS");
#endif NRF_MODE=='C'
#endif // DIAG

  return rdSta;
}

void showRx(bool crlf)
{ 
#ifdef DIAG
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" ");
  if(crlf){Serial.println();}
delay(1);
#endif // DIAG  
}

void showErr(bool crlf)
{
#ifdef DIAG
  Serial.println();Serial.print("$ tx=");Serial.print(trSta);Serial.print(" rx=");Serial.print(rdSta);
  Serial.print(" message ");Serial.print((char*)message);
  Serial.print(" lastSta ");if(nrfp.lastSta<0x10){Serial.print("0");}Serial.print(nrfp.lastSta,HEX);
  Serial.print(" ");Serial.print((char*)kk+(rdSta+6)*LMERR);Serial.print(" $");
  if(crlf){Serial.println();}
#endif // DIAG
}

#if NRF_MODE == 'P'

void iniTemp()
{
  //memcpy(thermo,THERMO,LTH);
  thN=THN;
  thSta=true;
  
#ifdef DS18X20  
  checkOn();
  thSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
  thN='B';if(ds1820.dsmodel==MODEL_S){thN='S';}
  readTemp();                               // include checkOff()
#endif DS18X20
}

void readTemp()
{
  if(retryCnt==0){            // pas de conversion si retry en cours
  thSta=true;

  
#ifdef DS18X20
    checkOn();                                // power on
    thSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
    ds1820.convertDs(WPIN);
    sleepPwrDown(TCONVDS1);
    sleepPwrDown(TCONVDS2);   
    temp=ds1820.readDs(WPIN);
    checkOff();                               // power off
/*    Serial.print(volts);Serial.print(" ");Serial.print(nbT);Serial.print(" ");Serial.print(temp);
#ifdef DIAG
Serial.print("/");Serial.print(previousTemp);
#endif // DIAG
    delay(1);
*/       
#endif DS18X20

  }                        
}
  



void sleepDly(uint16_t dly)                                                       // should be (nx250)
{
  delay(1);                     // serial
  dly=(dly/250)*250;
  while(dly>=250){durT+=sleepPwrDown(T250);dly-=250;}
}

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
   
  while(dly>0){
 
    for(int i=0;i<bnb;i++){
      digitalWrite(LED,HIGH);
      pinMode(LED,OUTPUT);
      delay(dur);                 // sleepDly() -> sleepPwrDown() -> hardwarePowerDown() -> pinMode(LED,input)
      digitalWrite(LED,LOW);
      sleepDly(bint);
      dly-=(dur+bint);
    }
    sleepDly(bdelay);
    dly-=bdelay;
  }
}
#endif // NRF_MODE == 'P'


#if NRF_MODE == 'C'

void delayBlk(int dur,int bdelay,int bint,uint8_t bnb,int dly)
/*  dur=on state duration ; bdelay=time between blink sequences ; bint=off state duration ; 
    bnb=(on+off) nb in one sequence ; dly=total delay time   */
{  
  unsigned long tt=millis(); 
  blktime=0;bcnt=1;blkdelay=0;
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);dly-=((dur+bint)*bnb+bdelay);}
}
#endif // NRF_MODE == 'C'

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

void led(unsigned long dur)
{
  digitalWrite(LED,HIGH);
  pinMode(LED,OUTPUT);
  delayMicroseconds(dur);
  digitalWrite(LED,LOW);
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
