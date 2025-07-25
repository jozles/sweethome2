#include <Arduino.h>
#include <shconst2.h>
#include <shutil2.h>
#include "radio_const.h"
#include "radio_util.h"
#include "config.h"
#include "radio_const.h"
#include "radio_powerSleep.h"
#include "radio_user_peri.h"
#include "radio_user_conc.h"

#include "eepr.h"
Eepr eeprom;
 
#ifdef DUE
#include <MemoryFree.h>
#endif // def DUE 

#ifdef NRF
#include "nRF24L01.h"
#include "nrf24l01s.h"
Nrfp radio;
uint8_t radioChannel[]={CHANNEL0,CHANNEL1,CHANNEL2,CHANNEL3};
#endif

#ifdef LORA
#include "LoRa.h"
#include "LoRa_const.h"
LoRaClass radio;
#endif

#ifdef DS18X20
#include <ds18x20.h>
Ds1820 ds1820;
byte     setds[]={0,0x7f,0x80,0x3f},readds[8];   // 1f=93mS 9 bits accu 0,5° ; 3f=187mS 10 bits accu 0,25° !!! TCONVDS to adjust
#define  TCONVDS1 T64   // sleep PwrDown mS !!
#define  TCONVDS2 T125  // sleep PwrDown mS !!
#endif // DS18X20 

#if RADIO_ADDR_LENGTH != RADIO_ADDR_LENGTH
  fail // RADIO_ADDR_LENTH
#endif


/*** LED ***/
uint16_t      blkdelay=0;
uint8_t       bcnt=0;
#define TBLK    1
#define DBLK    2000
#define IBLK    50
#define TMINBLK 500     // uS

unsigned long blktime=millis();

/*** scratch ***/

bool diags=false;

#define TO_READ 30                        // mS rxMessage time out 
long readTo;                              // compteur TO read()
uint32_t      tBeg=0;                     // date unix du 1er message reçu
unsigned long tdiag;                      // cumul des traitements de diag
unsigned long t_on;
unsigned long t_on0;
unsigned long t_on1;
unsigned long t_on2;
unsigned long t_on21;
unsigned long t_on3;
unsigned long t_on4;
unsigned long time_beg=millis();
unsigned long time_end;
unsigned long lastRead=millis();

byte    message[MAX_PAYLOAD_LENGTH+1];    // buffer pour write()
byte    messageIn[MAX_PAYLOAD_LENGTH+1];  // buffer pour read()
int     rdSta;                            // return status read() / available()
int     trSta;                            // return status write() / transmitting()
uint8_t pipe;
uint8_t pldLength;

uint8_t numT=0;                           // numéro périphérique dans table concentrateur

extern float   volts;                     // tension alim (VCC)

#define NTESTAD '1'                       // numéro testad dans table
byte    testAd[]={'t','e','s','t','x',NTESTAD};    // txaddr pour broadcast

#define LMERR 9           
const char*   kk="time out\0tx maxrt\0rx empty\0mac addr\0length  \0pipe nb \0--      \0ok      \0";         // codes retour et erreur

#define ECHO_LEN 10                       // echo message len

#define ECHO_MAC_REQ "UUUU0"              // echo req mac
uint8_t echoNb=0;                         // numéro du périf sur lequel demander l'écho
bool    echoOn=false;                     // fonction echo en cours (sur l'entrée de table 1 envoi de messages pour écho - test de portée)
                                          // le périphérique n'écoutant que les réponses à ses messages, il faut attendre une demande pour commencer.
                                          // en attente, echoOn=true ;
                                          // le concentrateur est bloqué pendant la maneuvre
 
uint8_t channel;

extern byte*  configVers;

#if MACHINE_CONCENTRATEUR

/* >>>> config concentrateur <<<<<< */

/* pointeurs dans l'enregitrement de config */

  extern uint16_t* cfgLen;            // cfg record length

  extern byte*     serverIp;          // server ip addr
  extern uint16_t* serverTcpPort;     // server port
  extern uint16_t* serverUdpPort;     // server udp port

  extern char*     peripass;          // mot de passe périphériques

  extern uint8_t*  concMac;           // macaddr concentrateur (5 premiers caractères valides le 6ème est le numéro dans la table)
  extern byte*     concIp;            // adresse IP concentrateur
  extern uint16_t* concPort;          // port concentrateur
  extern uint8_t*  concRx;            // RX Addr concentrateur
  extern uint16_t* concChannel;       // n° channel utilisé par le concentrateur
  extern uint16_t* concRfSpeed;       // RF_Speed concentrateur
  extern uint8_t*  concNb;

extern uint16_t hostPort;             // server Port (TCP/UDP selon TXRX_MODE)
extern struct ConTable tableC[];      // teble périf
bool menu=true;

char    bufServer[BUF_SERVER_LENGTH]; // to/from server buffer

#define UDPREF 600000                 // période par défaut exportData concentrateur
#define CONCTO 3                      // nbre exportData sans réponse avant autoreset

unsigned long concTime=millis();      // timer pour export de présence vers le serveur
unsigned long perConc=UDPREF;         // période pour export de présence
unsigned long lastUdpCall=millis();   // timer pour TO d'absence de connexion au serveur

uint16_t importCnt=0;
uint16_t etatImport0=0;
uint16_t etatImport1=0;
uint16_t etatImport2=0;
extern uint8_t etatImport;
uint8_t  exportCnt=0;

unsigned long timeImport=0;         // timer pour Import (si trop fréquent, buffer pas plein         
#define PERIMPORT 100

#define LDIAGMESS 80
char    diagMessT[LDIAGMESS];             // buffer texte diag Tx
char    diagMessR[LDIAGMESS];             // buffer texte diag Tx

char bid;

uint32_t ram_remanente __attribute__((section(".noinit")));

extern uint32_t uRScnt;

#endif // MACHINE_CONCENTRATEUR

#if MACHINE_DET328

extern const char*  chexa; //="0123456789ABCDEFabcdef\0";

extern float*    thFactor;
extern float*    thOffset;
extern float*    vFactor;
extern float*    vOffset;
extern byte*     periRxAddr;
extern byte*     concAddr;
extern uint8_t*  concNb;
extern uint8_t*  concChannel;
extern uint8_t*  concSpeed;
extern uint8_t*  concPeriParams;   // provenance des params de calibrage (0 périf ; 1 saisie serveur)

/*** gestion sleep ***/

bool      mustSend;                 // si une condition générant une transmission est présente
bool      forceSend=true;           // pour forcer une transmission en l'absence de condition (par exemple en retry)
int       awakeCnt=0;
int       awakeMinCnt=0;
int       retryCnt=0;
uint32_t  nbS=0;                    // nbre com
uint32_t  nbK=0;                    // nbre com KO
uint32_t  nbL=0;                    // nbre loops
bool      lowPower=false;
float     lowPowerValue=VOLTMIN;

uint16_t  aw_ok=AWAKE_OK_VALUE;
uint16_t  aw_min=AWAKE_MIN_VALUE;
uint16_t  aw_ko=AWAKE_KO_VALUE;
uint8_t   aw_retry=AWAKE_RETRY_VALUE;

float     timer1;
bool      timer1Ovf;
bool      extTimer;
float     period;
unsigned long   absTime=0;
unsigned long   absMillis=0;
unsigned long   periodCnt=0;
int32_t   sleepTime=0;

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
uint16_t  userData[2];
bool  thSta=true;                     // temp validity
char  thermo[]={THERMO};              // thermo name text
char  thN;                            // thermo code for version

void configPrint();
int  beginP(uint8_t pldL);
void echo();
void hardwarePwrUp();
int  txRxMessage(uint8_t pldL);
bool checkTemp();
void int_ISR()
{
  extTimer=true;
  //Serial.println("int_ISR");
}
void prtCom(const char* c){Serial.print(" n°");Serial.print(nbS);Serial.print(c);Serial.print("/");Serial.print(nbK);Serial.print("ko ");delay(2);}
void prtCom(const char* c,int8_t rdSta){prtCom(c);Serial.print("rdSta:");Serial.println(rdSta);delay(2);}
void diagT(char* texte,int duree);
void spvt(){Serial.print(" ");Serial.print(volts);Serial.print("V ");Serial.print(thermo); Serial.print(" ");Serial.print(temp);Serial.print("°C ");delay(4);}
void waitCell();
void getPeriod(){
  Serial.print("period ");delay(1);
  ///*
  unsigned long t_beg;
  unsigned long t_end;
  blink(1);
  while(digitalRead(2)==HIGH){};while(digitalRead(2)==LOW){}; // wait rising edge
  t_beg=micros();
  blink(1);
  while(digitalRead(2)==HIGH){};while(digitalRead(2)==LOW){}; // wait rising edge
  t_end=micros();
  period=(t_end-t_beg);period=period/1000000;
  blink(1);
  //*/
  //period=9.90;
  Serial.print(period*1000);Serial.print("ms ");
}
#endif // MACHINE_DET328

void ini_t_on();
void iniTemp();
void readTemp();
void showErr(bool crlf);
void showRx(bool crlf);
void showRx(byte* message,bool crlf);
void ledblk(int dur,int bdelay,int bint,uint8_t bnb);
void delayBlk(int dur,int bdelay,int bint,uint8_t bnb,long dly);
int  txMessage(bool ack,uint8_t len,uint8_t numP);
int  rxMessage(unsigned long to);
//void echo0(char* message,bool ack,uint8_t len,uint8_t numP);
void blkHS(){delayBlk(2016,0,0,1,1);}                   // hardware ko : 1x2sec blink}
#if MACHINE_CONCENTRATEUR
char getch();
void echo();
void broadcast(char a);
void getEchoNum();

unsigned long radioWd;
uint32_t radioInitCnt=0;
#define NO_RADIO_CX_TO 1250000 // millis() TO for radio cx

void radioInit()
{
  radioWd=millis();
  radioInitCnt++;
  
  radio.powerDown();
  channel=*concChannel;
  if(memcmp(configVers,"01",2)==0){*concNb=1;}
  
  radio.locAddr=concRx;                 // première init à faire !!
  tableCInit();
  memcpy(tableC[1].periMac,testAd,RADIO_ADDR_LENGTH+1);     // pour broadcast & test
  uint8_t speed=*concRfSpeed;
  if(!radio.powerOn(channel,speed,NBPERIF,CB_ADDR)){
    Serial.println("Starting LoRa failed!");
    while(1){blkHS()};                             // hardware ko : 1x2sec blinkblink(1); delay(500);};
  }; 

  Serial.print(" --- radioInit()#");Serial.print(radioInitCnt);Serial.print(' ');Serial.print(millis()-radioWd);Serial.println("ms");

}
#endif // MACHINE_CONCENTRATEUR


void setup() {

#if MACHINE_DET328

  delay(1000);
  Serial.begin(115200);
  Serial.println("\n+");

  initLed(PINLED,LEDOFF,LEDON);
  
  configInit();
  configLoad();

    /* ---- config pour conc 3 ----
  memcpy(periRxAddr,"peri9\0",RADIO_ADDR_LENGTH+1);
  memcpy(configVers,"02\0",3);
  memcpy(concAddr,"SHCO3",RADIO_ADDR_LENGTH);
  *concNb=3;
  *concChannel=radioChannel[*concNb];
  *concSpeed=RF_SPD_1MB; 
  configSave();
    //*/

  configPrint();

  radio.locAddr=periRxAddr;
  radio.ccAddr=concAddr;
  channel=*concChannel;
  //speed=*concSpeed;

  t_on=millis();
  hardwarePwrUp();
  
  wd();                           // watchdog
  iniTemp();
  
  Serial.print("\nStart setup v");Serial.print(VERSION);Serial.print(" ");delay(2);
  radio.printAddr((char*)periRxAddr,0);Serial.print(" to ");radio.printAddr((char*)concAddr,0);
  Serial.print('(');Serial.print(*concNb);Serial.print('-');Serial.print(channel);
  Serial.print('/');Serial.print(*concSpeed);Serial.print(")");
  
#ifndef NOCONFSER
  pinMode(STOPREQ,INPUT_PULLUP);
  if(digitalRead(STOPREQ)==LOW){        // chargement config depuis serveur
      Serial.print("Server Config ");
      getVolts();getVolts();spvt();
      blink(4);
      if(getServerConfig()>5){configSave();}
      configPrint();
      while(1){blink(1);delay(1000);}
  }
#endif // NOCONFSER

  diags=diagSetup(t_on);
  if(diags){
    Serial.println("+ every wake up ; ! mustSend true ; * force transmit (perRefr or retry)");
    Serial.println("€ showerr ; £ importData (received to local) ; $ diags fin loop");delay(10);
  }

  getPeriod();

  getVolts();getVolts();                  // read voltage and temperature (1ère conversion ADC ko)

  /* ------------------- */

  if(diags){spvt();}
  ini_t_on();  

  userResetSetup();

  Serial.println();

//diagT("sleepNoPower à suivre",10);
//sleepNoPwr(T8000);

#endif // MACHINE_DET328

#if MACHINE_CONCENTRATEUR

/*
  dumpstr(ram_remanente,16);
  ram_remanente[LRAMREM-1]='\0';
  memset(ram_remanente,'x',LRAMREM-1);
  
  Serial.println(ram_remanente);
  ram_remanente=1111;
*/
  
  initLed(PINLED,LEDOFF,LEDON);blink(1);  // start WD

  delay(1000);
  Serial.begin(115200);Serial1.begin(115200);

  Serial.println();Serial.print("start setup v");Serial.print(VERSION);
  Serial.print("+ ");Serial.print(TXRX_MODE);Serial.print(" ");

  //while(1){marker(MARKER);delay(2);marker(MARKER2);delay(2);}

#ifdef REDV1
  pinMode(POWCD,OUTPUT);                // power ON shield
  digitalWrite(POWCD,POWON);
  WDTRIG //trigwd(1000000);                      // uS
  delay(100);
#endif // REDV1

/* version avec params de réseau/radio en eeprom */

  configInit();

  blink(2);   

  // !!!!!!!!!!!!!!!!!!! soit l'écriture, soit la lecture de la flash ne fonctionne plus sur la carte due en cours !!!!!!!!!!!!!!!!!!!
  //configCreate();//while(1){}; //digitalRead(STOPREQ)==LOW){blink(1);delay(1000);}
  // ===== test avec la carte marquée TEST_CFG ... ce qui est écrit se relit bien après un power off ===== (jlink branché)

  pinMode(STOPREQ,INPUT_PULLUP);
  if(digitalRead(STOPREQ)==LOW){        // chargement config depuis serveur
      WDTRIG // trigwd();
      blink(4);
      Serial.print(getServerConfig());Serial.print(" ");
      configSave();
      while(digitalRead(STOPREQ)==LOW){blink(1);delay(4000);}
  }

  t_on=millis();
  diags=diagSetup(t_on);

  configCreate();                       // configSave/configLoad ne fonctionnent pas si poweroff !

  configLoad();

#if TXRX_MODE == 'U' 
    hostPort=*serverUdpPort;
#endif // TXRX_MODE U
#if TXRX_MODE == 'T' 
    hostPort=*serverTcpPort;
#endif // TXRX_MODE T

  configPrint();

  WDTRIG //trigwd(0);blktime=millis();

  userResetSetup(serverIp);             // doit être avant les inits radio (le spi.begin vient de la lib ethernet ?)

  WDTRIG //trigwd(0);

  radioInit();                          // inclu : tableCInit()

#ifdef DUE
  Serial.print("free=");Serial.print(freeMemory(), DEC);Serial.println(" ");
#endif //  

  time_beg=millis();
  //while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}          // 0,8sec (4 blink)
  blink(2);
   
  exportDataMail("START",4);
  blink(4);

  Serial.println();

  lastUdpCall=millis();
#endif // MACHINE_CONCENTRATEUR


  Serial.println("end setup\n");delay(1);

}

void loop() {

#if MACHINE_DET328

  /* un blink environ 1.2 mS lecture volts si réveil avec com (selon awake...Cnt etc)
     si com ok 4 blinks supplémentaires */

  if(lowPower){lethalSleep();}

  if(diags){  
    t_on4=micros();
    Serial.print("$ ");
    Serial.print(awakeMinCnt);Serial.print(" / ");Serial.print(awakeCnt);Serial.print(" / ");Serial.print(retryCnt);Serial.print(" ; ");
    Serial.print(volts);Serial.print("V ");
    Serial.print(deltaTemp);Serial.print(":");
    Serial.print(temp);Serial.print("/");Serial.print(previousTemp);Serial.print("° t(");
    Serial.print(t_on1-t_on);Serial.print("/");
    Serial.print(t_on2-t_on);Serial.print("/");
    Serial.print(t_on21-t_on);Serial.print("/");
    Serial.print(t_on3-t_on);Serial.print("/");
    Serial.print(micros()-t_on+5000+1000+1000);Serial.print("/diag(uS)="); // 1mS pour 4xSerial.print
    delay(5);
    tdiag+=micros()-t_on4+1000;
    Serial.print(tdiag);Serial.println(") $");
    delay(1);
  }            

  /* timing to usefull awake */
  while(((awakeMinCnt>0)&&(awakeCnt>0)&&(retryCnt==0))){
    awakeCnt--;
    awakeMinCnt--;
    sleepNoPwr(0);
    nbL++;
    periodCnt++;
    
    if(diags){
      Serial.print("+");delayMicroseconds(200);
      /*
      if(periodCnt==1 && (nbS&0xfff)==0){                // update period every 0xfff com
        Serial.println();Serial.print(period*1000);Serial.print(' ');
        getPeriod();periodCnt+=2;}
      */
    }
  }

  marker(MARKER);
  ini_t_on();
  sleepTime=0;

  /* usefull awake or retry */
  digitalWrite(PLED,HIGH);
  getVolts();                                 // 1.2 mS include notDS18X20 thermo reading and low voltage check (not blocking)                                   
  digitalWrite(PLED,LOW);
  readTemp();                                 // only for DS18X20
  awakeCnt=aw_ok;

  mustSend=false;
  if( radio.lastSta!=0xFF &&                   // hardware HS or missing no com
      ( checkThings(awakeCnt,awakeMinCnt,retryCnt) || 
        checkTemp() || 
        forceSend || 
        awakeMinCnt<=0 || 
        retryCnt!=0 
      )
    ){mustSend=true;}
  
  t_on1=micros();    // end of work... now send or sleep

  /* hardware ok, data ready or presence message time or retry -> send */
  if(mustSend){
    if(diags){
      unsigned long localTdiag=micros();    
      Serial.print("!");
      for(int nb=retryCnt;nb>0;nb--){Serial.print("*");}
      tdiag+=(micros()-localTdiag);
    }
    /* building message MMMMMPssssssssVVVVU.UU....... MMMMMP should not be changed */
    /* MMMMM mac P periNb ssssssss Seconds VVVV version U.UU volts ....... user data */
    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);memset(message,0x20,RADIO_ADDR_LENGTH+1);

    uint8_t outLength=RADIO_ADDR_LENGTH+1;                            // perinx     - 6
    memcpy(message+outLength,VERSION,LENVERSION);                     // version    - 4
    outLength+=LENVERSION;
    memcpy(message+outLength,&thN,1);                                 // modèle thermo ("B"/"S" DS18X20 "M"CP9700  "L"M335  "T"MP36    
    outLength++;                                                      //            - 1

    //sprintf((char*)(message+outLength+1),"%08lu",(uint32_t)tBeg);     // first connection unix time
    //outLength+=9;                                                     //            - 9
 
    messageBuild((char*)message,&outLength);                          // add user data 
    memcpy(message,periRxAddr,RADIO_ADDR_LENGTH);                     // macAddr
    message[RADIO_ADDR_LENGTH]=numT+48;                               // numéro du périphérique
    message[outLength]='\0';

    if(outLength>MAX_PAYLOAD_LENGTH){ledblink(BCODESYSERR,PULSEBLINK);}
    if(diags){Serial.print("\n (");Serial.print(outLength);Serial.print(")");Serial.println((char*)message);}delay(5);
    
    /* One transaction is tx+rx ; if both ok reset counters else retry management*/  

    rdSta=-1;
    //nbS++;

    if(numT!=0 || (absTime!=0 && absMillis!=0)){waitCell();}
    
    t_on2=micros();                                       // message build ... send   
    if(!radio.powerOn(channel,*concSpeed,NBPERIF,CB_ADDR)){
      blkHS();                               // hardware ko : 1x2sec blink
    };    // si waitCell rallonge
    trSta=0;
    marker2(MARKER);
    rdSta=txRxMessage(outLength);                         // retour -2 ko ; >0 ok
    radio.powerOff();  
    t_on21=micros();
    
    if(rdSta>=0){                                         // no error
      marker(MARKER);
      prtCom(" ok",rdSta);
      
      /* echo request ? (address field is 0x5555555555) */
      if(memcmp(messageIn,ECHO_MAC_REQ,RADIO_ADDR_LENGTH)==0){echo();}
      else {                                      
        importData(messageIn,pldLength);                  // user data
      }
        
      /* tx+rx ok -> reset every counters */
      forceSend=false;
      retryCnt=0;
      awakeCnt=aw_ok;
      awakeMinCnt=aw_min;

      delayBlk(32,0,96,4,1);                             // txRx ok : 4 blinks
    }
    
    t_on3=micros();  // message sent / received or error (rdSta)

    if(trSta<0 || rdSta<0){                               // error
    
      nbK++;
      prtCom(" ko",rdSta);
      forceSend=true;
      trSta=0;rdSta=0;
      showErr(true);
      numT=0;                                             // tx or rx error : refaire l'inscription au prochain réveil
      switch(retryCnt){                                   
        case 0:retryCnt=aw_retry;break;                   // retryCnt ==0 -> debut des retry
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;          // retryCnt ==1 -> ko (les retry n'ont pas réussi) long delay avant reprise (aw_ko sleeps)
               retryCnt=0;break;
        default:retryCnt--;break;                         // retryCnt >1  -> retry en cours pas de sleep
      }
                                                          // txRx KO pas de blink
    }
    //================================ version sans retry ===========================
    retryCnt=0;awakeCnt=aw_ok;awakeMinCnt=aw_min;
  }
  
  /* if radio HS or missing ; 1 long blink every 2 sleeps */
  if(radio.lastSta==0xFF){
    if(diags){
      unsigned long localTdiag=micros();    
      delay(2);Serial.println("radio HS/missing");delay(4);
      tdiag+=(micros()-localTdiag);
    }
    blkHS();                               // hardware ko : 1x2sec blink
    retryCnt=0;
    awakeCnt=1;
    awakeMinCnt=1;            
  }
#endif // MACHINE_DET328

#if MACHINE_CONCENTRATEUR

  if(menu){
    Serial.print("     ");
    radio.printAddr((char*)radio.locAddr,0);
    Serial.print(" init#");Serial.print(radioInitCnt);
    Serial.println(" (e)cho (b)roadcast (t)ableC (s)erver (q)uit");
    menu=false;
  }

  ledblk(TBLK,3000,IBLK,1);
  
  if((millis()&0x1ff)==0){
    marker(MARKER);
  }

  if((millis()-lastUdpCall)>UDPREF && exportCnt>=3){//CONCTO*perConc)){
    Serial.print(millis());Serial.print(" pas reçu de cx udp (valide) depuis plus de ");Serial.print(UDPREF/1000); //(CONCTO*perConc)/1000);
    Serial.println("sec - reset ");
    userResetSetup(serverIp,(char*)"UDP TO");}  //forceWd();}

  numT=0;                                               // will stay 0 if no registration
  pldLength=MAX_PAYLOAD_LENGTH;                         // max length
  memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
  rdSta=get_radio_message(messageIn,&pipe,&pldLength);  // get message from perif 
                                                        // <0 err ; 0 et pipe==1 reg to do ;
                                                        // 0 et pipe==2 conc radio Addr req ; >0 entry nb

  time_beg=micros();  

  //if(rdSta!=AV_EMPTY){lastRead=millis();}

  if((rdSta<0 && rdSta!=AV_EMPTY) || rdSta>=0){                    
    showRx(messageIn,false);
    if(rdSta<0 && rdSta!=AV_EMPTY){    
      showErr(true);}
    else if(rdSta>0){Serial.println();}
  }

  if(rdSta>=0){
    //marker(MARKER2);
    radioWd=millis();
  }                       // >=0 pas d'erreur

  if(rdSta==0){                                         // ==0 reg to do (pipe==1) or concAddr req (pipe=2)                      
    
  // ====== no error registration request or conc macAddr request (pipe 2) ======

      //radio.printAddr((char*)messageIn,0);
      //showRx(messageIn,false);                                        

      if(pipe==1){                                      // registration request 
        numT=cRegister((char*)messageIn,pldLength);     // retour NBPERIF full sinon N° perif dans tableC (1-n)
        if(numT<(NBPERIF)){                             // registration ok
          rdSta=numT;                                   // entry is valid -> rdSta >0              
          if(diags){Serial.print(" reg as ");Serial.print(numT);}         // numT = 0-(NBPERIF-1) ; rdSta=numT
        }
        else {if(diags){Serial.println(" full");}}                        // numT = NBPERIF   ; rdSta=0
      }
      if(pipe==2){                                                        // conc macAddr request
        memcpy(tableC[NBPERIF].periMac,messageIn,RADIO_ADDR_LENGTH+1);    // peri addr in table last entry
        memcpy(message,messageIn,RADIO_ADDR_LENGTH+1);
        memcpy(message+RADIO_ADDR_LENGTH+1,radio.locAddr,RADIO_ADDR_LENGTH);  // build message to perif with conc macAddr
        //dumpstr((char*)tableC,128);
        
        txMessage(ACK,MAX_PAYLOAD_LENGTH,NBPERIF);                        // end of transaction so auto ACK
        // le numéro de périf est NBPERIF car son adresse mac (pour read) est copiée dans la dernière entrée de table
      }                                                                   // rdSta=0 so ... end of loop
  }

  // numT=0 si pipe==2 sinon pipe==1 : registration
  // ====== no error && valid entry (rdSta=n° de perif fourni par le perif ou à l'issue de cRegister) ======         
  

  if((rdSta>0)                                          // >0 valid entry from registred perif
    && (memcmp(messageIn,tableC[rdSta].periMac,RADIO_ADDR_LENGTH)==0)){    // verif de l'adresse mac
                                                        // rdSta is table entry nb
      if(numT==0 && (echoNb!=rdSta || !echoOn)){        // numT=0 means : that is not a registration, and message is not an echo answer 
                                                        // so -> incoming message storage
        memcpy(tableC[rdSta].periBuf,messageIn,pldLength);
        memcpy(tableC[rdSta].periMac,messageIn,RADIO_ADDR_LENGTH);    
        tableC[rdSta].periBufLength=pldLength;
      }
      
      if(echoNb!=rdSta){                                // if no echo pending on this table entry
  /* build config */
        memcpy(message,messageIn,RADIO_ADDR_LENGTH+1);
        memcpy(message+RADIO_ADDR_LENGTH+1,tableC[rdSta].servBuf,MAX_PAYLOAD_LENGTH-RADIO_ADDR_LENGTH-1);    
                                                        // build message to perif with server data
                                                        // server data is MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
                                                        // see importData()
        if(memcmp(tableC[rdSta].periBuf+RADIO_ADDR_LENGTH+1,"2.c",3)>=0){     // ajout temps absolu cellules
          fillMess(message); 
        }
  /* send it to perif */    
        txMessage(NO_ACK,MAX_PAYLOAD_LENGTH,rdSta);      // end of transaction so auto ACK ; rdSta table entry nb
                                                      // NO_ACK pour cellules temporelles (pa se répétitions)
        message[MAX_PAYLOAD_LENGTH]='\0';Serial.print(" ");Serial.println((char*)message);
        // ******************************* réponse passée **********************************
        if(trSta==0){tableC[rdSta].periBufSent=true;} // trSta status transmission ; si ok le perif est à jour
  /* ======= formatting & tx to server ====== */
        if(numT==0){
          exportData(rdSta);}                         // numT==0 if perif already had registration nb
      }                                               // numT!=0 if perif only made registration request (no data)
      else {                                          
      /* echo pending  :                         // echoOn flag d'attente de réponse 
        if(!echoOn){sendEchoReq();echoOn=true;}  // sendEchoReq gère la tempo
        else {echOn=false; controle temps et réponse ; affichage de la réponse}
      */
        echo();
      }
  }
  
  // ====== error, full or empty -> ignore ======
  // peripheral must re-do registration so no answer to lead to rx error

  if(diags){
    if(rdSta>=0){
      Serial.print(" rx+tx");if(numT==0){Serial.print("+export");}
      Serial.print(" (");
      Serial.print(rdSta);Serial.print(")=");Serial.println(micros()-time_beg);}  // pas d'erreur, un cycle complet a été effectué
    else if(rdSta!=AV_EMPTY){Serial.print(" radio err ");Serial.print(rdSta);Serial.print('=');Serial.println(micros()-time_beg);}
  }

  // ====== RX from server ? ====  
  // importData returns MESSOK(ok)/MESSCX(no cx)/MESSLEN(len=0);MESSNUMP(numPeri HS)/MESSMAC(mac not found)

    int dt=importData();importCnt++;

    if(dt==MESSNUMP){tableC[rdSta].numPeri=0;Serial.print('+');}
    if(dt!=MESSLEN && dt!=MESSOK){Serial.print(" importData failure:");Serial.println(dt);}    // MESSLEN until well completed

    //if(dt!=MESSLEN){Serial.print(" ----------------- importData ");Serial.println(dt);}
#ifdef DIAG
  if((dt==MESSMAC)||(dt==MESSNUMP)){Serial.print(" importData=");Serial.print(dt);Serial.print(" bS=");Serial.println(bufServer);}
#endif // DIAG

  // ====== si rien reçu des périfs et rien du serveur, éventuel message de présence ====

  //if(rdSta==AV_EMPTY && (dt==MESSCX || dt==MESSLEN)){       // pas de réception valide ni importData
    
    if((millis()-concTime)>=perConc){
      //if(perConc<600000){Serial.println("perConc");while(1){};}   
      concTime=millis();exportDataMail(nullptr);    
      /*
      Serial.print(" importCnt:");Serial.print(importCnt);importCnt=0;Serial.print(" ");
      Serial.print(" etatImport0:");Serial.print(etatImport0);etatImport0=0;Serial.print(" ");
      Serial.print(" etatImport1:");Serial.print(etatImport1);etatImport1=0;Serial.print(" ");
      Serial.print(" etatImport2:");Serial.print(etatImport2);etatImport2=0;Serial.print(" ");
      Serial.print(concTime-lastRead);Serial.println(" Exp_conc");exportDataMail();}
      */
    }
  //} 
  
    if((millis()-radioWd)>NO_RADIO_CX_TO){    // wd radio
      Serial.print("pas de cx radio depuis ");Serial.print(NO_RADIO_CX_TO/1000);Serial.println("sec - re_init ");delay(10);
      configLoad();
      Serial.print('@');
      radioInit();
    }

// ====== menu choice ====== 
  char a=getch();
  switch(a){
    case 'e':getEchoNum();menu=true;break;
    case 'b':broadcast('b');menu=true;break;
    case 't':Serial.println((char)a); dumpstr((char*)tableC,256);tableCPrint();menu=true;break;
    case 's':Serial.println((char)a);exportDataMail(nullptr);break;
    case 'q':Serial.print((char)a);echoNb=0;echoOn=false;menu=true;break;
    default:break;
  }
  
#endif // MACHINE_CONCENTRATEUR

} /******************** loop end  *******************/

#if MACHINE_CONCENTRATEUR

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
    memcpy(message,ECHO_MAC_REQ,RADIO_ADDR_LENGTH);
    sprintf((char*)message+RADIO_ADDR_LENGTH,"%05d",cnt);
    message[ECHO_LEN]='\0';
    Serial.print("sent ");Serial.print((char*)message);

    /* send request */
    time_beg = micros();
    radio.write(message,NO_ACK,ECHO_LEN,tableC[echoNb].periMac);
    trSta=1;
    while(trSta==1 && (micros()-time_beg)<ECHOTO){
      trSta=radio.transmitting(NO_ACK);}                 // trsta=0 if data sent ok ; -1 if maxRt
    if(trSta<0){Serial.println("********* maxRT ");break;}
    time_end = micros();
    
    /* wait for answer until receipt ok or TO or err */
    memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
    rdSta=AV_EMPTY; 
           
    while(rdSta==AV_EMPTY && (micros()-time_end)<ECHOTO){
      rdSta=get_radio_message(messageIn,&pipe,&pldLength);

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
      radio.printAddr((char*)tableC[1].periMac,0);Serial.println(" ; !=toogle ack ; q=exit)");}
   
//    while(Serial.available()){Serial.print(getch());}
    b=getch();
    switch (b){
      case 0:break;
      case '!':ack=!ack;titre=true;break;
      case 'q':return;
      default:
        if(b!=' '){
          testAd[0]=b;delay(10);for(int i=1;i<RADIO_ADDR_LENGTH;i++){testAd[i]=getch();}
          Serial.println((char*)testAd);
          memcpy(tableC[1].periMac,testAd,RADIO_ADDR_LENGTH);
          tableCPrint();}
        memcpy(message,testAd,RADIO_ADDR_LENGTH);
        message[RADIO_ADDR_LENGTH]='0';
        sprintf((char*)(message+RADIO_ADDR_LENGTH+1),"%08lu",millis());
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

#endif // MACHINE_CONCENTRATEUR

#if MACHINE_DET328

int beginP(uint8_t pldL)                        // manage registration ; output value >0 is numT else error with radio.powerOff()
{                               
  if(diags){
    unsigned long localTdiag=micros();
    Serial.print("beginP ");
    tdiag+=(micros()-localTdiag);
  }
  nbS++;
  int confSta=-1;
  int8_t beginP_retryCnt=2; // 1; // ================================================= version avec retry 
  memcpy(messageIn,message,pldL);
  pldLength=pldL;
  
  while(beginP_retryCnt>0){                         // confsta>=1 or -5 or wait     
    beginP_retryCnt--;
    confSta=radio.pRegister(messageIn,&pldLength);  // -5 maxRT ; -4 empty ; -3 mac ; -2 len ; -1 pipe ;
                                                    // 0 na ; >=1 ok numT                 
    if(diags){          
      unsigned long localTdiag=micros();
      Serial.print('#');Serial.print(beginP_retryCnt);Serial.print(':');Serial.print(confSta);Serial.print(' ');
      tdiag+=(micros()-localTdiag);}    // après pReg pour que la sortie n'interfère pas avec les tfr SPI                                 
    
    if(confSta>0){
      awakeMinCnt=-1;                   // force data upload
      beginP_retryCnt=0;                // ok ... no retry
    }

    if(confSta==-5){
      radio.lastSta=0xFF;               // KO mode : radio missing or HS
      beginP_retryCnt=0;                // ko ... no retry
    }
  }                                     // next attempt

  if(diags){
    unsigned long localTdiag=micros();    
    Serial.println();delay(1);
    tdiag+=(micros()-localTdiag);
  }
  
  return confSta;                     // peripheral registered or radio HS
}

void echo()
{
#define MAXECHOERR 3                  // max consecutive errors to leave
#define ECHOTO  10000                 // TO mS

  uint8_t cntErr=0;
  unsigned long to=ECHOTO;
  char echoRef[RADIO_ADDR_LENGTH+1];
  memcpy(echoRef,radio.locAddr,RADIO_ADDR_LENGTH);
  echoRef[RADIO_ADDR_LENGTH]='0'+numT;
  byte echoMess[ECHO_LEN+1];
  
 // Serial.println("start echo");

  radio.powerOn(channel,*concSpeed,NBPERIF,CB_ADDR);
  
  while(1){

    /* send echo */
    memcpy(echoMess,messageIn,ECHO_LEN);
    memcpy(echoMess,echoRef,RADIO_ADDR_LENGTH+1);
    echoMess[ECHO_LEN]='\0';
    radio.write(echoMess,NO_ACK,ECHO_LEN,nullptr);
    trSta=1;
    while(trSta==1){
      trSta=radio.transmitting(NO_ACK);}                     // trsta=0 if data sent ok ; -1 if maxRt

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
      rdSta=radio.read(messageIn,&pipe,&pldLength,NBPERIF);
      if((micros()-time_beg)/1000>to){rdSta=ER_RDYTO;time_beg=micros();}
      if(rdSta<0 && rdSta!=AV_EMPTY){showErr(true);cntErr++;}                
    }

    /* if max err or no req mess leave */
    if(cntErr>=MAXECHOERR || memcmp(messageIn,ECHO_MAC_REQ,4)!=0 ){break;}
    else cntErr=0;
  }
  radio.powerOff();
  Serial.println("stop echo");delay(1);
}

void waitCell()                             // attente cellule temporelle
{     
      
  if(absTime!=0){
      int32_t delta1=absMillis+(CELLSIZE-absTime-4);
      int32_t delta2=((int32_t)(periodCnt*period*1000)-delta1)%CELLSIZE+11;
      int32_t ter=numT; if(ter!=0){ter--;}
      int32_t dly=CELLSIZE-delta2-POWONDLY-20+ter*CELLDUR; //+3;

      if(dly<0){dly=0;}
      if(dly>CELLSIZE){dly-=CELLSIZE;}
      if(diags){
        Serial.print(" delta2:");Serial.print(delta2);
        Serial.print(" dly:");Serial.print(dly);delay(2);
      }
      
      marker2(MARKER);
      markerLow(MARKER2);          // la durée entre les 2 markers doit être == tmicros2+slpt0  
      delay(sleepDly(dly,&sleepTime));
      markerLow(MARKER2);          // la durée entre les 2 markers doit être == tmicros2+slpt0

      if(diags){
        Serial.print(" absMillis:");Serial.print(absMillis);
        Serial.print(" absTime:");Serial.print(absTime);
        Serial.print(" delta2:");Serial.print(delta2);
        Serial.print(" dly:");Serial.println(dly);
      }
  }
  
}

int txRxMessage(uint8_t pldL)       // utilise beginP : doit avoir message[] chargé avec au moins adresseMac et version
{                                   // pour que le concentrateur renvoie le bon format de données
  if(numT==0){

    int8_t bp=beginP(pldL);
    if(bp<=0){return -2;}
    numT=bp;
    return bp;
  }

  message[RADIO_ADDR_LENGTH]=numT+48;
  memcpy(messageIn,message,pldLength);
  nbS++;
  return radio.txRx(messageIn,&pldLength);
}

#endif // MACHINE_DET328

#if MACHINE_CONCENTRATEUR

int txMessage(bool ack,uint8_t len,const uint8_t numP)  // retour 0 ok ; -1 maxRt ; -2 registration ko
{
#if MACHINE_DET328  
  /*     TxMessage n'est pas utilisé par MACHINE_DET328   --- pour modifier, message[] doit être préchargé avec au moins la version en plus de 
  //                                                          l'adresse mac pour que le concentrateur renvooie le bon format de données si numT=0
  if(numT==0){
    numT=beginP();
    if(numT<=0){trSta=-2;return trSta;}                 // beginP n'a pas fonctionné
  }
  message[RADIO_ADDR_LENGTH]=numT+48;
  radio.write(message,ack,len,(byte*)&numP);            // send message
  */
#endif // MACHINE_DET328

  radio.write(message,ack,len,tableC[numP].periMac);    // send message

  trSta=1;
  time_beg = micros();
  while(trSta==1){trSta=radio.transmitting(ack);}       // wait for ack from dest ; trsta=0 if data sent ok ; -1 if maxRt

  time_end=micros();

  if(diags){
  #define LBUFCV 7
    char    bufCv[LBUFCV];                              // buffer conversion sprintf
   
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
    sprintf(bufCv,"%ld",(time_end - time_beg));
    strcat(diagMessT,bufCv);
    strcat(diagMessT,"uS");
  }
  
  return trSta;
}
#endif // MACHINE_CONCENTRATEUR

int rxMessage(unsigned long to) // retour rdSta=ER_RDYTO TO ou sortie de available/read (0=full, pipe err, length err)
{
  time_beg = micros();
  if(to==0){to=TO_READ;}
  //memset(messageIn,0x00,MAX_PAYLOAD_LENGTH+1);
  pldLength=MAX_PAYLOAD_LENGTH;                    // max length
  rdSta=AV_EMPTY;
  readTo=0;
  while((rdSta==AV_EMPTY) && (readTo>=0)){
    rdSta=get_radio_message(messageIn,&pipe,&pldLength);

    readTo=to-(micros()-time_beg)/1000;}    
  if(readTo<0){rdSta=ER_RDYTO;}
  // rdSta=AV_EMPTY ou 0:full ou >0:numPer ou <0:pipe_err,length_err,mac_addr_table_err
  radio.readStop();
  messageIn[pldLength]=0x00;
  time_end=micros();

  #if MACHINE_CONCENTRATEUR
  if(diags){
  #define LBUFCV 7
    char    bufCv[LBUFCV];                    // buffer conversion sprintf

    memset(bufCv,0x00,LBUFCV);
    memcpy(diagMessR,message,pldLength);
    diagMessR[pldLength]='\0';
    strcat(diagMessR," rdSta=");
    sprintf(bufCv,"%-1d",rdSta);
    strcat(diagMessR,bufCv);
    strcat(diagMessR," in ");
    sprintf(bufCv,"%ld",(time_end - time_beg));
    strcat(diagMessR,bufCv);
    strcat(diagMessR,"uS");
  }
  #endif // MACHINE_CONCENTRATEUR
  return rdSta;
}

void showRx(byte* message,bool crlf)
{ 
  if(diags){
    Serial.print(millis());
    Serial.print(' ');
    Serial.print(rdSta);
    Serial.print(" reçu l=");Serial.print(pldLength);
    Serial.print(" p=");Serial.print(pipe);
    Serial.print(" ");
    if(message!=nullptr){Serial.print((char*)message);}
    if(crlf){Serial.println();}
    //delay(2);
  }
}

void showRx(bool crlf){
  showRx(nullptr,crlf);
}

void showErr(bool crlf)
{
if(diags){
#if  MACHINE_DET328
  Serial.println();
#endif // MACHINE_DET328
  Serial.print("€ tx=");Serial.print(trSta);Serial.print(" rx=");Serial.print(rdSta);
#if  MACHINE_DET328
  Serial.print(" message ");Serial.print((char*)message);
  delay(3);
  Serial.print(" lastSta ");if(radio.lastSta<0x10){Serial.print("0");}Serial.print(radio.lastSta,HEX);
  delay(2);
#endif // MACHINE_DET328
  Serial.print(" ");Serial.print((char*)kk+(rdSta+6)*LMERR);Serial.print(" €");
  if(crlf){Serial.println();}
  delay(2);
}  
}

void ini_t_on()
{
  tdiag=0;
  t_on=micros();
  t_on1=t_on;
  t_on2=t_on;
  t_on21=t_on;  
  t_on3=t_on;
  t_on4=t_on;  
}

#if MACHINE_DET328

void iniTemp()
{
  //memcpy(thermo,THERMO,LTH);
  thN=THN;
  thSta=true;
  previousTemp=0;
  
#ifdef DS18X20  
  checkOn();
  thSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
  thN='B';if(ds1820.dsmodel==MODEL_S){thN='S';}
  readTemp();                               // include checkOff()
#endif // DS18X20
}

void readTemp()
{
  if(retryCnt==0){            // pas de conversion si retry en cours
  thSta=true;
 
#ifdef DS18X20
    checkOn();                                // power on
    thSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
    ds1820.convertDs(WPIN);
    sleepNoPwr(TCONVDS1);
    sleepNoPwr(TCONVDS2);   
    temp=ds1820.readDs(WPIN);
    checkOff();                               // power off
#endif // DS18X20
  }                        
}

void prt(const char* d)
{
    Serial.print(deltaTemp);Serial.print(":");
    Serial.print(temp);Serial.print(d);Serial.print(previousTemp);
    delay(1);
}

bool checkTemp()
{
  if( temp>(previousTemp+deltaTemp) ){                                
    //prt(">");
    previousTemp=temp-(deltaTemp/2);
    return true;}
  else if( temp<(previousTemp-deltaTemp) ){
    //prt("<");
    previousTemp=temp+(deltaTemp/2);
    return true;}
  return false;
}


void delayBlk(int dur,int bdelay,int bint,uint8_t bnb,long dly)
/*  dur=on state duration ; bdelay=time between blink sequences ; bint=off state duration ; 
    bnb=(on+off) nb in one sequence ; dly=total delay time   
    
    delays are in sleepPwrDown() mode
    dur,bint, bdelay must be n*32mS 
    if dur<32 -> delay(dur) no sleep     
    at least 1 bdelay is executed (even dly smaller)
    usable if dly > (dur+bint)*bnb+bdelay
    hardwarePowerDown() at beginning of every sleepPwrDown() ; hardwarePowerUp() at end of delayBlk

    exemples :

    delayBlk(1,0,250,1,5000);         // 5sec blinking (1/250)
    delayBlk(1,0,250,3,1);            // 3 blinks (1/250)
    delayBlk(300,0,0,1,1);            // 1 pulse (300)
    
*/    
{
  while(dly>0){
    for(int i=0;i<bnb;i++){
      digitalWrite(PLED,HIGH);
      pinMode(PLED,OUTPUT);
      delay(sleepDly(dur,&sleepTime));
      /*if(dur<DLYSTP){delay(dur);}       // sleepPwrDown is about 10mAmS ; awake is about 4mA => no reason to sleep if dur<3mS
                                        // for 32mS sleep, power saving is greater than 90%
      else {sleepDly(dur,&sleepTime);}*/
      digitalWrite(PLED,LOW);
      if(bint!=0){
        delay(sleepDly(bint,&sleepTime));}    // 1 blink doesnt need bint
      dly-=(dur+bint);
    }
    if(bdelay!=0){
      delay(sleepDly(bdelay,&sleepTime));dly-=bdelay;}
  }
}

#endif // MACHINE == 'P'


#if MACHINE_CONCENTRATEUR

void delayBlk(int dur,int bdelay,int bint,uint8_t bnb,unsigned long dly)
/*  dur=on state duration ; bdelay=time between blink sequences ; bint=off state duration ; 
    bnb=(on+off) nb in one sequence ; dly=total delay time   */
{  
  unsigned long tt=millis(); 
  blktime=0;bcnt=1;blkdelay=0;
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);dly-=((dur+bint)*bnb+bdelay);}
}
#endif // MACHINE_CONCENTRATEUR

void ledblk(int dur,int bdelay,int bint,uint8_t bnb)
{   // dur = durée on ; bdelay = delay entre séquences ; bint = intervalle entre blinks ; bnb = nbre blinks
  uint16_t blkdur=millis()-blktime;
  if(blkdur>blkdelay){
    if(digitalRead(PLED)==LOW){
      digitalWrite(PLED,HIGH);
      if(blkdur>(blkdelay+1000)){Serial.print("================== blink overrun ");Serial.println(blkdur);}
      blkdelay=dur;}
    else{digitalWrite(PLED,LOW);
      if(bcnt<bnb){blkdelay=bint;bcnt++;}
      else{blkdelay=bdelay;bcnt=1;}}
    blktime=millis();
  }
}

void led(unsigned long dur)
{
  digitalWrite(PLED,HIGH);
  pinMode(PLED,OUTPUT);
  delayMicroseconds(dur);
  digitalWrite(PLED,LOW);
}

void diagT(char* texte,int duree)
{
    Serial.println(texte);delay(duree*1000);
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
