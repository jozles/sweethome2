//#define TEST_CSE

#ifndef ESP32
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <WiFi.h>
#endif
#include <shconst2.h>
#include <shutil2.h>
#include <shmess2.h>
#include <ds18x20.h>
#include "const.h"
#include "utilWifi.h"
#include "util.h"
#include "dynam.h"
#include "peripherique2.h"

#ifdef PWR_CSE7766
#include <CSE7766.h>
CSE7766 myCSE7766;
#define CSEUPDATE 2000
uint8_t powSw;
#endif // CSE7766


#ifdef MAIL_SENDER
#define MAIL_CONFIG
#include <EMailSender.h>
EMailSender* emailSend=nullptr;
EMailSender::EMailMessage message;                            // STORAGE_SD doit etre "ndef"

#ifdef MAIL_CONFIG
#define FROM_MAIL_LEN 64
#define FROM_PASS_LEN 64
char fromMail[FROM_MAIL_LEN];
char fromPass[FROM_PASS_LEN];
#endif // MAIL_CONFIG

#ifndef MAIL_CONFIG
char* fromMail={"alain66p@gmail.com"};
char* fromPass={"bncfuobkxmhnbwgi"};  // old="uuunclajxtrabnpj"
#endif // MAIL_CONFIG

#endif // MAIL_SENDER

#ifndef ESP32
extern "C" {                  
#include <user_interface.h>                 // pour struct rst_info, system_deep_sleep_set_option(), rtc_mem
}
#endif

#if CONSTANT==EEPROMSAVED
#include <EEPROM.h>
#endif

#ifdef CAPATOUCH
#include <capaTouch.h>          // décommenter le path dans platformio.ini
uint8_t keys[]={KEY2,KEY1};
Capat capaKeys;
#endif

#ifdef ANALYZE
// !!!!!!!!!! vérifier que les GPIOS utilisés pour l'analyseur sont disponibles !!!!!!!!!!
#define AN0 ANPIN0  // 13 si VRR !!!!!!!!!!!!!!!!!!!! vérifier sa disponibilité !!!!!!!!!!
#define AN1 ANPIN1  // 16 si VRR
#define AN2 ANPIN2  // 10 si VRR
// toujours un LOW en premier ppour eviter les glitchs à 1 sur la sortie de la porte 
#define SRVAV   digitalWrite(AN2,LOW);digitalWrite(AN1,HIGH);digitalWrite(AN0,HIGH);  // 110 ordrext server.available
#define RCVEND  digitalWrite(AN0,LOW);digitalWrite(AN1,HIGH);digitalWrite(AN2,HIGH);  // 011 ordrext receive end
//#define MESTOS  digitalWrite(AN0,LOW);digitalWrite(AN1,LOW;digitalWrite(AN2,HIGH);    // 001 messToServer     ! DEFINI dans shconst
//#define GHTTPR  digitalWrite(AN0,LOW);digitalWrite(AN1,LOW);digitalWrite(AN2,LOW);    // 000 getHttpResponse  ! DEFINI dans shconst
#define FORCV   digitalWrite(AN1,LOW);digitalWrite(AN0,HIGH);digitalWrite(AN2,HIGH);  // 101 ordrext checks end ; start rec fonct managing
#define ANSW    digitalWrite(AN2,LOW);digitalWrite(AN1,LOW);digitalWrite(AN0,HIGH);   // 100 ordrext answer
#define ANSWE   digitalWrite(AN2,LOW);digitalWrite(AN1,HIGH);digitalWrite(AN0,LOW);   // 010 ordrext answer end
#define STOPALL digitalWrite(AN0,HIGH);digitalWrite(AN1,HIGH);digitalWrite(AN2,HIGH); // end
#endif // ANALYZE

unsigned long lastRefr=0;
uint8_t answerCnt=0;

Ds1820 ds1820;
//extern byte dsmodel;

  char model[LENMODEL];

  unsigned long dateOn=millis();          // awake start time
  unsigned long boucleTime=millis();

  const char* ssid;                       // current ssid
  const char* ssidPwd;                    // current ssid pwd 
  char ssidNb=1;                          // numéro ssid courant

//*/
//  const IPaddr* host//="192.168.0.35";  //= HOSTIPADDR2;         // HOSTIPADDRx est une chaine de car donc de la forme "192.168.0.xxx"
//  const int   port=1786;  //= PORTPERISERVER2; 

WiFiClient cli;                           // instance du serveur externe (utilisé pour dataread/save)

#ifdef  _SERVER_MODE
WiFiClient cliext;                        // instance du serveur local
WiFiServer* server=nullptr;
bool serverStarted=false;

  #define LHTTPMESS 600
  char   httpMess[LHTTPMESS];             // buffer d'entrée en mode serveur
#endif // _SERVER

  //#define LSRVTEXTIP TEXTIPADDRLENGTH+1
  //char textFrontalIp[LSRVTEXTIP];                  // sweethome server alpha IpAddr

// enregistrement pour serveur externe

  char  bufServer[LBUFSERVER];            // buffer des envois/réceptions de messages
  int   periMess;                         // diag de réception de message

  const char* fonctions={"set_______ack_______etat______reset_____sleep_____sw0__ON___sw0__OFF__sw1__ON___sw1__OFF__mail______mds_______mail_init_last_fonc_"};
  uint8_t fset_______,fack_______,fetat______,freset_____,fsleep_____,ftestaoff__,ftesta_on__,ftestboff__,ftestb_on__,fmail______,fmds_______,fmail_init_;
  int     nbfonct;
  uint8_t fonction;                       // la dernière fonction reçue

#define LTEMPSTR 16                     // chaine temp+analog value pour les transferts
  char           tempstr[LTEMPSTR];       // buffer temp+analog value
  float          temp;
  unsigned long  tempTime=0;              // (millis) timer température pour mode loop
  uint16_t       tempPeriod=PERTEMP;      // (sec) période courante check température 
  char           ageSeconds[8];           // secondes 9999999s=115 jours
  bool           tempchg=FAUX;

  unsigned long  clkTime=millis();        // timer automate rapide
  uint8_t        clkFastStep=0;           // stepper automate rapide
  uint8_t        clkSlowStep=0;           // stepper automate /10
  extern uint8_t nbreBlink;
  unsigned long  blkTime=millis();
  int            blkPer=2000;
  unsigned long  debTime=millis();        // pour mesurer la durée power on
  unsigned long  debConv=millis();        // pour attendre la fin du délai de conversion
  int            tconversion=0;
  unsigned long  detTime[MAXDET]={millis(),millis(),millis(),millis()};    // temps pour debounce
  uint32_t       locmem=0;                // local mem rules bits
  uint32_t       locMaskbit[]={0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                       0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                       0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                       0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};
  #define LOCMEM_STA_BIT 0x80000000       // bit réservé pour thermostat 

  /* paramètres switchs (les états et disjoncteurs sont dans cstRec.SWcde) */

  //bool oneShow=false;
  bool dataParFlag=false;

  uint8_t outSw=0;                            // image mémoire des switchs (1 bit par switch)
  uint8_t old_outSw=0x0F;                     // pour debug outSw
  #define OUTPUTDLY 250                       // délai mini pour décolage et ouverture relai fermé
  unsigned long outPutDly=millis();           // tempo après ouverture relais avant fermeture

  uint8_t pinSw[MAXSW]={PINSWA,PINSWB,PINSWC,PINSWD};       // switchs pins
  uint8_t cloSw[MAXSW]={CLOSA,CLOSB,CLOSC,CLOSD};           // close value for every switchs (relay/triac etc ON)
  uint8_t openSw[MAXSW]={OPENA,OPENB,OPENC,OPEND};          // open value for every switchs (relay/triac etc OFF)
  byte    staPulse[NBPULSE];                                // état clock pulses
  extern uint32_t  cntPulseOne[NBPULSE];                    // temps debut pulse 1
  extern uint32_t  cntPulseTwo[NBPULSE];                    // temps debut pulse 2
  extern uint32_t  cntPulse[NBPULSE*2];                     // temps restant après STOP pour START

  unsigned long    impDetTime[NBPULSE];                     // timer pour gestion commandes impulsionnelles     
  uint8_t pinDet[MAXDET]={PINDTA,PINDTB,PINDTC,PINDTD};     // les détecteurs
  #ifdef TOOGBT
  uint8_t toogSw=0;                                         // sortie en toogle depuis TOOGBT
  #endif  // TOOGBT

  uint32_t irqCnt=0;

  int   i=0,j=0,k=0;
  //uint8_t oldswa[]={0,0,0,0};                 // 1 par switch

constantValues cstRec;

char* cstRecA=(char*)&cstRec.cstlen;

  float voltage=0;                            // tension alim
  
  byte  mac[6];
  
  int   cntreq=0;

#if POWER_MODE != NO_MODE
  ADC_MODE(ADC_VCC);
#endif

  extern const char*  chexa; //="0123456789ABCDEFabcdef\0";
  uint8_t      bitMsk[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
  //const byte      mask[]={0x00,0x01,0x03,0x07,0x0F};
  //uint32_t  memDetServ=0x00000000;    // image mémoire NBDSRV détecteurs (32)  
  
  bool diags=true;
  unsigned long t_on=millis();

  uint8_t cntMTS=0;  // compteur pour blocage de l'IP de frontal... outil de debug

  uint8_t messageCnt=0;

  uint8_t sercnt=0;

   /* prototypes */

bool wifiAssign();
void talkServer();
void talkClient(char* etat);

int act2sw(int sw1,int sw2);
uint8_t runPulse(uint8_t sw);

void  getTemp();
void  getTempEtc();
char* tempStr();
int   buildData(const char* nomfonction,const char* data);
int   dataSave();
int   dataRead();
int   dataMail(char* mailData);
void  dataTransfer(char* data);
void  readTemp();
void  ordreExt();
void  outputCtl();
void  readAnalog();
uint16_t  getServerConfig();
void  ordreExt0();
void  showBS(char* buf);
void  thermostat();
#ifdef IRQCNT
void  irqCntUpdate();
#endif // IRQCNT



#ifdef MAIL_SENDER
void mail(char* subj,char* dest,char* msg);
#ifdef MAIL_CONFIG
void mailInit(char* login,char* pass)
{ 
  if(emailSend!=nullptr && (memcmp(login,fromMail,strlen(login))!=0 || memcmp(login,fromPass,strlen(pass)!=0))){
    delete emailSend;emailSend=nullptr;
  }
  if(emailSend==nullptr){
    if((strlen(login)<FROM_MAIL_LEN) && (strlen(pass)<FROM_PASS_LEN)){
      strcpy(fromMail,login);
      strcpy(fromPass,pass);
      emailSend = new EMailSender(login,pass);
    }
    else Serial.println("login/pass oversized");
  }
}
#endif // MAIL_CONFIG
#endif // MAILSENDER

#ifdef WPIN
void tmarker()
{
  pinMode(WPIN,OUTPUT);for(int t=0;t<6;t++){digitalWrite(WPIN,LOW);delayMicroseconds(100);digitalWrite(WPIN,HIGH);delayMicroseconds(100);}
}
#endif // WPIN

#ifdef PWR_CSE7766

void getCSE7766()
{
    //if(pinSw[powSw]==cloSw[powSw]){               // ? utile ?

        myCSE7766.handle();   // read CSE7766

        Serial.print("volts:");Serial.print(myCSE7766.getVoltage());Serial.print(" current:");Serial.print(myCSE7766.getCurrent());
        Serial.print(" power:");Serial.print(myCSE7766.getActivePower());Serial.print(" energy:");Serial.println(myCSE7766.getEnergy());
        
        double volts=myCSE7766.getVoltage();cstRec.powVolt=(uint16_t)(volts*10);
        double current=myCSE7766.getCurrent();cstRec.powCurr=(uint16_t)(current*1000);
        double power=myCSE7766.getActivePower();cstRec.powPower=(uint16_t)(power*10);
        double energy=myCSE7766.getEnergy();cstRec.powEnergy=(uint32_t)energy;        
    
    //}
}
#endif // PWR         


void setup() 
{ 

//pinMode(PINLED,OUTPUT);while(1){yield();digitalWrite(PINLED,HIGH);delay(500);digitalWrite(PINLED,LOW);delay(500);}

/* >>>>>> pins Init <<<<<< */

  checkVoltage();                   // power off au plus vite si tension insuffisante (no serial)

/*   stop modem
#if POWER_MODE!=NO_MODE
WiFi.disconnect();
WiFi.forceSleepBegin();
delay(1);
#endif // PM!=NO_MODE
*/

  Serial.begin(115200);delay(100);

#if POWER_MODE==PO_MODE
Serial.println("\n+");
delay(1);

  digitalWrite(PINPOFF,LOW);
  pinMode(PINPOFF,OUTPUT);
#endif // PM==PO_MODE

#if POWER_MODE==NO_MODE
  diags=false;
  delay(4000);
  //Serial.print("\nSerial buffer size =");Serial.println(Serial.getRxBufferSize());
  Serial.print("\nstart setup ");Serial.print(VERSION);
  Serial.print(" power_mode=");Serial.print(POWER_MODE);
  Serial.print(" carte=");Serial.print(CARTE);


#ifdef TEST_CSE
  Serial.print(" ; une touche pour start ");
  #define MAXW 50
  uint8_t ss=0;while(ss<MAXW){
    ss++;Serial.print(".");delay(500);if(Serial.available()){Serial.read();break;}}
  Serial.println();
  if(ss>=MAXW){while(1){yield();}}

  Serial2.begin(4800,SERIAL_8E1,16,-1);
  uint16_t cnt=0;
  char c;
  #define LDATA 1024
  char data[LDATA];memset(data,'\0',LDATA);
  while(cnt<(LDATA-1)){
    while(Serial2.available() && cnt<(LDATA-1)){
      c=Serial2.read();data[cnt]=c;
      if((c&0xf0)==0){Serial.print('0');}Serial.print(c,HEX);Serial.print(' ');
      cnt++;
    }
  }
  dumpstr(data,LDATA);
  while(1){}
#endif



#ifdef ANALYZE
  Serial.print(" ANALYZE ");
#endif // ANALYZE
  Serial.print(" ; une touche pour diags ");
  uint8_t i=0;while(i<7){i++;Serial.print(".");delay(500);if(Serial.available()){Serial.read();diags=true;break;}}
  Serial.println();
#endif // PM==NO_MODE  

#ifdef CAPATOUCH
    capaKeys.init(SAMPLES,COMMON,keys,KEYNB);
    capaKeys.calibrate();
#endif    

  initLed(PINLED,LEDOFF,LEDON);

#ifdef PINLEDR
  pinMode(PINLEDR,OUTPUT);
#endif // PINLEDR

#if CARTE==VR || CARTE==VRR || CARTE==VRDEV || CARTE==SFRFR2 || CARTE==SFPOW
  for(uint8_t sw=0;sw<MAXSW;sw++){
    digitalWrite(pinSw[sw],openSw[sw]);
    pinMode(pinSw[sw],OUTPUT);}

#ifdef TOOGBT
for(uint8_t i=0;i<NBSW;i++){if(pinSw[i]==TOOGSW){toogSw=i;break;}}
#endif // TOOGBT

#ifdef PWR_CSE7766
for(uint8_t i=0;i<NBSW;i++){if(pinSw[i]==POWSW){powSw=i;break;}}
#endif // PWR_CSE7766

#ifndef CAPATOUCH
  for(uint8_t i=0;i<NBDET;i++){pinMode(pinDet[i],INPUT_PULLUP);}
#endif // CAPATOUCH

#endif // VR||VRR

/* >>>>>> inits variables <<<<<< */

  model[0]=CARTE;
  model[1]=POWER_MODE;
  model[2]=CONSTANT;
  model[3]='1';
  model[4]=(char)(NBSW+48);
  model[5]=(char)(NBDET+48);  

  nbfonct=(strstr(fonctions,"last_fonc_")-fonctions)/LENNOM;  
  fset_______=(strstr(fonctions,"set_______")-fonctions)/LENNOM;
  fack_______=(strstr(fonctions,"ack_______")-fonctions)/LENNOM;
  fetat______=(strstr(fonctions,"etat______")-fonctions)/LENNOM;
  freset_____=(strstr(fonctions,"reset_____")-fonctions)/LENNOM;
  fsleep_____=(strstr(fonctions,"sleep_____")-fonctions)/LENNOM;  
  ftestaoff__=(strstr(fonctions,"sw0__OFF__")-fonctions)/LENNOM;
  ftesta_on__=(strstr(fonctions,"sw0__ON___")-fonctions)/LENNOM;
  ftestboff__=(strstr(fonctions,"sw1__OFF__")-fonctions)/LENNOM;  
  ftestb_on__=(strstr(fonctions,"sw1__ON___")-fonctions)/LENNOM;
  fmail______=(strstr(fonctions,"mail______")-fonctions)/LENNOM;
  fmds_______=(strstr(fonctions,"mds_______")-fonctions)/LENNOM;

/* >>>>>> debut <<<<<< */

  if(diags){
#if CARTE != THESP01
#if CARTE != THESP02    
    Serial.println();
    Serial.print("    ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(sw);}Serial.println();
    Serial.print("pin ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(pinSw[sw]);}Serial.println();
    Serial.print("clo ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(cloSw[sw]);}Serial.println();
    Serial.print("ope ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(openSw[sw]);}Serial.println();
#endif
#endif
  }


/* >>>>>> gestion ds18x00 <<<<<< */
#ifdef WPIN
 byte setds[4]={0,0x7f,0x80,TBITS},readds[8];    // fonction, high alarm, low alarm, config conversion 
 int v=ds1820.setDs(WPIN,setds,readds); // init & read rom
 tconversion=TCONVERSIONB;if(readds[0]==0X10 || TBITS==T12BITS){tconversion=TCONVERSIONS;}
 if(v==1){Serial.print("\nDS1820 0x");Serial.print(readds[0],HEX);Serial.print(" Tconv=");Serial.println(tconversion);}
 else {Serial.print(" DS1820 error ");Serial.println(v);}
  
#if POWER_MODE==NO_MODE
  debConv=millis();
  ds1820.convertDs(WPIN); // readTemp ignoré jusqu'à fin de la conversion
  //delay(tconversion);
#endif // PM==NO_MODE
#if POWER_MODE==PO_MODE
  if(v==1){
    debConv=millis();
    ds1820.convertDs(WPIN); // readTemp() attend la fin de la conversion
  }
#endif // PM==PO_MODE

#if POWER_MODE==DS_MODE
/* si pas sortie de deep sleep faire une conversion 
   et initialiser les variables permanentes sinon les variables permanentes sont supposées valides */
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  Serial.print("\nresetinfo ");Serial.println(resetInfo->reason);
  if((resetInfo->reason)!=5){           // 5 deepsleep awake ; 6 external reset
    ds1820.convertDs(WPIN);
    delay(tconversion);
    //initConstant();
    }
#endif // PM==DS_MODE
#endif // WPIN

/* >>>>>> gestion variables permanentes <<<<<< */

#if CONSTANT==EEPROMSAVED
  Serial.print("EEPROM ");
  EEPROM.begin(512);
#endif
#if CONSTANT==RTCSAVED
  Serial.print("RTC ");
#endif

/* si erreur sur les variables permanentes (len ou crc faux), initialiser et sauver */
initConstant();             // à supprimer en production
  if(!readConstant()){   
    Serial.println("KO -> init ");
    initConstant();
    yield();
    if(cstRec.cstlen!=LENCST){
      Serial.print(" len RTC=");Serial.print(cstRec.cstlen);Serial.print("/");Serial.print(LENCST);
      while(1){blink(1);delay(1000);}} // blocage param faux, le programme a changé
  }

  printConstant();
  
  // Ip au format texte pour MessToServer
  // charIp ne fonctionne pas avec le format IPAddress ; changer pour byte* modifie cstRec
  // Ip au format IPAddress (v2.a synchrone avec frontal2 1.8p)
  //char buf[TEXTIPADDRLENGTH+1];memset(buf,0X00,TEXTIPADDRLENGTH+1);
  //for(uint8_t i=0;i<4;i++){
    //sprintf(buf+strlen(buf),"%d",cstRec.serverIp[i]);if(i<3){strcat(buf,".");}}
  //memcpy(textFrontalIp,buf,LSRVTEXTIP);

/* config via serial from server */
  #define FRDLY 5  // sec
#if CARTE != THESP01
  pinMode(PINDTC,INPUT_PULLUP);
  if(digitalRead(PINDTC)==LOW){                     
    blink(4);delay(2000);
    yield();
    if(getServerConfig()>0){writeConstant();while(1){blink(1);delay(1000);}} // getServerConfig bloque si ko
  }
#endif // != THESP01
  Serial.print("cstRec.serverIp ");Serial.print(cstRec.serverIp);//Serial.print(" textFrontalIp ");Serial.println(textFrontalIp);
  Serial.print(" time=");Serial.println(millis()-debTime);
  Serial.print("ssid=");Serial.print(cstRec.ssid1);Serial.print(" - ");Serial.println(cstRec.ssid2);
  Serial.println();
//  Serial.print("locmem=");Serial.println(locmem,HEX);

#if POWER_MODE==NO_MODE
  cstRec.talkStep=0;
  cstRec.serverTime=cstRec.serverPer+1;
  talkReq(); 

  memdetinit();pulsesinit();
  yield();
  
#ifdef  _SERVER_MODE
  Serial.print("_SERVER_MODE ");
  clkFastStep=1;talkReq();                  // forçage com pour acquisition port perif server

  while(!wifiAssign()){                     // setup ssid,ssidPwd par défaut
    delay(2000);blink(3);}
  
#ifdef MAIL_SENDER
  Serial.print("MAIL_SENDER ");
#ifndef MAIL_CONFIG
  emailSend = new EMailSender(fromMail,fromPass);
#endif // MAIL_CONFIG
#ifdef MAIL_CONFIG
  Serial.print("MAIL_CONFIG ");
#endif
#endif // MAIL_SENDER

#endif // def_SERVER_MODE
  Serial.println(">>>> fin setup\n");
  actionsDebug();
#ifdef ANALYZE
  STOPALL
  pinMode(ANPIN0,OUTPUT);
  pinMode(ANPIN1,OUTPUT);
  pinMode(ANPIN2,OUTPUT);
#endif // ANALYZE  
  clkTime=millis();

#ifdef IRQCNT
  attachInterrupt(digitalPinToInterrupt(IRQPIN), irqCntUpdate, FALLING);
#endif  // IRQCNT


#ifdef PWR_CSE7766
    myCSE7766.setRX(PINSRX);
    myCSE7766.begin(); // will initialize serial to 4800 bps
#endif // CSE7766

  }    // fin setup NO_MODE

  void loop(){  //=== NO_MODE =================================      

  // la réception de commande est prioritaire sur tout et (si une commande valide est reçue) toute communication
  // en cours est avortée pour traiter la commande reçue.
  //
  // automate de séquencement : horloge à 50mS et à 500mS ;
  //    10 slots dans chaque ; un traitement peut exister sur plusieurs slots
  //
  // si un appel au serveur est en cours (talkSta()!=0), l'automate talkServer est actif
  // sinon la boucle d'attente tourne : 
  //                                    (en continu)
  //                                      - réception d'un ordre extérieur
  //                                    (toutes les 50mS chaque)
  //                                      - talkServer
  //                                      - exécution des actions
  //                                      - debounce détecteurs physiques
  //                                      - polling  détecteurs physiques
  //                                      - test de l'heure de blink
  //                                     (toutes les 100mS)
  //                                      - clock pulses  
  //                                     (toutes les 500mS)
  //                                      - test de l'heure de mesure de température (ou autre)
  //                                      - test de l'heure d'appel du serveur (inclus dans readTemp())
  //
  // Si l'appel n'aboutit pas (pas de cx wifi, erreurs de com, rejet par le serveur-plus de place)
  // le délai d'appel au serveur devient PERSERVKO pour économiser les batteries
  //
  // la période est allongée par les communications avec le serveur (appel ou réception d'ordre)

  #ifdef  _SERVER_MODE

      ordreExt();
      
      if(millis()>(clkTime)){        // période 5mS/step

        clkTime+=PERFASTCLK;
        switch(clkFastStep++){

/*
 * En 1 ordreExt() pour assurer que le traitement des commandes reçues et la collecte des données soit effectués pour le prochain dataSave
 * (ordreExt() positionne cstRec.talkStep!=0) ; au reset, server = nullptr 
 * Puis talkstep pour le forçage de communication d'acquisition du port au reset 
 * clkFastStep et cstRec.talkStep == 1 
*/
          case 1:   if(cstRec.talkStep!=0){talkServer();}//oneShow=false;
                    break;
          case 2:   break;
          case 3:   wifiConnexion(ssid,ssidPwd,NOPRINT);break;
          case 4:   pulseClk();break;
          case 5:   swDebounce();break;         // doit être avant polDx              
          case 6:   actions();break;
          case 7:   polAllDet();break;          // polDx doit être après swDebounce et dernier avant outputCtl pour que le tooglepushbutton soit maitre de tout
          case 8:   outputCtl();break;          // quand toutes les opérations sont terminées
          case 9:   ledblink(-1,PULSEBLINK);break;
          case 10:  clkFastStep=0;              // période 50mS/step                         
                    switch(clkSlowStep++){
                      case 1:   break;
                      case 2:   break;
                      case 3:   break;
                      case 4:   break;
                      case 5:   break;
                      case 6:   break;
                      case 7:   readAnalog();break;
                      case 8:   readTemp();break;
                      case 9:   thermostat();break;
                      case 10:  clkSlowStep=0;break;
                      default:  break;
                    }
                    break;
                   
          default: 
          break;
        }
      }
  #endif // def_SERVER_MODE  

#endif // PM==NO_MODE


#if POWER_MODE!=NO_MODE

  readTemp();

  Serial.print("durée (no comm)=");Serial.print(millis());Serial.print(" - ");
  Serial.print(dateOn);Serial.print(" = ");Serial.println(millis()-dateOn);
  
  while(cstRec.talkStep!=0){
    Serial.print("   talkStep=");Serial.println(cstRec.talkStep,HEX);
    yield();talkServer();}

  /* sauvegarde variables permanentes avant sleep ou power off */
  writeConstant();

  Serial.print("durée ");Serial.print(millis());Serial.print(" - ");
  Serial.print(dateOn);Serial.print(" = ");Serial.print(millis()-dateOn);
  Serial.print("   talkStep=");Serial.print(cstRec.talkStep,HEX);
  delay(10); // purge serial

  #if POWER_MODE==DS_MODE
  /* deep sleep */
    Serial.println(" deep sleep");
    delay(10);
    
    ESP.deepSleep(cstRec.tempPer*1e6, WAKE_RF_DEFAULT);    // microseconds
    //ESP.deepSleep(cstRec.tempPer*1e6, WAKE_RF_DISABLED);   // ne fonctionne pas ... on ne peut pas redémarrer le modem sans redémarrer le 8266
    while(1){delay(1000);};                  
  #endif // PM==DS_MODE
  #if POWER_MODE==PO_MODE
  /* power off */
    Serial.println(" power down");
    delay(10);
    digitalWrite(PINPOFF,HIGH);        // power down
    pinMode(PINPOFF,OUTPUT);
    while(1){delay(1000);};
  #endif // PM==PO_MODE

  yield();
  delay(2000);                           // si l'alim reste allumée
  cstRec.serverTime=cstRec.serverPer+1;  // force une communication au prochain démarrage
  writeConstant();
  while(1){delay(1000);};

  } //  fin setup si != NO_MODE
void loop() {
#endif // PM!=NO_MODE

}  // fin loop pour toutes les options

/* =================== communications ========================

fServer() réception et chargement de la réponse à dataRead/Save

dataTransfer() contrôle et chargement de set/ack

talkServer()  automate de fragmentation temporelle d'envoi de dataRead/Save et gestion réponses
talkReq()     déclenche talkServer à la prochaine loop
talkSta()     renvoie 0 si talkServer est inactif

buildData()     construction message fonction dataRead ou dataSave

buildReadSave() construction et envoi message read/save

dataRead()

dataSave()

ordreExt() test présence/réception et chargement message reçu en mode serveur (declenche talkServer pour envoyer le résultat des commandes - position switchs)

talkClient() réponse à un message reçu en mode serveur

wifiConnexion()

readAnalog() (pour NO_MODE seul : les autres modes utilisent l'ADC pour lire l'alim)
la valeur analogique est soit la tension lue sur le pin A0, soit le compteur d'impulsions sur IRQPIN (GPIO13)

readTemp() gestion communications cycliques (déclenche talkServer)

wifiAssign() positionne le wifi par défaut

*/

/* ----------------- gestion données ------------------ */

int fServer(uint8_t fwaited)          // réception du message réponse du serveur pour DataRead/Save;
                                      // contrôles et transfert 
                                      // retour periMess  
{      
        periMess=getHttpResponse(&cli,bufServer,LBUFSERVER,&fonction,diags);
        if(diags){
          Serial.print("fserver (OK=");Serial.print(MESSOK);Serial.print(") periMess=");Serial.print(periMess);
          Serial.print(" fwaited=");Serial.print(fwaited);Serial.print(" recu=");Serial.print(fonction);} 
        if(periMess==MESSOK){
          //Serial.println(bufServer);
          if(fonction==fwaited){dataTransfer(bufServer);}
          else {periMess=MESSFON;}
        }
        return periMess;
}

void dataTransfer(char* data)   // transfert contenu de set ou ack dans variables locales selon contrôles
                                        // data sur fonction
                                        //    contrôle mac addr et numPeriph ;
                                        //    si pb -> numPeriph="00" et ipAddr=0
                                        //    si ok -> tfr params
                                        // retour periMess
{
  size_t messLength=0;                                    // len usefull data
  for(k=0;k<4;k++){
    messLength*=10;messLength+=data[MPOSLEN+k]-'0';}      // conv len message atob
    messLength+=(11+2);                                     // + 'fonction__=' + 'nn' crc
  if(strlen(data)!=messLength){Serial.print("dataTransfer invalid length ");Serial.print(strlen(data));Serial.print('/');Serial.println(messLength);}
  
  byte fromServerMac[6];
  
        periMess=MESSOK;
        packMac(fromServerMac,(char*)(data+MPOSMAC));
        if(memcmp(data+MPOSNUMPER,"00",2)==0){periMess=MESSNUMP;}
        //else if(!compMac(mac,fromServerMac)){periMess=MESSMAC;}
        else if(memcmp(mac,fromServerMac,6)!=0){periMess=MESSMAC;}
        else {
                             // si ok transfert des données
if(diags){Serial.println(" dataTransfer() ");}       
          memcpy(cstRec.numPeriph,data+MPOSNUMPER,2);                         // num périph

          int sizeRead;
          cstRec.serverPer=(long)convStrToNum(data+MPOSPERREFR,&sizeRead);    // per refresh server
          cstRec.tempPer=(uint16_t)convStrToNum(data+MPOSTEMPPER,&sizeRead);  // per check température (invalide/sans effet en PO_MODE)
          cstRec.tempPitch=(long)convStrToNum(data+MPOSPITCH,&sizeRead);      // pitch mesure (100x)

          for(uint8_t i=0;i<MAXSW;i++){                                       // 1 byte disjoncteurs : 2 bits / switch (voir const.h du frontal) 
            cstRec.swCde=(cstRec.swCde)<<2;
            cstRec.swCde |= (*(data+MPOSSWCDE+i)-PMFNCVAL);
          }                                                                   // valeurs disjoncteur (0/1/2)x4

          conv_atoh(data+MPOSANALH,(byte*)&cstRec.analLow);conv_atoh(data+MPOSANALH+2,(byte*)&cstRec.analLow+1);      // analogLow
          conv_atoh(data+MPOSANALH+4,(byte*)&cstRec.analHigh);conv_atoh(data+MPOSANALH+6,(byte*)&cstRec.analHigh+1);  // analogHigh

          uint16_t posMds=MPOSPULSONE;
          if(memcmp(data,"mds_______",LENNOM)!=0){                            // pas de pulses ni de rules si mds_______
            posMds=MPOSMDETSRV;

            for(int i=0;i<NBPULSE;i++){                                       // pulses values NBPULSE*ONE+NBPULSE*TWO
              cstRec.durPulseOne[i]=(long)convStrToNum(data+MPOSPULSONE+i*(LENVALPULSE+1),&sizeRead);
              cstRec.durPulseTwo[i]=(long)convStrToNum(data+MPOSPULSTWO+i*(LENVALPULSE+1),&sizeRead);}

            for(int ctl=PCTLLEN-1;ctl>=0;ctl--){                              // pulses control
              conv_atoh((data+MPOSPULSCTL+ctl*2),&cstRec.pulseMode[ctl]);}

            uint16_t i1=NBPERRULES*PERINPLEN;                                 // size inputs
            byte bufoldlev[i1];
            for(uint16_t k=0;k<i1;k++){                                       // inputs !!! ne pas écraser les bits oldlevel !!!
              conv_atoh((data+MPOSPERRULES+2*k),&bufoldlev[k]);
              if((k%4)==2){bufoldlev[k]&=~PERINPOLDLEV_VB;bufoldlev[k]|=cstRec.perInput[k]&PERINPOLDLEV_VB;}
            }
            memcpy(&cstRec.perInput,bufoldlev,i1);
          } // !mds

          int mdsl=MDSLEN;
          if(*(data+posMds+8)=='_'){mdsl=4;}
          for(int k=0;k<mdsl;k++){                                            // détecteurs externes
            conv_atoh((data+posMds+k*2),(byte*)(cstRec.extDetec+mdsl-1-k));
          }   

          //if(diags){Serial.print("==");Serial.print((char*)data+posMds+mdsl*2+1);Serial.print(' ');Serial.println(sizeRead);}
          cstRec.periPort=(uint16_t)convStrToNum(data+posMds+mdsl*2+1,&sizeRead); // port server
          
          #ifdef _SERVER_MODE
            if(server==nullptr && cstRec.periPort!=0){server=new WiFiServer(cstRec.periPort);Serial.println("newS");}
          #endif

          cstRec.periAnal=packHexa(data+messLength-7-6,4);                // periAnalOut consigne analogique 0-FF
          cstRec.periCfg=packHexa(data+messLength-2-6,2);                 // periCfg '_hh'         

        } // periMess==MESSOK
        if(periMess!=MESSOK){
          memcpy(cstRec.numPeriph,"00",2);cstRec.IpLocal=IPAddress(0,0,0,0);
        }
}

int buildData(const char* nomfonction,const char* data,const char* mailData)               
                                                                      // assemble une fonction data_read_ ou data_save_ ou data_par__ ou data_mail_
{                                                                     // et concatène dans bufServer - retour longueur totale
  char message[LENVAL];
  int sb=0,i=0;
  
      memcpy(message,cstRec.numPeriph,2);                             // N° périf                    - 3
      memcpy(message+2,"_\0",2);
      sb=3;
      unpackMac((char*)(message+sb),mac);                             // macaddr                    - 18
      sb+=17;
      memcpy(message+sb,"_\0",2);
      strcat(message,data);strcat(message,"_");                       // temp, analog (dans data_save seul) - 15
      sb=strlen(message);
      sprintf(message+sb,"%1.2f",voltage);                            // alim                        - 5
      memcpy(message+sb+4,"_\0",2);
      memcpy(message+sb+5,VERSION,LENVERSION);                        // VERSION contient le "_"     - 3
      char ds='B';if(ds1820.dsmodel==MODEL_S){ds='S';}
      memcpy(message+sb+5+LENVERSION-1,&ds,1);                        // modele DS18x20              - 2
      memcpy(message+sb+5+LENVERSION,"_\0",2);
      
      sb+=5+LENVERSION+1;
      message[sb]=(char)(NBSW+PMFNCVAL);                              // nombre switchs              - 1   
      for(i=0;i<NBSW;i++){
        uint8_t ssw=0;if(digitalRead(pinSw[i])==cloSw[i]){ssw=1;}
        message[sb+1+(MAXSW-1)-i]=(char)(PMFNCVAL+ssw);}              // état                        - 5
      if(NBSW<MAXSW){for(i=NBSW;i<MAXSW;i++){message[sb+1+(MAXSW-1)-i]='x';}}message[sb+5]='_';

      sb+=MAXSW+2;
      message[sb]=(char)(NBDET+48);                                   // nombre détecteurs
      for(i=(NBDET-1);i>=0;i--){message[sb+1+(NBDET-1)-i]=(char)(chexa[cstRec.memDetec[i]]);} // état -6 
      if(NBDET<MAXDET){for(i=NBDET;i<MAXDET;i++){message[sb+1+i]='x';}}                              
      strcpy(message+sb+1+MAXDET,"_\0");
      sb+=MAXDET+2;

      if(mailData!=nullptr){strcat(message,mailData);strcat(message,"_\0");sb+=(strlen(mailData)+1);}

      for(i=0;i<NBPULSE;i++){message[sb+i]=chexa[staPulse[i]];}
      strcpy(message+sb+NBPULSE,"_\0");                               // clock pulse status          - 5

      sb+=NBPULSE+1;
      memcpy(message+sb,model,LENMODEL);

      strcpy(message+sb+LENMODEL,"_\0");                                                     //      - 7
      sb+=LENMODEL+1;

      if(memcmp(nomfonction,"data_par__",LENNOM)==0){
        strcpy(message+sb,"swcde=");conv_htoa(message+sb+6,&cstRec.swCde);
        strcpy(message+sb+8,";\0");
        sb+=9;
        #ifdef PWR_CSE7766
        /* cse_values */
        //sprintf(message+sb,"power=s:%02d,v:%04d,c:%05d,p:%05d,e:%09d;\0",myCSE7766.cse_status,cstRec.powVolt,cstRec.powCurr,cstRec.powPower,cstRec.powEnergy);
        /* cse_data */
        #define LCSEDATA 24
        char cse_data[LCSEDATA*2];
        for(uint8_t cd=0;cd<LCSEDATA;cd++){
          conv_htoa(&cse_data[2*cd],&myCSE7766._data[cd]);}
        sprintf(message+sb,"power=s:%02d,d:%s;\0",myCSE7766.cse_status,cse_data);
        
        sb=strlen(message); //6+powMessageLen);
        #endif

        strcpy(message+sb,";_\0");    
        sb+=2;
      }

      messageCnt++;
      if(messageCnt>99){messageCnt=0;}
      sprintf(message+sb,"%02d",messageCnt);                 // N° ordre du message (00-99)
      memcpy(message+sb+2,"_\0",2);
      sb+=3;

      bool noZero=false;            // si aucun compteur n'est utilisé ET qu'ils sont tous à 0, pas de transmission
      for(i=0;i<NBPULSE;i++){
        if(cntPulseOne[i]!=0 || cntPulseTwo[i]!=0){ //noZero=true;break;}
          if(staPulse[i]!=PM_DISABLE && staPulse[i]!=PM_IDLE && staPulse[i]!=0x00){noZero=true;break;}
        }
      }
      if(noZero){
        uint32_t currt;
        if(diags){Serial.print("buildData ");dumpfield((char*)cstRec.pulseMode,2);Serial.print("  ");}
        for(int i=0;i<NBPULSE;i++){         // loop compteurs
          if(diags){Serial.print(":");Serial.print(staPulse[i]);Serial.print(" ");}
          currt=0;
          if(cntPulseOne[i]!=0){currt=1+((uint32_t)millis()-cntPulseOne[i])/1000;}
          if(diags){Serial.print(currt);if(currt>=cstRec.durPulseOne[i]){Serial.print('>');}else {Serial.print('<');}Serial.print(cstRec.durPulseOne[i]);
          Serial.print("-");}
          for(uint8_t j=0;j<4;j++){conv_htoa((char*)(message+sb+i*2*8+j*2),(byte*)(&currt)+j);}       // loop bytes (4/8)
          currt=0;
          if(cntPulseTwo[i]!=0){currt=1+((uint32_t)millis()-cntPulseTwo[i])/1000;}
          if(diags){Serial.print(currt);if(currt>=cstRec.durPulseTwo[i]){Serial.print('>');}else {Serial.print('<');}Serial.print(cstRec.durPulseTwo[i]);
          Serial.print("  ");}
          for(uint8_t j=0;j<4;j++){conv_htoa((char*)(message+sb+(i*2+1)*8+j*2),(byte*)(&currt)+j);}   // loop bytes (4/8)
        }
        if(diags){Serial.println();}
        sb+=NBPULSE*2*sizeof(uint32_t)*2+1;
      } 
      *(message+sb-1)='*';                    // identifie le car suivant comme SsidNb pour periDataRead dans frontal2.cpp
      *(message+sb)=(char)(ssidNb+0x30);
      memcpy(message+sb+1,"_\0",2);

  if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL,PULSEBLINK);}      
  
  return buildMess(nomfonction,message,"",diags);        // concatène et complète dans bufserver
}

int buildData(const char* nomfonction,const char* data)               // assemble une fonction data_read_ ou data_save_ ou data_par__ ou data_mail_
{                                                      
  return buildData(nomfonction,data,nullptr);
}

int buildReadSave(const char* nomFonction,const char* data,const char* mailData)   // construit et envoie une commande GET complète
                                                  //   avec fonction peri_pass_ + dataRead ou dataSave
                                                  //   peri_pass_=nnnnpppppp..cc?
                                                  //   data_rs.._=nnnnppmm.mm.mm.mm.mm.mm_[-xx.xx_aaaaaaa_v.vv]_r.r_siiii_diiii_ffff_cc
                                                  //   (sortie MESSCX connexion échouée)                                                  
{
  //Serial.print(nomFonction);Serial.print(' ');
  strcpy(bufServer,"GET /cx?\0");
  if(buildMess("peri_pass_",cstRec.peripass,"?",diags,true)!=MESSOK){
    if(diags){Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(cstRec.peripass);return MESSDEC;};}

  buildData(nomFonction,data);
  
  if(diags){//showBS(bufServer);
  }

//dumpstr(bufServer,144);

// frontal devient inaccessible ce qui permet de tester l'interruption de messToServer par un ordreExt()
  return messToServer(&cli,&cstRec.serverIp,cstRec.serverPort,bufServer,server,&cliext); 

}

int buildReadSave(const char* nomFonction,const char* data)
{
  return buildReadSave(nomFonction,data,nullptr);
}

char* tempStr()
{
      memset(tempstr,0x00,LTEMPSTR);
      sprintf(tempstr,"%+02.2f",temp/100);                                // 6 car
      //Serial.print(temp);Serial.print(" ");Serial.println(tempstr);
      if(strstr(tempstr,"nan")!=0){memcpy(tempstr,"+00.00\0",7);}
      strcat(tempstr,"_");                                                // 1 car
      sprintf((char*)(tempstr+strlen(tempstr)),"%06d",cstRec.analVal);    // 6 car
      strcat(tempstr,"\0");                                               // 1 car
  return tempstr;
}

int dataSave()
{
  return buildReadSave("data_save_",tempStr());
}

int dataRead()
{
   return buildReadSave("data_read_","_");
}

int dataPar()
{
  return buildReadSave("data_par__",tempStr());
}

int dataMail(char* mailData)
{
  return buildReadSave("data_mail_",tempStr(),mailData);
}


/* ----------------- talkServer ------------------ */

void talkReq(uint8_t bit)
{
  cstRec.talkStep|=bit;
}

void talkReq()
{
  talkReq(TALKREQBIT);
}


void talkGrt()
{
  cstRec.talkStep&=~TALKREQBIT;
  cstRec.talkStep|=TALKGRTBIT;
}

void talkClr()
{
  cstRec.talkStep&=~TALKGRTBIT;
  cstRec.talkStep&=~TALKCNTBIT;
}

void talkSet(uint8_t cnt)
{
  cstRec.talkStep&=TALKSTABIT;
  cstRec.talkStep+=cnt;
}

uint8_t talkSta()
{
  if((cstRec.talkStep&(TALKGRTBIT|TALKREQBIT|TALKCNTBIT))==0){return 0;}
  return cstRec.talkStep&TALKCNTBIT;
}

void talkKo(int v)
{
  Serial.print("MESS=");Serial.println(v);delay(2);
  memcpy(cstRec.numPeriph,"00",2);
  if(v==MESSSRV){ordreExt0();}
  talkClr();
}

void talkKo()
{
  talkKo(MESSOK);
}

void talkWifiKo()
{  
#if POWER_MODE!=NO_MODE
      cstRec.serverPer=PERSERVKO;     // pas de modif en NO_MODE pour ne pas risquer le déclenchement du WD
#endif
      
  cstRec.serverTime=0;
  talkKo();
}

#define TALKSKO     99  // pb -> pas de maj des datas ; numPeriph=0
#define TALKWIFIKO  98  // connexion Wifi échouée
#define TALKWIFI1    1  // tentative connexion wifi 1
#define TALKWIFI2    2  // tentative connexion wifi 1
#define TALKDATA     4  // dataRead/Save
#define TALKDATASAVE 6  // dataSave
#define TALKSWCDE    7  // dataSwcd


void talkServer()   // si numPeriph est à 0, dataRead pour se faire reconnaitre ; 
                    // si ça fonctionne réponse numPeriph!=0 ; dataSave 
                    // renvoie 0 et periMess valorisé si la com ne s'est pas bien passée.
{

dateOn=millis();

if((cstRec.talkStep && TALKREQBIT)!=0 && (cstRec.talkStep&TALKCNTBIT)==0){cstRec.talkStep+=TALKWIFI1;}

switch(cstRec.talkStep&=TALKCNTBIT){
  case 0:break;
  
  case TALKWIFI1:

      if(!wifiAssign()){talkWifiKo();}
      if((millis()-dateOn)>1){talkSet(TALKDATA);ssidNb=1;break;}

  case TALKDATA:        // connecté au wifi
                        // si le numéro de périphérique est 00 ---> récup (dataread), ctle réponse et maj params
      {int v=0;
      uint8_t f=0;
        talkGrt();
        if(memcmp(cstRec.numPeriph,"00",2)==0){
          if(!dataParFlag){f=fset_______;v=dataRead();} else {dataParFlag=false;f=fack_______;v=dataPar();}   // v=dataRead();
          //Serial.print("outofDR v=");Serial.println(v);
          if(v==MESSOK && fServer(f)==MESSOK){
            talkSet(TALKDATASAVE);}
          else {talkKo(v);}    // pb com -> recommencer
          break;
        }  
      }
              
  case TALKDATASAVE:          // (6) si numPeriph !=0 ou réponse au dataread ok -> datasave
                              // sinon recommencer au prochain timing                              
      { int v;
        if(!dataParFlag){v=dataSave();} else {dataParFlag=false;v=dataPar();}
        //Serial.print("outofDS v=");Serial.println(v);
        if(v!=MESSOK || fServer(fack_______)!=MESSOK){talkKo(v);}   // si ko recommencer au prochain timing             
        
        else {
          talkClr();  // terminé ; tout s'est bien passé les 2 côtés sont à jour 

#ifdef  _SERVER_MODE
          if(server!=nullptr && !(server->available())){      //} && !serverStarted){
           server->begin(cstRec.periPort);
            serverStarted=true;
            Serial.print(" server.begin:");Serial.println((int)cstRec.periPort);
          }
#endif // def_SERVER_MODE
        }
      }
      break;

  case TALKSWCDE:
      break;
        
  default: //Serial.print(" 1/");Serial.print(cstRec.talkStep,HEX);
      talkKo();break;
  }
//if(ts!=0){Serial.print(">>");Serial.print(cstRec.talkStep,HEX);}
}

#ifdef _SERVER_MODE

/* ----------------- ordreExt ------------------ */

#ifdef MAIL_SENDER
void mail(char* subj,char* dest,char* msg)
{
unsigned long beg=millis();

    Serial.print("---mail--- ");

    char* m1=strstr(msg,"##");      // login
    char* m2=strstr(m1,"==");       // pswd
    char* m3=strstr(m2,"##");       // fin pswd
    if(m1!=0 && m2!=0 && m3!=0){*m2='\0';*m3='\0';mailInit(m1+2,m2+2);}

      #define LMLOC 16
      char a[LMLOC];a[0]=' ';sprintf(a+1,"%+02.2f",temp/100);a[7]='\0';
      strcat(a,"°C ");strcat(a,VERSION);
      if(strlen(a)>=LMLOC){ledblink(BCODESHOWLINE,PULSEBLINK);}
      a[LMLOC-1]='\0';strcat(msg,a);    // écrase les params d'init

    if(emailSend==nullptr){Serial.println(">>>>>>>>>>>> no conf for mail - restart server for mailInit");}
    else {
      wifiConnexion(ssid,ssidPwd);

      char s[64]={"sh "};strcat(s,subj);
      message.subject = s;
      message.message = msg ;

      EMailSender::Response resp = emailSend->send(dest, message);
      resp.code[1]=0;resp.desc[16]=0; // 0 sent ; 1 SMTP time out ; 2 not connect to server
      Serial.print(">>> email ");

Serial.print("dest ");Serial.print(dest);
Serial.print(" mess ");Serial.println(msg);
Serial.print(" resp.code ");Serial.print(resp.code);
Serial.print(" resp.desc ");Serial.print(resp.desc);

      Serial.print(" millis()=");Serial.println(millis()-beg);
    }
}

#endif // MAIL_SENDER

void talkClient(char* mess) // réponse à une requête
{
  
            // en-tête réponse HTTP 
            cliext.write("HTTP/1.1 200 OK\n");
            cliext.write("Content-type:text/html\n");
            cliext.write("Connection: close\n\n");
            // page Web 
            cliext.write("<!DOCTYPE html><html>\n");
            //cliext.println("<head></head>");
            
            cliext.write("<body>");
            cliext.write(mess);
            cliext.write("</body></html>\n");
  if(diags){Serial.print("talk...done (");Serial.print(strlen(mess));Serial.println(')');}            
}

void showMD() // display hexa locmem, extDetec, swCde
{
  if(diags){
    for(uint8_t i=0;i<4;i++){
      if(*(char*)(&locmem+3-i)<16){Serial.print('0');}Serial.print(*(char*)(&locmem+3-i),HEX);
      Serial.print(' ');
    }
    dumpstr((char*)&locmem-4,8);
    Serial.print("   ");
    for(uint8_t i=0;i<8;i++){
      if(cstRec.extDetec[8-i-1]<16){Serial.print('0');}Serial.print(cstRec.extDetec[8-i-1],HEX);
      Serial.print(' ');
    }
    Serial.print(" swCde=");if(cstRec.swCde<16){Serial.print('0');}Serial.print(cstRec.swCde,HEX);Serial.print(' ');
    Serial.println();
  }
}

void showBS(char* buf)
{
  Serial.print("BS=");Serial.println(buf);delay(10);
}

void answer(const char* what)
{
#ifdef ANALYZE
  ANSW
#endif // ANALYZE  
  Serial.print(" answer:");Serial.println(what);
  bufServer[0]='\0';
  #define FILL   9    // 9 = 4 len + 2 crc + 1 '=' + 1 '_' + 1 '\0'
  if(memcmp(what,"data_save_",LENNOM)==0 || memcmp(what,"data_na___",LENNOM)==0){
    buildData("data_na___",tempStr());                          // suite à un ordre reçu, on n'attend pas de réponse du serveur
                                                                // les évenutelles màj de memdet etc... ont été passées dans l'ordre
    //buildMess("data_save_","02_84.F3.EB.CC.5F.85_+0.00_000000_0.00_2.0B_2xx00_3100x_1111_WNE123*1_","\0");
    //buildMess("done______","02_84.F3.EB.CC.5F.85_+0.00_000000_0.00_2.0B_2xx00_3100x_1111_WNE123*1_","\0");
  }
  else {
    if(strlen(what)>=LBUFSERVER-LENNOM-FILL){buildMess("done______","***OVF***","\0");}
    else {buildMess("done______",what,"\0",diags);}
  }
  
  if(diags){showBS(bufServer);}
  talkClient(bufServer);
  Serial.print(' ');Serial.println(millis());
  cliext.stop();
  ledblink(4,PULSEBLINK);                        // connexion réussie 
#ifdef ANALYZE
  ANSWE
#endif // ANALYZE  
  answerCnt++;
  showMD();
  clkFastStep=0;delay(1);   
}

void rcvOrdreExt(char* data)  
/*  charge dans les variables locales le message reçu 
    update pulses, update actions, update sorties
    renvoie les données du périphérique (dont l'état des switchs)
    tout doit être effectué avant que waitRefCli() du serveur ne tombe en TO (délai TOFINCHCLI 1sec?)
*/
{
  if(diags){Serial.println(data);}
  dataTransfer(data);
  showMD();
  pulseClk();actions();outputCtl();  // récup data,compute rules,exec résultat // 9,2/6,3mS
  answer("data_na___");
}

void ordreExt()
/* timings cde set_______ answer datasave__ from server.available()
            4,9mS rcv
            1,0mS chk
            0,6mS tfr+actions vides
            2,7mS answer (write)  + delay Serial.print
            4,2mS cli.stop() (variable)
    total  13,4mS  
*/
{
  if(server!=nullptr && talkSta()==0 && wifiConnexion(ssid,ssidPwd,NOPRINT)){     
  // server démarré, pas de com->SH en cours, wifi on    
  
    cliext = server->available();

    if (cliext) {ordreExt0();}
  }
}

void ordreExt0()  // 'cliext = server->available()' déjà testé
{

#ifdef ANALYZE
  SRVAV // 0mS
#endif // ANALYZE

      Serial.print(millis());Serial.print(" Cliext ");
      memset(httpMess,0x00,LHTTPMESS);
      uint16_t hm=0;
      char c;
      unsigned long trx=millis();

      while (cliext.connected()) {
#define TO_ORDREXT 50
        if((millis()-trx)>TO_ORDREXT){Serial.print("TO_available");break;}
        if (cliext.available()) {
          c = cliext.read();//Serial.print(c);
          httpMess[hm]=c;
          if (c == '\n' && hm!=0 && httpMess[hm-1]==c) {break;}     // 2 LF fin requete => sortie boucle while
          if (hm>=LHTTPMESS){Serial.print("buffer OVF");break;}
          hm++;
        }
      } // while connected
      // format message "GET /FFFFFFFFFF=nnnn....CC" FFFFFFFFFF instruction (ETAT___, SET____ etc)
      //                                             nnnn nombre de car décimal zéros à gauche 
      //                                             .... éventuels arguments de la fonction
      //                                             CC crc
#ifdef ANALYZE
  RCVEND  // 4,9mS
#endif
      Serial.print(' ');Serial.println(millis());
      if(diags){
        Serial.print(" reçu(");Serial.print(hm);Serial.print(")  =");
        Serial.print(strlen(httpMess));Serial.print(" httpMess=");Serial.println(httpMess);}
     
      int v0=-1;
      char* vx=strstr(httpMess,"GET /");
      if(vx>=0){v0=vx-httpMess;}
      if(v0>=0){                            // si commande GET trouvée contrôles et décodage nom fonction 
        int jj=4,ii=convStrToNum(httpMess+v0+5+10+1,&jj);   // recup eventuelle longueur
        if(v0+5+10+1+ii+2<LHTTPMESS){
          httpMess[v0+5+10+1+ii+2]=0x00;      // place une fin ; si long invalide check sera invalide
        }
        int checkMess=checkHttpData(&httpMess[v0+5],&fonction);
        if(checkMess==MESSOK){
          showMD();
          Serial.print("rcv fnct=");Serial.print(fonction);Serial.print("  ");
#ifdef ANALYZE
  FORCV   // 5,9mS
#endif // ANALYZE        

          switch(fonction){
            case  0: rcvOrdreExt(&httpMess[v0+5]);break;        // tfr data , pulseClk , action , outputCtl , answer 
            case  1: answer("ack_______");break;                // ack ne devrait pas se produire (page html seulement)
            case  2: answer("data_save_");break;                // dataread/save   http://192.168.0.6:80/etat______=0006xxx
            case 10: rcvOrdreExt(&httpMess[v0+5]);break;
            case  3: break;                                     // reset (future use)
            case  4: break;                                     // sleep (future use)
            
            case  5: digitalWrite(pinSw[0],cloSw[0]);answer("0_ON______");delay(1000);digitalWrite(pinSw[0],cloSw[0]);break;   // test on  A  1sec  http://xxx.xxx.xxx.xxx:nnnn/sw0__ON___=0005_5A
            case  6: digitalWrite(pinSw[0],openSw[0]);answer("0_OFF_____");delay(1000);digitalWrite(pinSw[0],openSw[0]);break;   // test off A  1sec  http://192.168.0.6:80/sw0__OFF__=0005_5A
            case  7: digitalWrite(pinSw[1],cloSw[1]);answer("1_ON______");delay(1000);digitalWrite(pinSw[1],cloSw[1]);break;   // test on  B  1sec  http://82.64.32.56:1796/sw1__ON___=0005_5A
            case  8: digitalWrite(pinSw[1],openSw[1]);answer("1_OFF_____");delay(1000);digitalWrite(pinSw[1],openSw[1]);break;   // test off B  1sec  adresse/port indifférent crc=5A
            
            case  9: // mail
              #ifdef MAIL_SENDER                      
                      if(diags){Serial.print(">>>>>>>>>>> len=");Serial.print(ii);Serial.print(" data=");Serial.println(httpMess+v0);}
                      v0+=21;
                      {uint16_t vx=strlen(httpMess);
                        if(vx<LBUFSERVER){
                          httpMess[vx-2]='\0';             // erase CRC                   
                          uint16_t v1=strstr(httpMess,"==")-httpMess;
                          httpMess[v1]='\0';
                          uint16_t v2=strstr(httpMess+v1+1,"==")-httpMess;
                          httpMess[v2]='\0';

                          answer("mail______");                   
                          mail(httpMess+v0,httpMess+v1+2,httpMess+v2+2);
                        }
                        else if(diags){Serial.println("overflow message fmail");}
                      }
              #endif // MAIL_SENDER
              #ifndef MAIL_SENDER
                      if(diags){Serial.println("no mail on this board");}
              #endif // MAIL_SENDER
                      break;                 
            case 11: // mail_init_
              #ifdef MAIL_SENDER
              #ifdef MAIL_CONFIG
                      if(diags){Serial.print(">>>>>>>>>>> len=");Serial.print(ii);Serial.print(" data=");Serial.println(httpMess+v0);}
                      Serial.print(">>>>>>>>>>> len=");Serial.print(ii);Serial.print(" data=");Serial.println(httpMess+v0);
                      v0+=21;
                      {uint16_t vx=strlen(httpMess);
                        if(vx<LBUFSERVER){
                          httpMess[vx-2]='\0';             // erase CRC                   
                          uint16_t v1=strstr(httpMess,"==")-httpMess;
                          httpMess[v1]='\0';
                          answer("mail_init_"); 
                          //Serial.print('!');Serial.print(httpMess+v0);Serial.println('!');
                          //Serial.print('!');Serial.print(httpMess+v1+2);Serial.println('!');
                          delay(100);                 
                          mailInit(httpMess+v0,httpMess+v1+2);
                        }
                        else if(diags){Serial.println("overflow message fmail_init_");}
                      }
              #endif // MAIL_CONFIG
              #ifndef MAIL_CONFIG
                      if(diags){Serial.println("no MAIL_CONFIG on this board");}
              #endif // MAIL_CONFIG
              #endif // MAIL_SENDER
              #ifndef MAIL_SENDER
                      if(diags){Serial.println("no mail on this board");}
              #endif // MAIL_SENDER
                      break;                
            default:break;
          }          
          Serial.println();
        }
        else {Serial.print("reçu ko :");Serial.print(checkMess);Serial.print(" ");Serial.println(httpMess);}
        //if(strstr(httpMess,"favicon")>0){htmlImg(&cliext,favicon,favLen);}
        cntreq++;
      }                   // une éventuelle connexion a été traitée si controles ko elle est ignorée
      //purgeCli(&cliext,diags); 
      cliext.stop();
//    }   // if(cliext (server.available)
//  }     // if(server!=nullptr){
#ifdef ANALYZE
  STOPALL         // 16,7mS/14,8 (set+dataSave)
#endif // ANALYZE
}       // ordreExt()

#endif // _SERVER_MODE


/* Read analog ----------------------- */

void readAnalog()
{
 #ifndef IRQCNT
 cstRec.analVal=analogRead(A0); 
 #endif
 #ifdef IRQCNT
 cstRec.analVal=(irqCnt&0x003fffff); // 2^22/1000=33554 ok pour 16 bits
 #endif
}

/* Thermostat ------------------------ */

void thermostat()
{
  if((cstRec.periCfg&=PERI_STA)!=0){
    if(temp/100>(cstRec.periAnal+0.5)){locmem&=~LOCMEM_STA_BIT;}
    else if(temp/100<(cstRec.periAnal-0.5)){
      locmem|=LOCMEM_STA_BIT;
      if(diags){
        Serial.print(temp/100);Serial.print(' ');Serial.print(cstRec.periAnal);Serial.print(' ');showMD();
      }
    } 
  }
  else locmem&=~LOCMEM_STA_BIT;
}

/* Output control -------------------- */

void outputCtl()        // cstRec.swCde contient 4 paires de bits disjoncteurs 0=DISJ ; 1=ON ; 2 FORCE
                        // le résultat des règles dans outSw encodé selon la carte openSW/cloSw
                        // (la valeur contenue dans swCde n'est pas forcément identique au contenu du périf dans periTable
                        // car l'éventuelle remote mère multiple est prise en compte dans disjValue() de frontal2)
                        // les ouvertures de switchs sont prioritaires.
                        // après une ouverture, un délai est respecté avant les fermetures pour assurer un non recouvrement
{
    if(diags){if(outSw!=old_outSw){Serial.print("outputCtl() ; outSw=");Serial.println(outSw);old_outSw=outSw;}}

    //if(oneShow){Serial.print("outputCtl swCde ");Serial.print(cstRec.swCde,HEX);}
    bool isOpenSw=false;
      for(uint8_t sw=0;sw<NBSW;sw++){                       // recherche de switch à ouvrir
        if(!((((cstRec.swCde>>(sw*2))&0x03)==2) || (((cstRec.swCde>>(sw*2))&0x03)!=0 && ((outSw>>sw)&0x01)!=0))){       // ni forcé ni (pas disjoncté et devenant on)
                                                                                                                        // ignorer restant fermé ou à fermer
            if(digitalRead(pinSw[sw])==cloSw[sw]){          // ignore les switchs déjà "open"
              isOpenSw=true;
              outPutDly=OUTPUTDLY;}
              //if(oneShow){Serial.print(" open:");Serial.println(talkSta());}
              digitalWrite(pinSw[sw],openSw[sw]);
              #ifdef PINLEDR
              if(sw==toogSw){digitalWrite(PINLEDR,LEDROFF);}                            
              #endif // PINLEDR
        }
      }
    if(!isOpenSw && (outPutDly-millis()>=OUTPUTDLY)){       // les fermetures quand pas d'ouvertures et délai terminé
      for(uint8_t sw=0;sw<NBSW;sw++){                       // recherche de switch à fermer
        if(((((cstRec.swCde>>(sw*2))&0x03)==2) || (((cstRec.swCde>>(sw*2))&0x03)!=0 && ((outSw>>sw)&0x01)!=0))){        // forcé ou (pas disjoncté et devenant on)
          isOpenSw=true;
          //if(oneShow){Serial.print(" close:");Serial.println(talkSta());}
          digitalWrite(pinSw[sw],cloSw[sw]);
          #ifdef PINLEDR
          if(sw==toogSw){digitalWrite(PINLEDR,LEDRON);}                            
          #endif // PINLEDR
        }
      }
    }
/*    
    for(uint8_t sw=0;sw<NBSW;sw++){
      if(((cstRec.swCde>>(sw*2))& (0x03)==2) || (((cstRec.swCde>>(sw*2))&0x03)!=0 && ((outSw>>sw)&0x01)!=0)){  
        digitalWrite(pinSw[sw],cloSw[sw]);}                                       // forced ou (disjoncteur ON et résultat règles ON)
                                                                                  
      else {digitalWrite(pinSw[sw],openSw[sw]);}                                  // disjoncté donc open value
    }
*/  
}

/* Read temp ------------------------- */

#if POWER_MODE==NO_MODE
void readTemp()
{
  if((((millis()-lastRefr)/1000)>cstRec.serverPer) && (talkSta()==0)){
    Serial.print(millis());Serial.print(" T ");Serial.print(lastRefr);Serial.print(" P ");Serial.println(cstRec.serverPer);
    lastRefr=millis();
    getTempEtc();
    talkReq();
  }
}
#endif

#if POWER_MODE!=NO_MODE
void readTemp()
{
  if(talkSta() == 0){     // !=0 ne peut se produire qu'en NO_MODE 
                          // (les autres modes terminent avec 0)

#if POWER_MODE==DS_MODE
uint16_t tempPeriod0=cstRec.tempPer;  // (sec) durée depuis dernier check température
#endif // PM==DS_MODE
#if POWER_MODE==PO_MODE
uint16_t tempPeriod0=PERTEMP;  // (sec) durée depuis dernier check température (fixe si PO_MODE : resistance 5111 ; sinon tempPer)
#endif // PM==PO_MODE
#if POWER_MODE==NO_MODE

    if(chkTrigTemp()){
      uint16_t tempPeriod0=(millis()-tempTime)/1000;   // (sec) durée depuis dernier check température
      trigTemp();
#endif // PM==NO_MODE

///* avance timer server ------------------- */
      cstRec.serverTime+=tempPeriod0;             // tempPeriod0 temps écoulé depuis dernier réveil/passage par readTemp
      
      ///* si temps maxi atteint depuis dernière cx, forçage cx */
      if(cstRec.serverTime>cstRec.serverPer){     // serverTime temps écoulé depuis dernier talkReq()
                                                  // serverPer max entre 2 talkReq
        getTempEtc();
        cstRec.serverTime=0;
        talkReq();
      }
      ///* si pas de ko en cours, getTemp() au cas où elle soit changée */
      else if (cstRec.serverPer!=PERSERVKO){  // si dernière cx wifi ko, pas de comm jusqu'à fin de tempo    
        getTempEtc();
        ///* temp (suffisament) changée ? */
        if(temp>cstRec.oldtemp+cstRec.tempPitch){
          cstRec.oldtemp=(int16_t)temp-cstRec.tempPitch/2; // new oldtemp décalé pour effet trigger (temp-tempPitch/2)
          cstRec.serverTime=0;
          talkReq(); 
        }
        else if(temp<cstRec.oldtemp-cstRec.tempPitch){
          cstRec.oldtemp=(int16_t)temp+cstRec.tempPitch/2; // new oldtemp décalé pour effet trigger (temp+tempPitch/2)
          cstRec.serverTime=0;
          talkReq(); 
        }
        ///* si pas de changt suffisant rien */
      }

#if POWER_MODE==NO_MODE
    }   // chkTigTemp
#endif // PM==NO_MODE

  }     // talkSta == 0
/* writeConstant avant sleep ou power off */
}
#endif

void getTemp()
{  
#if POWER_MODE!=DS_MODE
      unsigned long ms=millis();           // attente éventuelle de la fin de la conversion initiée à l'allumage
      if(diags){Serial.print(" Tconv=");Serial.print(tconversion);Serial.print(" delay=");Serial.print((long)ms-(long)debConv);}
#endif // PM!=DS_MODE
#if POWER_MODE==PO_MODE
      if(((long)ms-(long)debConv)<tconversion){
        delay((long)tconversion-(ms-debConv));
      }
#endif // PM==PO_MODE
#if POWER_MODE!=NO_MODE
      temp=ds1820.readDs(WPIN);
      temp*=100;                    // tempPitch 100x
      Serial.print(" temp ");Serial.print(temp/100);
#endif // PM!=NO_MODE
#if POWER_MODE==NO_MODE
#ifdef WPIN
      if(((long)ms-(long)debConv)>tconversion){
        temp=ds1820.readDs(WPIN);
        temp*=100;                  // tempPitch 100x
        ds1820.convertDs(WPIN);     // conversion pendant attente prochain accès
        debConv=millis();          
        Serial.print(" temp ");Serial.print(temp/100);Serial.print(" ");Serial.print(debConv);
      }
#endif // WPIN      
#endif // PM==NO_MODE
#if POWER_MODE==DS_MODE
      ds1820.convertDs(WPIN);     // conversion pendant deep/sleep ou pendant attente prochain accès
#endif // PM!=DS_MODE

      checkVoltage();
#ifdef IRQCNT
      Serial.print(irqCnt);Serial.print(' ');
#endif 
}

void getTempEtc()
{
  getTemp();
  #ifdef PWR_CSE7766
  getCSE7766();
  dataParFlag=true;
  #endif // CSE7766
}

bool wifiAssign()
{
  bool ret=true;

  if(ssidNb==1){ssid=cstRec.ssid1;ssidPwd=cstRec.pwd1;}
  else {ssid=cstRec.ssid2;ssidPwd=cstRec.pwd2;}

  if(!wifiConnexion(ssid,ssidPwd)){
    if(ssidNb==1){
      if(wifiConnexion(cstRec.ssid2,cstRec.pwd2)){
        ssidNb=2;ssid=cstRec.ssid2;ssidPwd=cstRec.pwd2;}
      else ret=false;
    }
    else {
      if(wifiConnexion(cstRec.ssid1,cstRec.pwd1)){
        ssidNb=1;ssid=cstRec.ssid1;ssidPwd=cstRec.pwd1;}
      else ret=false;
    }
  }
  return ret;
}

uint16_t getServerConfig()
{
  char bf[MAXSER];//*buf='\0';
  
  serPurge(0);
  for(uint8_t i=0;i<=TSCNB;i++){Serial.print(RCVSYNCHAR);}
  Serial.print(WIFICFG);
  
  uint16_t rcvl=0;                          // longueur effectivement reçue (strlen(bf))
  
  while(rcvl==0){rcvl=serialRcv(bf,MAXSER,0);}

  if(rcvl>5){                               // nnnn;  length
    Serial.print(strlen(bf));Serial.print(" ");Serial.println(bf);
    Serial.print("checkData : ");
    uint16_t ll=0;
    int cd=checkData(bf,&ll);               // longueur stockée dans le message
    Serial.print(cd);
    if(cd!=1){                              // renvoie mess = MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3) 
      Serial.println(" ko");return 0;}     
    Serial.println(" ok");

    char a=' ';
    char* b=bf;
    uint8_t cntpv=0;

    while(cntpv<2 && a!='\0' && b<(bf+rcvl)){a=*b++;if(a==';'){cntpv++;}}                       // skip len+name+version
    uint16_t temp=0;
    for(uint8_t i=0;i<4;i++){temp=0;conv_atob(b,&temp);b+=4;cstRec.serverIp[i]=temp;}           // serverIp  
    temp=0;conv_atob(b,&temp);b+=6;cstRec.serverPort=temp;                                      // serverPort
    b+=12;                                                                                      // skip remote+udp ports

    temp=0;a=' ';while(a!=';' && a!='\0' && b<(bf+rcvl) && temp<LPWD){a=b[temp];cstRec.peripass[temp]=a;temp++;}  // peripass
    cstRec.peripass[temp]='\0';b+=temp;

    temp=0;a=' ';while(a!=';' && a!='\0' && b<(bf+rcvl)){a=b[temp];cstRec.ssid1[temp]=a;temp++;}
    cstRec.ssid1[temp]='\0';b+=temp;
    temp=0;a=' ';while(a!=';' && a!='\0' && b<(bf+rcvl)){a=b[temp];cstRec.pwd1[temp]=a;temp++;}
    cstRec.pwd1[temp]='\0';b+=temp;

    temp=0;a=' ';while(a!=';' && a!='\0' && b<(bf+rcvl)){a=b[temp];cstRec.ssid2[temp]=a;temp++;}
    cstRec.ssid2[temp]='\0';b+=temp;
    temp=0;a=' ';while(a!=';' && a!='\0' && b<(bf+rcvl)){a=b[temp];cstRec.pwd2[temp]=a;temp++;}
    cstRec.pwd2[temp]='\0';b+=temp;
  }
  else {Serial.print(" ko ");Serial.println(rcvl);ledblink(BCODESDCARDKO,PULSEBLINK);}
  return rcvl;
}

#ifdef IRQCNT
ICACHE_RAM_ATTR void irqCntUpdate()
{
  irqCnt++;
}
#endif // IRQCNT
