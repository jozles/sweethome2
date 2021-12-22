
#include <ESP8266WiFi.h>
#include <shconst2.h>
#include <shutil2.h>
#include <shmess2.h>
#include <ds18x20.h>
#include "const.h"
#include "utilWifi.h"
#include "util.h"
#include "dynam.h"
#include "peripherique2.h"

#ifdef MAIL_SENDER
#include <EMailSender.h>                            // STORAGE_SD doit etre "ndef"
EMailSender emailSend("lucieliu66", "eicul666");
EMailSender::EMailMessage message;
#endif //MAIL_SENDER

extern "C" {                  
#include <user_interface.h>                 // pour struct rst_info, system_deep_sleep_set_option(), rtc_mem
}

#if CONSTANT==EEPROMSAVED
#include <EEPROM.h>
#endif

#ifdef CAPATOUCH
#include <capaTouch.h>
uint8_t keys[]={KEY2,KEY1};
Capat capaKeys;
#endif

Ds1820 ds1820;
//extern byte dsmodel;

  char model[LENMODEL];

  unsigned long dateOn=millis();          // awake start time
  unsigned long boucleTime=millis();

  const char* ssid;                       // current ssid
  const char* ssidPwd;                    // current ssid pwd 
#define DEVOLO  
#ifdef DEVOLO
  const char* ssid2; //= "pinks";
  const char* password2; //= "cain ne dormant pas songeait au pied des monts";
  const char* ssid1; //= "devolo-5d3";
  const char* password1; //= "JNCJTRONJMGZEEQL";
#endif // DEVOLO
#ifndef DEVOLO
  const char* ssid1= "pinks";
  const char* password1 = "cain ne dormant pas songeait au pied des monts";
  const char* ssid2= "devolo-5d3";
  const char* password2= "JNCJTRONJMGZEEQL";
#endif // DEVOLO
//*/
//  const IPaddr* host//="192.168.0.35";  //= HOSTIPADDR2;         // HOSTIPADDRx est une chaine de car donc de la forme "192.168.0.xxx"
//  const int   port=1786;  //= PORTPERISERVER2; 

WiFiClient cli;                           // instance du serveur externe (utilisé pour dataread/save)

#ifdef  _SERVER_MODE
WiFiClient cliext;                        // instance du serveur local
WiFiServer* server=nullptr;
bool serverStarted=false;

  #define LHTTPMESS 500
  char   httpMess[LHTTPMESS];             // buffer d'entrée en mode serveur
#endif // _SERVER

  #define LSRVTEXTIP TEXTIPADDRLENGTH+1
  char textServerIp[LSRVTEXTIP];                  // sweethome server alpha IpAddr

// enregistrement pour serveur externe

  char  bufServer[LBUFSERVER];            // buffer des envois/réceptions de messages
  int   periMess;                         // diag de réception de message

  const char* fonctions={"set_______ack_______etat______reset_____sleep_____sw0__ON___sw0__OFF__sw1__ON___sw1__OFF__mail______last_fonc_"};
  uint8_t fset_______,fack_______,fetat______,freset_____,fsleep_____,ftestaoff__,ftesta_on__,ftestboff__,ftestb_on__,fmail______;
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
  uint32_t       locmem=0;                  // local mem rules bits


  /* paramètres switchs (les états et disjoncteurs sont dans cstRec.SWcde) */

  uint8_t pinSw[MAXSW]={PINSWA,PINSWB,PINSWC,PINSWD};       // switchs pins
  uint8_t cloSw[MAXSW]={CLOSA,CLOSB,CLOSC,CLOSD};           // close value for every switchs (relay/triac etc ON)
  uint8_t openSw[MAXSW]={OPENA,OPENB,OPENC,OPEND};          // open value for every switchs (relay/triac etc OFF)
  byte    staPulse[NBPULSE];                                // état clock pulses
  unsigned long    impDetTime[NBPULSE];                     // timer pour gestion commandes impulsionnelles     
  uint8_t pinDet[MAXDET]={PINDTA,PINDTB,PINDTC,PINDTD};     // les détecteurs

  int   i=0,j=0,k=0;
  uint8_t oldswa[]={0,0,0,0};         // 1 par switch

constantValues cstRec;

char* cstRecA=(char*)&cstRec.cstlen;

  float voltage=0;                    // tension alim
  
  byte  mac[6];
  //char  buf[3];                       //={0,0,0};

  int   cntreq=0;

#if POWER_MODE != NO_MODE
  ADC_MODE(ADC_VCC);
#endif

  const char*     chexa="0123456789ABCDEFabcdef\0";
  //const byte      mask[]={0x00,0x01,0x03,0x07,0x0F};
  uint32_t  memDetServ=0x00000000;    // image mémoire NBDSRV détecteurs (32)  
  uint32_t  mDSmaskbit[]={0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                       0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                       0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                       0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};

  bool diags=true;
  unsigned long t_on=millis();


   /* prototypes */

void talkServer();
void talkClient(char* etat);

int act2sw(int sw1,int sw2);
uint8_t runPulse(uint8_t sw);

void  getTemp();
char* tempStr();
int   buildData(const char* nomfonction,const char* data);
int   dataSave();
int   dataRead();
void  dataTransfer(char* data);  
void  readTemp();
void  ordreExt();
void  outputCtl();
void  readAnalog();
uint16_t  getServerConfig();

#ifdef MAIL_SENDER
void mail(char* subj,char* dest,char* msg);
#endif // MAILSENDER

void tmarker()
{
  pinMode(WPIN,OUTPUT);for(int t=0;t<6;t++){digitalWrite(WPIN,LOW);delayMicroseconds(100);digitalWrite(WPIN,HIGH);delayMicroseconds(100);}
}


void setup() 
{ 

/* >>>>>> pins Init <<<<<< */

  checkVoltage();                   // power off au plus vite si tension insuffisante (no serial)

/*   stop modem
#if POWER_MODE!=NO_MODE
WiFi.disconnect();
WiFi.forceSleepBegin();
delay(1);
#endif // PM!=NO_MODE
*/

  Serial.begin(115200);delay(10);
  //while(1){delay(1);}

/*
char* dcrc1={"82.64.32.56:1796/sw0__OFF__=0005_"};    // 88
int ldcrc1=strlen(dcrc1);
//Serial.println(calcCrc(dcrc1,ldcrc1),HEX);
Serial.println(calcCrc("0005_",5),HEX);   // 5A

char* dcrc2={"82.64.32.56:1796/sw0__ON___=0005_"};    // ED
int ldcrc2=strlen(dcrc2);
Serial.println(calcCrc(dcrc2,ldcrc2),HEX);

char* dcrc3={"82.64.32.56:1796/sw1__OFF__=0005_"};    // A4
int ldcrc3=strlen(dcrc3);
Serial.println(calcCrc(dcrc3,ldcrc3),HEX);

char* dcrc4={"82.64.32.56:1796/sw1__ON___=0005_"};    // C1
int ldcrc4=strlen(dcrc4);
Serial.println(calcCrc(dcrc4,ldcrc4),HEX);
*/


#if POWER_MODE==PO_MODE
Serial.println("\n+");
delay(1);

  digitalWrite(PINPOFF,LOW);
  pinMode(PINPOFF,OUTPUT);
#endif // PM==PO_MODE

#if POWER_MODE==NO_MODE
  diags=false;
  delay(1000);
  //Serial.print("\nSerial buffer size =");Serial.println(Serial.getRxBufferSize());
  Serial.print("\nstart setup ");Serial.print(VERSION);
  Serial.print(" power_mode=");Serial.print(POWER_MODE);
  Serial.print(" carte=");Serial.print(CARTE);
  Serial.print(" ; une touche pour diags ");
  while((millis()-t_on)<6000){Serial.print(".");delay(500);if(Serial.available()){Serial.read();diags=true;break;}}
  Serial.println();
#endif // PM==NO_MODE  

#ifdef CAPATOUCH
    capaKeys.init(SAMPLES,COMMON,keys,KEYNB);
    capaKeys.calibrate();
#endif    

  initLed();

#if CARTE==VR || CARTE==VRR || CARTE==VRDEV
  for(uint8_t sw=0;sw<MAXSW;sw++){
    digitalWrite(pinSw[sw],openSw[sw]);
    pinMode(pinSw[sw],OUTPUT);}

#ifndef CAPATOUCH
  pinMode(PINDTA,INPUT_PULLUP);
  pinMode(PINDTB,INPUT_PULLUP);  
  pinMode(PINDTC,INPUT_PULLUP);  
#endif // CAPATOUCH
#endif // VR||VRR

#ifdef PININT_MODE 
  pinMode(PININTA,INPUT_PULLUP);
  pinMode(PININTB,INPUT_PULLUP); 
  
  //if(digitalRead(PINDTA==0) || digitalRead(PININTA)==0 || (digitalRead(PININTA)!=0 && digitalRead(PININTB)!=0)){cntIntA=1;}
// fonctions d'interruption
// isrD[0]=isrD0;
// isrD[1]=isrD1;
// isrD[2]=isrD2;
// isrD[3]=isrD3;
#endif // PININT_MODE

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
  ftestaoff__=(strstr(fonctions,"testaoff__")-fonctions)/LENNOM;
  ftesta_on__=(strstr(fonctions,"testa_on__")-fonctions)/LENNOM;
  ftestboff__=(strstr(fonctions,"testboff__")-fonctions)/LENNOM;  
  ftestb_on__=(strstr(fonctions,"testb_on__")-fonctions)/LENNOM;
  fmail______=(strstr(fonctions,"mail______")-fonctions)/LENNOM;

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

/* >>>>>> gestion variables permanentes <<<<<< */

#if CONSTANT==EEPROMSAVED
  Serial.print("EEPROM ");
  EEPROM.begin(512);
#endif
#if CONSTANT==RTCSAVED
  Serial.print("RTC ");
#endif


/* si erreur sur les variables permanentes (len ou crc faux), initialiser et sauver */
initConstant();
  if(!readConstant()){   
    Serial.println("KO -> init ");
    initConstant();
    yield();
    if(cstRec.cstlen!=LENCST){Serial.print(" len RTC=");Serial.print(cstRec.cstlen);Serial.print("/");Serial.print(LENCST);
    while(1){blink(1);delay(1000);}} // blocage param faux, le programme a changé
  }

  printConstant();

  // Ip au format texte pour MessToServer
  // charIp ne fonctionne pas avec le format IPAddress ; changer pour byte* modifie cstRec
  char buf[TEXTIPADDRLENGTH+1];memset(buf,0X00,TEXTIPADDRLENGTH+1);
  for(uint8_t i=0;i<4;i++){
    sprintf(buf+strlen(buf),"%d",cstRec.serverIp[i]);if(i<3){strcat(buf,".");}}
  memcpy(textServerIp,buf,LSRVTEXTIP);
  Serial.print("cstRec.serverIp ");Serial.print((IPAddress)cstRec.serverIp);//Serial.print(" textServerIp ");Serial.println(textServerIp);

  Serial.print(" time=");Serial.println(millis()-debTime);

  #define FRDLY 5  // sec
#if CARTE != THESP01
  pinMode(PINDTC,INPUT);
  if(digitalRead(PINDTC)==LOW){
    blink(4);delay(2000);
    yield();
    if(getServerConfig()>0){writeConstant();while(1){blink(1);delay(1000);}} // getServerConfig bloque si ko
  }
#endif    
  Serial.println();
  Serial.print("locmem=");Serial.println(locmem,HEX);

#if POWER_MODE==NO_MODE
  cstRec.talkStep=0;
  cstRec.serverTime=cstRec.serverPer+1;
  talkReq(); 

  memdetinit();pulsesinit();
  Serial.print("ssid=");Serial.print(cstRec.ssid1);Serial.print(" - ");Serial.println(cstRec.ssid2);
  yield();
  
#ifdef  _SERVER_MODE
  Serial.print("_SERVER_MODE ");
  clkFastStep=1;talkReq();                  // forçage com pour acquisition port perif server
  
  ssid=cstRec.ssid1;ssidPwd=cstRec.pwd1;    // setup wifi pour ordreExt()
  if(!wifiConnexion(ssid,ssidPwd)){
    ssid=cstRec.ssid2;ssidPwd=cstRec.pwd2;
    wifiConnexion(ssid,ssidPwd);            // tentative sur ssid bis si existe
  }
#endif // def_SERVER_MODE

  Serial.println(">>>> fin setup\n");
  actionsDebug();
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
  
      if(millis()>(clkTime+PERFASTCLK)){        // période 5mS/step
        ordreExt();
        switch(clkFastStep++){

/* En 1 toujours talkstep pour le forçage de communication d'acquisition du port au reset 
 * En 2 ordreExt() pour assurer que le traitement des commandes reçues et la collecte des données soit effectués pour le prochain dataSave
 * (ordreExt() positionne cstRec.talkStep!=0)
 * clkFastStep et cstRec.talkStep == 1 
*/
          case 1:   if(cstRec.talkStep!=0){talkServer();}break;
          case 2:   break;
          case 3:   wifiConnexion(ssid,ssidPwd,NOPRINT);break;
          case 4:   break;
          case 5:   actions();break;
          case 6:   outputCtl();break;
          case 7:   ledblink(-1);break;
          case 8:   swDebounce();break;         // doit être avant polDx
          case 9:   polAllDet();break;          // polDx doit être après swDebounce                            
          case 10:  clkFastStep=0;              // période 50mS/step
                    switch(clkSlowStep++){
                      case 1:   break;
                      case 2:   pulseClkisr();break;
                      case 3:   break;
                      case 4:   pulseClkisr();break;
                      case 5:   break;
                      case 6:   pulseClkisr();break;
                      case 7:   readAnalog();break;
                      case 8:   pulseClkisr();break;
                      case 9:   readTemp();
                                Serial.print("!");Serial.print(digitalRead(PINDTC));
                                break;
                      case 10:  pulseClkisr();
                                clkSlowStep=0;
                                break;
                      default:  break;
                    }
                    break;
          default:  //if(clkFastStep==0){Serial.print(" cFS0 ");}
          break;
        }
        clkTime=millis();
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

readTemp() gestion communications cycliques (déclenche talkServer)

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
          
          if(fonction==fwaited){dataTransfer(bufServer);}
          else {periMess=MESSFON;}
        }
        return periMess;
}

void dataTransfer(char* data)           // transfert contenu de set ou ack dans variables locales selon contrôles
                                        // data sur fonction
                                        //    contrôle mac addr et numPeriph ;
                                        //    si pb -> numPeriph="00" et ipAddr=0
                                        //    si ok -> tfr params
                                        // retour periMess
{
  int  ddata=16;                        // position du numéro de périphérique  
  byte fromServerMac[6];
  
        periMess=MESSOK;
        packMac(fromServerMac,(char*)(data+ddata+3));
        if(memcmp(data+ddata,"00",2)==0){periMess=MESSNUMP;}
        else if(!compMac(mac,fromServerMac)){periMess=MESSMAC;}
        else {
                             // si ok transfert des données
if(diags){Serial.println(" dataTransfer() ");}                              
            memcpy(cstRec.numPeriph,data+MPOSNUMPER,2);                         // num périph

            int sizeRead;
            cstRec.serverPer=(long)convStrToNum(data+MPOSPERREFR,&sizeRead);    // per refresh server
            cstRec.tempPer=(uint16_t)convStrToNum(data+MPOSTEMPPER,&sizeRead);  // per check température (invalide/sans effet en PO_MODE)
            cstRec.tempPitch=(long)convStrToNum(data+MPOSPITCH,&sizeRead);      // pitch mesure (100x)
            conv_atoh(data+MPOSANALH,(byte*)&cstRec.analLow);conv_atoh(data+MPOSANALH+2,(byte*)&cstRec.analLow+1);      // analogLow
            conv_atoh(data+MPOSANALH+4,(byte*)&cstRec.analHigh);conv_atoh(data+MPOSANALH+6,(byte*)&cstRec.analHigh+1);  // analogHigh
            
            uint8_t mskSw[] = {0xfd,0xf7,0xdf,0x7f};
            for(uint8_t i=0;i<MAXSW;i++){                                       // 1 byte état/cdes serveur + 4 bytes par switch (voir const.h du frontal)
              cstRec.swCde &= mskSw[i];
              cstRec.swCde |= (*(data+MPOSSWCDE+i)-48)<<((2*(MAXSW-i))-1);}     // bit cde (bits 8,6,4,2 pour switchs 3,2,1,0)  

            uint8_t i1=NBPERINPUT*PERINPLEN;                                    // size inputs
            byte bufoldlev[i1];
            for(uint8_t k=0;k<i1;k++){                                          // inputs !!! ne pas écraser les bits oldlevel !!!
              conv_atoh((data+MPOSPERINPUT+2*k),&bufoldlev[k]);
              if((k%4)==2){bufoldlev[k]&=~PERINPOLDLEV_VB;bufoldlev[k]|=cstRec.perInput[k]&PERINPOLDLEV_VB;}
            }
            memcpy(&cstRec.perInput,bufoldlev,i1);
            
            for(int i=0;i<NBPULSE;i++){                                         // pulses values NBPULSE*ONE+NBPULSE*TWO
              cstRec.durPulseOne[i]=(long)convStrToNum(data+MPOSPULSONE+i*(LENVALPULSE+1),&sizeRead);
              cstRec.durPulseTwo[i]=(long)convStrToNum(data+MPOSPULSTWO+i*(LENVALPULSE+1),&sizeRead);}

            for(int ctl=PCTLLEN-1;ctl>=0;ctl--){                                // pulses control
              conv_atoh((data+MPOSPULSCTL+ctl*2),&cstRec.pulseMode[ctl]);}
   
            for(int k=0;k<MDSLEN;k++){                                          // détecteurs externes
              conv_atoh((data+MPOSMDETSRV+k*2),((byte*)&cstRec.extDetec)+MDSLEN-1-k);
              }   

            cstRec.periPort=(uint16_t)convStrToNum(data+MPOSPORTSRV,&sizeRead);    // port server
            //printConstant();
          #ifdef _SERVER_MODE
            if(server==nullptr){server=new WiFiServer(cstRec.periPort);}
          #endif
        }
        if(periMess!=MESSOK){
          memcpy(cstRec.numPeriph,"00",2);cstRec.IpLocal=IPAddress(0,0,0,0);
        }
}


int buildData(const char* nomfonction,const char* data)             // assemble une fonction data_read_ ou data_save_
{                                                   // et concatène dans bufServer - retour longueur totale
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
      memcpy(message+sb+5,VERSION,LENVERSION);                       // VERSION contient le "_"     - 3
      char ds='B';if(ds1820.dsmodel==MODEL_S){ds='S';}
      memcpy(message+sb+5+LENVERSION-1,&ds,1);                       // modele DS18x20              - 2
      memcpy(message+sb+5+LENVERSION,"_\0",2);
      
      sb+=5+LENVERSION+1;
      message[sb]=(char)(NBSW+48);                                    // nombre switchs              - 1   
//      for(i=(NBSW-1);i>=0;i--){message[sb+1+(NBSW-1)-i]=(char)(48+digitalRead(pinSw[i]));}   
      for(i=0;i<NBSW;i++){
        uint8_t ssw=0;if(digitalRead(pinSw[i])==cloSw[i]){ssw=1;}
        message[sb+1+(MAXSW-1)-i]=(char)(48+ssw);}                    // état                        - 5
      if(NBSW<MAXSW){for(i=NBSW;i<MAXSW;i++){message[sb+1+(MAXSW-1)-i]='x';}}message[sb+5]='_';

      sb+=MAXSW+2;
      message[sb]=(char)(NBDET+48);                                   // nombre détecteurs
      for(i=(NBDET-1);i>=0;i--){message[sb+1+(NBDET-1)-i]=(char)(chexa[cstRec.memDetec[i]]);} // état -6 
      if(NBDET<MAXDET){for(i=NBDET;i<MAXDET;i++){message[sb+1+i]='x';}}                              
      strcpy(message+sb+1+MAXDET,"_\0");

      sb+=MAXDET+2;
      for(i=0;i<NBPULSE;i++){message[sb+i]=chexa[staPulse[i]];}
      strcpy(message+sb+NBPULSE,"_\0");                               // clock pulse status          - 5

      sb+=NBPULSE+1;
      memcpy(message+sb,model,LENMODEL);

      strcpy(message+sb+LENMODEL,"_\0");                                                      //      - 7
      sb+=LENMODEL+1;
      bool noZero=false;
      for(i=0;i<NBPULSE;i++){
        //Serial.print("=======================staPulse[");Serial.print(i);Serial.print(")=");Serial.println(staPulse[i]);
        if(staPulse[i]!=PM_DISABLE && staPulse[i]!=PM_IDLE){noZero=true;break;}
      }
      if(noZero){
        for(int i=0;i<NBPULSE*2;i++){                                                                                           // loop compteurs (4*2)
          uint32_t currt=0;
          byte* pcurr=(byte*)&currt;
          if(cstRec.cntPulse[i]!=0)
          {
            currt=(millis()-cstRec.cntPulseOne[i])/1000;
          }
          for(uint8_t j=0;j<sizeof(uint32_t);j++)
          {
            conv_htoa((char*)(message+sb+2*(i*sizeof(uint32_t)+j)),(byte*)(pcurr+j));
          }       // loop bytes (4/8)
        }
        sb+=NBPULSE*2*sizeof(uint32_t)*2+1;
      } 
      strcpy(message+sb-1,"_\0");

  if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL);}      
  
  return buildMess(nomfonction,message,"",diags);        // concatène et complète dans bufserver
}

int buildReadSave(const char* nomfonction,const char* data)   // construit et envoie une commande GET complète
                                                  //   avec fonction peri_pass_ + dataRead ou dataSave
                                                  //   peri_pass_=nnnnpppppp..cc?
                                                  //   data_rs.._=nnnnppmm.mm.mm.mm.mm.mm_[-xx.xx_aaaaaaa_v.vv]_r.r_siiii_diiii_ffff_cc
                                                  //   (sortie MESSCX connexion échouée)                                                  
{
  strcpy(bufServer,"GET /cx?\0");
  if(!buildMess("peri_pass_",cstRec.peripass,"?",diags)==MESSOK){
    if(diags){Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(cstRec.peripass);return MESSDEC;};}

  buildData(nomfonction,data);

  return messToServer(&cli,textServerIp,cstRec.serverPort,bufServer); 
}

char* tempStr()
{
      memset(tempstr,0x00,LTEMPSTR);
      sprintf(tempstr,"%+02.2f",temp/100);                                // 6 car
      //Serial.print(temp);Serial.print(" ");Serial.println(tempstr);
      if(strstr(tempstr,"nan")!=0){strcpy(tempstr,"+00.00\0");}
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

/* ----------------- talkServer ------------------ */

void talkReq()
{
  cstRec.talkStep|=TALKREQBIT;
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

void talkKo()
{
  memcpy(cstRec.numPeriph,"00",2);
  talkClr();
  //talkReq();                    // ???????????????????
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


void talkServer()   // si numPeriph est à 0, dataRead pour se faire reconnaitre ; 
                    // si ça fonctionne réponse numPeriph!=0 ; dataSave 
                    // renvoie 0 et periMess valorisé si la com ne s'est pas bien passée.
{
//uint8_t ts=cstRec.talkStep;
//if(ts!=0){Serial.print(" tS");Serial.print(ts,HEX);}

dateOn=millis();

if((cstRec.talkStep&TALKREQBIT)!=0 && (cstRec.talkStep&TALKCNTBIT)==0){cstRec.talkStep+=TALKWIFI1;}

//if(ts!=0){Serial.print("->");Serial.print(cstRec.talkStep,HEX);}

switch(cstRec.talkStep&=TALKCNTBIT){
  case 0:break;
  
  case TALKWIFI2:
      ssid=cstRec.ssid2;ssidPwd=cstRec.pwd2; // tentative sur ssid bis
      if(wifiConnexion(ssid,ssidPwd)){talkSet(TALKDATA);}
      else {talkWifiKo();}
      break;
  
  case TALKWIFI1:
      ssid=cstRec.ssid1;ssidPwd=cstRec.pwd1;
      //Serial.print("+");
      if(!wifiConnexion(ssid,ssidPwd)){talkSet(TALKWIFI2);break;}
      if((millis()-dateOn)>1){talkSet(TALKDATA);break;}

  case TALKDATA:        // connecté au wifi
                        // si le numéro de périphérique est 00 ---> récup (dataread), ctle réponse et maj params
      talkGrt();
      if(memcmp(cstRec.numPeriph,"00",2)==0){
        if(dataRead()==MESSOK && fServer(fset_______)==MESSOK){
          talkSet(TALKDATASAVE);}
        else {talkKo();}    // pb com -> recommencer au prochain timing
        break;
      }
              
  case TALKDATASAVE:          // (6) si numPeriph !=0 ou réponse au dataread ok -> datasave
                              // sinon recommencer au prochain timing                              
      if(dataSave()!=MESSOK){talkKo();}
                  
      else if(fServer(fack_______)!=MESSOK){talkKo();}   // si ko recommencer au prochain timing             
        
      else {
        talkClr();  // terminé ; tout s'est bien passé les 2 côtés sont à jour 

#ifdef  _SERVER_MODE
        if(server!=nullptr){      //} && !serverStarted){
          server->begin(cstRec.periPort);
          serverStarted=true;
          Serial.print(" server.begin:");Serial.println((int)cstRec.periPort);
        }
#endif // def_SERVER_MODE
      }
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

    //Serial.println("---mail---");
    
    wifiConnexion(ssid,ssidPwd);

    char s[64]={"sh "};strcat(s,subj);
    message.subject = s;
    message.message = msg ;

    EMailSender::Response resp = emailSend.send(dest, message);

Serial.print(">>> email millis()=");Serial.println(millis()-beg);
}
#endif // MAIL_SENDER

void talkClient(char* etat) // réponse à une requête
{
  
            // en-tête réponse HTTP 
            cliext.write("HTTP/1.1 200 OK\n");
            cliext.write("Content-type:text/html\n");
            cliext.write("Connection: close\n\n");
            // page Web 
            cliext.write("<!DOCTYPE html><html>\n");
            //cliext.println("<head></head>");
            
            cliext.write("<body>");
            cliext.write(etat);//Serial.print(etat);
            cliext.write("</body></html>\n");
}

void answer(const char* what)
{
  Serial.print(" ");Serial.println(what);
  bufServer[0]='\0';
  #define FILL   9    // 9 = 4 len + 2 crc + 1 '=' + 1 '_' + 1 '\0'
  if(memcmp(what,"data_save_",LENNOM)==0){buildData("data_save_",tempStr());}
  else {
    if(strlen(what)>=LBUFSERVER-LENNOM-FILL){buildMess("done______","***OVF***","\0");}
    else {buildMess("done______",what,"\0",diags);}
  }
  talkClient(bufServer);
  ledblink(4);                        // connexion réussie 
}

void ordreExt()
{
  //uint32_t boe=millis();

  if(server!=nullptr && talkSta()==0 && wifiConnexion(ssid,ssidPwd,NOPRINT)){     
  // server démarré, pas de com->SH en cours, wifi on    
  
    uint16_t hm=0,nl=0;
    memset(httpMess,0x00,LHTTPMESS);
  
    cliext = server->available();

    if (cliext) {

      unsigned long trx=0;
      char c;
      Serial.print("\nCliext ");
      while (cliext.connected()) {
#define TO_ORDREXT 10
        if(trx==0){trx=millis();}
        if((millis()-(unsigned long)trx)>TO_ORDREXT){Serial.print("TO_available");break;}
        if (cliext.available()) {
          c = cliext.read();
          //Serial.print(c);
          httpMess[hm]=c;
          if (c == '\n') {
            if(nl==0){break;}                      // 2 LF fin requete => sortie boucle while
            else nl=0;
          }
          if(hm<LHTTPMESS){hm++;nl++;}
          trx=0;
        }
      } // while connected
      // format message "GET /FFFFFFFFFF=nnnn....CC" FFFFFFFFFF instruction (ETAT___, SET____ etc)
      //                                             nnnn nombre de car décimal zéros à gauche 
      //                                             .... éventuels arguments de la fonction
      //                                             CC crc

      //Serial.println();
      if(diags){}
        Serial.print(" reçu(");Serial.print(hm);Serial.print(")  =");Serial.print(strlen(httpMess));Serial.print(" httpMess=");Serial.println(httpMess);
     
      int v0=-1;
      char* vx=strstr(httpMess,"GET /");
      if(vx>=0){v0=vx-httpMess;}
      if(v0>=0){                            // si commande GET trouvée contrôles et décodage nom fonction 
        int jj=4,ii=convStrToNum(httpMess+v0+5+10+1,&jj);   // recup eventuelle longueur
        httpMess[v0+5+10+1+ii+2]=0x00;      // place une fin ; si long invalide check sera invalide

        if(checkHttpData(&httpMess[v0+5],&fonction)==MESSOK){
          Serial.print("reçu message fonct=");Serial.print(fonction);
          switch(fonction){
              case 0: dataTransfer(&httpMess[v0+5]);actions();outputCtl();  // récup data,compute rules,exec résultat 
                      answer("data_save_");break;                       // set ---> réponse message data_save_ complet
              case 1: answer("ack_______");break;                       // ack ne devrait pas se produire (page html seulement)
              case 2: answer("etat______");talkReq();break;     // etat -> dataread/save   http://192.168.0.6:80/etat______=0006xxx
              case 3: break;                                            // sleep (future use)
              case 4: break;                                            // reset (future use)
              case 5: digitalWrite(pinSw[0],cloSw[0]);answer("0_ON______");delay(1000);break;     // test on  A        http://xxx.xxx.xxx.xxx:nnnn/sw0__ON___=0005_5A
              case 6: digitalWrite(pinSw[0],openSw[0]);answer("0_OFF_____");delay(1000);break;    // test off A        http://192.168.0.6:80/sw0__OFF__=0005_5A
              case 7: digitalWrite(pinSw[1],cloSw[1]);answer("1_ON______");delay(1000);break;     // test on  B        http://82.64.32.56:1796/sw1__ON___=0005_5A
              case 8: digitalWrite(pinSw[1],openSw[1]);answer("1_OFF_____");delay(1000);break;    // test off B        adresse/port indifférent crc=5A
              case 9: 
              #ifdef MAIL_SENDER                      
                      if(diags){Serial.print(">>>>>>>>>>> len=");Serial.print(ii);Serial.print(" data=");Serial.println(httpMess+v0);}
                      v0+=21;
                      {httpMess[strlen(httpMess)-2]='\0';             // erase CRC                   
                      uint16_t v1=strstr(httpMess,"==")-httpMess;
                      httpMess[v1]='\0';
                      uint16_t v2=strstr(httpMess+v1+1,"==")-httpMess;
                      httpMess[v2]='\0';
                      #define LMLOC 16
                      char a[LMLOC];a[0]=' ';sprintf(a+1,"%+02.2f",temp/100);a[7]='\0';
                      strcat(a,"°C ");strcat(a,VERSION);
                      if(strlen(a)>=LMLOC){ledblink(BCODESHOWLINE);}
                      a[LMLOC-1]='\0';strcat(httpMess+v2+2,a);
                      answer("mail______");                   
                      mail(httpMess+v0,httpMess+v1+2,httpMess+v2+2);
                      }
              #endif // MAIL_SENDER
              #ifndef MAIL_SENDER
                      if(diags){Serial.println("no mail on this board");}
              #endif // MAIL_SENDER
                      break;                 
              default:break;
          }          
          Serial.println();
        }
        else {Serial.print("reçu ko :");Serial.println(httpMess);}
        //if(strstr(httpMess,"favicon")>0){htmlImg(&cliext,favicon,favLen);}
        cntreq++;
      }                   // une éventuelle connexion a été traitée si controles ko elle est ignorée
      purgeServer(&cliext,diags);
      cliext.stop();
    }   // if(cliext
  }     // if(server!=nullptr){
}       // ordreExt()



#endif // _SERVER_MODE


/* Read analog ----------------------- */

void readAnalog()
{
 cstRec.analVal=analogRead(A0); 
}

/* Output control -------------------- */

void outputCtl()            // cstRec.swCde contient 4 paires de bits (gauche disjoncteur 1=ON, droite résultat règles encodé selon la carte openSW/cloSw)
{
  //if(diags){Serial.print("cstRec.swCde = ");if(cstRec.swCde<16){Serial.print("0");}Serial.print(cstRec.swCde,HEX);Serial.print(" ");}
  for(uint8_t sw=0;sw<NBSW;sw++){
    if(((cstRec.swCde>>(sw*2+1))&0x01)==0x01){                                              // disjoncteur ON
        digitalWrite(pinSw[sw],(cstRec.swCde>>(sw*2))&0x01);                              // value (encodé dans le traitement des regles)
        //if(diags){Serial.print((cstRec.swCde>>(sw*2))&0x01,HEX);Serial.print("x ");}   
        }
    else {digitalWrite(pinSw[sw],openSw[sw]);                                             // disjoncté donc open value
        //if(diags){Serial.print(openSw[sw],HEX);Serial.print("o ");}
        }
  }
  //if(diags){Serial.println();}
}

/* Read temp ------------------------- */
 
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

/* avance timer server ------------------- */
      cstRec.serverTime+=tempPeriod0;             // tempPeriod0 temps écoulé depuis dernier réveil/passage par readTemp
      
      /* si temps maxi atteint depuis dernière cx, forçage cx */
      if(cstRec.serverTime>cstRec.serverPer){     // serverTime temps écoulé depuis dernier talkReq()
                                                  // serverPer max période programmée depuis le serveur entre 2 talkReq
        getTemp();
        cstRec.serverTime=0;
        talkReq(); 
      }
      /* si pas de ko en cours, getTemp() au cas où elle soit changée */
      else if (cstRec.serverPer!=PERSERVKO){  // si dernière cx wifi ko, pas de comm jusqu'à fin de tempo    
        getTemp();
        /* temp (suffisament) changée ? */
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
        /* si pas de changt suffisant rien */
      }

#if POWER_MODE==NO_MODE
    }   // chkTigTemp
#endif // PM==NO_MODE
  }     // talkStep = 0
/* writeConstant avant sleep ou power off */
}

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
      if(((long)ms-(long)debConv)>tconversion){
        temp=ds1820.readDs(WPIN);
        temp*=100;                  // tempPitch 100x
        ds1820.convertDs(WPIN);     // conversion pendant attente prochain accès
        debConv=millis();          
        Serial.print(" temp ");Serial.print(temp/100);Serial.print(" ");Serial.print(debConv);
      }
#endif // PM==NO_MODE
#if POWER_MODE==DS_MODE
      ds1820.convertDs(WPIN);     // conversion pendant deep/sleep ou pendant attente prochain accès
#endif // PM!=DS_MODE

      checkVoltage();
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
  else {Serial.print(" ko ");Serial.println(rcvl);ledblink(BCODESDCARDKO);}
  return rcvl;
}