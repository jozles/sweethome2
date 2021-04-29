
#define _MODE_DEVT    
/* Mode développement */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"
#include "ds18x20.h"
#include "const.h"
#include "utilWifi.h"
#include "util.h"
#include "dynam.h"

#ifdef MAIL_SENDER
#include "EMailSender.h"
EMailSender emailSend("lucieliu66", "eicul666");
EMailSender::EMailMessage message;
#endif // MAIL_SENDER


extern "C" {                  
#include <user_interface.h>                 // pour struct rst_info, system_deep_sleep_set_option(), rtc_mem
}

#if CONSTANT==EEPROMSAVED
#include <EEPROM.h>
#endif //

Ds1820 ds1820;
//extern byte dsmodel;

  char model[LENMODEL];

  unsigned long dateon=millis();            // awake start time
  unsigned long boucleTime=millis();

  const char* ssid;
  const char* password;
#define DEVOLO  
#ifdef DEVOLO
  const char* ssid2= "pinks";
  const char* password2 = "cain ne dormant pas songeait au pied des monts";
  const char* ssid1= "devolo-5d3";
  const char* password1= "JNCJTRONJMGZEEQL";
#endif // DEVOLO
#ifndef DEVOLO
  const char* ssid1= "pinks";
  const char* password1 = "cain ne dormant pas songeait au pied des monts";
  const char* ssid2= "devolo-5d3";
  const char* password2= "JNCJTRONJMGZEEQL";
#endif // DEVOLO

  const char* host = HOSTIPADDR2;         // HOSTIPADDRx est une chaine de car donc de la forme "192.168.0.xxx"
  const int   port = PORTPERISERVER2; 

WiFiClient cli;                           // instance pour serveur externe (utilisé pour dataread/save)

#ifdef  _SERVER_MODE
WiFiClient cliext;                        // instance pour serveur local
WiFiServer* server=nullptr;

  #define RAFSRVTMP 300000                // période réactivation server
  unsigned long  timeservbegin=0;

  #define LHTTPMESS 500
  char   httpMess[LHTTPMESS];             // buffer d'entrée en mode serveur
#endif //  _SERVER

  const char* srvpswd=PERIPASS;
  byte  lsrvpswd=LPWD;

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
  char  buf[3];                       //={0,0,0};

  int   cntreq=0;

#if POWER_MODE != NO_MODE
  ADC_MODE(ADC_VCC);
#endif //

  const char*     chexa="0123456789ABCDEFabcdef\0";
  byte      mask[]={0x00,0x01,0x03,0x07,0x0F};
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

char* tempStr();
int   buildData(const char* nomfonction,const char* data);
int   dataSave();
int   dataRead();
void  dataTransfer(char* data);  
void  readTemp();
void  ordreExt();
void  outputCtl();
void  mail(char* subj,char* dest,char* msg);
void  readAnalog();
void  getTemp();

void tmarker()
{
  pinMode(WPIN,OUTPUT);for(int t=0;t<6;t++){digitalWrite(WPIN,LOW);delayMicroseconds(100);digitalWrite(WPIN,HIGH);delayMicroseconds(100);}
}


void setup() 
{ 
//pinMode(5,OUTPUT);pinMode(2,OUTPUT);digitalWrite(5,1);digitalWrite(2,1);while(1){};
/*#if POWER_MODE!=NO_MODE
WiFi.disconnect();
WiFi.forceSleepBegin();
delay(1);
#endif // PM!=NO_MODE
*/
/* >>>>>> pins Init <<<<<< */

#if POWER_MODE==PO_MODE
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  digitalWrite(PINPOFF,LOW);
  pinMode(PINPOFF,OUTPUT);
#endif // PM==PO_MODE

  Serial.begin(115200);

  checkVoltage();                   // power off au plus vite si tension insuffisante (no serial)

#if POWER_MODE==NO_MODE
  diags=false;
  Serial.println();Serial.print("start setup v");Serial.print(VERSION);Serial.print(" ; une touche pour diags ");
  while((millis()-t_on)<4000){Serial.print(".");delay(500);if(Serial.available()){Serial.read();diags=true;break;}}
  Serial.println();
#endif // PM==NO_MODE  

  pinMode(PINLED,OUTPUT);

#if CARTE==VR || CARTE==VRR || CARTE==VRDEV
  for(uint8_t sw=0;sw<MAXSW;sw++){
    digitalWrite(pinSw[sw],openSw[sw]);
    pinMode(pinSw[sw],OUTPUT);
  }

  pinMode(PINDTA,INPUT_PULLUP);
  pinMode(PINDTB,INPUT_PULLUP);  
  pinMode(PINDTC,INPUT_PULLUP);  
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

  Serial.print(" Slave 8266 ");
#ifdef _MODE_DEVT
  Serial.print("MODE_DEVT ");
#endif // _MODE_DEVT

  Serial.print(VERSION);Serial.print(" power_mode=");Serial.print(POWER_MODE);
  Serial.print(" carte=");Serial.print(CARTE);

  if(diags){
    Serial.println();
    Serial.print("    ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(sw);}Serial.println();
    Serial.print("pin ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(pinSw[sw]);}Serial.println();
    Serial.print("clo ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(cloSw[sw]);}Serial.println();
    Serial.print("ope ");for(int sw=0;sw<MAXSW;sw++){Serial.print(" ");Serial.print(openSw[sw]);}Serial.println();
  }
  


/* >>>>>> gestion ds18x00 <<<<<< */

 byte setds[4]={0,0x7f,0x80,TBITS},readds[8];    // fonction, high alarm, low alarm, config conversion 
 int v=ds1820.setDs(WPIN,setds,readds); // init & read rom
 tconversion=TCONVERSIONB;if(readds[0]==0X10 || TBITS==T12BITS){tconversion=TCONVERSIONS;}
 if(v==1){Serial.print(" DS1820 0x");Serial.print(readds[0],HEX);Serial.print(" Tconv=");Serial.println(tconversion);}
 else {Serial.print(" DS1820 error ");Serial.println(v);}
  
#if POWER_MODE==NO_MODE
  debConv=millis();
  ds1820.convertDs(WPIN); // readTemp ignoré jusqu'à fin de la conversion
  //delay(tconversion);
#endif // PM==NO_MODE
#if POWER_MODE==PO_MODE
  //ds1820.convertDs(WPIN);delay(250);
  debConv=millis();
  ds1820.convertDs(WPIN); // readTemp() attend la fin de la conversion
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
    initConstant();
    }
#endif // PM==DS_MODE

/* >>>>>> gestion variables permanentes <<<<<< */

#if CONSTANT==EEPROMSAVED
  EEPROM.begin(512);
#endif //

#if POWER_MODE!=DS_MODE
/* si erreur sur les variables permanentes (len ou crc faux), initialiser et lancer une conversion */
  //la longueur est chargée depuis le 1er car de l'enregistrement ; si fausse ou crc faux readConstant renvoie 0
  if(!readConstant()){    
    initConstant();
    if(cstRec.cstlen!=LENRTC){Serial.print(" len RTC=");Serial.print(cstRec.cstlen);while(1){};} // blocage param faux, le programme a changé
    }
#endif // PM!=DS_MODE

  Serial.print("CONSTANT=");Serial.print(CONSTANT);Serial.print(" time=");Serial.print(millis()-debTime);Serial.println(" ready !");
  yield();
  printConstant();
delay(20);

#if POWER_MODE==NO_MODE

  cstRec.serverTime=cstRec.serverPer+1;
  forceTrigTemp();    // force connexion server 

  memdetinit();pulsesinit();
  Serial.print(" ssid=");Serial.print(ssid1);Serial.print(" - ");Serial.println(ssid2);
  yield();
  Serial.println(">>>> fin setup\n");
  
#ifdef  _SERVER_MODE
  clkFastStep=1;cstRec.talkStep=1 ; // forçage com pour acquisition port perif server
#endif //  def_SERVER_MODE

  }    // fin setup NO_MODE
  void loop(){  //=== NO_MODE =================================      

  // la réception de commande est prioritaire sur tout et (si une commande valide est reçue) toute communication
  // en cours est avortée pour traiter la commande reçue.
  //
  // automate de séquencement : horloge à 50mS et à 500mS ;
  //    10 slots dans chaque ; un traitement peut exister sur plusieurs slots
  //
  // si un appel au serveur est en cours (cstRec.talkStep != 0), l'automate talkServer est actif
  // sinon la boucle d'attente tourne : 
  //                                    (toutes les 50mS)
  //                                      - réception d'un ordre extérieur
  //                                      - exécution des actions
  //                                      - debounce détecteurs physiques
  //                                      - polling  détecteurs physiques
  //                                      - test de l'heure de blink
  //                                     (toutes les 100mS)
  //                                      - clock pulses  
  //                                     (toutes les 500mS)
  //                                      - test de l'heure de mesure de température (ou autre)
  //                                      - test de l'heure d'appel du serveur  
  //
  // (changer cstRec.talkStep déclenche un transfert au serveur (cx wifi/dataRead/Save)
  // Si l'appel n'aboutit pas (pas de cx wifi, erreurs de com, rejet par le serveur-plus de place)
  // le délai d'appel au serveur est doublé à concurence de 7200sec entre les appels.
  //
  // la période est allongée par les communications avec le serveur (appel ou réception d'ordre)
   

  #ifdef  _SERVER_MODE
  
      if(millis()>(clkTime+PERFASTCLK)){        // période mini 5mS/step
        switch(clkFastStep++){

/* En 1 toujours talkstep pour le forçage de communication d'acquisition du port au reset 
 * En 2 ordreExt() pour assurer que le traitement des commandes reçues et la collecte des données soit effectués pour le prochain dataSave
 * (ordreExt() positionne cstRec.talkStep!=0)
 * clkFastStep et cstRec.talkStep == 1 
*/
          case 1:   //timeOvfSet(1);
                    if(cstRec.talkStep!=0){talkServer();}//timeOvfCtl(1);   // période talkStep mini 50mS
                    break;
          case 2:   //ledblink(-1);break;                                 // période ledBlink mini env 10mS
          case 3:   //timeOvfSet(3);
                    ordreExt();//timeOvfCtl(3);                         // période ordreExt mini 50mS
                    break;
          case 4:   //ledblink(-1);break;
          case 5:   //timeOvfSet(5);
                    actions();//timeOvfCtl(5);                          // période actions mini 50mS
                    break;
          case 6:   outputCtl();break;                                  // période outputCtl mini 50mS
          case 7:   //ledblink(-1);
                    break;
          case 8:   swDebounce();break;                                 // doit être avant polDx
          case 9:   //timeOvfSet(9);
                    polAllDet();//timeOvfCtl(9);
                    break;      // polDx doit être après swDebounce                            
          case 10:  ledblink(-1);                                       // 1 flash
                    //timeOvfSet(10);
                    clkFastStep=0;              // période mini env 50mS/step 
                    switch(clkSlowStep++){
                      case 1:   break;
                      case 2:   pulseClkisr();break;                    // période pulseClkisr mini env 100mS
                      case 3:   break;
                      case 4:   pulseClkisr();break;
                      case 5:   break;
                      case 6:   pulseClkisr();break;
                      case 7:   readAnalog();break;
                      case 8:   pulseClkisr();break;
                      case 9:   readTemp();break;
                      case 10:  pulseClkisr();
                                clkSlowStep=0;
                                break;
                    }
                    //timeOvfCtl(10);
                    break;
          default:  break;
        }
        clkTime=millis();
      }
  #endif // def_SERVER_MODE  

#endif // PM==NO_MODE


#if POWER_MODE!=NO_MODE

  readTemp();

  Serial.print("durée (no comm)=");Serial.print(millis());Serial.print(" - ");
  Serial.print(dateon);Serial.print(" = ");Serial.println(millis()-dateon);
  
  while(cstRec.talkStep!=0){
    Serial.print("   talkStep=");Serial.println(cstRec.talkStep);
    yield();talkServer();}

  /* sauvegarde variables permanentes avant sleep ou power off */
  writeConstant();

  Serial.print("durée ");Serial.print(millis());Serial.print(" - ");
  Serial.print(dateon);Serial.print(" = ");Serial.print(millis()-dateon);
  Serial.print("   talkStep=");Serial.print(cstRec.talkStep);
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

talkServer() automate de fragmentation temporelle d'envoi de dataRead/Save et gestion réponses (step env 50mS mini)

buildReadSave() construction et envoi message read/save

buildData()     construction message fonction dataRead ou dataSave

dataRead()

dataSave()

ordreExt() test présence/réception et chargement message reçu en mode serveur (declenche talkServer pour envoyer le résultat des commandes - position switchs)

talkClient() réponse à un message reçu en mode serveur

wifiConnexion()

readAnalog() (pour NO_MODE seul : les autres modes utilisent l'ADC pour lire l'alim)

readTemp() gestion communications cycliques (déclenche talkServer)

*/

void infos(const char* mess,const char* data,uint16_t val)           // Serial.print de fonctionnement du périphérique
{
/*        
        char ff[LENNOM+1];memcpy(ff,data,LENNOM);ff[LENNOM]='\0';
        char np[3];strncpy(np,data+MPOSNUMPER,2);np[2]='\0';               
        Serial.print(mess);Serial.print("(");Serial.print(ff);Serial.print(") numper=");Serial.print(np);
        Serial.print(";");Serial.print(val);
        Serial.print(" periMess=");Serial.println(periMess);  
*/
}

void fServer(uint8_t fwaited)          // réception du message réponse du serveur pour DataRead/Save;
                                       // contrôles et transfert 
                                       // retour periMess  
{      
        periMess=getHttpResponse(&cli,bufServer,LBUFSERVER,&fonction,diags);
        if(diags){Serial.print("fserver (OK=");Serial.print(MESSOK);Serial.print(") periMess=");Serial.print(periMess);Serial.print(" fwaited=");Serial.print(fwaited);Serial.print(" recu=");Serial.print(fonction);} 
        infos("gHResp",bufServer,0);
        if(periMess==MESSOK){
          
          if(fonction==fwaited){dataTransfer(bufServer);}
          else {periMess=MESSFON;}
        }
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
if(diags){Serial.print(" dataTransfer()");}                              
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

            cstRec.portServer=(uint16_t)convStrToNum(data+MPOSPORTSRV,&sizeRead);    // port server
            printConstant();
            if(server==nullptr){server=new WiFiServer(cstRec.portServer);}
        }
        if(periMess!=MESSOK){
          memcpy(cstRec.numPeriph,"00",2);cstRec.IpLocal=IPAddress(0,0,0,0);
        }
        infos("dataTransfer",data,0);
}

void ssTS()                   // connecté au wifi
{                             // si le numéro de périphérique est 00 ---> récup (data_read), ctle réponse et maj params
                              // sortie 5  data_read ok
                              // sortie 7  data_save ok
                              // sortie 9  data_read ko
                              // sortie 99 data_save ko
    int v=0;
    if(memcmp(cstRec.numPeriph,"00",2)==0){         // numPeriph ==0 -> data_read
      v=dataRead();infos("  dataRead","",v);
      if(v==MESSOK){cstRec.talkStep=5;}
      else {cstRec.talkStep=9;}                     // pb com -> recommencer au prochain timing
    }
    else {v=dataSave();infos("  dataSave","",v);    // numPeriph !=0 -> data_save
      if(v==MESSOK){cstRec.talkStep=7;}
      else {cstRec.talkStep=99;}
    }               
}

void talkServer()   // si numPeriph est à 0, dataRead pour se faire reconnaitre ; 
                    // si ça fonctionne réponse numPeriph!=0 ; dataSave 
{

#ifdef  _SERVER_MODE
  dateon=millis();
#endif //  def_SERVER_MODE

switch(cstRec.talkStep){
  case 1:
      ssid=ssid1;password=password1;
      if(wifiConnexion(ssid,password)){ssTS();}
      else {cstRec.talkStep=2;}
      break;
      
  case 2:
      ssid=ssid2;password=password2; // tentative sur ssid bis
      if(wifiConnexion(ssid,password)){ssTS();} 
      else {cstRec.talkStep=98;}
      break;

  case 3:    
      break;
      
  case 4:         
      break;
        
  case 5:          // gestion réponse au dataRead

    fServer(fset_______);           // récupération adr mac, numPériph, tempPer et tempPitch dans bufServer (ctle CRC & adr mac)
                                    // le num de périph est mis à 0 si la com ne s'est pas bien passée
    cstRec.talkStep=STEPDATASAVE;   // si le numéro de périphérique n'est pas 00 ---> ok (datasave), ctle réponse et maj params
    writeConstant();
    break;
      
  case STEPDATASAVE:          // (6) si réponse au dataread ok -> datasave
                              // sinon recommencer au prochain timing
                              
      if(memcmp(cstRec.numPeriph,"00",2)==0){cstRec.talkStep=9;} 
      else {ssTS();}          // numPeriph !=0 donc data_save sortie 7 ok ou 99 ko 
      break;

  case 7:         // gestion réponse au dataSave
                  // si la réponse est ok -> terminer
                  // sinon recommencer au prochain timing

       fServer(fack_______);
       
       // le num de périph a été mis à 0 si la com ne s'est pas bien passée
       cstRec.talkStep=9;
#ifdef  _SERVER_MODE    // activation server après chaque fin d'échange data_save / ack
  if(server!=nullptr){
    server->begin(cstRec.portServer);       // une seule fois après acquisition ans/set 
    Serial.print("server.begin(");Serial.print((int)cstRec.portServer);Serial.print(") durée=");Serial.println(millis()-timeservbegin);
    timeservbegin=millis();
  }
#endif //  def_SERVER_MODE

       break; 
                   // terminé ; si tout s'est bien passé les 2 côtés sont à jour 
                   // sinon numpériph est à 00 et l'adresse IP aussi

  case 9:
       cstRec.talkStep=0;
       break;


  case 98:      // pas réussi à connecter au WiFi ; tempo longue
#if POWER_MODE!=NO_MODE
        cstRec.serverPer=PERSERVKO;     // pas de modif en NO_MODE pour ne pas risquer le déclenchement du WD
#endif //
        
        cstRec.serverTime=0;

  case 99:      // mauvaise réponse du serveur ou wifi ko ; raz numPeriph
        memcpy(cstRec.numPeriph,"00",2);
        cstRec.talkStep=0;
        break;
        
  default: break;
  }
}

#ifdef _SERVER_MODE

// **************** mode serveur

void answer(const char* what)
{
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
  if(server!=nullptr){
    uint16_t hm=0,nl=0;
    memset(httpMess,0x00,LHTTPMESS);
  
    cliext = server->available();

    if (cliext) {

      unsigned long trx=0;
      char c;
      Serial.print("\nCliext ");
      while (cliext.connected()) {
#define TO_ORDREXT 2
        if(trx==0){trx=millis();}
        if((millis()-(unsigned long)trx)>TO_ORDREXT){break;}
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
      }
      // format message "GET /FFFFFFFFFF=nnnn....CC" FFFFFFFFFF instruction (ETAT___, SET____ etc)
      //                                             nnnn nombre de car décimal zéros à gauche 
      //                                             .... éventuels arguments de la fonction
      //                                             CC crc

      //Serial.println();
      if(diags){Serial.print(" reçu(");Serial.print(hm);Serial.print(")  =");Serial.print(strlen(httpMess));Serial.print(" httpMess=");Serial.println(httpMess);}
     
      int v0=-1;
      char* vx=strstr(httpMess,"GET /");
      if(vx>=0){v0=vx-httpMess;}
      if(v0>=0){                            // si commande GET trouvée contrôles et décodage nom fonction 
        int jj=4,ii=convStrToNum(httpMess+v0+5+10+1,&jj);   // recup eventuelle longueur
        httpMess[v0+5+10+1+ii+2]=0x00;      // place une fin ; si long invalide check sera invalide

        if(checkHttpData(&httpMess[v0+5],&fonction)==MESSOK){
          Serial.print("reçu message fonction=");Serial.println(fonction);
          switch(fonction){
              case 0: dataTransfer(&httpMess[v0+5]);actions();outputCtl();  // récup data,compute rules,exec résultat 
                      answer("data_save_");break;                       // set ---> réponse message data_save_ complet
              case 1: answer("ack_______");break;                       // ack ne devrait pas se produire (page html seulement)
              case 2: answer("etat______");cstRec.talkStep=1;break;     // etat -> dataread/save   http://192.168.0.6:80/etat______=0006xxx
              case 3: break;                                            // sleep (future use)
              case 4: break;                                            // reset (future use)
              case 5: digitalWrite(pinSw[0],cloSw[0]);answer("0_ON______");delay(1000);break;     // test on  A        http://192.168.0.6:80/sw0__ON___=0005_5A
              case 6: digitalWrite(pinSw[0],openSw[0]);answer("0_OFF_____");delay(1000);break;    // test off A        http://192.168.0.6:80/sw0__OFF__=0005_5A
              case 7: digitalWrite(pinSw[1],cloSw[1]);answer("1_ON______");delay(1000);break;     // test on  B        http://192.168.0.6:80/sw1__ON___=0005_5A
              case 8: digitalWrite(pinSw[1],openSw[1]);answer("1_OFF_____");delay(1000);break;    // test off B        http://192.168.0.6:80/sw0__OFF__=0005_5A
              case 9: if(diags){Serial.print(">>>>>>>>>>> len=");Serial.print(ii);Serial.print(" data=");Serial.println(httpMess+v0);}
                      v0+=21;
                      {httpMess[strlen(httpMess)-2]='\0';             // erase CRC                   
                      uint16_t v1=strstr(httpMess,"==")-httpMess;
                      httpMess[v1]='\0';//Serial.print("<<<<");Serial.println(httpMess+v0);
                      uint16_t v2=strstr(httpMess+v1+1,"==")-httpMess;
                      httpMess[v2]='\0';//Serial.print("<<<<");Serial.println(httpMess+v1+2);
                      char a[10];a[0]=' ';sprintf(a+1,"%+02.2f",temp/100);a[7]='\0';
                      strcat(a,"°C ");strcat(httpMess+v2+2,a);
                      //Serial.print("<<<<");Serial.println(httpMess+v2+2);
                      answer("mail______");                    
                      mail(httpMess+v0,httpMess+v1+2,httpMess+v2+2);
                      }break;                 
              default:break;
          }          
          Serial.println();
          cstRec.talkStep=6;      // après maj des sw et collecte des données -> dataSave       
          //cstRec.serverTime=0;    // force connexion 
        }
        if(strstr(httpMess,"favicon")>0){htmlImg(&cli,favicon,favLen);}
        cntreq++;
      }                     // une éventuelle connexion a été traitée
                          // si controles ko elle est ignorée
      purgeServer(&cliext,diags);
      cliext.stop();
    }   // if(cliext){
    /*    réactivation server à chaque test sans connexion... semble ne plus recevoir les connexions après un certain nombre (?) 
    else {
      if(((millis()-timeservbegin)>RAFSRVTMP )|| timeservbegin==0 ){
        server->begin(cstRec.portServer);timeservbegin=millis();                    // réactivation périodique
        Serial.print(millis());Serial.print(" ");Serial.println("server.begin");
      }       
    }*/   // !client
  }     // if(server!=nullptr){
}       // ordreExt()

void mail(char* subj,char* dest,char* msg)
{
#ifdef MAIL_SENDER

unsigned long beg=millis();

    Serial.println("---mail---");
    
    wifiConnexion(ssid,password);

    char s[64]={"sh speaking "};strcat(s,subj);
    message.subject = s;
    message.message = msg ;

    EMailSender::Response resp = emailSend.send(dest, message);

Serial.print(">>> email millis()=");Serial.println(millis()-beg);
#endif // MAIL_SENDER
}

/*
void ordreExt0()          // version avec string
{
  cliext = server.available();

  if (cliext) {
    char c;
    Serial.println("\nCliext");
    String input = "";                    // buffer ligne
    headerHttp = "";                      // buffer en-tête
    while (cliext.connected()) {
      if (cliext.available()) {
        c = cliext.read();
        headerHttp+=c;                    //remplissage en-tête
        //Serial.write(c);
        if (c == '\n') {                  // LF fin de ligne
            if (input.length() == 0) {    // si ligne vide fin de requête
              //Serial.println("\n 2 fois LF fin de la requête HTTP");
              break;                      // sortie boucle while
            }
            else {input = "";}            // si 1 LF vidage buffer ligne pour recevoir la ligne suivante
        }
        else if(c != '\r'){input+=c;}   // remplissage ligne
      }
    }
    // headerHttp contient la totalité de l'en-tête 
    //
    // format message "GET /FFFFFFFFFF=nnnn....CC" FFFFFFFFFF instruction (ETAT___, SET____ etc)
    //                                             nnnn nombre de car décimal zéros à gauche 
    //                                             .... éventuels arguments de la fonction
    //                                             CC crc
    int v0=headerHttp.indexOf("GET /");
    if(diags){Serial.print(" reçu=");Serial.print(strlen(httpMess));Serial.print(" httpMess=");Serial.println(httpMess);}
    if(v0>=0){                            // si commande GET trouvée contrôles et décodage nom fonction 
      int jj=4,ii=convStrToNum(&headerHttp[0]+v0+5+10+1,&jj);   // recup eventuelle longueur
      headerHttp[v0+5+10+1+ii+2]=0x00;    // place une fin ; si long invalide check sera invalide
      //Serial.print("len=");Serial.print(ii);Serial.print(" ");Serial.println(headerHttp+v0);
      if(checkHttpData(&headerHttp[v0+5],&fonction)==MESSOK){
        if(diags){Serial.print("reçu message fonction=");Serial.println(fonction);}
        switch(fonction){
            case 0:dataTransfer(&headerHttp[v0+5]);break;  // set
            case 1:break;                             // ack ne devrait pas se produire (page html seulement)
            case 2:cstRec.talkStep=1;break;           // etat -> dataread/save   http://192.168.0.6:80/etat______=0006AB8B
            case 3:break;                             // sleep (future use)
            case 4:break;                             // reset (future use)
            case 5: digitalWrite(pinSw[0],cloSw[0]);break;    // test on  A        http://192.168.0.6:80/sw0__ON___=0006xxxx
            case 6: digitalWrite(pinSw[0],openSw[0]);break;   // test off A        http://192.168.0.6:80/sw0__OFF__=0006xxxx
            case 7: digitalWrite(pinSw[1],cloSw[1]);break;    // test on  B        http://192.168.0.6:80/sw1__ON___=0006xxxx
            case 8: digitalWrite(pinSw[1],openSw[1]);break;   // test off B        http://192.168.0.6:80/sw0__OFF__=0006xxxx
            case 9: mail("s_h test","jozles@hotmail.fr","message de test sweet_home");break;                 
            
            default:break;
        }
        char etat[]="done______=0006AB8B\0";
        talkClient(etat);
        Serial.println();
      }
      cntreq++;
      cliext.stop();
      headerHttp="";
    }                     // une éventuelle connexion a été traitée
                          // si controles ko elle est ignorée
    purgeServer(&cliext);
  }
}
*/
void talkClient(char* data) // réponse à une requête
{
  
            // en-tête réponse HTTP 
            cliext.println("HTTP/1.1 200 OK");
            cliext.println("Content-type:text/html");
            cliext.println("Connection: close\n");
            // page Web 
            cliext.println("<!DOCTYPE html><html>");
            //cliext.println("<head></head>");
            
            cliext.print("<body>");
            cliext.print(data);
            cliext.println("</body></html>");
}


#endif // def_SERVER_MODE

//***************** dataRead/dataSave

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
  if(!buildMess("peri_pass_",srvpswd,"?",diags)==MESSOK){
    if(diags){Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(srvpswd);return MESSDEC;};}

  buildData(nomfonction,data);

  infos("buildReadSave",bufServer,port);
  return messToServer(&cli,host,port,bufServer); 
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
  if(cstRec.talkStep == 0){     // !=0 ne peut se produire qu'en NO_MODE 
                                // (les autres modes terminent talkServer avec talkStep=0)

#if POWER_MODE==DS_MODE
uint16_t tempPeriod0=cstRec.tempPer;  // (sec) durée depuis dernier check température
#endif // PM==DS_MODE
#if POWER_MODE==PO_MODE
uint16_t tempPeriod0=PERTEMP;  // (sec) durée depuis dernier check température (fixe : resistance 5111)
#endif // PM==PO_MODE
#if POWER_MODE==NO_MODE
    if(chkTrigTemp()){
      uint16_t tempPeriod0=(millis()-tempTime)/1000;   // (sec) durée depuis dernier check température
      trigTemp();
#endif // PM==NO_MODE

/* avance timer server ------------------- */
      cstRec.serverTime+=tempPeriod0;
      if(cstRec.serverTime>cstRec.serverPer){
        getTemp();
        cstRec.serverTime=0;
        cstRec.talkStep=1; 
      }
      else if (cstRec.serverPer!=PERSERVKO){  // si dernière cx wifi ko, pas de comm jusqu'à fin de tempo    
/* temp (suffisament) changée ? */
        getTemp();
        if( temp>(cstRec.oldtemp+cstRec.tempPitch) || temp<(cstRec.oldtemp-cstRec.tempPitch)){
          cstRec.oldtemp=(int16_t)temp;
          cstRec.talkStep=1;     // temp changée -> talkServer
          cstRec.serverTime=0;
        }
      }

#if POWER_MODE==NO_MODE
    }   // chkTigTemp
#endif // PM==NO_MODE
  }     // talkStep = 0
}

void getTemp()
{  
#if POWER_MODE!=DS_MODE
      unsigned long ms=millis();           // attente éventuelle de la fin de la conversion initiée à l'allumage
      if(diags){
        Serial.print(" Tconv=");Serial.print(tconversion);Serial.print(" delay=");Serial.println((long)ms-(long)debConv);}
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
        Serial.print(" temp ");Serial.print(temp/100);
        ds1820.convertDs(WPIN);     // conversion pendant attente prochain accès
        debConv=millis();          
      }
#endif // PM==NO_MODE
#if POWER_MODE==DS_MODE
      ds1820.convertDs(WPIN);     // conversion pendant deep/sleep ou pendant attente prochain accès
#endif // PM!=DS_MODE

      checkVoltage();
}
