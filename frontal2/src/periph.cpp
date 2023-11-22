#include <Arduino.h>
#include <SdFat.h>
#include "const.h"
#include <Wire.h>
#include <shconst2.h>
#include <shutil2.h>
#include <shmess2.h>
#include "utilether.h"
#include "periph.h"
#include "utilhtml.h"


/* ---------- config ---------- */

File32 fconfig;     // fichier config

extern char configRec[CONFIGRECLEN];
  
extern byte*      mac;
extern byte*      localIp;
extern uint16_t*  perifPort;
extern uint16_t*  browserPort;
extern uint16_t*  remotePort;
extern uint16_t*  serverUdpPort;
extern char*      serverName;

extern char*      peripass;
extern char*      ssid;   
extern char*      passssid;
extern uint8_t*   ssid1;
extern uint8_t*   ssid2;
extern uint8_t*   concMac;
extern byte*      concIp;               
extern uint16_t*  concPort; 
extern uint8_t*   concRx;
extern uint16_t*  concChannel;
extern uint16_t*  concRfSpeed;      
extern uint8_t*   concNb;            

extern uint8_t*   concPeriParams; 
extern float*     thFactor;
float*            thFactor0;        // voir les commentaires de configRecordUpdate()
float             thF;
extern float*     thOffset;
float*            thOffset0;        // voir les commentaires de configRecordUpdate()
float             thO;
extern float*     vFactor;
float*            vFactor0;         // voir les commentaires de configRecordUpdate()
float             vF;
extern float*     vOffset;
float*            vOffset0;         // voir les commentaires de configRecordUpdate()
float             vO;
extern byte*      periRxAddr;

extern char*      usrnames;  
extern char*      usrpass;     
extern unsigned long* usrtime;
extern unsigned long* usrpretime;
extern uint16_t* toPassword;
extern unsigned long* maxCxWt;    
extern unsigned long* maxCxWu;

extern char*      mailFromAddr; 
extern char*      mailPass;     
extern char*      mailToAddr1;  
extern char*      mailToAddr2;  
extern uint16_t*  periMail1;    
extern uint16_t*  periMail2;
       
extern byte*      configBegOfRecord;
extern byte*      configEndOfRecord;

/* ---------- périphériques ---------- */

File32 fperi;       // fichiers perif

extern char       periRec[PERIRECLEN];          // 1er buffer de l'enregistrement de périphérique
extern char       periCache[PERIRECLEN*NBPERIF];   // cache des périphériques
extern bool       periCacheStatus[NBPERIF];     // indicateur de validité du cache d'un périph
  
extern int        periCur;                      // Numéro du périphérique courant

extern uint16_t*  periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern int32_t*   periPerRefr;                  // ptr ds buffer : période datasave minimale
extern uint16_t*  periPerTemp;                  // ptr ds buffer : période de lecture tempèrature
extern int16_t*   periPitch_;                   // ptr ds buffer : variation minimale de température pour datasave
extern int16_t*   periLastVal_;                 // ptr ds buffer : dernière valeur de température  
extern int16_t*   periAlim_;                    // ptr ds buffer : dernière tension d'alimentation
extern char*      periLastDateIn;               // ptr ds buffer : date/heure de dernière réception
extern char*      periLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
extern char*      periLastDateErr;              // ptr ds buffer : date/heure de derniere anomalie com
extern int8_t*    periErr;                      // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
extern char*      periNamer;                    // ptr ds buffer : description périphérique
extern char*      periVers;                     // ptr ds buffer : version logiciel du périphérique
extern char*      periModel;                    // ptr ds buffer : model du périphérique
extern byte*      periMacr;                     // ptr ds buffer : mac address 
extern byte*      periIpAddr;                   // ptr ds buffer : Ip address
extern uint16_t*  periPort;                     // ptr ds buffer : port periph server
extern byte*      periSwNb;                     // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*      periSwCde;                    // ptr ds buffer : état/cde des switchs
extern byte*      periInput;                    // ptr ds buffer : Mode fonctionnement inters (1 par switch)           
extern uint32_t*  periSwPulseOne;               // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t*  periSwPulseTwo;               // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t*  periSwPulseCurrOne;           // ptr ds buffer : temps courant pulses ON
extern uint32_t*  periSwPulseCurrTwo;           // ptr ds buffer : temps courant pulses OFF
extern byte*      periSwPulseCtl;               // ptr ds buffer : mode pulses
extern byte*      periSwPulseSta;               // ptr ds buffer : état clock pulses
extern uint8_t*   periSwSta;                    // ptr ds buffer : dispo
extern bool*      periProg;                     // ptr ds buffer : flag "programmable" (périphériques serveurs)
extern byte*      periDetNb;                    // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*      periDetVal;                   // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern int16_t*   periThOffset_;                // ptr ds buffer : offset correctif sur mesure température
extern int16_t*   periThmin_;                   // ptr ds buffer : mini last 24h
extern int16_t*   periThmax_;                   // ptr ds buffer : maxi last 24h
extern int16_t*   periVmin_;                    // ptr ds buffer : alarme mini volts
extern int16_t*   periVmax_;                    // ptr ds buffer : alarme maxi volts
extern byte*      periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*      periProtocol;                 // ptr ds buffer : protocole ('T'CP/'U'DP)
extern uint16_t*  periAnal;                     // ptr ds buffer : analog value
extern uint16_t*  periAnalLow;                  // ptr ds buffer : low analog value 
extern uint16_t*  periAnalHigh;                 // ptr ds buffer : high analog value 
extern uint16_t*  periAnalOffset1;              // ptr ds buffer : offset on adc value
extern float*     periAnalFactor;               // ptr ds buffer : factor to float for analog value
extern float*     periAnalOffset2;              // ptr ds buffer : offset on float value
extern uint8_t*   periAnalCb;                   // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*   periAnalDestDet;              // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*   periAnalRefDet;               // ptr ds buffer : 5 x n° détect serveur pour op logique (0xff si rien)
extern int8_t*    periAnalMemo;                 // ptr ds buffer : 5 x n° mémo dans table mémos
extern uint8_t*   periDigitCb;                  // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*   periDigitDestDet;             // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*   periDigitRefDet;              // ptr ds buffer : 4 x n° détect serveur pour op logique (0xff si rien)
extern int8_t*    periDigitMemo;                // ptr ds buffer : 5 x n° mémo dans table mémos
extern byte*      periSsidNb;                     // ptr ds buffer : n° dernier ssid utilisé
      
extern byte*      periBegOfRecord;
extern byte*      periEndOfRecord;

extern int8_t     periMess;                    // code diag réception message (voir MESSxxx shconst.h)
extern byte       periMacBuf[6]; 

//extern byte       lastIpAddr[4];

extern char inptyps[];                        // libellés types sources regles switchs
extern char inptypd[];                        // libellés types destinations regles switchs
extern char inpact[];                         // libellés actions

/* ---------- remotes ----------  */

File32 fremote;     // fichier remotes

extern struct SwRemote remoteT[MAXREMLI];
extern char*  remoteTA;
extern long   remoteTlen;
extern struct Remote remoteN[NBREMOTE];
extern char*  remoteNA;
extern long   remoteNlen;

/* ---------- Timers ---------- */

File32 ftimers;     // fichier timers

extern struct Timers timersN[NBTIMERS];
extern char*  timersNA;
extern long   timersNlen;

/* ---------- Thermos ---------- */

File32 fthermos;    // fichier thermos

extern struct Thermo thermos[NBTHERMOS];
extern char*  thermosA;
extern long   thermoslen;

/* ---------- détecteurs serveur ---------- */

File32 fmemdet;     // fichier détecteurs serveur

extern char       libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t   sourceDetServ[];       // actionneurs (ssnnnnnn ss type 00, 01 perif, 10 remote, 11 timer / nnnnnn n°)
extern uint8_t    memDetServ[];          // image mémoire NBDSRV détecteurs

extern uint8_t    mDSmaskbit[];

/* ---------- Memos ---------- */

File32 fmemos;      // fichier memos
extern char   memosTable[LMEMO*NBMEMOS];

uint8_t channelTable[]={CHANNEL0,CHANNEL1,CHANNEL2,CHANNEL3};   // canal vs N° conc 
const char  concMacTable[] = {CC_ADDRX};
uint16_t portTable[MAXCONC] = {CC_UDP0,CC_UDP1,CC_UDP2,CC_UDP3};
uint16_t speedTable[MAXCONC]= {CC_SPEED,CC_SPEED,CC_SPEED,CC_SPEED};

#define DEFCONC 0

extern uint16_t perrefr;
extern char strdate[33];
extern char temp[3],temp0[3],humid[3];

extern  char* fonctions;
extern  int   nbfonct,faccueil,fdatasave,fdatana,fperiSwVal,fperiDetSs,fdone,fpericur,fperipass,fpassword,fusername,fuserref,fperitst,ffavicon;

/* ---------- configuration ---------- */

void factoryResetConfig()
{
  memset(serverName,0x00,LNSERV);
  uint8_t srvnl=strlen(DEFNOMSERV);if(srvnl>LNSERV){srvnl=LNSERV;}
  memcpy(serverName,DEFNOMSERV,srvnl);
  memcpy(mac,DEFMACADDR,MACADDRLENGTH);
  *perifPort=DEFSERVERPORT;
  *browserPort=DEFSERVERPORT+1;
  *remotePort=DEFSERVERPORT+2;
  *serverUdpPort=DEFSERVERPORT+3;
  memset(usrnames,0x00,LENUSRNAME);
  memcpy(usrnames,"admin",5);
  memset(usrpass,0x00,LENUSRPASS);
  memcpy(usrpass,"admin",5);
  memset(peripass,0x00,LPWD);
  memcpy(peripass,"sweeth",6);
  *toPassword=TO_PASSWORD;
  *maxCxWt=MAXCXWT;
  *maxCxWu=MAXCXWU;

  memcpy(concMac,concMacTable,MACADDRLENGTH*MAXCONC);
  memset(concIp,0x00,4*MAXCONC);
  for(uint8_t i=0;i<MAXCONC;i++){
    memcpy(concRx+i*RADIO_ADDR_LENGTH,CC_NRF_ADDR,RADIO_ADDR_LENGTH); //(MAXCONC-1)*(RADIO_ADDR_LENGTH-1));
    *(concRx+i*RADIO_ADDR_LENGTH+RADIO_ADDR_LENGTH-1)=PMFNCVAL+i;
    concPort[i]=portTable[i];
    concChannel[i]=channelTable[i];
    concRfSpeed[i]=speedTable[i];
  }
  configPrint();
  Serial.println("\nFactoryReset (lâcher HALT si OK)");
  while(digitalRead(STOPREQ)==LOW){blink(1);delay(500);}
  configSave();
}

void configInitVar()
{
memset(mac,0x00,6);
memset(localIp,0x00,4); 
*perifPort = 0;
*browserPort = 0;
*remotePort = 0;
memset(serverName,0x00,LNSERV);
memset(peripass,0x00,LPWD+1);
memset(ssid,0x00,MAXSSID*(LENSSID+1));
//memcpy(ssid,SSID1,strlen(SSID1));memcpy(ssid+LENSSID+1,SSID2,strlen(SSID2));   
memset(passssid,0x00,MAXSSID*(LPWSSID+1));
//memcpy(passssid,PWDSSID1,strlen(PWDSSID1));memcpy(passssid+LPWSSID+1,PWDSSID2,strlen(PWDSSID2));
memset(concMac,0x00,MAXCONC*MACADDRLENGTH);
memset(concRx,' ',MAXCONC*RADIO_ADDR_LENGTH);
memset(concChannel,0x00,MAXCONC*sizeof(uint8_t));
memset(concRfSpeed,0x00,MAXCONC*sizeof(uint8_t));
memset(concIp,0x00,MAXCONC*4);
memset(concPort,0x00,MAXCONC*sizeof(uint16_t));
*concNb=0;
*concPeriParams=0; 
*thFactor=0;
*thOffset=0;
*vFactor=0;
*vOffset=0;
memset(periRxAddr,0x00,RADIO_ADDR_LENGTH);

memset(usrnames,0x00,NBUSR*LENUSRNAME);memset(usrpass,0x00,NBUSR*LENUSRPASS);
//memcpy(usrnames,"admin",5);memcpy(usrpass,"17515A\0\0",8);
memset(usrtime,0x00,NBUSR*sizeof(long));
memset(usrpretime,0x00,NBUSR*sizeof(long));
*toPassword=0;
*maxCxWt=0;
*maxCxWu=0;
}

/* le compilateur du stm32 semble nécessiter des variables float alignées
   les champs float sont recopiés dans des variables du tas lors du configLoad
   et dans l'enregistrement lors du configSave 
    
   C'est la modif la plus légère imaginée ppour résoudre le pb :
   4 champs pour la config et 2 pour les périphériques 
   
   Ultérieurement, ils pourront être transformés en uint32_t 
   si ce type n'a pas de contraintes d'alignement.
   Il est possible aussi d'aligner la totazlité de l'enregistrement */

void configRecordUpdate()               // local fields (float) -> record
{
  memcpy(thFactor0,thFactor,sizeof(float));
  memcpy(thOffset0,thOffset,sizeof(float));
  memcpy(vFactor0,vFactor,sizeof(float));
  memcpy(vOffset0,vOffset,sizeof(float));
}

void configLocalUpdate()                // record fields (float) -> local
{
  memcpy(thFactor,thFactor0,sizeof(float));
  memcpy(thOffset,thOffset0,sizeof(float));
  memcpy(vFactor,vFactor0,sizeof(float));
  memcpy(vOffset,vOffset0,sizeof(float));
}

void configInit()
{
byte* temp=(byte*)configRec;

  configBegOfRecord=(byte*)temp;         // doit être le premier !!!
 
  mac=(byte*)temp;
  temp+=6;
  localIp=(byte*)temp;
  temp+=4;
  perifPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  remotePort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  serverName=(char*)temp;
  temp+=LNSERV;
  serverUdpPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  temp+=16;                         // dispo  
  peripass=(char*)temp;
  temp+=(LPWD+1);
  ssid=(char*)temp;
  temp+=(MAXSSID*(LENSSID+1));
  passssid=(char*)temp;
  temp+=(MAXSSID*(LPWSSID+1));
  ssid1=(uint8_t*)temp;          
  temp+=sizeof(uint8_t);
  ssid2=(uint8_t*)temp;          
  temp+=sizeof(uint8_t);
  temp+=2;                           // dispo
  concMac=(uint8_t*)temp;
  temp+=(MAXCONC*MACADDRLENGTH);
  concChannel=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concRfSpeed=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concIp=(byte*)temp;
  temp+=(MAXCONC*sizeof(byte*)*4);
  concPort=(uint16_t*)temp;
  temp+=(MAXCONC*sizeof(uint16_t));
  concNb=(uint8_t*)temp;
  temp+=sizeof(uint8_t);
  concRx=(uint8_t*)temp;
  temp+=RADIO_ADDR_LENGTH*MAXCONC;
  concPeriParams=(uint8_t*)temp;
  temp+=sizeof(uint8_t*);
  thFactor0=(float*)temp;thFactor=&thF;
  temp+=sizeof(float*);
  thOffset0=(float*)temp;thOffset=&thO;
  temp+=sizeof(float*);
  vFactor0=(float*)temp;vFactor=&vF;
  temp+=sizeof(float*);
  vOffset0=(float*)temp;vOffset=&vO;
  temp+=sizeof(float*);
  periRxAddr=(byte*)temp;
  temp+=RADIO_ADDR_LENGTH+1;
  temp+=105;                         // dispo 
  usrnames=(char*)temp;
  temp+=NBUSR*(LENUSRNAME+1);
  usrpass=(char*)temp;
  temp+=NBUSR*(LENUSRPASS+1);
  usrtime=(unsigned long*)temp;
  temp+=NBUSR*sizeof(long);
  toPassword=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  usrpretime=(unsigned long*)temp;
  temp+=NBUSR*sizeof(long);
  maxCxWt=(unsigned long*)temp;
  temp+=sizeof(long);    
  maxCxWu=(unsigned long*)temp;
  temp+=sizeof(long);

  mailFromAddr=(char*)temp;
  temp+=LMAILADD+1;
  mailPass=(char*)temp;     
  temp+=LMAILPWD+1;
  mailToAddr1=(char*)temp;
  temp+=LMAILADD+1;
  mailToAddr2=(char*)temp;
  temp+=LMAILADD+1;
  periMail1=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  periMail2=(uint16_t*)temp;
  temp+=sizeof(uint16_t);
  browserPort=(uint16_t*)temp;
  temp+=sizeof(uint16_t);          

  configEndOfRecord=(byte*)temp;      // doit être le dernier !!!

  configInitVar();

  
//  Serial.print((long)configEndOfRecord);Serial.print(" ");Serial.println((long)configBegOfRecord);
  long configRecLength=(long)configEndOfRecord-(long)configBegOfRecord+1;  
  Serial.print("CONFIGRECLEN=");Serial.print(CONFIGRECLEN);Serial.print("/");Serial.print(configRecLength);Serial.print("  ");
  Serial.print("MLMSET/LENMESS=");Serial.print(MLMSET);Serial.print("/");Serial.print(LENMESS);
  delay(10);if((configRecLength!=CONFIGRECLEN) || MLMSET>LENMESS) {ledblink(BCODECONFIGRECLEN);}
  
  nbfonct=(strstr(fonctions,"last_fonc_")-fonctions)/LENNOM;
  faccueil=(strstr(fonctions,"accueil___")-fonctions)/LENNOM;
  fdatasave=(strstr(fonctions,"data_save_")-fonctions)/LENNOM;
  fdatana=(strstr(fonctions,"data_na___")-fonctions)/LENNOM;
  fperiSwVal=(strstr(fonctions,"peri_intv0")-fonctions)/LENNOM;
  fdone=(strstr(fonctions,"done______")-fonctions)/LENNOM;
  fpericur=(strstr(fonctions,"peri_cur__")-fonctions)/LENNOM;
  fperipass=(strstr(fonctions,"peri_pass_")-fonctions)/LENNOM;
  fpassword=(strstr(fonctions,"password__")-fonctions)/LENNOM;
  fusername=(strstr(fonctions,"username__")-fonctions)/LENNOM;
  fuserref=(strstr(fonctions,"user_ref__")-fonctions)/LENNOM;
  fperitst=(strstr(fonctions,"peri_tst__")-fonctions)/LENNOM;
  ffavicon=(strstr(fonctions,"favicon___")-fonctions)/LENNOM;
  
  Serial.print("  nbfonct=");Serial.println(nbfonct);
  Serial.print("RECCHAR=");Serial.print(RECCHAR);Serial.print(" LBUFSERVER=");Serial.println(LBUFSERVER);
}


void configExport(char* bec)
{
  uint16_t ll=0;
  memset(bec,0x00,LBEC);
  strcat(bec,"0000;");                                        // len ascii
  strcat(bec,serverName);                                     // nom server
  strcat(bec," v");strcat(bec,VERSION);                       // version
  strcat(bec,";");
  ll=strlen(bec);
  
  for(int pp=0;pp<4;pp++){                                    // ip server
    sprintf(bec+ll+pp*4,"%03u",(uint16_t)localIp[pp]);
    if(pp<3){*(bec+ll+(pp+1)*4-1)='.';}}ll+=15;
  *(bec+ll)=';';ll++;
  sprintf(bec+ll,"%05u",(uint16_t)*perifPort);ll+=5;          // perifPort
  *(bec+ll)=';';ll++;
  //sprintf(bec+ll,"%05u",(uint16_t)*browserPort);ll+=5;      // browser !!!!!!!!!! modifier la cible de l'export !!!!!!!!!!
  //*(bec+ll)=';';ll++;
  sprintf(bec+ll,"%05u",(uint16_t)*remotePort);ll+=5;         // remote
  *(bec+ll)=';';ll++;
  sprintf(bec+ll,"%05u",(uint16_t)*serverUdpPort);ll+=5;      // udp
  *(bec+ll)=';';ll++;
  strcat(bec+ll,peripass);                                    // peripass
  strcat(bec,";");
}

void wifiExport(char* bec,uint8_t selssid)
{
  strcat(bec,ssid+(selssid-1)*(LENSSID+1));                    // ssid1
  strcat(bec,";");
  strcat(bec,passssid+(selssid-1)*(LPWSSID+1));                // pwd1
  strcat(bec,";");
}  

void concExport(char* bec,uint8_t concNb)
{
  uint16_t ll=strlen(bec);
  uint8_t i0=concNb;
  uint8_t i1=concNb;
  if(concNb>MAXCONC){i0=0;i1=MAXCONC;}

  for(uint8_t i=i0;i<=i1;i++){                                                // conc Mac
    //memcpy(bec+ll,(concMac+i*MACADDRLENGTH),MACADDRLENGTH);
    unpackMac(bec+ll,(concMac+i*MACADDRLENGTH));
    ll+=MACADDRLENGTH*3-1;
    *(bec+ll)=';';ll++;
    for(int pp=0;pp<4;pp++){                                                  // ip concentrateur
      sprintf(bec+ll+pp*4,"%03u",(uint16_t)*(concIp+i*4));
      if(pp<3){*(bec+ll+(pp+1)*4-1)='.';}}
    ll+=15;
    *(bec+ll)=';';ll++;
    sprintf(bec+ll,"%05u",(uint16_t)*(concPort+i));ll+=5;                     // concPort
    *(bec+ll)=';';ll++;
    *(bec+ll)='\0';
  }
}

void concExport(char* bec)
{
  return concExport(bec,99);
}

void periExport(char* bec ,uint8_t concNb)
{
    uint16_t ll=strlen(bec);

    memcpy(bec+ll,concRx+concNb*RADIO_ADDR_LENGTH,RADIO_ADDR_LENGTH);             // conc Rx addr
    ll+=RADIO_ADDR_LENGTH;
    *(bec+ll)=';';ll++;
    sprintf(bec+ll,"%03u",(uint16_t)*(concChannel+concNb));ll+=3;                 // concChannel
    *(bec+ll)=';';ll++;
    sprintf(bec+ll,"%01u",(uint16_t)*(concRfSpeed+concNb));ll+=1;                 // concRfSpeed
    *(bec+ll)=';';ll++;
    sprintf(bec+ll,"%01u",concNb);ll+=1;                                          // rang conc 
    *(bec+ll)=';';ll++; 
    
    uint16_t value;
    value=(uint16_t)(*vFactor*10000);
    sprintf(bec+ll,"%04u",value);ll+=4;                                           // vFactor
    *(bec+ll)=';';ll++;
    value=(uint16_t)(*vOffset);
    sprintf(bec+ll,"%04u",value);ll+=4;                                           // vOffset
    *(bec+ll)=';';ll++;
    value=(uint16_t)(*thFactor*10000);
    sprintf(bec+ll,"%04u",value);ll+=4;                                           // thFactor
    *(bec+ll)=';';ll++;
    value=(uint16_t)(*thOffset);
    sprintf(bec+ll,"%04u",value);ll+=4;                                           // thOffset
    *(bec+ll)=';';ll++;
    *(bec+ll)=*concPeriParams+PMFNCVAL;ll+=1;                                          // provenance periParams (0=périf 1=saisie server)
    *(bec+ll)=';';ll++;
    memcpy(bec+ll,periRxAddr,RADIO_ADDR_LENGTH);                                  // perif Rx addr
    ll+=RADIO_ADDR_LENGTH;
    *(bec+ll)=';';ll++;
    *(bec+ll)='\0';
}

void periImport(char* bec)
{   
    Serial.println(bec);
    Serial.print("checkData=");
    uint16_t ll=0;
    int cd=checkData(bec,&ll);                        // longueur stockée dans le message
    Serial.print(cd);
    if(cd!=1){Serial.println(" ko");}                 // renvoie mess = MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3) 
    else {
      Serial.println(" ok");
      int sr=0;
      if(*concPeriParams==0){
        Serial.println(bec);
        bec+=5;      // skip len
        *vFactor=convStrToNum((char*)(bec),&sr)/10000;bec+=5;
        *vOffset=convStrToNum((char*)(bec),&sr);bec+=5;
        *thFactor=convStrToNum((char*)(bec),&sr)/10000;bec+=5;
        *thOffset=convStrToNum((char*)(bec),&sr);bec+=5;
        memcpy(periRxAddr,bec,RADIO_ADDR_LENGTH);
        configPrint();                                          
      }
    }
}

void subcprint(char* str1,void* strv,uint8_t nbl,uint8_t len1,int len2,unsigned long* cxtime)
{
  char* str2=(char*)strv;
  #define LBUFCPRINT LENSSID+1+LPWSSID+1+3+4+8
  char bufcprint[LBUFCPRINT];

  for(int nb=0;nb<nbl;nb++){
    if(*(str1+(nb*(len1+1)))!='\0'){
        memset(bufcprint,0x00,LBUFCPRINT);bufcprint[0]=0x20;sprintf(bufcprint+1,"%1u",nb+1);strcat(bufcprint," ");if((nb+1)<10){strcat(bufcprint," ");}
        strcat(bufcprint,str1+(nb*(len1+1)));strcat(bufcprint," ");
        int lsp=(len1-strlen(str1+nb*(len1+1)));for(int ns=0;ns<lsp;ns++){strcat(bufcprint," ");}
        strcat(bufcprint,str2+(nb*(len2+1)));
        lsp=(len2-strlen(str2+nb*(len2+1)));for(int ns=0;ns<lsp;ns++){strcat(bufcprint," ");}
        Serial.print(bufcprint);if(cxtime[nbl]!=0){Serial.print(cxtime[nbl]);}Serial.println();
    }
  }
}

void subConcPrint()
{
  Serial.println("concentrateurs :");

  for(uint8_t i=0;i<MAXCONC;i++){
    Serial.print(i+1);Serial.print(" ");
    serialPrintMac(concMac+i*MACADDRLENGTH,0);Serial.print(" ");
    serialPrintIp(concIp+i*4*sizeof(byte));Serial.print("/");
    Serial.print(*(concPort+i));Serial.print(" ");
    for(uint8_t j=0;j<RADIO_ADDR_LENGTH;j++){Serial.print((char)*(concRx+i*RADIO_ADDR_LENGTH+j));}Serial.print(" ");
    Serial.print(*(concChannel+i));Serial.print(" ");
    Serial.print(*(concRfSpeed+i));Serial.print(" ");
    Serial.println();
  }
 
  Serial.print("N° concentrateur pour perif ");Serial.println(*concNb);
  Serial.print("0-keep/1-new ");Serial.print(*concPeriParams);Serial.print("     periRxAddr ");Serial.println((char*)periRxAddr);
  Serial.print("vFactor ");delay(10);Serial.print(*vFactor*10000);
  Serial.print(" vOffset ");Serial.print(*vOffset);
  Serial.print(" thFactor ");Serial.print(*thFactor*10000);
  Serial.print(" thOffset ");Serial.println(*thOffset);
  Serial.println();
}

void configPrint()
{
  Serial.print("serverName=");Serial.println(serverName);
  Serial.print(" Mac=");serialPrintMac(mac,0);
  Serial.print(" localIp=");serialPrintIp(localIp);
  Serial.print("/");Serial.print(*perifPort);Serial.print("/");Serial.print(*browserPort);
  Serial.print("/");Serial.print(*remotePort);Serial.print("/");Serial.println(*serverUdpPort);      
  Serial.print(" peripass=");Serial.print(peripass);Serial.print(" toPassword=");Serial.println(*toPassword);
  Serial.print(" table ssid ");Serial.print(*ssid1);Serial.print("/");Serial.println(*ssid2);
  subcprint(ssid,passssid,MAXSSID,LENSSID,LPWSSID,0);
  Serial.println(" table user ");subcprint(usrnames,usrpass,NBUSR,LENUSRNAME,LENUSRPASS,usrtime);
  Serial.print(" mailFrom=");Serial.print(mailFromAddr);Serial.print(" pass=");Serial.println(mailPass);     
  Serial.print(" mailTo1=");Serial.print(mailToAddr1);Serial.print(" mailTo2=");Serial.println(mailToAddr2);  
  Serial.print(" peri1=");Serial.print(*periMail1);Serial.print(" peri2=");Serial.println(*periMail2);
  Serial.print(" maxCxWt ");Serial.print(*maxCxWt);Serial.print(" maxCxWu ");Serial.println(*maxCxWu);
  subConcPrint();
  Serial.println();
}

int configLoad()
{
  int i=0;
  char configFile[]="srvconf\0";
  if(sdOpen(configFile,&fconfig)==SDKO){return SDKO;}
  for(i=0;i<CONFIGRECLEN;i++){configRec[i]=fconfig.read();}
  fconfig.close();
  configLocalUpdate();
  return SDOK;
}

int configSave(uint8_t addedLength)
{
// pour ajouter des variables à l'enregistrement de config :
//        1) créer les pointeurs dans frontal et las ajouter en extern dans periph.cpp
//        2) nnn longueur ajoutée ; ajouter configSave(nnn) dans le setup juste après configLoad()
//        3) télécharger 
//        4) enlever configSave(nnn), 
//           ajouter la variable à la fin de configInit(), dans configInitVar() et configPrint() ; 
//           modifier PERIRECLEN et télécharger

  int i=0;
  int sta;
  int cl=CONFIGRECLEN;
  char configFile[]="srvconf\0";
  
  configRecordUpdate();

  if(sdOpen(configFile,&fconfig)!=SDKO){
    sta=SDOK;
    fconfig.seek(0);
  
      if(addedLength!=0){
        fconfig.remove();
        if (!fconfig.open(configFile, O_RDWR | O_CREAT | O_TRUNC)) {
          Serial.println(" create failed");}
        else {
          fconfig.seek(0);
          char a=0x00;
          for(i=0;i<CONFIGRECLEN;i++){fconfig.write(configRec[i]);}
          for(i=0;i<addedLength;i++){fconfig.write(a);}
          fconfig.close();
          Serial.println(" create ok");
        }
        while(1){ledblink(1);}
      }  
  
  for(i=0;i<CONFIGRECLEN;i++){fconfig.write(configRec[i]);}
  fconfig.close();
  }
  else sta=SDKO;
  Serial.print("configSave status=");Serial.print(sta);Serial.print(" len=");Serial.println(cl);
  return sta;
}

int configSave()
{
  return configSave(0);
}

/* ---------- périphériques ---------- */

void periCheck(uint16_t num,const char* text){periSave(NBPERIF+1,PERISAVESD);periLoad(num);Serial.print(" ");Serial.print(text);Serial.print(" perinum(");Serial.print(num);Serial.print(") sw=");Serial.print(*periSwNb);Serial.print(" det=");Serial.println(*periDetNb);periLoad(NBPERIF+1);}


void periFname(uint16_t num,char* fname)
{
  strcpy(fname,"PERI");
  fname[4]=(char)(num/10+48);
  fname[5]=(char)(num%10+48);
  fname[6]='\0';
}

void periInputPrint(byte* input)
{
  Serial.print("inputs ");
#define LBINP 23
  char binput[LBINP];
  byte a;
  char ed[]="de",es[]="es";  
  
  for(int ninp=0;ninp<NBPERRULES;ninp++){  

      memset(binput,0x20,LBINP-1);binput[LBINP-1]=0x00;
      binput[0]=ed[((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPEN_PB)&0x01)];                         // en input
      binput[2]=es[((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPDETES_PB)&0x01)];                      // edge/static input
      binput[4]=((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPOLDLEV_PB)&0x01)+48;                      // prev level
      binput[6]=((*(uint8_t*)(input+2+ninp*PERINPLEN)>>PERINPVALID_PB)&0x01)+48;                       // valid level
      a=*(uint8_t*)(input+ninp*PERINPLEN)&PERINPNT_MS;
      if(a>3){a=4;}binput[8]=*(inptyps+2+a*2);binput[9]=*(inptyps+2+a*2+1);                            // type détec src
      a=*(uint8_t*)(input+ninp*PERINPLEN)>>PERINPNVLS_PB;conv_htoa(binput+11,&a);                      // n° détec src
      a=*(uint8_t*)(input+ninp*PERINPLEN+3)&PERINPNT_MS;
      if(a>3){a=4;}binput[14]=*(inptypd+2+a*2+1);binput[15]=*(inptypd+2+a*2+1);                        // type détec dest
      a=*(uint8_t*)(input+ninp*PERINPLEN+3)>>PERINPNVLS_PB;conv_htoa(binput+17,&a);                    // n° détec dest
      a=(*((uint8_t*)(input+2+ninp*PERINPLEN))&PERINPACT_MS)>>PERINPACTLS_PB;conv_htoa(binput+20,&a);  // act input
      Serial.print(binput);
      //for(int tact=0;tact<LENTACT;tact++){Serial.print(inpact+2+a*LENTACT+tact);}
      sp("  / ",0);
    }
    Serial.println();
}

void periPulsePrint(uint16_t* pulseCtl,uint32_t* pulseOne,uint32_t* pulseTwo,uint32_t* currOne,uint32_t* currTwo)
{
  Serial.print("pulses(f-e1 t1/cur1 e2 t2/cur2) ");
  for(int pu=0;pu<NBPULSE;pu++){
    Serial.print((*(uint16_t*)pulseCtl>>(PMFRO_VB+pu*PCTLBIT))&0x01);sp("-",0);         // fr bit
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTOE_VB+pu*PCTLBIT))&0x01);sp(" ",0);       // time one en
    Serial.print(*(uint32_t*)(pulseOne+pu));sp("/",0);                                  // time one
    Serial.print(*(uint32_t*)(currOne+pu));sp(" ",0);                                   // curr one    
    Serial.print(((*(uint16_t*)pulseCtl)>>(PMTTE_VB+pu*PCTLBIT)&0x01));sp(" ",0);       // time two en
    Serial.print(*(uint32_t*)(pulseTwo+pu));sp("/",0);                                  // time two
    Serial.print(*(uint32_t*)(currTwo+pu));                                             // curr two    
    if(pu<NBPULSE-1){sp("  |  ",0);}
  }Serial.println();
}

void periDetServPrint(uint8_t* detserv)
{
  Serial.print(" detserv=");
  for(int d=NBDSRV-1;d>=0;d--){Serial.print(mDSval(d));} 
}  


void  periPrint(uint16_t num)
{
  Serial.print(num);Serial.print("/");Serial.print(*periNum);Serial.print(" ");Serial.print(periNamer);Serial.print(" ");
  serialPrintMac(periMacr,0);Serial.print(" ");serialPrintIp(periIpAddr);Serial.print(" port=");Serial.print(*periPort);
  Serial.print(" sw=");Serial.print(*periSwNb);Serial.print(" det=");Serial.print(*periDetNb);Serial.print(" ");
  for(int ver=0;ver<LENVERSION;ver++){Serial.print(periVers[ver]);}Serial.println();
  Serial.print("SWcde=(");if((*periSwCde&0xF0)==0){Serial.print("0");}Serial.print(*periSwCde,HEX);Serial.print(") ");
  for(int s=MAXSW;s>=1;s--){Serial.print(periSwRead(s));}
  periDetServPrint(memDetServ);Serial.println(millis());
  periPulsePrint((uint16_t*)periSwPulseCtl,periSwPulseOne,periSwPulseTwo,periSwPulseCurrOne,periSwPulseCurrTwo);
  periInputPrint(periInput);
/*  Serial.print("Anal=");Serial.print(*periAnal);Serial.print(" low=");Serial.print(*periAnalLow);Serial.print(" high=");Serial.print(*periAnalHigh);
  Serial.print(" adcOffset=");Serial.print(*periAnalOffset1);Serial.print(" adcFactor=");Serial.print(*periAnalFactor);Serial.print(" floatOffset=");Serial.println(*periAnalOffset2);
  for(int k=0;k<NBANST;k++){byte p=periAnalCb[k];Serial.print(" an Cb(0-FF)=");Serial.print(p,HEX);Serial.print(" an Det=");Serial.print(periAnalDestDet[k]);Serial.print(" an Ref Det=");Serial.print(periAnalRefDet[k]);Serial.print(" an n° memo=");Serial.println(periAnalMemo[k]);}Serial.println();
  for(int k=0;k<MAXDET;k++){Serial.print(" dg Cb(0-FF)=");Serial.print(*(byte*)&periDigitCb[k],HEX);Serial.print(" dg Det=");Serial.print(periDigitDestDet[k]);Serial.print(" dg Ref Det=");Serial.print(periDigitRefDet[k]);Serial.print(" dg n° memo=");Serial.println(periDigitMemo[k]);}Serial.println();
*/
}

void periSub(uint16_t num,int sta,bool sd)
{
  Serial.print(num);Serial.print("/");Serial.print(*periNum);Serial.print(")status=");Serial.print(sta);Serial.print(";save=");Serial.print(sd);Serial.print(" NbSw=");Serial.print(*periSwNb);Serial.print(" srv=");Serial.print(*periProg);Serial.print(" port=");Serial.println(*periPort);
}

int periCacheLoad(uint16_t num)
{
    char periFile[7];periFname(num,periFile);
    if(sdOpen(periFile,&fperi)==SDOK){
      for(int i=0;i<PERIRECLEN;i++){periCache[(num-1)*PERIRECLEN+i]=fperi.read();}              
      fperi.close();
      periCacheStatus[num]=CACHEISFILE;           // le cache est à l'image du fichier
      return SDOK;    
    }
    Serial.print(periFile);
    return SDKO;
}  

void memDetPrint(uint32_t ds)
{
  Serial.print(" memDet=");
  for(int i=3;i>=0;i--){
    for(int j=7;j>=0;j--){char a=0x30+((ds>>(i*8+j))&0x01);Serial.print(a);}Serial.print(" ");
  }
}

   
void periDSU(uint8_t* mds,uint8_t dd,byte cb,uint8_t res)
{    
    //memDetPrint(*mds);Serial.print(" cb=");Serial.print(cb,HEX);
    //Serial.print(" destDet=");Serial.print(dd);Serial.print(" res=");Serial.print(res);
    
    uint8_t ds=0;
    //for(uint8_t i=0;i<MDSLEN;i++){if((mds[i] & mDSmaskbit[dd*MDSLEN+i]) !=0){ds=1;break;}}  // ds état du detServ
    uint8_t mi=dd>>3;if((mds[mi] & mDSmaskbit[dd*MDSLEN+mi]) !=0){ds=1;}  // ds état du detServ
    //Serial.print(" ds=");Serial.println(ds);
        switch(cb>>4){
          case 0: break;
          case 1: if(res==1){mds[mi] &= ~mDSmaskbit[dd*MDSLEN+mi];}break;
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] &= ~mDSmaskbit[dd*MDSLEN+i];}}break;
          case 2: if(res==1){mds[mi] |= mDSmaskbit[dd*MDSLEN+mi];}break;
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] |= mDSmaskbit[dd*MDSLEN+i];}}break;
          case 3: if(ds+res!=0){mds[mi] |= mDSmaskbit[dd*MDSLEN+mi];}
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] |= mDSmaskbit[dd*MDSLEN+i];}}
                  else {mds[mi] &= ~mDSmaskbit[dd*MDSLEN+mi];}
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] &= ~mDSmaskbit[dd*MDSLEN+i];}}
                  break;
          case 4: if(ds+res==2){mds[mi] |= mDSmaskbit[dd*MDSLEN+mi];}
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] |= mDSmaskbit[dd*MDSLEN+i];}}
                  else {mds[mi] &= ~mDSmaskbit[dd*MDSLEN+mi];}
                    //for(uint8_t i=0;i<MDSLEN;i++){mds[i] &= ~mDSmaskbit[dd*MDSLEN+i];}}
                  break;
          case 5: if((ds+res!=0)&&(ds+res!=2)){mds[mi] |= mDSmaskbit[dd*MDSLEN+mi];}
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] |= mDSmaskbit[dd*MDSLEN+i];}}
                  else {mds[mi] &= ~mDSmaskbit[dd*MDSLEN+mi];}
                    //for(uint8_t i=0;i<MDSLEN;i++){mds[i] &= ~mDSmaskbit[dd*MDSLEN+i];}}
                  break;
          case 6: if(res==1){mds[mi] |= mDSmaskbit[dd*MDSLEN+mi];}
                  //for(uint8_t i=0;i<MDSLEN;i++){mds[i] |= mDSmaskbit[dd*MDSLEN+i];}}
                  else {mds[mi] &= ~mDSmaskbit[dd*MDSLEN+mi];}
                    //for(uint8_t i=0;i<MDSLEN;i++){mds[i] &= ~mDSmaskbit[dd*MDSLEN+i];}}
                  break;
          default: break;
        }
}

void periDetServUpdate()
{
  /* analog */
  //Serial.println("\nAnalog ");
  for(int i=0;i<NBANST;i++){
    uint8_t result=0;
    byte c=periAnalCb[i];
    //Serial.print(" i=");Serial.print(i);Serial.print(" cb=");Serial.print(c,HEX);
    if((c&0x08)!=0){            // enable ?      
      uint8_t r=periAnalCb[i]&0x04;       // active level
      switch(i){
        case 0: if((*periAnal>*periAnalHigh && r==1) || (*periAnal<=*periAnalHigh && r==0) ){result=1;}break;
        case 1: if((*periAnal=*periAnalHigh && r==1) || (*periAnal!=*periAnalHigh && r==0) ){result=1;}break;
        case 2: if(((*periAnal<*periAnalHigh && *periAnal>*periAnalLow) && r==1) || ((*periAnal>=*periAnalHigh && *periAnal<=*periAnalLow) && r==0) ){result=1;};break;
        case 3: if((*periAnal=*periAnalLow && r==1) || (*periAnal!=*periAnalLow && r==0) ){result=1;}break;
        case 4: if((*periAnal<*periAnalLow && r==1) || (*periAnal>=*periAnalLow && r==0) ){result=1;}break;
        default: break;
      }
      periDSU(memDetServ,periAnalDestDet[i],c,result);        
    }
    //Serial.println();
  }

  /*  digital */
  //Serial.println("Digital ");
  for(int i=0;i<*periDetNb;i++){
    byte c=periDigitCb[i];
    //Serial.print(" i=");Serial.print(i);Serial.print(" cb=");Serial.print(c,HEX);
    if((c&0x08)!=0){             // enable ?
      uint8_t val=(*periDetVal>>(i*2))&DETBITLH_VB;
      uint8_t result=0;if(((periDigitCb[i]&0x04)>>2)==val){result=1;}     // 1 direct, 2 inverse
      periDSU(memDetServ,periDigitDestDet[i],c,result);        
    }
    //Serial.println();
  }
}

int periCacheSave(uint16_t num)                                     // sauve les perifs du cache != file
{     
    int sta=SDOK;
    char periFile[7];periFname(num,periFile);

    if(periCacheStatus[num]!=CACHEISFILE){                          // si le fichier n'est pas à jour du cache -> sauvegarde
      if(sdOpen(periFile,&fperi)==SDOK){
        periDetServUpdate();
        fperi.seek(0);
        for(int i=0;i<PERIRECLEN;i++){fperi.write(periCache[(num-1)*PERIRECLEN+i]);}
        fperi.close();
        periCacheStatus[num]=CACHEISFILE;                           // le fichier est à l'image du cache
        //for(int x=0;x<4;x++){lastIpAddr[x]=periIpAddr[x];}
      }
      else{Serial.print(periFile);Serial.println(" ko");sta=SDKO;}
    }
    return sta;
}

int periLoad(uint16_t num)                                          // charge le cache dans les variables (periRec)
{
if(num<=0 || num>NBPERIF){Serial.print(" periLoad ");Serial.println(num);delay(5);ledblink(BCODENUMPER);} 
  int i=0;
  int sta=SDOK;
  
    for(i=0;i<PERIRECLEN;i++){periRec[i]=periCache[(num-1)*PERIRECLEN+i];}    // copie dans variables  
  
  return sta;
}

int periSave(uint16_t num,bool sd)            // si sd==PERISAVELOCAL copie periRec dans cache seul
                                              // sinon                copie sur SD aussi
{
  //Serial.print("\nperiSave ");Serial.print(sd);Serial.print(" ");Serial.println(num);
  int sta=SDOK;
  
  for(int i=0;i<PERIRECLEN;i++){periCache[(num-1)*PERIRECLEN+i]=periRec[i];}    // chargement perif courant dans cache[num]
  periCacheStatus[num]=!CACHEISFILE;                                            // cache != file

  *periNum=num;
  
  if(sd!=PERISAVELOCAL){
    sta=periCacheSave(num);
    if(sta){periCacheStatus[num]=CACHEISFILE;}               // le fichier est à l'image du cache si sta = ok
  }

#ifdef SHDIAGS    
  Serial.print(" periSave(");periSub(num,sta,sd);
#endif //
  return sta;
}

void periTableLoad()                            // au démarrage du systeme ; charge le cache complet
{
  Serial.print("Load table perif ");
  periInit();
  long periRecLength=(long)periEndOfRecord-(long)periBegOfRecord+1;
  Serial.print("PERIRECLEN=");Serial.print(PERIRECLEN);Serial.print("/");Serial.print(periRecLength);
  delay(10);if(periRecLength!=PERIRECLEN){ledblink(BCODEPERIRECLEN);}

  for(int h=1;h<=NBPERIF;h++){
    Serial.print(" ");Serial.print(h);
    if(periCacheLoad(h)==SDKO){Serial.println(" KO");mail("PERITABLE_LOAD_HALT","");while(1){trigwd();delay(1000);}};}
  Serial.println(" ALL OK");
}  

void periTableSave(bool force)                  // à l'arret du systeme ; sauve le cache (complet si FORCE sinon perifs modifiés seuls)
{
  Serial.print("Save table perif ");
  
  for(int h=1;h<=NBPERIF;h++){
    Serial.print(".");
    if(force){periCacheStatus[h]=!CACHEISFILE;}
    if(periCacheSave(h)==SDKO){Serial.println(" KO");mail("PERITABLE_SAVE_ERROR_HALT","");while(1){trigwd();delay(1000);}};}
    Serial.println(" ALL OK");
}  

void periTableSave()
{
  periTableSave(false);
}

int periRemove(uint16_t num)
{
  char periFile[7];periFname(num,periFile);
  if(sdOpen(periFile,&fperi)==SDOK){fperi.remove();}
  return SDOK;
}

int periRaz(uint16_t num)
{
  periRemove(num);

  char periFile[7];periFname(num,periFile);

  if (!fperi.open(periFile, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.print(periFile);Serial.println(" create failed");
    mail("SD CREATE FAIL",periFile);
    return SDKO;
  }
// contiguous clusters
  fperi.truncate(0);
  if (!fperi.preAllocate(PERIRECLEN+100)) {
    mail("periRaz SD PREALLOC FAIL",periFile);
    Serial.print(periFile);Serial.println(" preallocation failed");
  }
  fperi.close();
  return SDOK;
}

int periLoadDirect(uint16_t num)
{
  int ret;
  char periFile[7];periFname(num,periFile);
  Serial.print(periFile);
  if(sdOpen(periFile,&fperi)==SDOK){
    for(int i=0;i<PERIRECLEN;i++){periRec[i]=fperi.read();}              
    ret=SDOK;
  }
  else {ret=SDKO;}
  fperi.close();
  return ret;
}


void periModification()        
// ------------- Redimensionne le nombre de règles à 64 dans les fichiers et periRec ------------------ 
/* Pour redimensionner une variable : 
 *  ajouter periModification() avant periTableLoad() dans frontal.ino
 *  NE PAS changer PERIRECLEN ni NBPERRULES
 *  compiler, charger et laisser démarrer le serveur
 *  le programme stoppe avant d'effecuer les modifs des fichiers périphériques pour permettre de changer la carte SD
 *  la led clignote rapidement ; appuyer le bouton HALT pour démarrer 
 *  à la fin la led clignote lentement
 *  corriger NBPERRULES et les éventuels paramètres modifiés dans shconst2.h
 *  corriger PERIRECLEN dans const.h
 *  enlever periModification() dans frontal.ino
 *  compiler, charger
 */
{
  Serial.println("modification en cours...");

  unsigned long t=millis();
  while(digitalRead(STOPREQ)!=LOW){
    t=millis();digitalWrite(PINLED,HIGH);while((millis()-t)<1500){if(digitalRead(STOPREQ)==LOW){break;}}
    Serial.print('*');
    t=millis();digitalWrite(PINLED,LOW);while((millis()-t)<1500){if(digitalRead(STOPREQ)==LOW){break;}}
  }
  
  periInit();
  periInitVar();
  for(uint16_t i=1;i<=NBPERIF;i++){
  //for(uint16_t i=3;i<=3;i++){    
    if(periLoadDirect(i)!=SDOK){Serial.println(" load KO");}
    else{
      Serial.print(*periNum);Serial.print(' ');
      Serial.print(periNamer);
      Serial.println(" load OK");

      #define ADDED 24
      #define PERIRECLEN_NEW PERIRECLEN+ADDED*PERINPLEN // 24+24=48
      char periRec_New[PERIRECLEN_NEW];memset(periRec_New,0x00,PERIRECLEN_NEW);

      /* modif nbre rules      
      unsigned long pos=(byte*)periInput-(byte*)periRec;
      unsigned long len=(unsigned long)PERIRECLEN-(unsigned long)(pos+NBPERRULES*PERINPLEN);
      Serial.print("             ");Serial.print(PERIRECLEN);Serial.print(' ');Serial.print(PERIRECLEN_NEW);Serial.print(' ');Serial.print(pos);Serial.print(' ');Serial.print(len);
      memcpy(periRec_New,periRec,pos+NBPERRULES*PERINPLEN);                           // jusqu'à la fin des règles anciennes
      memcpy(periRec_New+pos+48*PERINPLEN,periInput+NBPERRULES*PERINPLEN,len);        // 64 règles dans fichier (48 utilisées : eeprom esp12 trop petite)
      char* new_periSwCde=(char*)periSwCde-periRec+periRec_New;
      *new_periSwCde=0x00;
      char* new_periSwSta=(char*)periSwSta-periRec+periRec_New;
      *new_periSwSta=0x00;
      
      //dumpstr(periRec,PERIRECLEN);Serial.println();dumpstr(periRec_New,PERIRECLEN_NEW);
      
      char* new_periNamer=(char*)periNamer-periRec+periRec_New;
      char* new_periPort=(char*)periPort-periRec+periRec_New+ADDED*PERINPLEN;
      char* new_periProtocol=(char*)periProtocol-periRec+periRec_New+ADDED*PERINPLEN;
      char* new_periProg=(char*)periProg-periRec+periRec_New+ADDED*PERINPLEN;
      char* new_periDetNb=(char*)periDetNb-periRec+periRec_New+ADDED*PERINPLEN;
      Serial.print(' ');Serial.print(new_periNamer);Serial.print(' ');Serial.print(*periPort);Serial.print('/');Serial.print(*(uint16_t*)new_periPort);Serial.print(" ");
      Serial.print(*periProtocol);Serial.print('/');Serial.print(*(char*)new_periProtocol);Serial.print(' ');      
      Serial.print(*periDetNb);Serial.print('/');Serial.print(*(uint8_t*)new_periDetNb);Serial.print(' ');      
      Serial.print(*periProg);Serial.print('/');Serial.print(*(bool*)new_periProg);Serial.print(' ');      
      */
      char periFile[7];periFname(i,periFile);
      Serial.print(periFile);
      //Serial.println();dumpstr(periRec_New,406);
      
      fperi.remove();
      if (!fperi.open(periFile, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.println(" create failed");}
      else {
        fperi.seek(0);
        for(int i=0;i<PERIRECLEN_NEW;i++){fperi.write(periRec_New[i]);}
        fperi.close();
        Serial.println(" create ok");
      }
    }
    Serial.println();
  }
  Serial.println("terminé");
  while(1){digitalWrite(PINLED,HIGH);delay(200);digitalWrite(PINLED,LOW);delay(200);}
}

void periConvert()        // !!!!!!!!!!!!!!!!!!!!!!!!! periLoad ne fonctionne plus avec le cache ; corriger avant d'utiliser !!!!!!!!!!!!!!!!
/* Pour ajouter une variable : 
 *  ajouter en fin de liste son descripteur dans frontal.ino, periInit() et periIinitVar()
 *  ajouter periconvert() devant periMaintenance() dans frontal.ino
 *  mettre en rem periTableLoad() s'il est placé avant periConvert() dans frontal.ino
 *  changer la ligne de "save" dans periSave() et ajouter la longueur supplémentaire
 *  NE PAS changer PERIRECLEN
 *  compiler, charger et laisser démarrer le serveur
 *  remettre la ligne de "save" normale dans periSave() 
 *  corriger PERIRECLEN dans const.h
 *  enlever les // devant periTableLoad() dans frontal.ino
 *  enlever periconvert() dans frontal.ino
 *  compiler, charger
 */
{
  Serial.println("conversion en cours...");
  
  char periFile[7];

  periInitVar();            // les champs ajoutés sont initialisés ; les autres récupèreront les valeurs précédentes
  for(uint16_t i=1;i<=NBPERIF;i++){
    periFname(i,periFile);Serial.print(periFile);
    if(periLoad(i)!=SDOK){Serial.print(" load KO");}
    else{
        fperi.remove();
        memset(periAnalCb,0x00,NBANST);
        memset(periAnalDestDet,0x00,NBANST);
        memset(periAnalRefDet,0xFF,NBANST);  
        memset(periAnalMemo,0xFF,NBANST);
        memset(periDigitCb,0x00,MAXDET);
        memset(periDigitDestDet,0x00,MAXDET);
        memset(periDigitRefDet,0xFF,MAXDET);  
        memset(periDigitMemo,0xFF,MAXDET);
      
      if(periSave(i,PERISAVESD)!=SDOK){Serial.print(" save KO");} 
    }
    Serial.println();
  }
  Serial.println("terminé");
  while(1){blink(1);};
}

void periInit()                 // pointeurs de l'enregistrement de table courant
{
  for(uint16_t nbp=0;nbp<NBPERIF;nbp++){periCacheStatus[nbp]=!CACHEISFILE;}
  
  periCur=0;
  byte* temp=(byte*)periRec;

  periBegOfRecord=temp;         // doit être le premier !!!
  periNum=(uint16_t*)temp;                           
  temp +=sizeof(uint16_t);
  periPerRefr=(int32_t*)temp;
  temp +=sizeof(int32_t);
  periPitch_=(int16_t*)temp;
  temp +=sizeof(int16_t);
  periLastVal_=(int16_t*)temp;
  temp +=sizeof(int16_t);
  periAlim_=(int16_t*)temp;
  temp +=sizeof(int16_t);
  periLastDateIn=(char*)temp;
  temp +=LENPERIDATE;
  periLastDateOut=(char*)temp;
  temp +=LENPERIDATE;
  periLastDateErr=(char*)temp;
  temp +=LENPERIDATE;
  periErr=(int8_t*)temp;
  temp +=sizeof(int8_t);
  periNamer=(char*)temp;
  temp +=PERINAMLEN;
  periVers=(char*)temp;
  temp +=LENVERSION;  
  periModel=(char*)temp;
  temp +=LENMODEL;                
  periMacr=(byte*)temp;
  temp +=6;
  periIpAddr=(byte*)temp;
  temp +=4;
  periSwNb=(byte*)temp;
  temp +=sizeof(byte);
  periSwCde=(byte*)temp;
  temp +=sizeof(byte);
  periInput=(byte*)temp;
  temp +=NBPERRULES*PERINPLEN*sizeof(byte);
  periSwPulseOne=(uint32_t*)temp;
  temp +=NBPULSE*sizeof(uint32_t);
  periSwPulseTwo=(uint32_t*)temp;
  temp +=NBPULSE*sizeof(uint32_t);  
  periSwPulseCurrOne=(uint32_t*)temp;
  temp +=NBPULSE*sizeof(uint32_t);
  periSwPulseCurrTwo=(uint32_t*)temp;
  temp +=NBPULSE*sizeof(uint32_t);  
  periSwPulseCtl=(byte*)temp;
  temp +=PCTLLEN*sizeof(byte);  
  periSwPulseSta=(byte*)temp;
  temp +=NBPULSE*sizeof(byte);
  periSwSta=(uint8_t*)temp;
  temp +=sizeof(uint8_t);
  periProg=(bool*)temp;
  temp +=sizeof(bool);
  periDetNb=(byte*)temp;
  temp +=sizeof(byte);
  periDetVal=(byte*)temp;
  temp +=sizeof(byte);
  periThOffset_=(int16_t*)temp;
  temp +=sizeof(int16_t); 
  periThmin_=(int16_t*)temp;
  temp +=sizeof(int16_t);
  periThmax_=(int16_t*)temp;
  temp +=sizeof(int16_t);  
  periVmin_=(int16_t*)temp;
  temp +=sizeof(int16_t);  
  periVmax_=(int16_t*)temp;
  temp +=sizeof(int16_t);  
  periDetServEn=(byte*)temp;
  temp +=1*sizeof(byte);
  periPerTemp=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periPort=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periProtocol=(byte*)temp;
  temp +=sizeof(byte);
  periAnal=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periAnalLow=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periAnalHigh=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periAnalOffset1=(uint16_t*)temp;
  temp +=sizeof(uint16_t);
  periAnalFactor=(float*)temp;
  temp +=sizeof(float);
  periAnalOffset2=(float*)temp;
  temp +=sizeof(float);
  periAnalCb=(uint8_t*)temp;
  temp +=NBANST*sizeof(uint8_t);
  periAnalDestDet=(uint8_t*)temp;
  temp +=NBANST*sizeof(uint8_t);  
  periAnalRefDet=(uint8_t*)temp;
  temp +=NBANST*sizeof(uint8_t);  
  periAnalMemo=(int8_t*)temp;
  temp +=NBANST*sizeof(int8_t);  
  periDigitCb=(uint8_t*)temp;
  temp +=MAXDET*sizeof(uint8_t);  
  periDigitDestDet=(uint8_t*)temp;
  temp +=MAXDET*sizeof(uint8_t);
  periDigitRefDet=(uint8_t*)temp;
  temp +=MAXDET*sizeof(uint8_t);
  periDigitMemo=(int8_t*)temp;
  temp +=MAXDET*sizeof(int8_t);  
  periSsidNb=(byte*)temp;
  temp +=1*sizeof(byte);
  periEndOfRecord=(byte*)temp;      // doit être le dernier !!!
  temp ++;

  periInitVar();
}

void periInitVar0()                  // pour bouton erase
{   // attention : perInitVar ne concerne que les variables de l'enregistrement de périphérique
    // lorsque periLoad est effectué periInitVar n'est pas utile
   memset(periInput,0x00,NBPERRULES*PERINPLEN*sizeof(byte));
   memset(periSwPulseOne,0x00,NBPULSE*sizeof(uint32_t));
   memset(periSwPulseTwo,0x00,NBPULSE*sizeof(uint32_t)); 
   memset(periSwPulseCurrOne,0x00,NBPULSE*sizeof(uint32_t)); 
   memset(periSwPulseCurrTwo,0x00,NBPULSE*sizeof(uint32_t));       
   memset(periSwPulseCtl,0x00,PCTLLEN);
   memset(periSwPulseSta,0x00,NBPULSE);      
}


void periInitVar()   // attention : perInitVar ne concerne que les variables de l'enregistrement de périphérique
{                    // lorsque periLoad est effectué periInitVar n'est pas utile
  *periNum=0;
  *periPerRefr=MAXSERVACCESS;
  *periPerTemp=TEMPERPREF;
  *periPitch_=DEFPITCH;
  *periLastVal_=0;
  *periAlim_=0;
  memset(periLastDateIn,0x00,LENPERIDATE);
  memset(periLastDateOut,0x00,LENPERIDATE);
  memset(periLastDateErr,0x00,LENPERIDATE);
  *periErr=0;
  memset(periNamer,' ',PERINAMLEN);periNamer[PERINAMLEN-1]='\0';
  memset(periVers,' ',LENVERSION);periVers[LENVERSION-1]='\0';
  memset(periModel,' ',LENMODEL);
  memset(periMacr,0x00,6);
  memset(periIpAddr,0x00,4);
  *periSwNb=0;
  *periSwCde=0;
  *periProg=FAUX;
  if(*periProg==FAUX){*periPort=0;}
  *periDetNb=0;
  *periDetVal=0;
  *periThOffset_=0;
  *periThmin_=-9900;
  *periThmax_=9900;
  *periVmin_=325;
  *periVmax_=350;
   memset(periDetServEn,0x00,2);
  *periProtocol=0; 
  *periAnal=0;      
  *periAnalLow=0;   
  *periAnalHigh=0;  
  *periAnalOffset1=0;
  *periAnalFactor=1;
  *periAnalOffset2=0;
  memset(periAnalCb,0x00,NBANST);
  memset(periAnalDestDet,0x00,NBANST);
  memset(periAnalRefDet,0xFF,NBANST);  
  memset(periAnalMemo,0xFF,NBANST);
  memset(periDigitCb,0x00,MAXDET);
  memset(periDigitDestDet,0x00,MAXDET);
  memset(periDigitRefDet,0xFF,MAXDET);  
  memset(periDigitMemo,0xFF,MAXDET);

  
   periInitVar0();
   // attention : perInitVar ne concerne que les variables de l'enregistrement de périphérique
   // lorsque periLoad est effectué periInitVar n'est pas utile
}

void periSwCdUpdate(uint8_t sw,uint8_t stat)    // maj sw courant selon stat
{
  byte msk=(byte)0x03<<(sw*2);                  // 03 0C 30 C0 disjoncteurs
  *periSwCde  &= ~msk;
  if(stat!=0){*periSwCde |= msk;}               
}

byte periSwRead(uint8_t sw)                      // etat sw courant
{
  return (*periSwCde>>(sw*2))&0x03;
}

uint8_t getMotherRemoteStatus(uint16_t peri,uint8_t sw)
{
  for(uint8_t i=0;i<MAXREMLI;i++){
    if(remoteT[i].peri==peri && remoteT[i].sw==sw && remoteT[i].multRem!=0){
      return remoteN[remoteT[i].multRem-1].enable;
    }
  }
  return 0xff;
}

/*void periSwLevUpdate(uint8_t sw,uint8_t stat)   // maj lev(sw) selon stat
{
  byte msk=(byte)0x01<<(sw);                  // 01 02 04 08 10 20 40 80 levels
  *periSwSta &= msk;
  if(stat==1){*periSwSta |= msk;}
}*/

/*byte periSwLev(uint8_t sw)                      // lev de sw courant
{
  return (*periSwCde>>(sw*2))&0x03;
}*/

/*void remMemDetUpdate(uint8_t rem,uint8_t endet)               // maj memDetServ suite à chgt état remote
{
  if(endet==REM_ENABLE){
    uint8_t mi=remoteT[rem].deten>>3;
    uint8_t mi1=(remoteT[rem].deten+1)>>3;
    memDetServ[mi] &= ~mDSmaskbit[remoteT[rem].deten*MDSLEN+mi];                                        // raz memDetServ disj
    //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] &= ~mDSmaskbit[remoteT[rem].deten*MDSLEN+i];}           // raz memDetServ disj
    memDetServ[mi1] &= ~mDSmaskbit[(remoteT[rem].deten+1)*MDSLEN+mi1];                                  // raz memDetServ forcage
    //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] &= ~mDSmaskbit[(remoteT[rem].deten+1)*MDSLEN+i];}       // raz memDetServ forcage
    if(remoteN[remoteT[rem].num-1].oldenable!=0){
      memDetServ[mi] &= ~mDSmaskbit[(remoteT[rem].deten+1)*MDSLEN+mi];}                                 // set memDetServ disj
      //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] |= mDSmaskbit[remoteT[rem].deten*MDSLEN+i];}}         // set memDetServ disj
    if(remoteN[remoteT[rem].num-1].oldenable>1){
      memDetServ[mi1] &= ~mDSmaskbit[(remoteT[rem].deten+1)*MDSLEN+mi1];}                               // set memDetServ forcage
      //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] &= ~mDSmaskbit[(remoteT[rem].deten+1)*MDSLEN+i];}}    // set memDetServ forçage
  }
  if(endet==REM_DETEC){
    uint8_t mi=remoteT[rem].detec>>3;
    
    //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] &= ~mDSmaskbit[remoteT[rem].detec*MDSLEN+i];}              // update memDetServ on/off
    if(remoteN[remoteT[rem].num-1].onoff!=0){
      memDetServ[mi] |= mDSmaskbit[remoteT[rem].detec*MDSLEN+mi];}
    else {memDetServ[mi] &= ~mDSmaskbit[remoteT[rem].detec*MDSLEN+mi];}                                          // update memDetServ on/off
      //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] |= mDSmaskbit[remoteT[rem].detec*MDSLEN+i];}}
  }
}*/

/*void periSwSync()                               // synchro memdetserv sur remotes au démarrage (sychronisation periSwCde obsolete)
{
   // remoteN[k].onoff   etat du bit k memDetServ remoteT[k].detec
   // remoteN[k].oldenable  etat du bit k memDetServ remoteT[k].deten (dijoncteur) 

  uint8_t nbsync=0;
  for(uint8_t k=0;k<MAXREMLI;k++){
    if(remoteT[k].peri!=0 && remoteT[k].peri<=NBPERIF && remoteT[k].sw<MAXSW){
      
        //periCur=remoteT[k].peri;periLoad(periCur);
        //periSwCdUpdate(remoteT[k].sw,remoteN[remoteT[k].num-1].oldenable);             // update disjoncteur perif
        //remMemDetUpdate(k,REM_ENABLE);                                              // update memDetServ disj et forçage
        remMemDetUpdate(k,REM_DETEC);                                               // update memDetServ on/off
      
        //periSave(periCur,PERISAVELOCAL);
        nbsync++;
      
    }
  }
  Serial.print("sync ");Serial.print(nbsync);Serial.println(" switchs");
}*/

void periModif()
/*    Pour modifier la structure des données 
 *     
 *     COPIER LES FICHIERS AVANT MODIF
 *     
 *     modifier les 2 listes de pointeurs (Iperixxx) plus loin avec la nouvelle structure 
 *    
 *     modifier les transferts de données 
 *     modifier la nouvelle longueur totale
 *     enlever les // devant derrière la séquence dans frontal.ino
 *     
 *     compiler, charger et laisser démarrer le serveur
 *     
 *     remettre les // devant derrière la séquence dans frontal.ino     
 *     
 *     corriger PERIRECLEN dans const.h
 *     copier les pointeurs dans frontal.ino et periInit
 *     enlever les 'I'
 *     
 *  compiler, charger 
 */
{
/*
  int*      IperiNum;                      // ptr ds buffer : Numéro du périphérique courant
  long*     IperiPerRefr;                  // ptr ds buffer : période datasave minimale
  float*    IperiPitch;                    // ptr ds buffer : variation minimale de température pour datasave
  float*    IperiLastVal;                  // ptr ds buffer : dernière valeur de température  
  float*    IperiAlim;                     // ptr ds buffer : dernière tension d'alimentation
  char*     IperiLastDateIn;               // ptr ds buffer : date/heure de dernière réception
  char*     IperiLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
  char*     IperiLastDateErr;              // ptr ds buffer : date/heure de derniere anomalie com
  int8_t*   IperiErr;                      // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
  char*     IperiNamer;                    // ptr ds buffer : description périphérique
  char*     IperiVers;                     // ptr ds buffer : version logiciel du périphérique
  byte*     IperiMacr;                     // ptr ds buffer : mac address 
  byte*     IperiIpAddr;                   // ptr ds buffer : Ip address
  byte*     IperiSwNb;                    // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
  byte*     IperiSwVal;                   // ptr ds buffer : état/cde des inter  
  byte*     IperiSwMode;                  // ptr ds buffer : Mode fonctionnement inters            
  uint16_t* IperiSwPulse;                 // ptr ds buffer : durée pulses sec si interrupteur (0=stable, pas de pulse)
  uint16_t* IperiSwPulseCurr;             // ptr ds buffer : temps courant pulses
  byte*     IperiSwPulseCtl;              // ptr ds buffer : mode pulses
  byte*     IperiSwPulseTrig;             // ptr ds buffer : durée trig 
  uint8_t*  Idispo;                       // ptr ds buffer : nbre sonde
  bool*     IperiProg;                    // ptr ds buffer : flag "programmable"
  byte*     IperiDetNb;                   // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
  byte*     IperiDetVal;                  // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
  

  
  periCur=0;
  int* filler;

  char periRecNew[200];    // la taille n'a pas d'importance, c'est la longueur exacte qui sera enregistrée
  
  char* temp=periRecNew;
  
  IperiNum=(int*)temp;
  temp +=sizeof(int);
  IperiPerRefr=(long*)temp;
  temp +=sizeof(long);
  IperiPitch=(float*)temp;
  temp +=sizeof(float);
  IperiLastVal=(float*)temp;
  temp +=sizeof(float);
  IperiAlim=(float*)temp;
  temp +=sizeof(float);
  IperiLastDateIn=(char*)temp;
  temp +=6;
  IperiLastDateOut=(char*)temp;
  temp +=6;
  IperiLastDateErr=(char*)temp;
  temp +=6;
  IperiErr=(int8_t*)temp;
  temp +=sizeof(int8_t);   
  IperiNamer=(char*)temp;
  temp +=PERINAMLEN;
  IperiVers=(char*)temp;
  temp +=3;                  
  IperiMacr=(byte*)temp;
  temp +=6;
  IperiIpAddr=(byte*)temp;
  temp +=4;
  IperiSwNb=(byte*)temp;
  temp +=sizeof(byte);
  IperiSwCde=(byte*)temp;
  temp +=sizeof(byte);
  IperiSwMode=(byte*)temp;
  temp +=MAXSW*sizeof(byte);
  IperiSwPulse=(uint16_t*)temp;
  temp +=MAXSW*sizeof(uint16_t);
  IperiSwPulseCurr=(uint16_t*)temp;
  temp +=MAXSW*sizeof(uint16_t);
  IperiSwPulseCtl=(byte*)temp;
  temp +=MAXSW*sizeof(byte);
  IperiSwPulseTrig=(byte*)temp;
  temp +=MAXSW*sizeof(byte);      
  //dispo=(uint8_t*)temp;
  temp +=sizeof(uint8_t);
  IperiProg=(bool*)temp;
  temp +=sizeof(bool);
  IperiDetNb=(byte*)temp;
  temp +=sizeof(byte);
  IperiDetVal=(byte*)temp;
  temp +=sizeof(bool);

  temp ++;

int periNewLen=temp-periRecNew;
Serial.print("periNewLen=");Serial.println(periNewLen);

  // =========== transfert =============== 

  char periFile[7];
  char periNewFile[8];
  int i=0;

  for(i=1;i<=NBPERIF;i++){
    periFname(i,periFile);Serial.print(periFile);

// chargement ancien enregistrement
    
    if(periLoad(i)!=SDOK){Serial.println(" inaccessible");}
    else{ 
      
Serial.print(" transfert enregistrement ");
// transfert

  *IperiNum=          i;
  *IperiPerRefr=      *periPerRefr;
  *IperiPitch=        *periPitch;
  *IperiLastVal=      *periLastVal;
  *IperiAlim=         *periAlim;
  packDate(IperiLastDateIn,periLastDate+2);
  packDate(IperiLastDateOut,periLastDate+2);
  packDate(IperiLastDateErr,periLastDate+2);
  *IperiErr=          0;
  memcpy(IperiNamer,periNamer,PERINAMLEN);
  memcpy(IperiVers,periVers,3);
  memcpy(IperiMacr,periMacr,6);
  memcpy(IperiIpAddr,periIpAddr,4);
  *IperiSwNb=        *periSwNb;
  *IperiSwCde=       *periSwCde;
  memset(IperiSwMode,0x00,4);
for(int k=0;k<MAXSW;k++){
  IperiSwPulse[k]=  periSwPulse[k];
  IperiSwPulseCurr[k]= 0;
  IperiSwPulseCtl[k]= 0;
  IperiSwPulseTrig[k]= 0;
}
  *Idispo=            1;
  *IperiProg=         0;
  *IperiDetNb=        *periDetNb;
  *IperiDetVal=       0;

Serial.print(" save ");

// nouveau fichier et save

    memcpy(periNewFile,periFile,7);periNewFile[6]='N';periNewFile[7]='\0';
    if(sdOpen(FILE_WRITE,&fperi,periNewFile)==SDKO){
      Serial.print(" SDOPEN WRITE KO");}
    else{fperi.seek(0);
      for(int j=0;j<periNewLen;j++){fperi.write(periRecNew[j]);}
      fperi.close();
     }
    Serial.println();
    }
  }
*/
}



/* ---------- maintenance fichiers peri ---------- */

void periMaintenance()
{
/*  
for(int i=0;i=NBPERIF;i++){
  periLoad(i);
  memset(periAnalDestDet,0x00,MAXDET);memset(periAnalMemo,0x00,MAXDET);memset(periAnalCb,0x00,MAXDET);
  memset(periDigitDestDet,0x00,NBANST);memset(periDigitMemo,0x00,NBANST);memset(periDigitCb,0x00,NBANST);
  periSave(i,PERISAVESD);
}
while(1){}
*/
/*
for(i=0;i<50;i++){
  Serial.print(i);if(i<10){Serial.print(" ");}Serial.print(" ");
  for(j=0;j<10;j++){Serial.print((char)fonctions[i*10+j]);}
  Serial.println();if((strstr(fonctions+i*10,"last")-(fonctions+i*10))==0){i=100;}}
while(1){} 
*/
/* 
    if(SD.exists("fdhisto.txt")){SD.remove("fdhisto.txt");}
    while(1){}
*/    
/*  après changement de format : chargement cache puis remove, init (mettre en // les variabes à ne pas modifier) et save 
    Serial.println("conv en cours");
    for(int h=0;h<NBPERIF;h++){
      periLoad(h);periRemove(h);periInitVar();periSave(h,1);
    }
    Serial.println("terminé");
    while(1){}
*/
/*  création des fichiers de périphériques  
    periInit();
    for(int i=1;i<=NBPERIF;i++){
      periRemove(i);
      periCacheStatus[i]=0x01;
      periCur=i;periSave(i,1);
    }
    while(1){}
*/
/* //correction de valeurs dans les fichiers de périphériques
    Serial.print("correction en cours...");
    periInit();
    for(i=1;i<=NBPERIF;i++){periSave(i,PERISAVESD);}
    Serial.println("terminé");
    while(1){}
*/
/*  init timers
 
    timersInit();
    timersSave();
    while(1){}; 
*/
/*  init detecteurs
    memDetInit();
    memDetSave();
    while(1){};
*/
/*  modif config

    configInit();
    configPrint();
    configLoad();
    memset(thermolowenable,0x00,NBTHERMO*sizeof(bool));
    memset(thermohighenable,0x00,NBTHERMO*sizeof(bool));
    configPrint();
    configSave();
    while(1){};
*/
}

/*********** remotes ************/

void remPrint(uint8_t num)
{
  Serial.print("   ");Serial.print(num);Serial.print("/");Serial.print(remoteT[num].num);Serial.print(" ");
  //Serial.print(remoteT[num].detec);Serial.print(" ");
  //Serial.print(remoteT[num].enable);Serial.print(" ");
  //Serial.print(remoteT[num].oldenable);Serial.print(" ");
  Serial.print(remoteT[num].peri);Serial.print(" ");
  Serial.print(remoteT[num].sw);Serial.print(" ");
}

void remotePrint()
{
  for(uint8_t num=0;num<NBREMOTE;num++){
    Serial.print(num+1);Serial.print(" ");if(num<10){Serial.print(" ");}Serial.print(remoteN[num].nam);
    for(int nr=LENREMNAM-strlen(remoteN[num].nam);nr>=0;nr--){Serial.print(" ");}
    //Serial.print(remoteN[num].onoff);Serial.print("/");Serial.print(remoteN[num].newonoff);Serial.print(" ");
    //Serial.print(remoteN[num].oldenable);Serial.print("/");Serial.print(remoteN[num].newenable);Serial.print(" ");
    for(uint8_t numd=0;numd<MAXREMLI;numd++){
      if(remoteT[numd].num==num+1){
        remPrint(numd);
      }
    }
    Serial.println();
  }
  Serial.println();
}

int remLoad(const char* remF,uint16_t remL,char* remA)
{
    Serial.print("Load ");Serial.print(remF);Serial.print(" ");
    if(sdOpen(remF,&fremote)==SDKO){Serial.println(" KO");return SDKO;}
    for(uint16_t i=0;i<remL;i++){*(remA+i)=fremote.read();}              
    fremote.close();Serial.println(" OK");
    return SDOK;
}

int remSave(const char* remF,uint16_t remL,const char* remA)
{
    Serial.print("Save ");Serial.print(remF);
    if(sdOpen(remF,&fremote)==SDKO){Serial.println(" KO");return SDKO;}
    fremote.seek(0);
    for(uint16_t i=0;i<remL;i++){fremote.write(*(remA+i));}
    fremote.close();Serial.println(" OK");
    return SDOK;
}

void remInit()
{      
    for(int nb=0;nb<MAXREMLI;nb++){
      remoteT[nb].num=0;
      //remoteT[nb].detec=0;
      //remoteT[nb].deten=0;
      //remoteT[nb].enable=0;
      remoteT[nb].peri=0;
      remoteT[nb].sw=0;
      remoteT[nb].multRem=0;
    }

    for(int nb=0;nb<NBREMOTE;nb++){
      memset(remoteN[nb].nam,'\0',LENNOM);
      remoteN[nb].enable=0;
      remoteN[nb].detec=0;
      remoteN[nb].multRem=0;
      //remoteN[nb].newenable=0;
      //remoteN[nb].onoff=0;
      //remoteN[nb].newonoff=0;
    }
}

void remoteLoad()
{
  remLoad(REMOTETFNAME,remoteTlen,remoteTA);
  remLoad(REMOTENFNAME,remoteNlen,remoteNA);
  //remotePrint();
}

void remoteSave()
{
  for(uint8_t nb=0;nb<NBREMOTE;nb++){
    if(remoteN[nb].nam[0]=='\0'){
      //remoteN[nb].onoff=0;
      //remoteN[nb].newonoff=0;
      remoteN[nb].enable=0;
      remoteN[nb].detec=0;
      remoteN[nb].multRem=0;
      //remoteN[nb].newenable=0;
    }
  }
  remSave(REMOTETFNAME,remoteTlen,remoteTA);
  remSave(REMOTENFNAME,remoteNlen,remoteNA);
  //remotePrint();
}

void remoteNPlus(int plus)
{
// ajoute "plus" entrées à remoteN 
  
struct Remote remoteNNew[NBREMOTE+plus];
char*  remoteNANew=(char*)&remoteNNew;
unsigned long   remoteNlenNew=(sizeof(Remote))*(NBREMOTE+plus);

  remoteLoad();
  
  for(int nb=0;nb<NBREMOTE;nb++){
      memcpy(remoteNNew[nb].nam,remoteN[nb].nam,LENNOM);
      remoteNNew[nb].enable=remoteN[nb].enable;
      remoteNNew[nb].detec=remoteN[nb].detec;
      //remoteNNew[nb].newenable=remoteN[nb].newenable;
      //remoteNNew[nb].onoff=remoteN[nb].onoff;
      //remoteNNew[nb].newonoff=remoteN[nb].newonoff;
  }
  for(int nb=NBREMOTE;nb<NBREMOTE+plus;nb++){
      memset(remoteNNew[nb].nam,'\0',LENNOM);
      remoteNNew[nb].enable=0;
      remoteNNew[nb].detec=0;
      //remoteNNew[nb].newenable=0;
      //remoteNNew[nb].onoff=0;
      //remoteNNew[nb].newonoff=0;
  }
  
  sdRemove(REMOTENFNAME,&fremote);
  remSave(REMOTENFNAME,remoteNlenNew,remoteNANew);
}

void remoteTConvert()
{
  // ajout/modif champs dans la structure
  // faire une copie de la carte SD
  // créer la nouvelle structure dans const.h (newSwRemote dans l'exemple)
  // (dupliquer la structure et ajouter les nouvelles variables)
  // dans la boucle de recopie des lignes initialiser les nouvelles variables
  // faire tourner frontal2 avec la ligne remoteConvert décommentée
  // la recommenter
  // mettre à jour la structure "normale" dans const.h et supprimer la nouvelle

  // pour la migration de la carte serveur RUN, le plus simple est de copier le fichier tablerem depuis la carte DEV
/*
  struct NewSwRemote newRemoteT[MAXREMLI];
  char*  newRemoteTA=(char*)&newRemoteT;
  unsigned long   newRemoteTlen=(sizeof(NewSwRemote))*MAXREMLI;

  if(remLoad(REMOTETFNAME,remoteTlen,remoteTA)!=SDOK){while(1){yield();}}

  for(uint8_t i=0;i<MAXREMLI;i++){
    newRemoteT[i].num=remoteT[i].num ;
    newRemoteT[i].detec=remoteT[i].detec ;
    newRemoteT[i].deten=remoteT[i].deten ;
    newRemoteT[i].oldenable=remoteT[i].oldenable ;
    newRemoteT[i].peri=remoteT[i].peri ;
    newRemoteT[i].sw=remoteT[i].sw ;
    newRemoteT[i].butModel=0 ;
    newRemoteT[i].multRem=0 ;
  }

  fremote.remove();

  if (!fremote.open(REMOTETFNAME, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println(" create failed");
    return;
  }

  fremote.close();
  if(remSave(REMOTETFNAME,newRemoteTlen,newRemoteTA)!=SDOK){while(1){yield();}}
*/
  Serial.println("conversion remoteT terminée");
}

void remoteNConvert()
{
  // ajout/modif champs dans la structure
  // faire une copie de la carte SD
  // créer la nouvelle structure dans const.h (newRemote dans l'exemple)
  // (dupliquer la structure et ajouter les nouvelles variables)
  // dans la boucle de recopie des lignes initialiser les nouvelles variables
  // faire tourner frontal2 avec la ligne remoteNConvert décommentée
  // la recommenter
  // mettre à jour la structure "normale" dans const.h et supprimer la nouvelle

  // pour la migration de la carte serveur RUN, le plus simple est de copier le fichier noms_rem depuis la carte DEV

  struct newRemote newRemoteN[NBREMOTE];
  char*  newRemoteNA=(char*)&newRemoteN;
  unsigned long   newRemoteNlen=(sizeof(newRemote))*NBREMOTE;

  if(remLoad(REMOTENFNAME,remoteNlen,remoteNA)!=SDOK){while(1){yield();}}

  for(uint8_t i=0;i<NBREMOTE;i++){
    memcpy(&newRemoteN[i].nam,&remoteN[i].nam,LENREMNAM) ;
    newRemoteN[i].enable=remoteN[i].enable ;
//    newRemoteN[i].onoff=remoteN[i].onoff ;
//    newRemoteN[i].newonoff=remoteN[i].newonoff ;
//    newRemoteN[i].oldenable=remoteN[i].oldenable ;
//    newRemoteN[i].newenable=remoteN[i].newenable ;
//    newRemoteN[i].multRem=0 ;
    newRemoteN[i].detec=remoteN[i].detec ;
    newRemoteN[i].butModel=remoteN[i].butModel ;
    newRemoteN[i].osTimStat=0;
    newRemoteN[i].osType=0;
    memset(newRemoteN[i].osDurat,'0',6);
    memset(newRemoteN[i].osRemT,'0',6);
    memset(newRemoteN[i].osEndDate,'0',15);
  }

  fremote.remove();

  if (!fremote.open(REMOTENFNAME, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println(" create failed");
    return;
  }

  fremote.close();
  if(remSave(REMOTENFNAME,newRemoteNlen,newRemoteNA)!=SDOK){while(1){yield();}}

  Serial.println("conversion remoteN terminée");
}

/*********** timers ************/

void timersPrint()
{
  for(uint8_t nt=0;nt<NBTIMERS;nt++){
    Serial.print("   ");Serial.print(nt);Serial.print("/");Serial.print(timersN[nt].detec);Serial.print(" ");
    Serial.print(timersN[nt].nom);Serial.print(" ");Serial.print(timersN[nt].enable);Serial.print(" ");
    Serial.print(timersN[nt].perm);Serial.print(" ");Serial.print(timersN[nt].curstate);Serial.print(" ");
    Serial.print(timersN[nt].cyclic);Serial.print(" ");Serial.print(timersN[nt].forceonoff);Serial.print(" ");
    Serial.print(timersN[nt].hdeb);Serial.print(" ");Serial.print(timersN[nt].hfin);Serial.print(" ");
    for(int nd=7;nd>=0;nd--){Serial.print((char)(((timersN[nt].dw>>nd)&0x01)+48));}Serial.print(" "); // 0=tous 1-7
    Serial.print(timersN[nt].dhdebcycle);Serial.print(" ");Serial.println(timersN[nt].dhfincycle);
    Serial.print("last :");Serial.print(timersN[nt].dhLastStart);Serial.print(" ");Serial.print(timersN[nt].dhLastStop);
    Serial.print(" periode :");Serial.print(timersN[nt].dayPeriode);Serial.print(" ");Serial.println(timersN[nt].timePeriode);
  }
}

void timersInit()
{
  memset(timersN,0x00,timersNlen);
  for(int nt=0;nt<NBTIMERS;nt++){
    memset(timersN[nt].dhdebcycle,'0',16);
    memset(timersN[nt].dhfincycle,'9',16);
    memset(timersN[nt].dhLastStart,'0',16); 
    memset(timersN[nt].dhLastStop,'0',16);
  }
  
/*  for(uint8_t nt=0;nt<NBTIMERS;nt++){  
    timersN[nt].numdetec=0;
    memset(timersN[nt].nom,0x00,LENTIMNAM);
    memset(timersN[nt].heuredeb,0x00,7);
    memset(timersN[nt].heurefin,0x00,7);
    timersN[nt].cyclic=0;
    timersN[nt].enable=0; 
    timersN[nt].currstate=0;
    timersN[nt].forceonoff=0;
    timersN[nt].dw=0;
  }*/
}

int timersLoad()
{
    Serial.print("Load timers   ");
    if(sdOpen(TIMERSNFNAME,&ftimers)==SDKO){Serial.println(" KO");return SDKO;}
    ftimers.seek(0);
    for(uint16_t i=0;i<timersNlen;i++){*(timersNA+i)=ftimers.read();}             
    ftimers.close();Serial.println(" OK");
    return SDOK;
}

int timersSave()
{
    Serial.print("Save timers ");
    if(sdOpen(TIMERSNFNAME,&ftimers)==SDKO){Serial.println(" KO");return SDKO;}
    ftimers.seek(0);
    for(uint16_t i=0;i<timersNlen;i++){ftimers.write(*(timersNA+i));}             
    ftimers.close();Serial.println(" OK");
    return SDOK;
}

int timersConvert()
{
    timersInit();
    Serial.println("convert timers ");
    
    Serial.print("Load timers   ");
    if(sdOpen(TIMERSNFNAME,&ftimers)==SDKO){Serial.println(" KO");return SDKO;}
    ftimers.seek(0);
    for(uint8_t i=0;i<NBTIMERS;i++){
      for(uint16_t j=0;j<sizeof(TimersOld);j++){
        *(timersNA+i*sizeof(Timers)+j)=ftimers.read();}
      memset(timersN[i].dhLastStart,'0',16);
      memset(timersN[i].dhLastStop,'0',16);
      timersN[i].dayPeriode=0;
      memset(timersN[i].timePeriode,'0',7);
      if(i>7){
        memcpy(timersN[i].dhdebcycle,timersN[7].dhdebcycle,16);
        memcpy(timersN[i].dhfincycle,timersN[7].dhfincycle,16);
      }
    }
    ftimers.close();Serial.println(" OK");

    Serial.println("check");timersPrint();
    Serial.print("ok=CR ");
    char inp=' ';while(inp==' '){if(Serial.available()){inp=Serial.read();if(inp!=13){inp=' ';}}}
    Serial.println();
    //while(1){};

    ftimers.remove();    

    if (!ftimers.open(TIMERSNFNAME, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.print(TIMERSNFNAME);Serial.println(" create failed");ledblink(BCODESDCARDKO);}
    else {ftimers.close();Serial.println(" create OK");}

    timersSave();

    Serial.println("terminé");
    while(1){blink(1);}

}


/**************** thermometres ******************/

void subchp(bool enable,uint16_t state,uint16_t value,uint16_t offset,uint8_t det)
{
  Serial.print("   ");Serial.print(enable);Serial.print(" ");
  Serial.print((float)value/100);Serial.print(" ");Serial.print((float)offset/100);Serial.print(" ");
  Serial.print(det);Serial.print(" ");Serial.print(state);
}

void thermosPrint()
{
  for(uint8_t nt=0;nt<NBTHERMOS;nt++){
    Serial.print(nt);Serial.print(" ");Serial.print((char*)thermos[nt].nom);Serial.print(" ");Serial.print(thermos[nt].peri);Serial.print(" ");
    subchp(thermos[nt].lowenable,thermos[nt].lowstate,thermos[nt].lowvalue,thermos[nt].lowoffset,thermos[nt].lowdetec);
    subchp(thermos[nt].highenable,thermos[nt].highstate,thermos[nt].highvalue,thermos[nt].highoffset,thermos[nt].highdetec); 
    Serial.println();
  }
}

void thermosInit()
{
  for(uint8_t nt=0;nt<NBTHERMOS;nt++){
    memset(thermos[nt].nom,0x00,LENTHNAME);
    thermos[nt].peri=0;
    thermos[nt].lowenable=0;
    thermos[nt].highenable=0;
    thermos[nt].lowstate=0;
    thermos[nt].highstate=0;
    thermos[nt].lowvalue=0;
    thermos[nt].highvalue=0;
    thermos[nt].lowoffset=0;
    thermos[nt].highoffset=0;  
    thermos[nt].lowdetec=0;
    thermos[nt].highdetec=0;  
  }
}

int thermosLoad()
{
    Serial.print("Load thermos  ");
    if(sdOpen(THERMOSFNAME,&fthermos)==SDKO){Serial.println(" KO");return SDKO;}
    fthermos.seek(0);
    for(uint16_t i=0;i<thermoslen;i++){*(thermosA+i)=fthermos.read();}              
    fthermos.close();Serial.println(" OK");
    return SDOK;
}

int thermosSave()
{
    
    Serial.print("Save thermos ");
    if(sdOpen(THERMOSFNAME,&fthermos)==SDKO){Serial.println(" KO");return SDKO;}
    fthermos.seek(0);
    for(uint16_t i=0;i<thermoslen;i++){fthermos.write(*(thermosA+i));}             
    fthermos.close();Serial.println(" OK");
    return SDOK;
}


/************** détecteurs serveur **************/

int memDetLoad()
{
    Serial.print("Load detServ  ");
    if(sdOpen(MEMDETFNAME,&fmemdet)==SDKO){Serial.println(" KO");return SDKO;}
    fmemdet.seek(0);
    
    for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i]=fmemdet.read();Serial.print(' ');if(memDetServ[i]<16){Serial.print('0');}Serial.print(memDetServ[i],HEX);}    
    for(uint8_t i=0;i<NBDSRV;i++){
      for(uint8_t j=0;j<LENLIBDETSERV;j++){libDetServ[i][j]=fmemdet.read();}
    }

    for(uint8_t i=0;i<NBDSRV;i++){sourceDetServ[i]=fmemdet.read();}
    fmemdet.close();Serial.println(" OK");
    return SDOK;
}

int memDetSave()
{
    Serial.print("Save detServ ");
    if(sdOpen(MEMDETFNAME,&fmemdet)==SDKO){Serial.println(" KO");return SDKO;}
    
    fmemdet.seek(0);
    for(uint8_t i=0;i<MDSLEN;i++){fmemdet.write(memDetServ[i]);}
    for(uint8_t i=0;i<NBDSRV;i++){
      for(uint8_t j=0;j<LENLIBDETSERV;j++){fmemdet.write(libDetServ[i][j]);}}
    for(uint8_t i=0;i<NBDSRV;i++){fmemdet.write(sourceDetServ[i]);}
    
    fmemdet.close();Serial.println(" OK");
    return SDOK;  
}

void memDetInit()
{
    memset(&libDetServ,0x00,LENLIBDETSERV*NBDSRV);

    memcpy(&libDetServ[31][0],"HC      ",LENLIBDETSERV-1);
    memcpy(&libDetServ[30][0],"HCfin   ",LENLIBDETSERV-1);
    memcpy(&libDetServ[29][0],"nuit    ",LENLIBDETSERV-1);
    memcpy(&libDetServ[28][0],"eau     ",LENLIBDETSERV-1);
    memcpy(&libDetServ[27][0],"chauffe ",LENLIBDETSERV-1);
    memcpy(&libDetServ[26][0],"chauffeF",LENLIBDETSERV-1);    
    memcpy(&libDetServ[25][0],"chauffeD",LENLIBDETSERV-1);
    memcpy(&libDetServ[24][0],"LL      ",LENLIBDETSERV-1);    
    memcpy(&libDetServ[23][0],"LL_F    ",LENLIBDETSERV-1);    
    memcpy(&libDetServ[22][0],"LL_D    ",LENLIBDETSERV-1);
    memcpy(&libDetServ[21][0],"eauW    ",LENLIBDETSERV-1);              
    memcpy(&libDetServ[20][0],"eauW_F  ",LENLIBDETSERV-1);                  
    memcpy(&libDetServ[19][0],"eauW_D  ",LENLIBDETSERV-1);              
    memcpy(&libDetServ[21][0],"ctserv  ",LENLIBDETSERV-1);              
    memcpy(&libDetServ[20][0],"ctserv_F",LENLIBDETSERV-1);                  
    memcpy(&libDetServ[19][0],"ctserv_D",LENLIBDETSERV-1);              
    memcpy(&libDetServ[21][0],"eauE    ",LENLIBDETSERV-1);              
    memcpy(&libDetServ[20][0],"eauE_F  ",LENLIBDETSERV-1);                  
    memcpy(&libDetServ[19][0],"eauE_D  ",LENLIBDETSERV-1);                          
}

void memDetPrint()
{
    dumpfield((char*)memDetServ,MDSLEN);Serial.println(" ");
    for(uint8_t i=0;i<NBDSRV;i++){
      Serial.print(i);if(i<10){Serial.print(" ");}Serial.print("  ");
      Serial.print(mDSval(i));Serial.print("  ");
      for(uint8_t j=0;j<LENLIBDETSERV;j++){Serial.print(libDetServ[i][j]);}Serial.print(" ");
      if(sourceDetServ[i]<16){Serial.print('0');};Serial.println(sourceDetServ[i],HEX);
    }
    Serial.println();
}

int memDetConvert()
{
/*  Conversion nombre de mds ; le paramètres NBDSRV et MDSLEN doivent avoir les nouvelles valeurs ;
    Les variables mdsl et mbdsrv les anciennes valeurs
*/

    int mdsl=4;     // longueur ancienne 
    int nbdsrv=32;  // nombre ancien    

    Serial.println("convert detServ ");

    Serial.print("Load detServ  ");
    if(sdOpen(MEMDETFNAME,&fmemdet)==SDKO){Serial.println(" KO");return SDKO;}
    fmemdet.seek(0);
    
    for(uint8_t i=0;i<mdsl;i++){memDetServ[i]=fmemdet.read();Serial.print(' ');if(memDetServ[i]<16){Serial.print('0');}Serial.print(memDetServ[i],HEX);}    
    for(uint8_t i=0;i<nbdsrv;i++){
      for(uint8_t j=0;j<LENLIBDETSERV;j++){libDetServ[i][j]=fmemdet.read();}
    }

    for(uint8_t i=0;i<nbdsrv;i++){sourceDetServ[i]=fmemdet.read();}
    fmemdet.close();Serial.println(" OK");

    fmemdet.remove();    

    if (!fmemdet.open(MEMDETFNAME, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.print(MEMDETFNAME);Serial.println(" create failed");ledblink(BCODESDCARDKO);}
    else {fmemdet.close();Serial.println(" create OK");}

    memDetSave();

    Serial.println("terminé");
    while(1){blink(1);}
}


/************** memos **************/

int memosLoad(int m)        // si <0 tout le fichier
{
    Serial.print("Load Memos ");Serial.print(m);Serial.print(" ");
    if(sdOpen(MEMOSFNAME,&fmemos)==SDKO){Serial.println(" KO");return SDKO;}
    uint16_t sk=0;if(m>=0){sk=m*LMEMO;}
    fmemos.seek(0);fmemos.seek(sk);    

    uint16_t lm=LMEMO;if(m<0){lm*=NBMEMOS;}
    for(uint16_t i=0;i<lm;i++){memosTable[sk+i]=fmemos.read();}    
    
    fmemos.close();Serial.println(" OK");
    return SDOK;
}

int memosFind()                 // return -1 full 
{   
    int m=-1;
    for(uint8_t i=0;i<NBMEMOS;i++){
      if(memosTable[i*LMEMO]=='\0'){m=i;break;}}
    return m;
}

int memosSave(int m)        // si <0 tout le fichier
{
    Serial.print("Save Memos ");
    if(sdOpen(MEMOSFNAME,&fmemos)==SDKO){Serial.println(" KO");return SDKO;}
    if(m<0){
      fmemos.seek(0);
      for(uint16_t i=0;i<LMEMO*NBMEMOS;i++){fmemos.write(memosTable[i]);}}
    else {fmemos.seek(m*LMEMO);fmemos.write(memosTable+m*LMEMO);}
    fmemos.close();Serial.println(" OK");
    return SDOK;  
}

void memosInit()
{
    memset(memosTable,0x00,LMEMO*NBMEMOS);
}

void memosPrint()
{
  memosLoad(-1);
  dumpstr(memosTable,300);
  /*
    for(uint8_t i=0;i<NBMEMOS;i++){
      Serial.print(i);Serial.print("  ");
      for(uint8_t j=0;j<LMEMO;j++){
        Serial.print(memosTable[i*LMEMO+j]);}
      Serial.println();
    }
    Serial.println();
*/
}

/********************* génération SD card *************************/

void sdCardGen()
{

// config

  configInit();
  configSave();

// perifiles

  for(int i=1;i<=NBPERIF;i++){
    periInit();
    periSave(i,1);
  }

// détecteurs serveur

  memDetInit();
  memDetSave();

// remotes

  remInit();
  remoteSave();

// timers

  timersInit();
  timersSave();

// thermos

  thermosInit();
  thermosSave();

}
