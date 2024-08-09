#include <Arduino.h>
#include <SPI.h>      //bibliothèqe SPI pour W5100
#include <Ethernet.h> //bibliothèque W5x00 Ethernet
#include <EthernetUdp.h>
#include <Wire.h>     //biblio I2C pour RTC 3231
#include "ds3231.h"
#include <shconst2.h>
#include <shmess2.h>
#include <shutil2.h>
#include <MemoryFree.h>
#include "FreeStack.h"
#include "const.h"
#include "utilether.h"
#include "periph.h"
#include "peritalk.h"
#include "peritable.h"
#include "pageshtml.h"
#include "utilhtml.h"

//#define DEBUG_ON          // ajoute des delay(20) pour obtenir les sorties sur le terminal

//#define _AVEC_AES
#ifdef _AVEC_AES
#include "aes.h"      //encryptage AES
#define KEY {0x2d,0x80,0x17,0x18,0x2a,0xb0,0xd4,0xa8,0xad,0xf9,0x17,0x8a,0x0b,0xD1,0x51,0x3e}
#define IV  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f}
struct AES_ctx ctx;
uint8_t key[16]=KEY,iv[16]=IV;
uint8_t chaine[16+1]={0}; // chaine à encrypter/décrypter ---> void xcrypt()
#endif // _AVEC_AES

extern "C" {
 #include "utility/w5100.h"
}

#define MAXTPS 1                    // nbre instances pour TCP périfs 
                                    // plusieurs permet un .stop() rapide
                                    // l'envoi de la réponse se fait pendant que le frontal tourne
                                    // si saturation des instances, .stop() de l'instance en cours
  EthernetClient cli_a[MAXTPS];     // instances serveur de periphériques 

uint8_t tPS=0;                      // pointeur prochaine instance cli_a à utiliser
unsigned long tPSStop[MAXTPS];      // heure fin d'usage des instances pour faire .stop()
char ab;                            // protocole et type de la connexion en cours
                                    // 'a' TCP periTable 'b' TCP remote 'u' UDP

//  EthernetClient cli_a;             // instance serveur de periphériques et browser configuration
  EthernetClient cli_b;             // instance du serveur remote 
  EthernetClient cli_c;             // instance du serveur config (browsers)
  EthernetClient cliext;            // instance client serveurs externes  

  char udpData[UDPBUFLEN];          // buffer paquets UDP
  uint16_t udpDataLen;              // taille paquet contenu
    
  EthernetUDP Udp1;                 // serveur Udp 1
  EthernetUDP Udp2;                 // serveur Udp 2
  EthernetUDP* udp[2];
  uint8_t udpNb;
  uint8_t udpSockIndex;

extern char sssa[MAX_SOCK_NUM+2];   // valeurs alpha pour sockets status
extern char prevsssa[MAX_SOCK_NUM+2];

/* ---------- config frontal ---------- */

char configRec[CONFIGRECLEN];       // enregistrement de config  

/* pointeurs dans l'enregitrement de config */

  byte*     mac;              // mac adresse server
  byte*     localIp;          // ip  adresse server
  uint16_t* perifPort;        // port perifs
  uint16_t* browserPort;      // port config
  uint16_t* remotePort;       // port remote
  uint16_t  udpPort[2];       // 2 ports udp
  char*     serverName;       // nom server
  char*     peripass;         // mot de passe périphériques
  char*     ssid;             // liste des ssid pour peripherique2
  char*     passssid;         // liste des password ssid pour peripherique2
  uint8_t*  ssid1;            // n° 1er ssid à essayer pour peripherique2 (1-n)
  uint8_t*  ssid2;            // n° 2nd ssid à essayer pour peripherique2 (1-n)
  uint8_t*  concMac;          // (table concentrateurs udp) macaddr concentrateur (5 premiers caractères valides le 6ème est le numéro dans la table)
  byte*     concIp;           // (table concentrateurs udp) adresse IP concentrateur
  uint16_t* concPort;         // (table concentrateurs udp) port concentrateur
  uint8_t*  concRx;           // (table concentrateurs udp) radio Addr length
  uint16_t* concChannel;      // (table concentrateurs udp) n° channel nrf utilisé par le concentrateur
  uint16_t* concRfSpeed;      // (table concentrateurs udp) RF_Speed concentrateur
  uint8_t*  concNb;           // numéro de concentrateur udp pour config concentrateurs udp et périphériques udp
  uint8_t*  concPeriParams;   // peri params 0=keep 1=new
  float*    thFactor;
  float*    thOffset;
  float*    vFactor;
  float*    vOffset;
  byte*     periRxAddr;

  char*     usrnames;         // usernames
  char*     usrpass;          // userpass
  unsigned long* usrtime;     // user cx time
  unsigned long* usrpretime;  // user cx time précédent
  uint16_t* toPassword;       // Délai validité password en sec !
  unsigned long* maxCxWt;     // Délai WD TCP
  unsigned long* maxCxWu;     // Délai WD UDP
  uint8_t*  openSockScan;     // Délai scan sockets secondes
  uint8_t*  openSockTo;       // TO open sockets secondes

  char* mailFromAddr=nullptr; // Adresse exp mail
  char* mailPass=nullptr;     // mot de passe exp
  char* mailToAddr1;          // Adresse dest mail 1
  char* mailToAddr2;          // Adresse dest mail 2
  uint16_t* periMail1;        // N° perif mail 1
  uint16_t* periMail2;        // N° perif mail 2

  char* thermoPrev;           // antériorité pour calculs mini/maxi thermos
  char* thermoLastScan;       // date/heure last scan
  char* thermoLastPrev;       // antériorité last scan
  char* thermoLastBeg;        // date/heure dans l'histo

  byte* configBegOfRecord;
  byte* configEndOfRecord;

  char  thermoCurPrev[16];    // antériorité courante


  bool mailEnable=FAUX;     // interdit les mails si config n'est pas chargé et periCacheLoad n'est pas terminé 

  bool    periPassOk=FAUX;  // contrôle du mot de passe des périphériques
  int     usernum=-1;       // numéro(0-n) de l'utilisateur connecté (valide durant commonserver)   

EthernetServer* periserv=nullptr;             // serveur perif
EthernetServer* browserserv=nullptr;          // serveur browser
EthernetServer* remoteserv=nullptr;           // serveur remote
  
  uint8_t   remote_IP[4]={0,0,0,0};           // periserver
  uint8_t   remote_IP_cur[4]={0,0,0,0};       // périphériques periserver
  byte      remote_MAC[6]={0,0,0,0,0,0};      // periserver
  uint16_t  remote_Port_Udp=0;                   

  int8_t  numfonct[NBVAL];                    // les fonctions trouvées  (au max version 1.1k 23+4*57=251)
  
  const char*   fonctions="per_temp__peri_pass_username__password__user_ref__to_passwd_per_refr__peri_tofs_switchs___deco______dump_his__hist_sh_p_data_save_data_read_peri_tst__peri_cur__peri_raz__perifonc__data_na___accueil___peri_tabledata_storedata_mail_data_par__peri_inp__dispo_____dispo_____dispo_____remote____testhtml__timersctl_peri_t_sw_peri_otf__p_inp1____p_inp2____peri_sw___antim_____antim_cb__antim_hh__antim_vv__antimcfg__dsrv_init_mem_dsrv__ssid______passssid__usrname___usrpass___cfgserv___null_fnct_percocfg__peripcfg__ethcfg____remotecfg_remote_c__remote_o__dispo_____mailcfg___thparams__thermoshowthermoscfgtim_ctl___tim_name__tim_det___tim_hdf___tim_chkb__timershtmldsrvhtml__libdsrv___periline__done______peri_ana__rul_ana___rul_dig___rul_init__favicon___last_fonc_";
  
  /*  nombre fonctions, valeur pour accueil, data_save_ fonctions multiples etc */
  int     nbfonct=0,faccueil=0,fdatasave=0,fdatana=0,fperiSwVal=0,fperiDetSs=0,fdone=0,fpericur=0,fperipass=0,fpassword=0,fusername=0,fuserref=0,fperitst=0,ffavicon=0,fthermoshow=0,fdatamail=0,fdatapar=0;
  int8_t  numfonc;                     // copie numfonct[i]
  char    valeurs[LENVALEURS];         // les valeurs associées à chaque fonction trouvée
  uint16_t nvalf[NBVAL];               // offset dans valeurs[] des valeurs trouvées (séparées par '\0')
  char*   valf;                        // pointeur dans valeurs en cours de décodage
  char    libfonctions[NBVAL*2];       // les 2 caractères de fin des noms de fonctions
  int     nbreparams=0;                // 
  int     what=0;                      // ce qui doit être fait après traitement des fonctions (0=rien)
  uint32_t loopCnt=0;
  uint8_t scanCnt=0;
  char*   mailData;                    // pointeur chaine à transmettre en provenance d'un périf
  char*   jsonData;

#define HTTPCDLENGTH 6
  const char*   httpCdes="GET   POST  \0";   // commandes traitées par le serveur
  char          strHisto[RECCHAR]={0};       // buffer enregistrement histo SD
  const char*   strHistoEnd="<br>\r\n\0";
  char          buf[12];
  long          fhsize;                      // remplissage fhisto

  char bufServer[LBUFSERVER];          // buffer entrée/sortie dataread/save

  float         oldth=0;               // pour prev temp DS3231
  unsigned long temptime=0;            // last millis() pour temp
#define PTEMP 120                      // secondes
  uint32_t      pertemp=PTEMP;         // période ech temp sur le serveur
  uint16_t      perrefr=0;             // periode rafraichissement de l'affichage

  unsigned long lastcxt=0;             // last TCP server connection for watchdog
  unsigned long lastcxu=0;             // last UDP server connection for watchdog 
  unsigned long last_shscksta=0;       // last 'showSocketsStatus()' 
   
#define WDSD    "W"                    // Watchdog record
#define TCPWD   "T"                    // TCP watchdog event
#define UDPWD   "U"                    // UDP watchdog event
#define HALTREQ "H"                    // Halt request record
#define TEMP    "M"                    // Temp record
#define RESET   "R"                    // Reset record
#define UBOOT   "B"                    // User Request Boot record
// reservés a,u,b,c pour les serveurs
  unsigned long cxtime=0;              // durée connexion client
  unsigned long remotetime=0;          // mesure scans remote
  unsigned long oneShotRemTime=100;    // last millis pour one_shot_timers des remotes
  unsigned long srvdettime=0;          // mesure scans détecteurs
  unsigned long timerstime=300;        // last millis pour timers (désynchro)
  uint8_t cyclicTimersState[NBTIMERS];                // état selon cycle
  unsigned long unixCyclicTimersCurDate[NBTIMERS];    // prochaine date à laquelle l'état du timer doit changer  
  unsigned long unixCyclicTimersOnState[NBTIMERS];    // accélérateur calcul
  unsigned long unixCyclicTimersOffState[NBTIMERS];   // accélérateur calcul
  unsigned long antimerstime=0;        // last millis pour anTimers
#define POSREMOTE 1                    // secondes
  uint32_t  perOSR=POSREMOTE;          // période scan one_shot_timers
#define PTIMERS 1                      // secondes
  uint32_t  pertimers=PTIMERS;         // période scan timers 
  unsigned long thermosTime=400;       // last millis pour thermos
#define PTHERMOS 4                     // secondes
  uint32_t  perThermos=PTHERMOS;       // période ctle thermos
#define PANTIMERS 2                    // secondes
  uint32_t  perAnTimers=PANTIMERS;     // période ctle anTimers
  unsigned long datetime=0;            // last millis() pour date 
#define PDATE 3600*24                  // secondes
  unsigned long perdate=PDATE;         // période ctle date & perisave general
  
  int   stime=0;int mtime=0;int htime=0;
  unsigned long  curdate=0;

/* image mémoire détecteurs du serveur */

  uint8_t   memDetServ[MDSLEN]; //=0x00000000;    // image mémoire NBDSRV détecteurs (32)  
  char      libDetServ[NBDSRV][LENLIBDETSERV];
  char      mdsSrc[]=" PRHT";
  uint16_t  sourceDetServ[NBDSRV];   // qui actionne le detServ (sssnnnnnnnn ss type 000, P 001 perif, R 010 remote, H 011 thermos, T 100 timers / nnnnnnnn n°)
  uint8_t   mDSmaskbit[NBDSRV*MDSLEN];
  /*
  ={0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                       0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                       0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                       0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};
  uint32_t  mDSmaskneg[]={0xfffffffe,0xfffffffd,0xfffffffb,0xfffffff7,0xffffffef,0xffffffdf,0xffffffbf,0xffffff7f,
                       0xfffffeff,0xfffffdff,0xfffffbff,0xfffff7ff,0xffffefff,0xffffdfff,0xffffbfff,0xffff7fff,
                       0xfffeffff,0xfffdffff,0xfffbffff,0xfff7ffff,0xffefffff,0xffdfffff,0xffbfffff,0xff7fffff,
                       0xfeffffff,0xfdffffff,0xfbffffff,0xf7ffffff,0xefffffff,0xdfffffff,0xbfffffff,0x7fffffff};
  */
  uint8_t  bakDetServ[MDSLEN];

void iniDetServ()
{
  if(MDSLEN*8 != NBDSRV){Serial.print("MDSLEN invalide");while(1){ledblink(BCODESYSERR);}}
  memset(memDetServ,0x00,MDSLEN);
  memset(mDSmaskbit,0x00,NBDSRV*MDSLEN);
  memset(libDetServ,0x00,LENLIBDETSERV*NBDSRV);
  memset(sourceDetServ,0x00,NBDSRV*sizeof(uint16_t));

  byte curMask[MDSLEN];memset(curMask,0x00,MDSLEN);curMask[0]=0x01;
  for(uint8_t i=0;i<NBDSRV;i++){
    for(uint8_t j=0;j<MDSLEN;j++){
      mDSmaskbit[i*MDSLEN+j]=curMask[j];
    }
    for(uint8_t j=0;j<MDSLEN;j++){
      if(curMask[j]==0x80){
        curMask[j]=0x00;
        if(j<(MDSLEN-1)){curMask[j+1]=0x01;}
        break;}
      else curMask[j]<<=1;
    }
  }
  /*
  dumpstr((char*)mDSmaskbit,NBDSRV*MDSLEN);
  Serial.println();
  uint32_t ref[]=     {0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                       0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                       0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                       0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};
  dumpstr((char*)ref,NBDSRV*MDSLEN);
  */
}

  char onoff[]={'O','N','\0','O','F','F','\0'}; // poolperif

/*  enregistrement de table des périphériques ; un fichier par entrée
    (voir periInit() pour l'ordre physique des champs + periSave et periLoad=
*/
  char      periRec[PERIRECLEN];                // 1er buffer de l'enregistrement de périphérique
  char      periCache[PERIRECLEN*(NBPERIF+1)];  // cache des périphériques - la ligne supplémentaire est utilisée par periCheck()
  bool      periCacheStatus[(NBPERIF+1)];       // indicateur de validité du cache d'un périph  (vaut CACHEISFILE si cache==fichier)
  
  uint16_t  periCur=0;                      // Numéro du périphérique courant
  uint16_t  periSrc=0;                      // Numéro du périphérique source pour copy rules

  uint16_t* periNum;                        // ptr ds buffer : Numéro du périphérique courant
  uint32_t* periPerRefr;                    // ptr ds buffer : période maximale d'accès au serveur
  uint16_t* periPerTemp;                    // ptr ds buffer : période de lecture tempèrature
  int16_t*  periPitch_;                     // ptr ds buffer : variation minimale de température pour datasave
  int16_t*  periLastVal_;                   // ptr ds buffer : dernière valeur de température  
  int16_t*  periAlim_;                      // ptr ds buffer : dernière tension d'alimentation
  char*     periLastDateIn;                 // ptr ds buffer : date/heure de dernière réception
  char*     periLastDateOut;                // ptr ds buffer : date/heure de dernier envoi  
  char*     periLastDateErr;                // ptr ds buffer : date/heure de derniere anomalie com
  int8_t*   periErr;                        // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
  char*     periNamer;                      // ptr ds buffer : description périphérique
  char*     periVers;                       // ptr ds buffer : version logiciel du périphérique
  char*     periModel;                      // ptr ds buffer : model du périphérique
  byte*     periMacr;                       // ptr ds buffer : mac address 
  byte*     periIpAddr;                     // ptr ds buffer : Ip address
  uint16_t* periPort;                       // ptr ds buffer : port periph server
  byte*     periSwNb;                       // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
  byte*     periSwCde;                      // ptr ds buffer : état/cde des switchs
  byte*     periInput;                      // ptr ds buffer : table des règles switchs           
  uint32_t* periSwPulseOne;                 // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
  uint32_t* periSwPulseTwo;                 // ptr ds buffer : durée pulses sec OFF(mode astable)
  uint32_t* periSwPulseCurrOne;             // ptr ds buffer : temps courant pulses ON
  uint32_t* periSwPulseCurrTwo;             // ptr ds buffer : temps courant pulses OFF
  byte*     periSwPulseCtl;                 // ptr ds buffer : mode pulses
  byte*     periSwPulseSta;                 // ptr ds buffer : état clock pulses
  uint8_t*  periSwSta;                      // ptr ds buffer : état des switchs
  uint8_t*  periCfg;                        // ptr ds buffer : Conf peri (serveur,analog etc)
  byte*     periDetNb;                      // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
  byte*     periDetVal;                     // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
  int16_t*  periThOffset_;                  // ptr ds buffer : offset correctif sur mesure température
  int16_t*  periThmin_;                     // ptr ds buffer : mini last 24h 
  int16_t*  periThmax_;                     // ptr ds buffer : maxi last 24h
  int16_t*  periVmin_;                      // ptr ds buffer : alarme mini volts
  int16_t*  periVmax_;                      // ptr ds buffer : alarme maxi volts
  byte*     periDetServEn;                  // ptr ds buffer : 1 byte 8*enable detecteurs serveur
  byte*     periProtocol;                   // ptr ds buffer : protocole ('T'CP/'U'DP)
  uint8_t*  periUdpPortNb;                  // ptr ds buffer : n° instance udp utilisée
  uint16_t* periAnal;                       // ptr ds buffer : analog value
  uint16_t* periAnalLow;                    // ptr ds buffer : low analog value 
  uint16_t* periAnalHigh;                   // ptr ds buffer : high analog value 
  uint16_t* periAnalOffset1;                // ptr ds buffer : offset on adc value
  float*    periAnalFactor;                 // ptr ds buffer : factor to float for analog value
  float*    periAnalOffset2;                // ptr ds buffer : offset on float value
  uint8_t*  periAnalCb;                     // ptr ds buffer : 5 x (4 bits pour checkbox + 4 bits pour operation logique)
  uint8_t*  periAnalDestDet;                // ptr ds buffer : 5 x n° détect serveur
  uint8_t*  periAnalRefDet;                 // ptr ds buffer : 5 x n° détect serveur pour op logique (0xff si rien)
  int8_t*   periAnalMemo;                   // ptr ds buffer : 5 x n° mémo dans table mémos
  uint16_t* periAnalOut;                    // ptr ds buffer : consigne analogique poour le périf
  uint8_t*  periDigitCb;                    // ptr ds buffer : 5 x (4 bits pour checkbox + 4 bits pour operation logique)
  uint8_t*  periDigitDestDet;               // ptr ds buffer : 4 x n° détect serveur
  uint8_t*  periDigitRefDet;                // ptr ds buffer : 4 x n° détect serveur pour op logique (0xff si rien)
  int8_t*   periDigitMemo;                  // ptr ds buffer : 5 x n° mémo dans table mémos
  char*     periSsidNb;                     // ptr ds buffer : n° dernier ssid utilisé
  uint8_t*  periMessCnt;                    // ptr ds buffer : compteur d'exports du périf (après v2.9)

  int8_t    periMess;                       // code diag réception message (voir MESSxxx shconst.h)
  byte      periMacBuf[6]; 

  byte*     periBegOfRecord;
  byte*     periEndOfRecord;

  //byte      lastIpAddr[4]; 

extern char rulop[];                        // libellés opérations logiques regles analog/digital inputs
extern char inptyps[];                      // libellés types sources inputs
extern char inptypd[];                      // libellés types destinations inputs
extern char inpact[];                       // libellés actions  

uint8_t tablePerToSend[NBPERIF];            // liste des périphériques à mettre à jour après modif de détecteur (timers ou remote)

struct SwRemote remoteT[MAXREMLI];
char*  remoteTA=(char*)&remoteT;
unsigned long   remoteTlen=(sizeof(SwRemote))*MAXREMLI;

struct Remote remoteN[NBREMOTE];
char*  remoteNA=(char*)&remoteN;
unsigned long   remoteNlen=(sizeof(Remote))*NBREMOTE;

struct Timers timersN[NBTIMERS];
char*  timersNA=(char*)&timersN;
unsigned long   timersNlen=(sizeof(Timers))*NBTIMERS;

//struct TimersOld timersN_Old[NBTIMERS];
//char*  timersNA=(char*)&timersN;
//unsigned long   timersNlen=(sizeof(Timers))*NBTIMERS;

struct AnalTimers analTimers[NBANTIMERS];
char*  anTimersA=(char*)&analTimers;
unsigned long   anTimersLen=(sizeof(AnalTimers))*NBANTIMERS;

struct Thermo thermos[NBTHERMOS];
char*  thermosA=(char*)&thermos;
unsigned long   thermoslen=(sizeof(Thermo))*NBTHERMOS;

char   memosTable[LMEMO*NBMEMOS];

/* DS3231 */

#define PINVCCDS 19   // alim + DS3231
#define PINGNDDS 18   // alim - DS3231

  //Ymdhms dt;
  Ds3231 ds3231;
  char now[LNOW];
  unsigned long unixNow=0;

/*
 * =========== mécanisme de la librairie 
 * 
 *  Ethernet.begin connecte au réseau (mac, IP dans LAN par DHCP ou forcé) 
 *  
 *  Ethernet.localIP() rend l'adresse IP si elle provient du DHCP
 *  Ethernet.maintain() pour renouveler le bail DHCP
 *                                                                                                        
 *  l'objet client est utilisé pour l'accès aux serveurs externes ET l'accès aux clients du serveur interne
 *  EthernetClient nom_client crée les objets clients pour serveur externe ou interne (capacité 8 clients)
 *                                                                                                          
 *  pour creer/utiliser un serveur : EthernetServer nom_serveur(port) crée l'objet ; 8 serveurs possibles sur 8 ports différents
 *                                   nom_serveur.begin()  activation
 *                                   connexions des appels entrants : nom_client_in=nom_serveur.available()
 *                                   nom_serveur.write(byte) ou (buf,len) envoie la donnée à tous les clients 
 *                                   nom_serveur.print(variable) envoie la variable en ascii
 *                                   
 *  pour connecter un client de serveur externe : nom_client_ext.connect(IP ou nom du serveur,port)  
 *  
 *  pour les 2 types de clients :
 *    nom_client.connected() renvoie l'état de la connexion (indique qu'un message est dispo)
 *    nom_client.available() indique car dispo à lire
 *    nom_client.read() réception d'un car jusqu'à ce que available soit faux
 *    nom_client.write(char) envoie un car au client
 *    nom_client.print(variable) envoie la variable en ascii
 *    nom_client.flush() vidage buffer vers client
 *  
 *    en fin de transaction : nom_client.stop() déconnecte le client du serveur 
 *    et on reboucle sur nom_serveur.available ou nom_client.connect selon le cas
 *  
*/

bool oneIcon=false;

char inch=' ';
char strdate[LDATEB];       // buffer date
char strd3[4]={0};
byte js=0;
uint32_t amj=0, hms=0;

uint32_t histoPos=0;        // SD current pos. pour dump
uint16_t histoPeri=0;       // SD selected perif
char histoDh[LDATEA]={'\0'};   // SD dh pour dump

int   i=0,j=0;
char  c=' ';
char  b[2]={0,0};
extern const char* chexa; //="0123456789ABCDEFabcdef\0";
const byte  maskbit[]={0xfe,0x01,0xfd,0x02,0xfb,0x04,0xf7,0x08,0xef,0x10,0xdf,0x20,0xbf,0x40,0x7f,0x80};
const byte mask[]={0x00,0x01,0x03,0x07,0x0F};

unsigned long blinkt=millis();
extern uint32_t pinLed;
extern bool wdEnable;

/* buffer export config */

char bec[LBEC];        
//uint16_t lbec;
uint16_t rcvcnt=0;              // cnt requetes 

/* prototypes */

int  getnv(EthernetClient* cli);
void xcrypt();
void frecupptr(char* nomfonct,uint8_t* v,uint8_t* b,uint8_t lenpersw);
void bitvSwCtl(byte* data,uint8_t sw,uint8_t datalen,uint8_t shift,byte msk);
void test2Switchs();
void tcpPeriServer();
void browserServer();
void remoteServer();
void udpPeriServer();
int8_t perToSend(uint8_t* tablePerToSend,unsigned long begTime);
void poolperif(uint8_t* tablePerToSend,uint8_t detec,const char* nf,const char* src);
void cyclicTimersInit();
void scanTimers();
void scanRemote();
void scanDate();
void scanTemp();
void scanThermos();
void scanAnTimers();
void testUdp();
void cidDmp();
void watchdog();
void stoprequest();
void wdReboot(const char* msg,unsigned long maxCx);
void usrReboot();
void periDetecUpdate(const char* src);
void testSwitch(const char* command,char* perihost,int periport);
void serialServer();
uint16_t serialRcv(char* rcv,uint16_t maxl);
void disjValue(uint8_t val,uint8_t rem,uint8_t remTNum);

void yield()
{
  //trigwd();
}

void trigWdSetup()
{
  digitalWrite(PINLED,HIGH);delay(10);digitalWrite(PINLED,LOW);   // trig watchdog
}

void factoryReset()
{
  factoryResetConfig();
}

void checkPeri(bool set)
{
  uint8_t per[5];per[0]=8;per[1]=15;per[2]=16;per[3]=19;per[4]=22;
  for(uint8_t ppp=0;ppp<4;ppp++){

      if(*(periCache+(per[ppp]-1)*PERIRECLEN+(uint32_t)periDetNb-(uint32_t)periRec)>MAXDET || *(periCache+(per[ppp]-1)*PERIRECLEN+(uint32_t)periCfg-(uint32_t)periRec)!=0x01){
        Serial.println();Serial.print((uint8_t)*periDetNb);Serial.println((uint8_t)*periCfg,HEX);
        dumpstr(periCache+(per[ppp]-1)*PERIRECLEN,100);

        *(periCache+(per[ppp]-1)*PERIRECLEN+(uint32_t)periDetNb-(uint32_t)periRec)=MAXDET; 
        *(periCache+(per[ppp]-1)*PERIRECLEN+(uint32_t)periCfg-(uint32_t)periRec)=0x01;
      }
  }
}


void setup() {                          // ====================================

/* ---------- hardware setup ---------- */

  delay(3000);  // éponge le délai entre la fin de l'upload et le reset du Jlink

pinMode(16,OUTPUT);pinMode(17,OUTPUT); // timing spi sd et w5100 voir spi.cpp


  initLed(PINLED,LEDOFF,LEDON);
  wdEnable=true;trigwd(10000);

  SERIALX.begin (115200);               // export config periphériques

  Serial.begin (115200);
  
  delay(1000);
  Serial.print("+");

  pinMode(STOPREQ,INPUT_PULLUP);        // push button "HALT REQ"

/* ---------- config ---------- */  

  Serial.println();Serial.print(VERSION);
  
  #ifdef ANALYZE
  Serial.print(" ANALYZE ");
  #endif // ANALYZE
  #ifdef DUE
  Serial.print(" DUE ");
  #endif
  #ifndef DUE
  Serial.print(" NUCLEO ");
  #endif

  Serial.print(MODE_EXEC);Serial.print(" free=");Serial.print(freeMemory(), DEC);Serial.print(" FreeStack: ");Serial.println(FreeStack());

#ifdef REDV0
  Serial.print("carte red v0 ");
  digitalWrite(PINGNDDS,LOW);pinMode(PINGNDDS,OUTPUT);  // alim DS3231
  digitalWrite(PINVCCDS,HIGH);pinMode(PINVCCDS,OUTPUT); 
#endif // REDV0
#ifdef REDV1
  Serial.print("carte red v1");
  pinMode(POWCD,OUTPUT);                // ext card power on
  digitalWrite(POWCD,POWON);
  trigwd(100000);                       // uS
#ifdef AP2112
  Serial.println(".2 (AP2112)");
#endif // AP2112  
#ifndef AP2112
  Serial.println(".1 (LD1117)");
#endif // AP2112  
#endif // REDV1

  WIRE.begin();

  uint32_t        amj2,hms2;
  byte            js2;
  ds3231.i2cAddr=DS3231_I2C_ADDRESS; // doit être avant getDate
  ds3231.getDate(&hms2,&amj2,&js2,strdate);
  Serial.print("DS3231 time ");Serial.print(js2);Serial.print(" ");Serial.print(amj2);Serial.print(" ");Serial.println(hms2);
  
  ds3231.alphaNow(now);                                             
  unixNow=alphaDateToUnix(now,false);

//remInit();remoteSave();while(1){ledblink(0);delay(1000);};
//periConvert();
//periMaintenance();

  //sdExfatFormat();
  sdInit();
  //extern SdFile fhisto;sdRemove("fdhisto.txt",&fhisto);
  //while(1){}

  configInit();configLoad();//configSave();
  configPrint();
  memcpy(thermoCurPrev,thermoPrev,14);thermoCurPrev[14]='\0';
    
/* ---------- load variables du systeme : périphériques, table et noms remotes, 
              timers, détecteurs serveur ---------- */

  blink(4);
  
  if(digitalRead(STOPREQ)==LOW){
      trigwd();
        blink(4);
        factoryReset();
        while(1){blink(2);delay(1000);}
  }

  //periModification();             // chgt de structure de l'enregistrement perif (periRec)
  periTableLoad();                  // le premier (après config) pour permettre les mails

  iniDetServ();
  //memDetConvert();                // chgt du nombre de detServ
  memDetLoad();                     // le second pour Sync 
  //remoteNPlus(8);while(1){};
  //remoteTConvert();
  //remoteNConvert();while(1){};
  remoteLoad();//remotePrint();//periSwSync();
  //timersConvert();                // chgt du nombre de timers
  timersLoad();
  cyclicTimersInit();
  //timersConvert();
  //thermosInit();thermosSave();    // si NBPERIF change
  thermosLoad();
  //memosInit();memosSave(-1);  
  memosLoad(-1);
  
  //antRemove();anTimersInit();anTimersSave();
  anTimersLoad();
  //dumpstr(anTimersA,anTimersLen);

/* ---------- ethernet start ---------- */

  memcpy(mac,REDMAC,6);
  *perifPort=PORT_FRONTAL;
  *browserPort=PORT_BROWSER;
  *remotePort=PORT_REMOTE;
  udpPort[0]=PORTUDP;         // from const.h
  udpPort[1]=PORTUDP2;        // from const.h

  Serial.print(MODE_EXEC);
  Serial.print(" mac=");serialPrintMac(mac,0);
  Serial.print(" perifPort=");Serial.print(*perifPort);
  Serial.print(" browserPort=");Serial.print(*browserPort);
  Serial.print(" remotePort=");Serial.print(*remotePort);
  Serial.print(" serverUdpPort1=");Serial.print(udpPort[0]);
  Serial.print(" serverUdpPort2=");Serial.print(udpPort[1]);

  trigwd();

  if(Ethernet.begin(mac) == 0)
    {
    Serial.print("\nFailed with DHCP... forcing Ip ");serialPrintIp(localIp);Serial.println();  // config record IP
    Ethernet.begin (mac, localIp); 
    }
  Serial.print(" localIP=");
  for(i=0;i<4;i++){localIp[i]=Ethernet.localIP()[i];Serial.print(localIp[i]);if(i<3){Serial.print(".");}}Serial.println();
  configSave();
//  configExport(bec);wifiExport(bec,2);wifiExport(bec,1);concExport(bec);setExpEnd(bec);Serial.println(bec);

  udp[0]=&Udp1;
  udp[1]=&Udp2;
  for(uint8_t su=0;su<2;su++){
    Serial.print("udp");Serial.print(su);
    Serial.print(" Udp.begin(");Serial.print(udpPort[su]);Serial.print(") ");
    if(!udp[su]->begin(udpPort[su])){Serial.print("ko");mail("UDP_BEGIN_ERROR_HALT","");while(1){trigwd(1000000);}}
    
    //if(!Udp.begin(udpPort[su])){Serial.print("ko");mail("UDP_BEGIN_ERROR_HALT","");while(1){trigwd(1000000);}}
    Serial.println("ok");
    trigwd();
  }

  periserv=new EthernetServer(*perifPort);
  periserv->begin();Serial.print(" periserv.begin(");Serial.print(*perifPort);Serial.println(")");        // serveur périphériques

  browserserv=new EthernetServer(*browserPort);
  browserserv->begin();Serial.print(" browserserv.begin(");Serial.print(*browserPort);Serial.println(")");  //  browser serveur

  remoteserv=new EthernetServer(*remotePort);
  remoteserv->begin();Serial.print(" remoteserv.begin(");Serial.print(*remotePort);Serial.println(")");     //  remote serveur

  showSocketsStatus(false,false,true,"all servers begin ");

  Serial.print("size_of EthernetServer=");Serial.println(sizeof(EthernetServer));

/* ---------- RTC ON, check date/heure et maj éventuelle par NTP ---------- */
/* ethernet doit être branché pour l'udp */

  Serial.println();
  initDate();
  
/*  while(1){
  int udpav=Udp.parsePacket();
  char c;if(udpav>0){Udp.read(&c,1);Serial.print(c);}
  }*/

  histoStore_textdh(RESET,"","<br>\n\0");

  trigwd();

  mailEnable=VRAI;

  if(mailFromAddr!=nullptr && mailPass!=nullptr){
    Serial.print("START Mail ");mail("START","");
  }
  else Serial.println(" no mail config");

#ifdef ANALYZE
  STOPALL
  pinMode(ANPIN0,OUTPUT);
  pinMode(ANPIN1,OUTPUT);
  pinMode(ANPIN2,OUTPUT);
#endif // ANALYZE

  ds3231.alphaNow(now);                                             
  unixNow=alphaDateToUnix(now,false);
  
  Serial.println(">>>>>>>>> fin setup\n");
}

/* ================================== fin setup ================================= */

/* ==================================== loop ===================================== */

void loop()                         
{
            if(memcmp(prevsssa,sssa,MAX_SOCK_NUM)!=0){
              memcpy(prevsssa,sssa,MAX_SOCK_NUM+2);
#ifdef SOCK_DEBUG
              showSocketsStatus(false,false,true,"** "); 
#endif // SOCK_DEBUG              
            }
            loopCnt++;
            scanCnt=0;
//Serial.print("tcp=");Serial.println(millis());
            tcpPeriServer();     // *** périphérique TCP ou maintenance
//Serial.print("udp=");Serial.println(millis());
            udpPeriServer();     // *** périphérique UDP via NRF
            browserServer();     // *** browser
//Serial.print("pil=");Serial.println(millis());            
            remoteServer();      // *** remotes
//Serial.print("led=");Serial.println(millis());
            ledblink(0);
//Serial.print("ser=");Serial.println(millis());            
            serialServer();

ds3231.alphaNow(now);                                                 // après les serveurs qui peuvent être longs
unixNow=alphaDateToUnix(now,false);

//Serial.print("tem=");Serial.println(millis());
            scanTemp(); 
//Serial.print("dat=");Serial.println(millis());
            scanDate();         
            
            scanThermos();

            scanTimers();
//Serial.print("wdg=");Serial.println(millis());
            scanRemote();

            scanAnTimers();

            watchdog();
//Serial.print("hal=");Serial.println(millis());
            stoprequest();
}


/* ======================= tools et scans =========================== */

void stoprequest()
{
  if(digitalRead(STOPREQ)==LOW){
    trigwd();
    histoStore_textdh(HALTREQ,"","<br>\n\0");
    periTableSave();
    Serial.println("Halt request =====");
    mail("HALTed",(char*)(usrnames+usernum*LENUSRNAME));

    while(1){
      pinMode(PINLED,OUTPUT);wdEnable=false;
      digitalWrite(PINLED,HIGH);delay(500);digitalWrite(PINLED,LOW);delay(500);
    }
  }
}

void watchdog()
{
  if(millis()-lastcxt>*maxCxWt && lastcxt!=0){wdReboot("\n>>>>>>>>>>>>>>> TCP cx lost ",*maxCxWt);}
  if(millis()-lastcxu>*maxCxWu && lastcxu!=0){wdReboot("\n>>>>>>>>>>>>>>> UDP cx lost ",*maxCxWu);}
  
  // showSocketsStatus reste utile pour fermer les sockets restés ouvert lors d'un reset du MCU
  if(*openSockTo<2){*openSockTo=2;}
  if(*openSockScan<4){*openSockScan=4;}
  if((millis()-lastcxt>(*openSockTo*1000)) && (millis()-lastcxu)>(*openSockTo*1000) && (millis()-last_shscksta)>(*openSockScan*1000)){
    last_shscksta=millis();showSocketsStatus(true);}
}

void usrReboot()
{
    trigwd();
    Serial.println("user reBoot");delay(2);
    histoStore_textdh(UBOOT,(char*)(usrnames+usernum*LENUSRNAME),"<br>\n\0");
    periTableSave();
    mail("UsrBOOT",(char*)(usrnames+usernum*LENUSRNAME));
    forceWd();                             // wait for hardware watchdog
}

void wdReboot(const char* msg,unsigned long maxCx)
{
    trigwd();
    Serial.print(msg);Serial.print(maxCx/1000);Serial.println("sec");delay(4);
    histoStore_textdh(WDSD,msg,"<br>\n\0");
    periTableSave();
    mail("reBOOT",msg);
    forceWd();                             // wait for hardware watchdog
}

void scanTemp()
{
    if((millis()-temptime)>pertemp*1000 && scanCnt<10){   // *** maj température  
      
      unsigned long scanTempTime=micros();
      temptime=millis();
      scanCnt++;
      char buf[]={0,0,'.',0,0,0};
      float th;
      ds3231.readTemp(&th);
      if(fabs(th-oldth)>MINTHCHGE){
        oldth=th;sprintf(buf,"%02.02f",th);
        histoStore_textdh(TEMP,buf,"<br>\n\0");
      }
      if(0){Serial.print("   scanTemp(mic)=");Serial.println(micros()-scanTempTime);}
    }       
}

void scanDate()
{
    if((millis()-datetime)>perdate*1000 && scanCnt<10){   // *** maj date
      
      unsigned long scanDateTime=micros();
      datetime=millis();
      scanCnt++;
      trigwd();
      initDate();
      histoStore_textdh("D","","<br>\n\0");
      periTableSave();
      
      mail("DATE","");
      if(0){Serial.print("   scanDate(mic)=");Serial.println(micros()-scanDateTime);}
    }
}

void scanThermos()                                                        // positionnement détecteurs associés aux thermos
{                                                                         // maj tablePerToSend
  if((millis()-thermosTime)>perThermos*1000 && scanCnt<10){
    
  unsigned long scanThermosTime=micros();
  thermosTime=millis();
  scanCnt++;
  memcpy(bakDetServ,memDetServ,MDSLEN);
  memset(tablePerToSend,0x00,NBPERIF);      // !=0 si (periSend) periReq à faire sur le perif          

  uint8_t th,det,mds;
  uint8_t detLst[NBDSRV];                                // liste détecteurs concernés
  uint8_t detSta[NBDSRV];                                // état  après scan thermos
    
    /* repérage detecteurs */
    memset(detLst,0x00,NBDSRV);
    for(th=0;th<NBTHERMOS;th++){
      if(thermos[th].lowenable){detLst[thermos[th].lowdetec]++;}
      if(thermos[th].highenable){detLst[thermos[th].highdetec]++;}
    }

    /* maj état thermos */
    memset(detSta,0x00,NBDSRV);
    // déclenchement au seuil, retour sous/sur offset
    for(th=0;th<NBTHERMOS;th++){
      uint8_t per=(uint8_t)thermos[th].peri;
      if(per!=0){        
          periLoad(per);
          if( thermos[th].lowenable && *periLastVal_<=thermos[th].lowvalue){                                                        // set low  (tjrs on si<ref)
            thermos[th].lowstate=1;
          } else if( thermos[th].lowenable && *periLastVal_>(thermos[th].lowvalue+(thermos[th].lowoffset*thermos[th].lowstate))){   // clr low si > ref+offset*etat prec
            thermos[th].lowstate=0;}
        
          detSta[thermos[th].lowdetec]+=thermos[th].lowstate;
        
          if( thermos[th].highenable && *periLastVal_>=thermos[th].highvalue){                                                      // set high (tjrs on si>ref)
            thermos[th].highstate=1;
          } else if(thermos[th].highenable && *periLastVal_<(thermos[th].highvalue-thermos[th].highoffset*thermos[th].highstate)){  // clr high si < ref+offset*etat prec
            thermos[th].highstate=0;}
          
          detSta[thermos[th].highdetec]+=thermos[th].highstate;
      }      
    }

    /* maj détecteurs, tablePerToSend, peripheriques */
    //char onoff[]="O\0I";
    
    for(det=0;det<NBDSRV;det++){

      if(detLst[det]!=0){
        if(detSta[det]!=0){detSta[det]=1;}
        mds=0;
        uint8_t mi=det>>3;if((memDetServ[mi]&mDSmaskbit[det*MDSLEN+mi]) !=0){mds=1;}
        //if((memDetServ & mDSmaskbit[det])!=0){mds=1;}
        if((detSta[det] ^ mds)!=0){                                                   // change ?
          poolperif(tablePerToSend,det,&onoff[detSta[det]*3],"scanth");              
          uint8_t mi=det>>3;memDetServ[mi] ^= mDSmaskbit[det*MDSLEN+mi];
          //for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] = memDetServ[i] ^ mDSmaskbit[det*MDSLEN+i] ;}
          //memDetServ = memDetServ ^ mDSmaskbit[det];
        }
      }
    }
    periDetecUpdate("pDUth");                 // mise à jour remotes, fichier perif et tablePerToSend
    perToSend(tablePerToSend,thermosTime);    // mise à jour périphériques de tablePerToSend
    if(0){Serial.print("   scanThermos(mic)=");Serial.println(micros()-scanThermosTime);}
  }
}

void poolperif(uint8_t* tablePerToSend,uint8_t detec,const char* nf,const char* src)  
//  recherche des périphériques ayant une règle sur le memDet 'detec' qui a changé
//  ce qui nécessite une transmission aux périphériques concernés ; 
//  les byte de tablePerToSend associés aux périphériques concernés sont incrémentés ;
//  à la fin des modifs de memDetServ, perToSend liera tablePerToSend pour effectuer les envois

{                                                                     // et màj de tablePerToSend
  uint16_t offs;
  uint8_t eni,ninp;
  byte model=(detec<<PERINPNVLS_PB)|DETYEXT;                          // valeur pour N°detecteur type externe
  
  Serial.print(" poolperif (detec ");Serial.print(detec);Serial.print(" -> ");Serial.print(nf);Serial.print(") ");Serial.println(src);
  for(uint8_t np=1;np<=NBPERIF;np++){                                 // boucle périphériques

// ======================= accélérer façon anTimersScan ====================

    periLoad(np);
    if(*periSwNb!=0){                                                 // peripherique avec switchs ?
            
        for(ninp=0;ninp<NBPERRULES;ninp++){                           // boucle regles          
          offs=ninp*PERINPLEN;
          eni=((*(uint8_t*)(periInput+2+offs)>>PERINPEN_PB)&0x01);    // enable          
          if(eni!=0 && model==*(byte*)(periInput+offs)){              // trouvé usage du détecteur dans periInput 

            tablePerToSend[np-1]++;                                   // (periSend) periReq à faire sur ce périf            
          } // enable et model ok
        }   // input suivant   
    }       // periswNb !=0
  }         // perif suivant
}

int8_t perToSend(uint8_t* tablePerToSend,unsigned long begTime)       // maj des périphériques repérés dans la table spécifiée 
{
      periMess=MESSOK;
      
      for(uint16_t np=1;np<=NBPERIF;np++){
        if(tablePerToSend[np-1]!=0){
          cliext.stop();                                              // !!!!!!! et si cliext est en cours d'utilisation ? ... devrait pas
          periMess=periReq(&cliext,np,"mds_______");} 
      }
      memset(tablePerToSend,0x00,NBPERIF);
      return periMess;
}

int8_t perToSend(uint8_t* tablePerToSend)
{
  return perToSend(tablePerToSend,millis());
}

bool dhTimer(uint8_t nt)            // retour true si l'heure(now) est dans la fourchette hdeb/hfin et jour semaine ok
{
   if(
                (   ( memcmp(timersN[nt].hfin,timersN[nt].hdeb,6)>0   // heure fin > heure deb
                      && memcmp(timersN[nt].hdeb,(now+8),6)<0             // heure deb < now
                      && memcmp(timersN[nt].hfin,(now+8),6)>0)            // heure fin > now
                    ||
                    ( memcmp(timersN[nt].hfin,timersN[nt].hdeb,6)<0    // heure fin < heure deb
                      &&  (  memcmp(timersN[nt].hdeb,(now+8),6)<0          // heure deb < now
                          || memcmp(timersN[nt].hfin,(now+8),6)>0))        // heure fin > now
                )
                && ((timersN[nt].dw & maskbit[1+now[14]*2])!=0)       // jour semaine
      )
    return true;
    else return false;
}

void cyclicTimersInit(uint8_t nt)     // recalage état et unixCyclicTimersCurDate 
{   
  if(timersN[nt].cyclic_!=0){
    if(unixNow==0){
      ds3231.alphaNow(now);
      unixNow=alphaDateToUnix(now,false);
    }

    unixCyclicTimersCurDate[nt]=alphaDateToUnix(timersN[nt].dhdebcycle,false);
    unixCyclicTimersOnState[nt]=alphaDateToUnix(timersN[nt].onStateDur,false);
    unixCyclicTimersOffState[nt]=alphaDateToUnix(timersN[nt].offStateDur,false);
    Serial.print(nt);Serial.print(" cyclicInit >>>");Serial.print(timersN[nt].onStateDur);Serial.print(" ");Serial.print(unixCyclicTimersOnState[nt]);Serial.print(" ");Serial.print(timersN[nt].offStateDur);Serial.print(" ");Serial.println(unixCyclicTimersOffState[nt]);

    uint8_t k=0;                        // pas déclenché état courant off 
    unsigned long warn=millis();
    if(unixCyclicTimersCurDate[nt]<=unixNow){              // si déclenché rechercher l'état actuel et la dh de la prochaine transition
      while (unixCyclicTimersCurDate[nt]<unixNow){
          if(unixCyclicTimersCurDate[nt]<unixNow){unixCyclicTimersCurDate[nt]+=unixCyclicTimersOnState[nt];k=1;}   // état courant ON
          if(unixCyclicTimersCurDate[nt]<unixNow){unixCyclicTimersCurDate[nt]+=unixCyclicTimersOffState[nt];k=2;}  // état courant OFF
          if((millis()-warn)>2000){warn=millis();trigwd();}
      }
    }
      
    if(k==1){cyclicTimersState[nt]=1;}
    else cyclicTimersState[nt]=0;

    //char* cTCD=cyclicTimersCurDate+16*nt;cTCD[14]='\0';
    //unixDateToStr(unixCurD,cTCD);                     // dh prochaine transition
  }
}

void cyclicTimersInit()
{
  ds3231.alphaNow(now);
  unixNow=alphaDateToUnix(now,false);
  for(uint8_t t=0;t<NBTIMERS;t++){
    cyclicTimersInit(t);
    //Serial.print(t+1);Serial.print(" ");Serial.print((char*)(cyclicTimersCurDate+16*t));Serial.print(" ");Serial.print(cyclicTimersState[t]);Serial.print(" ");Serial.println(timersN[t].curstate);
  }
}

void cyclicTimerUpdate(uint8_t nt,unsigned long unixNow)    // positionne cyclicTimersState[nt] et unixCyclicTimersCurDate[nt]
{                                                           // (doit etre effectué au moins 1 fois par seconde)
  if(unixNow==0){
    ds3231.alphaNow(now);
    unixNow=alphaDateToUnix(now,false);
  }
  //Serial.print(unixCyclicTimersCurDate[nt]);Serial.print(" ");
  if(unixNow>unixCyclicTimersCurDate[nt]){
    cyclicTimersState[nt]^=1;
    if(cyclicTimersState[nt]==0){unixCyclicTimersCurDate[nt]+=unixCyclicTimersOffState[nt];}
    else{unixCyclicTimersCurDate[nt]+=unixCyclicTimersOnState[nt];}
    //Serial.print(nt+1);Serial.print(" ");Serial.print(cyclicTimersState[nt]);Serial.print(" ");Serial.print(unixCyclicTimersOnState[nt]);Serial.print(" ");Serial.print(unixCyclicTimersOffState[nt]);Serial.print(" ");
  }

//  Serial.print("====>> cyc upd ");Serial.println(unixNow);
}

void scanTimers()                                             //   recherche timer ayant changé d'état 
{                                                             //      si (en.perm.dh.js) (ON) et état OFF -> état ON, det ON, poolperif
                                                              //      sinon              (OFF) et état ON -> état OFF, det OFF, poolperif
                                                              //      màj tablePerToSend
/*
  si cyclic est off (mode static)
    si enable
    si permanent ou dans période dhdebcycle/dhfincycle
    si dans période hdeb/hfin     
  
  si cyclic est on
    la mise à jour de l'état et pochaine date/heure de changement est permanente
    (au reset ou changement de static à cyclic, cyclicTimersInit recale l'état et unixCyclicTimersCurDate )
    si enable
    si cycle dans période onState
    si permanent ou dans période dhdebcycle/dhfincycle
    si dans période hdeb/hfin     

*/
    if((millis()-timerstime)>pertimers*1000 && scanCnt<10){

      unsigned long scanTimersTime=micros();
      timerstime=millis();
      scanCnt++;
      memcpy(bakDetServ,memDetServ,MDSLEN);
      //bakDetServ=memDetServ;
          
      memset(tablePerToSend,0x00,NBPERIF);      // !=0 si (periSend) periReq à faire sur le perif          
      ds3231.alphaNow(now);
      unsigned long unixNow=alphaDateToUnix(now,false);

      for(uint8_t nt=0;nt<NBTIMERS;nt++){

        bool timerOn=false;
        if(timersN[nt].cyclic_==0 
          && timersN[nt].enable==1
          && (timersN[nt].perm==1 || (memcmp(timersN[nt].dhdebcycle,now,14)<0 && memcmp(timersN[nt].dhfincycle,now,14)>0))
          && dhTimer(nt)
          )                                  
        {timerOn=true;}
          
        if(timersN[nt].cyclic_==1){
          cyclicTimerUpdate(nt,unixNow);
          if(timersN[nt].enable==1
            && cyclicTimersState[nt]!=0
            && (timersN[nt].perm==1 || (memcmp(timersN[nt].dhdebcycle,now,14)<0 && memcmp(timersN[nt].dhfincycle,now,14)>0))
            && dhTimer(nt)
          )
          {timerOn=true;} 
        }

    // timerOn valorisé application sur détecteur

        if(timerOn==true){
          if(timersN[nt].curstate!=1){                              // et état précédent 0, chgt->1
            timersN[nt].curstate=1;
            memcpy(timersN[nt].dhLastStart,now,16);                 // mise à jour lastStart
            uint8_t mi=timersN[nt].detec>>3;memDetServ[mi] |= mDSmaskbit[timersN[nt].detec*MDSLEN+mi]; // maj détecteur
          }
        }
        else  {
          if(timersN[nt].curstate!=0){                              // et état précédent 1, chgt->0
            timersN[nt].curstate=0;
            memcpy(timersN[nt].dhLastStop,now,16);                  // mise à jour lastStop            
            uint8_t mi=timersN[nt].detec>>3;memDetServ[mi] &= ~mDSmaskbit[timersN[nt].detec*MDSLEN+mi]; // maj détecteur
          }
        }
      }
      periDetecUpdate("pDUti");                         // mise à jour remotes, fichier perif et tablePerToSend
      perToSend(tablePerToSend,timerstime);             // mise à jour périphériques de tablePerToSend
      if(0){Serial.print("   scanTimers(mic)=");Serial.println(micros()-scanTimersTime);}
    }
}

void osRemInit(uint8_t r)
{
  //Serial.print(">==osRemInit==<");Serial.print(r);Serial.print('_');Serial.println(remoteN[r].enable);
                memset(remoteN[r].osRemT,'\0',7);
                memset(remoteN[r].osEndDate,'\0',LDATEA-1);
                remoteN[r].osStatus=0;                              // status STOP
                for(uint8_t nr=0;nr<MAXREMLI;nr++){
                  //Serial.print(nr);Serial.print('_');Serial.print(remoteT[nr].num);Serial.print('_');Serial.println(r+1);
                  if(remoteT[nr].num==r+1){
                    disjValue(remoteN[r].enable,r,nr);              // restore periSwCde et maj perifs
                  }  
                }
}

void scanRemote()
{
  if((millis()-oneShotRemTime)>perOSR*1000 && scanCnt<10){  
  
    unsigned long scanRemoteTime=micros();
    oneShotRemTime=millis();
    scanCnt++;
    for(uint8_t r=0;r<NBREMOTE;r++){
      switch (remoteN[r].osStatus){
        case 0:break;                                               // STOP
        case 1:break;                                               // paused        
        case 2:                                                     // running
          //if(remoteN[r].osStatus==2){Serial.print(">>===");Serial.print(r);Serial.print(remoteN[r].osStatus);Serial.print(' ');Serial.print(remoteN[r].osEndDate);Serial.print(' ');Serial.println(now);}
          if(memcmp(remoteN[r].osEndDate,now,14)<=0){               // fin timing
              osRemInit(r);                                         // status STOP
              Serial.print(">>=== stop os remote ");Serial.println(r);
            }
            break;
        default:break;
      }
    }
    if(0){Serial.print("   scanRemote(mic)=");Serial.println(micros()-scanRemoteTime);}
  }
}

void printn(char* data,uint8_t len,bool nl)
{
  for(uint8_t ll=0;ll<len;ll++){Serial.print(*(data+ll));}
  if(nl){Serial.println();}
}

void dumpprintn(char* data,uint8_t len,bool nl)
{
  for(uint8_t ll=0;ll<len;ll++){
    Serial.print(chexa[*(data+ll)>>4]);Serial.print(chexa[*(data+ll)&0x0f]);Serial.print(' ');}
  if(nl){Serial.println();}
}


void scanAnTimers()
{
  // si première heure == 0 timer inutilisé
  // si heure suivante == 0 ou fin 
  //  heure suivante = première heure 
  //  si heure début >= heure(now) ou heure(now) < heure suivante ==> trouvé
  // sinon si heure début >= heure(now) et heure(now) < heure suivante ==> trouvé
  //
  
  if((millis()-antimerstime)>perAnTimers*1000 && scanCnt<10){
    
    unsigned long anTimScan=micros();
    antimerstime=millis();
    scanCnt++;
    char* hhbeg;
    char* hhfirst;
    char* hhnext;
    char hhnow[3];pack(now+8,hhnow,3,false);
    char val0[]={'\0','\0','\0'};
    uint16_t newval[NBANTIMERS];
    uint8_t  newvalnb[NBANTIMERS];memset(newvalnb,0x00,NBANTIMERS); //(uint8_t nv=0;nv<NBEVTANTIM;nv++){newvalnb[nv]=0;}
    uint8_t biten=maskbit[1+ANT_BIT_ENABLE*2];
    uint8_t dwNow=maskbit[1+2*(7-now[14])];
    uint8_t lastee=NBEVTANTIM-1;
    uint8_t ee;
    bool found=false;
    bool gfound=false;
    
    // scan anTimers
    for(uint8_t aa=0;aa<NBANTIMERS-1;aa++){
      hhfirst=&analTimers[aa].heure[0];
      //Serial.print("hhfirst:");dumpprintn(hhfirst,3,false);Serial.print(" val0:");dumpprintn(val0,3,false);
      //Serial.print(" dw:");dumpprintn((char*)&analTimers[aa].dw,1,false);Serial.print(" now:");dumpprintn(now,16,false);Serial.print(" dwNow:");dumpprintn((char*)&dwNow,1,true);
      if( ((analTimers[aa].cb)&(biten))!=0 
            && memcmp(hhfirst,val0,3)!=0
            && ((analTimers[aa].dw & 0x01)!=0 || (analTimers[aa].dw & dwNow)!=0 )){
        hhbeg=hhfirst;
        found=false;
        for(ee=0;ee<NBEVTANTIM;ee++){
            hhnext=&analTimers[aa].heure[3*(ee+1)];
      //Serial.print("hhbeg:");dumpprintn(hhbeg,3,false);Serial.print(" hhnext:");dumpprintn(hhnext,3,false);
      //Serial.print(" hhnow:");dumpprintn(hhnow,3,false);Serial.print(" lastee:");Serial.println(lastee);
            if((memcmp(hhnext,val0,3)==0 || ee==lastee)){
              if((memcmp(hhbeg,hhnow,3)<=0) || (memcmp(hhnow,hhfirst,3)<0)){found=true;break;}
            }
            else if((memcmp(hhbeg,hhnow,3)<=0) && (memcmp(hhnow,hhnext,3)<0)){found=true;break;}
            hhbeg=hhnext;
        }
        if(found && analTimers[aa].valeur[ee]!=analTimers[aa].curVal){       
            // trouvé une valeur à mettre à jour
            newvalnb[aa]++;
            newval[aa]=analTimers[aa].valeur[ee];
            gfound=true;
      //Serial.print(" curval:");Serial.print(analTimers[aa].curVal);Serial.print(" new:");Serial.println(analTimers[aa].valeur[ee]);
        }
      }
    }

    if(gfound){
      memset(tablePerToSend,0x00,NBPERIF);      
      for(uint8_t aa=0;aa<NBANTIMERS;aa++){
        // recherche périfs concernés : avec anal set et meme dsrv en première position des rules (perinput sur 1ère règle ; byte 0 type/n°)
        // =======================   utiliser tablePerToSend  =======================
        if(newvalnb[aa]!=0){
          for(uint8_t peri=1;peri<NBPERIF;peri++){
            
            char*     curRec=(char*)(&periCache[(peri-1)*PERIRECLEN]);
            uint8_t*  curCfg=(uint8_t*)curRec + (uint32_t)periCfg-(uint32_t)periRec;
            byte*     curInp=(byte*)curRec+(uint32_t)periInput-(uint32_t)periRec;
            uint8_t*  curAna=(uint8_t*)curRec+(uint32_t)periAnalOut-(uint32_t)periRec;
           /*
            uint8_t* curCfg=periCfg;
            byte* curInp=periInput;
            uint16_t* curAna=periAnalOut;
            periLoad(peri);      
           */ 
            if((*curCfg&PERI_ANAL) !=0
            && ((*curInp&PERINPNT_MS)>>PERINPNTLS_PB)==DETYEXT                // ctl type dsrv
            && analTimers[aa].detecOut==(*curInp&PERINPV_MS)>>PERINPNVLS_PB   // ctl n° dsrv
            && *(uint16_t*)curAna!=newval[aa]                                 // analog value changed
            ){tablePerToSend[peri-1]++;*(uint16_t*)curAna=newval[aa];
            Serial.print(" aa:");Serial.print(aa);Serial.print(" mdet:");Serial.print(analTimers[aa].detecOut);Serial.print(" peri:");Serial.print(peri);Serial.print(" val:");Serial.print(*(uint16_t*)curAna);
            }
          }
          analTimers[aa].curVal=newval[aa];
        }
      }
      perToSend(tablePerToSend,antimerstime);
    }
    if(0){Serial.print("   anTimScan(mic)=");Serial.println(micros()-anTimScan);}
  } 
}

void sser(uint8_t det,uint8_t valnou,const char* src) // si un det a changé (!= old) -> inscription perif éventuel dans tablePerToSend
{
  //char newval[]={'0','\0'};
  uint8_t newV=1;
  //uint32_t msk=mDSmaskbit[det];                                                                        
  uint8_t msk[MDSLEN];
  uint8_t mskneg[MDSLEN];
  //for(uint8_t i=0;i<MDSLEN;i++){msk[i]=mDSmaskbit[det*MDSLEN+i];mskneg[i]=~msk[i];}
  memcpy(msk,mDSmaskbit+det*MDSLEN,MDSLEN);memcpy(mskneg,msk,MDSLEN);
  //uint32_t mem=memDetServ & msk;                                            
  uint8_t mem[MDSLEN];
  memcpy(mem,memDetServ,MDSLEN);                                              // current detec value)
  if(valnou!=0){
    //memDetServ |= msk;
    for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i] |= msk[i];}                   // set bit (1)
    //newval[0]='1';}                       
    newV=0;}  // 0=ON
  else {for(uint8_t i=0;i<MDSLEN;i++){memDetServ[i]&=mskneg[i];}}             // clr bit (0)
    //memDetServ &= mDSmaskneg[det];}        
  for(uint8_t i=0;i<MDSLEN;i++){if((memDetServ[i] & msk[i]) != mem[i]){       // memDet chge => poolperif
  //if((memDetServ & msk) != mem){                                            
    poolperif(tablePerToSend,det,&onoff[newV*3],src);break;}}              // si le memDet est utilisé dans un périf, ajout du périf dans tablePerRoSend
}

void periDetecUpdate(const char* src)                          
// les actions qui modifient memDetServ (timers/thermos/remotes/modifs manuelles)
// doivent effectuer periDetecUpdate() qui fait poolperif

{
  uint8_t of=3;                                 // 0 si memDet=1 ou 3 si memDet=0
  srvdettime=millis();                                        
  
  memset(tablePerToSend,0x00,NBPERIF);          // périphériques !=0 => periReq à faire via pertoSend()
  
  for(uint8_t ds=0;ds<NBDSRV;ds++){                                               
    //if((memDetServ&mDSmaskbit[ds]) != (bakDetServ&mDSmaskbit[ds])){   // si le détecteur ds a changé
    uint8_t mds1[MDSLEN],mds2[MDSLEN];
    for(uint8_t i=0;i<MDSLEN;i++){mds1[i]=memDetServ[i]&mDSmaskbit[MDSLEN*ds+i];mds2[i]=bakDetServ[i]&mDSmaskbit[MDSLEN*ds+i];}
    if(memcmp(mds1,mds2,MDSLEN)!=0){

      of=3;
      //if((memDetServ&mDSmaskbit[ds])!=0){of=0;st=1;}
      for(uint8_t i=0;i<MDSLEN;i++){if(mds1[i]!=0){of=0;break;}}
      poolperif(tablePerToSend,ds,&onoff[of],src);                    // et si utilisé dans un périf, ajout du périf dans tablePerRoSend
    }
  }
}

/* ================================ decodage ligne GET/POST ================================ */

void cliPurge(EthernetClient* cli)
{
  if(ab!='u'){                                      // rien à faire en udp
    int q=cli->available();
    while(q>0){
      for(int qq=q;qq>0;qq--){cli->read();}
      q=cli->available();
    }
  }
}

void cliWrite(EthernetClient* cli,const char* data)                             // !!!!!!!!!!!!!! provisoirement ne gère plus l'udp !!!!!!!!!!!!!!!!!!!!!
{
  if(ab=='u'){return;}  // Udp.write(data,strlen(data));return;} !!!!!! doit être udp[x]->write(...) !!!!! inutilisé
  cli->write(data);
}

int cliAv(EthernetClient* cli,uint16_t len,uint16_t* dataCharNb)
{
  if(ab=='u'){if(*dataCharNb<len){return len-*dataCharNb;}else return 0;}
  return cli->available();
}

char cliRead(EthernetClient* cli,const char* data,uint16_t len,uint16_t* dataCharNb)
{
  if(ab=='u'){if(*dataCharNb<len){(*dataCharNb)++;return data[(*dataCharNb)-1];}else return '\0';}
  return cli->read();
}

int getcde(EthernetClient* cli,const char* data,uint16_t dataLen,uint16_t* dataCharNb) // décodage commande reçue selon tables 'httpCdes' longueur maxi HTTPCDLENGTH
{
  char c='\0',inCd[HTTPCDLENGTH+2];
  int cdNb=0;
  uint16_t cdCharCnt=0;
  while (cliAv(cli,HTTPCDLENGTH,dataCharNb) && c!='/' && cdCharCnt<HTTPCDLENGTH) {
      c=cliRead(cli,data,HTTPCDLENGTH,dataCharNb);Serial.print(c);                    // extrait la commande 
      if(c!='/'){inCd[cdCharCnt]=c;cdCharCnt++;}
      inCd[cdCharCnt]='\0';                                                           
  }
  delay(2);

  uint8_t ko=0;
  char* cdPtr=strstr(httpCdes,inCd);
  if(c!='/'){ko=1;}                                                                   // pas de commande, message 400 Bad Request
  else if(cdPtr==nullptr){ko=2;}                                                      // commande inconnue 501 Not Implemented}
  else cdNb=1+(cdPtr-httpCdes)/HTTPCDLENGTH;                                          // numéro de commande
    
  if(ko!=0 && (ab=='b' || ab=='c')){                                                                          // pas de cde valide -> vidage + message}
      switch(ko){
        case 1:cliWrite(cli,"<body><br><br> err. 400 Bad Request <br><br></body></html>");break;
        case 2:cliWrite(cli,"<body><br><br> err. 501 Not Implemented <br><br></body></html>");break;
        default:break;
      }
  }

  return cdNb;                                                                        // cdNb=0 si KO ; 1 à n numéro de commande
}

int analyse(EthernetClient* cli,const char* data,uint16_t dataLen,uint16_t* dataCharNb)  // decode la chaine et remplit les tableaux noms/valeurs 
{                                             // prochain car = premier du premier nom
                                              // les caractères de ctle du flux sont encodés %HH par le navigateur
                                              // '%' encodé '%25' ; '@' '%40' etc... 
  bool nom=VRAI,val=FAUX,termine=FAUX;
  int i=0,j=0;
  uint8_t c,cpc=0;                            // cpc pour convertir les séquences %hh 
  char noms[LENNOM+1]={0},nomsc[LENNOM-1];noms[LENNOM]='\0';nomsc[LENNOM-2]='\0';
  memset(libfonctions,0x00,sizeof(libfonctions));

      nvalf[0]=0;
      memset(valeurs,0,LENVALEURS);                     // effacement critique (password etc...)
      memset(noms,'\0',LENNOM);
      numfonct[0]=-1;                                   // aucune fonction trouvée

      while (cliAv(cli,dataLen,dataCharNb)){
        c=(uint8_t)cliRead(cli,data,dataLen,dataCharNb);

        if(c=='%'){cpc=c;}                              // %
        else {
        
          if(cpc=='%'){cpc=(c&0x0f)<<4;}                // % reçu ; traitement 1er car
          else {
        
            if(cpc!=0x00){
              if(c>0x39){c-=0x37;}else {c&=0x0F;}
              c=cpc+c;
              cpc=0x00;}     // traitement second                 
            
            Serial.print((char)c);trigwd();
//if(c=='&'){Serial.println();Serial.print(i);Serial.print("/");Serial.print(j);Serial.print(" ");Serial.print(noms);Serial.print(" ");Serial.print(nom);Serial.println(val);}            
//Serial.println();Serial.print(i);Serial.print("/");Serial.print(j);Serial.print(":");Serial.print((char)c);Serial.print(" ");Serial.print(nom);Serial.print(" ");Serial.print(val);Serial.print(" ");Serial.println(termine);            
            if (!termine){
              if (nom==FAUX && (c=='?' || c=='&')){nom=VRAI;val=FAUX;j=0;memset(noms,0x00,LENNOM);
                                                  if(i<(NBVAL-1)){i++;}
                                                  else {nom=FAUX;termine=VRAI;i=GETNV_FNNB_OVF;}     // tableaux saturés -> erreur
                                                  Serial.println(libfonctions+2*(i-1));}  // fonction suivante ; i indice fonction courante ; numfonct[i] N° fonction trouvée
              else if (nom==VRAI){
                if(j==LENNOM){
//Serial.println();Serial.println(c);                  
                  if(c==':' || c=='='){                                         // séparateur trouvé, ctle validité nom fonction
//Serial.print("\n");Serial.println(noms);
                    nom=FAUX;val=VRAI;
                    nvalf[i+1]=nvalf[i]+1;
                    if(i==0){nvalf[1]=0;}                                       // permet de stocker le tout premier car dans valeurs[0]
                    else {nvalf[i]++;}                                          // skip l'intervalle entre 2 valeurs           
                    
                    long numfn=(strstr(fonctions,noms)-fonctions)/LENNOM;     // acquisition nom terminée récup N° fonction
                    memcpy(libfonctions+2*i,noms+LENNOM-2,2);                   // les 2 derniers car du nom de fonction si nom court
//Serial.print("nom=");Serial.print(nom);Serial.print("val=");Serial.print(val);Serial.print(" numfn long=");Serial.println(numfn);
                    if(numfn<0 || numfn>=nbfonct){
                      memcpy(nomsc,noms,LENNOM-2);nomsc[LENNOM-2]=0x00;
                      numfn=(strstr(fonctions,nomsc)-fonctions)/LENNOM;       // si nom long pas trouvé, recherche nom court (complété par nn)
                      if(numfn<0 || numfn>=nbfonct){
//Serial.print(" numfn court=");Serial.println(numfn);
                        termine=VRAI;nom=FAUX;val=FAUX;i=GETNV_INVALID_FN1;}    // fonction inconnue -> erreur
                      else {numfonct[i]=numfn;}
                    }
                    else {numfonct[i]=numfn;}                                 // une fonction est reçue avec = ou :    
                  }
                  else {
//Serial.println();Serial.print((char)c);Serial.print(" ");Serial.print(noms);Serial.print(" ");Serial.print(numfonct[i]);Serial.print(" ");Serial.println(j);
                    nom=FAUX;termine=VRAI;i=GETNV_INV_SEP;}                     // séparateur invalide -> erreur
                }
                else if(j<LENNOM-2){
//Serial.println((char)c);                        
                       if((c>='A' && c<='Z') || (c>='a' && c<='z') || c=='_' || (c>='0' && c<='9')){
                         noms[j]=c;j++;}                                        // acquisition 8 premiers caractères
                       else {nom=FAUX;termine=VRAI;i=GETNV_INVALID_FN2;}        // car invalide -> erreur
                }
                else if(j<LENNOM && c>=PMFNCVAL){noms[j]=c;j++;}                 // 2 derniers caractères codés avec PMFNCVAL ou PMFNCHAR
              } // les 2 derniers car codent avec PMFNCVAL et PMFNCHAR ils peuvent prendre toutes les valeurs sauf les caractères spéciaux
                // la commande est de la forme : GET /[cx]?ffffffffff=aaaa..aaa&ffff... etc
        
              else if (val==VRAI){
//Serial.print(" ");Serial.println((char)c);                
                if (c!='&' && c!=':' && c!='=' && c>' '){
                  valeurs[nvalf[i+1]]=c;
                  if(nvalf[i+1]<(LENVALEURS-2)){nvalf[i+1]++;}                  // contrôle decap !
                  else {val=FAUX;termine=VRAI;i=GETNV_RCD_OVF;}}                // valeurs saturé -> erreur
                
                else if (c=='&'){nom=VRAI;val=FAUX;j=0;                         // acquisition valeur terminée ... fonction suivante
                  Serial.println();}
                else if (c<=' '){termine=VRAI;Serial.println();}                // ' ' interdit dans valeur : indique fin données
                else {val=FAUX;termine=VRAI;i=GETNV_INVALID_VAL;Serial.println();}                       
              }
              else {Serial.print("getnv ni nom ni val");while(1){ledblink(BCODESYSERR);}}   // ni nom ni val ... system error                                                                                      
            }//Serial.println(libfonctions+2*(i-1));
          }                                                                                             
        }
      }
      Serial.print("\n---- fin getnv i=");Serial.print(i);Serial.print(' ');delay(10); // delai affichage pour debug ===============================================
      if(numfonct[0]<0 && i>=0){i=GETNV_NO_FN;}
      return i;
}

int getnv(EthernetClient* cli,const char* data,uint16_t dataLen)        // décode commande, chaine et remplit les tableaux noms/valeurs
{                                     // sortie -1 pas de commande ; >=0 nbre de noms/valeurs-1 (0=1) 
  #ifdef DEBUG_ON
  delay(10);
  #endif
  uint16_t dataCharNb=0;
  numfonct[0]=-1;
#define LBUFLI 12
  char bufli[LBUFLI];
  
  Serial.print(" -- getnv ");if(ab=='u'){Serial.println(dataLen);}else Serial.println();
  delay(5);
  int cdNb=getcde(cli,data,dataLen,&dataCharNb); 

  Serial.print(" cdNb=");Serial.println(cdNb);delay(2);          

  char c;
  uint8_t pbli;
  switch(cdNb){
        case 0:                                                               // pas de commande
            cliPurge(cli);return -1;
        case 1:
            c=' ';
            pbli=0;
            while (cliAv(cli,dataLen,&dataCharNb) && c!='?' && pbli<(LBUFLI-1)){                  // attente '?' ou 'favicon'
              c=cliRead(cli,data,dataLen,&dataCharNb);Serial.print(c);                          // version "accélérée" : print des maxi 12 premiers caractères
              bufli[pbli]=c;pbli++;}                                          // le reste sera purgé
            
            /*
            while (cliAv(cli,dataLen,&dataCharNb) && c!='?'){                 // attente '?' 
              c=cliRead(cli,data,dataLen,&dataCharNb);Serial.print(c);
              bufli[pbli]=c;if(pbli<LBUFLI-1){pbli++;bufli[pbli]='\0';}
            }
            */
            delay(2);                                                         // GET
            if(strstr(bufli,"favicon")>0){                                    // favicon
              numfonct[0]=ffavicon;
              cliPurge(cli);
              return 0;}
            else if(bufli[pbli-1]=='?'){
              //if(ab=='u'){dumpstr((char*)data,128);}
              uint8_t k=analyse(cli,data,dataLen,&dataCharNb);if(k<0){cliPurge(cli);}return k;}
            else if(ab=='c' || ab=='b'){numfonct[0]=faccueil;cliPurge(cli);return 0;}  // pas de '?' = commande invalide -> faccueil
            else {cliPurge(cli);return-1;}
        case 2:                                                               // POST  
            return -1;                                                        // POST pas traité
        default:cliPurge(cli);return-1;
  } 
}

#ifdef _AVEC_AES
void xcrypt()
{
    AES_init_ctx_iv(&ctx, key, iv);
    AES_CTR_xcrypt_buffer(&ctx, chaine, 16);
}
#endif // _AVEC_AES


void test2Switchs()
{

  char ipAddr[16];memset(ipAddr,'\0',16);
  //charIp(lastIpAddr,ipAddr);
  for(int x=0;x<4;x++){
//Serial.print(x);Serial.print(" test2sw ");Serial.println(ipAddr);
    testSwitch("GET /testb_on__=0006AB8B\n\n",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testa_on__=0006AB8B\n\n",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testboff__=0006AB8B\n\n",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testaoff__=0006AB8B\n\n",ipAddr,*periPort);
    delay(2000);
  }
}

void testSwitch(const char* command,char* perihost,int periport)
{
            uint8_t fonct;
            
            memset(bufServer,'\0',32);
            memcpy(bufServer,command,24);

            int z=messToServer(&cliext,perihost,periport,bufServer);
            Serial.println(z);
            if(z==MESSOK){
              trigwd();
              periMess=getHttpResponse(&cliext,bufServer,LBUFSERVER,&fonct);
              Serial.println(periMess);
            }
            //purgeCli(&cliext);
            cliext.stop();        // en principe iniutile (purge fait stop)
            delay(1);
}


/* ================================ utilitaires serveur ================================= */

void setSourceDet(uint8_t detNb,uint8_t sourceCod,uint8_t sourceNb)
{
  sourceDetServ[detNb]=sourceCod*256+sourceNb;
}

void textfonc(char* nf,int len)
{
  memset(nf,0x00,len);
  memcpy(nf,valf,nvalf[i+1]-nvalf[i]);
}

byte inpsub(byte* ptr,byte mask,byte lshift,char* libel,uint8_t len)    // entre dans la ligne de règle *ptr à la position lshift,mask le n° du libellé valf trouvé dans libel au pas de len
{
  uint8_t lreel=0;
  char typ[8];
  for(int i=0;i<8;i++){if(*(valf+i)==0x00){lreel=i;i=8;}}
  
  byte v=0;
  byte num=0xff;
  
  if(lreel>0 && lreel<8 && len>0 && len<8){
    memcpy(typ,valf,lreel);for(int i=lreel;i<len;i++){typ[i]=' ';}typ[len]=0x00;
    char* p=strstr(libel,typ);
    if(p>=0){  
      v=(byte)((p-libel)/len);
      num=v;
      //Serial.print(" p=");Serial.print((long)p);Serial.print(" pos=");Serial.print(p-libel);Serial.print(" rang=");Serial.print(v);
      v = v << lshift;
    }
  }
  *ptr &= ~mask;
  *ptr |= v;

//Serial.print(" valf=");Serial.print(valf);Serial.print(" len=");
//Serial.print(len);Serial.print(" lreel=");Serial.print(lreel);Serial.print(" typ=");Serial.print(typ);Serial.print(" libel=");Serial.print(libel);Serial.print(" v=");Serial.println(v,HEX);
  return num;
}


void rulesfonc(uint8_t* cb,uint8_t* det,uint8_t* rdet,int8_t* memo)            // traitement des fonctions 'rules' de périf
{                                                                 //  (4*cb 1*det 1*memo)
  uint8_t li=*(libfonctions+2*i+1)-PMFNCVAL;                      // row
  uint8_t co=*(libfonctions+2*i)-PMFNCHAR;                        // column 
    
  if(co<4){cb[li]|=maskbit[(3-co)*2+1];
            }                                                     // cb
  if(co==4){inpsub(cb+li,0xf0,4,rulop,5);
            }                                                     // opération logique
  if(co==5){sourceDetServ[rdet[li]]=0;                            // det
            rdet[li]=0xff;
            uint16_t d;
            if(nvalf[i+1]-nvalf[i]>1){                            // si rien 0xff (refDet facultatif)
              conv_atob(valf,&d);rdet[li]=(uint8_t)d;
              setSourceDet(rdet[li],MDSPER,periCur);
              }               
            }
  if(co==6){sourceDetServ[det[li]]=0;                             // det
            det[li]=0xff;                    
            uint16_t d;conv_atob(valf,&d);det[li]=(uint8_t)d;
            setSourceDet(det[li],MDSPER,periCur);
            }
  if(co==7){                                                      // memo
    char bm[LMEMO];memset(bm,0x00,LMEMO);
    uint8_t lm=nvalf[i+1]-nvalf[i];
    alphaTfr(bm,LMEMO,valf,lm);
    if(bm[0]!=0 && lm!=0){memo[li]=memosFind();
      if(memo[li]>=0){
        memcpy(memosTable+memo[li]*LMEMO,bm,LMEMO-1);
        //memosSave(memo[li]);memosPrint();}                        // devrait enregistrer un memo à la fois ; 
        //memosSave(-1);memosPrint();}                              // fmemos.write(...) ne fonctionne apparemment qu'en "append"
      }
    }                       
  Serial.println();
  }
}

void getPC(bool load)
{
  if(periCur<=0){Serial.print("\n!!!!<periCur=");Serial.print(periCur);Serial.println(valf);periCur=NBPERIF;}
  if(periCur>NBPERIF){Serial.print("\n!!!!>periCur=");Serial.print(periCur);Serial.println(valf);periCur=NBPERIF;}
  if(load){periLoad(periCur);}
}

void getPeriCurLibf(bool load)
{
  periCur=*(libfonctions+2*i+1)-PMFNCHAR;
  getPC(load);
}

void getPeriCurValf(bool load)
{
  periCur=*valf-PMFNCHAR;
  getPC(load);
}

void pushSliderRemote(EthernetClient* cli,uint8_t rem)
{
                        uint8_t mi=remoteN[rem].detec>>3;uint16_t ptmi=remoteN[rem].detec*MDSLEN+mi; 
                        Serial.print("det=");Serial.print(remoteN[rem].detec);Serial.print(' ');
                        uint8_t val=*valf-PMFNCVAL;
                        uint8_t disjVal=*(valf+2)-PMFNCHAR;                                       // valeur à forcer
                        Serial.print(" rem=");Serial.print(rem);Serial.print(" val=");Serial.print(val);

                        if(val!=0){memDetServ[mi] |= mDSmaskbit[ptmi];Serial.print(" 1 ");}       // push envoie toujours 1
                        else {memDetServ[mi] &= ~mDSmaskbit[ptmi];Serial.print(" 0 ");}       
                        for(int8_t i=(NBDSRV>>3)-1;i>=0;i--){if(memDetServ[i]<16){Serial.print('0');}Serial.print(memDetServ[i],HEX);}Serial.println();
                        if(remoteN[rem].butModel!=PUSH){memDetSave();}                            

                          if(!remoteN[rem].multRem){                                              // remote simple
                            if(remoteN[rem].butModel!=PUSH || disjVal<10){                        // PUSH inactif si remote 'mère' disjonctée
                              uint16_t peri=*(valf+1)-PMFNCHAR;                                   // valide pour remote simple only
if(peri>=NBPERIF){Serial.print("pushSliderRemote_periLoad peri invalide");}                                      
                              periLoad(peri);
                              if(periSwCde!=0){
                                periReq(&cliext,peri,"mds_______");}                              // modif slider/push si switch pas disjoncté
                            }
                          }
                          else {                                                                  // remote multiple
                              if(remoteN[rem].enable!=0){                                         // pas disjonctée
                                  uint8_t mimul,ptmimul,detmul;
                                  memset(tablePerToSend,0x00,NBPERIF);
                                  for(uint8_t i=0;i<MAXREMLI;i++){
                                    if(remoteT[i].multRem==rem+1){                                // repérage péris concernés
                                      if(remoteN[remoteT[i].num-1].butModel == PUSH){periLoad(remoteT[i].peri);}
                                      if(remoteN[remoteT[i].num-1].butModel != PUSH || periSwCde!=0){     // non disjoncté (mais le périf n'exécutera pas anyway)
                                                                                                  // économise le periReq() si PUSH disjoncté
                                                                                                  // si slider disjoncté le disjoncteur change de valeur
                                        tablePerToSend[remoteT[i].peri]=1;                        // chargement tablePerTosend : les periReq à effectuer

                                        if((remoteN[rem].butModel!=PUSH) && (remoteN[remoteT[i].num-1].butModel) !=PUSH){   
                                                                                                  // si slider modif det remotes slider liées
                                                                                                  // si PUSH rien ne change (les MDS sont transmis seuls)
                                          detmul=remoteN[remoteT[i].num-1].detec;                 
                                          mimul=detmul>>3;
                                          ptmimul=detmul*MDSLEN+mimul;
                                          if(val!=0){memDetServ[mimul] |= mDSmaskbit[ptmimul];}
                                          else {memDetServ[mimul] &= ~mDSmaskbit[ptmimul];}
                                        }
                                      }
                                    }
                                  }                                     
                                  for(uint16_t i=0;i<NBPERIF;i++){
                                    if(tablePerToSend[i]!=0){periReq(&cliext,i,"mds_______");}
                                  }
                              }
                          }
                        if(remoteN[rem].butModel==PUSH){memDetServ[mi] &= ~mDSmaskbit[ptmi];}     // push envoie toujours 1 donc raz
}

void disjValue(uint8_t val,uint8_t rem,uint8_t remTNum)     // force val (=0 ou 1 ou 2) dans periSwCde
                                                            // et màj du ou des périf(s) concerné(s)
                                                            // selon remote simple/multiple
{
  Serial.print(">==(disjValue==<");Serial.print(rem);Serial.print('_');Serial.print(remTNum);Serial.print('_');Serial.println(val);
  uint8_t swMsk[]={0xFC,0xF3,0xCF,0x3F};
  
  if(!remoteN[rem].multRem){              // remote simple 
    //uint8_t remTNum=*valf-PMFNCHAR;       // n° switch dans table remoteT
    if(remTNum<MAXREMLI){
        periCur=remoteT[remTNum].peri;
        uint8_t curSw=remoteT[remTNum].sw;            
        periLoad(periCur);*periSwCde&=swMsk[curSw];*periSwCde|=val<<(curSw*2);  // update swCde
        periSave(periCur,PERISAVESD);
        periReq(&cliext,periCur,"mds_______");                                  // update périf
    }
  }
  else {                                      // val 0/1/2 du disjoncteur appuyé de la remote
    remoteN[rem].enable=val;
    memset(tablePerToSend,0x00,NBPERIF);
  
    for(uint16_t i=0;i<MAXREMLI;i++){         // recherche périfs affectés
                                              // les fichiers perifs ne sont pas modifiés : lors d'assyset(), la valeur
                                              // de swCde est recalculée en fonction des remotes multiples pour la 
                                              // valorisation du disjoncteur du périf physique
      if(remoteT[i].multRem==rem+1 && remoteT[i].peri!=0){
        tablePerToSend[remoteT[i].peri-1]=1;
      }
    }
    for(uint16_t i=0;i<NBPERIF;i++){
      if(tablePerToSend[i]!=0){                                                       
        periReq(&cliext,i+1,"mds_______");    // update périfs
      }
    }
  }
}

/* ================================ serveur ================================= */

void commonserver(EthernetClient* cli,EthernetUDP* udpCli,const char* bufData,uint16_t bufDataLen)
{
  scanCnt+=10;
  trigwd();
  EthernetClient* cli_debug=cli;    // backup cli pour vérifier sa stabilité

      unsigned long cxDur=millis();
/*
    Les messages peuvent provenir soit d'une connexion TCP soit UDP soit autre. 
        Si TCP (ab!='u') bufData et bufDataLen sont sans objet.
        Si UDP ou autre cli est invalide et sans objet (le membre est vide).
    
    Si ab='a' ou 'b', une connexion TCP est valide sur le serveur courant (clixx=xxxserv.available() et .connected() ont fonctionné)
    Sinon bufData contient bufDataLen caractères d'un paquet UDP ou autre.

    Si le client est un périphérique, la fonction peripass doit précéder dataread/datasave et le mot de passe est systématiquement contrôlé
    Si le client est un browser (la première fonction n'est pas peripass) les 2 premières fonctions doivent être : 
      soit username__ et password__ en cas de login.
      soit usr_ref___ la référence fournie en première zone (hidden) de la page (num user en libfonction+2xi+1) et millis() comme valeur;
*/
      cxtime=millis();    // pour rémanence pwd

      Serial.println();now[14]='\0';Serial.print((char*)(now+4));Serial.print(' ');
      Serial.print(cxtime);
      Serial.print(" sock:");if(ab=='u'){Serial.print(udpSockIndex);}else{Serial.print(cli->sockindex);}
      Serial.print(" serveur(");Serial.print((char)ab);
      if(ab=='a'){Serial.print(tPS);}
      if(ab=='u'){Serial.print(udpNb);}
      Serial.print(") ");serialPrintIp(remote_IP);Serial.print(" ");serialPrintMac(remote_MAC,false);
      delay(10);

      nbreparams=getnv(cli,bufData,bufDataLen);    // Serial.print("---- nbparams ");Serial.println(nbreparams);
      if(nbreparams<0){
        Serial.print(" getnv error #");Serial.println(nbreparams);
        //accueilHtml(cli);
      }
      else{

/*  getnv() décode la chaine GET ou POST ; le reste est ignoré
    forme ?nom1:valeur1&nom2:valeur2&... etc (NBVAL max)
    le séparateur nom/valeur est ':' ou '=' 
    la liste des noms de fonctions existantes est dans fonctions* ; ils sont de longueur fixe (LENNOM) ; 
    les 2 derniers caractères du nom de fonction peuvent être utilisés pour passer des paramètres, 
    la recherche du nom dans la table se fait sur une longueur raccourcie si rien n'est trouvé sur la longueur LENNOM.
    (C'est utilisé pour réduire le nombre de noms de fonctions)
    la liste des fonctions trouvées est dans numfonct[]
    les valeurs associées sont dans valeurs[] (LENVAL car max)
    les 2 derniers car de chaque fonction trouvée sont dans libfonctions[]
    i pointe sur tout ça !!!
    nbreparams le nbre de fonctions trouvées
    il existe nbfonct fonctions 
    la fonction qui renvoie à l'accueil est faccueil (par ex si le numéro de fonction trouvé est > que nbfonct)
    Il y a une fonction par champ de l'enregistrement de la table des périphériques 
    periCur est censé être valide à tout moment si != 0  
    Lorsque le bouton "maj" d'une ligne de la table html des produits est cliqué, les fonctions/valeurs des variables sont listées
    par POST et getnv() les décode. La première est periCur (qui doit etre en premiere colonne).
    Les autres sont transférées dans periRec puis enregistrées avec periSave(periCur).

    Toutes les commandes concernent les browsers clients html sauf 3 : peri_pass_ / data_read_ / data_save_ 
    qui réalisent l'interface avec les périphériques. Format de la donnée : nnnn_...._cc 
    (HTTP est respecté et on peut simuler un périphérique depuis un browser html)
    Lorsqu'un périphérique se connecte pour la première fois, son adresse mac est inconnue du serveur et il n'a pas
    de numéro de périphérique (=0). Il envoie une demande "data_read_".
    Le serveur cherche une place libre et lui attribue le num. correspondant.
    Si son adresse mac est déjà connue il lui donne le N° de périphérique correspondant.
    Il renvoie une commande 'set_______' avec les params du périphérique tel qu'ils sont dans la table :
    N° de périphérique et adresse Mac (0 s pas de place), pitch et période pour les mesures de température, 
    nombre de switchs, la durée des pulses éventuels et état des switchs, le nombre de détecteurs, pour chaque switch
    leur mode d'action sur le switch. Plus la date et l'heure.

    Le serveur peut envoyer au périphérique une demande d'état "etat______", la réponse est "data_save_" avec les 
    données du périphérique qui sont stockées dans la table. La réponse du serveur est "ack_______" 
    avec date et heure comme donnée.
    Le périphérique qui a un numéro envoie des "data_save_" selon la période programmée.
    
    Un périphérique qui n'a jamais été identifié a le numéro 0 ; si le serveur donne la valeur 0 lors d'un data_read_
    c'est que l'adresse mac du périph ne correspond pas à celle de la table ou n'existe pas. Le périph remet son numéro
    à 0 et refait une procédure d'initialisation (en principe, data_read_).

    En cas d'anomalie de transmission le périphérique met son numéro à 0.
    Au reset, le numéro de périph est 0.
    
    Le symbole "_" sert de séparateur de champ et n'est pas autorisé dans les data.

    format des messages d'échange avec les périphériques :    nom_fonction=nnnn_...donnée...cc
              nom_fonction 10 caractères
              nnnn longueur sur 4 chiffres complétés avec 0 à gauche de la longueur au crc inclus
              cc crc

    La longueur maxi théorique des datas en provenance des périphériques est limitée par la commande GET et à 99999 avec POST
    Pratiquement voir les paramètres NBVAL LENVAL LVAL LBUFSERVER etc...
    
*/
        //periInitVar();        // pas de rémanence des données des périphériques entre 2 accès au serveur

        memset(strHisto,0x00,sizeof(strHisto)); memset(buf,0,sizeof(buf));charIp(strHisto,(char*)&remote_IP);        // histo :
        sprintf(buf,"%d",nbreparams+1);strcat(strHisto," ");strcat(strHisto,buf);strcat(strHisto," = ");             // une ligne par transaction
        strcat(strHisto,strHistoEnd);

/*      
    Un formulaire (<form>....</form>) contient des champs de saisie dont le nom et la valeur sont transmis dans l'ordre d'arrivée 
    quand la fonction submit incluse dans le formulaire est déclenchée (<input type="submit" value="MàJ">)

    La première fonction du formulaire (après le mot de passe) doit fixer les paramètres communs à toutes les autres du formulaire :
      pour chaque ligne de periTable ou table de switchs, le traitement à effectuer après que toutes les fonctions 
      soient traitées, le numéro de périphérique, le chargement du périphérique, d'éventuels autres traitements communs préalables.
    premières fonctions de formulaires : pericur (lignes de péritable), peri_t_sw_)
    
    what indique le traitement à effectuer (voir switch(what) pour les détails)
*/
        what=0;                           // pas de traitement subsidiaire (what=99 pour accueil)

/*
      3 modes de fonctionnement :

      1 - accueil : saisie de mot de passe
      2 - serveur pour périphériques : seules fonctions dataRead et dataSave ; contiennent l'adr mac pour identifier le périphérique.
      3 - serveur pour navigateur : dans les message GET ou POST, 
          pour assurer la validité des données si un périphérique particulier est concerné, periCur doit être positioné et periLoad() 
          effectué par une fonction qui précède les fonctions modifiant des variables du formulaire concerné 
          formIntro permet d'insérer une fonction après user_ref (peri_cur__ ou une fonction dédiée pour positionner what si 5 ne va pas)
          et le paramètre ninp permet de passer periCur

    si la première fonction est fperipass, c'est un périphérique, sinon un browser 
    si c'est un browser : username__ puis password__ en cas de login, 
                          sinon user_ref_n=ttt... et contrôle du TO (ttt... millis() du dernier accés)
                            pour permettre le refresh demandé par le navigateur qui va renvoyer ttt de la dernière commande (et non celui du dernier accès)
                            les fonctions dont le nom se terminent par html acceptent ttt de la dernière commande.
                          user_ref__ seule génère peritable
    
    fonctionnement de la rémanence de password_ :
    chaque entrée de la table des utilisateurs incorpore un champ usrtime qui stocke millis() de password puis de la dernière fonction usr_ref__
    les boutons et autres commandes recoivent usrtime et le renvoient lorsque déclenchés 
    usrTime est la valeur asociée à la fonction user_ref_@ à placer en tête de formulaire (1ère fonction après <form> via usrFormHtml ou un dérivé usrFormInit/usrPeriCur). 
    Lorsque le bouton submit est appuyé usr_ref sort en tête pour la gestion de rémanence. 
    ====== ATTENTION ===== pericur n'est pas valide lorsque getnv décode la ligne GET ; il faut le fixer en ajoutant dans la fonction d'init qui suit usr_ref_ (produite avec usrPeriCur)
    Si usrtime a changé ou si le délai de validité est dépassé ---> accueil.
    Le délai est modifiable dans la config (commun à tous les utilisateurs)
    ------------------------------------------------------------------------------------------------------------------------------------------------------
    la structure des pages doit être : <form> (éventuellemnt dans pageLineOne) usrPericur() {boutons etc... toute la page - Un bouton de MàJ par formulaire} </form>
    <form>...</form> délimite l'espace d'action de sbmit du bouton MàJ  ;  plusieurs formulaires possibles dans une page
    ------------------------------------------------------------------------------------------------------------------------------------------------------
                                       

    Sécurité à développer : pour assurer que le mot de passe n'est pas dérobable et que sans mot de passe on ne peut avoir de réponse du serveur,
    le mot de passe doit être crypté dans une chaine qui change de valeur à chaque transmission ; donc crypter mdp+heure. 
    Le serveur accepte une durée de validité (10sec?) au message et refuse le ré-emploi de la même heure.
    Ajouter du javascript pour crypter ce qui sort du navigateur ? (les 64 premiers caractères de GET / : username,password,heure/user_ref,heure)
    
*/     

      if(numfonct[0]!=fperipass && numfonct[0]!=ffavicon){                                          // si la première fonction n'est pas peri_pass_ (mot de passe des périfs)
        if((numfonct[0]!=fusername || numfonct[1]!=fpassword) && numfonct[0]!=fuserref){            //   si (la 1ère fonct n'est pas username__ ou la 2nde pas password__ ) et la 1ère pas user_ref__
                                                                                                    //   ... en résumé : ni un périf, ni une nlle cx utilisateur, ni une continuation d'utilisateur  
        // pourquoi faire un cas particulier de nbreparams!=0 ? pas périf, pas utilisateur donc ko et accueil devrait suffire ???
          if(nbreparams==0){what=-1;}nbreparams=-1;     //  what==-1 -> accueil (pas de params donc tentative de connexion) 
        }                                               //  sinon what==0 aucune action - attente d'une connexion valide
      }                                                                                             //  nbreparams==-1   skip all
/*
    boucle des fonctions accumulées par getnv 
    (numfonct[0] == soit fperipass donc un peri, soit fusername ou fuserref+fpassword donc un utilisateur, soit faccueil) 
*/   

        uint16_t transferVal=0;         // pour passer "quelque chose" entre 2 fonctions 
        
        periMess=MESSOK;

        for (i=0;i<=nbreparams;i++){    // boucle de traitement des fonctions ; utiliser i comme pointeur est une foutue mauvaise idée... surtout que c'est une variable globale qu'on peut retrouver n'importe où...
//if(i==0 && ab=='u'){Serial.println(bufData);}
          if(i<NBVAL && i>=0 && periMess==MESSOK){     // si une anomalie, fin de la boucle (periMess positionné dans checkData et periDataRead)
          
            trigwd();
          
            valf=valeurs+nvalf[i];      // valf pointe la ième chaine à traiter dans valeurs[] (terminée par '\0')
                                        // nvalf longueur dans valeurs
                                        // si c'est la dernière chaîne, strlen(valf) est sa longueur 
                                        // c'est obligatoirement le cas pour data_read_ et data_save_ qui terminent le message          
/*            
    controle de dépassement de capacité du buffer strHisto ; si ok, ajout de la fonction, sinon ajout de '*'  
*/
            if((strlen(strHisto)+strlen(valf)+5+strlen(strHistoEnd))<RECCHAR){
              strHisto[strlen(strHisto)-strlen(strHistoEnd)]='\0';sprintf(buf,"%d",numfonct[i]);strcat(strHisto,buf);
              strcat(strHisto," ");strcat(strHisto,(char*)valf);strcat(strHisto,";");strcat(strHisto,strHistoEnd);}
            else {strHisto[strlen(strHisto)-strlen(strHistoEnd)]='*';}

//Serial.print(i);Serial.print(" numfonct[i]=");Serial.print(numfonct[i]);Serial.print(" valf=");Serial.println((char*)valf);
//Serial.print("strHisto=");Serial.println(strHisto);
#ifdef DEBUG_ON      
      delay(60); // pour permettre l'affichage des Serial.print en debug
#endif

            numfonc=numfonct[i];
            switch (numfonc)
              {
              case 0:  pertemp=0;conv_atobl(valf,&pertemp);break;                                    // pertemp serveur
              case 1:  periMess=checkData(valf);                                                     // peri_pass_
                       if(periMess==MESSOK){
                         periPassOk=ctlpass(valf+5,peripass);                                        // skip len
                         if(!periPassOk){
                          memset(remote_IP_cur,0x00,4);histoStore_textdh("pp","ko",strHisto);
                          Serial.print(" periPass ");for(uint8_t c=0;c<6;c++){Serial.print(*(valf+5+c));}Serial.print(" ");for(uint8_t c=0;c<6;c++){Serial.print(*(peripass+c));}Serial.println(" ko");
                         }
                         else {memcpy(remote_IP_cur,(byte*)&remote_IP,4);}
                       }break;
              case 2:  usernum=searchusr(valf);if(usernum<0){                                        // username__
                          what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;}
                       Serial.print("username:");Serial.print(valf);Serial.print(" usernum=");
                       Serial.print(usernum);Serial.print("/");Serial.print(usrnames+usernum*LENUSRNAME);
                       Serial.print(" usrtime=");Serial.print(usrtime[usernum]);
                       #ifdef DEBUG_ON
                       delay(50);
                       #endif
                       break;
              case 3:  if(!ctlpass(valf,usrpass+usernum*LENUSRPASS)){                                // password__
                         what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;usrtime[usernum]=0;          // si faux accueil (what=-1)
                         histoStore_textdh("pw","ko",strHisto);
                         Serial.print(" password ");for(uint8_t c=0;c<LENUSRPASS;c++){Serial.print(*(valf+c));}Serial.print(" ");for(uint8_t c=0;c<LENUSRPASS;c++){Serial.print(*(usrpass+c));}Serial.print(" ko w=");
                       }                                   
                       else {Serial.print(" password ok w=");usrtime[usernum]=millis();if(nbreparams==1){what=2;}}
                       Serial.println(what);
                       break;                                                                        
              case 4:  {usernum=*(libfonctions+2*i+1)-PMFNCHAR;                                      // user_ref__ (argument : millis() envoyées avec la fonction au chargement de la page)
                        unsigned long cxtime=0;conv_atobl(valf,(uint32_t*)&cxtime);
                        Serial.print("user_ref__ : usrnum=");Serial.print(usernum);Serial.print(" millis()/1000=");Serial.print(millis()/1000);Serial.print(" cxtime=");Serial.print(cxtime);
                        Serial.print(" usrtime[nb]=");Serial.print(usrtime[usernum]);Serial.print(" usrpretime[nb]=");Serial.print(usrpretime[usernum]);
                        #ifdef DEBUG_ON
                        delay(50);
                        #endif
                        // !( usrtime ok || (html && usrpretime ok) ) || time out  => accueil 
                        if( !(usrtime[usernum]==cxtime 
                          || (usrpretime[usernum]==cxtime 
                          && memcmp(&fonctions[numfonct[i+1]*LENNOM]+(LENNOM-3),"html",4)==0)) 
                          || (millis()-usrtime[usernum])>(*toPassword*1000)){
                          what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;usrtime[usernum]=0;}
                        else {Serial.print(" user ");Serial.print(usrnames+usernum*LENUSRNAME);Serial.print(" ok");
                          usrtime[usernum]=millis();
                          usrpretime[usernum]=cxtime;
                          if(nbreparams==0){what=2;}
                        }
                        Serial.println();
#ifdef DEBUG_ON
  delay(10);
#endif
                       }
                       break;  
              case 5:  *toPassword=TO_PASSWORD;conv_atob(valf,toPassword);break;                    // to_passwd_
              case 6:  what=2;perrefr=0;conv_atob(valf,&perrefr);                                   // (en tête peritable) periode refresh browser
                       break;                                                                               
              case 7:  break; //*periThOffset_=0;*periThOffset_=(int16_t)(convStrToNum(valf,&j)*100);break;  // (periLine) Th Offset
              case 8:  getPeriCurLibf(PERILOAD);                                                    // bouton switchs___ (periLine/showLine/switchs)
                       switch (*(libfonctions+2*i)){
                         case 'X':periInitVar0();
                                  periSave(periCur,PERISAVELOCAL);
                                  swCtlTableHtml(cli);break;                                        // + bouton erase    (switchs)
                         case 'Y':for(uint8_t ninp=0;ninp<NBPERRULES;ninp++){
                                    if((*(periInput+ninp*PERINPLEN+2)&PERINPACT_MS)!=PMDCA_VIDE){   // seules les lignes avec action sont modifiées
                                      *(periInput+ninp*PERINPLEN+2) ^= PERINPEN_VB;}}
                                  periSave(periCur,PERISAVELOCAL);
                                  swCtlTableHtml(cli);break;                                        // + bouton en/dis all    (switchs)
                         case 'W':cliext.stop();periReq(&cliext,periCur,"etat______");
                                  swCtlTableHtml(cli);break;                                        // + bouton refresh switchs/showLine/periLine
                         case 'U':if(periSrc!=0){                                                   // + bouton copy from (switchs)
                                    memcpy(periInput,periCache+(periSrc-1)*PERIRECLEN+(periInput-periBegOfRecord),PERINPLEN*NBPERRULES);
                                    periSave(periCur,PERISAVESD);periSrc=0;swCtlTableHtml(cli);}
                                  break;
                         default:break;
                       }break;                                                                               
              case 9:  what=99;{byte a=*(libfonctions+2*i+1);
                        if(a=='B'){usrReboot();}
                       }break;                                                                      // si pas 'R' déco donc -> accueil                                             
              case 10: histoHtml(cli);break;                                                        // bouton dump_histo dump_his__ (inutile le bouton n'existe plus)
              case 11: {what=16;byte a=*(libfonctions+2*i);                                         // (hist_sh_) saisies paramètres dump
                        switch(a){ 
                          case 'D':memcpy(histoDh,valf,LDATEA-2);if(histoDh[8]==0x2B){histoDh[8]=0x20;}break;  // (histohtml) date/heure au format "AAAAMMDD HHMMSS"
                          case 'P':histoPeri=0;conv_atob(valf,&histoPeri);break;                    // (histohtml) périf
                          case 'p':histoPos=0;conv_atobl(valf,&histoPos);break;                     // (histohtml) position
                          default:break;
                        }
                       }break;                           
              case 12: if(periPassOk==VRAI){what=1;periDataRead(valf);periPassOk=FAUX;}break;       // data_save
              case 13: if(periPassOk==VRAI){what=3;periDataRead(valf);periPassOk=FAUX;}break;       // data_read
              case 14: {byte a=*(libfonctions+2*i);                                                 // (periLine) - tests de perif serveur
                        getPeriCurLibf(PERILOAD);                                                   // a cde (N°sw/mail) ; k etat à sortir 
                        char fptst[LENNOM+1];                                                        
                        char swcd[]={"sw0__ON___sw0__OFF__sw1__ON___sw1__OFF__mail______"};
                        char msg[64];msg[0]='\0';
                        if(a=='m'){a=2;strcat(msg,"TEST==");strcat(msg,mailToAddr1);strcat(msg,"==test peri ");concatn(msg,periCur);strcat(msg," ");strcat(msg,alphaDate());}
                        else {a-=PMFNCVAL;}
                        memcpy(fptst,swcd+LENNOM*a*2,LENNOM);
                        cliext.stop();periReq(&cliext,periCur,fptst,msg);
                        periLineHtml(cli);                        
                       }break;                                                                       
              case 15: what=5;getPeriCurValf(PERILOAD);break;                                             // (periLine) - peri_cur__ bouton submit
              case 16: what=5;getPeriCurLibf(false);                                                      // (periLine) peri_raz___
                       periInitVar();
                       break;
              case 17: {char pLFonc=*(libfonctions+2*i);                                                  // (periLine) sauf boutons et anal/dig
                        switch (pLFonc){
                          case 'N':alphaTfr(periNamer,PERINAMLEN,valf,nvalf[i+1]-nvalf[i]);               // (periLine) - peri_lf_N_ nom
                                  *periCfg&=~PERI_ANAL;                                                   // efface check box flag analogique
                                  *periCfg&=~PERI_SERV;                                                   // efface check box flag serveur                                                   
                                  *periCfg&=~PERI_RAD;                                                    // efface check box flag radiateur
                                  *periCfg&=~PERI_STA;break;                                              // efface check box flag thermostat
                          case 'M':for(j=0;j<6;j++){conv_atoh(valf+j*2,(periMacr+j));}break;              // (periLine) - peri_lf_M_ mac
                          case 'v':*periVmin_=0;*periVmin_=(int16_t)convStrToInt(valf,&j);break;          // (periLine) - peri_lf_v_ Vmin
                          case 'V':*periVmax_=0;*periVmax_=(int16_t)convStrToInt(valf,&j);break;          // (periLine) - peri_lf_V_ Vmax
                          case 'h':*periThmin_=0;*periThmin_=(int16_t)convStrToInt(valf,&j);break;        // (periLine) - peri_lf_h_ th min inutilisé?            
                          case 'H':*periThmax_=0;*periThmax_=(int16_t)convStrToInt(valf,&j);break;        // (periLine) - peri_lf_H_ th max inutilisé?
                          case 't':*periPerTemp=0;conv_atob(valf,periPerTemp);break;                      // (periLine) - peri_lf_t_ per temp      
                          case 'p':*periPitch_=0;*periPitch_=(int16_t)(convStrToNum(valf,&j)*100);break;  // (periLine) - peri_lf_p_ pitch
                          case 'o':*periThOffset_=0;*periThOffset_=(int16_t)(convStrToNum(valf,&j)*100);break;  // (periLine) - peri_lf_o_ th offset 
                          case 'r':*periPerRefr=0;conv_atobl(valf,periPerRefr);break;                     // (periLine) - peri_lf_r_ per refr
                          case 'P':if((*valf-PMFNCVAL)!=0){*periCfg|=PERI_SERV;};break;                   // (periLine) - peri_lf_P_ prog
                          case 'a':if((*valf-PMFNCVAL)!=0){*periCfg|=PERI_ANAL;};break;                   // (periLine) - peri_lf_a_ anal
                          case 'R':if((*valf-PMFNCVAL)!=0){*periCfg|=PERI_RAD;};break;                    // (periLine) - peri_lf_r_ radiateur
                          case 'T':if((*valf-PMFNCVAL)!=0){*periCfg|=PERI_STA;};break;                    // (periLine) - peri_lf_T_ thermostat
                          case 'A':uint16_t value;conv_atob(valf,&value);*periAnalOut=0;*periAnalOut=value;break;   // (periLine) - consigne analogique
                          case 'i':*periSwNb=*valf-PMFNCVAL;if(*periSwNb>MAXSW){*periSwNb=MAXSW;}break;             // (periLine) - peri_lf_i_ sw nb
                          case 'd':*periDetNb=*valf-PMFNCVAL;if(*periDetNb>MAXDET){*periDetNb=MAXDET;}break;        // (periLine) - peri_lf_d_ det nb
                          case 'x':*periPort=0;conv_atob(valf,periPort);break;                                      // (periLine) - peri_lf_x_ port
                          case 'W':{uint8_t sw=*(libfonctions+2*i+1)-PMFNCHAR;                                      // (periLine) - peri_lf_W_ sw Val 
                                   uint8_t cd=*valf-PMFNCVAL;
                                   periSwCdUpdate(sw,cd);                 // maj periSwCde (periCur ok, periLoad effectué)
                                   //remoteUpdate(periCur,sw,cd,PERILINE);  // maj remotes concernées
                                   }break;
                          default :break;
                        }
                       }break;
              case 18: if(periPassOk==VRAI){what=14;periDataRead(valf);periPassOk=FAUX;}break;      // data_na___ pas de réponse à faire ; (idem data_save sans réponse serveur)
              case 19: accueilHtml(cli);break;                                                      // accueil
              case 20: periTableHtml(cli);break;                                                    // peri table
              case 21: what=0;break;                                                                // data_store
              case 22: if(periPassOk==VRAI){what=17;periDataRead(valf);periPassOk=FAUX;}break;      // data_mail_ idem data_save_ + texte mail ; pas de réponse à faire
              case 23: if(periPassOk==VRAI){what=1;periDataRead(valf);                              // data_par__ idem data_save_ + json params ; réponse idem data_save_
                       periPassOk=FAUX;}break;
              case 24: {what=4;                                                                     // (lignes-regles) submit peri_inp__ set periCur raz cb
                       getPeriCurValf(PERILOAD);                                                    // periCur à jour via formIntro/peri_cur__
                       uint8_t nuinp=*(libfonctions+2*i+1)-PMFNCVAL;
                       uint16_t offs=nuinp*PERINPLEN;
                       *(uint8_t*)(periInput+2+offs)&=(uint8_t)PERINPACT_MS;
                       //*(byte*)(periInput+((uint8_t)(*(libfonctions+2*i+1))-PMFNCVAL)*PERINPLEN+2)&=PERINPACT_MS;  // effacement cb (oldlev/active/edge/en)
                       //Serial.print(nuinp);Serial.print("=====");Serial.print(offs);Serial.print(" ");Serial.println((char)*(periInput+2+offs),HEX);
                       }break;                                                                      
              case 25: break;                                                                       // dispo  
              case 26: break;                                                                       // dispo  
              case 27: break;                                                                       // dispo  
              case 28: cfgRemoteHtml(cli);remotePrint();break;                                      // bouton remotecfg_
              case 29: testHtml(cli);break;                                                         // bouton testhtml
              case 30: timersCtlHtml(cli);break;                                                    // timersctl_
              case 31: what=4;                                                                      // (pulses swCtlTableHtml/en-tête) peri_t_sw_ 
                       getPeriCurValf(PERILOAD);                                                    // periCur à jour via formIntro/peri_cur__
                       memset(periSwPulseCtl,0x00,PCTLLEN);                                         // effact bits cb otf enable pulses et free run 
                       Serial.print("periCur=");Serial.println(periCur);
                       periLoad(periCur);
                       break;  
              case 32: {uint8_t pu=*(libfonctions+2*i)-PMFNCHAR,b=*(libfonctions+2*i+1);            // (pulses cb swCtlTableHtml) peri_otf__ bits généraux (FOT)
                        uint16_t sh=0;
                        switch (b){
                           case 'F':sh=PMFRO_VB;break;      // Free run
                           case 'O':sh=PMTOE_VB;break;      // pulse One enable
                           case 'T':sh=PMTTE_VB;break;      // pulse Two enable
                           default:break;
                        }
                        sh=sh<<pu*PCTLBIT;
                        *(uint16_t*)periSwPulseCtl|=sh;
                       }break;       
              case 33: {uint8_t nfct=*(libfonctions+2*i)-PMFNCHAR,nuinp=*(libfonctions+2*i+1)-PMFNCVAL;   // (regles switchs) p_inp1__  
                        uint16_t offs=nuinp*PERINPLEN;                                                    // (enable/type/num detec/action)
                        uint16_t vl=0;
                        // pericur est à jour via peri_inp_
                        switch (nfct){
                          case 1:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPEN_VB;break;               // enable
                          case 2:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPOLDLEV_VB;break;           // prev level
                          case 3:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPDETES_VB;break;            // edge/static
                          case 4:inpsub((periInput+offs),PERINPNT_MS,PERINPNTLS_PB,inptyps+2,2);break;    // type src
                          case 5:conv_atob(valf,&vl);if(vl>NBDSRV){vl=NBDSRV;}
                                 *(periInput+offs)&=~PERINPV_MS;
                                 *(periInput+offs)|=(uint8_t)(vl<<PERINPNVLS_PB);break;                   // num detec src
                          case 6:if(((*(periInput+3+offs)&PERINPNT_MS)>>PERINPNTLS_PB)==DETYEXT){
                                   sourceDetServ[offs/PERINPLEN]=0;}     // si la dest  était un detServ il faut effacer sourceDetServ
                                 transferVal=(uint16_t)inpsub((periInput+3+offs),PERINPNT_MS,PERINPNTLS_PB,inptypd+2,2);break;    // type dest
                          case 7:conv_atob(valf,&vl);if(vl>NBDSRV){vl=NBDSRV;}
                                 *(periInput+3+offs)&=~PERINPV_MS;
                                 *(periInput+3+offs)|=(uint8_t)(vl<<PERINPNVLS_PB);
                                 if(transferVal==DETYEXT){setSourceDet(vl,MDSPER,periCur);}  // si la dest est un detServ màj sourceDetServ (periCur from peri_inp_)
                                 break;                                                                           // num detec dest                                 
                          case 8:inpsub((periInput+2+offs),PERINPACT_MS,PERINPACTLS_PB,inpact+2,LENTACT);break;   // action
                          case 9:*(uint8_t*)(periInput+offs+2)|=(uint8_t)PERINPVALID_VB;break;                    // active level
                          case 10:getPeriCurValf(PERILOAD);                                                       // bouton raz
                                 //dumpstr((char*)periInput,NBPERRULES*PERINPLEN);
                                 memset(periInput+offs,0x00,PERINPLEN);what=4;break; 
                          case 11:getPeriCurValf(PERILOAD);                                                       // bouton ins
                                 Serial.println(nuinp);
                                 for(uint8_t i=NBPERRULES-1;i>nuinp;i--){
                                  //Serial.print(i);Serial.print(' ');dumpstr((char*)(periInput+(i-1)*PERINPLEN),(NBPERRULES-i)*PERINPLEN);
                                  memcpy(periInput+i*PERINPLEN,periInput+(i-1)*PERINPLEN,PERINPLEN);}
                                 memset(periInput+offs,0x00,PERINPLEN);what=4;break;
                          case 12:getPeriCurValf(PERILOAD);                                                       // bouton del
                                 memcpy(periInput+offs,periInput+offs+PERINPLEN,PERINPLEN*(NBPERRULES-nuinp-1)); 
                                 memset(periInput+(NBPERRULES-1)*PERINPLEN,0x00,PERINPLEN);what=4;break;
                          default:break;
                        }
                        //Serial.print(nuinp);Serial.print("=====");Serial.print(nfct);Serial.print(" ");Serial.println((char)*(periInput+2+offs),HEX);
                       }break;                                                                      
              case 34: Serial.println("\n£££££££ne doit pas se produire££££££££\n");mail("p_inp2_ s'est produit","");break; 
              case 35: {int pu=*(libfonctions+2*i)-PMFNCHAR;                                            // (pulses) peri_sw_nx Pulse one/two
                        char puNb=*(libfonctions+2*i+1);  // 'O'ou 'T'
                        uint32_t* pulseNb=periSwPulseOne;                                          
                        switch(puNb){
                          case 'O': pulseNb=periSwPulseOne;*(pulseNb+pu)=0;*(pulseNb+pu)=(uint32_t)convStrToInt(valf,&j);break;
                          case 'T': pulseNb=periSwPulseTwo;*(pulseNb+pu)=0;*(pulseNb+pu)=(uint32_t)convStrToInt(valf,&j);break;
                          case 'V': periSrc=0;conv_atob(valf,&periSrc);
                          Serial.print("******* ");Serial.println(periSrc);
                          break;                           // srce periph for copy
                          default : break;
                        }
                       }break;                                                                      
              case 36: what=15;                                                                         // (antim___) analog timers
                        {uint8_t av=*(libfonctions+2*i);                                        
                        uint8_t nt=*(libfonctions+2*i+1)-PMFNCHAR;if(nt>=NBANTIMERS){nt=NBANTIMERS-1;}  // n° anTimer
                        switch(av){
                          case 'n': alphaTfr(analTimers[nt].nom,LENTIMNAM,valf,nvalf[i+1]-nvalf[i]);    // (antim___n_) nom 
                                    //Serial.print((char*)&analTimers[nt].nom);Serial.print('|');Serial.print(valf);Serial.print('|');
                                    analTimers[nt].dw=0x00;                                                   // init dw
                                    analTimers[nt].cb=0x00;                                                   // init cb 
                                    break;
                          case 'i': analTimers[nt].detecIn=0;analTimers[nt].detecIn=convStrToInt(valf,&j);    // (antim___i_) n° detecteur in
                                    break;     
                          case 'o': analTimers[nt].detecOut=0;analTimers[nt].detecOut=convStrToInt(valf,&j);  // (antim___o_) n° detecteur out
                                    break;
                          case 'O': analTimers[nt].offset=0;analTimers[nt].offset=convStrToNum(valf,&j);      // (antim___O_) offset
                                    break;
                          case 'F': analTimers[nt].factor=0;analTimers[nt].factor=convStrToNum(valf,&j);      // (antim___F_) factor
                                    break;
                        }
                        //Serial.print("ndof:");Serial.print((char)av);Serial.print('/');Serial.println(nt);
                       }break;                                                                          
              case 37: what=15;                                                                         // (antim_cb ) analog timers
                        {uint8_t av=*(libfonctions+2*i)-PMFNCHAR;                                       
                        uint8_t nt=*(libfonctions+2*i+1)-PMFNCHAR;if(nt>=NBANTIMERS){nt=NBANTIMERS-1;}  // n° anTimer
                        //Serial.print("cb:");Serial.print((char)(av+PMFNCHAR));Serial.print('/');Serial.println(nt);
                          /* dw 0-7*/
                          if(av>=0 && av<=7){
                            analTimers[nt].dw|=maskbit[1+2*(7-av)];                          
                          }                                                               
                          switch(av){
                            case 0: analTimers[nt].dw=0xFF;break;                                       // (anti_cb0 ) dw 0-7
                            case 8: analTimers[nt].cb|=maskbit[1+ANT_BIT_ENABLE*2];break;               // (anti_cb8 ) enable
                            case 9: analTimers[nt].cb|=maskbit[1+ANT_BIT_ANTEPOST*2];break;             // (anti_cb9 ) ante/post
                            default: break;
                          }
                        }break;                                                                           
              case 38:what=15;                                                                          // (antim_hh ) analog timers heures
                        {uint8_t av=*(libfonctions+2*i)-PMFNCHAR;if(av>=NBEVTANTIM){av=NBEVTANTIM-1;}                                        
                        uint8_t nt=*(libfonctions+2*i+1)-PMFNCHAR;if(nt>=NBANTIMERS){nt=NBANTIMERS-1;}  // n° anTimer
                        pack(valf,&analTimers[nt].heure[av*3],6,false);
                        //Serial.print("hh:");Serial.print(valf);Serial.print('|');for(uint8_t h=0;h<3;h++){Serial.print((char)*(&analTimers[nt].heure[av]+h),HEX);
                        //Serial.print(',');}Serial.print("-");Serial.print((char)(av+PMFNCHAR));Serial.print('/');Serial.println(nt);                                            // heures
                        }break;
              case 39: what=15;                                                                         // (antim_vv ) analog timers valeurs
                        {uint8_t av=*(libfonctions+2*i)-PMFNCHAR;if(av>=NBEVTANTIM){av=NBEVTANTIM-1;}             
                        uint8_t nt=*(libfonctions+2*i+1)-PMFNCHAR;if(nt>=NBANTIMERS){nt=NBANTIMERS-1;}  // n° anTimer
                        analTimers[nt].valeur[av]=0;conv_atob(valf,&(analTimers[nt].valeur[av]));         // valeurs
                        //Serial.print("vv:");Serial.print((char)(av+PMFNCHAR));Serial.print('/');Serial.print(nt);Serial.print('=');Serial.println(analTimers[nt].valeur[av]);
                        }break;
              case 40: anTimersHtml(cli);break;                                                         // (bouton antimcfg__)
              case 41: what=10;memcpy(bakDetServ,memDetServ,MDSLEN);
                       memset(memDetServ,0x00,MDSLEN);                                                  // (dsrv_init_) bouton submit detecteurs serveur ; effct cb
                       break;
              case 42: {int nb=*(libfonctions+2*i+1)-PMFNCVAL;                                          // (mem_dsrv__) set det bit
                       uint8_t mi=nb>>3;memDetServ[mi] |= mDSmaskbit[MDSLEN*nb+mi];
                       }break;
              case 43: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) ssid[libf+1]
                       alphaTfr(ssid+nb*(LENSSID+1),LENSSID,valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 44: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) passssid[libf+1]
                       alphaTfr(passssid+nb*(LPWSSID+1),LPWSSID,valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 45: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) usrname[libf+1]
                       alphaTfr(usrnames+nb*(LENUSRNAME+1),LENUSRNAME,valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 46: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) usrpass[libf+1]
                       alphaTfr(usrpass+nb*(LENUSRPASS+1),LENUSRPASS,valf,nvalf[i+1]-nvalf[i]);
                       }break;                       
              case 47: if(memcmp((usrnames+usernum*LENUSRNAME),"admin",5)==0){cfgServerHtml(cli);}      // bouton config cfgserv___
                       break;                                                        
              case 48: break;                                                                           // null_fnct_ rien à faire (permet les boutons muets)
              case 49: what=6;                                                                         
                       {uint8_t nC=*(libfonctions+2*i)-PMFNCVAL;                                        // num concentrateur             
                        int rr=0;uint16_t aa=0;  
                          switch(*(libfonctions+2*i+1)){                                            
                            case '_': break;                                                            // (config) percocfg__ bouton Màj périfs concentrés 
                            case 'I': memset((concIp+4*nC*sizeof(byte)),0x00,4);                        // (config) concIp
                                      textIp((byte*)valf,concIp+4*nC*sizeof(uint8_t));break;
                            case 'P': *(concPort+nC)=0;conv_atob(valf,(concPort+nC));break;             // (config) concPort
                            case 'M': for(j=0;j<6;j++){conv_atoh(valf+j*2,(concMac+MACADDRLENGTH*nC+j));}break;          // (config) concMac
                            case 'R': alphaTfr((char*)(concRx+nC*RADIO_ADDR_LENGTH),RADIO_ADDR_LENGTH,valf,nvalf[i+1]-nvalf[i],0);break;  // (config) Radio RX Addr
                            case 'C': *(concChannel+nC)=0;conv_atob(valf,(concChannel+nC));break;       // (config) concchannel
                            case 'S': *(concRfSpeed+nC)=0;conv_atob(valf,(concRfSpeed+nC));break;       // (config) concRfSpeed
                            case 'N': *concNb=0;conv_atob(valf,&aa);if(aa>MAXCONC){aa=0;}*concNb=(uint8_t)aa;                                      
                                      Serial.print(" ");Serial.println(*concNb);break;                  // (config) N° conc pour périf
                            case 'k': *concPeriParams=*valf-PMFNCVAL;break;                             // (config) keep(0)/new(1)
                            case 'c': alphaTfr((char*)periRxAddr,RADIO_ADDR_LENGTH,valf,nvalf[i+1]-nvalf[i],0);break;      // (config) periRxAddr
                            case 'y': *vFactor=0;*vFactor=convStrToNum(valf,&rr)/10000;break;           // (config) voltsFactor
                            case 'v': *vOffset=0;*vOffset=convStrToNum(valf,&rr);break;                 // (config) voltsOffset
                            case 'b': *thFactor=0;*thFactor=convStrToNum(valf,&rr)/10000;break;         // (config) tempFactor
                            case 'e': *thOffset=0;*thOffset=convStrToNum(valf,&rr);break;               // (config) voltsOffset
                            default: break;
                          }
                       }break;                                      
              case 50:  memset(peripass,0x00,LPWD);memcpy(peripass,valf,nvalf[i+1]-nvalf[i]);break;     // (config) peripcfg__ // submit depuis cfgServervHtml                              
              case 51:  what=6;{                                                                        // (config) ethcfg___
                          uint16_t v1=0;
                          switch(*(libfonctions+2*i+1)){                                            
                            case 'i': memset(localIp,0x00,4);                                           // (config) localIp
                                      textIp((byte*)valf,localIp);break;   
                            case 'p': *perifPort=0;conv_atob(valf,perifPort);break;                     // (config) perifPort
                            case 'y': *browserPort=0;conv_atob(valf,browserPort);break;                 // (config) browserPort
                            case 't': *remotePort=0;conv_atob(valf,remotePort);break;                   // (config) remotePort
                            case 'u': udpPort[0]=0;conv_atob(valf,&udpPort[0]);break;                   // (config) serverUdpPort 1
                            case 'U': udpPort[1]=0;conv_atob(valf,&udpPort[1]);break;                   // (config) serverUdpPort 2
                            case 'm': for(j=0;j<6;j++){conv_atoh(valf+j*2,(mac+j));}break;              // (config) mac
                            case 'q': *maxCxWt=0;conv_atobl(valf,maxCxWt);break;                        // (config) TO sans TCP
                            case 'r': *maxCxWu=0;conv_atobl(valf,maxCxWu);break;                        // (config) TO sans UDP
                            case 'x': conv_atob(valf,&v1);*openSockScan=(uint8_t)v1;break;              // (config) open sockets scan dly
                            case 'z': conv_atob(valf,&v1);*openSockTo=(uint8_t)v1;break;                // (config) To open Sockets
                            case 's': alphaTfr(serverName,LNSERV,valf,nvalf[i+1]-nvalf[i]);break;       // (config) nom serveur
                            case 'W': *ssid1=0;*ssid1=*valf-PMFNCVAL;break;                             // (config) ssid1
                            case 'w': *ssid2=0;*ssid2=*valf-PMFNCVAL;break;                             // (config) ssid2
                            case 'v': alphaTfr(thermoPrev,15,valf,nvalf[i+1]-nvalf[i]);break;           // (config) thermo prev

                            default: break;
                          }
                        }break;
              case 52: what=8;                                                                          // submit depuis cfgRemotehtml
                       {int nb=*(libfonctions+2*i+1)-PMFNCHAR;
                        uint16_t v1=0;
                        char a=*(libfonctions+2*i);
                        switch(a){                                               
                            case 'n': alphaTfr(remoteN[nb].nam,LENREMNAM,valf,nvalf[i+1]-nvalf[i]);     // (remotecf) nom remote courante              
                                      remoteN[nb].multRem=0;remoteN[nb].detec=0;break;
                            case 'g': remoteN[nb].multRem=*valf-PMFNCHAR;break;                         // (remotecf) multiple table noms
                            case 'h': remoteN[nb].detec=convStrToInt(valf,&j);                          // (remotecf) n° detecteur on/off table noms
                                      if(remoteN[nb].detec>NBDSRV){remoteN[nb].detec=NBDSRV;}break;     
                            case 'j': remoteN[nb].enable=convStrToInt(valf,&j);                          // (remotecf) n° detecteur enable table noms
                                      if(remoteN[nb].enable>2){remoteN[nb].enable=0;}break;     
                            case 'k': remoteN[nb].butModel=*valf-PMFNCVAL;break;                        // (remotecf) modèle bouton table noms
                            case 'u': remoteT[nb].num=convStrToInt(valf,&j);                            // (remotecf) N° remote table sw
                                      break;                                                            // (remotecf) effacement cb table switch                                        
                            case 'p': conv_atob(valf,&v1);remoteT[nb].peri=(uint8_t)v1;break;           // (remotecf) n° peri table switchs
                            case 's': conv_atob(valf,&v1);remoteT[nb].sw=(uint8_t)v1;                   // (remotecf) n° sw
                                      if(remoteT[nb].peri < NBPERIF && remoteT[nb].peri>0 ){            // controle validité peri/sw
                                        periLoad(remoteT[nb].peri);periCur=remoteT[nb].peri;
                                        if(remoteT[nb].sw < *periSwNb){break;}                          // ok
                                      }
                                      remoteT[nb].peri=0;remoteT[nb].sw=0;break;                        // ko
                            case 'v': conv_atob(valf,&v1);if(v1==0 || remoteN[v1-1].multRem){           // (remotecf) n° remote multiple table switchs
                                      remoteT[nb].multRem=(uint8_t)v1;}
                                      break;
                            default:break;
                          }
                       }break;
              case 53:  what=0;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                  // submit depuis remoteHtml (disjoncteurs/push/slider)
                          char nf=*(libfonctions+2*i);                                                                                                       // si nb= n° remoteN faire +1 (remoteN[1->n])
                          switch(nf){                                               
                            case 'a': disjValue(0,nb,*valf-PMFNCHAR);break;                             // (remote_ca) 1ère position disjoncteur (disjoncté)
                            case 'b': disjValue(1,nb,*valf-PMFNCHAR);break;                             // (remote_cb) 2nde position disjoncteur (on)
                            case 'c': disjValue(2,nb,*valf-PMFNCHAR);break;                             // (remote_cc) 2nde position disjoncteur (forcé)
                            case 'r': break;                                                            // (remote_cr) remoteHtml seul 
                            case 't': break;                                                            // (remote_ct) accès fenêtre one_shot
                            case 'u': pushSliderRemote(cli,nb);break;                                   // (remote_cu) push/slider
                            default:break;
                          }
                          if(nf!='t'){remoteHtml(cli);}
                          else remoteTimHtml(cli,nb);
                        }break;                                                                       
              case 54:  what=0;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                  // submit depuis remoteTimHtml
                          char nf=*(libfonctions+2*i);                                                  // si nb= n° remoteN faire +1 (remoteN[1->n])
                          Serial.print(">===>");Serial.print(nb);Serial.print(' ');Serial.print(valf-PMFNCHAR);Serial.print(' ');Serial.println(nf);
                          switch(nf){                                               
                            case 'a': remoteN[nb].osEnable=0;break;                                     // (remote_oa) 1ère position disjoncteur (disjoncté)
                            case 'b': remoteN[nb].osEnable=1;break;                                     // (remote_ob) 2nde position disjoncteur (on)
                            case 'c': remoteN[nb].osEnable=2;break;                                     // (remote_oc) 2nde position disjoncteur (forcé)
                            case 'd': remoteN[nb].osStatus=0;                                           // (remote_od) stop
                                      osRemInit(nb);
                                      break;
                            case 'e': remoteN[nb].osStatus=1;                                           // (remote_oe) pause
                                      char remT[LDATEA];memset(remT,'0',LDATEA);memcpy(remT+8,remoteN[nb].osRemT,7);
                                      subTime(remT,remoteN[nb].osEndDate,now,VRAI);
                                      memcpy(remoteN[nb].osRemT,remT+8,6);
                                      //Serial.print(">==========");Serial.print(remoteN[nb].osEndDate);Serial.print('-');Serial.print(now);Serial.print('=');Serial.println(remoteN[nb].osRemT);
                                      break;                                     
                            case 'f': remoteN[nb].osStatus=2;                                           // (remote_of) start
                                      disjValue(remoteN[nb].osEnable,nb,*valf-PMFNCHAR);                // chargement état one_shot
                                      //Serial.print(">==========");Serial.print(remoteN[nb].osDurat);Serial.print(' ');Serial.println(remoteN[nb].osRemT);
                                      if(*remoteN[nb].osDurat!=0 && *remoteN[nb].osRemT==0){
                                        memcpy(remoteN[nb].osRemT,remoteN[nb].osDurat,7);}
                                      char durat[LDATEA];memset(durat,'0',14);memcpy(durat+8,remoteN[nb].osRemT,7);
                                      addTime(remoteN[nb].osEndDate,now,durat,VRAI);
                                      //Serial.print(">==========");Serial.print(now);Serial.print('+');Serial.print(durat);Serial.print('=');Serial.println(remoteN[nb].osEndDate);
                                      break;
                            case 't': textfonc(remoteN[nb].osDurat,6);
                                      //Serial.print(">===<");Serial.print(remoteN[nb].osDurat);Serial.print(' ');Serial.println(valf);
                                      break;                                                            // (remote_ot) duration 
                            default:break;
                          }
                          if(remoteN[nb].osEnable==0){*remoteN[nb].osRemT='\0';}                        
                          remoteTimHtml(cli,nb);
                        }break;                                                     
              case 55:  break;                                                                          // dispo
              case 56:  what=60;
                        switch (*(libfonctions+2*i+1)){                                                 // mailcfg___
                          case 'f':alphaTfr(mailFromAddr,LMAILADD,valf,nvalf[i+1]-nvalf[i]);break;      // (config) mailFrom
                          case 'w':alphaTfr(mailPass,LMAILPWD,valf,nvalf[i+1]-nvalf[i],0);break;        // (config) pwd mailFrom
                          case '1':alphaTfr(mailToAddr1,LMAILADD,valf,nvalf[i+1]-nvalf[i]);break;       // (config) mailTo1
                          case '2':alphaTfr(mailToAddr2,LMAILADD,valf,nvalf[i+1]-nvalf[i]);break;       // (config) mailTo2              
                          case 'p':conv_atob(valf,periMail1);                                           // (config) peri1
                                   if(*periMail1>NBPERIF){*periMail1=NBPERIF;}     
                          case 'q':conv_atob(valf,periMail2);                                           // (config) peri2
                                   if(*periMail2>NBPERIF){*periMail2=NBPERIF;}     
                          default:break;
                        } 
                        break;
              case 57:  what=12;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;char nf=*(libfonctions+2*i);      // submit depuis thparams__ (thermosCfg())
                        //Serial.print("cfgTh lf+1/lf=");Serial.print(nb);Serial.print(" lf=");Serial.println(nf);
                          switch (nf){
                            case 'n':alphaTfr((char*)&thermos[nb].nom,LENTHNAME,valf,nvalf[i+1]-nvalf[i]);
                                   thermos[nb].lowenable=0;                                                                 // effacement cb
                                   thermos[nb].highenable=0;
                                   thermos[nb].lowstate=0;
                                   thermos[nb].highstate=0;
                                   break;
                            case 'p':thermos[nb].peri=0;thermos[nb].peri=convStrToInt(valf,&j);
                                   if((thermos[nb].peri)>NBPERIF){(thermos[nb].peri)=NBPERIF;}
                                   break;
                            case 'e':thermos[nb].lowenable=*valf-PMFNCVAL;break;                                            // enable low
                            case 'E':thermos[nb].highenable=*valf-PMFNCVAL;break;                                           // enable high
                            case 's':thermos[nb].lowstate=*valf-PMFNCVAL;break;                                             // state  low
                            case 'S':thermos[nb].highstate=*valf-PMFNCVAL;break;                                            // state  high
                            case 'v':thermos[nb].lowvalue=0;thermos[nb].lowvalue=(int16_t)convStrToInt(valf,&j);break;      // value low
                            case 'V':thermos[nb].highvalue=0;thermos[nb].highvalue=(int16_t)convStrToInt(valf,&j);break;    // value high
                            case 'o':thermos[nb].lowoffset=0;thermos[nb].lowoffset=(int16_t)convStrToInt(valf,&j);break;    // offset low
                            case 'O':thermos[nb].highoffset=0;thermos[nb].highoffset=(int16_t)convStrToInt(valf,&j);break;  // offset high
                            case 'd':sourceDetServ[remoteN[nb].detec]=0;                                                    // det low
                                   thermos[nb].lowdetec=0;thermos[nb].lowdetec=convStrToInt(valf,&j);
                                   if(thermos[nb].lowdetec>NBDSRV){thermos[nb].lowdetec=NBDSRV;}
                                   if(thermos[nb].lowdetec!=0){setSourceDet(thermos[nb].lowdetec,MDSTHE,nb+1);}
                                   break;                                                                                 
                            case 'D':thermos[nb].highdetec=0;thermos[nb].highdetec=convStrToInt(valf,&j);                   // det high
                                   if(thermos[nb].highdetec>NBDSRV){thermos[nb].highdetec=NBDSRV;}
                                   if(thermos[nb].highdetec!=0){setSourceDet(thermos[nb].highdetec,MDSTHE,nb+1);}
                                   break;                                                                                                           
                            default:break;
                          } 
                        }break;
              case 58:  switch(*(libfonctions+2*i+1)){                                                   // thermoshow
                          /*
                          case 's': if(scalcTh(now,thermoLastBeg,thermoCurPrev)==SDOK){                  // update periphériques
                                      memcpy(thermoLastScan,now,15);thermoLastScan[15]='\0';
                                      memcpy(thermoLastPrev,thermoPrev,15);thermoLastPrev[15]='\0';
                                      configSave();}
                                    break;
                                    */
                          case 'p': alphaTfr(thermoCurPrev,15,valf,nvalf[i+1]-nvalf[i]);
                                    if(scalcTh(now,thermoLastBeg,thermoCurPrev)==SDOK){                  // update periphériques
                                      memcpy(thermoLastScan,now,15);thermoLastScan[15]='\0';
                                      memcpy(thermoLastPrev,thermoPrev,15);thermoLastPrev[15]='\0';
                                      configSave();}
                                    break;
                          default:  break;
                        }
                        thermoShowHtml(cli);break;                                                       // thermoshow
              case 59:  thermoCfgHtml(cli);break;                                                        // thermos___ (bouton thermo_cfg)
              case 60:  what=0;{uint8_t nt=*(libfonctions+2*i+1)-PMFNCHAR;
                          timersN[nt].enable=!timersN[nt].enable;
                        }
                        timersCtlHtml(cli);
                        break;                                                                           // (timersCtl) tim_ctl___
              case 61:  what=7;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                   // (timers) tim_name__
                        textfonc(timersN[nb].nom,LENTIMNAM);
                       //Serial.print("efface cb timers ");Serial.print(nb);Serial.print(" ");Serial.print(timersN[nb].nom);
                        timersN[nb].enable=0;                                                            // (timers) effacement cb     
                        timersN[nb].perm=0;
                        timersN[nb].cyclic_=0;
                        timersN[nb].curstate=0;
                        timersN[nb].forceonoff=0;
                        timersN[nb].dw=0;
                        }break;
              case 62:  {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_det___      
                        sourceDetServ[timersN[nb].detec]=0;                            
                        timersN[nb].detec=convStrToInt(valf,&j);
                        if(timersN[nb].detec>NBDSRV){timersN[nb].detec=NBDSRV;}
                        setSourceDet(timersN[nb].detec,MDSTIM,nb+1);
                        }break;
              case 63:  {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_hdf___
                          switch (*(libfonctions+2*i)){         
                            case 'd':textfonc(timersN[nb].hdeb,6);break;
                            case 'f':textfonc(timersN[nb].hfin,6);break;
                            case 'p':textfonc(timersN[nb].onStateDur,14);                                // tim_hdf_p
                                    unixCyclicTimersOnState[nb]=alphaDateToUnix(timersN[nb].onStateDur,false);break;                          
                            case 'P':textfonc(timersN[nb].offStateDur,14);                               // tim_hdf_P
                                    unixCyclicTimersOffState[nb]=alphaDateToUnix(timersN[nb].offStateDur,false);break;
                            case 'b':textfonc(timersN[nb].dhdebcycle,14);break;
                            case 'e':textfonc(timersN[nb].dhfincycle,14);break;
                            case 's':textfonc(timersN[nb].dhLastStart,14);break;
                            case 'S':textfonc(timersN[nb].dhLastStop,14);break;
                            default:break;
                          } 
                        }break;
              case 64:  {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_chkb__
                        int nv=*(libfonctions+2*i)-PMFNCHAR;
                          /* e_p_c_f */
                          switch (nv){         
                            case 0:timersN[nb].enable=*valf-'0';break;
                            case 1:timersN[nb].perm=*valf-'0';break;
                            case 2:timersN[nb].cyclic_=*valf-'0';if(timersN[nb].cyclic_==1){cyclicTimersInit(nb);};break;
                            default:break;
                          }
                          /* dw */
                          if(nv==NBCBTIM){timersN[nb].dw=0xFF;}
                          if(nv>NBCBTIM){
                            timersN[nb].dw|=maskbit[1+2*(7-nv+NBCBTIM)];                          
                          }                         
                        }break;
              case 65:  Serial.println("timersHtml()");timersHtml(cli);break;                            // timershtml
              case 66:  Serial.println("cfgDetServHtml()");cfgDetServHtml(cli);break;                    // cfgdetservhtml              
              case 67:  what=11;{int nb=*(libfonctions+2*i+1)-PMFNCVAL;                                  // lib detserv
                        alphaTfr(&libDetServ[nb][0],LENLIBDETSERV,valf,nvalf[i+1]-nvalf[i]);
                        }break;                          
              case 68:  getPeriCurLibf(PERILOAD);                                                        // (showline) bouton periph
                        periLineHtml(cli);
                        break;                                                                                                    
              case 69:  break;                                                                           // done         
              case 70:  {uint16_t val=0;
                        switch(*(libfonctions+2*i)){                                                     // analog_
                          case '@': conv_atob(valf,&val);                                                // limite analogique basse
                                    *periAnalLow=0;*periAnalLow+=val;break;             
                          case 'A': conv_atob(valf,&val);                                                // limite analogique haute
                                    *periAnalHigh=0;*periAnalHigh+=val;break;           
                          case 'B': *periAnalOffset1=0;conv_atob(valf,periAnalOffset1);break;
                          case 'C': *periAnalFactor=0;*periAnalFactor=convStrToNum(valf,&j);break;
                          case 'D': *periAnalOffset2=0;*periAnalOffset2=convStrToNum(valf,&j);break;
                          default: break;
                        }
                        }break;
              case 71:  what=13;rulesfonc(periAnalCb,periAnalDestDet,periAnalRefDet,periAnalMemo);break;       // anrul___ analog input rules
              case 72:  what=13;rulesfonc(periDigitCb,periDigitDestDet,periDigitRefDet,periDigitMemo);break;   // dgrul___ digital input_rules
              case 73:  {uint8_t nf=*(libfonctions+2*i+1)-PMFNCHAR;                                      // bouton submit rul_init__                                                                                                        
                                                                                                         // n° de la fonction qui utilise rul_init__
                          periCur=0;conv_atob(valf,&periCur);                                            // (0-n, 0=analog input rules ; 1=digital)                                                                                                        
                          if(periCur>NBPERIF){periCur=NBPERIF;}periLoad(periCur); 
                       //Serial.print(" fonct 73======");periPrint(periCur);
                          uint8_t* cb=periAnalCb;
                          int8_t* memo=periAnalMemo;                                                     // effacement check box et memos précédents
                          uint8_t ncb=0;
                          switch(nf){
                            case 0: cb=periAnalCb;ncb=5;memo=periAnalMemo;break;
                            case 1: cb=periDigitCb;ncb=4;memo=periDigitMemo;break;
                            default: break;
                          }
                          if(nf<2){
                            memset(cb,0x00,ncb);                                                         // cb
                            for(uint8_t nm=0;nm<ncb;nm++){                                               // memos
                              if(memo[nm]<NBMEMOS && memo[nm]>=0){memset(memosTable+memo[nm]*LMEMO,0x00,LMEMO);memo[nm]=-1;}
                            }
                          }
                        }break;                                                                                                        
              case 74:  what=0;htmlFavicon(cli);
                        break;
              
              /* fin des fonctions */
              default:break;
              }
          
            }     // i<NBVAL
          
          }       // fin boucle nbre params
          Serial.print((unsigned long)millis());
          Serial.print(" what=");Serial.print(what);
          Serial.print(" periMess=");Serial.print(periMess);
          Serial.print(" periSrc=");Serial.print(periSrc);
          Serial.print(" periCur=");Serial.println(periCur);
#ifdef SHDIAGS            
          Serial.print(" strHisto=");Serial.print(strHisto);
#endif //            
          char aabb[2]={ab,'\0'};
          histoStore_textdh(aabb,"",strHisto);

          //periMess=MESSOK;
          // what==99 pour accueil
          //if(what==0){Serial.print("!*!*!");}
          switch(what){                                           
            case 0: break;                                                
            case 1: if(periMess==MESSOK){                                 // data_save (si periMess KO, periDataRead ne s'est pas exécuté et periCur n'est pas valorisé)
                      periMess=periAns(cli,udpCli,"ack_______");}break;   // donc pas de periAns possible
            case 2: Serial.print("ab=");Serial.println(ab);
                    if(ab=='c'){periTableHtml(cli);}                      // peritable suite à login
                    if(ab=='b'){remoteHtml(cli);}                         // remote    suite à login
                    break;
            case 3: if(periMess==MESSOK){                                 // data_read (si periMess KO, periDataRead ne s'est pas exécuté et periCur n'est pas valorisé)
                      periMess=periAns(cli,udpCli,"set_______");}break;   // donc pas de periAns possible
            case 4: periMess=periSave(periCur,PERISAVESD);                // switchs
                    swCtlTableHtml(cli);
                    cliext.stop();periMess=periReq(&cliext,periCur,"set_______");break;
            case 5: periMess=periSave(periCur,PERISAVESD);                // (periLine) modif ligne de peritable
                    periTableHtml(cli); 
                    cliext.stop();periMess=periReq(&cliext,periCur,"set_______");break;
            case 6: configPrint();configSave();
                    mail("SERV_CONFIG"," ");cfgServerHtml(cli);break;     // config serveur            
            case 60:configPrint();configSave();
                    mail("MAIL_CONFIG"," ");cfgServerHtml(cli);break;     // config mail     
            case 7: timersSave();timersHtml(cli);break;                   // timers
            case 8: remoteSave();cfgRemoteHtml(cli);break;                // bouton remotecfg puis submit
// à réviser
            //case 9: periRemoteUpdate();                                  // bouton remotehtml ou remote ctl puis submit 
            //        periMess=perToSend(tablePerToSend,remotetime);
            //        remoteHtml(cli);break; 
            case 10:memDetSave();periDetecUpdate("pDUma");                // bouton submit détecteurs serveur
                    periMess=perToSend(tablePerToSend,srvdettime);
                    periTableHtml(cli);break;
            case 11:memDetSave();cfgDetServHtml(cli);break;               // bouton cfgdetserv puis submit         
            case 12:thermosSave();thermoCfgHtml(cli);break;               // thermos
            case 13:memosSave(-1);
                    periSave(periCur,PERISAVESD);                         // bouton submit periLine (MàJ/analog/digital)                                             
                    periLineHtml(cli);
                    break;                                                                                                           
            case 14:break;                                                // data_na___ (idem data_save sans réponse serveur)
            case 15:anTimersSave();anTimersHtml(cli);break;               // antim___ & anti_ch_
            case 16:dumpHisto(cli);break;                                 // bouton submit histo show
            case 17:if(periMess==MESSOK){mail("PERIF ",mailData);         // data_mail_
                    periMess=periAns(cli,udpCli,"set_______");}
                    break;
            default:accueilHtml(cli);break;                               // what=-1
          }
        } // getnv nbreparams>=0  
      
        valeurs[0]='\0';                                                            
        
        if(ab!='u'){
          Serial.print(millis());Serial.print(' ');
          Serial.print("-sock stop ");Serial.print(cli->sockindex);Serial.print(" ");                                                              
          if(cli!=cli_debug){Serial.print("cli hs");while(1){trigwd();}}
          cli->stop();            // tcp only ********* !!!!!!!! stop géré en instances multiples pour cli_a 
          cliext.stop();          // en principe rapide : la dernière action est une entrée
        }
                                  // la gestion d'instances multiples ne fonctionne pas avec le navigateur ??
        if(ab=='a'){
          tPSStop[tPS]=millis();if(tPSStop[tPS]==0){tPSStop[tPS]=1;} //heure du stop TCP
        }

        Serial.print(millis());
        Serial.print(" pM=");Serial.print(periDiag(periMess));
        if(what==0){Serial.print(" w0 ");}
        #define LENDM 17
        char endMessage[LENDM];
        memcpy(endMessage," *** end tcp  - ",LENDM-1);endMessage[LENDM-1]='\0';
        if(ab=='u'){memcpy(endMessage+9,"udp",3);endMessage[12]=udpNb+48;}                      // Serial.print(" *** end udp");Serial.print(udpNb+48);Serial.print(" - ");}
        else {endMessage[12]=ab;}                                                               //Serial.print(" *** end tcp");Serial.print(ab);Serial.print(" - ");}
        Serial.print(endMessage);
#ifdef SOCK_DEBUG        
        Serial.println();
        showSocketsStatus(false,true,true,"");
#endif // SOCK_DEBUG
        
        Serial.println(millis()-cxDur);
#ifdef DEBUG_ON
  delay(20);
#endif        
}

/* ***************** serveurs *********************** */

void getremote_IP(EthernetClient* client,uint8_t* ptremote_IP,byte* ptremote_MAC)
{ 
    W5100.readSnDHAR(client->getSocketNumber(), ptremote_MAC);
    W5100.readSnDIPR(client->getSocketNumber(), ptremote_IP);
}

void udpError(EthernetUDP* uu,const char* message,uint16_t udpPacketLen)
{
      Serial.print(message);
      //serialPrintIp(remote_IP);Serial.print("/");Serial.print(remote_Port_Udp);Serial.print('/');Serial.println(udpPacketLen);
      //Serial.println(udpData);
      //Udp.stop();
      uint16_t recLen=udpPacketLen;
      if(recLen>=UDPBUFLEN-1){recLen=UDPBUFLEN-2;}
      uu->read(udpData,udpPacketLen);udpData[udpPacketLen]='\0';
      Serial.print("udpData ko:");Serial.println(udpData);delay(10);

      #define RPULEN 40
      char rpu[RPULEN];memset(rpu,'\0',RPULEN);
      for(uint8_t ii=0;ii<4;ii++){sprintf(rpu+strlen(rpu),"%u",remote_IP[ii]);if(ii<3){*(rpu+strlen(rpu))='.';}}
      *(rpu+strlen(rpu))='/';sprintf(rpu+strlen(rpu),"%u",remote_Port_Udp);
      *(rpu+strlen(rpu))='/';sprintf(rpu+strlen(rpu),"%u",udpPacketLen);
      *(rpu+strlen(rpu))='\0';
      Serial.println(rpu);
        
        udpPacketLen = uu->parsePacket();
        while(udpPacketLen){
          uu->read(udpData,udpPacketLen);
          udpPacketLen = uu->parsePacket();
        }
      
      //mail(message,rpu);
}

void udpPeriServer()
{
  ab='u';
  IPAddress rip;

  for(uint8_t uu=0;uu<2;uu++){

    int udpPacketLen = udp[uu]->parsePacket();
 
    if (udpPacketLen){
      udpSockIndex=udp[uu]->sockindex;
      udpDataLen=udpPacketLen;
      rip = (uint32_t) udp[uu]->remoteIP();
      memcpy(remote_IP,(char*)&rip+4,4);
      remote_Port_Udp = (uint16_t) udp[uu]->remotePort();
      if(remote_IP[0]!=192 || remote_IP[1]!=168 || remote_IP[2]!=0 || (remote_IP[3]!=108 && remote_IP[3]!=31)){
        // ************ conc1 : 31 *** conc2 : 11 ************* DHCP/baux statiques
        udpError(udp[uu],"\nUDP_IP_KO IP/port:",udpPacketLen);
        }
      else {
        if(udpPacketLen<UDPBUFLEN){
          udp[uu]->read(udpData,udpDataLen);udpData[udpDataLen]='\0';
          //Serial.println(udpPacketLen);dumpstr((char*)udpData,128);
          packMac((byte*)remote_MAC,(char*)(udpData+MPOSMAC+33));   // 33= "GET /cx?peri_pass_=0011_17515A29?"
          lastcxu=millis();     // trig watchdog
          udpNb=uu;
          commonserver(nullptr,udp[uu],udpData,udpDataLen);              
        }
        else{
          udpError(udp[uu],"UDP_OVERFLOW IP/len:",udpPacketLen);
        }
      }
    }
  }
}

/* En TCP
*  A la fin de dataRead/dataSave, periAns envoie un message set ou ack ;
*  A la fin des fonctions html, un raffraichissement de page est envoyé ;
*  Le .stop() dure plusieurs centaines de mS ou secondes (protocole http) ;
*  plusieurs instances sont utilisées alternativement pour TcpPeriServer()
*  ce qui permet d'effectuer le .stop() lorsque l'envoi est complété
*  ou au moment d'utiliser l'instance (.available()).
*  La plupart du temps sans perte de disponibilité du processeur.
*/

void tcpPeriServer()
{
  ab='a';
  uint8_t preTPS=tPS+1;                         // attribue l'instance suivante
  if(preTPS>=MAXTPS){preTPS=0;}

  /*
  // available() teste le port de chaque socket du w5500 ; 
  // il peut donc trouver plusieurs sockets avec le même port server (periserv)
  // dont ceux déjà en cours de vidage ou attente de 'stop' 
  
  for(int t=0;t<MAXTPS;t++){                    // effectue les .stop() éventuels pour libérer les instances
                                                // qui ont eu le temps d'effectuer leur dernier .write
    if(tPSStop[t]!=0 && (millis()-tPSStop[t])>600){     // tPSStop heure de la fin d'utilisation de l'instance
                                                        // (dernier .write en principe) si 0 inutilisée
      tPSStop[t]=0;
      //purgeCli(&cli_a[t]);
      cli_a[t].stop();
    }
  }

  if(tPSStop[preTPS]!=0){                       // si instance pas encore libérée -> libération
    unsigned long tStop=millis();               // tStop heure du stop de l'instance   
    cli_a[preTPS].stop();                       // force la libération de l'instance
    if((millis()-tStop)>1){
      Serial.print(loopCnt);Serial.print(" tStop=");Serial.println(millis()-tStop); // libération d'instance avec attente
                                                                                    // ne devrait se produire que rarement
    }
  }
  
  */
  if(cli_a[preTPS] = periserv->available())     // attente d'un client périf
  {
    getremote_IP(&cli_a[preTPS],remote_IP,remote_MAC);  // récupère les coordonnées du serveur DNS ????
    if (cli_a[preTPS].connected()){
      lastcxt=millis();                         // trig soft watchdog
      tPS=preTPS;                               // valide l'instance
      commonserver(&cli_a[tPS],nullptr,nullptr,0);      
    }
    else cli_a[preTPS].stop();
  }
}

void browserServer()
{
  ab='c';

  //cli_c.stop(); normalement déjà effectué dans commonserver
  if(cli_c = browserserv->available())      // attente d'un client browser sur port config
  {
    getremote_IP(&cli_c,remote_IP,remote_MAC);      
    if (cli_c.connected()){
      lastcxt=millis();             // trig watchdog
      commonserver(&cli_c,nullptr,nullptr,0);
    }
    else cli_c.stop();
  }     
}

void remoteServer()
{
  ab='b';

  //cli_b.stop(); normalement déjà effectué dans commonserver
  if(cli_b = remoteserv->available())      // attente d'un client browser sur port remote
  {
    getremote_IP(&cli_b,remote_IP,remote_MAC);      
    if (cli_b.connected()){
      lastcxt=millis();             // trig watchdog
      commonserver(&cli_b,nullptr,nullptr,0);
    }
    else cli_b.stop();
  }     
}

void serialServer()
{
  //while(1){if(Serial1.available()){char a=Serial1.read();Serial.print(":");Serial.print(a);}}       
  
  char serialBuf[MAXSER];
  memset(bec,0x00,LBEC);

  uint16_t lrcv=serialRcv(serialBuf,MAXSER,1);
  
  if(lrcv!=0){
    scanCnt+=100;
    rcvcnt++;
    Serial.print(rcvcnt);Serial.print(" ");Serial.print(lrcv);Serial.print("->");Serial.println(serialBuf);
    
  /* !!!!!!!!! maxi 255 caractères sinon agrandir le buffer de Tx !!!!!!!!!*/

    if(memcmp(serialBuf,WIFICFG,10)==0){
      configExport(bec);
      wifiExport(bec,*ssid1);
      wifiExport(bec,*ssid2);
      setExpEnd(bec);
      Serial.println("wifi ");//dumpstr(bec,300);
    }
    if(memcmp(serialBuf,CONCCFG,10)==0){
      configExport(bec);
      concExport(bec,*concNb);
      periExport(bec,*concNb);
      setExpEnd(bec);
      Serial.println("conc ");//dumpstr(bec,300);      
    }
    if(memcmp(serialBuf,PERICFG,10)==0){
      periImport(serialBuf+10);
      memset(bec,0x00,LBEC);memcpy(bec,"0000;",5);  // len
      periExport(bec,*concNb);
      setExpEnd(bec);
      Serial.println("peri ");//dumpstr(bec,300);      
    }
    
    for(uint8_t i=0;i<TSCNB+1;i++){SERIALX.print(RCVSYNCHAR);}
    for(uint16_t lb=0;lb<strlen(bec);lb++){SERIALX.print(*(bec+lb));delay(1);}
    Serial.print("bec:");Serial.println(bec);
  }
}

void testUdp()
{
#define MAX_LENGTH_TEST 200

  Serial.println("\nlancer le test Udp sur l'autre machine \n");
  
  while(1){
    trigwd(); 
    int packetSize = udp[0]->parsePacket(); 
    IPAddress ipAddr;
    unsigned int rxPort;
    char data[MAX_LENGTH_TEST];
    if (packetSize){
      ipAddr = (uint32_t) udp[0]->remoteIP();
      Serial.print("Received packet of size ");Serial.println(packetSize);
      Serial.print("From ");Serial.print(ipAddr);Serial.print(" ");
    
      rxPort = (unsigned int) udp[0]->remotePort();
      Serial.print(", port ");Serial.println(rxPort);

      if(packetSize<MAX_LENGTH_TEST){
        udp[0]->read(data, packetSize);
        data[packetSize]='\0';
        Serial.print("Contents: ");Serial.println(data);
  
        udp[0]->beginPacket(ipAddr,rxPort);

        char data[]="hello Slave";
        Serial.print("sending (");Serial.print(strlen(data));Serial.print(")>");Serial.print(data);
        Serial.print("< to ");Serial.print(ipAddr);Serial.print(":");Serial.println(rxPort);
  
        udp[0]->write(data,strlen(data));
        udp[0]->endPacket();
      }
      else{Serial.println("paquet trop gros...");udp[0]->flush();}
    }  
  }
}
