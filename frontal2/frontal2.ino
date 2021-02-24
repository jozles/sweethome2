
#include <SPI.h>      //bibliothèqe SPI pour W5100
#include <Ethernet.h> //bibliothèque W5x00 Ethernet
#include <EthernetUdp.h>
#include <Wire.h>     //biblio I2C pour RTC 3231
#include "SdFat.h"
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

SdFat32 sd32;
File32 fhisto;            // fichier histo sd card
File32 fhtml;             // fichiers pages html

//#define _AVEC_AES
#ifdef _AVEC_AES
#include "aes.h"      //encryptage AES
#define KEY {0x2d,0x80,0x17,0x18,0x2a,0xb0,0xd4,0xa8,0xad,0xf9,0x17,0x8a,0x0b,0xD1,0x51,0x3e}
#define IV  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f}
struct AES_ctx ctx;
uint8_t key[16]=KEY,iv[16]=IV;
uint8_t chaine[16+1]={0}; // chaine à encrypter/décrypter ---> void xcrypt()
#endif _AVEC_AES


extern "C" {
 #include "utility/w5100.h"
}

  EthernetClient cli_a;             // client du serveur periphériques et browser configuration
  EthernetClient cli_b;             // client du serveur pilotage
  EthernetClient cliext;            // client de serveur externe  
  EthernetClient cli_udp;           // client inutilisé pour la compatibilité des arguments des fonctions mixtes TCP/UDP

char ab;

  char udpData[UDPBUFLEN];         // buffer paquets UDP
  uint16_t udpDataLen;             // taille paquet contenu
    
  extern EthernetUDP Udp;
    
/*    
    IPAddress test(82,64,32,56);
    IPAddress espdev(192,168,0,38);
    IPAddress esptv(192,168,0,208);
    IPAddress gggg(74,125,232,128);    
*/

//    unsigned long mill=millis();
    
/* >>>> config server <<<<<< */

char configRec[CONFIGRECLEN];

  byte* mac;                  // adresse server
  byte* localIp;              // adresse server
  uint16_t* portserver;       //
  char* nomserver;            //
  char* userpass;             // mot de passe browser
  char* modpass;              // mot de passe modif
  char* peripass;             // mot de passe périphériques
  char* ssid;                 // MAXSSID ssid
  char* passssid;             // MAXSSID password SSID
  int*  nbssid;               // inutilisé
  char* usrnames;             // usernames
  char* usrpass;              // userpass
  unsigned long* usrtime;     // user cx time
  unsigned long* usrpretime;  // user cx time précédent
  uint16_t* toPassword;       // Délai validité password en sec !
  unsigned long* maxCxWt;     // Délai WD TCP
  unsigned long* maxCxWu;     // Délai WD UDP

  byte* configBegOfRecord;
  byte* configEndOfRecord;

// =============================================================== à intégrer dans config ================================
  char mailToAddr[]="pinkasfeld@combox.fr";
  uint16_t periMail=3;  
// =============================================================== à intégrer dans config ================================

  bool    periPassOk=FAUX;  // contrôle du mot de passe des périphériques
  int     usernum=-1;       // numéro(0-n) de l'utilisateur connecté (valide durant commonserver)   

EthernetServer periserv(PORTSERVER);  // serveur perif et table port 1789 service, 1790 devt, 1786 devt2
EthernetServer pilotserv(PORTPILOT);  // serveur pilotage 1792 devt, 1788 devt2

  uint8_t lip[]=LOCALSERVERIP;                       
  uint8_t remote_IP[4]={0,0,0,0};           // periserver
  uint8_t remote_IP_cur[4]={0,0,0,0};       // périphériques periserver
  uint8_t remote_IP_Mac[4]={0,0,0,0};       // maintenance periserver
  uint8_t remote_IPb[4]={0,0,0,0};          // pilotserver
  byte    remote_MAC[6]={0,0,0,0,0,0};      // periserver
  byte    remote_MACb[6]={0,0,0,0,0,0};     // pilotserver
  uint16_t remote_Port=0;                   

  int8_t  numfonct[NBVAL];             // les fonctions trouvées  (au max version 1.1k 23+4*57=251)
  
  char*   fonctions="per_temp__peri_pass_username__password__user_ref__to_passwd_per_refr__peri_tofs_switchs___deco______dump_his__hist_sh___data_save_data_read_peri_tst__peri_cur__peri_refr_peri_nom__peri_mac__accueil___peri_tableperi_prog_peri_sondeperi_pitchperi_inp__peri_detnbperi_intnbperi_rtempremote____testhtml__peri_vsw__peri_t_sw_peri_otf__p_inp1____p_inp2____peri_pto__peri_ptt__peri_thminperi_thmaxperi_vmin_peri_vmax_dsrv_init_mem_dsrv__ssid______passssid__usrname___usrpass___cfgserv___pwdcfg____modpcfg___peripcfg__ethcfg____remotecfg_remote_ctlremotehtmlperi_raz___dispo_____thparams__thermoshowthermoscfgperi_port_tim_name__tim_det___tim_hdf___tim_chkb__timershtmldsrvhtml__libdsrv___periline__done______peri_ana__rul_ana___rul_dig___rul_init__last_fonc_";
  
  /*  nombre fonctions, valeur pour accueil, data_save_ fonctions multiples etc */
  int     nbfonct=0,faccueil=0,fdatasave=0,fperiSwVal=0,fperiDetSs=0,fdone=0,fpericur=0,fperipass=0,fpassword=0,fusername=0,fuserref=0,fperitst=0;
  char    valeurs[LENVALEURS];         // les valeurs associées à chaque fonction trouvée
  uint16_t nvalf[NBVAL];               // offset dans valeurs[] des valeurs trouvées (séparées par '\0')
  char*   valf;                        // pointeur dans valeurs en cours de décodage
  char    libfonctions[NBVAL*2];       // les 2 caractères de fin des noms de fonctions
  int     nbreparams=0;                // 
  int     what=0;                      // ce qui doit être fait après traitement des fonctions (0=rien)

#define LENCDEHTTP 6
  char*   cdes="GET   POST  \0";       // commandes traitées par le serveur
  char    strHisto[RECCHAR]={0};          // buffer enregistrement histo SD
  char*   strHistoEnd="<br>\r\n\0";
  char    buf[6];
  long    fhsize;                      // remplissage fhisto

  char bufServer[LBUFSERVER];          // buffer entrée/sortie dataread/save

  float         oldth=0;               // pour prev temp DS3231
  unsigned long temptime=0;            // last millis() pour temp
#define PTEMP 120                      // secondes
  uint32_t      pertemp=PTEMP;         // période ech temp sur le serveur
  uint16_t      perrefr=0;             // periode rafraichissement de l'affichage

  unsigned long lastcxt=0;             // last TCP server connection for watchdog
  unsigned long lastcxu=0;             // last UDP server connection for watchdog  
   
#define WDSD    "W"                    // Watchdog record
#define TCPWD   "T"                    // TCP watchdog event
#define UDPWD   "U"                    // UDP watchdog event
#define HALTREQ "H"                    // Halt request record
#define TEMP    "T"                    // Temp record
#define RESET   "R"                    // Reset record
#define BOOT    "B"                    // Boot record
  unsigned long cxtime=0;              // durée connexion client
  unsigned long remotetime=0;          // mesure scans remote
  unsigned long srvdettime=0;          // mesure scans détecteurs
  unsigned long timerstime=0;          // last millis pour timers
#define PTIMERS 1                      // secondes
  uint32_t  pertimers=PTIMERS;         // période ctle timers 
  unsigned long thermosTime=0;         // last millis pour thermos
#define PTHERMOS 4                     // secondes
  uint32_t  perThermos=PTHERMOS;       // période ctle thermos
  unsigned long datetime=0;            // last millis() pour date 
#define PDATE 3600*24                  // secondes
  unsigned long perdate=PDATE;         // période ctle date & perisave general
  
  int   stime=0;int mtime=0;int htime=0;
  unsigned long  curdate=0;

/* iùage mémoire détecteurs du serveur */

  uint32_t  memDetServ=0x00000000;    // image mémoire NBDSRV détecteurs (32)  
  char      libDetServ[NBDSRV][LENLIBDETSERV];
  char      mdsSrc[]=" PRHT";
  uint16_t  sourceDetServ[NBDSRV];   // actionneurs (sssnnnnnnnn ss type 000, P 001 perif, R 010 remote, H 011 thermos, T 100 timers / nnnnnnnn n°)
  uint32_t  mDSmaskbit[]={0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                       0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                       0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                       0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};
  uint32_t  bakDetServ;
/*  enregistrement de table des périphériques ; un fichier par entrée
    (voir periInit() pour l'ordre physique des champs + periSave et periLoad=
*/
  char      periRec[PERIRECLEN];                // 1er buffer de l'enregistrement de périphérique
  char      periCache[PERIRECLEN*(NBPERIF+1)];  // cache des périphériques  
  bool      periCacheStatus[(NBPERIF+1)];       // indicateur de validité du cache d'un périph  (vaut CACHEISFILE si cache==fichier)
  
  uint16_t  periCur=0;                      // Numéro du périphérique courant
  
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
  byte*     periSwVal;                      // ptr ds buffer : état/cde des inter  
  byte*     periInput;                      // ptr ds buffer : table des règles switchs           
  uint32_t* periSwPulseOne;                 // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
  uint32_t* periSwPulseTwo;                 // ptr ds buffer : durée pulses sec OFF(mode astable)
  uint32_t* periSwPulseCurrOne;             // ptr ds buffer : temps courant pulses ON
  uint32_t* periSwPulseCurrTwo;             // ptr ds buffer : temps courant pulses OFF
  byte*     periSwPulseCtl;                 // ptr ds buffer : mode pulses
  byte*     periSwPulseSta;                 // ptr ds buffer : état clock pulses
  uint8_t*  periSondeNb;                    // ptr ds buffer : nbre sonde
  boolean*  periProg;                       // ptr ds buffer : flag "programmable" (périphériques serveurs)
  byte*     periDetNb;                      // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
  byte*     periDetVal;                     // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
  int16_t*  periThOffset_;                  // ptr ds buffer : offset correctif sur mesure température
  int16_t*  periThmin_;                     // ptr ds buffer : mini last 24h 
  int16_t*  periThmax_;                     // ptr ds buffer : maxi last 24h
  int16_t*  periVmin_;                      // ptr ds buffer : alarme mini volts
  int16_t*  periVmax_;                      // ptr ds buffer : alarme maxi volts
  byte*     periDetServEn;                  // ptr ds buffer : 1 byte 8*enable detecteurs serveur
  byte*     periProtocol;                   // ptr ds buffer : protocole ('T'CP/'U'DP)
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
  uint8_t*  periDigitCb;                    // ptr ds buffer : 5 x (4 bits pour checkbox + 4 bits pour operation logique)
  uint8_t*  periDigitDestDet;               // ptr ds buffer : 4 x n° détect serveur
  uint8_t*  periDigitRefDet;                // ptr ds buffer : 4 x n° détect serveur pour op logique (0xff si rien)
  int8_t*   periDigitMemo;                  // ptr ds buffer : 5 x n° mémo dans table mémos
    
  int8_t    periMess;                       // code diag réception message (voir MESSxxx shconst.h)
  byte      periMacBuf[6]; 

  byte*     periBegOfRecord;
  byte*     periEndOfRecord;

  byte      lastIpAddr[4]; 

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

struct Thermo thermos[NBTHERMOS];
char*  thermosA=(char*)&thermos;
unsigned long   thermoslen=(sizeof(Thermo))*NBTHERMOS;

char   memosTable[LMEMO*NBMEMOS];

/* DS3231 */

#define PINVCCDS 19   // alim + DS3231
#define PINGNDDS 18   // alim - DS3231

  //Ymdhms dt;
  Ds3231 ds3231;
 
/*
 * =========== mécanisme de la librairie 
 * 
 *  Ethernet.begin connecte au réseau (mac, IP dans LAN par DHCP ou forcé) 
 *  
 *  Ethernet.localIP() rend l'adresse IP si elle provient du DHCP
 *  Ethernet.maintain() pour renouveler le bail DHCP
 *                                                                                                        
 *  l'objet client est utilisé pour l'accès aux serveurs externes ET l'accès aux clients du serveur interne
 *  EthernetClient nom_client crée les objets clients pour serveur externe ou interne (capacité 4 clients)
 *                                                                                                          
 *  pour creer/utiliser un serveur : EthernetServer nom_serveur(port) crée l'objet ; 4 serveurs possibles sur 4 ports différents
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

char inch=' ';
char strdate[LDATEB];       // buffer date
char strd3[4]={0};
byte js=0;
uint32_t amj=0, hms=0;

uint32_t histoPos=0;       // SD current pos. pour dump
char histoDh[LDATEA]={'\0'};   // SD dh pour dump

int   i=0,j=0;
char  c=' ';
char  b[2]={0,0};
char* chexa="0123456789ABCDEFabcdef\0";
byte  maskbit[]={0xfe,0x01,0xfd,0x02,0xfb,0x04,0xf7,0x08,0xef,0x10,0xdf,0x20,0xbf,0x40,0x7f,0x80};
byte  mask[]={0x00,0x01,0x03,0x07,0x0F};


/* prototypes */

int  getnv(EthernetClient* cli);
void xcrypt();
void frecupptr(char* nomfonct,uint8_t* v,uint8_t* b,uint8_t lenpersw);
void bitvSwCtl(byte* data,uint8_t sw,uint8_t datalen,uint8_t shift,byte msk);
void test2Switchs();
void tcpPeriServer();
void pilotServer();
void udpPeriServer();
int8_t perToSend(uint8_t* tablePerToSend,unsigned long begTime);
void poolperif(uint8_t* tablePerToSend,uint8_t detec,char* nf);
void scanTimers();
void scanDate();
void scanTemp();
void scanThermos();
void testUdp();
void cidDmp();


void setup() {                              // ====================================

  Serial.begin (115200);delay(1000);
  Serial.print("+");delay(100);

  initLed(PINLED);
  digitalWrite(PINLED,HIGH);delay(10);digitalWrite(PINLED,LOW);

  pinMode(STOPREQ,INPUT_PULLUP);

/* >>>>>>     config     <<<<<< */  
  
  Serial.println();Serial.print(VERSION);Serial.print(" ");
  Serial.print(MODE_EXEC);Serial.print(" free=");Serial.print(freeMemory(), DEC);Serial.print(" FreeStack: ");Serial.println(FreeStack());
  
  //Serial.print(lip[0]);Serial.print(".");Serial.print(lip[1]);Serial.print(".");Serial.print(lip[2]);Serial.print(".");Serial.println(lip[3]);

  digitalWrite(PINGNDDS,LOW);pinMode(PINGNDDS,OUTPUT);  
  digitalWrite(PINVCCDS,HIGH);pinMode(PINVCCDS,OUTPUT); 
  ds3231.i2cAddr=DS3231_I2C_ADDRESS; // doit être avant getDate
  Wire.begin();
  
  sdInit();
  
  uint32_t        amj2,hms2;
  byte            js2;
  ds3231.getDate(&hms2,&amj2,&js2,strdate);histoStore_textdh0("R0",""," ");
  Serial.print("DS3231 time ");Serial.print(js2);Serial.print(" ");Serial.print(amj2);Serial.print(" ");Serial.println(hms2);

//remInit();remoteSave();while(1){ledblink(0);delay(1000);};
//periConvert();
//periMaintenance();

  trigwd();
  
  configInit();configLoad();*toPassword=TO_PASSWORD;if(*maxCxWt==0 || *maxCxWu==0){*maxCxWt=MAXCXWT;*maxCxWu=MAXCXWU;configSave();}
  memcpy(mac,MACADDR,6);memcpy(localIp,lip,4);*portserver=PORTSERVER;configSave();
  configPrint();

//for(int z=0;z<nbfonct;z++){Serial.print(z);Serial.print(" ");for(int w=0;w<10;w++){Serial.print(fonctions[z*10+w]);}Serial.println();}
    
/* >>>>>> load variables du systeme : périphériques, table et noms remotes, timers, détecteurs serveur <<<<<< */

  memDetLoad();                     // le premier pour Sync 
  periTableLoad();
  remoteLoad();periSwSync();  
  timersLoad();  
  thermosLoad();
  //memosInit();memosSave(-1);  
  memosLoad(-1);

  trigwd();
  
/* >>>>>> ethernet start <<<<<< */

  Serial.print("NOMSERV=");Serial.print(NOMSERV);Serial.print(" PORTSERVER=");Serial.print(PORTSERVER);Serial.print(" PORTPILOT=");Serial.print(PORTPILOT);Serial.print(" PORTUDP=");Serial.println(PORTUDP);
  
  if(Ethernet.begin(mac) == 0)
    {Serial.print("Failed with DHCP... forcing Ip ");serialPrintIp(localIp);Serial.println();
    Ethernet.begin (mac, localIp); //initialisation de la communication Ethernet
    }
  Serial.print("localIP=");Serial.println(Ethernet.localIP());

  trigwd();
  
  Serial.print("Udp.begin(");Serial.print(PORTUDP);Serial.print(") ");
  if(!Udp.begin(PORTUDP)){Serial.print("ko");while(1){}}
  Serial.println("ok");

  trigwd();
  
  periserv.begin();Serial.println("periserv.begin ");   // serveur périphériques

  pilotserv.begin();Serial.println("pilotserv.begin ");  //  remote serveur
  
/*
  if(cliext.connect(esptv,1791)){Serial.print("esptv connected");}
  cliext.stop();
  if(cliext.connect(gggg,80)){Serial.print("gggg connected");}
  cliext.stop();*/
  
  delay(100);

/* >>>>>> RTC ON, check date/heure et maj éventuelle par NTP  <<<<<< */
/* ethernet doit être branché pour l'udp */

  initDate();
  
/*  while(1){
  int udpav=Udp.parsePacket();
  char c;if(udpav>0){Udp.read(&c,1);Serial.print(c);}
  }*/

  histoStore_textdh(RESET,"","<br>\n\0");

  lastcxt=millis();
  lastcxu=millis();

  Serial.println(">>>>>>>>> fin setup\n");
}

/* ================================== fin setup ================================= */

void getremote_IP(EthernetClient* client,uint8_t* ptremote_IP,byte* ptremote_MAC)
{ 
    W5100.readSnDHAR(client->getSocketNumber(), ptremote_MAC);
    W5100.readSnDIPR(client->getSocketNumber(), ptremote_IP);
}

/* ==================================== loop ===================================== */

void loop()                         
{

            tcpPeriServer();     // *** périphérique TCP ou maintenance

            udpPeriServer();     // *** périphérique UDP via NRF
            
            pilotServer();       // *** pilotage

            ledblink(0);
            
            scanTemp(); 

            scanDate();         
            
            scanThermos();

            scanTimers();

            watchdog();

            stoprequest();

}


/* ==================================== tools =================================== */

void stoprequest()
{
  if(digitalRead(STOPREQ)==LOW){
    trigwd();
    histoStore_textdh(HALTREQ,"","<br>\n\0");
    periTableSave();
    Serial.println("Halt request =====");

    while(1){
      digitalWrite(PINLED,HIGH);delay(300);digitalWrite(PINLED,LOW);delay(300);
    }
  }
}

void watchdog()
{
  if(millis()-lastcxt>*maxCxWt && lastcxt!=0){wdReboot("TCP cx lost",*maxCxWt);}
  if(millis()-lastcxu>*maxCxWu && lastcxu!=0){wdReboot("UDP cx lost",*maxCxWu);}
}

void mail(char* a,char* mm)
{
      #define LMSG 64  
      char ms[LMSG];
      strcat(ms,a);strcat(ms,"==");
      strcat(ms,mailToAddr);strcat(ms,"==");
      if(strlen(mm)<=(LMSG-strlen(ms))){strcat(ms,mm);}
      periReq(&cliext,periMail,"mail______",ms);  
}

void wdReboot(char* msg,unsigned long maxCx)
{
    trigwd();
    histoStore_textdh(WDSD,msg,"<br>\n\0");
    periTableSave();
    mail("reBOOT",msg);
    Serial.print("no cx for ");Serial.print(maxCx/1000);Serial.println("sec");
    delay(30000);      // wait for hardware watchdog
}

void scanTemp()
{
    if((millis()-temptime)>pertemp*1000){   // *** maj température  
      temptime=millis();
      char buf[]={0,0,'.',0,0,0};
      float th;
      ds3231.readTemp(&th);
      if(fabs(th-oldth)>MINTHCHGE){
        oldth=th;sprintf(buf,"%02.02f",th);
        histoStore_textdh(TEMP,buf,"<br>\n\0");
      }
    }       
}

void scanDate()
{
    if((millis()-datetime)>perdate*1000){   // *** maj date
      initDate();
      datetime=millis();
      histoStore_textdh("D","","<br>\n\0");
      periTableSave();
      
      mail("DATE",alphaDate());
    }
}

void scanThermos()                                                        // positionnement détecteurs associés aux thermos
{                                                                         // maj tablePerToSend
  if((millis()-thermosTime)>perThermos*1000){
    
  thermosTime=millis();
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
        }
        else if( thermos[th].lowenable && *periLastVal_>(thermos[th].lowvalue+(thermos[th].lowoffset*thermos[th].lowstate))){     // clr low si > ref+offset*etat prec
          thermos[th].lowstate=0;
        }
        detSta[thermos[th].lowdetec]+=thermos[th].lowstate;
        
        if( thermos[th].highenable && *periLastVal_>=thermos[th].highvalue){                                                      // set high (tjrs on si>ref)
          thermos[th].highstate=1;
        }
        else if(thermos[th].highenable && *periLastVal_<(thermos[th].highvalue-thermos[th].highoffset*thermos[th].highstate)){    // clr high si < ref+offset*etat prec
          thermos[th].highstate=0;
        }
        detSta[thermos[th].highdetec]+=thermos[th].highstate;
//Serial.print(th);Serial.print(" ");Serial.print(per);Serial.print(" ");Serial.print(*periLastVal_);Serial.print(" ");Serial.print(thermos[th].lowenable);Serial.print(" ");
//Serial.print(thermos[th].lowstate);Serial.print(" ");Serial.print(thermos[th].lowvalue);Serial.print(" ");Serial.print(thermos[th].lowdetec);Serial.print(" ");Serial.println(detSta[thermos[th].lowdetec]);
      }      
    }
//Serial.println();

    /* maj détecteurs, tablePerToSend, peripheriques */
    char onoff[]="O\0I";
    
    for(det=0;det<NBDSRV;det++){

      if(detLst[det]!=0){
        //Serial.print(det);Serial.print(" ");Serial.print(detSta[det]);Serial.print(" ");memDetPrint();
        if(detSta[det]!=0){detSta[det]=1;}
        mds=0;
        if((memDetServ & mDSmaskbit[det])!=0){mds=1;}
        if((detSta[det] ^ mds)!=0){                                                   // change ?
          poolperif(tablePerToSend,det,&onoff[detSta[det]*2]);              
          memDetServ = memDetServ ^ mDSmaskbit[det];
        }
        //Serial.print(det);Serial.print(" ");Serial.print(detSta[det]);Serial.print(" ");memDetPrint();
      }
    }
    perToSend(tablePerToSend,thermosTime);
  }
}

void poolperif(uint8_t* tablePerToSend,uint8_t detec,char* nf)        // recherche des périphériques ayant une input sur le détecteur externe 'detec'
{                                                                     // et màj de tablePerToSend
  uint16_t offs;
  uint8_t eni,ninp;
  byte model=(detec<<PERINPNVLS_PB)|DETYEXT;                          // valeur pour N°detecteur type externe
  
  Serial.print(" poolperif (detec ");Serial.print(detec);Serial.print(" -> ");Serial.print(nf);Serial.println(")");
  for(uint8_t np=1;np<=NBPERIF;np++){                                 // boucle périphériques
    periLoad(np);
    if(*periSwNb!=0){                                                 // peripherique avec switchs ?

        for(ninp=0;ninp<NBPERINPUT;ninp++){                           // boucle inputs
          
          offs=ninp*PERINPLEN;
          eni=((*(uint8_t*)(periInput+2+offs)>>PERINPEN_PB)&0x01);    // enable          
          if(eni!=0 && model==*(byte*)(periInput+offs)){              // trouvé usage du détecteur dans periInput 
            Serial.print("  per=");Serial.print(np);Serial.print(" ninp=");Serial.print(ninp);
            tablePerToSend[np-1]++;                                   // (periSend) periReq à faire sur ce périf            
            //for(int nnp=0;nnp<NBPERIF;nnp++){Serial.print(tablePerToSend[nnp]);Serial.print(" ");}Serial.println();
          } // enable et model ok
        }   // input suivant
    }       // periswNb !=0
  }         // perif suivant
  Serial.println();
}

int8_t perToSend(uint8_t* tablePerToSend,unsigned long begTime)       // maj des périphériques repérés dans la table spécifiée 
{
      periMess=MESSOK;
      //if((millis()-begTime)>1){Serial.print("  durée scan =");Serial.print(millis()-begTime);}
      for(uint16_t np=1;np<=NBPERIF;np++){
        if(tablePerToSend[np-1]!=0){periMess=periReq(&cliext,np,"set_______");} 
      }
      /*if((millis()-begTime)>1){
        Serial.print("  durée scan+send =");Serial.print(millis()-begTime);
        Serial.print(" ");
        for(int nnp=0;nnp<NBPERIF;nnp++){Serial.print(tablePerToSend[nnp]);Serial.print(" ");}Serial.println();}*/
      memset(tablePerToSend,0x00,NBPERIF);
      return periMess;
}

void scanTimers()                                             //   recherche timer ayant changé d'état 
{                                                             //      si (en.perm.dh.js) (ON) et état OFF -> état ON, det ON, poolperif
                                                              //      sinon              (OFF) et état ON -> état OFF, det OFF, poolperif
                                                              //      màj tablePerToSend

    if((millis()-timerstime)>pertimers*1000){
       
      timerstime=millis();
      memset(tablePerToSend,0x00,NBPERIF);      // !=0 si (periSend) periReq à faire sur le perif          
      char now[LNOW];
      ds3231.alphaNow(now);
      
      for(int nt=0;nt<NBTIMERS;nt++){
        if(                                                     
          timersN[nt].enable==1                                 // enable & (permanent ou (dans les dates du cycle)
          && (timersN[nt].perm==1 || (memcmp(timersN[nt].dhdebcycle,now,14)<0 && memcmp(timersN[nt].dhfincycle,now,14)>0)) 
          && (  
               (   memcmp(timersN[nt].hfin,timersN[nt].hdeb,6)>0   // heure fin > heure deb
                && memcmp(timersN[nt].hdeb,(now+8),6)<0            // heure deb < now
                && memcmp(timersN[nt].hfin,(now+8),6)>0)           // heure fin > now
               ||
               (   memcmp(timersN[nt].hfin,timersN[nt].hdeb,6)<0   // heure fin < heure deb
                && (  memcmp(timersN[nt].hdeb,(now+8),6)<0         // heure deb < now
                   || memcmp(timersN[nt].hfin,(now+8),6)>0)))      // heure fin > now
          && (timersN[nt].dw & maskbit[1+now[14]*2])!=0 )       // jour semaine
          {                                                     // si timer déclenché
          if(timersN[nt].curstate!=1){                          // et état précédent 0, chgt->1
            timersN[nt].curstate=1;memDetServ |= mDSmaskbit[timersN[nt].detec]; // maj détecteur
            poolperif(tablePerToSend,timersN[nt].detec,"on");}  // recherche periphérique et mise à jour tablePerToSend
        }
        else {                                                  // si timer pas déclenché
          if(timersN[nt].curstate!=0){                          // et état précédent 1, chgt->0
            timersN[nt].curstate=0;memDetServ &= ~mDSmaskbit[timersN[nt].detec];    // maj détecteur
            if(timersN[nt].perm==0 && timersN[nt].cyclic==0){timersN[nt].enable=0;}; // si pas permanent et pas cyclique disable en fin
            poolperif(tablePerToSend,timersN[nt].detec,"off");} // recherche periphérique et mise à jour tablePerToSend
        }
      }
      perToSend(tablePerToSend,timerstime);     // mise à jour périphériques de tablePerToSend
    }
}

void sser(uint8_t det,uint8_t old)                                          // si un det a changé (!= old) -> inscription perif éventuel dans tablePerToSend
{
  uint32_t msk=mDSmaskbit[det];                                                                        
  uint32_t mem=memDetServ & msk;                                            // current detec value
  if(old==1){memDetServ |= msk;}                                            // set bit (1)
  else {memDetServ &= ~msk;}                                                // clr bit (0)
  if((memDetServ & msk) != mem){                                            // detec chge => poolperif
    poolperif(tablePerToSend,det," ");}                                     // si le détecteur est utilisé dans un périf, ajout du périf dans tablePerRoSend
}

void exploRemote(uint8_t nbr,char oe,uint8_t* old,uint8_t* nou)                   // si une des commandes de la remote (on/off ou enable/disjoncteur)
{                                                                           // a changé, vérification des détecteurs correspondant
                                                                            // et mise à jour des périfs concernés via tablePerToSend 
    
  if(*old+*nou==1){                                                         // changement d'état de la remote ?            
//    Serial.print("\n     chgt ");
    *old=*nou;*nou=0;
    for(uint8_t nbd=0;nbd<MAXREMLI;nbd++){                                  // recherche des détecteurs utilisés dans la remote
      if(remoteT[nbd].num==nbr+1){                                          // détecteur concerné ? (utilisé pour la remote courante)
        if(oe=='o'){sser(remoteT[nbd].detec,*old);}                         // vérif changement d'état du détecteur on/off 
        if(oe=='e'){
          Serial.print(nbr);Serial.print("/");Serial.print(nbd);Serial.print(" == ");Serial.print(remoteT[nbd].peri);Serial.print("/");Serial.print(remoteT[nbd].sw);Serial.print("/");Serial.print(*old);
          sser(remoteT[nbd].deten,*old);                                    // vérif changement d'état du détecteur enable pour maj perif
          if(remoteT[nbd].peri!=0 && remoteT[nbd].peri<=NBPERIF){           // maj periSwVal si peri/sw ok
            periCur=remoteT[nbd].peri;periLoad(periCur);
            periSwCdUpdate(remoteT[nbd].sw,*old);periSave(periCur,PERISAVESD);
            Serial.print("=");Serial.print(*periSwVal,HEX);
          }
          Serial.println();
        }
      }
    }
  }  
}

void periRemoteUpdate()                        //   recherche remote ayant changé d'état (onoff!=newonoff ou enable!=newenable)
                                               //     polling table des détecteurs pour trouver les détecteurs concernés
                                               //     màj de memDetServ et poolperif() pour trouver les périf concernés et charger tablePerToSend
{  
  remotetime=millis();
  memset(tablePerToSend,0x00,NBPERIF);         // périphériques !=0 si (periSend) periReq à faire via pertoSend())
  Serial.println("periRecRemoteUpdate() ");

  for(uint8_t nbr=0;nbr<NBREMOTE;nbr++){                                            // boucle des remotes
/*    Serial.print("nbr=");Serial.print(nbr);
    Serial.print("  on/off=");Serial.print(remoteN[nbr].onoff);Serial.print(" new on/off=");Serial.print(remoteN[nbr].newonoff);
    Serial.print("  enable=");Serial.print(remoteN[nbr].enable);Serial.print(" new enable=");Serial.print(remoteN[nbr].newenable);
*/
    exploRemote(nbr,'o',&remoteN[nbr].onoff,&remoteN[nbr].newonoff);         // le on/off de la remote a-til changé et les détecteurs associés aussi ?
    exploRemote(nbr,'e',&remoteN[nbr].enable,&remoteN[nbr].newenable);       // le enable de la remote a-til changé et les détecteurs associés aussi ?    
//  Serial.println();
  }
  remoteSave();
}

void periDetecUpdate()                              // détecteurs serveur - update périphériques
{
  char onoff[]={'O','N','\0','O','F','F','\0'};
  uint8_t of=3;
  srvdettime=millis();                                        
  memset(tablePerToSend,0x00,NBPERIF);         // périphériques !=0 => (periSend) periReq à faire via pertoSend())
  //Serial.print("bakDetServ=");Serial.print(bakDetServ,HEX);Serial.print(" memDetServ=");Serial.print(memDetServ,HEX);Serial.println();  
  
  for(uint8_t ds=0;ds<NBDSRV;ds++){                                                
    if((memDetServ&mDSmaskbit[ds]) != (bakDetServ&mDSmaskbit[ds])){   // si le détecteur ds a changé
      of=3;if(memDetServ&mDSmaskbit[ds]!=0){of=0;}
      poolperif(tablePerToSend,ds,&onoff[of]);}}                      // et si utilisé dans un périf, ajout du périf dans tablePerRoSend
}

/*
void checkdate(uint8_t num)                                           // détection des dates invalides (en général défaut de format des messages)
{
  if(periLastDateIn[0]==0x66){Serial.print("===>>> date ");Serial.print(num);Serial.print(" HS ");
    char dateascii[12];
    unpackDate(dateascii,periLastDateIn);for(j=0;j<12;j++){Serial.print(dateascii[j]);if(j==5){Serial.print(" ");}}Serial.println();
  }
}

/* ================================ decodage ligne GET/POST ================================ */

void cliWrite(EthernetClient* cli,char* data)
{
  if(ab=='u'){Udp.write(data,strlen(data));return;}
  cli->write(data);
}

int cliAv(EthernetClient* cli,uint16_t len,uint16_t* pt)
{
  if(ab=='u'){if(*pt<len){return len-*pt;}else return 0;}
  return cli->available();
}

char cliRead(EthernetClient* cli,char* data,uint16_t len,uint16_t* pt)
{
  if(ab=='u'){if(*pt<len){*pt=(*pt)+1;return data[(*pt)-1];}else return data[len-1];}
  return cli->read();
}

int getcde(EthernetClient* cli,char* data,uint16_t dataLen,uint16_t* ptr) // décodage commande reçue selon tables 'cdes' longueur maxi LENCDEHTTP
{
  char c='\0',cde[LENCDEHTTP];
  int ncde=0,ko=0,ptc=0;
  while (cliAv(cli,LENCDEHTTP,ptr) && c!='/' && *ptr<LENCDEHTTP) {
      c=cliRead(cli,data,LENCDEHTTP,ptr);Serial.print(c);                  // décode la commande 
      if(c!='/'){cde[ptc]=c;ptc++;}
      else {cde[ptc]=0;break;}
  }

  if (c!='/'){ko=1;}                                                                                // pas de commande, message 400 Bad Request
  else if (strstr(cdes,cde)==0){ko=2;}                                                              // commande inconnue 501 Not Implemented
  else {ncde=1+(strstr(cdes,cde)-cdes)/LENCDEHTTP ;}                                                // numéro de commande (ok si >0 && < nbre cdes)

  if ((ncde<=0) || (ncde>strlen(cdes)/LENCDEHTTP)){
    ko=1;ncde=0;
    while (cliAv(cli,dataLen,ptr)){
      c=cliRead(cli,data,dataLen,ptr);}}                                 // pas de cde valide -> vidage + message

  switch(ko){
    case 1:cliWrite(cli,"<body><br><br> err. 400 Bad Request <br><br></body></html>");break;
    case 2:cliWrite(cli,"<body><br><br> err. 501 Not Implemented <br><br></body></html>");break;
    default:break;
  }
  return ncde;                                // ncde=0 si KO ; 1 à n numéro de commande
}

int analyse(EthernetClient* cli,char* data,uint16_t dataLen,uint16_t* ptr)  // decode la chaine et remplit les tableaux noms/valeurs 
{                                             // prochain car = premier du premier nom
                                              // les caractères de ctle du flux sont encodés %HH par le navigateur
                                              // '%' encodé '%25' ; '@' '%40' etc... 

  boolean nom=VRAI,val=FAUX,termine=FAUX;
  int i=0,j=0,k=0;
  char c,cpc='\0';                            // cpc pour convertir les séquences %hh 
  char noms[LENNOM+1]={0},nomsc[LENNOM-1];noms[LENNOM]='\0';nomsc[LENNOM-2]='\0';
  memset(libfonctions,0x00,sizeof(libfonctions));

      nvalf[0]=0;
      memset(valeurs,0,LENVALEURS+1);                   // effacement critique (password etc...)
      memset(noms,' ',LENNOM);
      numfonct[0]=-1;                                   // aucune fonction trouvée
     
      while (cliAv(cli,dataLen,ptr)){
        c=cliRead(cli,data,dataLen,ptr);

        if(c=='%'){cpc=c;}                              // %
        else {
        
          if(cpc=='%'){cpc=(c&0x0f)<<4;}                // % reçu ; traitement 1er car
          else {
        
            if(cpc!='\0'){c=cpc|(c&0x0f);cpc='\0';}     // traitement second                 
            Serial.print(c);
            if (!termine){
          
              if (nom==FAUX && (c=='?' || c=='&')){nom=VRAI;val=FAUX;j=0;memset(noms,' ',LENNOM);if(i<NBVAL){i++;};Serial.println(libfonctions+2*(i-1));}  // fonction suivante ; i indice fonction courante ; numfonct[i] N° fonction trouvée
              if (nom==VRAI && (c==':' || c=='=')){                                    
                nom=FAUX;val=VRAI;
                nvalf[i+1]=nvalf[i]+1;
                if(i==0){nvalf[1]=0;}                                       // permet de stocker le tout premier car dans valeurs[0]
                else {nvalf[i]++;}                                          // skip l'intervalle entre 2 valeurs           

                long numfonc=(strstr(fonctions,noms)-fonctions)/LENNOM;     // acquisition nom terminée récup N° fonction
                memcpy(libfonctions+2*i,noms+LENNOM-2,2);                   // les 2 derniers car du nom de fonction si nom court

                if(numfonc<0 || numfonc>=nbfonct){
                  memcpy(nomsc,noms,LENNOM-2);
                  numfonc=(strstr(fonctions,nomsc)-fonctions)/LENNOM;       // si nom long pas trouvé, recherche nom court (complété par nn)

                  if(numfonc<0 || numfonc>=nbfonct){numfonc=faccueil;}
                  else {numfonct[i]=numfonc;}
                }
                else {numfonct[i]=numfonc;}
              }
              if (nom==VRAI && c!='?' && c!=':' && c!='&' && c>' '){noms[j]=c;if(j<LENNOM-1){j++;}}     // acquisition nom
              if (val==VRAI && c!='&' && c!=':' && c!='=' && c>' '){
                valeurs[nvalf[i+1]]=c;if(nvalf[i+1]<=LENVALEURS-1){nvalf[i+1]++;}}                      // contrôle decap !
              if (val==VRAI && (c=='&' || c<=' ')){
                nom=VRAI;val=FAUX;j=0;
                if(c<=' '){termine=VRAI;}
              Serial.println(); 
              }                                                                                         // ' ' interdit dans valeur : indique fin données                                 
            }//Serial.println(libfonctions+2*(i-1));
          }                                                                                             // acquisition valeur terminée (données aussi si c=' ')
        }
      }
      Serial.print("---- fin getnv i=");Serial.println(i);
      if(numfonct[0]<0){return -1;} else return i;
}

int getnv(EthernetClient* cli,char* data,uint16_t dataLen)        // décode commande, chaine et remplit les tableaux noms/valeurs
{                                     // sortie -1 pas de commande ; 0 pas de nom/valeur ; >0 nbre de noms/valeurs                                
  uint16_t ptr=0;
  numfonct[0]=-1;
  int cr=0,pbli=0;
#define LBUFLI 12
  char bufli[LBUFLI];
  
  Serial.println("--- getnv");
  
  int ncde=getcde(cli,data,dataLen,&ptr); 
      Serial.print("ncde=");Serial.print(ncde);Serial.println(" ");
      if(ncde==0){return -1;}  
      
      c=' ';
      while (cliAv(cli,dataLen,&ptr) && c!='?' && c!='.'){      // attente '?' ou '.'
        c=cliRead(cli,data,dataLen,&ptr);Serial.print(c);
        bufli[pbli]=c;if(pbli<LBUFLI-1){pbli++;bufli[pbli]='\0';}
      }Serial.println();          

        switch(ncde){
          case 1:           // GET
            if(strstr(bufli,"favicon")>0){htmlFavicon(cli);}
            if(bufli[0]=='?' || strstr(bufli,"page.html?")>0 || strstr(bufli,"cx?")>0){return analyse(cli,data,dataLen,&ptr);}
            break;
          case 2:           // POST
            if(c=='\n' && cr>=3){return analyse(cli,data,dataLen,&ptr);}
            else if(c=='\n' || c=='\r'){cr++;}
            else {cr=0;}
            break;
          default:break;
        }
      if(numfonct[0]<0){return -1;} else return 0; 
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
  charIp(lastIpAddr,ipAddr);
  for(int x=0;x<4;x++){
//Serial.print(x);Serial.print(" test2sw ");Serial.println(ipAddr);
    testSwitch("GET /testb_on__=0006AB8B",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testa_on__=0006AB8B",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testboff__=0006AB8B",ipAddr,*periPort);
    delay(2000);
    testSwitch("GET /testaoff__=0006AB8B",ipAddr,*periPort);
    delay(2000);
  }
}

void testSwitch(char* command,char* perihost,int periport)
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
            purgeServer(&cliext);
            cliext.stop();
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
  uint8_t lreel;
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
  uint8_t li=*(libfonctions+2*i+1)-PMFNCHAR;                      // row
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
    if(lm>=LMEMO-1){lm=LMEMO-1;}
    memcpy(bm,valf,lm);
    trailingSpaces(bm,LMEMO-1);
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


/* ================================ serveur ================================= */

void commonserver(EthernetClient cli,char* bufData,uint16_t bufDataLen)
{
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

      Serial.print("\n *** serveur(");Serial.print((char)ab);Serial.print(") ");serialPrintIp(remote_IP);Serial.print(" ");serialPrintMac(remote_MAC,1);
      
      cxtime=millis();    // pour rémanence pwd
      
      nbreparams=getnv(&cli,bufData,bufDataLen);Serial.print("\n---- nbreparams ");Serial.println(nbreparams);
        
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
        periInitVar();        // pas de rémanence des données des périphériques entre 2 accès au serveur

        memset(strHisto,0x00,sizeof(strHisto)); memset(buf,0,sizeof(buf));charIp((byte*)&remote_IP,strHisto);        // histo :
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
        what=0;                           // pas de traitement subsidiaire

/*
      3 modes de fonctionnement :

      1 - accueil : saisie de mot de passe
      2 - serveur pour périphériques : seules fonctions dataRead et dataSave ; contiennent l'adr mac pour identifier le périphérique.
      3 - serveur pour navigateur : dans les message GET ou POST, 
          pour assurer la validité des données du périphérique concerné, periCur doit être positioné et periLoad() effectué 
          par une fonction qui précède les fonctions modifiant des variables du formulaire concerné (peri_cur__, peri_t_sw_) 

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
    la structure des pages doit être : <form> (éventuellemnt dans pageHeader) usrPericur() {boutons etc... toute la page - Un bouton de MàJ par formulaire} </form>
    <form>...</form> délimite l'espace d'action de sbmit du bouton MàJ  ;  plusieurs formulaires possibles dans une page
    ------------------------------------------------------------------------------------------------------------------------------------------------------
                                       

    Sécurité à développer : pour assurer que le mot de passe n'est pas dérobable et que sans mot de passe on ne peut avoir de réponse du serveur,
    le mot de passe doit être crypté dans une chaine qui change de valeur à chaque transmission ; donc crypter mdp+heure. 
    Le serveur accepte une durée de validité (10sec?) au message et refuse le ré-emploi de la même heure.
    Ajouter du java pour crypter ce qui sort du navigateur ? (les 64 premiers caractères de GET / : username,password,heure/user_ref,heure)
    
*/     

      if(numfonct[0]!=fperipass){                                                                   // si la première fonction n'est pas peri_pass_ (mot de passe des périfs)
        if((numfonct[0]!=fusername || numfonct[1]!=fpassword) && numfonct[0]!=fuserref){            //   si (la 1ère fonct n'est pas username__ ou la 2nde pas password__ ) et la 1ère pas user_ref__
                                                                                                    //   ... en résumé : ni un périf, ni une nlle cx utilisateur, ni une continuation d'utilisateur
          what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;}    // -->> accueil en cas d'anomalie     //      --> accueil   (voir plus haut les explications)
      }
/*
    boucle des fonctions accumulées par getnv
*/   
//if(periCur==3){Serial.print(" intro serveur ================");periPrint(periCur);}

        uint16_t transferVal;         // pour passer "quelque chose" entre 2 fonctions 
        
        for (i=0;i<=nbreparams;i++){

          if(i<NBVAL && i>=0){

            trigwd();
          
            valf=valeurs+nvalf[i];    // valf pointe la ième chaine à traiter dans valeurs[] (terminée par '\0')
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

            switch (numfonct[i])
              {
              case 0:  pertemp=0;conv_atobl(valf,&pertemp);break;                                           // pertemp serveur
              case 1:  if(checkData(valf)==MESSOK){                                                         // peri_pass_
                         periPassOk=ctlpass(valf+5,peripass);                                               // skip len
                         if(periPassOk==FAUX){memset(remote_IP_cur,0x00,4);histoStore_textdh("pp","ko",strHisto);}
                         else {memcpy(remote_IP_cur,(byte*)&remote_IP,4);}
                       }break;
              case 2:  usernum=searchusr(valf);if(usernum<0){                                        // username__
                          what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;}
                       Serial.print("username:");Serial.print(valf);Serial.print(" usernum=");
                       Serial.print(usernum);Serial.print("/");Serial.print(usrnames+usernum*LENUSRNAME);
                       Serial.print(" usrtime=");Serial.println(usrtime[usernum]);
                       break;
              case 3:  if(!ctlpass(valf,usrpass+usernum*LENUSRPASS)){                                // password__
                         what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;usrtime[usernum]=0;          // si faux accueil (what=-1)
                         histoStore_textdh("pw","ko",strHisto);}                                   
                       else {Serial.println("password ok");usrtime[usernum]=millis();if(nbreparams==1){what=2;}}
                       break;                                                                        
              case 4:  {usernum=*(libfonctions+2*i+1)-PMFNCHAR;                                      // user_ref__ (argument : millis() envoyées avec la fonction au chargement de la page)
                        unsigned long cxtime=0;conv_atobl(valf,(uint32_t*)&cxtime);
                        Serial.print("usr_ref__ : usrnum=");Serial.print(usernum);Serial.print(" millis()/1000=");Serial.print(millis()/1000);Serial.print(" cxtime=");Serial.print(cxtime);
                        Serial.print(" usrtime[nb]=");Serial.print(usrtime[usernum]);Serial.print(" usrpretime[nb]=");Serial.println(usrpretime[usernum]);
                        // !( usrtime ok || (html && usrpretime ok) ) || time out  => accueil 
                        if(!(usrtime[usernum]==cxtime || (usrpretime[usernum]==cxtime && memcmp(&fonctions[numfonct[i+1]*LENNOM]+(LENNOM-3),"html",4)==0)) || (millis()-usrtime[usernum])>(*toPassword*1000)){
                          what=-1;nbreparams=-1;i=0;numfonct[i]=faccueil;usrtime[usernum]=0;}
                        else {Serial.print("user ");Serial.print(usrnames+usernum*LENUSRNAME);Serial.println(" ok");
                          usrtime[usernum]=millis();
                          usrpretime[usernum]=cxtime;
                          if(nbreparams==0){what=2;}}
                        }//if(periCur==3){Serial.println("user_ref_ =================");periPrint(periCur);}
                        break;  
              case 5:  *toPassword=TO_PASSWORD;conv_atob(valf,toPassword);Serial.print(" topass=");Serial.println(valf);break;                     // to_passwd_
              case 6:  what=2;perrefr=0;conv_atob(valf,&perrefr);                                    // (en tête peritable) periode refresh browser
                       break;                                                                               
              case 7:  *periThOffset_=0;*periThOffset_=(int16_t)(convStrToNum(valf,&j)*100);break;   // (ligne peritable) Th Offset
              case 8:  periCur=*(libfonctions+2*i+1)-PMFNCHAR;                                       // bouton switchs___ (ligne peritable)
                       periLoad(periCur);                                                            // + bouton refresh  (switchs)
                       if(*(libfonctions+2*i)=='X'){periInitVar0();}                                 // + bouton erase    (switchs)
                       else{periReq(&cliext,periCur,"etat______");}                                  // si pas erase demande d'état
                       SwCtlTableHtml(&cli);break;                                                                               
              case 9:  {byte a=*(libfonctions+2*i);
                        if(a=='B'){wdReboot(BOOT,millis());}
                       }break;                                                                       // si pas 'R' déco donc -> accueil                                             
              case 10: dumpHisto(&cli);break;                                                        // bouton dump_histo
              case 11: {what=2;byte a=*(libfonctions+2*i);                                           // (en-tete peritable) saisie histo pos/histo dh pour dump
                        if(a=='D'){memcpy(histoDh,valf,LDATEA-2);if(histoDh[8]==0x2B){histoDh[8]=0x20;}}  // saisie date/heure au format "AAAAMMDD HHMMSS"
                        else {histoPos=0;conv_atobl(valf,&histoPos);}                                // saisie position
                       }break;                           
              case 12: if(periPassOk==VRAI){what=1;periDataRead(valf);periPassOk==FAUX;}break;       // data_save
              case 13: if(periPassOk==VRAI){what=3;periDataRead(valf);periPassOk==FAUX;}break;       // data_read
              case 14: {byte a=*(libfonctions+2*i);byte b=*(libfonctions+2*i+1);                     // (ligne peritable) - tests de perif serveur
                        char fptst[LENNOM+1];                            
                        char swcd[]={"sw0__ON___sw0__OFF__sw1__ON___sw1__OFF__mail______"};
                        uint8_t k=0;uint8_t zer[]={1,0};
                        char msg[64]="TEST==";strcat(msg,mailToAddr);strcat(msg,"==test peri ");msg[strlen(msg)]=b;msg[strlen(msg)]='\0';
                        periCur=b-PMFNCHAR;periLoad(periCur);
                        if(a=='m'){k=4;}
                        else {k=zer[periSwCde(a-PMFNCVAL)]+(a-PMFNCVAL)*2;}
                        memcpy(fptst,swcd+LENNOM*k,LENNOM);
                        periReq(&cli,periCur,fptst,msg);
                        Serial.print("=========== periCur=");Serial.print(periCur);Serial.print(" k=");Serial.print(k);Serial.print(" a=");Serial.print(a-PMFNCVAL);Serial.print(" ~swcd=");Serial.print(zer[periSwCde(a-PMFNCVAL)]);Serial.print(" peritst=");Serial.println(fptst);
                        periLineHtml(&cli,periCur);
                       }break;                                                                       
              case 15: what=5;periCur=0;conv_atob(valf,&periCur);                                    // submit modifs dans ligne de peritable (peri cur)
                       if(periCur>NBPERIF){periCur=NBPERIF;}                                         
                       periInitVar();periLoad(periCur);
                       *periProg=0;
                       break;                                                                        
              case 16: *periPerRefr=0;conv_atobl(valf,periPerRefr);break;                            // (ligne peritable) periode maxi accès serveur periph courant
              case 17: memset(periNamer,0x00,PERINAMLEN-1);                                          // (ligne peritable) nom periph courant
                       memcpy(periNamer,valf,nvalf[i+1]-nvalf[i]);
                       trailingSpaces(periNamer,PERINAMLEN);
                       break;                                                                              
              case 18: for(j=0;j<6;j++){conv_atoh(valf+j*2,(periMacr+j));}break;                     // (ligne peritable) Mac periph courant
              case 19: accueilHtml(&cli);break;                                                      // accueil
              case 20: periTableHtml(&cli);break;                                                    // peri table
              case 21: *periProg=*valf-48;break;                                                     // (ligne peritable) peri prog
              case 22: *periSondeNb=*valf-48;if(*periSondeNb>MAXSDE){*periSondeNb=MAXSDE;}break;     // (ligne peritable) peri sonde
              case 23: *periPitch_=0;*periPitch_=(int16_t)(convStrToNum(valf,&j)*100);break;         // (ligne peritable) peri pitch
              case 24: what=4;periCur=0;conv_atob(valf,&periCur);                                    // (lignes-regles) submit peri_inp__ set periCur raz cb
                       if(periCur>NBPERIF){periCur=NBPERIF;}periInitVar();periLoad(periCur);
                       *(byte*)(periInput+((uint8_t)(*(libfonctions+2*i+1))-PMFNCHAR)*PERINPLEN+2)&=PERINPACT_MS;  // effacement cb (oldlev/active/edge/en)
                       break;                                                                      
              case 25: *periDetNb=*valf-48;if(*periDetNb>MAXDET){*periDetNb=MAXDET;}break;           // (ligne peritable) peri det Nb  
              case 26: *periSwNb=*valf-48;if(*periSwNb>MAXSW){*periSwNb=MAXSW;}break;                // (ligne peritable) peri sw Nb                       
              case 27: *periPerTemp=0;conv_atob(valf,periPerTemp);break;                             // periode check température
              case 28: cfgRemoteHtml(&cli);remotePrint();break;                                      // bouton remotecfg_
              case 29: testHtml(&cli);break;                                                         // bouton testhtml
              case 30: {uint8_t sw=*(libfonctions+2*i+1)-PMFNCHAR;                                   // (ligne peritable) peri Sw Val 
                       uint8_t cd=*valf-PMFNCVAL;
                       periSwCdUpdate(sw,cd);                                                        // + maj remotes concernées
                       //Serial.print("===========");Serial.print(periCur);Serial.print("/");Serial.print(sw);Serial.print("/");Serial.print(cd);Serial.print("/");Serial.print(*periSwVal,HEX);
                       for(uint8_t k=0;k<MAXREMLI;k++){if(remoteT[k].peri==periCur && remoteT[k].sw==sw){remoteN[remoteT[k].num-1].enable=cd;Serial.print("=");Serial.print(remoteT[k].num);}}Serial.println();
                       }break;
              case 31: what=4;periCur=0;conv_atob(valf,&periCur);                                    // (regles switchs-tête) submit pulses (peri_t_sw_)
                       if(periCur>NBPERIF){periCur=NBPERIF;}
                       periInitVar();periLoad(periCur);                                             
                       {memset(periSwPulseCtl,0x00,PCTLLEN);                                         // effact bits otf enable pulses et free run 
                       }break;  
              case 32: {uint8_t pu=*(libfonctions+2*i)-PMFNCHAR,b=*(libfonctions+2*i+1);             // (pulses) periSwPulseCtl (otf) bits généraux (FOT)
                       uint16_t sh=0;
                        switch (b){
                           case 'F':sh=PMFRO_VB;break;
                           case 'O':sh=PMTOE_VB;break;
                           case 'T':sh=PMTTE_VB;break;
                           default:break;
                        }
                        sh=sh<<pu*PCTLBIT;
                        *(uint16_t*)periSwPulseCtl|=sh;
                       }break;       
              case 33: {uint8_t nfct=*(libfonctions+2*i)-PMFNCHAR,nuinp=*(libfonctions+2*i+1)-PMFNCHAR;   // (regles switchs) p_inp1__  
                        uint8_t offs=nuinp*PERINPLEN;                                                     // (enable/type/num detec/action)
                        uint16_t vl=0;
                        // pericur est à jour via peri_inp_
                        switch (nfct){
                          case 1:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPEN_VB;break;               // enable
                          case 2:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPOLDLEV_VB;break;           // prev level
                          case 3:*(uint8_t*)(periInput+2+offs)|=(uint8_t)PERINPDETES_VB;break;            // edge/static
                          case 4:inpsub((periInput+offs),PERINPNT_MS,PERINPNTLS_PB,inptyps,2);break;      // type src
                          case 5:conv_atob(valf,&vl);if(vl>NBDSRV){vl=NBDSRV;}
                                 *(periInput+offs)&=~PERINPV_MS;
                                 *(periInput+offs)|=(uint8_t)(vl<<PERINPNVLS_PB);break;                   // num detec src
                          case 6:if(((*(periInput+3+offs)&PERINPNT_MS)>>PERINPNTLS_PB)==DETYEXT){
                            sourceDetServ[offs/PERINPLEN]=0;}     // si la dest  était un detServ il faut effacer sourceDetServ
                                 transferVal=(uint16_t)inpsub((periInput+3+offs),PERINPNT_MS,PERINPNTLS_PB,inptypd,2);break;    // type dest
                          case 7:conv_atob(valf,&vl);if(vl>NBDSRV){vl=NBDSRV;}
                                 *(periInput+3+offs)&=~PERINPV_MS;
                                 *(periInput+3+offs)|=(uint8_t)(vl<<PERINPNVLS_PB);
                                 if(transferVal==DETYEXT){setSourceDet(vl,MDSPER,periCur);}  // si la dest est un detServ màj sourceDetServ (periCur from peri_inp_)
                                 break;                                                                   // num detec dest                                 
                          case 8:inpsub((periInput+2+offs),PERINPACT_MS,PERINPACTLS_PB,inpact,LENTACT);break;    // action
                          case 9:*(uint8_t*)(periInput+offs+2)|=(uint8_t)PERINPVALID_VB;break;            // active level
                          default:break;
                        }
                       }break;                                                                      
              case 34: {uint8_t nfct=*(libfonctions+2*i)-PMFNCHAR,nuinp=*(libfonctions+2*i+1)-PMFNCVAL;   // (règles switchs) p_inp2__  8 bits inutilisés
                        uint8_t offs=nuinp*PERINPLEN;
                        // pericur est à jour via peri_inp_
                        *(uint8_t*)(periInput+offs+1)|=(uint8_t)PERINPRULESLS_VB<<nfct;}break;
              case 35: {int pu=*(libfonctions+2*i)-PMFNCHAR;                                            // (pulses) peri Pulse one (pto)
                        *(periSwPulseOne+pu)=0;*(periSwPulseOne+pu)=(uint32_t)convStrToInt(valf,&j);                              
                       }break;                                                                      
              case 36: {int pu=*(libfonctions+2*i)-PMFNCHAR;
                        *(periSwPulseTwo+pu)=0;*(periSwPulseTwo+pu)=(uint32_t)convStrToInt(valf,&j); 
                       }break;                                                                          // (pulses) peri Pulse two (ptt)
              case 37: *periThmin_=0;*periThmin_=(int16_t)convStrToInt(valf,&j);break;                  // (ligne peritable) Th min
              case 38: *periThmax_=0;*periThmax_=(int16_t)convStrToInt(valf,&j);break;                  // (ligne peritable) Th max
              case 39: *periVmin_=0;*periVmin_=(int16_t)convStrToInt(valf,&j);break;                    // (ligne peritable) V min
              case 40: *periVmax_=0;*periVmax_=(int16_t)convStrToInt(valf,&j);break;                    // (ligne peritable) V max
              case 41: what=10;bakDetServ=memDetServ;memDetServ=0;                                      // bouton submit detecteurs serveur ; effct cb
                       break;
              case 42: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (mem_dsrv__) set det bit
                       if(nb>=16){nb-=16;}
                       memDetServ |= mDSmaskbit[nb];
                       }break;
              case 43: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) ssid[libf+1]
                       memset(ssid+nb*(LENSSID+1),0x00,LENSSID+1);memcpy(ssid+nb*(LENSSID+1),valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 44: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) passssid[libf+1]
                       memset(passssid+nb*(LPWSSID+1),0x00,LENSSID+1);memcpy(passssid+nb*(LPWSSID+1),valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 45: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) usrname[libf+1]
                       memset(usrnames+nb*(LENUSRNAME+1),0x00,LENUSRNAME+1);memcpy(usrnames+nb*(LENUSRNAME+1),valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 46: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (config) usrpass[libf+1]
                       memset(usrpass+nb*(LENUSRPASS+1),0x00,LENUSRPASS+1);memcpy(usrpass+nb*(LENUSRPASS+1),valf,nvalf[i+1]-nvalf[i]);
                       }break;                       
              case 47: cfgServerHtml(&cli);configPrint();break;                                         // bouton config
              case 48: what=6;                                                                          // submit depuis cfgServervHtml 1ère fonction 
                       memset(userpass,0x00,LPWD);memcpy(userpass,valf,nvalf[i+1]-nvalf[i]);break;      // (config) pwdcfg____
              case 49: memset(modpass,0x00,LPWD);memcpy(modpass,valf,nvalf[i+1]-nvalf[i]);break;        // (config) modpcfg___
              case 50: memset(peripass,0x00,LPWD);memcpy(peripass,valf,nvalf[i+1]-nvalf[i]);break;      // (config) peripcfg__
              case 51: char eth;eth=*(libfonctions+2*i+1);                                              // (config) eth config
                       switch(eth){
                          case 'i': break;memset(localIp,0x00,4);                                       // (config) localIp
//                                    for(j=0;j<4;j++){conv_atob(valf,localIp+j);}break;   // **** à faire ****
                          case 'p': *portserver=0;conv_atob(valf,portserver);break;                     // (config) portserver
                          case 'm': for(j=0;j<6;j++){conv_atoh(valf+j*2,(mac+j));}break;                // (config) mac
                          case 'q': *maxCxWt=0;conv_atobl(valf,maxCxWt);break;                          // (config) TO sans TCP
                          case 'r': *maxCxWu=0;conv_atobl(valf,maxCxWu);break;                          // (config) TO sans UDP
                          default: break;
                       }
                       break;
              case 52: what=8;                                                                          // submit depuis cfgRemotehtml
                       {int nb=*(libfonctions+2*i+1)-PMFNCHAR;
                        uint16_t v1=0;
                        switch(*(libfonctions+2*i)){                                               
                            case 'n': memset(remoteN[nb].nam,0x00,LENREMNAM);                           // (remotecf) no nom remote courante              
                                      memcpy(remoteN[nb].nam,valf,nvalf[i+1]-nvalf[i]);
                                      remoteN[nb].enable=0;remoteN[nb].onoff=0;break;                   // (remotecf) effacement cb 
                            case 'e': remoteN[nb].enable=*valf-PMFNCVAL;break;                          // (remotecf) en enable remote courante
                            case 'o': remoteN[nb].onoff=*valf-PMFNCVAL;break;                           // (remotecf) on on/off remote courante
                            case 'u': remoteT[nb].num=*valf-PMFNCVAL;                                   // (remotecf) un N° remote table sw
                                      remoteT[nb].enable=0;break;                                       // (remotecf) effacement cb
                            case 'd': sourceDetServ[remoteT[nb].detec]=0;                 
                                      remoteT[nb].detec=convStrToInt(valf,&j);                          // (remotecf) n° detecteur on/off
                                      if(remoteT[nb].detec>NBDSRV){remoteT[nb].deten=NBDSRV;}
                                      if(remoteT[nb].detec!=0){setSourceDet(remoteT[nb].detec,MDSREM,nb+1);}
                                      break; 
                            case 'b': remoteT[nb].deten=convStrToInt(valf,&j);                          // (remotecf) n° detecteur enable
                                      if(remoteT[nb].deten>NBDSRV){remoteT[nb].deten=NBDSRV;}break;                                             
                            case 'p': conv_atob(valf,&v1);remoteT[nb].peri=(uint8_t)v1;break;           // (remotecf) n° peri
                            case 's': conv_atob(valf,&v1);remoteT[nb].sw=(uint8_t)v1;                   // (remotecf) n° sw
                                      if(remoteT[nb].peri < NBPERIF && remoteT[nb].peri>0 ){            // controle validité peri/sw
                                        periLoad(remoteT[nb].peri);periCur=remoteT[nb].peri;
                                        if(remoteT[nb].sw < *periSwNb){break;}                          // ok
                                      }
                                      remoteT[nb].peri=0;remoteT[nb].sw=0;break;                        // ko
                            default:break;
                          }
                       }break;
              case 53: what=9;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                   // submit depuis remoteHtml (cdes on/off)
                        switch(*(libfonctions+2*i)){                                               
                            case 'n': remoteN[nb].newonoff=0;break;                                     // (remote_cn) effacement cb on/off
                            case 't': remoteN[nb].newonoff=1;break;                                     // (remote_ct) check cb on/off
                            case 'm': remoteN[nb].newenable=0;break;                                    // (remote_cm) effacement cb enable
                            case 's': remoteN[nb].newenable=*valf-48;break;                             // (remote_cs) check cb enable
                            default:break;
                          }
                       }break;                                                                       
              case 54: remoteHtml(&cli);break;                                                          // remotehtml
              case 55: what=5;periInitVar();periRaz(periCur);break;                                     // peri_raz___  
              case 56: break;                                                                           //                            *************** dispo
              case 57: what=12;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                  // submit depuis thparams__ (thermosCfg())
                        switch (*(libfonctions+2*i)){
                          case 'n':memset(&thermos[nb].nom,0x00,LENTHNAME+1);
                                   memcpy(&thermos[nb].nom,valf,nvalf[i+1]-nvalf[i]);
                                   trailingSpaces((char*)&thermos[nb].nom,LENTHNAME+1);
                                   break;
                          case 'p':thermos[nb].peri=0;thermos[nb].peri=convStrToInt(valf,&j);
                                   if((thermos[nb].peri)>NBPERIF){(thermos[nb].peri)=NBPERIF;}
                                   break;
                          case 'e':thermos[nb].lowenable=*valf-48;break;                                           // enable low
                          case 'E':thermos[nb].highenable=*valf-48;break;                                          // enable high
                          case 's':thermos[nb].lowstate=*valf-48;break;                                            // state  low
                          case 'S':thermos[nb].highstate=*valf-48;break;                                           // state  high
                          case 'v':thermos[nb].lowvalue=0;thermos[nb].lowvalue=(int16_t)convStrToInt(valf,&j);break;       // value low
                          case 'V':thermos[nb].highvalue=0;thermos[nb].highvalue=(int16_t)convStrToInt(valf,&j);break;     // value high
                          case 'o':thermos[nb].lowoffset=0;thermos[nb].lowoffset=(int16_t)convStrToInt(valf,&j);break;     // offset low
                          case 'O':thermos[nb].highoffset=0;thermos[nb].highoffset=(int16_t)convStrToInt(valf,&j);break;   // offset high
                          case 'd':sourceDetServ[remoteT[nb].detec]=0;                 
                                   thermos[nb].lowdetec=0;thermos[nb].lowdetec=convStrToInt(valf,&j);
                                   if(thermos[nb].lowdetec>NBDSRV){thermos[nb].lowdetec=NBDSRV;}
                                   if(thermos[nb].lowdetec!=0){setSourceDet(thermos[nb].lowdetec,MDSTHE,nb+1);}
                                   break;                                                                           // det low
                          case 'D':thermos[nb].highdetec=0;thermos[nb].highdetec=convStrToInt(valf,&j);
                                   if(thermos[nb].highdetec>NBDSRV){thermos[nb].highdetec=NBDSRV;}
                                   if(thermos[nb].highdetec!=0){setSourceDet(thermos[nb].highdetec,MDSTHE,nb+1);}
                                   break;                                                                           // det high                          
                          default:break;
                        } 
                       }break;
              case 58: thermoShowHtml(&cli);break;                                                      // thermoshow
              case 59: thermoCfgHtml(&cli);thermosPrint();break;                                        // thermos___ (bouton thermo_cfg)
              case 60: *periPort=0;conv_atob(valf,periPort);break;                                      // (ligne peritable) peri_port_
              case 61: what=7;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                   // (timers) tim_name__
                       textfonc(timersN[nb].nom,LENTIMNAM);
                       //Serial.print("efface cb timers ");Serial.print(nb);Serial.print(" ");Serial.print(timersN[nb].nom);
                       timersN[nb].enable=0;                                                            // (timers) effacement cb     
                       timersN[nb].perm=0;
                       timersN[nb].cyclic=0;
                       timersN[nb].curstate=0;
                       timersN[nb].forceonoff=0;
                       timersN[nb].dw=0;
                       }break;
              case 62: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_det___      
                       sourceDetServ[timersN[nb].detec]=0;                            
                       timersN[nb].detec=convStrToInt(valf,&j);
                       if(timersN[nb].detec>NBDSRV){timersN[nb].detec=NBDSRV;}
                       setSourceDet(timersN[nb].detec,MDSTIM,nb+1);
                       }break;
              case 63: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_hdf___
                        switch (*(libfonctions+2*i)){         
                          case 'd':textfonc(timersN[nb].hdeb,6);break;
                          case 'f':textfonc(timersN[nb].hfin,6);break;
                          case 'b':textfonc(timersN[nb].dhdebcycle,14);break;
                          case 'e':textfonc(timersN[nb].dhfincycle,14);break;
                          default:break;
                        } 
                       }break;
              case 64: {int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                          // (timers) tim_chkb__
                        int nv=*(libfonctions+2*i)-PMFNCHAR;
                        /* e_p_c_f */
                        switch (nv){         
                          case 0:timersN[nb].enable=*valf-48;break;
                          case 1:timersN[nb].perm=*valf-48;break;
                          case 2:timersN[nb].cyclic=*valf-48;break;
                          default:break;
                        }
                        /* dw */
                        if(nv=NBCBTIM){timersN[nb].dw=0xFF;}
                        if(nv>NBCBTIM){
                          timersN[nb].dw|=maskbit[1+2*(7-nv+NBCBTIM)];                          
                        }                         
                       }break;
              case 65: Serial.println("timersHtml()");timersHtml(&cli);break;                           // timershtml
              case 66: Serial.println("cfgDetServHtml()");cfgDetServHtml(&cli);break;                   // cfgdetservhtml              
              case 67: what=11;{int nb=*(libfonctions+2*i+1)-PMFNCHAR;                                  // lib detserv
                       if(nb>=16){nb-=16;}
                       memset(&libDetServ[nb][0],0x00,LENLIBDETSERV);                                   
                       memcpy(&libDetServ[nb][0],valf,nvalf[i+1]-nvalf[i]);
                       }break;
              case 68: periCur=*(libfonctions+2*i+1)-PMFNCHAR;                                          // bouton periph periline__ (ligne peritable)
                       periLoad(periCur);                                                            
                       periLineHtml(&cli,periCur);
                       break;                                                                                                    
              case 69: break;                                                                           // done         
              case 70: switch(*(libfonctions+2*i)){                                                     // analog_
                          case '@': *periAnalLow=0;conv_atob(valf,periAnalLow);break;
                          case 'A': *periAnalHigh=0;conv_atob(valf,periAnalHigh);break;
                          case 'B': *periAnalOffset1=0;conv_atob(valf,periAnalOffset1);break;
                          case 'C': *periAnalFactor=0;*periAnalFactor=convStrToNum(valf,&j);break;
                          case 'D': *periAnalOffset2=0;*periAnalOffset2=convStrToNum(valf,&j);break;
                          default: break;
                       }break;
              case 71: what=13;rulesfonc(periAnalCb,periAnalDestDet,periAnalRefDet,periAnalMemo);break;         // anrul___ analog input rules
              case 72: what=13;rulesfonc(periDigitCb,periDigitDestDet,periDigitRefDet,periDigitMemo);break;     // dgrul___ digital input_rules
              case 73: {uint8_t nf=*(libfonctions+2*i+1)-PMFNCHAR;                                      // bouton submit rul_init__                                                                                                        
                                                                                                        // n° de la fonction qui utilise rul_init__
                       periCur=0;conv_atob(valf,&periCur);                                              // (0-n, 0=analog input rules ; 1=digital)                                                                                                
                       if(periCur>NBPERIF){periCur=NBPERIF;}periInitVar();periLoad(periCur); 
                       //Serial.print(" fonct 73======");periPrint(periCur);
                       uint8_t* cb;
                       int8_t* memo;                                                                    // effacement check box et memos précédents
                       uint8_t ncb=0;
                       switch(nf){
                         case 0: cb=periAnalCb;ncb=5;memo=periAnalMemo;break;
                         case 1: cb=periDigitCb;ncb=4;memo=periDigitMemo;break;
                         default: break;
                       }
                       if(nf<2){
                          memset(cb,0x00,ncb);                                                          // cb
                          for(uint8_t nm=0;nm<ncb;nm++){                                                // memos
                            if(memo[nm]<NBMEMOS && memo[nm]>=0){memset(memosTable+memo[nm]*LMEMO,0x00,LMEMO);memo[nm]=-1;}
                          }
                       }
                       }break;                                                                                                        
              
              /* fin des fonctions */
              default:break;
              }
              
            }     // i<NBVAL 
          }       // fin boucle nbre params
          
          if(nbreparams>=0){
            Serial.print((unsigned long)millis());Serial.print(" what=");Serial.print(what);Serial.print(" periCur=");Serial.print(periCur);
#ifdef SHDIAGS            
            Serial.print(" strHisto=");Serial.print(strHisto);
#endif            
            histoStore_textdh(&ab,"",strHisto);
            //Serial.print(" what=================");periPrint(periCur);            
          }                                           // 1 ligne par commande GET

/*
   periAns ou periReq ... periParamsHtml (fait perisave) effectue une réponse ack ou set ou envoie une commande get /set si appellé par perisend
*/                          
        periMess=MESSOK;
        switch(what){                                           
          case 0: break;                                                
          case 1: periMess=periAns(&cli,"ack_______");break;            // periParamsHtml(&cli," ",0,"ack_______");break; // data_save
          case 2: if(ab=='a'){periTableHtml(&cli);}                     // peritable ou remote suite à login
                  if(ab=='b'){remoteHtml(&cli);} break;     
          case 3: periMess=periAns(&cli,"set_______");break;            // data_read //periParamsHtml(&cli," ",0,"set_______");break; // data_read 
          case 4: periMess=periSave(periCur,PERISAVESD);                // switchs
                  SwCtlTableHtml(&cli);
                  cli.stop();
                  cliext.stop();
                  periMess=periReq(&cliext,periCur,"set_______");break;
          case 5: periMess=periSave(periCur,PERISAVESD);                // (periLine) modif ligne de peritable
                  periPrint(periCur);
                  periTableHtml(&cli); 
                  cli.stop();
                  cliext.stop();
                  periMess=periReq(&cliext,periCur,"set_______");break;
          case 6: configSave();cfgServerHtml(&cli);break;               // config serveur
          case 7: timersSave();timersHtml(&cli);break;                  // timers
          case 8: remoteSave();cfgRemoteHtml(&cli);break;               // bouton remotecfg puis submit
          case 9: periRemoteUpdate();remoteHtml(&cli);                  // bouton remotehtml ou remote ctl puis submit 
                  /* la mise à jour des périphériques (perToSend) ne peut pas être faite par periRemoteUpdate 
                   car il n'y a plus de socket dispo pour créer la connexion vers les périphériques (periserver/commonserver/cli_a/cliext)
                   donc remoteHtml d'abord puis perToSend */
                  cli.stop();
                  cliext.stop();
                  periMess=perToSend(tablePerToSend,remotetime);break; 
          case 10:memDetSave();periDetecUpdate();periTableHtml(&cli);  // bouton submit détecteurs serveur
                  /* voir periRemoteUpdate() */
                  cli.stop();
                  cliext.stop();
                  periMess=perToSend(tablePerToSend,srvdettime);break;
          case 11:memDetSave();cfgDetServHtml(&cli);break;                // bouton cfgdetserv puis submit         
          case 12:thermosSave();thermoCfgHtml(&cli);break;                // thermos
          case 13:memosSave(-1);
                  periSave(periCur,PERISAVESD);                          // bouton submit periLine (MàJ/analog/digital)                                             
                  periLineHtml(&cli,periCur);break;                                                                                                           

          default:accueilHtml(&cli);break;
        }
        valeurs[0]='\0';
        purgeServer(&cli);
        cli.stop();
        cliext.stop();
        Serial.print(" pM=");Serial.print(periDiag(periMess));Serial.print(" *** cli stopped - ");Serial.println(millis()-cxtime); 

    //} // cli.connected
}


/* ***************** serveurs ************************/

void udpPeriServer()
{
  ab='u';
  IPAddress rip;
  
  int udpPacketLen = Udp.parsePacket();
 
  if (udpPacketLen){
    udpDataLen=udpPacketLen;
    if(udpPacketLen>UDPBUFLEN){udpDataLen=UDPBUFLEN-1;}
    
      rip = (uint32_t) Udp.remoteIP();
      memcpy(remote_IP,(char*)&rip+4,4);
      remote_Port = (uint16_t) Udp.remotePort();
      Udp.read(udpData,udpDataLen);udpData[udpDataLen]='\0';

//      Serial.print("port=");Serial.println(remote_Port);
//      dumpstr(udpData,128);
      packMac((byte*)remote_MAC,(char*)(udpData+MPOSMAC+33));   // 33= "GET /cx?peri_pass_=0011_17515A29?"
      
      lastcxu=millis();     // trig watchdog
      commonserver(cli_udp,udpData,udpDataLen);                 // cli bid pour compatibilité d'arguments avec les fonction tcp
      
      Serial.println(" *** end udp");
    }
    //else{Udp.flush();Serial.print("Udp overflow=");Serial.print(udpPacketLen);Serial.print(" from ");Serial.println(rip);}
}



void tcpPeriServer()
{
  ab='a';
      if(cli_a = periserv.available())      // attente d'un client
      {
        getremote_IP(&cli_a,remote_IP,remote_MAC);      
        //serialPrintIp(remote_IP);Serial.println(" connecté");
        if (cli_a.connected()){         
          lastcxt=millis();             // trig watchdog
          commonserver(cli_a," ",1);}
      Serial.println(" *** end tcp");
      }
}

void pilotServer()
{
  ab='b';
     if(cli_b = pilotserv.available())      // attente d'un client
     {
        getremote_IP(&cli_b,remote_IP,remote_MAC);      
        //serialPrintIp(remote_IP);Serial.println(" connecté");
        if (cli_b.connected()){
          lastcxt=millis();             // trig watchdog
          commonserver(cli_b," ",1);}
     }     
}


void testUdp()
{
#define MAX_LENGTH_TEST 200

  Serial.println("\nlancer le test Udp sur l'autre machine \n");
  
  while(1){ 
    int packetSize = Udp.parsePacket(); 
    IPAddress ipAddr;
    unsigned int rxPort;
    char data[MAX_LENGTH_TEST];
    if (packetSize){
      ipAddr = (uint32_t) Udp.remoteIP();
      Serial.print("Received packet of size ");Serial.println(packetSize);
      Serial.print("From ");Serial.print(ipAddr);Serial.print(" ");
    
      rxPort = (unsigned int) Udp.remotePort();
      Serial.print(", port ");Serial.println(rxPort);

      if(packetSize<MAX_LENGTH_TEST){
        Udp.read(data, packetSize);
        data[packetSize]='\0';
        Serial.print("Contents: ");Serial.println(data);
  
        Udp.beginPacket(ipAddr,rxPort);

        char data[]="hello Slave";
        Serial.print("sending (");Serial.print(strlen(data));Serial.print(")>");Serial.print(data);
        Serial.print("< to ");Serial.print(ipAddr);Serial.print(":");Serial.println(rxPort);
  
        Udp.write(data,strlen(data));
        Udp.endPacket();
      }
      else{Serial.println("paquet trop gros...");Udp.flush();}
    }  
  }
}
