#include <Arduino.h>
#include "radio_const.h"
#include "radio_util.h"
#include "radio_user_conc.h"
#include "radio_powerSleep.h"

#ifdef  NRF
#include "nrf24l01s.h"
extern Nrfp radio;
#endif

#ifdef LORA
#include "LoRa.h"
#include "LoRa_const.h"
extern LoRaClass radio;
#endif

/* gestion user data du concentrateur */

#if MACHINE_CONCENTRATEUR
#ifndef DETS

struct ConTable tableC[NBPERIF+1];

/* user includes */

#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h> 
#include <shconst2.h>
#include <shutil2.h>
#include <shmess2.h>

extern bool diags;
extern unsigned long blktime;

#if TXRX_MODE == 'T'        // TCP

  byte        localIp[] = CONCTCPIPADDR;      // IP fixe pour carte W5x00   (192.168.0.30)
  int         port      = PORTTCPCONC;     
//byte        host[]    = HOSTIPADDR2;        // ip server sh devt2
  byte        host[]    = {82,64,32,56};
  int         hostPort     = PORTPERISERVER2;    // port server sh devt2

#define INTROLENGTH1 6  // <body>
#define INTROLENGTH2 15 // nom_fonct_=llll
#define INTROLENGTH INTROLENGTH2
#define PREVLENGTH 0
#define CLICX cli.connected()
#define CLIAV cli.available()
#define CLIRD cli.read()
//#define CLIST for(k=0;k<k1;k++){data[k+k2]=cli.read();}data[k+k2]='\0'
#define CLIST for(k=k2;k<k1;k++){data[k]=cli.read();}data[k]='\0'
//#define CLIST cli.readBytesUntil('\0',data+k2,k1);data[k1+k2]='\0'
#define CLIZER etatImport=0;
#define ELSECLIZER else{}

#endif //  TXRX_MODE == 'T'

#if TXRX_MODE == 'U'

EthernetUDP Udp;

//uint16_t          port     = 8888;    // conc udp port                          // PORTUDPCONC;         // (8887 intérieur ; 8888 ext)
//byte              mac[]    = {0xDE,0xAD,0xBE,0xEF,0xFE,0xEF};                   // MACADDRUDP2;         //{0xDE,0xAD,0xBE,0xEF,0xFE,0xED};      // mac addr for local ethernet carte W5x00

IPAddress         localIp;
extern byte*      serverIp;
extern byte*      concIp;

extern uint16_t*  concPort;
extern byte*      concMac;
extern uint8_t*   concNb;
extern uint16_t   hostPort;              // host port (TCP/UDP selon TXRX_MODE)    // PORTUDPSERVER2;      // port server sh devt2

IPAddress host;

IPAddress     rxIpAddr;   // IPAddress from received message
unsigned int  rxPort;     // port      from received message
uint32_t   cliav=0;       // len reçue dans le dernier paquet
uint32_t   clipt=0;       // prochain car à sortir du dernier paquet;
char  udpData[LBUFSERVER+1];
uint32_t   getUdp_cnt=0;
uint32_t   uRScnt=0;

#define INTROLENGTH1 6  // <body>
#define INTROLENGTH2 15 // nom_fonct_=llll
#define INTROLENGTH INTROLENGTH1+INTROLENGTH2
#define PREVLENGTH INTROLENGTH
#define CLICX 1
#define CLIAV (cliav-clipt)
#define CLIRD udpData[clipt];clipt++
#define CLIST memcpy(data+k2,udpData+clipt,k1-k2);clipt+=(k1-k2);data[k1]='\0'
#define CLIZER etatImport=0;cliav=0
#define CLIZER2 CLIZER;Serial.print(cliav);Serial.println(" CLIZER2")

#endif //  TXRX_MODE == 'U' 

int k,k1,k2;    // pour macros TCP/UDP
char c;         // pour macros TCP/UDP

  EthernetClient cli;   

/* user fields */

  extern unsigned long perConc;

#define LBODY 6 // "<body>"
  extern char bufServer[BUF_SERVER_LENGTH];

  extern char* peripass;

  const char* fonctions={"set_______ack_______etat______reset_____sleep_____testaoff__testa_on__testboff__testb_on__last_fonc_"};
  uint8_t fset_______,fack_______,fetat______,freset_____,fsleep_____,ftestaoff__,ftesta_on__,ftestboff__,ftestb_on__;;
  int     nbfonct;

  //const char* chexa="0123456789ABCDEFabcdef\0";

  char getHDmess[1000];

  extern char ram_remanente;

/* importData & getHData times */

  unsigned long t1;     // beg importData()
  unsigned long t1_0;   // MESSCX  
  unsigned long t1_01;  // udpRead()
  unsigned long t1_1;   // MESSLEN got '<body>'
  unsigned long t1_2;   // MESSLEN got len
  unsigned long t1_03;  // MESSLEN got '</body>'
  unsigned long t1_3;   // MESSLEN got '</body>' + check it
  unsigned long t1_4;   // MESSOK (crc checked)
  unsigned long t2;     // 
  unsigned long t2_0;   // 
  unsigned long t2_1;   // 
  unsigned long t2_2;   // 

/* exportDSata & mess2Server times */

  unsigned long t3;       // beg exportData
  unsigned long t3_0;     // fin message building
  unsigned long t3_01=0 ; // cx
  unsigned long t3_02=0 ; // tfr  
  unsigned long t3_1=0;   // 
  unsigned long t3_2=0;   //  

  uint8_t messageCnt=0;

/* réception/decodage messages serveur */

  uint32_t  hDataCnt=0;
  uint8_t   prevEtatImport=0;
  uint8_t   etatImport=0;
  const char*     intro="<body>";
  uint8_t   introLength1=6;   // <body>
  uint8_t   introLength2=INTROLENGTH2;  // nom_fonct_=llll
  uint8_t   introLength=INTROLENGTH1+INTROLENGTH2;
  uint8_t   prevLength=PREVLENGTH;
  const char*     suffix="</body>";
  uint8_t   suffixLength=7;
  uint32_t  messLength=0;
  uint16_t  l;
  uint8_t   crcAsc;
  uint8_t   crcCal;
  uint8_t   crcLength=2;
  int       lastPeriMess;
  char      indata[LBUFSERVER+1];
  uint8_t   lastEtatImport;
  uint32_t  tLastImport=0;
  extern unsigned long lastUdpCall;

  unsigned long blkwd=0;
  #define TBLKCTL 2000

  extern uint8_t exportCnt;

  uint8_t blkCtlPin1;
  uint8_t blkCtlPin2;
  uint8_t blkCtlPin3;

/* cycle functions */

int mess2Server(EthernetClient* cli,IPAddress host,uint16_t hostPort,char* data);    // connecte au serveur et transfère la data

void blkCtlInit(uint8_t blkPin1,uint8_t blkPin2,uint8_t blkPin3)
{
  blkCtlPin1=blkPin1;blkCtlPin2=blkPin2;blkCtlPin3=blkPin3;
  pinMode(blkCtlPin1,OUTPUT);pinMode(blkCtlPin2,OUTPUT);pinMode(blkCtlPin3,OUTPUT);
}

void blkCtl(uint8_t where)
{
  ram_remanente=where;
  digitalWrite(blkCtlPin1,!(where&0x01));digitalWrite(blkCtlPin2,!((where>>1)&0x01));digitalWrite(blkCtlPin3,!((where>>2)&0x01));
  //Serial.println(where);
  //if((millis()-blkwd)>TBLKCTL){
    //char a[8]={'#','#',' ',where,' ','\0'};
    //Serial.print(a);Serial.print(millis());Serial.print(' ');Serial.println(lastUdpCall);
  //}
}

void userResetSetup(byte* serverIp,const char* mailMessage)
{
  uRScnt++;
  exportCnt=0;

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
     
  unsigned long t_beg=millis();

  /* params conc1 
  memcpy(concMac,"\xDE\xAD\xBE\xEF\xFE\xED",6);
  concIp[0]=192;concIp[1]=168;concIp[2]=0;concIp[3]=31;
  *concPort=8887;
  */
 //*concPort=8888;
 
  uint32_t w5500hr=5000;
  uint32_t w5500st=100;
  Serial.print(" w5500 hard Reset ");Serial.print(w5500hr+w5500st);Serial.println("mS ");
  digitalWrite(A8,HIGH);pinMode(A8,OUTPUT);digitalWrite(A8,LOW);trigDly(w5500hr);
  digitalWrite(A8,HIGH);trigDly(100);    // w5500 hard Reset

  Serial.print(" Ethernet begin mac/Ip=");serialPrintMac(concMac,0);Serial.print('/');
  for(uint8_t i=0;i<4;i++){localIp[i]=concIp[i];}Serial.println((IPAddress)localIp);
  
  Ethernet.begin (concMac,localIp);
  /*
  if(Ethernet.begin (concMac) == 0){
    Serial.print("\nFailed with DHCP... forcing Ip ");serialPrintIp(concIp);Serial.println();
    for(uint8_t i=0;i<4;i++){localIp[i]=concIp[i];}Serial.print((IPAddress)localIp);Serial.println();
    WDTRIG //trigwd(0);
    Ethernet.begin (concMac, localIp); 
    Serial.print(" localIP=");
  }
  */
  Serial.print(" host :");
  for(uint8_t i=0;i<4;i++){host[i]=serverIp[i];localIp[i]=Ethernet.localIP()[i];}
  Serial.print((IPAddress)host);Serial.print("/");Serial.println(hostPort);
  Serial.print(" local:");
  Serial.print((IPAddress)localIp);Serial.print("/");Serial.println();

  WDTRIG //trigwd(0);

#if TXRX_MODE == 'U'
  /*if(ethernetUdp!=nullptr){
    delete ethernetUdp;ethernetUdp=nullptr;
  }
  if(ethernetUdp==nullptr){
    ethernetUdp = new  EthernetUDP ;}
  */
  Serial.print(" Udp.begin (");Serial.print(*concPort);Serial.print(")");

  Udp.stop();
  if(Udp.begin(*concPort)){Serial.print(" ok ");Serial.print(millis()-t_beg);Serial.println("mS");}
    else while(1){};      // si udp ko... reboot 

  WDTRIG //trigwd(0);
#endif // TXRX_MODE == 'U'

  blkwd=millis();

  cliav=0;

  if(mailMessage!=nullptr && uRScnt<19){
    char uRSMessage[16];uRSMessage[0]='\0';strcat(uRSMessage,mailMessage);
    char* v=uRSMessage+strlen(uRSMessage);
    sprintf(v,"%02d",(int)uRScnt);*(v+6)='\0';
    exportDataMail(uRSMessage,5);delay(10);
  }
}

void userResetSetup(byte* serverIp)
{
  userResetSetup(serverIp,nullptr);
}

int mess2Server(EthernetClient* cli,IPAddress host,uint16_t hostPort,char* data)    // connecte au serveur et transfère la data{
{
#ifdef DIAG
//if(diags){  
  Serial.print(TXRX_MODE);Serial.print(" to ");
  for(int i=0;i<4;i++){Serial.print((uint8_t)host[i]);Serial.print(" ");}
  Serial.print(":");Serial.print(hostPort);Serial.print("...");
//}
#endif //  DIAG

#if TXRX_MODE == 'U'
  Udp.beginPacket(host,hostPort);
  t3_01=micros();
  Udp.write(data,strlen(data));
  Udp.endPacket();
  delayMicroseconds(200);   // protect again concurrent SPI usage ? (@16Mhz 100uS=200 bytes sent)
  return MESSOK;
#endif //  TXRX_MODE == 'U'

#if TXRX_MODE == 'T'  
  int     cxStatus=0;
  int     repeat=-1;
  #define MAXREPEAT 4
  #define CXDLY 100    

  while(!cxStatus && repeat<MAXREPEAT){
    
    repeat++;
    cxStatus=cli->connect(host,hostPort);
    cxStatus=cli->connected();
    if(diags){
    Serial.print(repeat);Serial.print("/");Serial.print(cxStatus);
    switch(cxStatus){
        case  1:Serial.print(" ok ");break;
        case -1:Serial.print(" time out ");break;
        case -2:Serial.print(" invalid server ");break;
        case -3:Serial.print(" truncated ");break;
        case -4:Serial.print(" invalid response ");break;
        default:Serial.print(" unknown reason ");break;
    }
    }
    t3_01=micros();                   // t3_0 to t3_01 = cx time
    if(cxStatus){
      cli->write(data);
      //cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");
      return MESSOK;
    }
    delay(CXDLY);
  }
  return MESSTO;
 
#endif //  TXRX_MODE == 'T'
}       // messToServer

#if TXRX_MODE == 'U'

int get_Udp()
{ 
  getUdp_cnt++;
  clipt=0;
blkCtl('@');  
  cliav=Udp.parsePacket();
  if(cliav>0){
    rxIpAddr = (uint32_t) Udp.remoteIP();
    rxPort = (unsigned int) Udp.remotePort();
    if(cliav<LBUFSERVER-1){
blkCtl('a');
      Udp.read(udpData,cliav);udpData[cliav]='\0';
      if(diags){
      Serial.print("*eI=");Serial.print(etatImport);Serial.print(" cliav=");Serial.print(cliav);Serial.print(" ");udpData[cliav]='\0';Serial.println(udpData);
      }
    }
    else {
      Serial.print("\nudpPacketovf=");Serial.print(cliav);Serial.print(' ');Serial.print(getUdp_cnt);Serial.print(" uRScnt=");Serial.print(uRScnt);Serial.print(' ');
      Serial.println(" Udp ko... restart");userResetSetup(serverIp,"UDP OVF ");     // reset W5500 + mail OVF
      //if(!Udp.begin(*concPort)){Serial.println(" Udp ko");while(1){trigwd(1000);}}
      /*
      while (cliav>0){
          if(cliav>LBUFSERVER-1){
            Udp.read(udpData,LBUFSERVER-1);
            udpData[LBUFSERVER-1]='\0';
            cliav-=(LBUFSERVER-1);
          }
          else {
            Udp.read(udpData,cliav);cliav=0;
            udpData[cliav]='\0';
          }
          Serial.println(udpData);
      }
      */
    }
    t1_01=micros();
  }
  return cliav;
}
#endif //  TXRX_MODE == 'U'

int getHData(char* data,uint16_t* len)
{
/*
  !!!!!!!!!! fonctionne en Udp ; incomplet en TCP !!!!!!!!
  !!!!!!!!!! CLIRD ne contient pas cli.read()     !!!!!!!!
  !!!!!!!!!! CLIST idem           
  !!!!!!!!!! et cliav n'est pas valorisé

  mode_attente_<
    attendre le caractère '<'
    si la suite n'est pas 'body>' attendre

  mode_attente_longueur >=15 (10 fonction, 1 '=', 4 longueur)
    charger 15 car et décoder la longueur

  mode_attente_longueur >= longueur message+7 (7 '</body>')
    charger 
    contrôles </body> et crc si ko retour au mode_attente_<

  aussi longtemps que des packets trop courts pour l'étape en cours sont reçus, la procédure de réception est ré-initialisée :
  etatImport=0, vidage buffer etc... 
  En principe, un packet contient au moins les caractères de l'étape en cours sinon c'est un défaut de transmission
  (pratiquement un packet devrait contenir la totalité du message)
  En UDP, si plusieurs paquets concatènés devraient être traiés normalement (clipt n'est remis à 0 que lorsque cliav==0)
*/
/*    retour MESSCX not connected ; MESSLEN en cours selon etatImport ; MESSOK messLength in data  */

  hDataCnt++;
  if(etatImport==0){messLength=0;data[0]='\0';}

  if(cliav==0){                 // on suppose que les packets arrivent complets ; si il y a un morceau de paquet, il sera traité en erreur
    get_Udp();                  // get_Udp() charge un éventuel packet et met cliav à jour
  }

  /*
  if(etatImport==0){                           // charger un éventuel packet et controler sa validité
    intro_Udp();
    uint16_t packetLen=introLength1+introLength2+crcLength+suffixLength;
    uint16_t lmess=0;
    if(cliav>=packetLen){                
      for(k=4;k>0;k--){
      lmess*=10;lmess+=data[introLength1+introLength2-k]-'0';}
      packetLen+=lmess;
      if(cliav>=packetLen){
        crcAsc=0;crcCal=0;conv_atoh(&data[packetLen-suffixLength-crcLength],&crcAsc);    // récup crc
        crcCal=calcCrc((char*)(data+introLength1+introLength2-4),lmess);
            if(crcCal!=crcAsc){cliav=0;return MESSCRC;}
      }
      else cliav=0;return MESSLEN;
    }
    else cliav=0;return MESSLEN;                        // rien reçu
  }
  */  

  if(!CLICX){t1_0=micros()-t1;return MESSCX;}                                         // not connected (does not happen in Udp mode)
  
  switch(etatImport){
    case 0: {bool one_time_dump=false;
            uint32_t save_clipt=clipt;
            while(CLIAV>=introLength1 && etatImport==0){                              // search packet for valid intro (would be long : cliav*introLength1)
              for(k=0;k<introLength1;k++){
                char c=CLIRD;                                                         // clipt++
                if(c!=intro[k]){
                  if(!one_time_dump){
                    one_time_dump=true;
                    Serial.print("clipt=");Serial.print(clipt);Serial.print(" getUdp_cnt=");Serial.println(getUdp_cnt);
                    dumpstr(udpData,cliav);}
                  save_clipt++;clipt=save_clipt;
                  break;
                }
              }
              if(k>=introLength1){etatImport++;}                                      // si l'intro est hs etatImport reste 0, cliav inchangé jusqu'à ce que clipt rejoigne clizav
            }                                                                         // tout ce qui vient est lu jusqu'à une intro correcte
                                                                                      // si (cliav-clipt)<introLength1, intro_Udp() est refait et clipt=0
                                                                                      // si l'intro est ok -> suite (cliav=packet len, clipt=len1)
            if(etatImport==0) {CLIZER;}                                               // UDP : cliav trop petit ou intro ko -> attente du prochain paquet
            t1_1=micros()-t1;
            }break;
    case 1: if(CLIAV>=introLength2){                                                  // get message length 
              k1=introLength2;k2=0;CLIST;                                             // load fonction+len (15 car)
              for(k=4;k>0;k--){
                messLength*=10;messLength+=data[introLength2-k]-'0';}                 // conv len message atob
              messLength+=suffixLength;etatImport++;                                  // ajout len suffixe -> suite 
            }                                                                         
            else {CLIZER;}                                                            // UDP : cliav trop petit -> attente du prochain paquet
            t1_2=micros()-t1;
            break;
    case 2: if(CLIAV>=(messLength-2)){                                                // get message
              k2=introLength2;k1=(messLength-crcLength)+k2;CLIST;                     // load message+ctle suffixe
              t1_03=micros()-t1;
              k1=messLength-suffixLength+introLength2-crcLength;
              for(k=0;k<suffixLength;k++){
                if(data[k+k1]!=suffix[k]){
                  dumpstr(udpData,cliav);
                  CLIZER;break;}
              }                                                                       // controle suffixe              
              if(k>=suffixLength){etatImport++;}                          
              else {CLIZER;}                                                          // trop petit -> attente du prochain paquet
            }
            else {CLIZER2;}                                                           // UDP : cliav trop petit -> attente du prochain paquet
            t1_3=micros()-t1;
            break;
    case 3: crcAsc=0;crcCal=0;conv_atoh(&data[messLength-suffixLength+introLength2-crcLength-crcLength],&crcAsc);    // get crc
            crcCal=calcCrc((char*)(data+introLength2-4),messLength-suffixLength);
            if(crcCal==crcAsc){                                                       // contrôle crc
              t1_4=micros()-t1;                                                           
              etatImport=0;                                                             
              if(cliav<=clipt){cliav=0;}                                              // s'il reste à lire ça peut être un paquet suivant
              return MESSOK;                                                          // crc ok
            }
            dumpstr(udpData,cliav);                                                          
            CLIZER;return MESSCRC;                                                    // crc ko
            break;            
    default:CLIZER;break;
  }
  return MESSLEN;
}


int  importData()                // reçoit un message du serveur
                                 // update tLast
                                 // retour MESSOK   ok  
                                 //        MESSCX   pas connecté
                                 //        MESSLEN  vide
                                 //        MESSNUMP numPeri invalide
                                 //        MESSMAC  macaddr pas trouvée dans tableC

                                        // transfert contenu de set ou ack dans variables locales selon contrôles
                                        // déclenché par rxServ qui indique que bufServer est valide
                                        //    contrôle mac addr et numPeriph ;
                                        //    si ok -> tfr params
                                        // retour periMess
{
  int  numT=-99,nP;
  char fromServerMac[6];
  int  periMess;

  int  dataLen=LBUFSERVER;
  
  t1=micros();
  periMess=getHData(indata,(uint16_t*)&dataLen);                  // la longueur du message est messLength-suffixLength (si periMess=MESSOK)                                                                
                                                                  // donc les champs indexés sur la fin sont dans la position indata+messLength-suffixLength-xx
/*
        if(diags){ 
          if(prevEtatImport!=etatImport){
            prevEtatImport=etatImport;
            Serial.print(millis());
            Serial.print(" gu:");Serial.print(getUdp_cnt);
      //Serial.print(" hDataCnt");Serial.print(hDataCnt);
            Serial.print(" eI:");Serial.print(etatImport);
            Serial.print(" av:");Serial.print(cliav);
            Serial.print(" pt:");Serial.print(clipt);
            Serial.print(" pM:");Serial.println(periMess);
          }
        }
*/
  if(periMess==MESSOK){                                           // indata valide crc ok (ei=0)

        exportCnt=0;

        t2_0=micros();
        lastUdpCall=millis();

        packMac((byte*)fromServerMac,(char*)(indata+MPOSMAC));    // macaddr from set message (LBODY pour "<body>")
        int dataNpLen=0;
        nP=convStrToNum(indata+MPOSNUMPER,&dataNpLen);            // nP = numPeri from ack/set message (nb in server table)
        int nPfromTableC=0;
        numT=macSearch(fromServerMac,&nPfromTableC);              // numT mac reg nb in conc table
                                                                  // tableC[numT].numPeri should be == nP (if !=0 && mac found)
        //Serial.print(" nP=");Serial.print(nP);Serial.print(" numT=");Serial.print(numT);                                                                  
        conv_atobl(indata+MPOSDH,&tLastImport,UNIXDATELEN);       
        t2=micros();
        
        if(numT>=NBPERIF){periMess=MESSMAC;}                      // if mac doesnt exist -> error
        else if(numT!=0 && nPfromTableC!=0 && nPfromTableC!=nP){periMess=MESSNUMP;}  // iftableC[numT].numPeri doesnt match message -> error
        else {
          if(nPfromTableC==0){tableC[numT].numPeri=nP;}
          if(memcmp(tableC[numT].periBuf+RADIO_ADDR_LENGTH+1,"1.c",3)>0){         // version périf > 1.c
            extDataStore(nP,numT,0,indata+MPOSPERREFR,5);                 // format MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
            extDataStore(nP,numT,5,indata+MPOSPERREFR+6,5);               // format MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
            extDataStore(nP,numT,10,indata+MPOSPERREFR+12,4);             // format MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
            extDataStore(nP,numT,14,indata+MPOSANALH,8);                  // min/max analogique '_hhhhhhhh'
            extDataStore(nP,numT,22,indata+messLength-5,2);               // periAnalOut consigne analogique 0-FF
            extDataStore(nP,numT,24,indata+messLength-2,2);               // periCfg '_hh' 
          }
          else {                                                                // version périf <= 1.c
            extDataStore(nP,numT,0,indata+MPOSPERREFR,16);                // format MMMMM_UUUUU_PPPP  MMMMM aw_min value ; UUUUU aw_ok value ; PPPP pitch value 100x
            extDataStore(nP,numT,16,indata+MPOSANALH-1,9);                // min/max analogique '_hhhhhhhh'
          }
          if(numT==1){                                                          // entrée 1 de tableC pour concentrateur
            uint32_t pp=0;conv_atobl(tableC[numT].servBuf,&pp,5);perConc=pp*1000;
          }
        }                                                  
        t2_1=micros();

        Serial.print("  >>> getHD ");
        if(!diags){Serial.print(indata);}
        Serial.print("  nP=");Serial.print(nP);Serial.print('/');Serial.print(tableC[numT].numPeri);Serial.print(" numT=");Serial.print(numT);
        Serial.print(" import=");
        t2_2=micros()-t1;Serial.println(t2_2);
          
                    //Serial.print(rxIpAddr);Serial.print(":");Serial.print((int)rxPort);Serial.print(" l=");Serial.print(cliav);
                    //Serial.print("/");Serial.print(messLength);
                    //Serial.print(" noCX=");Serial.print(t1_0);Serial.print(" intro=");Serial.print(t1_1);Serial.print(" len=");Serial.print(t1_2);
                    //Serial.print(" suffix=");Serial.print(t1_03);Serial.print(" s+chk=");Serial.print(t1_3);
          
          //Serial.print(" last getHData =");Serial.print(t2_0-t1);
          //Serial.print("    data mngt =");Serial.print(t2_1-t2_0);Serial.print(" (extDataStore=");Serial.print(t2_1-t2);Serial.print(")");
                    //Serial.print(" fromServerMac"); Serial.print(" :");for(int x=0;x<5;x++){Serial.print(fromServerMac[x]);}
                    //Serial.print(" perConc=");Serial.println(perConc);
          //:::::::::::Serial.print(" print diags=");Serial.println(micros()-t2_1);
        
        //}
  }
blkCtl('b');
  return periMess;
}


int exportData(uint8_t numT,char* mailData)                             // formatting periBuf data in bufServer 
{                                                                       // sending bufServer to server 
  if(mailData==nullptr){Serial.print("  <<< export uRS:");Serial.print(uRScnt);Serial.print(' ');}
  else Serial.print("  <<< mail   ");

  t3=micros();                                          // debut exportData (buildMess+cx+tfr)

  strcpy(bufServer,"GET /cx?\0");
  if(buildMess("peri_pass_",peripass,"?")<=0){
    Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(peripass);return MESSDEC;};

  char message[LENVAL];
  int sb=0,i=0;
  char* perVersAd=tableC[numT].periBuf+6;
  char* perVoltsAd=tableC[numT].periBuf+6+LENVERSION+1+8;
  char* perTempAd=tableC[numT].periBuf+6+LENVERSION+1+8+4;
  char* perThModAd=tableC[numT].periBuf+6+LENVERSION;
  if(memcmp(perVersAd,"1.7",3)>0){
    perVoltsAd=tableC[numT].periBuf+6+LENVERSION+1;
    perTempAd=tableC[numT].periBuf+6+LENVERSION+1+4;
  }
  
      sprintf(message,"%02d",tableC[numT].numPeri);                 // N° périf dans table serveur                    
      memcpy(message+2,"_\0",2);                     
      sb=3;
      unpackMac((char*)(message+sb),tableC[numT].periMac);          // macaddr périf
      message[sb+16]=*concNb+'0';                                   // complétée du n° de concentrateur
      sb+=17;
      memcpy(message+sb,"_\0",2);
      sb+=1;
      memcpy(message+sb,perTempAd,6);                               // temp                              
      sb+=6;
      memcpy(message+sb,"_\0",2);
      sb+=1;
      memcpy(message+sb,"9999",4);                                  // analog value (4 decimal digits maxi) 
      sb+=4;                                                       
      memcpy(message+sb,"_\0",2);                                    
      sb+=1;
      memcpy(message+sb,perVoltsAd,4);                              // volts                              
      sb+=4;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;
      if(numT!=1){memcpy(message+sb,perVersAd,LENVERSION);}         // VERSION perif
      else {memcpy(message+sb,VERSION,LENVERSION);}                 // VERSION conc
      //memcpy(message+sb,"2.9 ",LENVERSION);                       // VERSION provisoire pour tests et debug
      sb+=LENVERSION;
      memcpy(message+sb-1,perThModAd,1);                            // thermo model take place of 4th version char
      memcpy(message+sb,"_\0",2);                             
      sb+=1;      
      
#define NBSW 0
      message[sb]=(char)(NBSW+48);                                  // nombre switchs              - 1   
      if(NBSW<MAXSW){for(i=NBSW;i<MAXSW;i++){message[sb+1+(MAXSW-1)-i]='x';}}
      memcpy(message+sb+MAXSW+1,"_\0",2);                           // message[sb+MAXSW+1]='_';
      sb+=MAXSW+2;
      
#define NBDET 0
      message[sb]=(char)(NBDET+48);                                 // nombre détecteurs
      //for(i=(NBDET-1);i>=0;i--){message[sb+1+(NBDET-1)-i]=(char)(chexa[cstRec.memDetec[i]]);}
      if(NBDET<MAXDET){for(i=NBDET;i<MAXDET;i++){message[sb+1+i]='x';}}
      memcpy(message+sb+MAXDET+1,"_\0",2);         
      sb+=MAXDET+2;

      if(mailData!=nullptr){strcat(message,mailData);strcat(message,"_\0");sb+=(strlen(mailData)+1);}

      for(i=0;i<NBPULSE;i++){message[sb+i]='0';} //chexa[staPulse[i]];}
      memcpy(message+sb+NBPULSE,"_\0",2);                           // clock pulse status          - 5
      sb+=NBPULSE+1; 

      memcpy(message+sb,MODEL,LENMODEL-1);*(message+sb+LENMODEL-1)=*concNb+48;
      memcpy(message+sb+LENMODEL,"_\0",2);
      sb+=LENMODEL+1;

      messageCnt++;
      if(messageCnt>99){messageCnt=0;}
      sprintf(message+sb,"%02d",messageCnt);                        // N° ordre du message (00-99)
      memcpy(message+sb+2,"_\0",2);
      sb+=3;

if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL,PULSEBLINK);}

    char fonctName[]={"data_read_"};
    if(tableC[numT].numPeri!=0){memcpy(fonctName,"data_save_",LENNOM);} // data_save_ -> ack
    if(mailData!=nullptr){memcpy(fonctName,"data_mail_",LENNOM);}
    buildMess(fonctName,message,"");                                    // build message for server

    t3_0=micros();                                                      // fin buildMess

/* send to server */
    
    int periMess=-1;
    int cnt=0;
        
    if(diags){Serial.println(bufServer);}
    else{Serial.print(tableC[numT].numPeri);Serial.print(' ');Serial.print(numT);Serial.print(' ');}
    
    #define MAXRST 2      // nombre de redémarrages ethernet si pas de connexion
    while(cnt<MAXRST){
      periMess=mess2Server(&cli,host,hostPort,bufServer);                          // send message to server
      
      if(periMess!=MESSCX){cnt=MAXRST;t3_02=micros();}
      else {cnt++;if(cnt<MAXRST){t3_1=micros();userResetSetup(serverIp,"SERV CX ");t3_2=micros();}}       // si connecté fin sinon redémarrer ethernet
    }

    //if(diags){
    Serial.print(millis());
    Serial.print(" pM=");Serial.print(periMess);
    if(periMess==MESSOK){exportCnt++;}
    if(mailData!=nullptr){Serial.println();}
    //Serial.print(" buildmess=");Serial.print(t3_0-t3);
    //Serial.print(" cx=");Serial.print(t3_01-t3_0);Serial.print(" tfr=");Serial.print(t3_02-t3_01);
    //Serial.print(" userResetSetup=");Serial.print(t3_2-t3_1);delay(1);
    //Serial.print("    ");Serial.print(bufServer);
    //}
    //Serial.println();
    return periMess;
}

void exportData(uint8_t numT)
{
  exportData(numT,nullptr);
}

void exportDataMail(const char* messName)
{
  char concName[LENMODEL+1]={'C','O','N','C','_','\0','\0'};
  concName[LENMODEL-1]=*concNb+48;
  uint8_t testPeri=1;                                             // peri=1 -> conc
  tableC[testPeri].periBufLength=MAX_PAYLOAD_LENGTH;
  memset(tableC[testPeri].periBuf,' ',MAX_PAYLOAD_LENGTH-1);
  *(tableC[testPeri].periBuf+MAX_PAYLOAD_LENGTH-1)='\0';
  memcpy(tableC[testPeri].periBuf+6,VERSION,LENVERSION);
  memcpy(tableC[testPeri].periBuf+6+LENVERSION,"x",1);
  memcpy(tableC[testPeri].periBuf+6+LENVERSION+1,"0.00",4);       // volts
  memcpy(tableC[testPeri].periBuf+6+LENVERSION+1+4,"+00.00",6);   // température

  char* concData=nullptr;
  char concMail[100];
  if(messName!=nullptr){
    char* v=concMail;
    concData=concMail;
    v[0]='\0';strcat(v,concName);strcat(v,"|");strcat(v,messName);
    char a=*v;
    while(a!='\0'){
      if(a=='_'){*v='-';}
      a=*(++v);
    }
  }
  exportData(testPeri,concData);  // test présence serveur avec périf virtuel des broadcast
  if(concData==nullptr){Serial.print(' ');Serial.println(testPeri);}
}

void exportDataMail(const char* messName,uint8_t maxRetry)    // wait for server response
{
  bool noResponseTo=true;
  #define RWTO 30000  // 30 secondes d'attente 

/*
  unsigned long responseWait=millis();
  exportDataMail(messName);
  while((millis()-responseWait)<RWTO && noResponseTo){
    WDTRIG //trigwd(0);      
    if(importData()==MESSOK){noResponseTo=false;}
  }
  if(noResponseTo){
    userResetSetup(serverIp);
    WDTRIG //trigwd(0);
    exportDataMail(messName);
    responseWait=millis();
    while((millis()-responseWait)<RWTO && noResponseTo){
      WDTRIG //trigwd(0);      
      if(importData()==MESSOK){noResponseTo=false;}
    }
  }
  if(noResponseTo){while(1){}}
*/

  uint8_t cnt=0; 
  while(noResponseTo){    // && cnt<maxRetry+1
      cnt++;       
      exportDataMail(messName);delay(10);           // exportDataMail until noResponseTo false
      unsigned long responseWait=millis();
      while((millis()-responseWait)<RWTO){          // et attente réponse RWTO mS 
        WDTRIG //trigwd(0);      
        if(importData()==MESSOK){noResponseTo=false;responseWait=millis()-RWTO-1;}
      }
    if(cnt==maxRetry && noResponseTo){              // lors du maxRetry export si pas de réponse reset ethernet
      userResetSetup(serverIp);   // ici pas de message !
      WDTRIG //trigwd(0);
    }
    if(cnt>=maxRetry+2){while(1){}}     // après un reset de la carte et 2 export, ça ne fonctionne toujours pas... reboot général
  }
}

void fillMess(byte* message)
{
    // rrrrttttaaappphhhhhhhhHHHHHHHHAACC  rrrr refresh per ; tttt temp per ;
    // aaa temps absolu {(0-7F)-20 5F=96 96*96*96 mS maxi } ; ppp pitch_temp*100 ;
    // hh... HH... userData 1/2 ; AA anal output ; CC periCfg
    memcpy(message+RADIO_ADDR_LENGTH+1,message+RADIO_ADDR_LENGTH+2,4);
    memcpy(message+RADIO_ADDR_LENGTH+1+4,message+RADIO_ADDR_LENGTH+2+4+1,4);
    
    byte q;    
    uint32_t abstime=millis();
  marker(MARKER2);
  Serial.print(' ');Serial.print(abstime,HEX);
    abstime=abstime&(ABSTIME-1);
  Serial.print(" absTime:");Serial.print(abstime);Serial.print('H');Serial.print(abstime,HEX);
    q=(byte)(((abstime>>(ABSTIME_STEP*2))&ABSMASK)+0x20);
    *(message+RADIO_ADDR_LENGTH+1+8)=q;
  //Serial.print(":");Serial.print(q,HEX);
    q=(byte)(((abstime>>ABSTIME_STEP)&ABSMASK)+0x20);
    *(message+RADIO_ADDR_LENGTH+1+9)=q;
  //Serial.print(":");Serial.print(q,HEX);
    q=(byte)((abstime&ABSMASK)+0x20);  
    *(message+RADIO_ADDR_LENGTH+1+10)=q;
  //Serial.print(":");Serial.print(q,HEX);
    
    memcpy(message+RADIO_ADDR_LENGTH+1+11,message+RADIO_ADDR_LENGTH+2+4+1+4+1,3);
}


#endif // DETS
#endif // MACHINE_CONCENTRATEUR