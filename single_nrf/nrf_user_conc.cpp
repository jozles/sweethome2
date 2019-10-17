
#include "nrf24l01s_const.h"
#include "nrf_user_conc.h"
#include "nrf24l01s.h"
#include "nrf_powerSleep.h"

#if NRF_MODE == 'C'

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;

/* user includes */

#include <SPI.h>
#include <Ethernet.h> //bibliothèque W5x00 Ethernet
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"


#if TXRX_MODE == 'T'

  byte        localIp[] = CONCNRFIPADDR;                        // IP fixe pour carte W5x00   (192.168.0.30)
  int         port      = PORTTCPCONC;                          //
//byte        host[]    = HOSTIPADDR2;                          // ip server sh devt2
  byte        host[]    = {82,64,32,56};
  int         hport     = PORTPERISERVER2;                      // port server sh devt2

#define CLICX cli.connected()
#define CLIAV cli.available()
#define CLIRD cli.read()
//#define CLIST for(k=0;k<k1;k++){data[k+k2]=cli.read();}data[k+k2]='\0'
#define CLIST for(k=k2;k<k1;k++){data[k]=cli.read();}data[k]='\0'
//#define CLIST cli.readBytesUntil('\0',data+k2,k1);data[k1+k2]='\0'
#define CLIZER etatImport=0

#endif TXRX_MODE == 'T'

#if TXRX_MODE == 'U'

#include <EthernetUdp.h>

  EthernetUDP Udp;

  byte          localIp[] = CONCNRFIPADDR;        // IP fixe pour carte W5x00   (192.168.0.31)
  unsigned int  port     = PORTUDPCONC;           // (8887)
  IPAddress     host(192, 168, 0, 36);            // ip server sh devt2
  //byte        host[]   = {82,64,32,56};
  //byte        host[]   = {192,168,0,36};
  unsigned int  hport    = 8886;                  // port server sh devt2

IPAddress     rxIpAddr;   // IPAddress from received message
unsigned int  rxPort;     // port      from received message
int   cliav=0;      // len reçue dans le dernier paquet
int   clipt=0;      // prochain car à sortir du dernier paquet;
char  udpData[LBUFSERVER+1];

#define CLICX 1
#define CLIAV (cliav-clipt)
#define CLIRD udpData[clipt];clipt++
#define CLIST memcpy(data+k2,udpData+clipt,k1-k2);clipt+=(k1-k2);data[k1]='\0'
#define CLIZER etatImport=0;cliav=0

#endif TXRX_MODE == 'U' 

int k,k1,k2;    // pour macros TCP/UDP
char c;         // pour macros TCP/UDP

  byte        mac[]     = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};      // mac addr for local ethernet carte W5x00

  EthernetClient cli;   

/* user fields */

  char model[]={"D32800"};

#define LBODY 6 // "<body>"
  extern char bufServer[BUF_SERVER_LENGTH];

  char* srvpswd=PERIPASS;
  byte  lsrvpswd=LPWD;

  char* fonctions={"set_______ack_______etat______reset_____sleep_____testaoff__testa_on__testboff__testb_on__last_fonc_"};
  uint8_t fset_______,fack_______,fetat______,freset_____,fsleep_____,ftestaoff__,ftesta_on__,ftestboff__,ftestb_on__;;
  int     nbfonct;

  char* chexa="0123456789ABCDEFabcdef\0";

  char getHDmess[1000];

/* importDSata & getHData times */

  unsigned long t1;     // beg importData()
  unsigned long t1_0;   // MESSCX
  unsigned long t1_01;  // udpRead()
  unsigned long t1_1;   // MESSLEN got '<body>'
  unsigned long t1_2;   // MESSLEN got len
  unsigned long t1_03;  // MESSLEN got '</body>'
  unsigned long t1_3;   // MESSLEN got '</body>' + check it
  unsigned long t1_4;   // MESSOK
  unsigned long t2;     // 
  unsigned long t2_1;   // 

/* exportDSata & mess2Server times */

  unsigned long t3;       // beg exportData
  unsigned long t3_0;     // fin message building
  unsigned long t3_01=0 ; // cx
  unsigned long t3_02=0 ; // tfr  
  unsigned long t3_1=0;   // 
  unsigned long t3_2=0;   //  

/* réception/decodage messages serveur */

  uint8_t   etatImport=0;
  char*     intro="<body>";
  uint8_t   introLength1=6;
  uint8_t   introLength2=15;
  char*     suffix="</body>";
  uint8_t   suffixLength=7;
  uint16_t  messLength=0;
  uint16_t  l;
  uint8_t   crcAsc;
  uint8_t   crcLength=2;
  int       lastPeriMess;
  char      indata[LBUFSERVER+1];
  uint8_t   lastEtatImport;

/* cycle functions */

void userResetSetup()
{
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
  
IPAddress slaveIp(192, 168, 0, 31);
#define IP slaveIp
#define PORT 8887
  Serial.println();
  for(int i=0;i<6;i++){Serial.print(mac[i],HEX);if(i!=5){Serial.print(":");}}
  Serial.print(" ready on ");
  for(int i=0;i<4;i++){Serial.print(IP[i]);if(i!=3){Serial.print(".");}}

// start ethernet,udp  
  Ethernet.begin(mac, IP);
  Serial.print(" Udp.begin ");Serial.print(PORT);
  if(!Udp.begin(PORT)){Serial.println(" ko");while(1){};}
  Serial.println(" ok");
/*  Serial.print("\nEthernet mac=");serialPrintMac(mac,0);
  if(Ethernet.begin((uint8_t*)mac) == 0){
    Serial.print(" failed with DHCP... forcing Ip ");serialPrintIp(localIp);delay(10);
    Ethernet.begin ((uint8_t*)mac, localIp);
  }
  
#if TXRX_MODE == 'U'
  Serial.print(" UDPport=");Serial.print(port);
  if(!Udp.begin(port)){Serial.print(" begin ko ");while(1){};}
  else{Serial.print(" begin ok");}
#endif TXRX_UDP  
*/  
  Serial.print(" local IP=");Serial.print(Ethernet.localIP());Serial.print(" ");Serial.println(millis()-t_beg);
}

//int mess2Server(EthernetClient* cli,byte* host,int hport,char* data)    // connecte au serveur et transfère la data
int mess2Server(EthernetClient* cli,IPAddress host,unsigned int hport,char* data)    // connecte au serveur et transfère la data
{
#ifdef DIAG
/*  
  Serial.print(TXRX_MODE);Serial.print(" to ");
  for(int i=0;i<4;i++){Serial.print((uint8_t)host[i]);Serial.print(" ");}
  Serial.print(":");Serial.print(hport);Serial.print("...");
*/
#endif DIAG

#if TXRX_MODE == 'U'
  Udp.beginPacket(host,hport);
  t3_01=micros();
  Udp.write(data,strlen(data));
  Udp.endPacket();
  return 1;
#endif TXRX_UDP

#if TXRX_MODE == 'T'  
  int     cxStatus=0;
  int     repeat=-1;
  #define MAXREPEAT 4
  #define CXDLY 100    

  while(!cxStatus && repeat<MAXREPEAT){
    
    repeat++;
    cxStatus=cli->connect(host,hport);
    cxStatus=cli->connected();
#ifdef DIAG 
    Serial.print(repeat);Serial.print("/");Serial.print(cxStatus);
    switch(cxStatus){
        case  1:Serial.print(" ok ");break;
        case -1:Serial.print(" time out ");break;
        case -2:Serial.print(" invalid server ");break;
        case -3:Serial.print(" truncated ");break;
        case -4:Serial.print(" invalid response ");break;
        default:Serial.print(" unknown reason ");break;
    }
#endif DIAG
    t3_01=micros();
    if(cxStatus){
      cli->write(data);
      //cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");
      return 1;
    }
    delay(CXDLY);
  }
  return 0;
 
#endif TXRX_TCP
}

#if TXRX_MODE == 'U'
int intro_Udp()
{
  cliav=Udp.parsePacket();
  if(cliav>0){
    clipt=0;
    rxIpAddr = (uint32_t) Udp.remoteIP();
    rxPort = (unsigned int) Udp.remotePort();
    //Serial.print(ipAddr);Serial.print(":");Serial.print((int)rxPort);Serial.print(" l=");Serial.print(cliav);
    Udp.read(udpData,cliav);udpData[cliav]='\0';
    t1_01=micros();
    //Serial.println(udpData);
  }
  return cliav;
}
#endif

int getHData(char* data,uint16_t* len)
{
/*
  mode_attente_<
    attendre le caractère '<'
    si la suite n'est pas 'body>' attendre

  mode_attente_longueur >=15 (10 fonction, 1 '=', 4 longueur)
    charger 15 car et décoder la longueur

  mode_attente_longueur >= longueur message+7 (7 '</body>')
    charger 
    contrôles </body> et crc si ko retour au mode_attente_<
*/
/*    retour MESSCX not connected ; MESSLEN en cours selon etatImport ; MESSOK messLength in data  */

  if(etatImport==0){messLength=0;data[0]='\0';}

#if TXRX_MODE == 'U'
  if(clipt>=cliav){intro_Udp();}
#endif
  
  if(!CLICX){t1_0=micros()-t1;return MESSCX;}                                         // not connected
  
  switch(etatImport){
    case 0: if(CLIAV>=introLength1){                                                  // attente intro et contrôle     
              for(k=0;k<introLength1;k++){char c=CLIRD;if(c!=intro[k]){break;}}
              if(k>=introLength1){etatImport++;}}
            t1_1=micros()-t1;
            break;
    case 1: if(CLIAV>=introLength2){                                                  // attente longueur message
              k1=introLength2;k2=0;CLIST;                                             // load fonction+len
              for(k=4;k>0;k--){
                //Serial.print(" ");Serial.print(introLength2-k);Serial.print("=");Serial.print(data[introLength2-k]);
                messLength*=10;messLength+=data[introLength2-k]-'0';}
              //Serial.print(" -> ");Serial.println(messLength);
              messLength+=suffixLength;etatImport++;}            
            t1_2=micros()-t1;
            break;
    case 2: if(CLIAV>=messLength-2){                                                  // attente message
              k2=introLength2;k1=(messLength-crcLength)+k2;CLIST;                     // load message+ctle suffixe
              t1_03=micros()-t1;
              k1=messLength-suffixLength+introLength2-crcLength;
              for(k=0;k<suffixLength;k++){
                if(data[k+k1]!=suffix[k]){break;}}                                     // controle suffixe              
              //Serial.print("l=");Serial.print(messLength);Serial.print(" GHD=");Serial.println(data);
              if(k>=suffixLength){etatImport++;}
              else {messLength=0;data[0]='\0';CLIZER;}}
            t1_3=micros()-t1;
            break;
    case 3: conv_atoh(&data[messLength-suffixLength+introLength2-crcLength-crcLength],&crcAsc);    // contrôle crc
            //Serial.print(crcAsc,HEX);Serial.print(" ");Serial.print((char*)(data+introLength2-4));Serial.print(" ");Serial.println(messLength-suffixLength);
            CLIZER;
            if(calcCrc((char*)(data+introLength2-4),messLength-suffixLength)==crcAsc){
              t1_4=micros()-t1;
              return MESSOK;}
            else messLength=0;data[0]='\0';CLIZER;
            break;            
    default:CLIZER;break;
  }
  return MESSLEN;
}

int exportData(uint8_t numT)                            // formatting periBuf data in bufServer 
                                                        // sending bufServer to server 
{
#ifdef DIAG
  Serial.print("<<< exportData "); 
#endif
  t3=micros();
  strcpy(bufServer,"GET /cx?\0");
  if(!buildMess("peri_pass_",srvpswd,"?")==MESSOK){
    Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(srvpswd);return MESSDEC;};

  char message[LENVAL];
  int sb=0,i=0;
  char x[2]={'\0','\0'};
  
      sprintf(message,"%02d",tableC[numT].numPeri);                 // N° périf                    
      memcpy(message+2,"_\0",2);                     
      sb=3;
      unpackMac((char*)(message+sb),tableC[numT].periMac);          // macaddr             
      sb+=17;
      memcpy(message+sb,"_\0",2);
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+6+LENVERSION+8+4+1,6); // temp                              
      sb+=6;
      memcpy(message+sb,"__\0",3);                                  // âge 
      sb+=2;
      memcpy(message+sb,tableC[numT].periBuf+6+LENVERSION+8,4);     // volts                              
      sb+=4;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+6,LENVERSION);         // VERSION
      sb+=LENVERSION-1;
      memcpy(message+sb,tableC[numT].periBuf+6+LENVERSION+8+4,1);   // modele DS18x20
      sb+=1;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;      
      
#define NBSW 0
      message[sb]=(char)(NBSW+48);                                  // nombre switchs              - 1   
      //for(i=0;i<NBSW;i++){message[sb+1+(MAXSW-1)-i]=(char)(48+digitalRead(pinSw[i]));}     
      if(NBSW<MAXSW){for(i=NBSW;i<MAXSW;i++){message[sb+1+(MAXSW-1)-i]='x';}}
      memcpy(message+sb+MAXSW+1,"_\0",2);                       // message[sb+MAXSW+1]='_';
      sb+=MAXSW+2;
      
#define NBDET 0
      message[sb]=(char)(NBDET+48);                                 // nombre détecteurs
      //for(i=(NBDET-1);i>=0;i--){message[sb+1+(NBDET-1)-i]=(char)(chexa[cstRec.memDetec[i]]);}
      if(NBDET<MAXDET){for(i=NBDET;i<MAXDET;i++){message[sb+1+i]='x';}}
      memcpy(message+sb+MAXDET+1,"_\0",2);         
      sb+=MAXDET+2;

      for(i=0;i<NBPULSE;i++){message[sb+i]='0';} //chexa[staPulse[i]];}
      memcpy(message+sb+NBPULSE,"_\0",2);                               // clock pulse status          - 5
      sb+=NBPULSE+1;

char model[LENMODEL];

  model[0]='D'; //CARTE;
  model[1]='D'; //POWER_MODE;
  model[2]='_'; //CONSTANT;
  model[3]='1';
  model[4]=(char)(NBSW+48);
  model[5]=(char)(NBDET+48);  

      memcpy(message+sb,model,LENMODEL);
      memcpy(message+sb+LENMODEL,"_\0",2);
      sb+=LENMODEL+1;
//Serial.print("exportData message=");Serial.println((char*)message);
if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL);}

    uint8_t fonction=0;                                                                       // data_read_ -> set
    char fonctName[]={"data_read_"};
    if(tableC[numT].numPeri!=0){memcpy(fonctName,"data_save_",LENNOM);fonction=1;}            // data_save_ -> ack
    buildMess(fonctName,message,"");                                                          // buld message for server

    t3_0=micros();

/* send to server */
    
    int periMess=-1;
    int cnt=0;
    #define MAXRST 2      // nombre de redémarrages ethernet si pas de connexion
    
    while(cnt<MAXRST){
      periMess=mess2Server(&cli,host,hport,bufServer);                                  // send message to server
      if(periMess!=-7){cnt=MAXRST;t3_02=micros();}
      else {cnt++;if(cnt<MAXRST){t3_1=micros();userResetSetup();t3_2=micros();}}}       // si connecté fin sinon redémarrer ethernet

#ifdef DIAG
    Serial.print(" periMess=");Serial.print(periMess);
    Serial.print(" buildmess=");Serial.print(t3_0-t3);
    Serial.print(" cx=");Serial.print(t3_01-t3_0);Serial.print(" tfr=");Serial.print(t3_02-t3_01);
    Serial.print(" userResetSetup=");Serial.println(t3_2-t3_1);
    Serial.print("    ");Serial.println(bufServer);
#endif
    return periMess;
}


int  importData()                // reçoit un message du serveur
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
  int  numT,nP,len,numPeri;
  char fromServerMac[6];
  int  periMess;

  int  dataLen=LBUFSERVER;
  
  t1=micros();
  periMess=getHData(indata,(uint16_t*)&dataLen);
/*  
    if(periMess!=lastPeriMess || etatImport!=lastEtatImport){lastPeriMess=periMess;lastEtatImport=etatImport;
    Serial.print("****** importData() periMess=");Serial.print(periMess);
    Serial.print(" etatImport=");Serial.print(etatImport);
    Serial.print(" len=");Serial.print(messLength);
    Serial.print(" indata=");Serial.println(indata);}
*/

  if(periMess==MESSOK){
        packMac((byte*)fromServerMac,(char*)(indata+MPOSMAC));    // mac from set message (LBODY pour "<body>")
        nP=convStrToNum(indata+MPOSNUMPER,&dataLen);              // numPer from set message
        numT=nrfp.macSearch(fromServerMac,&numPeri);              // numT mac reg nb in conc table ; numPeri from table numPeri 
                                                                  // numPeri should be same as nP (if !=0 && mac found)
        t2=micros();
        int eds=99;
        if(numT>=NBPERIF){periMess=MESSMAC;}                      // if mac doesnt exist -> error
        else if(numPeri!=0 && numPeri!=nP){periMess=MESSNUMP;}    // if numPeri doesnt match message -> error
        else {eds=nrfp.extDataStore(nP,numT,indata+MPOSPERREFR,SBLINIT);} // format MMMMM_UUUUU_xxxx MMMMM aw_min value ; UUUUU aw_ok value ; xxxx user dispo 
        t2_1=micros();                                                    // (_P.PP pitch value)
        
#ifdef DIAG                
        Serial.print(">>> getHD ");
        Serial.print(rxIpAddr);Serial.print(":");Serial.print((int)rxPort);Serial.print(" l=");Serial.print(cliav);
        Serial.print("/");Serial.print(messLength);Serial.print(" noCX=");Serial.print(t1_0);Serial.print(" intro=");Serial.print(t1_1);Serial.print(" len=");Serial.print(t1_2);
        Serial.print(" suffix=");Serial.print(t1_03);Serial.print(" s+chk=");Serial.print(t1_3);
        Serial.print(" ok=");Serial.println(t1_4);Serial.print("    data=");Serial.println(indata);
        Serial.print("    importData ok=");Serial.print(t2_1-t1);Serial.print(" (extDataStore=");Serial.print(t2_1-t2);Serial.print(")");
        Serial.print(" nP=");Serial.print(nP);Serial.print(" numT=");Serial.print(numT);Serial.print(" numPeri=");Serial.print(numPeri);
        Serial.print(" eds=");Serial.print(eds);Serial.print(" fromServerMac=");for(int x=0;x<5;x++){Serial.print(fromServerMac[x]);}
        Serial.print(" print diag=");Serial.print(micros()-t2_1);
        Serial.println();        
#endif        
  }
  return periMess;
}

#endif // NRF_MODE == 'C'
