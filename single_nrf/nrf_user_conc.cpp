
#include "nrf24l01s_const.h"
#include "nrf_user_conc.h"
#include "nrf24l01s.h"
#include "nrf_powerSleep.h"

#if NRF_MODE == 'C'

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;

/* user includes */

#include <Ethernet.h> //bibliothèque W5x00 Ethernet
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"

#ifdef TXRX_UDP
  #include <EthernetUdp.h>
  EthernetUDP Udp;
  #define PORTUDP PORTUDPSERVER2
#endif

  //const char* host  = HOSTIPADDR;
  byte        host[]={82, 64, 32, 56};
  //byte host[]={192,168,0,35};                                 // ip server sweethome
  //byte host[]={64, 233, 187, 99};
  //const int   port  = PORTPERISERVER;                         // port server sweethome
  int         port  = PORTPERISERVER2;
  byte        mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};   // mac addr ethernet carte W5x00
  byte        localIp[] = {192,168,1,30};                     // IP fixe pour carte W5x00

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

  unsigned long t0,t0b,t1,t1b,t1c,t1d,t1e,t2,t2b,t2c;




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

  if(Ethernet.begin((uint8_t*)mac) == 0){
    Serial.print("Failed with DHCP... forcing Ip ");serialPrintIp(localIp);Serial.println();delay(10);
    Ethernet.begin ((uint8_t*)mac, localIp);
  }
#ifdef TXRX_UDP
  if(!Udp.begin(PORTUDP)){Serial.println("Udp.begin ko");while(1){};}   
#endif TXRX_UDP
  
  Serial.print(millis()-t_beg);Serial.print(" ");Serial.print(Ethernet.localIP());Serial.print(" ");
}

int mess2Server(EthernetClient* cli,byte* host,int port,char* data)    // connecte au serveur et transfère la data
{

#ifdef DIAG  
  #ifdef TXRX_TCP  
    Serial.print("tx connecting ");
  #endif TXRX_TCP  
  #ifdef TXRX_UDP  
    Serial.print("sending (");Serial.print(strlen(data));Serial.print(")>");Serial.print(data);Serial.print("< to ");
  #endif TXRX_UDP  
  
  for(int i=0;i<4;i++){Serial.print((uint8_t)host[i]);Serial.print(" ");}
  Serial.print(":");Serial.print(port);Serial.print("...");
#endif DIAG

#ifdef TXRX_UDP
  Udp.beginPacket(host,port);
  Udp.write(data,strlen(data));
  Udp.endPacket();
#endif TXRX_UDP

#ifdef TXRX_TCP  
  int             cxStatus=0;
  uint8_t         repeat=-1;
  #define MAXREPEAT 4
  #define CXDLY 100    

  t0=micros();
  t0b=0;

  while(!cxStatus && repeat<MAXREPEAT){
    
    repeat++;
    cxStatus=cli->connect(host,port);
    cxStatus=cli->connected();

#ifdef DIAG 
    Serial.print(repeat);Serial.print("/");Serial.print(cxStatus);
    switch(cxStatus){
        case  1:Serial.println(" ok ");break;
        case -1:Serial.print(" time out ");break;
        case -2:Serial.print(" invalid server ");break;
        case -3:Serial.print(" truncated ");break;
        case -4:Serial.print(" invalid response ");break;
        default:Serial.print(" unknown reason ");break;
    }
#endif
        
    if(cxStatus){
      cli->print(data);
      cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");
      t0b=micros();
      return 1;
    }
    delay(CXDLY);
  }
#ifdef DIAG    
  Serial.println(" failed");return 0;
#endif  
#endif TXRX_TCP
}

int getHData()
{
  t1=micros();                                          // ************ t1 beg getHD
  t1c=0;                                                // wait time
  t1e=0;                                                // cli.read()
 
  char*     data=bufServer;
  uint16_t  len=LBUFSERVER-1;                            // bufServer data length
  int       qAvailable;                                  // data qty available

#ifdef TXRX_UDP
  
  qAvailable = Udp.parsePacket();
 
  if (qAvailable){
    if(qAvailable<len){len=qAvailable;}
    //*ipAddr = (uint32_t) Udp.remoteIP();
    //*rxPort = (unsigned int) Udp.remotePort();
    Udp.read(data,len);
  }
  
#endif TXRX_UDP

#ifdef TXRX_TCP

  #define TIMEOUT 1000

  int pt=0;
  char inch;
  unsigned long timerTo=millis();

  t1b=micros();

  if(!cli.connected()){return MESSCX;}                  // not connected
  else{  
    while((millis()<(timerTo+TIMEOUT))&&(pt<=(len-1))&&(pt<=(LBODY+MPOSPERREFR+SBLINIT))){
        qAvailable=cli.available();
        if(qAvailable>0){
          timerTo=millis();
          t1c+=(micros()-t1b);
          t1d=micros();
          data[pt]=cli.read();pt++;                     // store incoming char 
          t1e+=(micros()-t1d);          
          t1b=micros();
        }
    }
    len=pt;
  }
#endif TXRX_TCP

#ifdef DIAG
    Serial.print(" qAvailable=");Serial.println(qAvailable);        
#endif

    data[len]='\0';
    t2=micros();             // ********** t2 end getHD
    if(len==0){return MESSLEN;}
    return MESSOK;           // valid len >0 
}

int exportData(uint8_t numT)                            // formatting periBuf data in bufServer 
                                                        // sending bufServer to server 
                                                        // recieving server response in bufServer
{
  t2b=micros();
  strcpy(bufServer,"GET /cx?\0");
  if(!buildMess("peri_pass_",srvpswd,"?")==MESSOK){
    Serial.print("decap bufServer ");Serial.print(bufServer);Serial.print(" ");Serial.println(srvpswd);return MESSDEC;};

  char message[LENVAL];
  int sb=0,i=0,k;
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

if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL);}

    uint8_t fonction=0;                                                                       // data_read_ -> set
    char fonctName[]={"data_read_"};
    if(tableC[numT].numPeri!=0){memcpy(fonctName,"data_save_",LENNOM);fonction=1;}            // data_save_ -> ack
    buildMess(fonctName,message,"");                                                          // buld message for server

    t2c=micros()-t2b;

/* send to server */
    int periMess=-1;
    int cnt=0;
    
    while(cnt<2){
      periMess=mess2Server(&cli,host,port,bufServer);                                        // send message to server
      if(periMess!=-7){cnt=2;}else {cnt++;userResetSetup();}}
/*
    if(periMess==MESSOK){
      periMess=-99;
      periMess=getHData();                                             // rx server message
    }
    //cli.stop();
    
#ifdef DIAG                
    Serial.print(" getHData=");Serial.print(staGHD);
    Serial.print(" total=");Serial.print(t2-t0);Serial.print(" assy=");Serial.print(t2c);
    Serial.print(" cx+tx=");Serial.print(t0b-t0);
    Serial.print(" rx=");Serial.print(t2-t1);Serial.print(" rx wait=");Serial.print(t1c);
    Serial.print(" cli.read()=");Serial.print(t1e);Serial.print("uS");
    Serial.print(" periMess=");Serial.println(periMess);
    Serial.println(bufServer); 
#endif
*/
    return periMess;
}


int  importData()                // bufServer reçoit un message du serveur
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
  
  periMess=getHData();
  if(periMess==MESSOK){
        
        packMac((byte*)fromServerMac,(char*)(bufServer+MPOSMAC+LBODY));  // mac from set message (LBODY pour "<body>")
        nP=convStrToNum(bufServer+MPOSNUMPER+LBODY,&len);       // numPer from set message
        numT=nrfp.macSearch(fromServerMac,&numPeri);            // numT mac reg nb in conc table ; numPeri from table numPeri 
                                                                // numPeri should be same as nP (if !=0 && mac found)

        int eds=99;
        if(numT>=NBPERIF){periMess=MESSMAC;}                    // if mac doesnt exist -> error
        else if(numPeri!=0 && numPeri!=nP){periMess=MESSNUMP;}  // if numPeri doesnt match message -> error
        else {eds=nrfp.extDataStore(nP,numT,bufServer+MPOSPERREFR+LBODY,SBLINIT);} // format MMMMM_UUUUU_xxxx MMMMM aw_min value ; UUUUU aw_ok value ; xxxx user dispo 
                                                                              // (_P.PP pitch value)
#ifdef DIAG                
        Serial.print(" nP=");Serial.print(nP);Serial.print(" numT=");Serial.print(numT);Serial.print(" numPeri=");Serial.print(numPeri);Serial.print(" eds=");Serial.print(eds);Serial.print(" fromServerMac=");for(int x=0;x<5;x++){Serial.print(fromServerMac[x]);}//Serial.println();
#endif
  }        
        return periMess;
}

#endif // NRF_MODE == 'C'
