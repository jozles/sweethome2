#include "nrf_user_conc.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;

#if NRF_MODE == 'C'

/* user includes */


#include <Ethernet.h> //bibliothèque W5x00 Ethernet
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"

  //const char* host  = HOSTIPADDR;
  byte        host[]={82, 64, 32, 56};
  //byte host[]={192,168,0,35};                                 // ip server sweethome
  //byte host[]={64, 233, 187, 99};
  //const int   port  = PORTPERISERVER;                         // port server sweethome
  int         port  = 1790;
  byte        mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};   // mac addr ethernet carte W5100
  byte        localIp[] = {192,168,1,30};                     // IP fixe pour carte W5100

  EthernetClient cli;   

/* user fields */

  char model[]={"D32800"};

  char bufServer[LBUFSERVER];

  char* srvpswd=PERIPASS;
  byte  lsrvpswd=LPWD;

  char* fonctions={"set_______ack_______etat______reset_____sleep_____testaoff__testa_on__testboff__testb_on__last_fonc_"};
  uint8_t fset_______,fack_______,fetat______,freset_____,fsleep_____,ftestaoff__,ftesta_on__,ftestboff__,ftestb_on__;;
  int     nbfonct;

  char* chexa="0123456789ABCDEFabcdef\0";

  unsigned long t0,t1,t2;

int  dataTransfer(char* data);


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
  Serial.print(millis()-t_beg);Serial.print(" ");Serial.print(Ethernet.localIP());Serial.print(" ");
}

int mess2Server(EthernetClient* cli,byte* host,int port,char* data)    // connecte au serveur et transfère la data
{
  int             cxStatus=0;
  uint8_t         repeat=0;

  Serial.print("connecting ");    
  for(int i=0;i<4;i++){Serial.print((uint8_t)host[i]);Serial.print(" ");}
  Serial.print(":");Serial.print(port);
  Serial.print("...");
  
  while(!cxStatus && repeat<4){

    Serial.print(repeat);Serial.print("/");
    
    repeat++;

    t0=micros();
    cxStatus=cli->connect(host,port);
    cxStatus=cli->connected();
    if(!cxStatus){
        Serial.print(cxStatus);
        switch(cxStatus){
            case -1:Serial.print(" time out ");break;
            case -2:Serial.print(" invalid server ");break;
            case -3:Serial.print(" truncated ");break;
            case -4:Serial.print(" invalid response ");break;
            default:Serial.print(" unknown reason ");break;
        }
    }
    else {
      
      cli->print(data);
      cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");
      Serial.print(cxStatus);Serial.print(" ok ");Serial.print(micros()-t0);Serial.println("uS");
      return 1;
    }
    delay(100);
  }
  Serial.println(" failed");return 0;
}


int getHData(EthernetClient* cli,char* data,uint16_t* len)
{
  #define TIMEOUT 2000
  
  int pt=0;
  char inch;
  unsigned long timerTo=millis();

  if(*len==0){return -1;}

  t1=micros();

  if(cli->connected()!=0){
Serial.println("connected");    
      while(millis()<(timerTo+TIMEOUT)){
        if(cli->available()>0){
          timerTo=millis();
          inch=cli->read();
#ifdef DIAG          
          Serial.print(inch);
#endif
          if(pt<((*len)-1)){data[pt]=inch;pt++;}
        }
      }
      t2=micros();
      data[pt]='\0';
      *len=pt;
      return 1;               // data ok
  }
  return -2;                  // not connected
}

int exportData(uint8_t numT)
{
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

if(strlen(message)>(LENVAL-4)){Serial.print("******* LENVAL ***** MESSAGE ******");ledblink(BCODELENVAL);}

    uint8_t fonction=0;                                                                       // data_read_ -> set
    char fonctName[]={"data_read_"};
    if(tableC[numT].numPeri!=0){memcpy(fonctName,"data_save_",LENNOM);fonction=1;}            // data_save_ -> ack
    buildMess(fonctName,message,"");                                                          // buld message to server


/* tx/Rx to/from server */
    int periMess=-1;
    int cnt=0;
    while(cnt<2){
      periMess=mess2Server(&cli,host,port,bufServer);                                        // send message to server
      if(periMess!=-7){cnt=2;}else {cnt++;userResetSetup();}}
    
    if(periMess==MESSOK){
      uint16_t len=LBUFSERVER;
      int z=getHData(&cli,bufServer,&len);
      Serial.print(" getHD=");Serial.print(z);Serial.print(" ");Serial.print(len);Serial.println(" ");Serial.print(t2-t0);Serial.print("-");Serial.print(t2-t1);Serial.println("uS");
    }
    cli.stop();
    Serial.print("periMess=");Serial.println(periMess); 
}

/* user functions */

int  dataTransfer(char* data)           // transfert contenu de set ou ack dans variables locales selon contrôles
                                        // data sur fonction
                                        //    contrôle mac addr et numPeriph ;
                                        //    si pb -> numPeriph="00" et ipAddr=0
                                        //    si ok -> tfr params
                                        // retour periMess
{
  int  ddata=16;                        // position du numéro de périphérique  
  int  numT,nP,len,numPer;
  char fromServerMac[6];
  int  periMess;
  
        periMess=MESSOK;
        packMac((byte*)fromServerMac,(char*)(data+ddata+3));
        nP=convStrToNum(data+ddata,&len);
        numT=nrfp.macSearch(fromServerMac,&numPer);
        
        if(numT>=NBPERIF){periMess=MESSMAC;}
        else if(numPer!=0 && numPer!=nP){periMess=MESSNUMP;}
        else {nrfp.extDataStore(nP,numT,data+MPOSPERREFR,16);}            // format MMMMM_UUUUU_xxxx MMMMM aw_min value ; UUUUU aw_ok value ; xxxx user dispo 
                                                                          // (_P.PP pitch value)
        return periMess;
}

#endif // NRF_MODE == 'C'
