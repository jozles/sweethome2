#include "nrf24l01s_const.h"
#include "nrf_user_conc.h"
#include "nrf24l01s.h"
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
  Serial.print(millis()-t_beg);Serial.print(" ");Serial.print(Ethernet.localIP());Serial.print(" ");
}

int mess2Server(EthernetClient* cli,byte* host,int port,char* data)    // connecte au serveur et transfère la data
{
  int             cxStatus=0;
  uint8_t         repeat=0;

  t0=micros();
  t0b=0;

#ifdef DIAG  
  Serial.print("tx connecting ");    
  for(int i=0;i<4;i++){Serial.print((uint8_t)host[i]);Serial.print(" ");}
  Serial.print(":");Serial.print(port);
  Serial.print("...");
#endif  

  while(!cxStatus && repeat<4){

#ifdef DIAG  
    Serial.print(repeat);Serial.print("/");
#endif
    
    repeat++;

    cxStatus=cli->connect(host,port);
    cxStatus=cli->connected();
    if(!cxStatus){
#ifdef DIAG        
        Serial.print(cxStatus);
        switch(cxStatus){
            case -1:Serial.print(" time out ");break;
            case -2:Serial.print(" invalid server ");break;
            case -3:Serial.print(" truncated ");break;
            case -4:Serial.print(" invalid response ");break;
            default:Serial.print(" unknown reason ");break;
        }
#endif        
    }
    else {
      
      cli->print(data);
      cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");

#ifdef DIAG
      Serial.println(cxStatus);
#endif
      t0b=micros();
      return 1;
    }
    delay(100);
  }
#ifdef DIAG    
  Serial.println(" failed");return 0;
#endif  
}


int getHData() //EthernetClient* cli,char* data,uint16_t* len)
{
  #define TIMEOUT 1000

  char* data=bufServer;
  uint16_t len=LBUFSERVER;                              // bufServer length
  int qAvailable;                                       // data qty available
  int pt=0;
  char inch;
  unsigned long timerTo=millis();

  t1c=0;
  t1b=micros();
  t1=micros();                                          // ************ t1 beg getHD
  t1e=0;

  if(cli.connected()!=0 ){
    Serial.println(" rx cxd");    
    while(millis()<(timerTo+TIMEOUT)){
        qAvailable=cli.available();
        if(qAvailable>0){
#ifdef DIAG
          Serial.print(" timerTo=");Serial.print(millis()-timerTo);Serial.print(" qAvailable=");Serial.println(qAvailable);        
#endif
          t1c+=(micros()-t1b);
          timerTo=millis();
          t1d=micros();
          inch=cli.read();                             // incoming char from server
          t1e+=(micros()-t1d);

          if(pt<(len-1)){data[pt]=inch;pt++;}        // store incoming char 
          t1b=micros();
        }
        if(pt>(LBODY+MPOSPERREFR+SBLINIT)){break;}      // LBODY pour "<body>"
    }
    t2=micros();                                        // ********** t2 end getHD
    data[pt]='\0';
    len=pt;
    return 1;                 // data ok
  }
  return -2;                  // not connected or no data
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

/* tx/Rx to/from server */
    int periMess=-1;
    int cnt=0;
    int staGHD=-99;
    
    
    while(cnt<2){
      periMess=mess2Server(&cli,host,port,bufServer);                                        // send message to server
      if(periMess!=-7){cnt=2;}else {cnt++;userResetSetup();}}
/*    
 *     en TCP la réception devraitt être disjointe pour libérer le temps d'attente (plus de 250mS)
 *     (à quel moment fermer la connexion ?) 
 *     voir les performances de l'UDP...
 */
    if(periMess==MESSOK){
      staGHD=getHData();                                             // rx server message
    }
    cli.stop();
#ifdef DIAG                
    Serial.print(" getHData=");Serial.print(staGHD);//Serial.print(" l=");Serial.print(len);
    Serial.print(" total=");Serial.print(t2-t0);Serial.print(" tfr=");Serial.print(t2c);
    Serial.print(" cx+tx=");Serial.print(t0b-t0);
    Serial.print(" rx=");Serial.print(t2-t1);Serial.print(" rx wait=");Serial.print(t1c);
    Serial.print(" cli.read()=");Serial.print(t1e);Serial.print("uS");
    Serial.print(" periMess=");Serial.println(periMess);
    Serial.println(bufServer); 
#endif
    return staGHD;
}


int  dataTransfer(char* data)    // bufServer contient un message valide du serveur
                                 // retour MESSOK ok ; MESSNUMP numPeri invalide ; MESSMAC macaddr pas trouvée dans tableC

                                        // transfert contenu de set ou ack dans variables locales selon contrôles
                                        // déclenché par rxServ qui indique que bufServer est valide
                                        //    contrôle mac addr et numPeriph ;
                                        //    si ok -> tfr params
                                        // retour periMess
{
  
  int  numT,nP,len,numPeri;
  char fromServerMac[6];
  int  periMess;
  
        periMess=MESSOK;
        packMac((byte*)fromServerMac,(char*)(data+MPOSMAC+LBODY));  // mac from set message (LBODY pour "<body>")
        nP=convStrToNum(data+MPOSNUMPER+LBODY,&len);            // numPer from set message
        numT=nrfp.macSearch(fromServerMac,&numPeri);            // numT reg nb in conc table ; numPeri from table numPeri 
                                                                // numPeri should be same as nP (if mac found)

        int eds=99;
        if(numT>=NBPERIF){periMess=MESSMAC;}
        else if(numPeri!=0 && numPeri!=nP){periMess=MESSNUMP;}  
        else {eds=nrfp.extDataStore(nP,numT,data+MPOSPERREFR+LBODY,SBLINIT);} // format MMMMM_UUUUU_xxxx MMMMM aw_min value ; UUUUU aw_ok value ; xxxx user dispo 
                                                                              // (_P.PP pitch value)
#ifdef DIAG                
        Serial.print(" nP=");Serial.print(nP);Serial.print(" numT=");Serial.print(numT);Serial.print(" numPeri=");Serial.print(numPeri);Serial.print(" eds=");Serial.print(eds);Serial.print(" fromServerMac=");for(int x=0;x<5;x++){Serial.print(fromServerMac[x]);}//Serial.println();
#endif
        
        return periMess;
}

#endif // NRF_MODE == 'C'
