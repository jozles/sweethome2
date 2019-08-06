#include "nrf_user_conc.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern struct NrfConTable tableC[NBPERIF];
extern Nrfp nrfp;

#if NRF_MODE == 'C'

/* user includes */

#include <Ethernet.h> //bibliothèque W5100 Ethernet
#include "shconst2.h"
#include "shutil2.h"
#include "shmess2.h"

//  const char* host  = HOSTIPADDR;
  //byte host[]={192,168,0,35};
  byte host[]={64, 233, 187, 99};
  const int   port  = PORTPERISERVER;
  byte        mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};               // mac addr ethernet carte W5100
  byte        localIp[] = {192,168,0,30};      // adresse server

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

delay(3000);
for(int i=0;i<6;i++){Serial.print(mac[i],HEX);Serial.print(" ");}Serial.print(" ");for(int i=0;i<4;i++){Serial.print(localIp[i]);Serial.print(" ");}
Ethernet.begin(mac,(uint8_t*)localIp);
Serial.print(" -- ");
    
/*    if(Ethernet.begin((uint8_t*)mac) == 0)
    {Serial.print("Failed with DHCP... forcing Ip ");serialPrintIp(localIp);Serial.println();delay(100);
    Ethernet.begin ((uint8_t*)mac, localIp); //initialisation de la communication Ethernet
    }*/
  Serial.println(Ethernet.localIP());


Serial.println("connecting...");
while(1){
  if (cli.connect(host, 80)) {
    Serial.println("connected");
    cli.println("GET /search?q=arduino HTTP/1.0");
    cli.println();

    while(1){
      if(cli.available()) {
        char c = cli.read();
        Serial.print(c);
      }

      if (!cli.connected()) {
        Serial.println();
        Serial.println("disconnecting.");
        cli.stop();
        while(1){}  
      }
    }

    
  } else {
    Serial.println("connection failed");
    delay(1000);  
  }
}  
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
      memcpy(message+2,"_\0",2);                                    //                        - 3
      sb=3;
      unpackMac((char*)(message+sb),tableC[numT].periMac);          // macaddr                    - 18
      sb+=17;
      memcpy(message+sb,"_\0",2);
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+7+LENVERSION+8,6);     // temp                              
      sb+=6;
      memcpy(message+sb,"__\0",3);                                  // âge 
      sb+=2;
      memcpy(message+sb,tableC[numT].periBuf+7+LENVERSION+8+6,4);   // volts                              
      sb+=4;
      memcpy(message+sb,"_\0",2);                             
      sb+=1;
      memcpy(message+sb,tableC[numT].periBuf+6,LENVERSION-1);       // VERSION
      sb+=LENVERSION-1;
      memcpy(message+sb,tableC[numT].periBuf+6+LENVERSION+8,1);     // modele DS18x20
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
    char fonctName[]={"data__read_"};
    if(tableC[numT].numPeri!=0){memcpy(fonctName,"data_save_",8);fonction=1;}                 // data_save_ -> ack
    buildMess(fonctName,message,"");                                                          // buld message to server
    int periMess=-1;
//    periMess=messToServer(&cli,host,port,bufServer);                                          // send message to server

    if(periMess==MESSOK){periMess=getHttpResponse(&cli,bufServer,LBUFSERVER,&fonction);}      // get response
    if(periMess==MESSOK){dataTransfer(bufServer);}                                            
    if(periMess!=MESSOK){nrfp.extDataStore(0,numT,bufServer+MPOSPERREFR,0);}                  // re-init server buffer if received data ko

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
