#include <Arduino.h>
#include <SPI.h>      //bibliothèqe SPI pour W5100
#include <Ethernet.h> //bibliothèque W5100 Ethernet
//#include <SD.h>
#include "const.h"
//#include <Wire.h>
//#include "utilether.h"
#include "shconst2.h"
#include <shmess2.h>
#include "shutil2.h"
#include "periph.h"

/* >>>>>>>> fichier config <<<<<<< 

extern char configRec[CONFIGRECLEN];
  
extern byte* mac;
extern byte* localIp;
extern int*  portserver;
extern char* nomserver;
extern char* userpass;
extern char* modpass;
extern char* peripass;
extern char* ssid;   
extern char* passssid;
extern int*  nbssid;
extern char* usrnames;  
extern char* usrpass;     
extern long* usrtime;
extern long* usrpretime;
extern char* thermonames;
extern int16_t* thermoperis;
extern uint16_t* toPassword;
extern byte* configBegOfRecord;
extern byte* configEndOfRecord;

*/

/* >>>>>>> fichier périphériques <<<<<<<  */

extern char      periRec[PERIRECLEN];          // 1er buffer de l'enregistrement de périphérique
extern char      periCache[PERIRECLEN*NBPERIF];   // cache des périphériques
extern byte      periCacheStatus[NBPERIF];     // indicateur de validité du cache d'un périph
  
extern int       periCur;                      // Numéro du périphérique courant

extern uint16_t* periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern int32_t*  periPerRefr;                  // ptr ds buffer : période datasave minimale
extern uint16_t* periPerTemp;                  // ptr ds buffer : période de lecture tempèrature
extern float*    periPitch;                    // ptr ds buffer : variation minimale de température pour datasave
extern float*    periLastVal;                  // ptr ds buffer : dernière valeur de température  
extern float*    periAlim;                     // ptr ds buffer : dernière tension d'alimentation
extern char*     periLastDateIn;               // ptr ds buffer : date/heure de dernière réception
extern char*     periLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
extern char*     periLastDateErr;              // ptr ds buffer : date/heure de derniere anomalie com
extern int8_t*   periErr;                      // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
extern char*     periNamer;                    // ptr ds buffer : description périphérique
extern char*     periVers;                     // ptr ds buffer : version logiciel du périphérique
extern char*     periModel;                    // ptr ds buffer : model du périphérique
extern byte*     periMacr;                     // ptr ds buffer : mac address 
extern byte*     periIpAddr;                   // ptr ds buffer : Ip address
extern uint16_t* periPort;                     // ptr ds buffer : port periph server
extern byte*     periSwNb;                     // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*     periSwVal;                    // ptr ds buffer : état/cde des inter  
extern byte*     periInput;                    // ptr ds buffer : Mode fonctionnement inters (1 par switch)           
extern uint32_t* periSwPulseOne;               // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t* periSwPulseTwo;               // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t* periSwPulseCurrOne;           // ptr ds buffer : temps courant pulses ON
extern uint32_t* periSwPulseCurrTwo;           // ptr ds buffer : temps courant pulses OFF
extern byte*     periSwPulseCtl;               // ptr ds buffer : mode pulses
extern byte*     periSwPulseSta;               // ptr ds buffer : état clock pulses
extern uint8_t*  periSondeNb;                  // ptr ds buffer : nbre sonde
extern boolean*  periProg;                     // ptr ds buffer : flag "programmable" (périphériques serveurs)
extern byte*     periDetNb;                    // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*     periDetVal;                   // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern float*    periThOffset;                 // ptr ds buffer : offset correctif sur mesure température
extern float*    periThmin;                    // ptr ds buffer : alarme mini th
extern float*    periThmax;                    // ptr ds buffer : alarme maxi th
extern float*    periVmin;                     // ptr ds buffer : alarme mini volts
extern float*    periVmax;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur
      
extern byte*     periBegOfRecord;
extern byte*     periEndOfRecord;

extern byte      periMacBuf[6]; 

extern byte      lastIpAddr[4];

extern uint32_t  memDetServ;

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)

extern char      bufServer[LBUFSERVER];

extern char*     chexa;

extern int       nbfonct,faccueil,fdatasave,fperiSwVal,fperiDetSs,fdone,fpericur,fperipass,fpassword,fusername,fuserref;


void assySet(char* message,int periCur,char* diag,char* date14)     // assemblage dats pour périphérique ; foemat pp_mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._
{
  sprintf(message,"%02i",periCur);message[2]='\0';periMess=MESSOK;
  strcat(message,"_");

            if(periCur>0){                                          // si periCur >0 macaddr ok -> set
                unpackMac(message+strlen(message),periMacr);}

            else if(periCur<=0){
                  unpackMac(message+strlen(message),periMacBuf);    // si periCur<=0 plus de place -> ack
                  strcat(message,periDiag(periMess));}

            strcat(message,"_");

            memcpy(message+strlen(message),date14,14);
            //Serial.println(date14);
            strcat(message,"_");

            long v1,v2=0;
            if(periCur>0){                                          // periCur>0 tfr params
                v2=*periPerRefr;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerRefr
                strcat(message,"_");

                v2=*periPerTemp;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerTemp
                strcat(message,"_");

                v2=*periPitch*100;
                sprintf((message+strlen(message)),"%04d",v2);       // periPitch
                strcat(message,"_");

                v1=strlen(message);                                 // 4 bits disjoncteurs switchs (8,6,4,2)
                for(int k=MAXSW;k>0;k--){
                    byte a=*periSwVal;
                    message[v1+MAXSW-k]=(char)( 48+ ((a>>((k*2)-1)) &0x01) );
                }      // periSw (cdes)
                memcpy(message+v1+MAXSW,"_\0",2);

                v1+=MAXSW+1;

                for(int k=0;k<NBPULSE*2;k++){                       // 4*2 compteurs (8*(8+1)bytes)
                    sprintf(message+v1+k*(LENVALPULSE+1),"%08u",*(periSwPulseOne+k));
                    memcpy(message+v1+(k+1)*LENVALPULSE+k,"_\0",2);
                }

                v1+=NBPULSE*2*(8+1);                                // bits OTF * 4 = 2*2+1 bytes
                for(int k=0;k<PCTLLEN;k++){conv_htoa(message+v1+k*2,(byte*)(periSwPulseCtl+k));}
                memcpy(message+v1+2*PCTLLEN,"_\0",2);  

                v1+=2*PCTLLEN+1;
                for(int k=0;k<NBPERINPUT*PERINPLEN;k++){           // 24*4=96+1 
                    *(message+v1+2*k)=chexa[*(periInput+k)>>4];
                    *(message+v1+2*k+1)=chexa[*(periInput+k)&0x0F];
                }
                memcpy((message+v1+2*NBPERINPUT*PERINPLEN),"_\0",2);


                v1+=2*NBPERINPUT*PERINPLEN+1;
                byte byt;
                for(int mds=MDSLEN-1;mds>=0;mds--){                   // 32 bits memDetServ -> 8 car hexa
                    byt=(uint8_t)((uint32_t)(memDetServ>>(mds*8)));
                    //Serial.print(" memDetServ shifté(");Serial.print(mds);Serial.print(")");Serial.print((uint32_t)(memDetServ>>(mds*8)),HEX);
                    conv_htoa(message+v1+2*(MDSLEN-mds-1),(byte*)&byt);}//Serial.println();
                memcpy(message+v1+2*MDSLEN,"_\0",2);

                v1+=MDSLEN*2+1;
                v2=*periPort;
                sprintf((message+v1),"%04d",v2);          // periPort
                memcpy(message+v1+4,"_\0",2);

            }  // pericur != 0

            strcat(message,diag);                         // periMess
}


int periReq(EthernetClient* cli,uint16_t np,char* nfonct)                                 // fonction set ou ack vers périphérique
                    // envoie cde GET dans bufServer via messToServer + getHttpResp         (fonction set suite à modif dans periTable)
                    //                            status retour de messToServer ou getHttpResponse ou fonct invalide (doit être done___)
                    // np=periCur à jour (0 ou n) si periCur = 0 message set réduit
                    // format bufServer GET /message...
                    // format message   nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // format datas     NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._    NN numpériph ; mm.mm... mac
{                   

  char message[LENMESS]={'\0'};
  int8_t zz=MESSOK;
  char date14[LNOW];alphaNow(date14);
  char host[16];memset(host,'\0',16);

  periCur=np;periLoad(periCur);
  Serial.print("\nperiReq(peri=");Serial.print(periCur);Serial.print("-port=");Serial.print(*periPort);Serial.println(")");
  
    if(*periProg!=0 && *periPort!=0){charIp(periIpAddr,host);}
  
  if(memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0){
    assySet(message,periCur,periDiag(periMess),date14);}  // assemblage datas 
  *bufServer='\0';

          memcpy(bufServer,"GET /\0",6);                  // commande directe de périphérique en mode serveur
          buildMess(nfonct,message,"");                   // bufServer complété   

          zz=messToServer(cli,host,*periPort,bufServer);
          uint8_t fonct;
          if(zz==MESSOK){
              zz=getHttpResponse(cli,bufServer,LBUFSERVER,&fonct);
              if(zz==MESSOK && fonct!=fdone){zz=MESSFON;}
              delay(1);
              purgeServer(cli);
              }
          if(zz==MESSOK){packDate(periLastDateOut,date14+2);}
          *periErr=zz;
          periSave(periCur,PERISAVESD);                  // modifs de periTable et date effacèe par prochain periLoad si pas save
          cli->stop();        
          return zz;
}

int periAns(EthernetClient* cli,char* nfonct)   // réponse à périphérique cli 
                    // envoie une page html (bufServer encapsulé dans <body>...</body>) (fonction ack suite à réception de datasave - set si dataread)
                    // periCur est à jour (0 ou n) et periMess contient le diag du dataread/save reçu
                    // format bufServer <body>message...</body>
                    // format message   nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // format datas     NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._    NN numpériph ; mm.mm... mac
{                   

  char message[LENMESS]={'\0'};
  int8_t zz=MESSOK;
  char date14[LNOW];alphaNow(date14);

  Serial.print("\nperiAns(peri=");Serial.print(periCur);Serial.println(")");
  if(memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0){
    assySet(message,periCur,periDiag(periMess),date14);}  // assemblage datas 
  *bufServer='\0';
  buildMess(nfonct,message,"");                           // bufServer complété 

            //htmlIntro0(cli);                            // inutile pour le périphérique
            cli->print("<body>");
            cli->print(bufServer);
            cli->println("</body>");
            //cli->println("</html>");                    // inutile pour le périphérique
          
          if(zz==MESSOK){packDate(periLastDateOut,date14+2);}
          *periErr=zz;
          periSave(periCur,PERISAVESD);                   // modifs de periTable et date effacèe par prochain periLoad si pas save
          cli->stop();        
          return zz;
}


/*
void assySet(char* message,int periCur,char* diag,char* date14)
{
#ifndef PERIF

  sprintf(message,"%02i",periCur);message[2]='\0';periMess=MESSOK;
  strcat(message,"_");

            if(periCur>0){                                          // si periCur >0 macaddr ok -> set
                unpackMac(message+strlen(message),periMacr);}

            else if(periCur<=0){
                  unpackMac(message+strlen(message),periMacBuf);    // si periCur>=0 plus de place -> ack
                  strcat(message,periDiag(periMess));}

            strcat(message,"_");

            memcpy(message+strlen(message),date14,14);
            //Serial.println(date14);
            strcat(message,"_");

            long v1,v2=0;
            if(periCur!=0){                                         // periCur!=0 tfr params
                v2=*periPerRefr;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerRefr
                strcat(message,"_");

                v2=*periPerTemp;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerTemp
                strcat(message,"_");

                v2=*periPitch*100;
                sprintf((message+strlen(message)),"%04d",v2);      // periPitch
                strcat(message,"_");

                v1=strlen(message);                                 // 4 bits commande (8,6,4,2)
                for(int k=MAXSW;k>0;k--){
                    byte a=*periSwVal;
                    message[v1+MAXSW-k]=(char)( 48+ ((a>>((k*2)-1)) &0x01) );
                }      // periSw (cdes)
                memcpy(message+v1+MAXSW,"_\0",2);

                v1+=MAXSW+1;

                for(int k=0;k<NBPULSE*2;k++){                  // 2 compteurs/sw (8*(8+1)bytes)
                    sprintf(message+v1+k*(LENVALPULSE+1),"%08u",*(periSwPulseOne+k));
                    memcpy(message+v1+(k+1)*LENVALPULSE+k,"_\0",2);
                }

                v1+=NBPULSE*2*(8+1);                           // bits OTF * 4sw = 2*2+1 bytes
                for(int k=0;k<PCTLLEN;k++){conv_htoa(message+v1+k*2,(byte*)(periSwPulseCtl+k));}
                memcpy(message+v1+2*PCTLLEN,"_\0",2);   // 2*PCTLLEN ne fonctionne pas ????

                v1+=2*PCTLLEN+1;
                for(int k=0;k<NBPERINPUT*PERINPLEN;k++){           // 3*8=24*2=48+1 par sw
                    *(message+v1+2*k)=chexa[*(periInput+k)>>4];
                    *(message+v1+2*k+1)=chexa[*(periInput+k)&0x0F];
                }
                memcpy((message+v1+2*NBPERINPUT*PERINPLEN),"_\0",2);


                v1+=2*NBPERINPUT*PERINPLEN+1;
                for(int mds=0;mds<MDSLEN;mds++){
                    conv_htoa(message+v1+2*(MDSLEN-mds-1),(byte*)(&memDetServ+mds));}
                memcpy(message+v1+2*MDSLEN,"_\0",2);

                v1+=MDSLEN*2+1;
                v2=*periPort;
                sprintf((message+v1),"%04d",v2);     // periPort
                memcpy(message+v1+4,"_\0",2);

            }  // pericur != 0

            strcat(message,diag);                         // periMess
#endif  ndef PERIF
}*/


/*
void periSend(uint16_t np,char* nfonct)    // configure periParamsHtml pour envoyer une commande (set________ / etat______ etc)
                                           // au périph serveur np (avec cb prog cochée)
{                                          // periCur, periLoad rechargés avec np 
  periCur=np;periLoad(periCur);
  if(*periProg!=0 && *periPort!=0){
    checkdate(2);
    char ipaddr[16];memset(ipaddr,'\0',16);
    charIp(periIpAddr,ipaddr);
    periParamsHtml(&cliext,ipaddr,(int)*periPort,nfonct);
  }
}

int periParamsHtml(EthernetClient* cli,char* host,int port,char* nfonct)   // fonction set ou ack vers périphérique
                    // si port=0 envoie une page html (bufServer encapsulé dans <body>...</body>) (fonction ack suite à réception de datasave - set si dataread)
                    // sinon envoie cde GET dans bufServer via messToServer + getHttpResp         (fonction set suite à modif dans periTable)
                    //                            status retour de messToServer ou getHttpResponse ou fonct invalide (doit être done___)
                    // periCur est à jour (0 ou n) et periMess contient le diag du dataread/save reçu
                    // si periCur = 0 message set réduit
                    // format nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // datas NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._    NN numpériph ; mm.mm... mac
{                   

  char message[LENMESS]={'\0'};
  int8_t zz=MESSOK;
  char date14[LNOW];alphaNow(date14);
  
  //if((periCur!=0) && (what==1) && (port==0)){memcpy(nfonct,"ack_______",LENNOM);}    // ack pour datasave (what=1)

  Serial.print("\nperiParamsHtml(peri=");Serial.print(periCur);Serial.print("-port=");Serial.print(port);Serial.println(")");
  checkdate(3);
  if(memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0){
    assySet(message,periCur,periDiag(periMess),date14);}
  checkdate(4);
  *bufServer='\0';

          if(port!=0){memcpy(bufServer,"GET /\0",6);}   // message pour commande directe de périphérique en mode serveur
          buildMess(nfonct,message,"");                 // bufServer complété   

          if(port==0){                                  // réponse à dataRead/Save
            //htmlIntro0(cli);                          // inutile pour le périphérique
            cli->print("<body>");
            cli->print(bufServer);
            cli->println("</body>");
            //cli->println("</html>");                  // inutile pour le périphérique
          }
          else {                                        // envoi vers périphérique en mode serveur
            zz=messToServer(cli,host,port,bufServer);
            uint8_t fonct;
            if(zz==MESSOK){
              zz=getHttpResponse(cli,bufServer,LBUFSERVER,&fonct);
              if(zz==MESSOK && fonct!=fdone){zz=MESSFON;}
              delay(1);
              purgeServer(cli);
            }
          }
          checkdate(5);
          if(zz==MESSOK){packDate(periLastDateOut,date14+2);}
          *periErr=zz;
          periSave(periCur,PERISAVESD);                  // modifs de periTable et date effacèe par prochain periLoad si pas save
          cli->stop();        
          return zz;
}*/
