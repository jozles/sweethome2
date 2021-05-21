#include <Arduino.h>
#include <SPI.h>      //bibliothèqe SPI pour W5100
#include <Ethernet.h> //bibliothèque W5100 Ethernet
#include <EthernetUdp.h>
#include "ds3231.h"
#include "const.h"
#include <shconst2.h>
#include <shmess2.h>
#include "shutil2.h"
#include "periph.h"
#include "peritalk.h"

extern Ds3231 ds3231;

extern EthernetUDP Udp;

extern uint16_t remote_Port;
extern uint8_t remote_IP_cur[4];                   

extern char ab;

/* >>>>>>> fichier périphériques <<<<<<<  */

extern char      periRec[PERIRECLEN];          // 1er buffer de l'enregistrement de périphérique
extern char      periCache[PERIRECLEN*NBPERIF];   // cache des périphériques
extern bool      periCacheStatus[NBPERIF];     // indicateur de validité du cache d'un périph
  
extern uint16_t  periCur;                      // Numéro du périphérique courant

extern uint16_t* periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern int32_t*  periPerRefr;                  // ptr ds buffer : période datasave minimale
extern uint16_t* periPerTemp;                  // ptr ds buffer : période de lecture tempèrature
extern int16_t*  periPitch_;                    // ptr ds buffer : variation minimale de température pour datasave
extern int16_t*  periLastVal_;                  // ptr ds buffer : dernière valeur de température  
extern int16_t*  periAlim_;                     // ptr ds buffer : dernière tension d'alimentation
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
extern int16_t*  periThOffset_;                 // ptr ds buffer : offset correctif sur mesure température
extern int16_t*  periThmin_;                    // ptr ds buffer : alarme mini th
extern int16_t*  periThmax_;                    // ptr ds buffer : alarme maxi th
extern int16_t*  periVmin_;                     // ptr ds buffer : alarme mini volts
extern int16_t*  periVmax_;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*     periProtocol;                 // ptr ds buffer : protocole ('T'CP/'U'DP)
extern uint16_t* periAnal;                     // ptr ds buffer : analog value
extern uint16_t* periAnalLow;                  // ptr ds buffer : low analog value 
extern uint16_t* periAnalHigh;                 // ptr ds buffer : high analog value 
extern uint16_t* periAnalOffset1;              // ptr ds buffer : offset on adc value
extern float*    periAnalFactor;               // ptr ds buffer : factor to float for analog value
extern float*    periAnalOffset2;              // ptr ds buffer : offset on float value

      
extern byte*     periBegOfRecord;
extern byte*     periEndOfRecord;

extern byte      periMacBuf[6]; 

extern byte      lastIpAddr[4];

extern uint32_t  memDetServ;

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)

extern char      bufServer[LBUFSERVER];

extern char*     chexa;

extern  char*    fonctions;
extern int       nbfonct,faccueil,fdatasave,fperiSwVal,fperiDetSs,fdone,fpericur,fperipass,fpassword,fusername,fuserref;


void assySet(char* message,int periCur,const char* diag,char* date14)     
// assemblage datas pour périphérique ; format pp_mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._
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

            unsigned int v1,v2=0;
            if(periCur>0){                                          // periCur>0 tfr params
                v2=*periPerRefr;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerRefr
                strcat(message,"_");

                v2=*periPerTemp;
                sprintf((message+strlen(message)),"%05d",v2);       // periPerTemp
                strcat(message,"_");

                v2=*periPitch_;
                sprintf((message+strlen(message)),"%04d",v2);       // periPitch (100x)
                strcat(message,"_");

                v1=strlen(message);                                 // 4 bits disjoncteurs switchs (8,6,4,2)
                for(int k=MAXSW;k>0;k--){
                    message[v1+MAXSW-k]=(char)(PMFNCVAL + periSwCde(k-1));}
                memcpy(message+v1+MAXSW,"_\0",2);

                v1+=MAXSW+1;
                //dumpstr((char*)periAnalLow,2);
                //dumpstr((char*)periAnalHigh,2);
                for(int k=0;k<2;k++){conv_htoa(&message[v1+k*2],(byte*)((byte*)periAnalLow+k));}   // analog Low  (2+1) bytes
                v1+=4;
                for(int k=0;k<2;k++){conv_htoa(&message[v1+k*2],(byte*)((byte*)periAnalHigh+k));}  // analog High (2+1) bytes
                v1+=4;
                memcpy(message+v1,"_\0",2);
                v1+=1;
          
if(*periProg!=0){

                for(int k=0;k<NBPULSE*2;k++){                      // 4*2 compteurs (8*(8+1)bytes)
                    sprintf(message+v1+k*(LENVALPULSE+1),"%08lu",*(periSwPulseOne+k));
                    memcpy(message+v1+(k+1)*LENVALPULSE+k,"_\0",2);
                }

                v1+=NBPULSE*2*(8+1);                               // bits OTF * 4 = 2*2+1 bytes
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
                for(int mds=MDSLEN-1;mds>=0;mds--){               // 32 bits memDetServ -> 8 car hexa
                    byt=(uint8_t)((uint32_t)(memDetServ>>(mds*8)));
                    //Serial.print(" memDetServ shifté(");Serial.print(mds);Serial.print(")");Serial.print((uint32_t)(memDetServ>>(mds*8)),HEX);
                    conv_htoa(message+v1+2*(MDSLEN-mds-1),(byte*)&byt);}//Serial.println();
                memcpy(message+v1+2*MDSLEN,"_\0",2);

                v1+=MDSLEN*2+1;
                v2=*periPort;
                sprintf((message+v1),"%04d",v2);                  // periPort
                memcpy(message+v1+4,"_\0",2);

            }  // periprog != 0
            strcat(message,diag);                                 // periMess length=LPERIMESS
  }  // pericur != 0            
}



int  periReq(EthernetClient* cli,uint16_t np,const char* nfonct)
{
    periCur=np;periLoad(periCur);
    return periReq0(cli,nfonct,"");
}

int periReq(EthernetClient* cli,uint16_t np,const char* nfonct,const char* msg)     // fonction set ou ack vers périphérique
{
    periCur=np;periLoad(periCur);
    return periReq0(cli,nfonct,msg);
}

int periReq0(EthernetClient* cli,const char* nfonct,const char* msg)                // fonction set ou ack vers périphérique ; periCur/periload() ok
                    // envoie cde GET dans bufServer via messToServer + getHttpResp         (fonction set suite à modif dans periTable)
                    //                            status retour de messToServer ou getHttpResponse ou fonct invalide (doit être done___)
                    // np=periCur à jour (0 ou n) si periCur = 0 message set réduit
                    // format bufServer GET /message...
                    // format message   nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // si set ou ack :
                    //    format datas     NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._   NN numpériph ; mm.mm... mac
{                   // sinon chaine libre issue de msg
  Serial.print(millis());Serial.print(" periReq(");Serial.print((char)*periProtocol);Serial.print(" peri=");Serial.print(periCur);
  Serial.print(" port=");Serial.print(*periPort);Serial.print(") ");

  unsigned long dur=millis();
  char message[LENMESS]={'\0'};
  char date14[LNOW];ds3231.alphaNow(date14);
  char host[16];memset(host,'\0',16);

  if(*periProg!=0 && *periPort!=0){charIp(periIpAddr,host);}
  
  if(memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0){
        assySet(message,periCur,periDiag(periMess),date14);}  // assemblage datas 
  else if(strlen(msg)<LENMESS){strcat(message,msg);}

  *bufServer='\0';
  memcpy(bufServer,"GET /\0",6);                  // commande directe de périphérique en mode serveur
  buildMess(nfonct,message,"",NODIAGS);                   // bufServer complété   
  Serial.println(millis());

  if(*periProtocol=='T'){                         // UDP à développer
          periMess=messToServer(cli,host,*periPort,bufServer);
          //Serial.print("(");Serial.print(MESSOK);Serial.print(" si ok) periMess(messToServer)=");Serial.println(periMess);
          Serial.print(periMess);Serial.print("-");Serial.print(millis());Serial.print(" ");
          uint8_t fonct;
          if(periMess==MESSOK){
              trigwd();
              periMess=getHttpResponse(cli,bufServer,LBUFSERVER,&fonct,DIAGS);
              //Serial.print("(");Serial.print(MESSOK);Serial.print(" si ok) periMess(gHttpR)=");Serial.println(periMess);
              if(periMess==MESSOK){
                if(fonct>=nbfonct){fonct=nbfonct;periMess=MESSFON;}
                else {
                  Serial.println(bufServer);
                  Serial.print(periMess);Serial.print("-");Serial.print(millis());Serial.print(" ");
                  if(fonct==fdatasave){periDataRead(bufServer+LENNOM+1);}
                }
                char ff[LENNOM+1];ff[LENNOM]='\0';memcpy(ff,bufServer,LENNOM);Serial.print(ff);
              }
              purgeServer(cli,NODIAGS);
          }
          if(periMess==MESSOK){packDate(periLastDateOut,date14+2);}
          *periErr=periMess;
  }
  Serial.print(millis());
  cli->stop();               // cliext
  int8_t zz=periSave(periCur,PERISAVELOCAL);          // modifs de periTable et date effacèe par prochain periLoad si pas save
  if(periMess==MESSOK){periMess=zz;if(zz!=MESSOK){Serial.print(" periSave ko");}}
  Serial.print(" (");Serial.print(MESSOK);Serial.print(" si ok) periMess(periReq)=");Serial.print(periMess);Serial.print(" dur=");Serial.println(millis()-dur);
  return periMess;
}

int periAns(EthernetClient* cli,const char* nfonct)   // réponse à périphérique cli ... ou udp(remote_IP,remote_Port)
                    // envoie une page html (bufServer encapsulé dans <body>...</body>) (fonction ack suite à réception de datasave - set si dataread)
                    // periCur est à jour (0 ou n) et periMess contient le diag du dataread/save reçu
                    // format bufServer <body>message...</body>
                    // format message   nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // format datas     NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._    NN numpériph ; mm.mm... mac
{                   

  char message[LENMESS]={'\0'};
  char date14[LNOW];ds3231.alphaNow(date14);

  Serial.print("periAns(");Serial.print(periCur);Serial.print(") ");Serial.print((char)*periProtocol);
  Serial.print(" ");serialPrintIp(periIpAddr);Serial.print("/");Serial.print(*periPort);     
  if(memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0){
    assySet(message,periCur,periDiag(periMess),date14);
    }  // assemblage datas 

  memcpy(bufServer,"<body>\0",7);
  //Serial.print("\n a0=");Serial.println(millis());
  buildMess(nfonct,message,"\0");                           // bufServer complété 
  strcat(bufServer,"</body>");
          if(*periProtocol=='T'){
            cli->write(bufServer);
            //Serial.print(" a1=");Serial.println(millis());
            //cli->stop();
            //Serial.print(" a2=");Serial.println(millis());
          }
          if(*periProtocol=='U'){
            IPAddress udpAddress;
            memcpy((char*)(&udpAddress)+4,periIpAddr,4);
            Udp.beginPacket(udpAddress,*periPort);
            /*Serial.print("sending (");Serial.print(strlen(bufServer));Serial.print(")>");Serial.print(bufServer);
            Serial.print("< to ");serialPrintIp(periIpAddr);Serial.print(":");Serial.println(*periPort);*/
            Udp.write(bufServer,strlen(bufServer));
            Udp.endPacket();
          }
          packDate(periLastDateOut,date14+2);
          *periErr=MESSOK;                                  // assySet, buildMess, envoi ne génèrent pas d'erreur
          return periSave(periCur,PERISAVELOCAL);           // modifs de periTable et date effacèe par prochain periLoad si pas save
}


void checkdate(uint8_t num)                               // détection des dates invalides (en général défaut de format des messages)
{
  if(periLastDateIn[0]==0x66){Serial.print("===>>> date ");Serial.print(num);Serial.print(" HS ");
    char dateascii[12];
    unpackDate(dateascii,periLastDateIn);for(uint8_t j=0;j<12;j++){Serial.print(dateascii[j]);if(j==5){Serial.print(" ");}}Serial.println();
  }
}

void periDataRead(char* valf)   // traitement d'une chaine "dataSave" ou "dataRead" en provenance d'un periphérique 
                                // periInitVar() a été effectué
                                // controle len,CRC, charge periCur (N° périf du message), effectue periLoad()
                                // gère les différentes situations présence/absence/création de l'entrée dans la table des périf
                                // transfère adr mac du message reçu (periMacBuf) vers periRec (periMacr)                                
                                // retour periMess=MESSOK -> periCur valide, periLoad effectué, tfr données dataRead effectué
                                //        periMess=MESSFULL -> plus de place libre
                                //        periMess autres valeurs retour de checkData
{
  int i=0;
  char* k;
  int perizer=0;
  int messLen=strlen(valf)-2;   // longueur hors crc
//Serial.print("messLen=");Serial.print(messLen);Serial.print(" i=");Serial.print(i);Serial.print(" valf=");Serial.println((char*)valf);
  periCur=0;
                        // check len,crc
  periMess=checkData(valf);if(periMess!=MESSOK){periInitVar();return;}         
  
                        // len,crc OK
  valf+=5;conv_atob(valf,&periCur);packMac(periMacBuf,valf+3);                       

  if(periCur!=0){                                                 // si le périph a un numéro, ctle de l'adr mac
    periLoad(periCur);
    if(memcmp(periMacBuf,periMacr,6)!=0){periCur=0;}}
    
  if(periCur==0){                                                 // si periCur=0 recherche si mac connu   
    for(i=1;i<=NBPERIF;i++){                                      // et, au cas où, une place libre
      periLoad(i);
      if(compMac(periMacBuf,periMacr)){
        periCur=i;i=NBPERIF+1;
        //Serial.println(" DataRead/Save Mac connu");
      }                                                                         // mac trouvé
      if((memcmp("\0\0\0\0\0\0",periMacr,6)==0 || memcmp("      ",periMacr,6)==0) && perizer==0){
        perizer=i;
        //Serial.println(" DataRead/Save place libre");
      }        // place libre trouvée
    }
  }
    
  if(periCur==0 && perizer!=0){                                   // si pas connu utilisation N° perif libre "perizer"
      Serial.println(" DataRead/Save Mac inconnu");
      periInitVar();periCur=perizer;periLoad(periCur);            // ???????????????????? pourquoi periLoad ???????????????????????
      periMess=MESSFULL;                                          // ???????????????????? devrait être MESSOK et MESSFULL avant le test ????????????????????
  }

#define PNP 2+1+17+1                                                // (4 length +1) + 2 N° peri +1 + 17 Mac +1 message court de présence
  k=valf+PNP;
  if(periCur!=0){                                                   // si ni trouvé, ni place libre, periCur=0 
    memcpy(periMacr,periMacBuf,6);
    char date14[LNOW];ds3231.alphaNow(date14);checkdate(0);packDate(periLastDateIn,date14+2);checkdate(1);            // maj dates

    messLen-=(PNP+5);if(messLen>0){                                 // PNP + (4 length +1) longueur hors CRC si message terminé
    *periLastVal_=(int16_t)(convStrToNum(k,&i)*100);   // température si save
//Serial.print("messLen=");Serial.print(messLen);Serial.print(" i=");Serial.print(i);Serial.print(" k=");Serial.println((char*)k);      
#if PNP != HISTOPOSTEMP-HISTOPOSNUMPER
      cancelCompil();
#endif //
    }
    messLen-=i;if(messLen>0){
      k+=i;*periAnal=convStrToInt(k,&i);                            // analog value
    }
    messLen-=i;if(messLen>0){
      k+=i;*periAlim_=(int16_t)(convStrToNum(k,&i)*100);            // alim
    }
    messLen-=i;if(messLen>0){    
      k+=i;strncpy(periVers,k,LENVERSION);                          // version
      i=strchr(k,'_')-k+1;                                          // ????? LENVERSION variable ?
    }
//Serial.print("messLen=");Serial.print(messLen);Serial.print(" i=");Serial.print(i);Serial.print(" k=");Serial.println((char*)k);
    messLen-=i;if(messLen>0){      
      k+=i;
      k+=1;for(int i=MAXSW-1;i>=0;i--){periSwLevUpdate(i,*(k+MAXSW-i-1)-PMFNCVAL);}                                   // periSwVal états sw
      k+=MAXSW+1; *periDetNb=(uint8_t)(*k-48);                                                                        // nbre detec
      k+=1; *periDetVal=0;for(int i=MAXDET-1;i>=0;i--){*periDetVal |= ((*(k+i)-48)&DETBITLH_VB )<< 2*(MAXDET-1-i);}   // détecteurs
    }
    /* les pulses ne sont pas transmis si ils sont à 0 ; periSwPulseSta devrait être mis à 0 */
    messLen-=(1+MAXSW+1+1+MAXDET+1);if(messLen>0){
      k+=MAXDET+1;for(int i=0;i<NBPULSE;i++){periSwPulseSta[i]=(uint8_t)(strchr(chexa,(int)*(k+i))-chexa);}           // pulse clk status 
    }
    messLen-=(NBPULSE+1);if(messLen>0){
      k+=NBPULSE+1;for(int i=0;i<LENMODEL;i++){periModel[i]=*(k+i);periNamer[i]=*(k+i);}                              // model
    }
    messLen-=(LENMODEL+1);if(messLen>0){
      k+=LENMODEL+1;
      for(uint16_t i=0;i<2*NBPULSE*sizeof(uint32_t);i++){conv_atoh(k+2*i,(byte*)periSwPulseCurrOne+i);}                     // valeur courante pulses
    }
/*    messLen-=(2*i+1);if(messLen>0){
Serial.print("messLen=");Serial.print(messLen);Serial.print(" i=");Serial.print(i);Serial.print(" k=");Serial.println((char*)k);      
    }*/
#ifdef SHDIAGS    
    periPrint(periCur);Serial.print("periDataRead =");
#endif //    
    
  }
  if(ab=='u'){*periProtocol='U';}else *periProtocol='T';                                                         // last access protocol type
  memcpy(periIpAddr,remote_IP_cur,4);                                                                            // Ip addr
  if(remote_Port!=0 && *periProtocol=='U'){*periPort=remote_Port;}                                               // port
  periSave(periCur,PERISAVELOCAL);
  checkdate(6);
}
