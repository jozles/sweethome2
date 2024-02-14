#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "ds3231.h"
#include "const.h"
#include <shconst2.h>
#include <shmess2.h>
#include "shutil2.h"
#include "periph.h"
#include "peritalk.h"
#include "utilether.h"

extern Ds3231 ds3231;

//extern EthernetUDP* udp[2];

extern uint16_t remote_Port_Udp;
extern uint8_t remote_IP_cur[4];                   

extern char ab;

/* >>>>>>> fichier périphériques <<<<<<<  */

extern char      periRec[PERIRECLEN];           // 1er buffer de l'enregistrement de périphérique
extern char      periCache[PERIRECLEN*NBPERIF]; // cache des périphériques
extern bool      periCacheStatus[NBPERIF];      // indicateur de validité du cache d'un périph
  
extern uint16_t  periCur;                       // Numéro du périphérique courant

extern uint16_t* periNum;                       // ptr ds buffer : Numéro du périphérique courant
extern int32_t*  periPerRefr;                   // ptr ds buffer : période datasave minimale
extern uint16_t* periPerTemp;                   // ptr ds buffer : période de lecture tempèrature
extern int16_t*  periPitch_;                    // ptr ds buffer : variation minimale de température pour datasave
extern int16_t*  periLastVal_;                  // ptr ds buffer : dernière valeur de température  
extern int16_t*  periAlim_;                     // ptr ds buffer : dernière tension d'alimentation
extern char*     periLastDateIn;                // ptr ds buffer : date/heure de dernière réception
extern char*     periLastDateOut;               // ptr ds buffer : date/heure de dernier envoi  
extern char*     periLastDateErr;               // ptr ds buffer : date/heure de derniere anomalie com
extern int8_t*   periErr;                       // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
extern char*     periNamer;                     // ptr ds buffer : description périphérique
extern char*     periVers;                      // ptr ds buffer : version logiciel du périphérique
extern char*     periModel;                     // ptr ds buffer : model du périphérique
extern byte*     periMacr;                      // ptr ds buffer : mac address 
extern byte*     periIpAddr;                    // ptr ds buffer : Ip address
extern uint16_t* periPort;                      // ptr ds buffer : port periph server
extern byte*     periSwNb;                      // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*     periSwCde;                     // ptr ds buffer : état/cde des switchs  
extern byte*     periInput;                     // ptr ds buffer : Mode fonctionnement inters (1 par switch)           
extern uint32_t* periSwPulseOne;                // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t* periSwPulseTwo;                // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t* periSwPulseCurrOne;            // ptr ds buffer : temps courant pulses ON
extern uint32_t* periSwPulseCurrTwo;            // ptr ds buffer : temps courant pulses OFF
extern byte*     periSwPulseCtl;                // ptr ds buffer : mode pulses
extern byte*     periSwPulseSta;                // ptr ds buffer : état clock pulses
extern uint8_t*  periSwSta;                     // ptr ds buffer : état des switchs
extern uint8_t*  periCfg;                       // ptr ds buffer : flag "programmable" (périphériques serveurs)
extern byte*     periDetNb;                     // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*     periDetVal;                    // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern int16_t*  periThOffset_;                 // ptr ds buffer : offset correctif sur mesure température
extern int16_t*  periThmin_;                    // ptr ds buffer : alarme mini th
extern int16_t*  periThmax_;                    // ptr ds buffer : alarme maxi th
extern int16_t*  periVmin_;                     // ptr ds buffer : alarme mini volts
extern int16_t*  periVmax_;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                 // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*     periProtocol;                  // ptr ds buffer : protocole ('T'CP/'U'DP)
extern uint8_t*  periUdpPortNb;                 // ptr ds buffer : n° instance udp utilisée par le périf
extern uint8_t   udpNb;                         // n° instance udp courante
extern uint16_t* periAnal;                      // ptr ds buffer : analog value
extern uint16_t* periAnalLow;                   // ptr ds buffer : low analog value 
extern uint16_t* periAnalHigh;                  // ptr ds buffer : high analog value 
extern uint16_t* periAnalOffset1;               // ptr ds buffer : offset on adc value
extern float*    periAnalFactor;                // ptr ds buffer : factor to float for analog value
extern float*    periAnalOffset2;               // ptr ds buffer : offset on float value
extern uint16_t* periAnalOut;                   // ptr ds buffer :
extern byte*     periSsidNb;                    // ptr ds buffer : 
      
extern byte*     periBegOfRecord;
extern byte*     periEndOfRecord;

extern byte      periMacBuf[6]; 

//extern byte      lastIpAddr[4];

extern uint8_t   memDetServ[];

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)

extern char      bufServer[LBUFSERVER];

extern char*     chexa;

extern  char*    fonctions;
extern int       nbfonct,faccueil,fdatasave,fdatana,fperiSwVal,fperiDetSs,fdone,fpericur,fperipass,fpassword,fusername,fuserref;


void assySet(char* message,int periCur,const char* diag,char* date14,const char* fonct)     
// assemblage datas pour périphérique ; format pp_mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._
{
  Serial.print(" assySet ");Serial.print(fonct);Serial.print(' ');
  
  sprintf(message,"%02i",periCur);message[2]='\0';periMess=MESSOK;
  strcat(message,"_");

  if(periCur>0){                                          // si periCur >0 macaddr ok -> set
    unpackMac(message+strlen(message),periMacr);}

  else if(periCur<=0){
    unpackMac(message+strlen(message),periMacBuf);        // si periCur<=0 plus de place -> ack
    strcat(message,periDiag(periMess));}

    strcat(message,"_");

    memcpy(message+strlen(message),date14,14);
    strcat(message,"_");

    unsigned int v1,v2=0;
    if(periCur>0){                                        // periCur>0 tfr params
                v2=*periPerRefr;
                sprintf((message+strlen(message)),"%05d",v2);     // periPerRefr
                strcat(message,"_");

                v2=*periPerTemp;
                sprintf((message+strlen(message)),"%05d",v2);     // periPerTemp
                strcat(message,"_");

                v2=*periPitch_;
                sprintf((message+strlen(message)),"%04d",v2);     // periPitch (100x)
                strcat(message,"_");

                v1=strlen(message);                               // 4 bits disjoncteurs switchs (8,6,4,2)
                for(int k=MAXSW;k>0;k--){
                    uint8_t swCd=periSwRead(k-1);
                    if(getMotherRemoteStatus(periCur,k-1)==0){swCd=0;}
                    message[v1+MAXSW-k]=(char)(PMFNCVAL + swCd);}
                memcpy(message+v1+MAXSW,"_\0",2);

                v1+=MAXSW+1;
                //dumpstr((char*)periAnalLow,2);
                //dumpstr((char*)periAnalHigh,2);
                for(int k=0;k<2;k++){conv_htoa(&message[v1+k*2],(byte*)((byte*)periAnalLow+k));}   // mini analog 2 bytes
                v1+=4;
                for(int k=0;k<2;k++){conv_htoa(&message[v1+k*2],(byte*)((byte*)periAnalHigh+k));}  // maxi analog 2 bytes
                v1+=4;
                memcpy(message+v1,"_\0",2);         
                v1+=1;

      if(((*periCfg)&PERI_SERV)!=0){

          if(fonct!=nullptr && memcmp(fonct,"sw",2)!=0 && memcmp(fonct,"mds_______",LENNOM)!=0)
          {
                for(int k=0;k<NBPULSE*2;k++){                     // 2 fois 4 compteurs (8*(8+1)bytes) =72
                    sprintf(message+v1+k*(LENVALPULSE+1),"%08lu",*(periSwPulseOne+k));
                    memcpy(message+v1+(k+1)*LENVALPULSE+k,"_\0",2);
                }

                v1+=NBPULSE*2*(8+1);                              // bits OTF * 4 = 2*2+1 bytes =5
                for(int k=0;k<PCTLLEN;k++){conv_htoa(message+v1+k*2,(byte*)(periSwPulseCtl+k));}
                memcpy(message+v1+2*PCTLLEN,"_\0",2);  

                v1+=2*PCTLLEN+1;
                for(int k=0;k<NBPERRULES*PERINPLEN;k++){          // 48*4=192+1 =193
                    *(message+v1+2*k)=chexa[*(periInput+k)>>4];
                    *(message+v1+2*k+1)=chexa[*(periInput+k)&0x0F];
                }
                memcpy((message+v1+2*NBPERRULES*PERINPLEN),"_\0",2);

                v1+=2*NBPERRULES*PERINPLEN+1;
          } // pas swx ni mds_______

                byte byt;
                for(uint8_t mds=MDSLEN;mds>0;mds--){              // NBDSRV bits memDetServ -> MDSLEN car hexa
                    byt=memDetServ[mds-1];
                    conv_htoa(message+v1+2*(MDSLEN-mds),(byte*)&byt);}
                memcpy(message+v1+2*MDSLEN,"_\0",2);
                v1+=MDSLEN*2+1;

                v2=*periPort;
                sprintf((message+v1),"%04d",v2);                  // periPort
                v1+=4;*(message+v1)='_';
                v1++;
      }  // periCfg != 0
      /* ajouter de nouveaux champs ici, leur position est indexée sur la fin du message 
         (par exemple periCfg est à l'adresse message+longueur-2  (-3 pour le'_')
         et le suivant sera à l'adresse message-longueur-3-longueur du champ ajouté ;
         voir exemple dans (single_nrf_v2/sweet_home/nrf_user_conc.cpp-importData() )
      */

      for(int k=0;k<2;k++){conv_htoa(&message[v1+k*2],(byte*)((byte*)periAnalOut+1-k));}   // consigne analog 2 bytes msb first
      memcpy(message+v1+4,"_\0",2);
      v1+=4+1;

      unpack((char*)periCfg,message+v1,1);                        // periCfg
      memcpy(message+v1+2,"\0",2);
      strcat(message,diag);                                 // periMess length=LPERIMESS
    }  // pericur != 0            
}

void assySet(char* message,int periCur,const char* diag,char* date14)
{
  assySet(message,periCur,diag,date14,nullptr);
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

int periReq0(EthernetClient* cli,const char* nfonct,const char* msg)   // fonction set ou ack vers périphérique ; periCur/periload() ok
                    // envoie cde GET dans bufServer via messToServer + getHttpResp         (fonction set suite à modif dans periTable)
                    //                            status retour de messToServer ou getHttpResponse ou fonct invalide (doit être done___)
                    // np=periCur à jour (0 ou n) si periCur = 0 message set réduit
                    // format bufServer GET /message... (complété ici avec "\n\n" : fin de message pour le périf)
                    // format message   nomfonction_=nnnndatas...cc                nnnn len mess ; cc crc
                    // si set ou ack :
                    //    format datas     NN_mm.mm.mm.mm.mm.mm_AAMMJJHHMMSS_nn..._   NN numpériph ; mm.mm... mac
                    // sinon chaine libre issue de msg
                    //
                    // timings :
                    //        début periReq à messToServer   5m.0S
                    //        messToServer                 <170.0mS (connexion ok + cli.write())
                    //        server.available() sur perif   4.5mS (période boucle de test 5mS...)
                    //        réception waitRefCli           7.5mS 
                    //        réception dataSave+traitt      1.8mS
                    //    total                             89.0mS    
{
#ifdef ANALYZE
  PERIQ
#endif // ANALYZE

  unsigned long dur=millis();
  char message[LENMESS]={'\0'};
  char date14[LNOW];ds3231.alphaNow(date14);
  char host[16];memset(host,'\0',16);

  //periPrint(periCur);

  int ret=MESSCX; // pas de port pas de connexion
  if(((*periCfg)&PERI_SERV)!=0 && *periPort!=0){
    charIp(host,(char*)periIpAddr);
  
    Serial.print(" periReq(");Serial.print((char)*periProtocol);
    Serial.print(" peri=");Serial.print(periCur);
    Serial.print(" Ip=");serialPrintIp(periIpAddr);
    Serial.print(" port=");Serial.print(*periPort);Serial.print(") ");

    if(memcmp(nfonct,"mds_______",LENNOM)==0 || memcmp(nfonct,"set_______",LENNOM)==0 || memcmp(nfonct,"ack_______",LENNOM)==0 || memcmp(nfonct,"sw",2)==0 ){
        assySet(message,periCur,periDiag(periMess),date14,nfonct);}      // assemblage datas ; ci-après buildMess controle l'ovf
    else if(strlen(msg)<(LENMESS-2)){strcat(message,msg);}

    *bufServer='\0';
    memcpy(bufServer,"GET /\0",6);                                // commande directe de périphérique en mode serveur
    int lbs=buildMess(nfonct,message,"",NODIAGS);
    if(lbs==0 || (lbs+2)>LBUFSERVER){ledblink(BCODEPERIRECLEN);}  // bufServer complété
    strcat(bufServer,"\n\n");                                     // fin de message pour le périf
    
    Serial.print(" dur=");Serial.println(millis()-dur);//Serial.print(bufServer);

    if(*periProtocol=='T'){                                       // UDP à développer (sortie ret=MESSCX)
          showSocketsStatus(false,true,true,"periReq ");
          periMess=messToServer(cli,host,*periPort,bufServer);
          //Serial.print("(");Serial.print(MESSOK);Serial.print(" si ok) periMess(messToServer)=");Serial.println(periMess);
          //Serial.print(periMess);Serial.print("-");Serial.print(millis());Serial.print(" ");
          uint8_t fonct;
          if(periMess==MESSOK){
              trigwd();
              periMess=getHttpResponse(cli,bufServer,LBUFSERVER,&fonct,false);
              //Serial.print("(");Serial.print(MESSOK);Serial.print(" si ok) periMess(gHttpR)=");Serial.println(periMess);
              if(periMess==MESSOK){
                //if(fonct>=nbfonct){fonct=nbfonct;periMess=MESSFON;} // déjà testé dans getHttpResponse/chkHttpData
                //else {
                  //Serial.println(bufServer);
                  //Serial.print(periMess);Serial.print("-");Serial.print(millis());Serial.print(" ");
                  if(fonct==fdatasave || fonct==fdatana){
                    memcpy(remote_IP_cur,periIpAddr,4); // remote_IP_cur n'est pas valide ici 
                    periDataRead(bufServer+LENNOM+1);
                    periSave(periCur,PERISAVELOCAL);    // màj cache ... toujours OK (periCur from periDataRead)
                  }
              } // getHttpResponse==MESSOK
          } // messToServer==MESSOK 
          //purgeCli(cli,NODIAGS);
          if(periMess==MESSOK){packDate(periLastDateOut,date14+2);}
          *periErr=periMess;
    } // periProtocole=='T'
    
    if(*periProtocol=='U'){                             // service minimum en Udp                         
      sendUdpData(*periUdpPortNb,*periIpAddr,*periPort,bufServer);
      packDate(periLastDateOut,date14+2);
      periMess=MESSOK;
    }

    //Serial.print(millis());
    cli->stop();                              // cliext
    Serial.print(" periMess=");Serial.print(periMess);Serial.print(" periReq dur=");Serial.println(millis()-dur);
    ret=periMess;
  }
#ifdef ANALYZE
  STOPALL
#endif

  return ret;
}

int periAns(EthernetClient* cli,EthernetUDP* udpCli,const char* nfonct)   // réponse à périphérique cli ... ou udp(remote_IP,remote_Port_Udp)
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
    assySet(message,periCur,periDiag(periMess),date14,nfonct);
    }  // assemblage datas 

  memcpy(bufServer,"<body>\0",7);
  //Serial.print("\n a0=");Serial.println(millis());
  buildMess(nfonct,message,"\0");                           // bufServer complété 
  strcat(bufServer,"</body>");
  //Serial.println(bufServer);
          if(*periProtocol=='T'){
            cli->write(bufServer);
            //Serial.print(" a1=");Serial.println(millis());
            //cli->stop();
            //Serial.print(" a2=");Serial.println(millis());
          }
          if(*periProtocol=='U' && udpCli!=nullptr){
            IPAddress udpAddress;
            memcpy((char*)(&udpAddress)+4,periIpAddr,4);
            udpCli->beginPacket(udpAddress,*periPort);
            //Serial.print("sending (");Serial.print(strlen(bufServer));Serial.print(")>");Serial.print(bufServer);
            //Serial.print("< to ");serialPrintIp(periIpAddr);Serial.print(":");Serial.println(*periPort);
            udpCli->write(bufServer,strlen(bufServer));
            udpCli->endPacket();
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
  int perizer=0;
  int messLen=strlen(valf)-2;   // longueur hors crc
  
  //Serial.print(millis());Serial.print(" pDR:");

//Serial.print("messLen=");Serial.print(messLen);Serial.print(" i=");Serial.print(i);Serial.print(" valf=");Serial.println((char*)valf);

  periMess=checkData(valf);     // controle len,crc
  if(periMess==MESSOK){

// ======   debut gestion periCur    =======

    periCur=0;valf+=5;conv_atob(valf,&periCur);packMac(periMacBuf,valf+3);                       
  
    //Serial.print(periCur);
  
    if(periCur!=0){                                                 // si le périph a un numéro, ctle de l'adr mac
      periLoad(periCur);
      if(memcmp(periMacBuf,periMacr,6)!=0){periCur=0;}}
    
    if(periCur==0){                                                 // si periCur=0 recherche si mac connu   
      for(i=1;i<=NBPERIF;i++){                                      // et, au cas où, une place libre
        periLoad(i);
        if(memcmp(periMacBuf,periMacr,6)==0){
          periCur=i;break;}                                         // mac trouvé
        if(memcmp("\0\0\0\0\0\0",periMacr,6)==0 && perizer==0){
          perizer=i;}        // place libre trouvée
      }
    }

    if(periCur==0 && perizer!=0){                                   // si pas connu utilisation N° perif libre "perizer"
      //periInitVar();
      periCur=perizer;
      Serial.print(" DataRead/Save nouveau périphérique :");Serial.println(perizer);
    }  
    else if(periCur==0 && perizer==0){
      Serial.println(" DataRead/Save Mac inconnu periTable saturée ");
      periMess=MESSFULL; 
    }

// ======   fin gestion periCur    =======

// ====== debut transfert des données dans periRec =======

  
    #define PNP 2+1+17+1                                            // (4 length +1) + 2 N° peri +1 + 17 Mac +1 message court de présence
    
    int oriMessLen=messLen;
    char* k;                                                        // pointeur dans message reçu (valf)

    k=valf+PNP;
    if(periCur!=0){                                                 // si ni trouvé, ni place libre, periCur=0 
      if(periCur==perizer){memcpy(periMacr,periMacBuf,6);}
      char date14[LNOW];ds3231.alphaNow(date14);checkdate(0);packDate(periLastDateIn,date14+2);checkdate(1);            // maj dates

      messLen-=(PNP+5);                                             // PNP + (4 length +1) longueur hors CRC si message terminé
      if(messLen>0){                               
        *periLastVal_=(int16_t)(convStrToNum(k,&i)*100);            // température si save

    #if PNP != HISTOPOSTEMP-HISTOPOSNUMPER
      cancelCompil();
    #endif //
      }
      messLen-=i;if(messLen>0){
        k+=i;*periAnal=convStrToInt(k,&i);                          // analog value
      }
      messLen-=i;if(messLen>0){
        k+=i;*periAlim_=(int16_t)(convStrToNum(k,&i)*100);          // alim
      }
      messLen-=i;if(messLen>0){    
        k+=i;strncpy(periVers,k,LENVERSION);                        // version
        i=strchr(k,'_')-k+1;                                        // ????? LENVERSION variable ?
      }

      messLen-=i;
      if(messLen>0){      
        k+=i;
        uint8_t nbSw=*k-PMFNCVAL;
        k+=1;*periSwSta=0;for(int i=nbSw-1;i>=0;i--){*periSwSta=(*periSwSta<<1);*periSwSta|=*(k+MAXSW-i-1)-PMFNCVAL;}
        //Serial.print(" periSwSta=");if(*periSwSta<16){Serial.print('0');}Serial.print(*periSwSta,HEX);
        k+=MAXSW+1; *periDetNb=*k-PMFNCVAL;                                                                 // nbre detec
        k+=1; *periDetVal=0;for(int i=MAXDET-1;i>=0;i--){                                                   // détecteurs
        *periDetVal=(*periDetVal)<<1;*periDetVal |= (*(k+i)-PMFNCVAL);}
      }
      //Serial.println();
      // les pulses ne sont pas transmis si ils sont à 0 ; periSwPulseSta devrait être initialisé à 0 
      messLen-=(1+MAXSW+1+1+MAXDET+1);if(messLen>0){
        k+=MAXDET+1;
        for(int i=0;i<NBPULSE;i++){periSwPulseSta[i]=(uint8_t)(strchr(chexa,(int)*(k+i))-chexa);}           // pulse clk status 
      }
      messLen-=(NBPULSE+1);if(messLen>0){
        k+=NBPULSE+1;
        if(periCur==perizer){for(int i=0;i<LENMODEL;i++){periModel[i]=*(k+i);periNamer[i]=*(k+i);}}         // model
      }
      messLen-=(LENMODEL+1);
      if(messLen>(int)(NBPULSE*sizeof(uint32_t)*2)){
        k+=LENMODEL+1;
        for(uint16_t i=0;i<2*NBPULSE*sizeof(uint32_t);i++){conv_atoh(k+2*i,(byte*)periSwPulseCurrOne+i);}   // valeur courante pulses
        //if(*periSwPulseCurrOne>*periSwPulseOne){dumpstr((char*)periSwPulseCurrOne,16);}
        //if(*periSwPulseCurrTwo>*periSwPulseTwo){dumpstr((char*)periSwPulseCurrTwo,16);}
      }
      else {memset(periSwPulseCurrOne,0x00,NBPULSE*sizeof(uint32_t));memset(periSwPulseCurrTwo,0x00,NBPULSE*sizeof(uint32_t));}

    #ifdef SHDIAGS    
      periPrint(periCur);Serial.print("periDataRead =");
    #endif //    
    
      if(ab=='u'){*periProtocol='U';}else *periProtocol='T';                              // last access protocol type
      *periSsidNb='?';                                                                
      if(*(valf+oriMessLen-8)=='*'){*periSsidNb=*(valf+oriMessLen-7);}

      memcpy(periIpAddr,remote_IP_cur,4);                                                 // Ip addr
      *periUdpPortNb=udpNb;                                                               // n° instance udp
      
      if(remote_Port_Udp!=0 && *periProtocol=='U'){*periPort=remote_Port_Udp;}            // port

      periSave(periCur,PERISAVELOCAL);
    }

    
// ====== fin transfert des données dans periRec ======

  //checkdate(6);
  }
}
