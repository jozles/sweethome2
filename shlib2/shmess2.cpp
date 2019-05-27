
#include "Arduino.h"
#include "shutil2.h"
#include "shconst2.h"

//#define PERIF

#ifdef PERIF
#include <ESP8266WiFi.h>

extern WiFiClient cli;                 // client local du serveur externe
extern WiFiClient cliext;              // client externe du serveur local

#if CONSTANT==RTCSAVED
extern int cstlen;
#endif
#endif // PERIF

#ifndef PERIF
#include <Ethernet.h> //bibliothèque W5100 Ethernet

//extern EthernetClient cli;

extern int16_t*  periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern int32_t*  periPerRefr;                  // ptr ds buffer : période maximale accès serveur
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
extern byte*     periInput;                    // ptr ds buffer : Mode fonctionnement inters (MAXTAC=4 par switch)
extern uint32_t* periSwPulseOne;               // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t* periSwPulseTwo;               // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t* periSwPulseCurrOne;           // ptr ds buffer : temps courant pulses ON
extern uint32_t* periSwPulseCurrTwo;           // ptr ds buffer : temps courant pulses OFF
extern byte*     periSwPulseCtl;               // ptr ds buffer : mode pulses
extern byte*     periSwPulseSta;               // ptr ds buffer : état clock pulses
extern uint8_t*  periSondeNb;                  // ptr ds buffer : nbre sonde
extern boolean*  periProg;                     // ptr ds buffer : flag "programmable"
extern byte*     periDetNb;                    // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*     periDetVal;                   // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern float*    periThOffset;                 // ptr ds buffer : offset correctif sur mesure température
extern float*    periThmin;                    // ptr ds buffer : alarme mini th
extern float*    periThmax;                    // ptr ds buffer : alarme maxi th
extern float*    periVmin;                     // ptr ds buffer : alarme mini volts
extern float*    periVmax;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer ; 1 byte 8*enable detecteurs serveur

extern byte      periMacBuf[6];
extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)

#endif // PERIF

extern char*  chexa;

extern char*  fonctions;
extern int    nbfonct;
extern char   bufServer[LBUFSERVER];

extern byte   mac[6];

extern byte memDetServ;  // image mémoire NBDSRV détecteurs (8)

char*         periText={TEXTMESS};

#ifndef PERIF
void purgeServer(EthernetClient* cli)
#endif // PERIF
#ifdef PERIF
void purgeServer(WiFiClient* cli)
#endif // PERIF
{
    if(cli->connected()){
        //Serial.print(" purge ");
        while (cli->available()){Serial.print(cli->read());}
        Serial.println();
    }
    cli->stop();
}


int buildMess(char* fonction,char* data,char* sep)   // concatène un message dans bufServer
{                                                    // retourne la longueur totale dans bufServer ou 0 si ovf
Serial.print(" bufserver len=");Serial.print(strlen(bufServer));
Serial.print(" data len=");Serial.print(strlen(data));
Serial.print(" LBUFSERVER=");Serial.println(LBUFSERVER);
delay(100);
      if((strlen(bufServer)+strlen(data)+11+5+2+1)>LBUFSERVER)
        {return 0;}
      strcat(bufServer,fonction);
      strcat(bufServer,"=");
      int sb=strlen(bufServer);
      int d=strlen(data)+5;    //+strlen(sep);
      sprintf(bufServer+sb,"%04d",d);
      strcat(bufServer+sb+4,"_");
      strcat(bufServer+sb+5,data);
      setcrc(bufServer+sb,d);
      strcat(bufServer,sep);
      Serial.print("bS=");Serial.println(bufServer);
Serial.println(" buildmess return");
      return strlen(bufServer);
}

#ifndef PERIF
int messToServer(EthernetClient* cli,const char* host,int port,char* data)    // connecte au serveur et transfère la data
#endif
#ifdef PERIF
int messToServer(WiFiClient* cli,const char* host,const int port,char* data)    // connecte au serveur et transfère la data
#endif
{
  byte crc;
  int x=0,v=MESSOK,repeat=0;
  long beg=millis();

#ifndef PERIF
    //purgeServer(cli);
#endif // PERIF

  while(!x && repeat<4){
    x=cli->connect(host,port);
    v=MESSOK;
    repeat++;
    Serial.print("connexion serveur (");Serial.print(x);Serial.print(") ");
    Serial.print(host);Serial.print(":");Serial.print(port);
    Serial.print("...");
    if(!cli->connected()){
/*        switch(x){
            case -1:Serial.print("time out");break;
            case -2:Serial.print("invalid server");break;
            case -3:Serial.print("truncated");break;
            case -4:Serial.print("invalid response");break;
            default:Serial.print("unknown reason");break;
        }*/
        delay(100);Serial.print(repeat);Serial.println(" échouée");v=MESSCX;
    }
  }
  if(v==MESSOK){
        Serial.println(" ok");
        cli->print(data);
        cli->print("\r\n HTTP/1.1\r\n Connection:close\r\n\r\n");
  }
  return v;
}

#ifndef PERIF
int waitRefCli(EthernetClient* cli,char* ref,int lref,char* buf,int lbuf)   // attente d'un chaine spécifique dans le flot
#endif PERIF
#ifdef PERIF
int waitRefCli(WiFiClient* cli,char* ref,int lref,char* buf,int lbuf)       // attente d'un chaine spécifique dans le flot
#endif PERIF
// wait for ref,lref    si lbuf<>0 accumule le flot dans buf (ref incluse)
// sortie MESSTO (0) time out   MESSDEC (-1) décap     MESSOK (1) OK
{
  boolean termine=FAUX;
  int ptref=0,ptbuf=0;
  char inch;
  long timerTo=millis()+TOINCHCLI;


      Serial.print("attente =");Serial.print(ref);Serial.print(" ");Serial.println(lref);
      while(!termine){
        if(cli->available()>0){
          timerTo=millis()+TOINCHCLI;
          inch=cli->read();Serial.print(inch);
          if(lbuf!=0){
            if(ptbuf<lbuf){buf[ptbuf]=inch;ptbuf++;}else {return MESSDEC;}}
          if(inch==ref[ptref]){
            ptref++;if(ptref>=lref){termine=VRAI;}}
          else{ptref=0;}
        }
        else if(millis()>=timerTo){return MESSTO;}
      }
      if(lbuf!=0){buf[ptbuf-lref]='\0';}

      return MESSOK;
}


int checkData(char* data)            // controle la structure des données d'un message (longueur,crc)
{                                    // renvoie MESSOK (1) OK ; MESSCRC (-2) CRC ; MESSLEN (-3)  le message d'erreur est valorisé

  int i=4;
  int ii=convStrToNum(data,&i);//Serial.print("len=");Serial.print(ii);Serial.print(" strlen=");Serial.println(strlen(data));
  uint8_t c=0;
  conv_atoh(data+ii,&c);

  if(ii!=strlen(data)-2){i=MESSLEN;}
/*  Serial.print("CRC, c, lenin  =");Serial.print(calcCrc(valf,lenin-2),HEX);Serial.print(", ");
  Serial.print(c,HEX);Serial.print(" , ");Serial.println(lenin);
*/
  else if(calcCrc(data,ii)!=c){i=MESSCRC;}
  else i=MESSOK;

/*  Serial.print("\nlen/crc calc ");
  Serial.print(strlen(data));Serial.print("/");Serial.print(calcCrc(data,ii),HEX);
  Serial.print(" checkData=");Serial.println(i);
*/
  return i;
}

int checkHttpData(char* data,uint8_t* fonction)   // checkData et extraction de la fonction
{
    char noms[LENNOM+1];
    int q=0;

    memcpy(noms,data,LENNOM);noms[LENNOM]='\0';
    q=checkData(data+LENNOM+1);
    if(q==MESSOK){
        *fonction=(strstr(fonctions,noms)-fonctions)/LENNOM;
        Serial.print("\nnbfonct=");Serial.print(nbfonct);Serial.print(" fonction=");Serial.print((strstr(fonctions,noms)-fonctions));Serial.print("/");Serial.print(*fonction);
        if(*fonction>=nbfonct || *fonction<0){q=MESSFON;}
    }
    Serial.print(" mess=");Serial.println(q);
    return q;
}

#ifndef PERIF
int getHttpResponse(EthernetClient* cli, char* data,int lmax,uint8_t* fonction)  // attend un message d'un serveur ; ctle longueur et crc
#endif  PERIF
#ifdef PERIF
int getHttpResponse(WiFiClient* cli, char* data,int lmax,uint8_t* fonction)      // attend un message d'un serveur ; ctle longueur et crc
#endif  PERIF
// format <body>contenu...</body></html>\n\r                        la fonction est décodée et les données sont chargées
// contenu fonction__=nnnn_datacrc                                  renvoie les codes "MESSxxx"
{
  char* body="<body>\0";
  char* bodyend="</body>\0";
  int q,v=0;
  uint8_t crc=0;

  q=waitRefCli(cli,body,strlen(body),bufServer,0);
  if(q==MESSOK){q=waitRefCli(cli,bodyend,strlen(bodyend),data,lmax-strlen(bodyend));}
  if(q==MESSOK){q=checkHttpData(data,fonction);}
  Serial.println();
  return q;
}

char* periDiag(uint8_t diag)
{
  int v=0;
  if(diag>NBMESS){diag=MESSSYS;}
  if(diag!=MESSOK){v=LPERIMESS*(diag+1);}
  return periText+v;
}

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
}
