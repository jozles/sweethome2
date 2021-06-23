#include <Arduino.h>
#include <ds3231.h>
#include "const.h"
#include <shconst2.h>
#include <shutil2.h>
#include "utilether.h"
#include "utilhtml.h"
#include "periph.h"
#include "pageshtml.h"
#include "utiljs.h"

extern Ds3231 ds3231;

extern char*      nomserver;
extern byte*      mac;              // adresse server
extern byte*      localIp;
extern uint16_t*  portserver;
extern char*      userpass;         // mot de passe browser
extern char*      modpass;          // mot de passe modif
extern char*      peripass;         // mot de passe périphériques
extern unsigned long* maxCxWt;
extern unsigned long* maxCxWu;

extern char*      mailFromAddr; 
extern char*      mailPass;     
extern char*      mailToAddr1;  
extern char*      mailToAddr2;  
extern uint16_t*  periMail1;    
extern uint16_t*  periMail2;

extern char*      chexa;
extern byte       maskbit[];

extern int        periCur;          // Numéro du périphérique courant

extern byte*      periMacr;                     // ptr ds buffer : mac address 
extern char*      periNamer;                    // ptr ds buffer : description périphérique
extern int16_t*   periLastVal_;                 // ptr ds buffer : dernière valeur de température  
extern int16_t*   periThmin_;                   // ptr ds buffer : alarme mini th
extern int16_t*   periThmax_;                   // ptr ds buffer : alarme maxi th
extern int16_t*   periThOffset_;                // ptr ds buffer : offset correctif sur mesure température
extern char*      periLastDateIn;               // ptr ds buffer : date/heure de dernière réception
extern char*      periLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
extern char*      periVers;                     // ptr ds buffer : version logiciel du périphérique
extern char*      periModel;                    // ptr ds buffer : model du périphérique
extern byte*      periSwVal;                    // ptr ds buffer peri : état/cde des inter 
extern char*      periNamer;                    // ptr ds buffer : description périphérique

extern byte       periMacBuf[MACADDRLENGTH]; 

extern uint16_t   perrefr;
extern File32     fhisto;           // fichier histo sd card
extern long       fhsize;           // remplissage fhisto
extern long       histoPos;
extern char       histoDh[LDATEA];
extern char       strHisto[RECCHAR];


extern char*      ssid;
extern char*      passssid;
extern char*      usrnames;
extern char*      usrpass;
extern unsigned long*     usrtime;
extern uint16_t*  toPassword;

extern int        usernum;


File32 fimg;     // fichier image

extern struct SwRemote remoteT[MAXREMLI];
extern struct Remote remoteN[NBREMOTE];

extern struct Timers timersN[NBTIMERS];

extern struct Thermo thermos[NBTHERMOS];

extern int       fdatasave;

extern char      mdsSrc[];
extern uint32_t  memDetServ;                   // image mémoire NBDSRV détecteurs
extern char      libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t  sourceDetServ[NBDSRV];

   // Un formulaire ou une page html nécessite dans l'ordre :
   //      1) une fonction user_ref_ avec ses paramètres pour éviter le retour au mot de passe
   //      2) une fonction d'en tête hidden pour effectuer 
   //         les inits éventuels lors du submit de validation (posit what et effacements cb par ex)
   //         si pas d'inits à faire, elle n'est pas nécessaire
   //      3) n fonctions pour valoriser des champs
   //      ... n'importe où après (2) un bouton submit 
   //       
   // La fonction user_ref_ est fournie par userFormHtml()
   // S'il faut passer une fonction d'en-tête utiliser userFormInitHtml()
   // Si le passage de periCur est nécessaire utiliser usrPeriCur()
   //



int htmlImg(EthernetClient* cli,const char* fimgname)   
{
        unsigned long begIC=millis();
        Serial.print(fimgname);
        File32 fimg;                              // = SD.open(fimgname,FILE_READ);
        if(sdOpen(fimgname,&fimg)==SDKO){return SDKO;}
        else {
          
          cli->println("HTTP/1.1 200 OK");
          cli->println("CONTENT-Type: image/jpg");
          cli->println();

          long fimgSiz=fimg.size();
          Serial.print(" size=");Serial.print(fimgSiz);
          #define ICONLENGTH 1000
          if(fimgSiz>=ICONLENGTH){Serial.println(" fichier icon trop grand *********");}
          else {
            char icon[ICONLENGTH];            
            for(int i=0;i<fimgSiz;i++){icon[i]=fimg.read();}
            icon[fimgSiz]='\0';
            Serial.print(" ms_rd=");Serial.print(millis()-begIC);
            ethWrite(cli,icon);
            //dumpstr(icon,512);
          }
          fimg.close();        
        }
        Serial.print(" ms=");Serial.println(millis()-begIC);
        return SDOK;          // attention !!! pas de cli.stop sinon une suite éventuelle ne pourra pas partir (acceuilHtml par ex)
}

void htmlFavicon(EthernetClient* cli)
{
  htmlImg(cli,"sweeth.png");
}

void dumpHisto0(EthernetClient* cli,long histoPos)                 // liste le fichier histo depuis une adresse
{
  cli->print(histoPos);cli->print("/");cli->print(fhsize);cli->println("<br>");
  
  long ptr=histoPos;
  long ptr0=ptr;
  long ptra=ptr;
  
  const uint16_t lb0=LBUF4000;
  char buf[lb0];buf[0]='\0';

  fhisto.seek(histoPos);
  
  while(ptr<fhsize){
    trigwd();
    while((ptr-ptra) < lb0 && ptr<fhsize){           // -1 for end null char
      buf[ptr-ptra]=fhisto.read();ptr++;
    }
    buf[ptr-ptra]='\0';
    ethWrite(cli,buf);
    ptra=ptr;
    if((ptr-ptr0)>1000000){break;}
  }
  
  fhisto.close();
}

void dumpHisto(EthernetClient* cli)
{
  Serial.print(" dump histo ");delay(100);

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  //unsigned long begTPage=millis();     // calcul durée envoi page
  long pos=histoPos;
  char file[]={"fdhisto.txt"};

  htmlIntroB(buf,nomserver,cli);    // chargement CSS etc
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageHeader(buf,jsbuf);            // 1ère ligne page
  boutRetourB(buf,jsbuf,"retour",0);

  ethWrite(cli,buf,&lb);

  trigwd();

  strcat(buf,"histoSD ");
  if(sdOpen(file,&fhisto)==SDKO){strcat(buf,"KO");return;}
  fhsize=fhisto.size();

  if(histoDh[0]=='2'){
    shDateHist(histoDh,&pos);
    strcat(buf,histoDh);strcat(buf," - ");
  }
  ethWrite(cli,buf);

  dumpHisto0(cli,pos);

  boutRetourB(buf,"retour",0,1);
  strcat(buf,"</body></html>\n");
  ethWrite(cli,buf);
}

void shDateHist(char* dhasc,long* pos)
{ 
  long searchStep=100000;
  long ptr,curpos=fhisto.size();
  fhisto.seek(curpos);
  long pos0=curpos;  
  long t0=millis();
  char inch=0;
  char buf[RECCHAR];memset(buf,0x00,RECCHAR);
  int v;
  bool fini=FAUX;
  uint8_t trigcnt=10,pt,ldate=LDATEA-2;

/* recherche rétrograde de la date */

  while(curpos>0 && !fini){
    curpos-=searchStep;if(curpos<0){curpos=0;}
    fhisto.seek(curpos);                                                        // curpos sur bloc courant
    ptr=curpos;                                                                 // ptr dans bloc courant           
                        
    while(ptr<curpos+searchStep && inch!='\n'){inch=fhisto.read();ptr++;}       // lit jusqu'au 1er \n 
    if(inch!='\n'){fini=VRAI;break;}                                            // pas de \n donc fini
    inch='0';
    for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                       // \n trouvé : get date
    v=memcmp(buf,dhasc,ldate);
    trigcnt++;if(trigcnt>10){trigwd();trigcnt=0;}
    if(v<0){                                                                    // buf<dhasc la date trouvée est plus petite,
                                                                                // chercher la première >= dans le bloc courant
                                                                                // si fin du bloc, date(pos0) est la bonne
      while(ptr<curpos+searchStep){  
        inch=fhisto.read();ptr++;                                               // lit jusqu'au \n suivant
        if(inch=='\n'){                                                         // pas de \n donc fini
          for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                 // '\n' trouvé : get date
          v=memcmp(buf,dhasc,ldate);                                            // si la date trouvée est plus petite continuer sinon terminé
          trigcnt++;if(trigcnt>10){trigwd();trigcnt=0;}
          if(v>=0){fini=VRAI;pos0=ptr-ldate;break;}                             // date plus grande ou égale -> fin
        }
      }fini=VRAI;                                                               // pas trouvé de date plus grande donc date(pos0) est ok
    }  
    else if (v==0){fini=VRAI;pos0=ptr-ldate;}                                   // date trouvée égale
    else pos0=ptr-ldate;                                                        // si la date trouvée est plus grande explorer le bloc précédent                                                                             
                                                                                // s'il ne contient que de plus petites date(pos0) est ok
  }
  *pos=pos0;
  Serial.print("--- fin recherche ptr=");Serial.print(pos0);Serial.print(" millis=");Serial.println(millis()-t0);
}

void shDicDateHist(char* dhasc,long* but)
{ 
  long fhsize=fhisto.size();
  long pos=fhsize/2;
  long searchStep=fhsize;
  long ptr=0;
  long t0=millis();
  char inch=0;
  char buf[RECCHAR];
  bool fini=FAUX;
  uint8_t pt,v;
  uint8_t ldate=LDATEA-2;
  int miniL=50; // ???

/* recherche dichotomique de la date */

  *but=fhsize;

  while(searchStep>miniL && ptr<pos+searchStep && !fini){

    ptr=pos;
    inch=' ';
    while(inch!='\n' && ptr<pos+searchStep){inch=fhisto.read();}
    if(inch!='\n'){fini=VRAI;}
    else {
      fhisto.seek(ptr);
      for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}              
      v=memcmp(buf,dhasc,ldate);
      if(v>0){*but=ptr;searchStep/=2;pos-=searchStep;}
      else if(v==0){*but=ptr;fini=VRAI;}
      else {searchStep/=2;pos+=searchStep;}
    }
  }
    
  Serial.print("--- fin recherche ptr=");Serial.print(ptr);Serial.print(" millis=");Serial.println(millis()-t0);
}


void accueilHtml(EthernetClient* cli)
{
      const uint16_t lb0=LBUF4000;
      char buf[lb0];buf[0]='\0';
      unsigned long begAC=millis();
      
            Serial.print(begAC);Serial.print(" accueilHtml ");
            htmlIntro0B(buf);                                               // ,nomserver,cli);

            strcat(buf,"<body><form method=\"get\" >");
            strcat(buf,"<h1 class=\"point\">");
            strcat(buf,VERSION);strcat(buf,"<br>\n");

            strcat(buf,"<p><input type=\"username\" text style=\"width:220px;height:60px;font-size:40px\" placeholder=\"Username\" name=\"username__\"  value=\"\" size=\"6\" maxlength=\"8\" ></p>\n");            
            strcat(buf,"<p><input type=\"password\" text style=\"width:220px;height:60px;font-size:40px\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>\n");
            strcat(buf," <input type=\"submit\" text style=\"width:300px;height:60px;font-size:40px\" value=\"login\"><br>\n");
            strcat(buf,"</h1></form></body></html>\n");
            ethWrite(cli,buf);
            Serial.print(" ms=");Serial.println(millis()-begAC);
}          

void sscb(char* buf,bool val,const char* nomfonct,int nuf,int etat,uint8_t td,uint8_t nb)
{                                                                               // saisie checkbox ; 
                                                                                // le nom de fonction reçoit 2 caractères
  char nf[LENNOM+1];
  memcpy(nf,nomfonct,LENNOM);
  nf[LENNOM]='\0';
  nf[LENNOM-1]=(char)(nb+PMFNCHAR);
  nf[LENNOM-2]=(char)(nuf+PMFNCHAR);
  checkboxTableBHtml(buf,(uint8_t*)&val,nf,etat,td,"");
}

void sscfgtB(char* buf,const char* nom,uint8_t nb,const void* value,int len,uint8_t type)  // type=0 value ok ; type =1 (char)value modulo len*nt ; type =2 (uint16_t)value modulo nb
{
  int sizbx=len-3;if(sizbx<=0){sizbx=1;}
  strcat(buf,"<td><input type=\"text\" name=\"");strcat(buf,nom);concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
  if(type==0){strcat(buf,(char*)value);strcat(buf,"\" size=\"");concatn(buf,sizbx);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>");}
  if(type==2){concatn(buf,*((int16_t*)value+nb));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>");}
  if(type==1){strcat(buf,(char*)(((char*)value+(nb*(len+1)))));strcat(buf,"\" size=\"");concatn(buf,len);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>\n");}
  if(type==3){concatn(buf,*((int8_t*)value));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>\n");}
}
/*
void sscfgt(char* buf,char* nom,uint8_t nb,void* value,int len,uint8_t type)  // type=0 value ok ; type =1 (char)value modulo len*nt ; type =2 (uint16_t)value modulo nb
{
  int sizbx=len-3;if(sizbx<=0){sizbx=1;}
  strcat(buf,"<td><input type=\"text\" name=\"");strcat(buf,nom);concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
  if(type==0){strcat(buf,(char*)value);stract(buf,"\" size=\"");concatns(buf,sizbx);strcat(buf,"\" maxlength=\"");concatns(buf,len);strcat(buf,"\" ></td>\n");}
  if(type==2){concatns(buf,*((int16_t*)value+nb));stract(buf,"\" size=\"1\" maxlength=\"2\" ></td>\n");}
  if(type==1){strcat(buf,(char*)(((char*)value+(nb*(len+1)))));strcat(buf,"\" size=\"");concatns(buf,len);strcat(buf,"\" maxlength=\"");concatns(buf,len);strcat(buf,"\" ></td>\n");}
  if(type==3){concatns(buf,*((int8_t*)value));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>\n");}
}*/

void subcfgtable(char* buf,const char* titre,int nbl,const char* nom1,const char* value1,int len1,uint8_t type1,const char* nom2,void* value2,int len2,const char* titre2,uint8_t type2)
{   
    strcat(buf,"<table><col width=\"22\"><tr><th></th><th>");strcat(buf,titre);strcat(buf,"</th><th>");strcat(buf,titre2);strcat(buf,"</th></tr>\n");

    for(int nb=0;nb<nbl;nb++){
      strcat(buf,"<tr><td>");concatns(buf,nb);strcat(buf,"</td>");

      sscfgtB(buf,nom1,nb,value1,len1,type1);                      
      sscfgtB(buf,nom2,nb,value2,len2,type2);

      if(len2==-1){
        int16_t peri=*((int16_t*)value2+nb);
        if(peri>0){Serial.print(peri);periLoad(peri);strcat(buf,"<td>");strcat(buf,periNamer);strcat(buf,"</td>");}
        if(nb==nbl-1){Serial.println();}
      }
      strcat(buf,"</tr>");
    }
    strcat(buf,"</table>");
}

void cfgServerHtml(EthernetClient* cli)
{
            Serial.print(" config serveur ");

            char buf[LBUF4000];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;

            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            ethWrite(cli,buf);
            
            strcat(buf," <input type=\"submit\" value=\" MàJ \"><br> \n");
            
            /*cli->print(" password <input type=\"text\" name=\"pwdcfg____\" value=\"");cli->print(userpass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");
            cli->print("  modpass <input type=\"text\" name=\"modpcfg___\" value=\"");cli->print(modpass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");            
            cli->print(" peripass <input type=\"text\" name=\"peripcfg__\" value=\"");cli->print(peripass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");*/

            strcat(buf," serverMac <input type=\"text\" name=\"ethcfg___m\" value=\"");
            for(int k=0;k<MACADDRLENGTH;k++){concat1a(buf,chexa[mac[k]/16]);concat1a(buf,chexa[mac[k]%16]);}strcat(buf,"\" size=\"11\" maxlength=\"12\" >\n");                        
            strcat(buf," localIp <input type=\"text\" name=\"ethcfg___i\" value=\"");
            for(int k=0;k<4;k++){concatns(buf,localIp[k]);if(k!=3){strcat(buf,".");}}strcat(buf,"\" size=\"11\" maxlength=\"15\" >\n");                        
            strcat(buf," portserver ");numTf(buf,'d',portserver,"ethcfg___p",4,0,0);strcat(buf,"<br>\n");
            ethWrite(cli,buf);
            subcfgtable(buf,"SSID",MAXSSID,"ssid_____",ssid,LENSSID,1,"passssid_",passssid,LPWSSID,"password",1);
            ethWrite(cli,buf);
            strcat(buf," to password ");numTf(buf,'d',toPassword,"to_passwd_",6,0,0);strcat(buf,"<br>\n");
            subcfgtable(buf,"USERNAME",NBUSR,"usrname__",usrnames,LENUSRNAME,1,"usrpass__",usrpass,LENUSRPASS,"password",1);
            
            ethWrite(cli,buf);
            
            strcat(buf," mail From <input type=\"text\" name=\"mailcfg__f\" value=\"");strcat(buf,mailFromAddr);strcat(buf,"\" size=\"16\" maxlength=\"");concatns(buf,LMAILPWD);strcat(buf,"\" >\n");
            strcat(buf," password  <input type=\"text\" name=\"mailcfg__w\" value=\"");strcat(buf,mailPass);strcat(buf,"\" size=\"16\" maxlength=\"");concatns(buf,LMAILPWD);strcat(buf,"\" ><br>\n");
            strcat(buf," mail To #1 <input type=\"text\" name=\"mailcfg__1\" value=\"");strcat(buf,mailToAddr1);strcat(buf,"\" size=\"16\" maxlength=\"");concatns(buf,LMAILADD);strcat(buf,"\" >\n");
            strcat(buf," mail To #2 <input type=\"text\" name=\"mailcfg__2\" value=\"");strcat(buf,mailToAddr2);strcat(buf,"\" size=\"16\" maxlength=\"");concatns(buf,LMAILADD);strcat(buf,"\" ><br>\n");
            
            strcat(buf," perif 1 ");numTf(buf,'d',periMail1,"mailcfg__p",2,0,0);strcat(buf,"<br>\n");
            strcat(buf," perif 2 ");numTf(buf,'d',periMail2,"mailcfg__q",2,0,0);strcat(buf,"<br>\n");

            strcat(buf,"maxCxWt ");numTf(buf,'l',maxCxWt,"ethcfg___q",8,0,0);
            strcat(buf," maxCxWu ");numTf(buf,'l',maxCxWu,"ethcfg___r",8,0,0);strcat(buf,"<br>\n");
            strcat(buf,"</form></body></html>\n");
            
            ethWrite(cli,buf);
}

void cfgDetServHtml(EthernetClient* cli)
{
  
Serial.print("-> config detServ ");delay(100);

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();     // calcul durée envoi page
  uint8_t ni=0;

  htmlIntroB(buf,nomserver,cli);    // chargement CSS etc
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageHeader(buf,jsbuf);            // 1ère ligne page
  boutRetourB(buf,jsbuf,"retour",0);strcat(buf," ");    
  boutMaj(buf,jsbuf,"MàJ",0);

  ethWrite(cli,buf,&lb);            // tfr -> navigateur
/*
            uint16_t lb0=LBUF4000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;
            uint8_t ni=0;
            uint16_t lb;
 
            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf);
            usrFormBHtml(buf,1);
*/            

/* table libellés */
  tableBeg(buf,jsbuf,0);
  affText(buf,jsbuf,"   |      Nom      |",0,TRBEG|TDBEG|TREND);

  for(uint8_t nb=0;nb<NBDSRV;nb++){
    ni++;               
    uint8_t decal=0;if(nb>=16){decal=16;}
    affNum(buf,jsbuf,'s',&nb,0,0,TRBEG|TDBE);
    char nf[LENNOM+2]="libdsrv__ ";nf[LENNOM]=(nb+decal+PMFNCHAR);nf[LENNOM+1]=0x00;
    alphaTableHtmlB(buf,jsbuf,&libDetServ[nb][0],nf,LENLIBDETSERV-1,0,TDBE|TREND);
    //strcat(buf,"<tr><td>");concatns(buf,nb);strcat(buf,"</td><td><input type=\"text\" name=\"libdsrv__");concat1a(buf,(char)(nb+decal+PMFNCHAR));
    //            strcat(buf,"\" value=\"");strcat(buf,(char*)&libDetServ[nb][0]);strcat(buf,"\" size=\"12\" maxlength=\"");concatns(buf,(LENLIBDETSERV-1));
    //            strcat(buf,"\" ></td></tr>\n");
    lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
  }
  tableEnd(buf,jsbuf,BRYES);
  formEnd(buf,jsbuf,0,0);
  htmlEnd(buf,jsbuf);

  ethWrite(cli,buf,&lb);

  Serial.print("len buf=");Serial.print(lb);Serial.print("  len jsbuf=");Serial.print(strlen(jsbuf));
  Serial.print("  ms=");Serial.println(millis()-begTPage); 

}


void cfgRemoteHtml(EthernetClient* cli)
{
  char nf[LENNOM+1];nf[LENNOM]='\0';
  uint8_t val;
  const uint16_t lb0=LBUF4000;
  char buf[lb0];buf[0]='\0';
  char jsbuf[LBUF4000];*jsbuf=0x00;
  
            Serial.print(" config remote ");
 
            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            ethWrite(cli,buf);
            
            strcat(buf," <input type=\"submit\" value=\"MàJ\"><br>");

/* table remotes */

              strcat(buf,"<table><tr>\n");
              strcat(buf,"<th>   </th><th>      Nom      </th><th> on/off </th><th> en </th></tr>\n");

              for(int nb=0;nb<NBREMOTE;nb++){
                strcat(buf,"<tr><td>");concatn(buf,nb+1);strcat(buf,"</td>");                       // n° remote
                strcat(buf,"<td><input type=\"text\" name=\"remotecfn");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
                        strcat(buf,remoteN[nb].nam);strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,LENREMNAM-1);strcat(buf,"\" ></td>");

                memcpy(nf,"remotecfo_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // état on/off
                val=(uint8_t)remoteN[nb].onoff;
                checkboxTableBHtml(buf,&val,nf,NO_STATE,1,"");
                
                memcpy(nf,"remotecfe_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // enable (inutilisé ?)
                val=(uint8_t)remoteN[nb].enable;
                //checkboxTableBHtml(buf,&val,nf,NO_STATE,1,"");
                strcat(buf,"<td>");
                radioTableBHtml(buf,val,nf,3);
                   
                strcat(buf,"</td></tr>\n");
                if(nb-nb/5*5==0){ethWrite(cli,buf);}
              }
            strcat(buf,"</table><br></form>\n");
            ethWrite(cli,buf);

/* table détecteurs */

            strcat(buf,"<table><tr><th>  </th><th>remote </th><th>detec on/off</th><th>detec en</th><th>peri</th><th>switch</th></tr>\n");
              
              for(int nb=0;nb<MAXREMLI;nb++){
                
                strcat(buf,"<tr><form method=\"get\" >");
                usrFormBHtml(buf,1);
                strcat(buf,"<tr><td>");concatn(buf,nb+1);strcat(buf,"</td>");                  // n° ligne de table
                
                memcpy(nf,"remotecfu_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° remote
                numTf(buf,'b',&remoteT[nb].num,nf,2,2,0);
                if(remoteT[nb].num!=0){strcat(buf,remoteN[remoteT[nb].num-1].nam);}strcat(buf," </td>\n");

                memcpy(nf,"remotecfd_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° detec on/off
                numTf(buf,'b',&remoteT[nb].detec,nf,2,2,0);
                if(remoteT[nb].num!=0){
                  strcat(buf,(char*)(&libDetServ[remoteT[nb].detec][0]));strcat(buf," ");
                  concat1a(buf,(char)(((memDetServ>>remoteT[nb].detec)&0x01)+48));}
                strcat(buf," </td>\n");

                memcpy(nf,"remotecfb_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° detec enable
                numTf(buf,'b',&remoteT[nb].deten,nf,2,2,0);
                if(remoteT[nb].num!=0){
                  strcat(buf,(char*)(&libDetServ[remoteT[nb].deten][0]));strcat(buf," ");
                  concat1a(buf,(char)(((memDetServ>>remoteT[nb].deten)&0x01)+48));}
                strcat(buf," </td>\n");
                
                memcpy(nf,"remotecfp_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° périphérique
                numTf(buf,'b',&remoteT[nb].peri,nf,2,2,0);
                uint8_t rp=remoteT[nb].peri;
                if(rp!=0){periLoad(rp);periCur=rp;strcat(buf,periNamer);strcat(buf," ");}
                strcat(buf," </td>\n");

                memcpy(nf,"remotecfs_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° switch
                numTf(buf,'b',&remoteT[nb].sw,nf,2,2,0);
                strcat(buf," </td>\n");
                
                strcat(buf,"<td> <input type=\"submit\" value=\"MàJ\"><br></td>");
                strcat(buf,"</form></tr>\n");
                if(nb-nb/5*5==0){ethWrite(cli,buf);}
                //ni=0;               
              }
              
              
            strcat(buf,"</table></body></html>\n");
            ethWrite(cli,buf);
            Serial.println("fin cfgRemote");
}


void remoteHtml(EthernetClient* cli)
{              
            Serial.print(millis());Serial.print(" remoteHtml() ");

            uint16_t lb0=LBUF4000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;
            uint8_t ni=0;                                       // nbre lignes dans buffer
            uint16_t lb;
 
            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            ethWrite(cli,buf);
            
/* table remotes */
           
            strcat(buf,"<table>");            

            for(uint8_t nb=0;nb<NBREMOTE;nb++){
              ni++;
              if(remoteN[nb].nam[0]!='\0'){
                strcat(buf,"<tr><td>");concatn(buf,nb+1);strcat(buf,"</td>\n");  // numéro de ligne

                // affichage état d'un éventuel switch
                // boucle des détecteurs pour trouver un switch (voir le commentaire des disjoncteurs, c'est idem)               
                strcat(buf,"<td>");
                for(uint8_t td=0;td<MAXREMLI;td++){
                  if(remoteT[td].num==nb+1 && remoteT[td].peri!=0){           // même remote et présence périphérique
                    periCur=remoteT[td].peri;periLoad(periCur);
                    
                    if(periSwLev(remoteT[td].sw)==1){                         // switch ON
                      strcat(buf," ON <div id=\"rond_jaune\"></div>");
                    }
                    else {
                      strcat(buf," OFF ");}
                  }
                }
                
                strcat(buf,"</td><td> <font size=\"7\">");strcat(buf,remoteN[nb].nam);strcat(buf,"</font></td>");      // nom remote
                // slider on/off
                // chaque ligne de slider envoie 2 commandes : remote_cnx et remote_ctx (x n° de ligne)
                // l'input hidden remote_cnx assure la présence d'une fonction dans 'GET /' pour assurer l'effacement des cb
                // l'input remote_ctx renseigne le passage à "1" éventuel après l'effacement. La variable newonoff stocke le résultat
                // et la comparaison avec onoff permet de connaitre la transition (ou non transition)
                // periRemoteUpdate détecte les transitions, positionne les détecteurs et déclenche poolperif si nécessaire 
                // pour la maj via PerToSend des périphériques concernés
                strcat(buf,"<input type=\"hidden\" name=\"remote_cn");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">");
                sliderBHtml(buf,(uint8_t*)(&remoteN[nb].onoff),"remote_ct",nb,0,1);             // slider

                  strcat(buf,"<td>- - - - -</td>\n");                                           // ne devrait pas afficher le disjoncteur si une ligne précédente l'a déjà affiché
                  bool vert=FAUX;                                                               // pour ce perif/sw (créer une table fugitive des disj déjà affichés ?)
                  yradioTableBHtml(buf,remoteN[nb].enable,"remote_cs",2,vert,nb,1);             // renvoie 0,1,2 selon OFF,ON,FOR

                strcat(buf,"</tr>");
                
                lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}               
             }
            }
            if(buf[0]!='\0'){ethWrite(cli,buf);}
            strcat(buf,"</table>");
            
            strcat(buf,"<p align=\"center\" ><input type=\"submit\" value=\"MàJ\" style=\"height:120px;width:400px;background-color:LightYellow;font-size:60px;font-family:Courier,sans-serif;\"><br>\n");
            boutF(buf,"thermoshow","","températures",0,0,7,0);strcat(buf," \n");
            boutF(buf,"remotehtml","","refresh",0,0,7,0);
            strcat(buf,"</p>\n");
            
            strcat(buf,"</form></body></html>");
            ethWrite(cli,buf);
}

int scalcTh(int bd)           // maj temp min/max des périphériques sur les bd derniers jours
{
  
/* --- calcul date début --- */


  unsigned long t0=millis();
  
  int   ldate=LDATEA;

  int   yy,mm,dd,js,hh,mi,ss;
  byte  yb,mb,db,dsb,hb,ib,sb;
  ds3231.readTime(&sb,&ib,&hb,&dsb,&db,&mb,&yb);          // get date(now)
  yy=yb+2000;mm=mb;dd=db;hh=hb;mi=ib;ss=sb;
  calcDate(bd,&yy,&mm,&dd,&js,&hh,&mi,&ss);               // compute date-bd
  uint32_t amj=yy*10000L+mm*100+dd;
  uint32_t hms=hh*10000L+mi*100+ss;
  char     dhasc[ldate+1];
  sprintf(dhasc,"%.8lu",amj);strcat(dhasc," ");
  sprintf(dhasc+9,"%.6lu",hms);dhasc[15]='\0';            // dhasc date/heure recherchée imprimable
//  Serial.print("dhasc=");Serial.println(dhasc);
  
  if(sdOpen("fdhisto.txt",&fhisto)==SDKO){return SDKO;}
  
  long histoSiz=fhisto.size();
  long searchStep=100000;
  long ptr=0,curpos=histoSiz;
  fhisto.seek(curpos);
  long pos=fhisto.position();  
  
  Serial.print("--- start search date at ");Serial.print(curpos-searchStep);Serial.print(" histoSiz=");Serial.print(histoSiz);Serial.print(" pos=");Serial.println(pos);

/* --- recherche 1ère ligne --- */

  char inch1=0;
  char buf[RECCHAR];
  bool fini=FAUX;
  int pt;

/* recherche rétrograde de la date début */
  while(curpos>0 && !fini){
    curpos-=searchStep;if(curpos<0){curpos=0;}ptr=curpos;
    fhisto.seek(curpos);
    while(ptr<curpos+searchStep && inch1!='\n'){inch1=fhisto.read();ptr++;}
    for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                    // '\n' trouvé : get date
    if(memcmp(buf,dhasc,ldate)>0){ptr=curpos+searchStep;}                    // si la date trouvée est > reculer
    else {                                                                   // sinon chercher >
      
      while(!fini){
        while(ptr<pos && inch1!='\n'){inch1=fhisto.read();ptr++;}
        for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                // '\n' trouvé : get date
        if(memcmp(buf,dhasc,ldate)>0){                                       // si la date trouvé est > ok sinon continuer
          fini=VRAI;                                                         // ptr ok ; pt ok commencer l'acquisition
        }
      }
    }
  }
  Serial.print("--- fin recherche ptr=");Serial.print(ptr);Serial.print(" millis=");Serial.println(millis()-t0);

/* --- balayage et màj --- */

  unsigned long t1=millis();
  char strfds[3];memset(strfds,0x00,3);
  if(convIntToString(strfds,fdatasave)>2){
    Serial.print("fdatasave>99!! ");Serial.print("fdatasave=");Serial.print(fdatasave);Serial.print(" strfds=");Serial.println(strfds);ledblink(BCODESYSERR);
  }
  char* pc;
  int16_t th_;
  uint8_t np_;
  int lnp=0,nbli=0,nbth=0;
  bool save=false;

  for(int pp=1;pp<=NBPERIF;pp++){periLoad(pp);*periThmin_=9900;*periThmax_=-9900;periSave(pp,PERISAVELOCAL);}
                                                                             
                                                                         // acquisition
  fhisto.seek(ptr-ldate);                                                // sur début enregistrement
  fini=FAUX;
  while(ptr<pos){

    pt=0;
    inch1='\0';      
    while(ptr<pos && inch1!='\n'){inch1=fhisto.read();buf[pt]=inch1;pt++;ptr++;}   // get record
    buf[pt]='\0';
    nbli++;
    pc=strchr(buf,';');
    if(memcmp(pc+1,strfds,2)==0){                                         // datasave (après ';' soit '\n' soit'<' soit num fonction)
      np_=(uint8_t)convStrToInt(pc+HISTOPOSNUMPER,&lnp);                     // num périphérique
      th_=(int16_t)(convStrToNum(pc+HISTOPOSTEMP,&lnp)*100);                 // temp périphérique
      periLoad(np_);
//    Serial.print(np_);Serial.print(" ");Serial.print(*periThmin_);Serial.print(" ");Serial.println(*periThmax_);
      packMac(periMacBuf,pc+HISTOPOSMAC);                       
      if(compMac(periMacBuf,periMacr) && th_<9900 && th_>-9900){                                  // contrôle mac
//        Serial.println(buf);Serial.print(" per=");Serial.print(np_);Serial.print(" th=");Serial.print(th_);Serial.print(" thmin=");Serial.print(*periThmin_);Serial.print(" thmax=");Serial.print(*periThmax_);Serial.print(" - ");
        save=false;
        if(*periThmin_>th_){*periThmin_=(int16_t)th_;save=true;}
        if(*periThmax_<th_){*periThmax_=(int16_t)th_;save=true;}
        if(save){periSave(np_,PERISAVELOCAL);nbth++;}
//        Serial.println();
      }
    }
  }
  
  periTableSave();
  
  Serial.print("--- fin balayage ");Serial.print(nbli);Serial.print(" lignes ; ");Serial.print(nbth);
  Serial.print(" màj ; millis=");Serial.print(millis()-t1);Serial.print(" total=");Serial.print(millis()-t0);

  delay(1);
  //fhisto.seek(end);
  return 1;
  //return sdOpen("fdhisto.txt",&fhisto);
}

void thermoShowHtml(EthernetClient* cli)
{
            Serial.print(" show thermos ");
            
            uint16_t lb0=LBUF4000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;
            uint8_t ni=0;
            uint16_t lb;

            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            strcat(buf,"<br>");
            ethWrite(cli,buf);
 
            scalcTh(1);          // update periphériques

/* peritable températures */

         strcat(buf,"<table><tr><th>peri</th><th></th><th>TH</th><th>min</th><th>max</th><th>last in</th></tr>\n");

              for(int nuth=0;nuth<NBTHERMOS;nuth++){
                int16_t nuper=thermos[nuth].peri;
                if(nuper!=0){
                  periInitVar();periCur=nuper;periLoad(periCur);

                  if(periMacr[0]!=0x00){
                    ni++;
                    strcat(buf,"<tr><td>");concatns(buf,periCur);strcat(buf,"</td>\n");
                    strcat(buf,"<td> <font size=\"7\">");strcat(buf,thermos[nuth].nom);strcat(buf,"</font></td>\n");
                    strcat(buf,"<td> <font size=\"7\">");concatnf(buf,(float)(*periLastVal_+*periThOffset_)/100,2);strcat(buf,"</font></td>\n");
                    strcat(buf,"<td> <div style='text-align:right; font-size:30px;'>");concatnf(buf,(float)*periThmin_/100,2);strcat(buf,"</div></td>\n");
                    strcat(buf,"<td> <div style='text-align:right; font-size:30px;'>");concatnf(buf,(float)*periThmax_/100,2);strcat(buf,"</div></td>\n");
                    strcat(buf,"<td>");bufPrintPeriDate(buf,periLastDateIn);strcat(buf,"</td>\n");                      
                    strcat(buf,"</tr>\n");
                    lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
                  }
                }
              }
          strcat(buf,"</table><br>\n");

        strcat(buf,"<p align=\"center\">");
        boutF(buf,"remotehtml","","remote",0,0,7,0);                
        boutF(buf,"thermoshow","","refresh",0,0,7,0);
        
        strcat(buf,"</p><br>");for(int d=0;d<NBDSRV;d++){concat1a(buf,(char)(((memDetServ>>d)&0x01)+48));strcat(buf," ");}
        strcat(buf,"\n</body></html>\n");
        
        ethWrite(cli,buf);
}

void subthd(char* buf,char param,uint8_t nb,void* val,char type)
{
        char nf[LENNOM+1];
        memcpy(nf,"thparams__\0",LENNOM+1);
        nf[LENNOM-2]=param;
        nf[LENNOM-1]=nb+PMFNCHAR;                              
        
        switch(type){
          case 'd': numTf(buf,'b',val,nf,2,1,0);break;                           // n° detec/peri              
          case 'v': numTf(buf,'I',val,nf,4,1,0);break;                           // value/offset
          case 'e': checkboxTableBHtml(buf,(uint8_t*)val,nf,NO_STATE,1,"");break;      // enable/state
          default:break;
        }
}

void thermoCfgHtml(EthernetClient* cli)
{ 
            Serial.print(" config thermos ");

            const uint16_t lb0=LBUF4000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;
            uint8_t ni=0;
            uint16_t lb;
 
            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf,NOFORM);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            boutF(buf,"thermoscfg","","refresh",0,0,1,0);
            ethWrite(cli,buf);          

/* table thermos */

            strcat(buf,"<table><tr><th>   </th><th>      Nom      </th><th> peri </th><th>  </th><th> low<br> en</th><th> low<br> state</th><th> low<br> value</th><th> low<br> pitch</th><th> low<br> det</th><th> </th><th> high<br> en</th><th> high<br> state</th><th> high<br> value</th><th> high<br> offset</th><th> high<br> det</th></tr>\n");

              for(int nb=0;nb<NBTHERMOS;nb++){
                ni++;
                
                strcat(buf,"<tr><form method=\"get\" >");
                usrFormBHtml(buf,1);
                strcat(buf,"<td>");concatn(buf,nb+1);strcat(buf,"</td>");                                       // n° thermo
                
                strcat(buf,"<td><input type=\"text\" name=\"thparamsn");concat1a(buf,(char)(nb+PMFNCHAR));      // nom
                  strcat(buf,"\" value=\"");
                  strcat(buf,thermos[nb].nom);strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,LENTHNAME-1);strcat(buf,"\" ></td>\n");
    
                subthd(buf,'p',nb,&thermos[nb].peri,'d');                                                       // peri
                  periInitVar();periCur=thermos[nb].peri;if(periCur!=0){periLoad(periCur);}strcat(buf,"<td>");strcat(buf,periNamer);
                  strcat(buf,"|");concatnf(buf,(float)*periLastVal_/100);strcat(buf,"</td>\n");
                subthd(buf,'e',nb,&thermos[nb].lowenable,'e');                                                  // low enable
                subthd(buf,'s',nb,&thermos[nb].lowstate,'e');                                                   // low state
                subthd(buf,'v',nb,&thermos[nb].lowvalue,'v');                                                   // low value
                subthd(buf,'o',nb,&thermos[nb].lowoffset,'v');                                                  // low offset
                subthd(buf,'d',nb,&thermos[nb].lowdetec,'d');                                                   // low det
                  uint8_t vv=(uint8_t)thermos[nb].lowdetec;strcat(buf," <td>");if(vv!=0){strcat(buf,(char*)(&libDetServ[vv][0]));strcat(buf," ");concat1a(buf,(char)(((memDetServ>>vv)&0x01)+48));}strcat(buf," </td>\n");
                

                strcat(buf,"<td> <input type=\"submit\" value=\"MàJ\"><br></td></form></tr>\n");
                lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
              }
            strcat(buf,"</table></body></html>\n");
            ethWrite(cli,buf);
}


void timersHtml(EthernetClient* cli)
{
  int nucb;           
  //char bufdate[LNOW];ds3231.alphaNow(bufdate);
  
            Serial.print(" config timers ");

            uint16_t lb=0;
            const uint16_t lb0=LBUF4000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;

            htmlIntroB(buf,nomserver,cli);
            pageHeader(buf,jsbuf,NOFORM);
            usrFormBHtml(buf,1);
            boutRetourB(buf,"retour",0,0);
            boutF(buf,"timershtml","","refresh",0,0,1,0);
            ethWrite(cli,buf,&lb);

            detServHtml(cli,buf,nullptr,&lb,lb0,&memDetServ,&libDetServ[0][0]);

            strcat(buf,"<table>");
              strcat(buf,"<tr>");
                strcat(buf,"<th></th><th>nom</th><th>det</th><th>h_beg</th><th>h_end</th><th>OI det</th><th>e_p_c_</th><th>7_d_ l_m_m_ j_v_s</th><th>dh_beg_cycle</th><th>dh_end_cycle</th>");
              strcat(buf,"</tr>");

              for(int nt=0;nt<NBTIMERS;nt++){

                    strcat(buf,"<tr>");
                    strcat(buf,"<form method=\"GET \">");
                      usrFormBHtml(buf,1);
                      strcat(buf,"<td>");concatn(buf,nt+1);strcat(buf,"</td>");
                     
                      sscfgtB(buf,"tim_name_",nt,timersN[nt].nom,LENTIMNAM,0);
                      sscfgtB(buf,"tim_det__",nt,&timersN[nt].detec,2,3);                                            
                      sscfgtB(buf,"tim_hdf_d",nt,timersN[nt].hdeb,6,0);                                            
                      sscfgtB(buf,"tim_hdf_f",nt,timersN[nt].hfin,6,0);                   
                    
                      char oi[]="OI";   //,md=memDetServ>>timersN[nt].detec;
                      strcat(buf,"<td>_");
                      concat1a(buf,oi[timersN[nt].curstate]);strcat(buf,"__");
                      concatn(buf,(memDetServ>>timersN[nt].detec)&0x01);                      
                      strcat(buf,"</td><td>");
                      
                      nucb=0;sscb(buf,timersN[nt].enable,"tim_chkb__",nucb,NO_STATE,0,nt);
                      nucb++;sscb(buf,timersN[nt].perm,"tim_chkb__",nucb,NO_STATE,0,nt);
                      nucb++;sscb(buf,timersN[nt].cyclic,"tim_chkb__",nucb,NO_STATE,0,nt);   
ethWrite(cli,buf,&lb);                     
                      strcat(buf,"</td><td>");
                      for(int nj=7;nj>=0;nj--){
                        bool vnj; 
                        vnj=(timersN[nt].dw>>nj)&0x01;
                        nucb++;sscb(buf,vnj,"tim_chkb__",nucb,NO_STATE,0,nt);
                      }
                      strcat(buf,"</td>");
                    
                      sscfgtB(buf,"tim_hdf_b",nt,&timersN[nt].dhdebcycle,14,0);
                      sscfgtB(buf,"tim_hdf_e",nt,&timersN[nt].dhfincycle,14,0); 
                      strcat(buf,"<td> <input type=\"submit\" value=\"MàJ\"><br></td>");
                    //strcat(buf,"<td>");concat1aH(buf,timersN[nt].dw);strcat(buf," ");concat1aH(buf,maskbit[1+bufdate[14]*2]);                    
                    //strcat(buf,"</td>");
 
                    strcat(buf,"</form>");
                    strcat(buf,"</tr>\n");

                    ethWrite(cli,buf,&lb);
                  }

        strcat(buf,"</table>");
        strcat(buf,"</body></html>");
        ethWrite(cli,buf,&lb);
}


void detServHtml(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0,uint32_t* mds,char* lib)
{
  if(buf!=nullptr && lb0!=0){

    Serial.println("detServHtml ");

    formIntro(buf,jsbuf,"dsrv_init_",0,0);         // params pour retours navigateur (n° usr + time usr + pericur + locfonc pour inits)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit
  
          affText(buf,jsbuf,"<fieldset><legend>détecteurs serveur (n->0):</legend>",0,0);

          uint8_t ni=0;
          uint16_t lb1=0;

          for(int k=NBDSRV-1;k>=0;k--){
            ni++;
            char libb[LENLIBDETSERV];memcpy(libb,lib+k*LENLIBDETSERV,LENLIBDETSERV);
            if(libb[0]=='\0'){convIntToString(libb,k);}
            subDSnB(buf,jsbuf,"mem_dsrv__\0",*mds,k,libb);

            fontBeg(buf,jsbuf,1,0);
            affColonBeg(buf,jsbuf);
            affText(buf,jsbuf,&(mdsSrc[sourceDetServ[k]/256]),1,0,0);
            if(sourceDetServ[k]/256!=0){
              concatn(buf,jsbuf,sourceDetServ[k]&0x00ff);
              affText(buf,jsbuf,") ",0,0);}
            else{affText(buf,jsbuf,"--) ",0,0);}
            fontEnd(buf,jsbuf,0);
            lb1=strlen(buf);if(lb0-lb1<(lb1/ni+100)){ethWrite(cli,buf);ni=0;}
          }
          boutMaj(buf,jsbuf,"Per Update",0);
          
          formEnd(buf,jsbuf,TITLE,0,0);
          strcat(buf,"\n"); 
          
          ethWrite(cli,buf,lb);        
  }
}

void detServHtml(EthernetClient* cli,uint32_t* mds,char* lib)
{
  detServHtml(cli,nullptr,nullptr,0,0,mds,lib);
}

void testHtml(EthernetClient* cli)
{
            Serial.println(" page d'essais");
 htmlImg(cli,"sweeth.jpg");            
}           
