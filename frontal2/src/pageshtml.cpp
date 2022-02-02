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

extern bool oneIcon;

extern Ds3231 ds3231;

extern char*      serverName;
extern byte*      mac;              // adresse server
extern byte*      localIp;
extern uint16_t*  serverPort;
extern uint16_t*  remotePort;
extern uint16_t*  serverUdpPort;

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
extern uint8_t*   ssid1;
extern uint8_t*   ssid2;
extern uint8_t*   concMac;
extern uint8_t*   concRx;
extern uint16_t*  concChannel;
extern uint16_t*  concRfSpeed;      
extern byte*      concIp;               
extern uint16_t*  concPort;         
extern uint8_t*   concNb;            

extern uint8_t*   concPeriParams; 
extern float*     thFactor;
extern float*     thOffset;
extern float*     vFactor;
extern float*     vOffset;
extern byte*      periRxAddr;

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

extern int        fdatasave;

extern char       mdsSrc[];
extern uint32_t   memDetServ;                   // image mémoire NBDSRV détecteurs
extern char       libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t   sourceDetServ[NBDSRV];

extern bool       borderparam;
extern uint16_t   styleTdWidth;
extern const char*  courier;
extern char*      styleTdFont;
extern uint16_t   styleTdFSize;

extern bool styleLoadedGeneral;
extern bool styleLoadedRemote;

extern const char* introHttp;
extern const char* introHtmlE;
extern const char* generalSt;
extern const char* remoteSt;


int htmlImg(EthernetClient* cli,const char* fimgname)   
{
        unsigned long begIC=millis();
        Serial.print(fimgname);
        File32 fimg;                              // = SD.open(fimgname,FILE_READ);
        if(sdOpen(fimgname,&fimg)==SDKO){return SDKO;}
        else {
          long fimgSiz=fimg.size();
          Serial.print(" size=");Serial.println(fimgSiz);
          #define ICONLENGTH 1000
          if(fimgSiz>=ICONLENGTH){Serial.println(" fichier icon trop grand *********");}
          else {
            char icon[ICONLENGTH+JPGINTROLEN];icon[0]='\0';
            jpgIntro0(icon);
            Serial.print(' ');Serial.print(icon);Serial.print(' ');
            for(int i=strlen(icon);i<(fimgSiz+JPGINTROLEN);i++){icon[i]=fimg.read();
              //Serial.print(i);Serial.print(' ');Serial.println(icon[i],HEX);
              icon[i+1]='\0';}
            //icon[fimgSiz]='\0';
            Serial.print(" ms_rd=");Serial.print(millis()-begIC);Serial.print(" len=");Serial.print(strlen(icon));
            //cli->write(icon);
            
            //cli->write(icon,fimgSiz+JPGINTROLEN);
            ethWrite(cli,icon,fimgSiz+JPGINTROLEN);
            //dumpstr(icon,512);
          }
          fimg.close();        
        }
        Serial.print(" ms=");Serial.println(millis()-begIC);
#ifdef DEBUG_ON
  delay(10);
#endif
        return SDOK;          // attention !!! pas de cli.stop sinon une suite éventuelle ne pourra pas partir (acceuilHtml par ex)
}

void htmlFavicon(EthernetClient* cli)
{
  if(!oneIcon){
    htmlImg(cli,"sweeth.png");
    oneIcon=true;
  }
}

void dumpHisto0(EthernetClient *cli,char* buf,char*jsbuf,long histoPos,uint16_t lb0,uint16_t* lb)   // liste le fichier histo depuis une adresse
{
  scrDspNum(buf,jsbuf,'l',&histoPos,0,0,0);scrDspText(buf,jsbuf,"/",0,STRING|CONCAT|TDBEG);scrDspNum(buf,jsbuf,'l',&fhsize,0,0,BRYES);
  strcat(buf,"\n");

  ethWrite(cli,buf,lb);

  long ptr=histoPos;
  long ptr0=ptr;
  long ptra=ptr;
  
  fhisto.seek(histoPos);
  
  while(ptr<fhsize){
    trigwd();
    while(((ptr-ptra) < (long)lb0) && (ptr<fhsize)){           // -1 for end null char
      buf[ptr-ptra]=fhisto.read();ptr++;
    }
    buf[ptr-ptra]='\0';
    ethWrite(cli,buf,lb);
    ptra=ptr;
    if((ptr-ptr0)>1000000){break;}  // pour limiter la durée ...
  }
  
  fhisto.close();
}

void dumpHisto(EthernetClient* cli)
{
  Serial.print(" dump histo ");
#ifdef DEBUG_ON
  delay(100);
#endif
  char jsbuf[16000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf ne fonctionne pas avec dumpHisto0 !!!!!!!!!!!!!!!!!!!!!!!!! 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();

  //unsigned long begTPage=millis();     // calcul durée envoi page
  long pos=histoPos;
  char file[]={"fdhisto.txt"};

  htmlBeg(buf,jsbuf,serverName);     // chargement CSS etc
  
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);

  ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end
  trigwd();

  scrDspText(buf,jsbuf,"histoSD ",0,0);
  if(sdOpen(file,&fhisto)==SDKO){scrDspText(buf,jsbuf,"KO",0,0);return;}
  fhsize=fhisto.size();Serial.print(fhsize);

  if(histoDh[0]=='2'){
    shDateHist(histoDh,&pos);
    scrDspText(buf,jsbuf,histoDh,0,0);scrDspText(buf,jsbuf," - ",0,STRING|CONCAT|TDBEG);
  }
  ethWrite(cli,buf,&lb);

  dumpHisto0(cli,buf,jsbuf,pos,lb0,&lb);

  scrGetButRet(buf,jsbuf,"retour",1);
  htmlEnd(buf,jsbuf);

  ethWrite(cli,buf,&lb);

  bufLenShow(buf,jsbuf,lb,begTPage);
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
    
  Serial.print("--- fin recherche dic ptr=");Serial.print(ptr);Serial.print(" millis=");Serial.println(millis()-t0);
}


void accueilHtml(EthernetClient* cli)
{
      styleLoadedGeneral=false;
      styleLoadedRemote =false;

      uint16_t lb0=LBUF4000,lb=0;
      char buf[lb0];buf[0]='\0';
      char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;
      
      unsigned long begTPage=millis();
      
            Serial.print(" accueilHtml ");
            usernum=0;
            pageIntro0(buf,jsbuf);                       
            //formIntro(buf,jsbuf,0,0);
            
            scrDspText(buf,jsbuf,VERSION,5,BRYES);
            //strcat(buf,"<h1 class=\"point\">");
            //strcat(buf,VERSION);strcat(buf,"<br>");

            strcat(buf,"<form method=\"GET \">");
            strcat(buf,"<p><input type=\"username\" text style=\"width:220px;height:60px;font-size:40px\" placeholder=\"Username\" name=\"username__\"  value=\"\" size=\"6\" maxlength=\"8\" ></p>\n");            
            strcat(buf,"<p><input type=\"password\" text style=\"width:220px;height:60px;font-size:40px\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>\n");
            //scrGetButRet(buf,jsbuf,"Login",0);
            strcat(buf," <input type=\"submit\" text style=\"width:300px;height:60px;font-size:40px\" value=\"login\"><br></h1>\n");
            //formEnd(buf,jsbuf,0,0);
            
            //htmlEnd(buf,jsbuf);
            strcat(buf,"</form></body></html>\n");

            scrStore(jsbuf,GENSTYLE,generalSt);

            ethWrite(cli,buf,&lb);
            
            bufLenShow(buf,jsbuf,lb,begTPage);
}          

void sscb(char* buf,char* jsbuf,bool val,const char* nomfonct,int nuf,int etat,uint8_t ctl,uint8_t nb)
{                                                                               // saisie checkbox ; 
                                                                                // le nom de fonction reçoit 2 caractères
  char nf[LENNOM+1];
  memcpy(nf,nomfonct,LENNOM);
  nf[LENNOM]='\0';
  nf[LENNOM-1]=(char)(nb+PMFNCHAR);
  nf[LENNOM-2]=(char)(nuf+PMFNCHAR);
  scrGetCheckbox(buf,jsbuf,(uint8_t*)&val,nf,etat,"",0,ctl);
}

void sscfgtB(char* buf,char* jsbuf,const char* nom,uint8_t nb,const void* value,int len,uint8_t type,uint8_t ctl)   
                                                                    // nature valeur à saisir / nature de 'value'
                                                                    // type =0 char*   /    value=char*
                                                                    // type =1 char*   /    ((char*)value+nb*(len+1))=char*
                                                                    // type =2 int16_t /    (int16t*)value+nb
                                                                    // type =3 int8_t  /    value=int8_t*
{
  int8_t sizbx=len-3;if(sizbx<=0){sizbx=1;}
  char nf[LENNOM+1];memcpy(nf,nom,LENNOM-1);nf[LENNOM-1]=(char)(nb+PMFNCHAR);nf[LENNOM]=0x00;

  
  //strcat(buf,"<td><input type=\"text\" name=\"");strcat(buf,nom);concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
  if(type==0){scrGetText(buf,jsbuf,(char*)value,nf,sizbx,len,0,ctl);} //strcat(buf,(char*)value);strcat(buf,"\" size=\"");concatn(buf,sizbx);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>");}
  if(type==1){scrGetText(buf,jsbuf,(char*)((char*)value+(nb*(len+1))),nf,sizbx,len,0,ctl);} //strcat(buf,(char*)(((char*)value+(nb*(len+1)))));strcat(buf,"\" size=\"");concatn(buf,len);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>\n");}  
  if(type==2){scrGetNum(buf,jsbuf,'I',((int16_t*)value+nb),nf,2,1,0,ctl);} //concatn(buf,*((int16_t*)value+nb));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>");}
  if(type==3){scrGetNum(buf,jsbuf,'s',(int8_t*)value,nf,2,1,0,ctl);} //concatn(buf,*((int8_t*)value));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>\n");}
}

void subcfgtable(char* buf,char* jsbuf,const char* titre,int nbl,const char* nom1,const char* value1,int len1,uint8_t type1,const char* nom2,void* value2,int len2,const char* titre2,uint8_t type2)
{ 
    borderparam=NOBORDER; 
     
    tableBeg(buf,jsbuf,0);scrDspText(buf,jsbuf,"|",0,STRING|TRBEG|TDBEG);scrDspText(buf,jsbuf,titre,0,STRING|TDEND);scrDspText(buf,jsbuf,titre2,0,STRING|TREND);
    //strcat(buf,"<table><col width=\"22\"><tr><th></th><th>");strcat(buf,titre);strcat(buf,"</th><th>");strcat(buf,titre2);strcat(buf,"</th></tr>\n");

    for(int nb=0;nb<nbl;nb++){
      uint8_t n=nb+1;
      scrDspNum(buf,jsbuf,'s',&n,0,0,TRBEG|TDBE);
      //strcat(buf,"<tr><td>");concatns(buf,nb);strcat(buf,"</td>");

      sscfgtB(buf,jsbuf,nom1,nb,value1,len1,type1,TDBE);                      
      sscfgtB(buf,jsbuf,nom2,nb,value2,len2,type2,TDBE);

      if(len2==-1){
        int16_t peri=*((int16_t*)value2+nb);
        if(peri>0){Serial.print(peri);periLoad(peri);
          scrDspText(buf,jsbuf,periNamer,0,TDBE);}
          //strcat(buf,"<td>");strcat(buf,periNamer);strcat(buf,"</td>");}
        if(nb==nbl-1){Serial.println();}
      }
      scrDspText(buf,jsbuf," ",0,TREND);
      //strcat(buf,"</tr>");
    }
    tableEnd(buf,jsbuf,0);
}


void concPerParams(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0)
{
//  Serial.println(" concPerParams");            
  
  if(buf!=nullptr && lb0!=0){

    formIntro(buf,jsbuf,nullptr,0,"peripheriques concentrés",0,0);
    //formIntro(buf,jsbuf,"dsrv_init_",0,0);
    //scrDspText(buf,jsbuf,"<fieldset><legend>peripheriques concentrés</legend>",0,0);

    char periFn[]="percocfg__";       // same fonc
    char concFn[]="percocfg__";       // same fonc

    borderparam=NOBORDER;
    
    tableBeg(buf,jsbuf,0);scrDspText(buf,jsbuf,"|",0,STRING|TRBEG|TDBEG);scrDspText(buf,jsbuf,"mac|IP|Port|R Addr|channel|RF_S",0,STRING|TREND);
    
    for(int nb=0;nb<MAXCONC;nb++){
      concFn[LENNOM-2]=nb+PMFNCVAL;
      scrDspNum(buf,jsbuf,'s',&nb,0,0,TRBEG|TDBE);
    
      #define LBL 32
      char lbuf[LBL];*lbuf=0x00;
      for(uint8_t k=0;k<MACADDRLENGTH;k++){concat1a(lbuf,chexa[concMac[nb*MACADDRLENGTH+k]/16]);concat1a(lbuf,chexa[concMac[nb*MACADDRLENGTH+k]%16]);}
      concFn[LENNOM-1]='M';scrGetText(buf,jsbuf,lbuf,concFn,11,MACADDRLENGTH*2,0,TDBE);

      *lbuf=0x00;for(int k=0;k<4;k++){concatns(lbuf,concIp[nb*4+k]);if(k!=3){strcat(lbuf,".");}}
      concFn[LENNOM-1]='I';scrGetText(buf,jsbuf,lbuf,concFn,11,LBL,0,TDBE);
      concFn[LENNOM-1]='P';scrGetNum(buf,jsbuf,'d',(concPort+nb),concFn,5,0,0,TDBE);
    
      memset(lbuf,0x00,LBL);memcpy(lbuf,concRx+nb*RADIO_ADDR_LENGTH,RADIO_ADDR_LENGTH);
      concFn[LENNOM-1]='R';scrGetText(buf,jsbuf,lbuf,concFn,7,RADIO_ADDR_LENGTH,0,TDBE);    
      concFn[LENNOM-1]='C';scrGetNum(buf,jsbuf,'d',(concChannel+nb),concFn,4,0,0,TDBE);
      concFn[LENNOM-1]='S';scrGetNum(buf,jsbuf,'d',(concRfSpeed+nb),concFn,1,0,0,TDBE);

      scrDspText(buf,jsbuf," ",0,TREND);
    }
    tableEnd(buf,jsbuf,0);
  
    concFn[LENNOM-1]='N';scrDspText(buf,jsbuf,"N° concentrateur : ",0,0);scrGetNum(buf,jsbuf,'D',concNb,concFn,0,0,0,BRYES);strcat(buf,"\n");

    periFn[LENNOM-1]='k';
    scrGetRadiobut(buf,jsbuf,*concPeriParams,periFn,2,1,(char*)"keep perif values\0force server values",0,0);
  
    scrDspText(buf,jsbuf,"Radio_Peri_Addr : ",0,0);
    periFn[LENNOM-1]='c';scrGetText(buf,jsbuf,(char*)periRxAddr,periFn,11,RADIO_ADDR_LENGTH,0,BRYES);
    scrDspText(buf,jsbuf,"Volts factor : ",0,0);float factor=*vFactor*10000;
    periFn[LENNOM-1]='y';scrGetNum(buf,jsbuf,'f',&factor,periFn,4,7,2,0,0);            
    scrDspText(buf,jsbuf," Offset : ",0,0);
    periFn[LENNOM-1]='v';scrGetNum(buf,jsbuf,'f',vOffset,periFn,5,5,2,0,0);
    scrDspText(buf,jsbuf,"  Th Factor : ",0,0);factor=*thFactor*10000;
    periFn[LENNOM-1]='b';scrGetNum(buf,jsbuf,'f',&factor,periFn,4,7,2,0,0);
    scrDspText(buf,jsbuf," Offset : ",0,0);
    periFn[LENNOM-1]='e';scrGetNum(buf,jsbuf,'f',thOffset,periFn,5,5,2,0,0);
    affSpace(buf,jsbuf);
    scrGetButSub(buf,jsbuf,"Maj",0);

    formEnd(buf,jsbuf,TITLE,0,0);
    strcat(buf,"\n"); 
          
    ethWrite(cli,buf,lb);        
  }
}

void mailCfg(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0)
{
  if(buf!=nullptr && lb0!=0){

    formIntro(buf,jsbuf,nullptr,0,"mails",0,0);
    
    tableBeg(buf,jsbuf,NOBORDER,0);
    scrDspText(buf,jsbuf,"mailFrom ",2,TRBEG|TDBE);scrGetText(buf,jsbuf,mailFromAddr,"mailcfg__f",16,LMAILADD,0,TDBE);strcat(buf,"\n");
    scrDspText(buf,jsbuf," ",40,2,TDBE);scrDspText(buf,jsbuf,"password  ",0,TDBE);scrGetText(buf,jsbuf,mailPass,"mailcfg__w",16,LMAILPWD,0,TDBE|TREND);strcat(buf,"\n");
    scrDspText(buf,jsbuf,"mailTo #1 ",0,TRBEG|TDBE);scrGetText(buf,jsbuf,mailToAddr1,"mailcfg__1",16,LMAILADD,0,TDBE);strcat(buf,"\n");
    scrDspText(buf,jsbuf," ",40,2,TDBE);scrDspText(buf,jsbuf,"mailTo #2 ",0,TDBE);scrGetText(buf,jsbuf,mailToAddr2,"mailcfg__2",16,LMAILADD,0,TDBE|TREND);strcat(buf,"\n");
    tableEnd(buf,jsbuf,BRYES);

    scrDspText(buf,jsbuf,"mail perif 1 ",0,0);scrGetNum(buf,jsbuf,'d',periMail1,"mailcfg__p",2,0,0,0);
    scrDspText(buf,jsbuf," mail perif 2 ",0,0);scrGetNum(buf,jsbuf,'d',periMail2,"mailcfg__q",2,0,0,0);

    affSpace(buf,jsbuf);                    
    scrGetButSub(buf,jsbuf,"Maj",0);
    scrDspText(buf,jsbuf," si perif mail absent plantage...",0,0);
    formEnd(buf,jsbuf,TITLE,0,0);
    strcat(buf,"\n"); 
  }
}


void cfgServerHtml(EthernetClient* cli)
{
  Serial.print(" config serveur ");

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();     // calcul durée envoi page

  htmlBeg(buf,jsbuf,serverName);

  formIntro(buf,jsbuf,0,0);

  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
  scrGetButSub(buf,jsbuf,"MàJ",BRYES);

  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end
            
            /*cli->print(" peripass <input type=\"text\" name=\"peripcfg__\" value=\"");cli->print(peripass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");*/

fontBeg(buf,jsbuf,2,0);

            scrDspText(buf,jsbuf,"nom serveur ",2,TRBEG|TDBE);scrGetText(buf,jsbuf,serverName,"ethcfg___s",16,LNSERV,0,TDBE);strcat(buf,"\n");
            #define LBUFL 16
            char lbuf[LBUFL];*lbuf=0x00;for(uint8_t k=0;k<MACADDRLENGTH;k++){concat1a(lbuf,chexa[mac[k]/16]);concat1a(lbuf,chexa[mac[k]%16]);}
            scrDspText(buf,jsbuf," serverMac ",0,0);scrGetText(buf,jsbuf,lbuf,"ethcfg___m",11,MACADDRLENGTH*2,0,0);
            *lbuf=0x00;for(int k=0;k<4;k++){concatns(lbuf,localIp[k]);if(k!=3){strcat(lbuf,".");}}
            scrDspText(buf,jsbuf," localIp ",0,0);scrGetText(buf,jsbuf,lbuf,"ethcfg___i",11,LBUFL,0,BRYES);strcat(buf,"\n");
            scrDspText(buf,jsbuf," serverPort ",0,0);scrGetNum(buf,jsbuf,'d',serverPort,"ethcfg___p",5,0,0,0);
            scrDspText(buf,jsbuf," remotePort ",0,0);scrGetNum(buf,jsbuf,'d',remotePort,"ethcfg___t",5,0,0,0);
            scrDspText(buf,jsbuf," serverUdpPort ",0,0);scrGetNum(buf,jsbuf,'d',serverUdpPort,"ethcfg___u",5,0,0,BRYES);strcat(buf,"\n");
            scrDspText(buf,jsbuf,"peripass ",0,0);scrGetText(buf,jsbuf,peripass,"peripcfg__",5,LPWD,0,0);
            scrDspText(buf,jsbuf," TO sans cx tcp ",0,0);scrGetNum(buf,jsbuf,'l',maxCxWt,"ethcfg___q",8,0,0,0);
            scrDspText(buf,jsbuf," TO sans cx udp ",0,0);scrGetNum(buf,jsbuf,'l',maxCxWu,"ethcfg___r",8,0,0,BRYES);strcat(buf,"\n");
            //scrDspText(buf,jsbuf,"",0,BRYES);

            subcfgtable(buf,jsbuf,"USERNAME",NBUSR,"usrname__",usrnames,LENUSRNAME,1,"usrpass__",usrpass,LENUSRPASS,"password",1);
            scrDspText(buf,jsbuf," to password ",0,0);scrGetNum(buf,jsbuf,'d',toPassword,"to_passwd_",6,0,0,BRYES);strcat(buf,"\n");
            ethWrite(cli,buf,&lb);

            subcfgtable(buf,jsbuf,"SSID",MAXSSID,"ssid_____",ssid,LENSSID,1,"passssid_",passssid,LPWSSID,"password",1);
            scrDspText(buf,jsbuf,"ssid1 ",0,0);scrGetNum(buf,jsbuf,'b',ssid1,"ethcfg___W",1,1,0,0,0);
            scrDspText(buf,jsbuf," ssid2 ",0,0);scrGetNum(buf,jsbuf,'b',ssid2,"ethcfg___w",1,1,0,0,BRYES);strcat(buf,"\n");
            formEnd(buf,jsbuf,0,0);
            ethWrite(cli,buf,&lb);

            concPerParams(cli,buf,jsbuf,&lb,lb0);
            ethWrite(cli,buf,&lb);            
            
            mailCfg(cli,buf,jsbuf,&lb,lb0);
            ethWrite(cli,buf,&lb);

fontEnd(buf,jsbuf,0);
            htmlEnd(buf,jsbuf);
            
            ethWrite(cli,buf);

            bufLenShow(buf,jsbuf,lb,begTPage);
}

void cfgDetServHtml(EthernetClient* cli)
{
  
Serial.print("-> config detServ ");

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();                  // calcul durée envoi page
  uint8_t ni=0;

  htmlBeg(buf,jsbuf,serverName);                     // chargement CSS etc

  formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);       // params pour retours navigateur (n° usr + time usr + pericur)

  pageLineOne(buf,jsbuf);                            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
  scrGetButSub(buf,jsbuf,"MàJ",0);

  ethWrite(cli,buf,&lb);       
// ------------------------------------------------------------- header end

/* table libellés */
  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf,"   |      Nom      |",0,TRBEG|TDBEG|TREND);

  for(uint8_t nb=0;nb<NBDSRV;nb++){
    ni++;               
    uint8_t decal=0;if(nb>=16){decal=16;}
    scrDspNum(buf,jsbuf,'s',&nb,0,0,TRBEG|TDBE);
    char nf[LENNOM+2]="libdsrv__ ";nf[LENNOM]=(nb+decal+PMFNCHAR);nf[LENNOM+1]=0x00;
    scrGetText(buf,jsbuf,&libDetServ[nb][0],nf,14,LENLIBDETSERV-1,0,TDBE|TREND);
    //strcat(buf,"<tr><td>");concatns(buf,nb);strcat(buf,"</td><td><input type=\"text\" name=\"libdsrv__");concat1a(buf,(char)(nb+decal+PMFNCHAR));
    //            strcat(buf,"\" value=\"");strcat(buf,(char*)&libDetServ[nb][0]);strcat(buf,"\" size=\"12\" maxlength=\"");concatns(buf,(LENLIBDETSERV-1));
    //            strcat(buf,"\" ></td></tr>\n");
    lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
  }
  tableEnd(buf,jsbuf,BRYES);
  formEnd(buf,jsbuf,0,0);
  htmlEnd(buf,jsbuf);

  ethWrite(cli,buf,&lb);

  bufLenShow(buf,jsbuf,lb,begTPage);
}


void cfgRemoteHtml(EthernetClient* cli)
{
  Serial.print(" config remote ");
            
  char jsbuf[8000];*jsbuf=LF;*(jsbuf+1)=0x00;
  uint16_t lb=0,lb0=8000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();              // calcul durée envoi page
  char nf[LENNOM+1];nf[LENNOM]='\0';
  uint8_t val;
 
  htmlBeg(buf,jsbuf,serverName);                 // chargement CSS etc

  formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);   // params pour retours navigateur (n° usr + time usr + pericur)

  pageLineOne(buf,jsbuf);                        // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
  
  ethWrite(cli,buf,&lb);          
// ------------------------------------------------------------- header end 

  scrGetButSub(buf,jsbuf,"MàJ",BRYES);

/* table remotes */

              strcat(buf,"\n");
              tableBeg(buf,jsbuf,0);
              scrDspText(buf,jsbuf,"   |      Nom      | on/off | en ",0,TDBE|TRBE);

              for(int nb=0;nb<NBREMOTE;nb++){
                uint8_t nb1=nb+1;
                scrDspNum(buf,jsbuf,'s',&nb1,0,0,TRBEG|TDBE);                                   // n° remote
                memcpy(nf,"remotecfn_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);
                scrGetText(buf,jsbuf,remoteN[nb].nam,nf,14,LENREMNAM+1,0,TDBE);
                //strcat(buf,"<td><input type=\"text\" name=\"remotecfn");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
                //        strcat(buf,remoteN[nb].nam);strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,LENREMNAM-1);strcat(buf,"\" ></td>");

                memcpy(nf,"remotecfo_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);             // état on/off
                val=(uint8_t)remoteN[nb].onoff;
                scrGetCheckbox(buf,jsbuf,&val,nf,NO_STATE,"",0,TDBE);
                
                memcpy(nf,"remotecfe_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);             // enable (inutilisé ?)
                val=(uint8_t)remoteN[nb].enable;
                //scrGetCheckbox(buf,jsbuf,&val,nf,NO_STATE,"",TDBE);
                
                scrGetRadiobut(buf,jsbuf,val,nf,3,0,TDBE|TREND);
                strcat(buf,"\n");
 
                if(nb-nb/5*5==0){ethWrite(cli,buf);}
              }
            tableEnd(buf,jsbuf,0);
            formEnd(buf,jsbuf,0,0);

            ethWrite(cli,buf,&lb);

/* table détecteurs */

            borderparam=NOBORDER;
            tableBeg(buf,jsbuf,0);
            scrDspText(buf,jsbuf,"  |remote |detec on/off|detec en|peri|switch",0,TRBE|TDBE);
            strcat(buf,"\n");
              
            for(uint8_t nb=0;nb<MAXREMLI;nb++){
                
              formIntro(buf,jsbuf,nullptr,0,nullptr,0,TRBEG);
              uint8_t nb1=nb+1;
              scrDspNum(buf,jsbuf,'s',&nb1,0,0,TDBE);                                          // n° ligne de table

              memcpy(nf,"remotecfu_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° remote
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].num,nf,2,0,0,TDBEG);
              char ttsp[]={' ',0x00};
              char* tt=remoteN[remoteT[nb].num-1].nam;if(remoteT[nb].num==0){tt=ttsp;}
              scrDspText(buf,jsbuf,tt,0,TDEND);

              memcpy(nf,"remotecfd_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° detec on/off
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].detec,nf,2,0,0,TDBEG);
              #define DML PERINAMLEN+LENLIBDETSERV+1
              char dm[DML];memset(dm,0x00,DML);
              if(remoteT[nb].num!=0){
                strcat(dm,(char*)(&libDetServ[remoteT[nb].detec][0]));strcat(dm," ");
                concat1a(dm,(char)(((memDetServ>>remoteT[nb].detec)&0x01)+48));}
              scrDspText(buf,jsbuf,dm,0,TDEND);
              strcat(buf,"\n");

              memcpy(nf,"remotecfb_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° detec enable
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].deten,nf,2,0,2,TDBEG);
              memset(dm,0x00,DML);
              if(remoteT[nb].num!=0){
                strcat(dm,(char*)(&libDetServ[remoteT[nb].deten][0]));strcat(dm," ");
                concat1a(dm,(char)(((memDetServ>>remoteT[nb].deten)&0x01)+48));}
              scrDspText(buf,jsbuf,dm,0,TDEND);
              strcat(buf,"\n");

              memcpy(nf,"remotecfp_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° périphérique
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].peri,nf,2,0,2,TDBEG);
              memset(dm,0x00,DML);
              uint8_t rp=remoteT[nb].peri;
              if(rp!=0){periLoad(rp);periCur=rp;strcat(dm,periNamer);strcat(dm," ");}
              scrDspText(buf,jsbuf,dm,0,TDEND);
              strcat(buf,"\n");

              memcpy(nf,"remotecfs_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° switch
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].sw,nf,2,0,2,TDBE);
              strcat(buf,"\n");

              scrGetButSub(buf,jsbuf,"MàJ",TDBE|TREND);
              formEnd(buf,jsbuf,0,0);
              strcat(buf,"\n");
                
              if(nb-nb/5*5==0){ethWrite(cli,buf,&lb);}
            }
              
            tableEnd(buf,jsbuf,0);
            htmlEnd(buf,jsbuf);

            ethWrite(cli,buf,&lb);

            bufLenShow(buf,jsbuf,lb,begTPage);
}


void remoteHtml(EthernetClient* cli)
{              
            Serial.print(millis());Serial.print(" remoteHtml() ");

            uint16_t lb0=8000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;
            uint8_t ni=0;                                     // nbre lignes dans buffer
            uint16_t lb;
            unsigned long begTPage=millis();                  // calcul durée envoi page
 
            htmlBeg(buf,jsbuf,serverName,'R');

            pageLineOne(buf,jsbuf);
            formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);
            //usrFormBHtml(buf,1);
            scrGetButRet(buf,jsbuf,"retour",0);
            ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end

/* table remotes */
           
            tableBeg(buf,jsbuf,courier,true,0); 
            strcat(buf,"\n");            

            for(uint8_t nb=0;nb<NBREMOTE;nb++){
              ni++;
              if(remoteN[nb].nam[0]!='\0'){
                bool remHere=false;
                uint8_t nb1=nb+1;
                scrDspNum(buf,jsbuf,'s',&nb1,0,0,TRBEG|TDBE);                                         

                // affichage état d'un éventuel switch
                // boucle des détecteurs pour trouver un switch 
                // (voir le commentaire des disjoncteurs, c'est idem)               
                
                for(uint8_t td=0;td<MAXREMLI;td++){
                  if(remoteT[td].num==nb+1){ 
                    if(remoteT[td].peri!=0){           // même remote et présence périphérique
                      remHere=true;
                      periCur=remoteT[td].peri;periLoad(periCur);
                      if(periSwLev(remoteT[td].sw)==1){                         // switch ON
                        scrDspText(buf,jsbuf," ON ",0,TDBEG|BRYES);
                        affRondJaune(buf,jsbuf,TDEND);
                      }
                      else {
                        scrDspText(buf,jsbuf," OFF ",0,TDBE);}  //strcat(buf," OFF ");}
                    }
                    else {scrDspText(buf,jsbuf,".",0,TDBE);}
                  }
                }
                if(remHere==false){scrDspText(buf,jsbuf," --- ",0,TDBE);} // comble la colonne ON/OFF
                scrDspText(buf,jsbuf,remoteN[nb].nam,7,TDBE);
                // slider on/off
                // chaque ligne de slider envoie 2 commandes : remote_cnx et remote_ctx (x n° de ligne)
                // l'input hidden remote_cnx assure la présence d'une fonction dans 'GET /' pour assurer l'effacement des cb
                // l'input remote_ctx renseigne le passage à "1" éventuel après l'effacement. La variable newonoff stocke le résultat
                // et la comparaison avec onoff permet de connaitre la transition (ou non transition)
                // periRemoteUpdate détecte les transitions, positionne les détecteurs et déclenche poolperif si nécessaire 
                // pour la maj via PerToSend des périphériques concernés
                char fn[]="remote_cn_\0";fn[LENNOM-1]=(char)(nb+PMFNCHAR);
                scrGetHidden(buf,jsbuf,"",fn,0,0);
                sliderBHtml(buf,jsbuf,(uint8_t*)(&remoteN[nb].onoff),"remote_ct",nb,0,1);     // slider

                scrDspText(buf,jsbuf,"- - - - -",0,TDBE);                                // ne devrait pas afficher le disjoncteur si une ligne précédente l'a déjà affiché
                bool vert=FAUX;                                                       // pour ce perif/sw (créer une table fugitive des disj déjà affichés ?)
                yscrGetRadiobut(buf,jsbuf,remoteN[nb].enable,"remote_cs",2,vert,nb,TDBE|TREND);     // renvoie 0,1,2 selon OFF,ON,FOR
                
                lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}               
             }
            }
            if(buf[0]!='\0'){ethWrite(cli,buf);}
            tableEnd(buf,jsbuf,0);
            
            scrGetButSub(buf,jsbuf,"MàJ",ALICNO,7,BRYES); //<input type=\"submit\" value=\"MàJ\" style=\"height:120px;width:400px;background-color:LightYellow;font-size:60px;font-family:Courier,sans-serif;\"><br>\n");
            scrGetButFn(buf,jsbuf,"thermoshow","","températures",ALICNO,7,0);
            scrGetButFn(buf,jsbuf,"remotehtml","","refresh",ALICNO,7,0);
            
            formEnd(buf,jsbuf,0,0);
            htmlEnd(buf,jsbuf);
            ethWrite(cli,buf,&lb);
            
            bufLenShow(buf,jsbuf,lb,begTPage);
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
        if(save){periSave(np_,PERISAVELOCAL);nbth++;
          Serial.print(*periThmin_);Serial.print(" - ");Serial.println(*periThmax_);
        }
//        Serial.println();
      }
    }
  }
  
  periTableSave();
  
  Serial.print("--- fin balayage ");Serial.print(nbli);Serial.print(" lignes ; ");Serial.print(nbth);
  Serial.print(" màj ; millis=");Serial.print(millis()-t1);Serial.print(" total=");Serial.println(millis()-t0);
#ifdef DEBUG_ON
  delay(1);
#endif
  //fhisto.seek(end);
  return 1;
  //return sdOpen("fdhisto.txt",&fhisto);
}

void thermoShowHtml(EthernetClient* cli)
{
  Serial.print(" show thermos ");

  char jsbuf[8000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=8000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();     // calcul durée envoi page
  uint8_t ni=0;
  #define LLITH 200
  char lith[LLITH];

  htmlBeg(buf,jsbuf,serverName);     // chargement CSS etc

  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit
  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    

  ethWrite(cli,buf,&lb);            // tfr -> navigateur
 // ------------------------------------------------------------- header end 
  scalcTh(1);          // update periphériques

/* peritable températures */

  tableBeg(buf,jsbuf,courier,BORDER,BRYES|TRBEG);
  scrDspText(buf,jsbuf,"peri||TH|min|max|last in",0,TDBE|TREND);
  strcat(buf,"\n");
              for(int nuth=0;nuth<NBTHERMOS;nuth++){
                int16_t nuper=thermos[nuth].peri;
                if(nuper!=0){
                  periInitVar();periCur=nuper;periLoad(periCur);

                  if(periMacr[0]!=0x00){
                    ni++;
                    //memset(lith,0x00,LLITH);
                    //char* li=lith;
                    float th;

                    scrDspNum(buf,jsbuf,'I',&periCur,0,0,TRBEG|TDBE);scrDspText(buf,jsbuf,thermos[nuth].nom,7,TDBE);
                    th=(float)(*periLastVal_+*periThOffset_)/100;scrDspNum(buf,jsbuf,'f',&th,2,7,TDBE);
                    th=(float)(*periThmin_)/100;scrDspNum(buf,jsbuf,'f',&th,2,5,TDBE);                    
                    th=(float)(*periThmax_)/100;scrDspNum(buf,jsbuf,'f',&th,2,5,TDBE);                    
                    
                    memset(lith,0x00,LLITH);
                    bufPrintPeriDate(lith,periLastDateIn);
                    scrDspText(buf,jsbuf,lith,2,TDBE|TREND);
                    strcat(buf,"\n");                      
                    
                    lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
                  }
                }
              }
          tableEnd(buf,jsbuf,BRYES);

        //strcat(buf,"<p align=\"center\">");
        scrGetButFn(buf,jsbuf,"remotehtml","","remote",ALIC,7,0);                
        scrGetButFn(buf,jsbuf,"thermoshow","","refresh",ALIC,7,BRYES);
        
        //strcat(buf,"</p><br>");
        memset(lith,0x00,LLITH);
        for(int d=0;d<NBDSRV;d++){concat1a(lith,(char)(((memDetServ>>d)&0x01)+48));strcat(lith," ");}
        scrDspText(buf,jsbuf,lith,0,BRYES);
        htmlEnd(buf,jsbuf);
        strcat(buf,"\n");
        
        ethWrite(cli,buf);

bufLenShow(buf,jsbuf,lb,begTPage);
}

void subthd(char* buf,char* jsbuf,char param,uint8_t nb,void* val,char type)
{
        char nf[LENNOM+1];
        memcpy(nf,"thparams__\0",LENNOM+1);
        nf[LENNOM-2]=param;
        nf[LENNOM-1]=nb+PMFNCHAR;                              
        
        switch(type){
          case 'd': scrGetNum(buf,jsbuf,'b',val,nf,0,2,0,0,TDBE);break;                             // n° detec/peri              
          case 'v': scrGetNum(buf,jsbuf,'I',val,nf,0,4,2,1,TDBE);break;                             // value/offset
          case 'e': scrGetCheckbox(buf,jsbuf,(uint8_t*)val,nf,NO_STATE,"",0,TDBE);break;    // enable/state
          default:break;
        }
}

void subthc(char* buf,char* jsbuf,uint8_t vv)
{
    if(vv!=0){scrDspText(buf,jsbuf,(char*)(&libDetServ[vv][0]),2,STRING|TDBEG);
      scrDspText(buf,jsbuf,":",2,STRING|CONCAT);
      char oi[]={'0',0x00,'1',0x00};
      scrDspText(buf,jsbuf,&oi[((memDetServ>>vv)&0x01)*2],2,STRING|CONCAT);}
    else scrDspText(buf,jsbuf," ",0,TDBE);
}

void thermoCfgHtml(EthernetClient* cli)
{ 
  Serial.print(" config thermos ");
            
  char jsbuf[12000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=12000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();              // calcul durée envoi page
  uint8_t ni=0;

  htmlBeg(buf,jsbuf,serverName);                 // chargement CSS etc

  //formIntro(buf,jsbuf,nullptr,0,nullptr,0,0); // params pour retours navigateur (n° usr + time usr + pericur)
                                                // à charger au moins une fois par page 
  pageLineOne(buf,jsbuf);                        // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
          
  scrGetButFn(buf,jsbuf,"thermoscfg","","refresh",0,0,BRYES);
  
  ethWrite(cli,buf,&lb);          
// ------------------------------------------------------------- header end 

/* table thermos */

  strcat(buf,"\n");
  tableBeg(buf,jsbuf,courier,true,0);
  strcat(buf,"\n");
  tdSet(jsbuf,0,courier,12,0);
  scrDspText(buf,jsbuf,"   |      Nom      | peri |  | low~ en| low~ state| low~ value| low~ pitch| low~ det| | high~ en| high~ state| high~ value| high~ offset| high~ det| | ",0,TDBE|TRBE);
  tdReset();
  strcat(buf,"\n");

  for(uint8_t nb=0;nb<NBTHERMOS;nb++){
    ni++;
    
    formIntro(buf,jsbuf,nullptr,0,nullptr,2,TRBEG);                
    uint8_t nb1=nb+1;
    scrDspNum(buf,jsbuf,'s',&nb1,0,2,TDBE);                                              // n° thermo
    char nf[LENNOM+1];memset(nf,0x00,LENNOM+1);
    memcpy(nf,"thparamsn",LENNOM-1);nf[LENNOM-1]=(char)(nb+PMFNCHAR);
    scrGetText(buf,jsbuf,thermos[nb].nom,nf,LENTHNAME-1,0,TDBE);                 // nom
    //strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,LENTHNAME-1);
    
    subthd(buf,jsbuf,'p',nb,&thermos[nb].peri,'d');                                   // peri
    
    periInitVar();periCur=thermos[nb].peri;                                           // nom;lastVal    
    if(periCur!=0){
      periLoad(periCur);
      scrDspText(buf,jsbuf,periNamer,2,TDBEG);
      scrDspText(buf,jsbuf,":",2,0);
      float pl=(*periLastVal_)/100;
      scrDspNum(buf,jsbuf,'f',&pl,2,2,TDEND);}
    else {scrDspText(buf,jsbuf,"",0,TDBE);}
    
    subthd(buf,jsbuf,'e',nb,&thermos[nb].lowenable,'e');                              // low enable
    subthd(buf,jsbuf,'s',nb,&thermos[nb].lowstate,'e');                               // low state
    subthd(buf,jsbuf,'v',nb,&thermos[nb].lowvalue,'v');                               // low value
    subthd(buf,jsbuf,'o',nb,&thermos[nb].lowoffset,'v');                              // low offset
    subthd(buf,jsbuf,'d',nb,&thermos[nb].lowdetec,'d');                               // low det
    
    subthc(buf,jsbuf,(uint8_t)thermos[nb].lowdetec);

    subthd(buf,jsbuf,'E',nb,&thermos[nb].highenable,'e');                             // high enable
    subthd(buf,jsbuf,'S',nb,&thermos[nb].highstate,'e');                              // high state
    subthd(buf,jsbuf,'V',nb,&thermos[nb].highvalue,'v');                              // high value
    subthd(buf,jsbuf,'O',nb,&thermos[nb].highoffset,'v');                             // high offset
    subthd(buf,jsbuf,'D',nb,&thermos[nb].highdetec,'d');                              // high det

    subthc(buf,jsbuf,(uint8_t)thermos[nb].highdetec);
    
    scrGetButSub(buf,jsbuf,"MàJ",TDBE);
    formEnd(buf,jsbuf,0,TREND);
                
    uint16_t lb1=strlen(buf);if(lb0-lb1<(lb1/ni+100)){ethWrite(cli,buf,&lb);ni=0;}
  }
  tableEnd(buf,jsbuf,0);
  htmlEnd(buf,jsbuf);
  
  strcat(buf,"\n");
  
  ethWrite(cli,buf);

  bufLenShow(buf,jsbuf,lb,begTPage);
}


void timersHtml(EthernetClient* cli)
{
    
Serial.print(" config timers ");
#ifdef DEBUG_ON
  delay(100);
#endif
  char jsbuf[8000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=8000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();    // calcul durée envoi page
  int nucb;

  htmlBeg(buf,jsbuf,serverName);       // chargement CSS etc

  formIntro(buf,jsbuf,0,0);           // params pour retours navigateur (n° usr + time usr + pericur)
                                      // à charger au moins une fois par page ; pour les autres formulaires 
                                      // de la page formBeg() suffit

  pageLineOne(buf,jsbuf);              // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
  scrGetButFn(buf,jsbuf,"timershtml","","refresh",ALICNO,2,0);

  ethWrite(cli,buf,&lb);              // tfr -> navigateur
// ------------------------------------------------------------- header end 

  detServHtml(cli,buf,jsbuf,&lb,lb0,&memDetServ,&libDetServ[0][0]);

  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf,"|nom|det|h_beg|h_end|OI det|e_p_c_|7_d_ l_m_m_ j_v_s|dh_beg_cycle|dh_end_cycle",0,TRBE|TDBE);

  for(uint8_t nt=0;nt<NBTIMERS;nt++){
    formIntro(buf,jsbuf,0,TRBEG|TDBEG);
                      /*strcat(buf,"<form method=\"GET \">");
                      usrFormBHtml(buf,1);
                      strcat(buf,"<td>");*/
    scrDspNum(buf,jsbuf,'s',&(++nt),0,0,TDEND);nt--;
                     
    sscfgtB(buf,jsbuf,"tim_name_",nt,timersN[nt].nom,LENTIMNAM,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_det__",nt,&timersN[nt].detec,2,3,TDBE);                                            
    sscfgtB(buf,jsbuf,"tim_hdf_d",nt,timersN[nt].hdeb,6,0,TDBE);                                            
    sscfgtB(buf,jsbuf,"tim_hdf_f",nt,timersN[nt].hfin,6,0,TDBE);                   
                    
    char oo[7];memset(oo,'_',6);oo[6]=0x00;
    char oi[]="OI";  
    
    oo[1]=oi[timersN[nt].curstate];
    oo[4]=(PMFNCVAL)+((memDetServ>>timersN[nt].detec)&0x01);                     
    scrDspText(buf,jsbuf,oo,0,TDBE);
    nucb=0;sscb(buf,jsbuf,timersN[nt].enable,"tim_chkb__",nucb,NO_STATE,TDBEG,nt);affSpace(buf,jsbuf);
    nucb++;sscb(buf,jsbuf,timersN[nt].perm,"tim_chkb__",nucb,NO_STATE,0,nt);affSpace(buf,jsbuf);
    nucb++;sscb(buf,jsbuf,timersN[nt].cyclic,"tim_chkb__",nucb,NO_STATE,TDEND,nt);   

    ethWrite(cli,buf,&lb);

    uint8_t lctl=TDBEG;
    for(int nj=7;nj>=0;nj--){
      bool vnj; 
      vnj=(timersN[nt].dw>>nj)&0x01;
      nucb++;sscb(buf,jsbuf,vnj,"tim_chkb__",nucb,NO_STATE,lctl,nt);
      lctl=0;if(nj==1){lctl=TDEND;}
      if(nj>=1){affSpace(buf,jsbuf);}
    }
                    
    sscfgtB(buf,jsbuf,"tim_hdf_b",nt,&timersN[nt].dhdebcycle,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_e",nt,&timersN[nt].dhfincycle,14,0,TDBE); 
    scrGetButSub(buf,jsbuf,"MàJ",TDBE);

    formEnd(buf,jsbuf,0,TDEND|TREND);
    strcat(buf,"\n");

    ethWrite(cli,buf,&lb);
  }

  tableEnd(buf,jsbuf,0);
  htmlEnd(buf,jsbuf);
  
  ethWrite(cli,buf,&lb);

bufLenShow(buf,jsbuf,lb,begTPage);
}


void detServHtml(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0,uint32_t* mds,char* lib)
{
  if(buf!=nullptr && lb0!=0){

    Serial.println("detServHtml ");

    formIntro(buf,jsbuf,"dsrv_init_",0,0);         // params pour retours navigateur (n° usr + time usr + pericur + locfonc pour inits)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit
  
          scrDspText(buf,jsbuf,"<fieldset><legend>détecteurs serveur (n->0):</legend>",0,0);

          uint8_t ni=0;
          uint16_t lb1=0;

          for(int k=NBDSRV-1;k>=0;k--){
            ni++;
            char libb[LENLIBDETSERV];memcpy(libb,lib+k*LENLIBDETSERV,LENLIBDETSERV);
            if(libb[0]=='\0'){convIntToString(libb,k);}
            subDSnB(buf,jsbuf,"mem_dsrv__\0",*mds,k,libb);

            fontBeg(buf,jsbuf,1,0);
            scrDspText(buf,jsbuf,"(",0,0);
            scrDspText(buf,jsbuf,&(mdsSrc[sourceDetServ[k]/256]),0,0);
            if(sourceDetServ[k]/256!=0){
              concatn(buf,jsbuf,sourceDetServ[k]&0x00ff);
              scrDspText(buf,jsbuf,") ",0,0);}
            else{scrDspText(buf,jsbuf,"--) ",0,0);}
            fontEnd(buf,jsbuf,0);
            lb1=strlen(buf);if(lb0-lb1<(lb1/ni+100)){ethWrite(cli,buf);ni=0;}
          }
          scrGetButSub(buf,jsbuf,"Per Update",0);
          
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
