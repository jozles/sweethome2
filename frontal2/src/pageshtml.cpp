#include <Arduino.h>
#include <ds3231.h>
#include "const.h"
#include <shconst2.h>
#include <SdFat.h>
#include <shutil2.h>
#include "utilether.h"
#include "utilhtml.h"
#include "periph.h"
#include "pageshtml.h"
#include "utiljs.h"

extern bool oneIcon;

extern Ds3231 ds3231;
extern char now[16];
extern unsigned long unixNow;

extern char*      serverName;
extern byte*      mac;              // adresse server
extern byte*      localIp;
extern uint16_t*  perifPort;
extern uint16_t*  browserPort;
extern uint16_t*  remotePort;
extern uint16_t   udpPort[2];

extern char*      peripass;         // mot de passe périphériques
extern unsigned long* maxCxWt;
extern unsigned long* maxCxWu;
extern uint8_t*   openSockScan;
extern uint8_t*   openSockTo;

extern char*      mailFromAddr; 
extern char*      mailPass;     
extern char*      mailToAddr1;  
extern char*      mailToAddr2;  
extern uint16_t*  periMail1;    
extern uint16_t*  periMail2;

extern char*      thermoPrev;
extern char*      thermoLastScan;       // date/heure last scan
extern char*      thermoLastPrev;       // antériorité last scan
extern char*      thermoLastBeg;        // date/heure histo

extern char       thermoCurPrev[16];

extern char*      chexa;
extern byte       maskbit[];

extern int        periCur;          // Numéro du périphérique courant

extern char       periCache[PERIRECLEN*(NBPERIF+1)];

extern char*      periBegOfRecord;              // ptr début buffer
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
extern byte*      periSwCde;                    // ptr ds buffer : cde des switchs
extern char*      periNamer;                    // ptr ds buffer : description périphérique
extern uint8_t*   periSwSta;                    // ptr ds buffer : état des switchs 

extern byte       periMacBuf[MACADDRLENGTH]; 

//extern File32 fhisto;            // fichier histo sd card
//extern SdFile fhisto;            // fichier histo sd card
extern FsFile fhisto;            // fichier histo sd card
//File32 fimg;              // fichier image
//SdFile fimg;              // fichier image
FsFile fimg;              // fichier image
//File32 fhtml;             // fichiers pages html
//SdFile fhtml;             // fichiers pages html
FsFile fhtml;             // fichiers pages html

extern uint16_t   perrefr;
extern long       fhsize;           // remplissage fhisto
extern uint32_t   histoPos;
extern uint16_t   histoPeri;
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

extern struct SwRemote remoteT[MAXREMLI];
extern struct Remote remoteN[NBREMOTE];

extern struct Timers timersN[NBTIMERS];

extern struct AnalTimers analTimers[NBANTIMERS];

extern struct Thermo thermos[NBTHERMOS];

extern int        fdatasave;

extern char       mdsSrc[];
extern uint8_t    memDetServ[];                // image mémoire NBDSRV détecteurs
extern uint8_t    mDSmaskbit[];
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

const char* htmlIconBeg="<link rel=\"icon\" href=\"image/png;base64,"; // "<html>\n <html> <link rel=\"icon\" href=\"data:image/png;base64,";
const char* htmlIconEnd="\">"; //</html>";

int htmlImg(EthernetClient* cli,const char* fimgname)   
{
        uint16_t htmlIconBegLen=strlen(htmlIconBeg);
        uint16_t htmlIconEndLen=strlen(htmlIconEnd);
        unsigned long begIC=millis();
        Serial.print(' ');Serial.print(fimgname);

        if(sdOpen(fimgname,&fimg)==SDKO){return SDKO;}
        else {
          uint32_t fimgSiz=fimg.size();
          Serial.print(" size=");Serial.print(fimgSiz);
          #define ICONMAXLENGTH 2000
          uint32_t iconSiz=fimgSiz*3+htmlIconBegLen+htmlIconEndLen+1;
          if(iconSiz>ICONMAXLENGTH){Serial.println(" fichier icon trop grand *********");}
          else {
            char icon[iconSiz];
            memcpy(icon,htmlIconBeg,htmlIconBegLen);
            uint32_t limg;
            char img[fimgSiz+1];
            for(limg=0;limg<fimgSiz;limg++){img[limg]=fimg.read();}
            uint16_t len64=ato64(img,icon+htmlIconBegLen,fimgSiz);
            memcpy(icon+htmlIconBegLen+len64,htmlIconEnd,htmlIconEndLen);
            limg=htmlIconBegLen+len64+htmlIconEndLen;
            *(icon+limg)='\0';
            
            Serial.print(" limg:");Serial.println(limg);Serial.println(icon);
            
            ethWrite(cli,icon,limg);
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
    //htmlImg(cli,"sweeth.png");
    //oneIcon=true;
  }
}

void dumpHisto0(EthernetClient *cli,char* buf,char*jsbuf,long pos,uint16_t lb0,uint16_t* lb)   // liste le fichier histo depuis une adresse
{
  scrDspNum(buf,jsbuf,'d',&histoPeri,0,0,0);affSpace(buf,jsbuf);
  scrDspNum(buf,jsbuf,'l',&pos,0,0,0);scrDspText(buf,jsbuf,"/",0,STRING|CONCAT|TDBEG);scrDspNum(buf,jsbuf,'l',&fhsize,0,0,BRYES);
  strcat(buf,"\n");

  ethWrite(cli,buf,lb);

  #define LBLEN 1000
  char lineBuf[LBLEN];
  uint32_t linePeri=0;
  char* linePeriPtr;
  uint16_t linePtr=0;
  long  totalRead=0;

  fhisto.seek(pos);

  while((pos+totalRead)<fhsize && totalRead<1000000){
    memset(lineBuf,'\0',LBLEN);
    while(lineBuf[linePtr-1]!='\n' && linePtr<LBLEN){
      trigwd();
      lineBuf[linePtr]=fhisto.read();
      linePtr++;totalRead++;
    }
    
    linePeriPtr=strstr(lineBuf,";")+9;
    #define LINELMINI 8+1+6+1+1+1+11+7+9+11+18+2 // à améliorer pour caractériser une ligne dataxxxxxxx    
    if(histoPeri!=0 && linePtr>(linePeriPtr-lineBuf+LINELMINI)){     
      conv_atobl(linePeriPtr,&linePeri);
    }
    if(linePeri==histoPeri){
      ethWrite(cli,lineBuf,linePtr);  
    }
    linePtr=0;linePeri=0;
  }
}

void getPerif(uint16_t perif,long* pos,long* oldpos)   // pos en début de ligne, retour sur le début de ligne du périf 
{
  long localPos=*pos;
  uint16_t localPerif;
  char a='0';
  #define LBB 6
  char bbuf[LBB];bbuf[LBB-1]='\0';
  bool fini=false;
  long fhs=fhsize-1;
  while(!fini){
    if((*oldpos+10000)<*pos){trigwd();*oldpos+=10000;}
    localPos+=23;fhisto.seek(localPos);
    a=fhisto.read();localPos++;
    if(a=='a' || a=='u'){         // port perif only
      while(a!='\n' && a!=';' && localPos<fhs){a=fhisto.read();localPos++;}
      if(localPos>=fhs){fini=true;*pos=localPos;}
      else if(a=='\n'){*pos=localPos;}
      else {
        a='0';
        while(a!='\n' && a!='_' && localPos<fhs){a=fhisto.read();localPos++;}
        if(localPos>=fhs){fini=true;*pos=localPos;}
        else if(a=='\n'){*pos=localPos;}
        else {
          for(uint8_t bb=0;bb<LBB-2;bb++){bbuf[bb]=fhisto.read();localPos++;}
          bbuf[LBB-2]='!';localPerif=0;conv_atob(bbuf,&localPerif);
          //Serial.print(perif);Serial.print(' ');Serial.print(bbuf);Serial.print(' ');Serial.println(localPerif);
          if(localPerif==perif || perif>NBPERIF){fini=true;}
          else {          // skipline
            a='0';
            while(a!='\n' && localPos<fhs){a=fhisto.read();localPos++;}
            if(localPos>=fhs){fini=true;}
            *pos=localPos;
          }
        }
      }
    }
    else {                // skipline
      a='0';
      while(a!='\n' && localPos<fhs){a=fhisto.read();localPos++;}
      if(localPos>=fhs){fini=true;}
      *pos=localPos;
    }
  }
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

  unsigned long begTPage=millis();     // calcul durée envoi page
  long pos=histoPos;
  long oldpos=pos;

  htmlBeg(buf,jsbuf,serverName);     // chargement CSS etc
  
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);

  ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end
  trigwd();

  if(sdOpen("fdhisto.txt",&fhisto)==SDKO){scrDspText(buf,jsbuf,"fdhisto KO",0,0);return;}
  fhsize=fhisto.size();Serial.print("fhsize=");Serial.print(fhsize);Serial.print(" pos=");Serial.println(pos);

  scrDspText(buf,jsbuf,"histoSD size=",0,0);
  scrDspNum(buf,jsbuf,'l',&fhsize,0,0,0);
  scrDspText(buf,jsbuf," pos=",0,0);
  scrDspNum(buf,jsbuf,'l',&pos,0,0,0);
  scrDspText(buf,jsbuf," date deb",0,0);
  histoDh[9]='\0';
  scrDspText(buf,jsbuf,histoDh,0,0);
  scrDspText(buf,jsbuf," peri=",0,0);
  scrDspNum(buf,jsbuf,'d',&histoPeri,0,0,BRYES);

  if(histoDh[0]=='2'){shDateHist(histoDh,&pos);}
  else {                                  //  positionnement sur début ligne si pas de sélection de date
    fhisto.seek(pos);
    while(pos<fhsize){
      trigwd();
      char a=fhisto.read();pos++;Serial.print(a);
      if(a=='\n'){break;}
    }
  }
  if(histoPeri!=0){getPerif(histoPeri,&pos,&oldpos);} // recherche 1ère ligne avec périf ok
  Serial.print(" pos=");Serial.println(pos);

  ethWrite(cli,buf,&lb);
  
  #define LPP 1000
  char aa[LPP];
  oldpos=pos;
  
  while(pos<fhsize-1){
    fhisto.seek(pos);
    for(uint16_t pp=0;pp<LPP-1;pp++){
      aa[pp]=fhisto.read();pos++;
      if(aa[pp]=='\n'){aa[pp+1]='\0';break;}
    }
    Serial.print(aa);
    scrDspText(buf,jsbuf,aa,0,0);
    ethWrite(cli,buf,&lb);
    if(histoPeri!=0){getPerif(histoPeri,&pos,&oldpos);} // recherche 1ère ligne avec périf ok
    if((oldpos+10000)<pos){trigwd();oldpos+=10000;}
  }
  //dumpHisto0(cli,buf,jsbuf,pos,lb0,&lb);
  fhisto.close();

  scrGetButRet(buf,jsbuf,"retour",1);
  htmlEnd(buf,jsbuf);

  ethWrite(cli,buf,&lb);

  bufLenShow(buf,jsbuf,lb,begTPage);
}

void histoHtml(EthernetClient* cli)
{
  Serial.print(" dump histo ");
#ifdef DEBUG_ON
  delay(100);
#endif
  char jsbuf[16000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf ne fonctionne pas avec dumpHisto0 !!!!!!!!!!!!!!!!!!!!!!!!! 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();     // calcul durée envoi page
  
  htmlBeg(buf,jsbuf,serverName);     // chargement CSS etc
  
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",BRYES);

  ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end
setFont(buf,"Arial","10");
          scrDspText(buf,jsbuf,"ne fonctionne qu'avec un fichier < 2Go !!!! (modifier les variables en int64 et exFat)",0,0,0,BRYES);
          scrDspText(buf,jsbuf,"la version actuelle sélectionne le type de ligne pour les périfs (a,u)",0,0,0,BRYES);
tableBeg(buf,jsbuf,"Arial","10",false,"0",0);          
          scrDspText(buf,jsbuf,"W  Hardware Watchdog",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf,"   ",0,0,0,TDBE);
          scrDspText(buf,jsbuf,"T  TCP watchdog",0,0,0,TDBE|TREND);
          scrDspText(buf,jsbuf,"U  UDP watchdog",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf,"  ",0,0,0,TDBE);          
          scrDspText(buf,jsbuf,"H  Halt request",0,0,0,TDBE|TREND);
          scrDspText(buf,jsbuf,"M  Temp record",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf," ",0,0,0,TDBE);          
          scrDspText(buf,jsbuf,"R  Hard Reset",0,0,0,TDBE|TREND);
          scrDspText(buf,jsbuf,"B  user Request Boot",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf," ",0,0,0,TDBE);          
          scrDspText(buf,jsbuf,"a  TCP périf record",0,0,0,TDBE|TREND);          
          scrDspText(buf,jsbuf,"u  UDP périf record",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf," ",0,0,0,TDBE);          
          scrDspText(buf,jsbuf,"c  TCP browser record",0,0,0,TDBE|TREND); 
          scrDspText(buf,jsbuf,"b  TCP remote record",0,0,0,TRBEG|TDBE);
          scrDspText(buf,jsbuf,"__",0,0,0,TDBE|TREND|BRYES);          
tableEnd(buf,jsbuf,BRYES);                                                 
endFont(buf);
tableBeg(buf,jsbuf,"Arial","18",NOBORDER,"0",0);
          scrDspText(buf,jsbuf,"pos (",0,TRBEG|TDBEG);scrDspNum(buf,jsbuf,'l',&fhsize,0,0,0);scrDspText(buf,jsbuf,") ",0,TDEND);
          scrGetNum(buf,jsbuf,'i',(uint32_t*)&histoPos,"hist_sh_p_",9,0,0,TDBE|TREND);
          scrDspText(buf,jsbuf,"date debut ",0,TRBEG|TDBE);
          scrGetText(buf,jsbuf,histoDh,"hist_sh_D_",LDATEA-2,0,TDBE|TREND);
          affSpace(buf,jsbuf);
          scrDspText(buf,jsbuf,"n° périf ",0,TRBEG|TDBE);
          scrGetNum(buf,jsbuf,'d',&histoPeri,"hist_sh_P_",4,0,0,TDBEG);
          scrDspText(buf,jsbuf," (si >NBPERIF tous perifs ; si 0 tous types)",0,"Arial",12,TDEND|TREND);
tableEnd(buf,jsbuf,BRYES);

  fnHtmlEnd(buf,jsbuf,0,BRYES);
  scrGetButSub(buf,jsbuf,"dump",ALICNO,4,0);

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
  long miniL=50; // ???

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
            
            scrDspText(buf,jsbuf,VERSION,24,BRYES);
            //strcat(buf,"<h1 class=\"point\">");
            //strcat(buf,VERSION);strcat(buf,"<br>");

//htmlImg(cli,"sweeth.png");            

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

void mDSconc(char* concbuf,uint8_t num)
{
  char mdsbit='0';
  //for(uint8_t i=0;i<MDSLEN;i++){if(((memDetServ[i]) & (mDSmaskbit[num*MDSLEN+i])) != 0){mdsbit='1';break;}}
  uint8_t mi=num>>3;if(((memDetServ[mi]) & (mDSmaskbit[num*MDSLEN+mi])) != 0){mdsbit='1';}
  concat1a(concbuf,mdsbit);
}

void sscb(char* buf,char* jsbuf,bool val,const char* nomfonct,int nuf,int etat,uint8_t ctl,uint8_t nb)
{                                                                               // saisie checkbox ; bit nuf
  char nf[LENNOM+1];
  memcpy(nf,nomfonct,LENNOM);
  nf[LENNOM]='\0';
  nf[LENNOM-1]=(char)(nb+PMFNCHAR);
  nf[LENNOM-2]=(char)(nuf+PMFNCHAR);
  scrGetCheckbox(buf,jsbuf,(uint8_t*)&val,nf,etat,"",0,ctl);
}

void sscfgtB(char* buf,char* jsbuf,const char* nom,uint8_t nb,int8_t rg,const void* value,int len,uint8_t dec,uint8_t type,uint8_t size,uint8_t ctl) 
                                                                    // nature valeur à saisir / nature de 'value'
                                                                    // type =0 char*   /    value=char*
                                                                    // type =1 char*   /    ((char*)value+nb*(len+1))=char*
                                                                    // type =2 int16_t /    (int16t*)value+nb
                                                                    // type =3 int8_t  /    value=int8_t*
                                                                    // type =4 float   /    value=float*
                                                                    // type =5 int16_t /    (int16t*)value                                                                    
{
  int8_t sizbx=len-3;
  if(size==0){sizbx=30;}
  else if(sizbx<=0){sizbx=1;}
  char nf[LENNOM+1];memcpy(nf,nom,LENNOM-1);nf[LENNOM-1]=(char)(nb+PMFNCHAR);nf[LENNOM]=0x00;
  if(rg>=0){nf[LENNOM-2]=(char)(rg+PMFNCHAR);}

  //strcat(buf,"<td><input type=\"text\" name=\"");strcat(buf,nom);concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\" value=\"");
  if(type==0){scrGetText(buf,jsbuf,(char*)value,nf,sizbx,len,0,ctl);} //strcat(buf,(char*)value);strcat(buf,"\" size=\"");concatn(buf,sizbx);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>");}
  if(type==1){scrGetText(buf,jsbuf,(char*)((char*)value+(nb*(len+1))),nf,sizbx,len,0,ctl);} //strcat(buf,(char*)(((char*)value+(nb*(len+1)))));strcat(buf,"\" size=\"");concatn(buf,len);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" ></td>\n");}  
  if(type==2){scrGetNum(buf,jsbuf,'I',((int16_t*)value+nb),nf,2,1,0,ctl);} //concatn(buf,*((int16_t*)value+nb));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>");}
  if(type==3){scrGetNum(buf,jsbuf,'s',(int8_t*)value,nf,2,1,0,ctl);} //concatn(buf,*((int8_t*)value));strcat(buf,"\" size=\"1\" maxlength=\"2\" ></td>\n");}
  if(type==4){scrGetNum(buf,jsbuf,'F',(int8_t*)value,nf,len,dec,0,ctl);}
  if(type==5){scrGetNum(buf,jsbuf,'I',(int16_t*)value,nf,len,1,0,ctl);}
}

void sscfgtB(char* buf,char* jsbuf,const char* nom,uint8_t nb,int8_t rg,const void* value,int len,uint8_t dec,uint8_t type,uint8_t ctl)
{
  sscfgtB(buf,jsbuf,nom,nb,rg,value,len,dec,type,0,ctl);
}

void sscfgtB(char* buf,char* jsbuf,const char* nom,uint8_t nb,int8_t rg,const void* value,int len,uint8_t type,uint8_t ctl)
{
  sscfgtB(buf,jsbuf,nom,nb,rg,value,len,2,type,0,ctl);  
}

void sscfgtB(char* buf,char* jsbuf,const char* nom,uint8_t nb,const void* value,int len,uint8_t type,uint8_t ctl)   
{
  sscfgtB(buf,jsbuf,nom,nb,-1,value,len,2,type,0,ctl);
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
    
    tableBeg(buf,jsbuf,courier,"18",NOBORDER,0,0);
    scrDspText(buf,jsbuf,"mailFrom ",0,TRBEG|TDBE);scrGetText(buf,jsbuf,mailFromAddr,"mailcfg__f",16,LMAILADD,0,TDBE);strcat(buf,"\n");
    scrDspText(buf,jsbuf," ",40,0,TDBE);scrDspText(buf,jsbuf,"password  ",0,TDBE);scrGetText(buf,jsbuf,mailPass,"mailcfg__w",16,LMAILPWD,0,TDBE|TREND);strcat(buf,"\n");
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
  scrGetButSub(buf,jsbuf,"MàJ",ALICNO,2,BRYES);

  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end
            
            /*cli->print(" peripass <input type=\"text\" name=\"peripcfg__\" value=\"");cli->print(peripass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");*/

polBeg(buf,jsbuf,12,0);

            scrDspText(buf,jsbuf,"nom serveur ",0,TRBEG|TDBE);scrGetText(buf,jsbuf,serverName,"ethcfg___s",16,LNSERV,0,TDBE);strcat(buf,"\n");
            #define LBUFL 16
            char lbuf[LBUFL];*lbuf=0x00;for(uint8_t k=0;k<MACADDRLENGTH;k++){concat1a(lbuf,chexa[mac[k]/16]);concat1a(lbuf,chexa[mac[k]%16]);}
            scrDspText(buf,jsbuf," serverMac ",0,0);scrGetText(buf,jsbuf,lbuf,"ethcfg___m",11,MACADDRLENGTH*2,0,0);
            *lbuf=0x00;for(int k=0;k<4;k++){concatns(lbuf,localIp[k]);if(k!=3){strcat(lbuf,".");}}
            scrDspText(buf,jsbuf," localIp ",0,0);scrGetText(buf,jsbuf,lbuf,"ethcfg___i",11,LBUFL,0,BRYES);strcat(buf,"\n");
            scrDspText(buf,jsbuf," perifPort ",0,0);scrGetNum(buf,jsbuf,'d',perifPort,"ethcfg___p",5,0,0,0);
            scrDspText(buf,jsbuf," browserPort ",0,0);scrGetNum(buf,jsbuf,'d',browserPort,"ethcfg___y",5,0,0,0);
            scrDspText(buf,jsbuf," remotePort ",0,0);scrGetNum(buf,jsbuf,'d',remotePort,"ethcfg___t",5,0,0,BRYES);
            scrDspText(buf,jsbuf," serverUdp1 Port ",0,0);scrGetNum(buf,jsbuf,'d',&udpPort[0],"ethcfg___u",5,0,0,BRYES);strcat(buf,"\n");
            scrDspText(buf,jsbuf," serverUdp2 Port ",0,0);scrGetNum(buf,jsbuf,'d',&udpPort[1],"ethcfg___U",5,0,0,BRYES);strcat(buf,"\n");
            scrDspText(buf,jsbuf,"peripass ",0,0);scrGetText(buf,jsbuf,peripass,"peripcfg__",5,LPWD,0,0);
            scrDspText(buf,jsbuf," TO sans cx tcp ",0,0);scrGetNum(buf,jsbuf,'l',maxCxWt,"ethcfg___q",8,0,0,0);
            scrDspText(buf,jsbuf," TO sans cx udp ",0,0);scrGetNum(buf,jsbuf,'l',maxCxWu,"ethcfg___r",8,0,0,BRYES);
            scrDspText(buf,jsbuf," Délai scan sockets ",0,0);scrGetNum(buf,jsbuf,'D',openSockScan,"ethcfg___x",8,0,0,0);
            scrDspText(buf,jsbuf," TO open sockets ",0,0);scrGetNum(buf,jsbuf,'D',openSockTo,"ethcfg___z",8,0,0,BRYES);strcat(buf,"\n");            
            //scrDspText(buf,jsbuf,"",0,BRYES);

            subcfgtable(buf,jsbuf,"USERNAME",NBUSR,"usrname__",usrnames,LENUSRNAME,1,"usrpass__",usrpass,LENUSRPASS,"password",1);
            scrDspText(buf,jsbuf," to password ",0,0);scrGetNum(buf,jsbuf,'d',toPassword,"to_passwd_",6,0,0,BRYES);strcat(buf,"\n");
            ethWrite(cli,buf,&lb);

            subcfgtable(buf,jsbuf,"SSID",MAXSSID,"ssid_____",ssid,LENSSID,1,"passssid_",passssid,LPWSSID,"password",1);
            scrDspText(buf,jsbuf,"ssid1 ",0,0);scrGetNum(buf,jsbuf,'b',ssid1,"ethcfg___W",1,1,0,0,0);
            scrDspText(buf,jsbuf," ssid2 ",0,0);scrGetNum(buf,jsbuf,'b',ssid2,"ethcfg___w",1,1,0,0,BRYES);

            scrDspText(buf,jsbuf,"thermos default prev ",0,0);scrGetText(buf,jsbuf,thermoPrev,"ethcfg___v",14,15,0,0);
            scrDspText(buf,jsbuf," thermos last prev ",0,0);scrDspText(buf,jsbuf,thermoLastPrev,0,BRYES);
            scrDspText(buf,jsbuf," thermos last beg (du) ",0,0);scrDspText(buf,jsbuf,thermoLastBeg,0,0);
            scrDspText(buf,jsbuf," (au) thermos last scan ",0,0);scrDspText(buf,jsbuf,thermoLastScan,0,BRYES);strcat(buf,"\n");

            formEnd(buf,jsbuf,0,0);
            ethWrite(cli,buf,&lb);

            concPerParams(cli,buf,jsbuf,&lb,lb0);
            ethWrite(cli,buf,&lb);            
            
            mailCfg(cli,buf,jsbuf,&lb,lb0);
            ethWrite(cli,buf,&lb);

polEnd(buf,jsbuf,0);
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

  char nf[]={"libdsrv___\0"};
  for(uint8_t nb=0;nb<NBDSRV;nb++){
    ni++;               
    scrDspNum(buf,jsbuf,'s',&nb,0,0,TRBEG|TDBE);
    nf[LENNOM-1]=(char)(nb+PMFNCVAL);

    scrGetText(buf,jsbuf,&libDetServ[nb][0],nf,14,LENLIBDETSERV-1,0,TDBE|TREND);
    strcat(buf,"\n");
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

  scrGetButSub(buf,jsbuf,"MàJ",ALICNO,2,BRYES);
  scrDspText(buf,jsbuf,"Les remotes simples sont attachées à un unique périf/switch avec son disjoncteur (décrit dans la 2nde table)",0,BRYES);
  scrDspText(buf,jsbuf,"Les remotes multiples hébergent le disjoncteur qui contrôle tous les switchs associés",0,BRYES);
  scrDspText(buf,jsbuf,"Le push/slider contrôle un bit de MDS dans les 2 cas (aucun si 0)",0,BRYES);
  scrDspText(buf,jsbuf,"Il est transmis à chaque appui (et raz si push)",0,BRYES);

/* table remotes */

              strcat(buf,"\n");
              tableBeg(buf,jsbuf,0);
              scrDspText(buf,jsbuf,"   |      Nom      | mult | detec on/off | disj |slid/push",0,TDBE|TRBE);

              for(int nb=0;nb<NBREMOTE;nb++){
                uint8_t nb1=nb+1;
                scrDspNum(buf,jsbuf,'s',&nb1,0,0,TRBEG|TDBE);                                // n° remote
                memcpy(nf,"remotecfn_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);
                scrGetText(buf,jsbuf,remoteN[nb].nam,nf,14,LENREMNAM+1,0,TDBE);              // nom remote

                memcpy(nf,"remotecfg_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);             // flag remote multiple 
                val=(uint8_t)remoteN[nb].multRem;
                scrGetCheckbox(buf,jsbuf,&val,nf,NO_STATE,"",0,TDBE);

                memcpy(nf,"remotecfh_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);             // n° detec on/off
                scrGetNum(buf,jsbuf,'b',&remoteN[nb].detec,nf,2,0,0,TDBEG);
                #define DNL 2+LENLIBDETSERV+1
                char dn[DNL];memset(dn,0x00,DNL);
                if(remoteN[nb].detec!=0){
                  strcat(dn,(char*)(&libDetServ[remoteN[nb].detec][0]));strcat(dn," ");
                  mDSconc(dn,remoteN[nb].detec);}
                  scrDspText(buf,jsbuf,dn,0,TDEND);
                  strcat(buf,"\n");

                if(remoteN[nb].multRem){
                  memcpy(nf,"remotecfj_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);            // valeur disjoncteur si multiple
                  scrGetNum(buf,jsbuf,'b',&remoteN[nb].enable,nf,2,0,0,TDBE);
                  memset(dn,0x00,DNL);
                }
                else scrDspText(buf,jsbuf," ",0,TDBE);

                strcat(buf,"\n");

                memcpy(nf,"remotecfk_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // modèle bouton
                scrGetRadiobut(buf,jsbuf,remoteN[nb].butModel,nf,2,0,TDBE|TREND);
                strcat(buf,"\n");

                if(nb-nb/5*5==0){ethWrite(cli,buf);}
              }
            tableEnd(buf,jsbuf,0);
            formEnd(buf,jsbuf,0,0);

            ethWrite(cli,buf,&lb);

/* table détecteurs */

            borderparam=NOBORDER;
            tableBeg(buf,jsbuf,0);
            scrDspText(buf,jsbuf,"  |remote | mult |peri|switch",0,TRBE|TDBE);
            strcat(buf,"\n");
              
            for(uint8_t nb=0;nb<MAXREMLI;nb++){
              
              formIntro(buf,jsbuf,nullptr,0,nullptr,0,TRBEG);
              uint8_t nb1=nb+1;
              scrDspNum(buf,jsbuf,'s',&nb1,0,0,TDBE);                                       // n° ligne de table

              memcpy(nf,"remotecfu_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° remote
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].num,nf,2,0,0,TDBEG);
              char ttsp[]={' ',0x00};
              char* tt=remoteN[remoteT[nb].num-1].nam;if(remoteT[nb].num==0){tt=ttsp;}
              scrDspText(buf,jsbuf,tt,0,TDEND);

              memcpy(nf,"remotecfv_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° remote multiple
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].multRem,nf,2,0,0,TDBEG);
              ttsp[0]=' ';
              tt=remoteN[remoteT[nb].multRem-1].nam;if(remoteT[nb].multRem==0){tt=ttsp;}
              scrDspText(buf,jsbuf,tt,0,TDEND);

              #define DML PERINAMLEN+LENLIBDETSERV+1
              char dm[DML];memset(dm,0x00,DML);

              memcpy(nf,"remotecfp_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° périphérique
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].peri,nf,2,0,0,TDBEG);
              memset(dm,0x00,DML);
              uint8_t rp=remoteT[nb].peri;
              if(rp!=0){periLoad(rp);periCur=rp;strcat(dm,periNamer);strcat(dm," ");}
              scrDspText(buf,jsbuf,dm,0,TDEND);
              strcat(buf,"\n");

              memcpy(nf,"remotecfs_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);              // n° switch
              scrGetNum(buf,jsbuf,'b',&remoteT[nb].sw,nf,2,0,0,TDBE);
              strcat(buf,"\n");

              scrGetButSub(buf,jsbuf,"MàJ",ALICNO,0,TDBE|TREND);
              formEnd(buf,jsbuf,0,0);
              strcat(buf,"\n");
                
              if(nb-nb/5*5==0){ethWrite(cli,buf,&lb);}
            }
              
            tableEnd(buf,jsbuf,0);
            htmlEnd(buf,jsbuf);

            ethWrite(cli,buf,&lb);

            bufLenShow(buf,jsbuf,lb,begTPage);
}

void remoteTimHtml(EthernetClient* cli,int16_t rem)
{
            rem++;
            Serial.print(millis());Serial.print(" remoteTimHtml() ");

            uint16_t lb0=8000;
            char buf[lb0];buf[0]='\0';
            char jsbuf[LBUF4000];*jsbuf=0x00;

            uint16_t lb;
            char    fn[LENNOM+1];
 
            htmlBeg(buf,jsbuf,serverName,'R');

            pageLineOne(buf,jsbuf);
            formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);
            //usrFormBHtml(buf,1);
            scrGetButRet(buf,jsbuf,"retour",0);
            scrGetButRef(buf,jsbuf,"remote_ct",rem-1,BRYES);

            ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end
  
  int16_t min=0;
  int16_t max=MAXREMLI;
  //scrDspText(buf,jsbuf,"  ",0,BRYES);
  
  uint8_t disjVal=0;

  char codeFn[6]={'a','b','c','d','e','f'};                                         // pour disjoncteurs OFF/ON/FORCED
  const char* lib[3];lib[0]="OFF";lib[1]="ON";lib[2]="FOR";
  const char* libcd[3];libcd[0]="STOP";libcd[1]="PAUSE";libcd[2]="START";
  uint8_t colors[3]={DISJCOLOR,ONCOLOR,FORCEDCOLOR};                                // pour valeurs 0/1/2 du disjoncteur
  uint8_t color;

  uint8_t ctl=0;

  char    remTNum[]={'\0','\0'};
  memcpy(fn,"remote_o__\0",LENNOM+1);fn[LENNOM-1]=(char)(rem-1+PMFNCHAR);           // transmission n° remote

  // ----------------------- une ligne état initial de la remote hors one_shot_timer

  fnHtmlEnd(buf,jsbuf,0,BRYES);
  scrDspText(buf,jsbuf,"initial state",0,0);
  tableBeg(buf,jsbuf,courier,true,0,0);                                             
  scrDspNum(buf,jsbuf,&rem,&min,&max,true,0,BRYES|TDBE);
  scrDspText(buf,jsbuf,remoteN[rem-1].nam,0,TDBE);
  //strcat(buf,"\n");            

  int16_t nt;
  periCur=0;

  for(nt=0;nt<MAXREMLI;nt++){
    if(remoteT[nt].num==rem && remoteT[nt].peri!=0){
      *remTNum=nt+PMFNCHAR;
      periCur=remoteT[nt].peri;
      periLoad(periCur);
      //strcat(buf,"<td width=45>");                                                    // patch à intégrer dans le ctl des fonctions d'affichage

      scrDspNum(buf,jsbuf,&nt,&min,&max,true,0,TDBE);
      scrDspText(buf,jsbuf,remTNum,0,TDBE);
      if(remoteN[rem-1].osStatus==0){
        disjVal=periSwRead(remoteT[nt].sw);
        remoteN[rem-1].enable=disjVal;
       Serial.print("status=");Serial.print(nt); Serial.print("disjVal======");Serial.println(disjVal);
      }
      else disjVal=(remoteN[rem-1].enable);                                       // disjVal état disj hors os     
      if(((*periSwSta>>remoteT[nt].sw)&0x01)!=0){                                 // switch 'allumé'
            scrDspText(buf,jsbuf," ON ",0,TDBEG);
            affRondJaune(buf,jsbuf,TDEND);
      }
      else {scrDspText(buf,jsbuf," OFF ",0,TDBE);}
      break;
    }
  }
  if(periCur==0){scrDspText(buf,jsbuf,".",0,TDBE);}                               // pas de périf donc pas de switch

  //ctl=TDEND|TREND;
  color=colors[disjVal%10];
  if(disjVal>=10){color+=LIGHTVALUE;}  

  min=0;max=99;
  scrGetButFn(buf,jsbuf,"null_fnct_",remTNum,lib[disjVal%10],ALICNO,1,color,0,0,1,TDBE|TREND);
  tableEnd(buf,jsbuf,BRYES);

  strcat(buf,"\n");            

// ---------------------------- une ligne durées

  fnHtmlEnd(buf,jsbuf,0,BRYES);
  scrDspText(buf,jsbuf,"requested dur",0,0);
  tableBeg(buf,jsbuf,courier,true,0,0);
  scrDspText(buf,jsbuf,"duration(hhmmss)|rem time|end Time",0,TRBE|TDBE);
  fn[LENNOM-2]='t';
  //Serial.print(">==========");Serial.print(fn);Serial.print(' ');Serial.print(remoteN[rem-1].osEndDate);Serial.print(' ');Serial.println(rem);
  sscfgtB(buf,jsbuf,fn,rem-1,remoteN[rem-1].osDurat,6,0,TRBEG|TDBE);
  if(remoteN[rem-1].osStatus==2){
    char remT[LDATEA];memset(remT,'0',LDATEA);memcpy(remT+6,remoteN[rem-1].osRemT,7);
    subTime(remT,remoteN[rem-1].osEndDate,now,VRAI);memcpy(remoteN[rem-1].osRemT,remT+8,6);
    //Serial.print(">==========");Serial.println(remoteN[rem-1].osEndDate);
    }
  scrDspText(buf,jsbuf,remoteN[rem-1].osRemT,0,TDBE);
  scrDspText(buf,jsbuf,remoteN[rem-1].osEndDate,0,TDBE);
  scrGetButSub(buf,jsbuf,"Maj",TREND|TDBE);
  tableEnd(buf,jsbuf,BRYES);
  formEnd(buf,jsbuf,0,BRYES);

// ---------------------------- une ligne état souhaité

  scrDspText(buf,jsbuf,"requested state",0,0);
  tableBeg(buf,jsbuf,courier,true,0,BRYES);
  disjVal=remoteN[rem-1].osEnable;
  for(uint8_t i=0;i<3;i++){                                                       // affichage 3 boutons état one_shot souhaité
                  if(remoteN[rem-1].osStatus==0){fn[LENNOM-2]=codeFn[i];}
                  else memcpy(fn,"null_fnct_",LENNOM);                            // si running ou paused pas de changement possible
                  if(disjVal%10==i){color=colors[i]+LIGHTVALUE;} else color=OFFCOLOR;
                  if(disjVal>=10){color+=LIGHTVALUE;}
                  ctl=TDBE;if(i==2){ctl=TDBE|TREND;}
                  scrGetButFn(buf,jsbuf,fn,remTNum,lib[i],ALICNO,2,color,0,0,1,ctl);
  }
  tableEnd(buf,jsbuf,BRYES);
  fnHtmlEnd(buf,jsbuf,0,BRYES);

// ----------------------------- une ligne commande STOP/PAUSE/START

  scrDspText(buf,jsbuf,"action",0,0);
  tableBeg(buf,jsbuf,courier,true,0,BRYES);
  if(*remTNum!=0){                                                                                  // remTNum doit être valide pour disjValue qui positionnera disjVal
    disjVal=remoteN[rem-1].osStatus;
    char    fnv[LENNOM+1];fnv[LENNOM-1]=(char)(rem-1+PMFNCHAR);fnv[LENNOM]='\0';                    // N° remote
      // bouton stop 
      memcpy(fnv,"remote_od",LENNOM-1);
      if(disjVal%10==0){color=OFFCOLOR;memcpy(fnv,"null_fnct",LENNOM-1);} else color=CURCOLOR;
      //if(disjVal>=10){color+=LIGHTVALUE;}
      scrGetButFn(buf,jsbuf,fnv,remTNum,libcd[0],ALICNO,2,color,0,0,1,TRBEG|TDBE);
      // bouton pause
      memcpy(fnv,"remote_oe",LENNOM-1);
      if(disjVal%10!=2){color=OFFCOLOR;memcpy(fnv,"null_fnct",LENNOM-1);} else color=CURCOLOR;
      //if(disjVal>=10){color+=LIGHTVALUE;}
      scrGetButFn(buf,jsbuf,fnv,remTNum,libcd[1],ALICNO,2,color,0,0,1,TDBE);
      // bouton start
      memcpy(fnv,"remote_of",LENNOM-1);
      if(disjVal%10==2){color=OFFCOLOR;memcpy(fnv,"null_fnct",LENNOM-1);} else color=CURCOLOR;
      //if(disjVal>=10){color+=LIGHTVALUE;}
      scrGetButFn(buf,jsbuf,fnv,remTNum,libcd[2],ALICNO,2,color,0,0,1,TREND|TDBE);                          
  }

  tableEnd(buf,jsbuf,BRYES);

  formEnd(buf,jsbuf,0,0);
  htmlEnd(buf,jsbuf);
  strcat(buf,"\n");
  ethWrite(cli,buf,&lb);
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
            
            tableBeg(buf,jsbuf,0);
            scrGetButRet(buf,jsbuf,"retour",TRBEG|TDBE);
            scrGetButRef(buf,jsbuf,"remote_cr",0,TDBE|TREND|BRYES);
            scrGetButFn(buf,jsbuf,"thermoshos","","températures",false,6,0,0,0,0,TRBEG|TDBE);
            scrGetButFn(buf,jsbuf,"timersctl_","","timers",false,6,0,0,0,0,TDBE|TREND);
            tableEnd(buf,jsbuf,BRYES);
            ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end

/* Principes (voir aussi cfgRemote)
  Chaque remote possède un bouton soit push fugitif soit slider fixe et un disjoncteur à 3 position (off/on/forced)
  de plus un rond jaune indique l'état du switch associé
  Les remotes sont de 2 types : multiples (plusieurs switchs) ou simples (un seul switch)
    remotes simples   : contrôlent le disjoncteur d'un seul switch directement via les 3 boutons 
    remotes multiples : contrôlent plusieurs switchs (donc remotes simples éventuellement) 
                        via disjoncteur et variable remoteN[x].enable qui en stocke l'état
  Dans les 2 cas le slider/push correspond à un mds associé au bouton.
  Les push sont fugitifs : le mds est mis à 1 avant la transmission au périf, puis remis à 0 aussitôt après.
  Les slider/push sont désactivés si le disjoncteur de la même remote est OFF ou si le disjoncteur d'une remote multiple qui la controle est OFF
  Couleurs : chaque couleur représente l'état d'un bouton : Une couleur par état ON ou OFF
            la couleur est soit 'light' si le bouton est désactivé soit 'strong' si activé 
  On peut modifier l'état d'un bouton désactivé, mais cet état ne sera pris en compte (modif mds ou modif swCde) que lors de l'activation
  A chaque modif de mds ou swCde, un message periReq est envoyé au(x) périf(s) concerné(s)

  Codage
  la couleur de chaque bouton dépend de son état ON/OFF/FORCED et de son activation ou non
  disjVal regroupe ces infos : 0/1/2 + 10 si remote multiple (mère) disjonctée 
  Donc valeurs 0/1/2/10/11/12 pour remotes simples et 0/1/2 pour multiples

*/
           
            tableBeg(buf,jsbuf,courier,true,0,0); 
            strcat(buf,"\n");            

            for(uint8_t nb=0;nb<NBREMOTE;nb++){
              ni++;
              uint16_t periCur=0;
              uint8_t disjVal=0;                                        // valeur disjoncteur (0/1/2/10/11/12)
              uint8_t color;
              char mother[]={"- -- -"};
              char remTNum[]={'\0','\0'};                               // N° switch dans table remoteT (MAXREMLI ou pointeur valide)
              uint8_t nb1=nb+1;
              uint8_t butModel=remoteN[nb].butModel;                  
              char fn[LENNOM+1];
              char fnt[LENNOM+1];                                       // bouton '>'
              if(remoteN[nb].nam[0]!='\0'){
                strcat(buf,"<tr height=80>");                          // patch à intégrer dans le ctl des fonctions d'affichage
                scrDspNum(buf,jsbuf,'s',&nb1,0,0,TDBE);
                
                if(!remoteN[nb].multRem){                               // remote simple

                  uint8_t td=0;
                  periCur=0;
                  for(td=0;td<MAXREMLI;td++){                           // recherche du switch associé

                    if(remoteT[td].num==nb+1){                          // même remote -> trouvé

                      if(remoteT[td].peri!=0){                          // périphérique présent

                        periCur=remoteT[td].peri;
                        periLoad(periCur);
                        disjVal=periSwRead(remoteT[td].sw);             // disjval est le disjoncteur de cette remote simple
                        if(remoteT[td].multRem!=0){mother[2+convIntToString(&mother[2],(int)remoteT[td].multRem)]=' ';}
                        if(remoteT[td].multRem!=0 && remoteN[remoteT[td].multRem-1].enable==0){
                          disjVal+=10;}                                 // remote 'mère' disjonctée
                        //Serial.print(" rem=");Serial.print(nb);Serial.print(" switch=");Serial.print(td);Serial.print(" mRem=");Serial.print(remoteT[td].multRem);Serial.print(" disjmRem=");Serial.print(remoteN[remoteT[td].multRem].enable);Serial.print(" disjVal=");Serial.println(disjVal);
                        strcat(buf,"<td width=45>");                    // patch à intégrer dans le ctl des fonctions d'affichage
                        if(((*periSwSta>>remoteT[td].sw)&0x01)!=0){     // switch 'allumé'
                          scrDspText(buf,jsbuf," ON ",0,BRYES);
                          affRondJaune(buf,jsbuf,TDEND);
                        }
                        else {
                          scrDspText(buf,jsbuf," OFF ",0,TDEND);}  
                      }
                      else {scrDspText(buf,jsbuf,".",0,TDBE);}          // défaut de config (pas de périf)
                      break;                                            // périf trouvé remote simple, td numéro remoteT, disjVal état disjoncteur
                    }
                  }
                  remTNum[0]=td+PMFNCHAR;                               // n° de l'entrée du switch dans la table des switchs
                }
                else {disjVal=remoteN[nb].enable;}                      // disjVal est le disjoncteur de cette remote multiple  

                if(periCur==0){scrDspText(buf,jsbuf," --- ",0,TDBE);}   // comble la colonne ON/OFF-rond jaune
                
                scrDspText(buf,jsbuf,remoteN[nb].nam,0,"Arial",30,TDBE);

                if(remoteN[nb].detec!=0){                                                       // slider/push présent

                  char val[]={'1',(char)(periCur+PMFNCHAR),(char)(disjVal+PMFNCHAR),'\0'};val[1]=(char)(periCur+PMFNCHAR); 
                                                                                                // 1er caractère valeur pour mds si le slider/push est modifié
                                                                                                // 2nd car pour transmission periCur si remote simple
                                                                                                // 3rd car pour transmission disjVal
                  
                  uint8_t mi=remoteN[nb].detec>>3;uint16_t ptmi=(remoteN[nb].detec*MDSLEN)+mi;  // adresse mds slider/push  
                  color=ONCOLOR;                                                                // bleu si 1
                  if(((memDetServ[mi]&mDSmaskbit[ptmi])==0) ){color=OFFCOLOR;}                  // gris si 0                   
                  else {val[0]='0';}                                                            // valeur à mettre dans le bit          
                  if(disjVal==0 || disjVal>=10){color+=LIGHTVALUE;}                             // disjoncté

                  memcpy(fn,"remote_cu_\0",LENNOM+1);fn[LENNOM-1]=(char)(nb+PMFNCHAR);

                  if(butModel==SLIDER){                                                         // slider  
                    //Serial.print("det=");Serial.print(remoteN[nb].detec);Serial.print(" mi=");Serial.print(mi);Serial.print(" ptmi=");Serial.print(ptmi);Serial.print(" val=");Serial.print(val);Serial.print(" mds=");Serial.print(memDetServ[mi]&mDSmaskbit[ptmi],HEX);Serial.print(' ');Serial.println(memDetServ[mi]&mDSmaskbit[ptmi],HEX);
                    //Serial.print(" slider color=");Serial.println(color);
                    scrGetButFn(buf,jsbuf,fn,val,"SLIDER",ALICNO,4,color,0,1,RND,TDBEG);
                  }
                  else{                                                                         // push button
                    //Serial.print(" push color=");Serial.println(color);
                    color=color/10*10+PUSHCOLOR;                                                // conserve LIGHTVALUE
                    scrGetButFn(buf,jsbuf,fn,val,"PUSH",ALICNO,4,color,1,1,SQR,TDBEG);          // envoie toujours '1'
                  }
                }
                else {scrDspText(buf,jsbuf,"- -",0,TDBE);}                                    // slider/push absent

                
                scrDspText(buf,jsbuf,mother,0,TDBE);               
                strcat(buf,"\n");
                
                //scrDspText(buf,jsbuf,"",0,TDBEG);
                memcpy(fn,"remote_c__\0",LENNOM+1);fn[LENNOM-1]=(char)(nb+PMFNCHAR);            // transmission n° remote
                memcpy(fnt,"remote_ct_\0",LENNOM+1);fnt[LENNOM-1]=(char)(nb+PMFNCHAR);          // transmission n° remote pour one_shot_timer

                char codeFn[3]={'a','b','c'};                                                   // pour disjoncteurs OFF/ON/FORCED
                const char* lib[3];lib[0]="OFF";lib[1]="ON";lib[2]="FOR";
                uint8_t colors[3]={DISJCOLOR,ONCOLOR,FORCEDCOLOR};                              // pour valeurs 0/1/2 du disjoncteur                

                for(uint8_t i=0;i<3;i++){                                                       // affichage 3 boutons
                  fn[LENNOM-2]=codeFn[i];
                  if(disjVal%10==i){color=colors[i];} else color=OFFCOLOR;
                  if(disjVal>=10){color+=LIGHTVALUE;}
                  //Serial.print(" disj color=");Serial.println(color);
                  scrGetButFn(buf,jsbuf,fn,remTNum,lib[i],ALICNO,3,color,0,0,0,TDBEG);
                  uint8_t ctl=TDEND;
                  if(i==2){
                    ctl=TDEND|TREND|BRYES;
                    scrDspText(buf,jsbuf," ",0,TDEND);
                    scrDspText(buf,jsbuf," - ",0,TDBEG);
                    scrGetButFn(buf,jsbuf,fnt,remTNum,">",ALICNO,3,OFFCOLOR,0,0,0,0);
                  }
                  scrDspText(buf,jsbuf,"  ",0,ctl);
                }

                lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}               
              }
            }
            if(buf[0]!='\0'){ethWrite(cli,buf);}
            tableEnd(buf,jsbuf,BRYES);
            
            //scrGetButFn(buf,jsbuf,"thermoshow","","températures",ALICNO,5,STDBUTTON,1,0,1,0);
            //scrGetButFn(buf,jsbuf,"timersctl_","","timers",ALICNO,5,STDBUTTON,1,0,1,0);
            
            formEnd(buf,jsbuf,0,0);
            htmlEnd(buf,jsbuf);
            ethWrite(cli,buf,&lb);
            
            bufLenShow(buf,jsbuf,lb,begTPage);
}

void timersCtlHtml(EthernetClient* cli)
{              
  Serial.print(millis());Serial.print(" timersCtlHtml() ");

  uint16_t lb0=8000;
  char buf[lb0];buf[0]='\0';
  char jsbuf[LBUF4000];*jsbuf=0x00;

  uint16_t lb;
  unsigned long begTPage=millis();                  // calcul durée envoi page
 
  htmlBeg(buf,jsbuf,serverName,'R');

  pageLineOne(buf,jsbuf);
  formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);
            //usrFormBHtml(buf,1);
  scrGetButRet(buf,jsbuf,"retour",0);
  ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end


  tableBeg(buf,jsbuf,"courier","30",true,0,0);
  scrDspText(buf,jsbuf,"nom|det|h_beg|h_end|OI det|",0,TRBE|TDBE);

  for(uint8_t nt=0;nt<NBTIMERS;nt++){
    formIntro(buf,jsbuf,0,TRBEG|TDBEG|BRYES);
    
    //scrDspNum(buf,jsbuf,'s',&(++nt),0,0,TDEND);nt--;
    scrDspText(buf,jsbuf,timersN[nt].nom,0,TDBE);                 
    scrDspNum(buf,jsbuf,'D',&timersN[nt].detec,0,15,TDBE);                 
    scrDspText(buf,jsbuf,timersN[nt].hdeb,0,15,TDBE);
    scrDspText(buf,jsbuf,timersN[nt].hfin,0,15,TDBE);
                    
    char oo[7];memset(oo,'_',6);oo[6]=0x00;
    char oi[]="OI";  
    
    oo[1]=oi[timersN[nt].curstate];
    oo[4]=(PMFNCVAL)+mDSval(timersN[nt].detec);                     
    scrDspText(buf,jsbuf,oo,15,TDBE);

    char nf[]="tim_ctl___";nf[LENNOM-1]=nt+PMFNCHAR;
    const char* lib[2];
    lib[0]="enable";
    lib[1]="disable";
    uint8_t color=OFFCOLOR;
    if(timersN[nt].enable!=0){color=ONCOLOR;}
    scrGetButFn(buf,jsbuf,nf,"",lib[timersN[nt].enable],ALICNO,4,color,0,1,RND,TDBE|TREND);

    formEnd(buf,jsbuf,0,TDEND|TREND);
    strcat(buf,"\n");

    ethWrite(cli,buf,&lb);
  }

  tableEnd(buf,jsbuf,0);
  htmlEnd(buf,jsbuf);
  
  ethWrite(cli,buf,&lb);

  bufLenShow(buf,jsbuf,lb,begTPage);
}

void anTimersCfgHtml(EthernetClient* cli)
{              
  Serial.print(millis());Serial.print(" anTimersCtlHtml() ");

  uint16_t lb0=8000;
  char buf[lb0];buf[0]='\0';
  char jsbuf[LBUF4000];*jsbuf=0x00;

  uint16_t lb;
  unsigned long begTPage=millis();                  // calcul durée envoi page
 
  htmlBeg(buf,jsbuf,serverName,'R');

  pageLineOne(buf,jsbuf);
  formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);
            //usrFormBHtml(buf,1);
  scrGetButRet(buf,jsbuf,"retour",0);
  ethWrite(cli,buf,&lb);
// ------------------------------------------------------------- header end


  tableBeg(buf,jsbuf,courier,true,0,0);
  scrDspText(buf,jsbuf,"|nom|en|d-I|d-O|time-1|val-1|time-2|val-2|time-3|val-3|time-4|val-4|time-5|val-5|time-6|val-6|time-7|val-7|time-8|val-8|",0,TRBE|TDBE);

  char nf[]="atim_ctl__";
  const char* lib[2];
  lib[0]="enable";
  lib[1]="disable";

  for(uint8_t nt=0;nt<NBANTIMERS;nt++){
    nf[LENNOM-1]=nt+PMFNCHAR;
    formIntro(buf,jsbuf,0,TRBEG|TDBEG|BRYES);
    
    scrDspNum(buf,jsbuf,'s',&(++nt),0,0,TDEND);nt--;
    scrDspText(buf,jsbuf,analTimers[nt].nom,7,TDBE);                 
    
    uint8_t color=OFFCOLOR;
    if((analTimers[nt].cb & ANT_BIT_ENABLE)!=0){color=ONCOLOR;}
    nf[LENNOM-2]=NBCBANT+8+PMFNCHAR;
    scrGetButFn(buf,jsbuf,nf,"",lib[(analTimers[nt].cb & ANT_BIT_ENABLE)],ALICNO,4,color,0,1,RND,TDBE|TREND);

    scrDspNum(buf,jsbuf,'D',&analTimers[nt].detecIn,0,0,TDBE);
    scrDspNum(buf,jsbuf,'D',&analTimers[nt].detecOut,0,0,TDBE);

    char hhmmss[6];
    for(uint8_t ntv=0;ntv<NBEVTANTIM;ntv++){
      unpack(hhmmss,analTimers[nt].heure,3);
      scrDspText(buf,jsbuf,hhmmss,7,TDBE);
      scrDspNum(buf,jsbuf,'d',&analTimers[nt].valeur,0,0,TDBE);}  

    /*color=OFFCOLOR;
    nf[LENNOM-2]=NBCBANT+9+PMFNCHAR;
    scrGetButFn(buf,jsbuf,nf,"",libfo[analTimers[nt].factor_offset_mode],ALICNO,4,color,0,1,RND,TDBE|TREND);
    */

    formEnd(buf,jsbuf,0,TDEND|TREND);
    strcat(buf,"\n");

    ethWrite(cli,buf,&lb);
  }

  tableEnd(buf,jsbuf,0);
  htmlEnd(buf,jsbuf);
  
  ethWrite(cli,buf,&lb);

  bufLenShow(buf,jsbuf,lb,begTPage);
}


int scalcTh(const char* endDate,char* dhasc,const char* prev)           // maj temp min/max des périphériques sur les prev derniers jours
{                                                                       // jusqu'à endDate (non traité toujours now) 
                                                                        // retour date début dans dhasc
/* --- calcul date début --- */
 
  Serial.print("scalcTh ");
  memset(dhasc,'\0',16);
  
  unsigned long t0=millis();
  
  int   ldate=16;       // "YYYYMMDD HHMMSS\0"
  unsigned long unixPrev=alphaDateToUnix(prev,false,true);
  unsigned long unixBeg=unixNow-unixPrev;

//Serial.print(now);Serial.print(" - ");Serial.print(unixNow);Serial.print(" - ");Serial.print(prev);Serial.print(unixPrev);Serial.print(" - ");Serial.println(unixBeg);
  unixDateToStr(unixBeg,dhasc);
  for(uint8_t k=13;k>7;k--){dhasc[k+1]=dhasc[k];}dhasc[8]=' ';  // mise au format de l'histo

  //Serial.print(unixNow);Serial.print(" ");Serial.print(thermoPrev);Serial.print(" ");Serial.print(unixPrev);Serial.print(" ");Serial.println(unixBeg);
  
  if(sdOpen("fdhisto.txt",&fhisto)==SDKO){Serial.println("sdOpen fdhisto KO");return SDKO;}

  long histoSiz=fhisto.size();
  long searchStep=100000; // devrait etre un param de config
  long ptr=0,curpos=histoSiz;
  fhisto.seek(curpos);
  long pos=fhisto.position();  
  
  Serial.print("--- start search date:");Serial.print(dhasc);Serial.print(" at ");Serial.print(curpos-searchStep);Serial.print(" histoSiz=");Serial.print(histoSiz);Serial.print(" pos=");Serial.println(pos);

/* --- recherche 1ère ligne --- */

  char inch1=0;
  char buf[RECCHAR];
  bool fini=FAUX;
  int pt;

/* recherche rétrograde de la date début */
  
  char etat='0';

  trigwd();

  // 1) recule jusqu'à date < dhasc
  while(curpos>0 && !fini){
    curpos-=searchStep;if(curpos<0){curpos=0;etat=5;break;}
    ptr=curpos;
    fhisto.seek(curpos);
    //Serial.print("W ");
    inch1='\0';
    // 1.1 search \n pour positionnement sur début ligne
    while(inch1!='\n'){inch1=fhisto.read();//Serial.print(inch1);
          ptr++;}
    // 1.2 anomalie searchStep char lus sans \n 
    if(ptr>=(curpos+searchStep)){fini=true;etat='1';break;}                                                                        
    // 1.3 get date dans buf
    for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();}
    //buf[ldate]='\0';Serial.println(buf);                           // '\n' trouvé : get date
    // 1.4 trouvé date =
    if(memcmp(buf,dhasc,ldate)==0){fini=true;etat='2';break;}                                // ptr sur début ligne
    // 1.5 trouvé date <
    if(memcmp(buf,dhasc,ldate)<0){                                            // si la date trouvée est < chercher >= sinon reculer                                                                 
      // 1.5a continue lecture jusqu'à date >= ou rien trouvé
      while(!fini){
        fhisto.seek(ptr);
        //Serial.print("w ");
        inch1='\0';
        while(inch1!='\n'){inch1=fhisto.read();//Serial.print(inch1);
              ptr++;}             // ptr sur '\n'
        if(ptr>=(curpos+searchStep)){fini=true;etat='3';break;}
        
        for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();}                       // '\n' trouvé : get date
        //buf[ldate]='\0';Serial.println(buf);
        if(memcmp(buf,dhasc,ldate)>=0){                                       // si la date trouvé est >= ok sinon continuer
          fini=VRAI;etat='4';                                                 // ptr ok ; pt ok commencer l'acquisition
        }
      }
    }
    // fini etat 1 anomalie pas de \n dans les searchStep char avant la fin du fichier
    // fini etat 2 trouvé (date =)
    // fini etat 3 anomalie pas trouvé date >= 
    // fini etat 4 trouvé (date >= après avoir trouvé date <)
    // sinon trouvé date > donc reculer
    // etat 5 rien trouvé, début du fichier
    trigwd(); 
  }
  

  trigwd();
  fhisto.seek(ptr);
  Serial.print("--- fin recherche état ");Serial.print(etat);
  if(etat=='4' || etat=='2'){buf[15]='\0';Serial.print(" cur date=");Serial.print(buf);}
  Serial.print(" ptr=");Serial.print(ptr);Serial.print(" millis=");Serial.println(millis()-t0);
  //char c='\0';while(c!='\n'){c=fhisto.read();Serial.print(c);}

/* --- init ptrs et th min/max --- */

  unsigned long t1=millis();
  char strfds[3];memset(strfds,0x00,3);
  if(convIntToString(strfds,fdatasave)>2){
    Serial.print("fdatasave>99!! ");Serial.print("fdatasave=");Serial.print(fdatasave);Serial.print(" strfds=");Serial.println(strfds);ledblink(BCODESYSERR,PULSEBLINK);
  }

  byte* periMac[NBPERIF];         // pointeurs mac addr dans cache
  int16_t* periThMin[NBPERIF];    // pointeurs ThMin dans cache
  int16_t* periThMax[NBPERIF];    // pointeurs ThMax dans cache

  for(int pp=0;pp<NBPERIF;pp++){
    periMac[pp]=(byte*)((char*)periMacr-(char*)periBegOfRecord+(pp*PERIRECLEN)+(char*)periCache);
    periThMin[pp]=(int16_t*)((char*)periThmin_-(char*)periBegOfRecord+(pp*PERIRECLEN)+(char*)periCache);
    periThMax[pp]=(int16_t*)((char*)periThmax_-(char*)periBegOfRecord+(pp*PERIRECLEN)+(char*)periCache);
    *periThMin[pp]=9900;
    *periThMax[pp]=-9900;
  }

  //Serial.print((unsigned long)periBegOfRecord);Serial.print(" ");Serial.println((unsigned long)periCache);
  Serial.println("--- fin init ptrs periCache & th min/max ");Serial.print("--- ");

  /* --- acquisition lignes --- */
   
  char* pc;
  int16_t th_;
  uint8_t np_;
  int ldata=0,nbli=0,nbth=0;
  long poswd=ptr;
  long wdStep=100000; // devrait etre un param de config
                                                                         // acquisition
  fhisto.seek(ptr-ldate);                                                // sur début enregistrement
  fini=FAUX;
  while(ptr<pos){

    if((poswd+wdStep)<ptr){trigwd();poswd+=wdStep;}

    pt=0;
    inch1='\0';
    while(ptr<pos && inch1!='\n'){inch1=fhisto.read();buf[pt]=inch1;pt++;ptr++;}   // get record
    buf[pt]='\0';
    nbli++;
    pc=strchr(buf,';');if(pc>(buf+LBUFSERVER)){pc=0;}
  
    if(memcmp(pc+1,strfds,2)==0){                                         // datasave (après ';' soit '\n' soit'<' soit num fonction)
      np_=(uint8_t)convStrToInt(pc+HISTOPOSNUMPER,&ldata);                // num périphérique
      th_=(int16_t)(convStrToNum(pc+HISTOPOSTEMP,&ldata)*100);            // temp périphérique   
      if(np_==0 || np_>NBPERIF){Serial.print(nbli);Serial.print(" ligne histo anormale périf=");Serial.print(np_);Serial.print(" ");Serial.println(pc);}
      else{
        packMac(periMacBuf,pc+HISTOPOSMAC);
        if(memcmp(periMacBuf,periMac[np_-1],6)==0 && th_<9900 && th_>-9900){       // contrôle mac
          if(*periThMin[np_-1]>th_){*periThMin[np_-1]=(int16_t)th_;}
          if(*periThMax[np_-1]<th_){*periThMax[np_-1]=(int16_t)th_;}        
          nbth++;
        }
/*
        if(np_==17){
          periLoad(np_);
          Serial.print(buf);
          Serial.print(" ");Serial.print(th_);Serial.print(" ");Serial.print(*periThmin_);Serial.print(" ");Serial.print(*periThmax_);Serial.print(" ");
          Serial.print(*periThMin[np_]);Serial.print(" ");Serial.print(*periThMax[np_]);Serial.print(" ");
          serialPrintMac(periMac[np_-1],0);Serial.print(" ");serialPrintMac(periMacBuf,1);
        }      
*/        
      }      
    }
  }
  
  fhisto.close();
  periTableSave();

  Serial.print("--- fin balayage ");Serial.print(nbli);Serial.print(" lignes ; ");Serial.print(nbth);
  Serial.print(" màj ; millis=");Serial.print(millis()-t1);Serial.print(" total=");Serial.println(millis()-t0);
#ifdef DEBUG_ON
  delay(1);
#endif
  
  return SDOK;
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
 
  tableBeg(buf,jsbuf,courier,NOBORDER,BRYES|TRBEG);
  scrGetButRet(buf,jsbuf,"retour",TDBE);
  scrGetButRef(buf,jsbuf,"thermosho",0,TDBE|TREND);
  //formIntro(buf,jsbuf,0,0);                
  scrDspText(buf,jsbuf,"prev yyyymmddhhmmss",18,TRBEG|TDBEG|BRYES);
  scrGetText(buf,jsbuf,thermoCurPrev,"thermoshop",12,15,18,0,TDEND);
  //scrGetButFn(buf,jsbuf,"thermoshos","","màj min/max",false,2,7,1,1,0,0,TDBE);
  scrGetButSub(buf,jsbuf,"màj min/max",ALICNO,6,TDBE|TREND);
  formEnd(buf,jsbuf,0,0);
  tableEnd(buf,jsbuf,BRYES);

  ethWrite(cli,buf,&lb);            // tfr -> navigateur
 // ------------------------------------------------------------- header end 

/* peritable températures */
  char thls[16];memcpy(thls,thermoLastScan,8);thls[8]=' ';memcpy(thls+9,thermoLastScan+8,6);thls[15]='\0';
  scrDspText(buf,jsbuf,"min/max du ",0,0);scrDspText(buf,jsbuf,thermoLastBeg,0,0);scrDspText(buf,jsbuf," au ",0,0);scrDspText(buf,jsbuf,thls,0,BRYES);        // période scan
  
  tableBeg(buf,jsbuf,"Courier","30",BORDER,nullptr,BRYES|TRBEG);
  scrDspText(buf,jsbuf,"peri||_|TH|_|min| max|last in",0,TDBE|TREND);
  strcat(buf,"\n");
              for(int nuth=0;nuth<NBTHERMOS;nuth++){
                periCur=thermos[nuth].peri;
                if(periCur!=0){
                  periLoad(periCur);

                  if(periMacr[0]!=0x00){
                    ni++;
                    float th;
                    strcat(buf,"<tr height=80>");
                    scrDspNum(buf,jsbuf,'I',&periCur,0,0,TDBE);scrDspText(buf,jsbuf,thermos[nuth].nom,0,"arial",30,TDBE);
                    affSpace(buf,jsbuf,TDBE);
                    th=(float)(*periLastVal_+*periThOffset_)/100;scrDspNum(buf,jsbuf,'f',&th,0,0,TDBE);
                    affSpace(buf,jsbuf,TDBE);
                    th=(float)(*periThmin_+*periThOffset_)/100;scrDspNum(buf,jsbuf,'f',&th,0,15,TDBE);                    
                    th=(float)(*periThmax_+*periThOffset_)/100;scrDspNum(buf,jsbuf,'f',&th,0,15,TDBE);                    
                    
                    memset(lith,0x00,LLITH);
                    bufPrintPeriDate(lith,periLastDateIn);
                    scrDspText(buf,jsbuf,lith,15,TDBE|TREND);
                    strcat(buf,"\n");                      
                    
                    lb=strlen(buf);if(lb0-lb<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
                  }
                }
              }
          tableEnd(buf,jsbuf,BRYES);

        /*
        memset(lith,0x00,LLITH);
        for(int d=0;d<NBDSRV;d++){
          mDSconc(lith,d);
          strcat(lith," ");}
        scrDspText(buf,jsbuf,lith,0,BRYES);
        */
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
      scrDspText(buf,jsbuf,&oi[mDSval(vv)*2],2,STRING|CONCAT);}
      //scrDspText(buf,jsbuf,&oi[((memDetServ>>vv)&0x01)*2],2,STRING|CONCAT);}
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
          
  scrGetButRef(buf,jsbuf,"thermoscfg",0,BRYES);//scrGetButFn(buf,jsbuf,"thermoscfg","","refresh",ALICNO,0,BRYES);
  
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
      if(periCur>=NBPERIF){Serial.print("config Thermos");}
      periLoad(periCur);
      scrDspText(buf,jsbuf,periNamer,0,TDBEG);
      scrDspText(buf,jsbuf,":",0,0);
      float pl=(*periLastVal_)/100;
      scrDspNum(buf,jsbuf,'f',&pl,2,0,TDEND);}
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
  scrGetButRef(buf,jsbuf,"timerHtml",0,BRYES);

  ethWrite(cli,buf,&lb);              // tfr -> navigateur
// ------------------------------------------------------------- header end 

  detServHtml(cli,buf,jsbuf,&lb,lb0,memDetServ,&libDetServ[0][0]);  
  
  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf,"|nom|det|h_beg|h_end|OI det|e_p_c_|7_d_ l_m_m_ j_v_s|dh_beg_cycle|dh_end_cycle|onStateDur|offStateDur|dh_last_start|dh_last_stop",0,TRBE|TDBE);
  strcat(buf,"\n"); 

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
    //sscfgtB(buf,jsbuf,"tim_hdf_p",nt,&timersN[nt].dayPeriode,3,3,TDBE);                   
    //sscfgtB(buf,jsbuf,"tim_hdf_P",nt,timersN[nt].timePeriode,6,0,TDBE);                   
                    
    char oo[7];memset(oo,'_',6);oo[6]=0x00;
    char oi[]="OI";  
    
    oo[1]=oi[timersN[nt].curstate];
    oo[4]=(PMFNCVAL)+mDSval(timersN[nt].detec);                     
    //oo[4]=(PMFNCVAL)+((memDetServ>>timersN[nt].detec)&0x01);                     
    scrDspText(buf,jsbuf,oo,0,TDBE);
    nucb=0;sscb(buf,jsbuf,timersN[nt].enable,"tim_chkb__",nucb,NO_STATE,TDBEG,nt);affSpace(buf,jsbuf);    // cb 0 : enable
    nucb++;sscb(buf,jsbuf,timersN[nt].perm,"tim_chkb__",nucb,NO_STATE,0,nt);affSpace(buf,jsbuf);          // cb 1 : permanent
    nucb++;sscb(buf,jsbuf,timersN[nt].cyclic_,"tim_chkb__",nucb,NO_STATE,TDEND,nt);                       // cb 2 : cyclic

    ethWrite(cli,buf,&lb);

    uint8_t lctl=TDBEG;
    for(int nj=7;nj>=0;nj--){
      bool vnj; 
      vnj=(timersN[nt].dw>>nj)&0x01;
      nucb++;sscb(buf,jsbuf,vnj,"tim_chkb__",nucb,NO_STATE,lctl,nt);                                      // cb 3->n : dw
      lctl=0;if(nj==1){lctl=TDEND;}
      if(nj>=1){affSpace(buf,jsbuf);}
    }

    ethWrite(cli,buf,&lb);
    strcat(buf,"\n");

    sscfgtB(buf,jsbuf,"tim_hdf_b",nt,&timersN[nt].dhdebcycle,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_e",nt,&timersN[nt].dhfincycle,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_p",nt,&timersN[nt].onStateDur,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_P",nt,&timersN[nt].offStateDur,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_s",nt,&timersN[nt].dhLastStart,14,0,TDBE);
    sscfgtB(buf,jsbuf,"tim_hdf_S",nt,&timersN[nt].dhLastStop,14,0,TDBE); 

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

void anTimersHtml(EthernetClient* cli)
{
    
Serial.print(" config analog timers ");
#ifdef DEBUG_ON
  delay(100);
#endif
  char jsbuf[8000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=8000;
  char buf[lb0];*buf=0x00;

  unsigned long begTPage=millis();    // calcul durée envoi page

  htmlBeg(buf,jsbuf,serverName);       // chargement CSS etc

  //formIntro(buf,jsbuf,0,0);           // params pour retours navigateur (n° usr + time usr + pericur)
                                      // à charger au moins une fois par page ; pour les autres formulaires 
                                      // de la page formBeg() suffit

  pageLineOne(buf,jsbuf);              // 1ère ligne page
  scrGetButRet(buf,jsbuf,"retour",0);strcat(buf," ");    
  scrGetButRef(buf,jsbuf,"anTimCfg__",0,BRYES);

  ethWrite(cli,buf,&lb);              // tfr -> navigateur
// ------------------------------------------------------------- header end 

  //detServHtml(cli,buf,jsbuf,&lb,lb0,memDetServ,&libDetServ[0][0]);

  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf," |nom|en|deti|deto|7_d_ l_m_m_ j_v_s|heure 1|val1|heure 2|val2|heure 3|val3|heure 4|val4|heure 5|val5|heure 6|val6|heure 7|val7|heure 8|val8|post|factor|offset",0,TRBE|TDBE);
  strcat(buf,"\n");   

  for(uint8_t nt=0;nt<NBANTIMERS;nt++){                                            // NBANTIMERS
    formIntro(buf,jsbuf,0,TRBEG|TDBEG);

    scrDspNum(buf,jsbuf,'s',&(++nt),0,0,TDEND);nt--;
                     
    sscfgtB(buf,jsbuf,"antim___n_",nt,analTimers[nt].nom,LENTIMNAM,0,TDBE);
    sscb(buf,jsbuf,(analTimers[nt].cb>>ANT_BIT_ENABLE)&0x01,"antim_cb__",ANT_BIT_ENABLE+8,NO_STATE,TDBE,nt);affSpace(buf,jsbuf);    // cb 0 : enable
    
    sscfgtB(buf,jsbuf,"antim___i_",nt,&analTimers[nt].detecIn,2,3,TDBE);                                            
    sscfgtB(buf,jsbuf,"antim___o_",nt,&analTimers[nt].detecOut,2,3,TDBE);

    uint8_t lctl=TDBEG;
    uint8_t nucb=0;
    for(int nj=7;nj>=0;nj--){
      uint8_t vnj=(analTimers[nt].dw>>nj)&0x01;
      sscb(buf,jsbuf,vnj,"antim_cb__",nucb,NO_STATE,lctl,nt);                                         // dw 0-7
      nucb++;
      lctl=0;if(nj==1){lctl=TDEND;}
      if(nj>=1){affSpace(buf,jsbuf);}
    }
    
    ethWrite(cli,buf,&lb);

    char unpHeure[7];unpHeure[6]='\0';
    for(uint8_t hh=0;hh<NBEVTANTIM;hh++){
      unpack(&analTimers[nt].heure[hh*3],unpHeure,3);
      sscfgtB(buf,jsbuf,"antim_hh__",nt,hh,unpHeure,6,0,TDBE);
      sscfgtB(buf,jsbuf,"antim_vv__",nt,hh,&(analTimers[nt].valeur[hh]),6,5,TDBE);
    }

    ethWrite(cli,buf,&lb);

    sscb(buf,jsbuf,(analTimers[nt].cb>>ANT_BIT_ANTEPOST)&0x01,"antim_cb__",ANT_BIT_ANTEPOST+8,NO_STATE,TDBE,nt);affSpace(buf,jsbuf);  // cb 1 : ante/post
    sscfgtB(buf,jsbuf,"antim___F_",nt,-1,&analTimers[nt].factor,6,3,4,TDBE);
    sscfgtB(buf,jsbuf,"antim___O_",nt,-1,&analTimers[nt].offset,6,3,4,TDBE);

    scrGetButSub(buf,jsbuf,"MàJ",TDBE|TREND);

    formEnd(buf,jsbuf,0,0);
    strcat(buf,"\n");

    ethWrite(cli,buf,&lb);
  }

  tableEnd(buf,jsbuf,0);
  htmlEnd(buf,jsbuf);
  
  ethWrite(cli,buf,&lb);

bufLenShow(buf,jsbuf,lb,begTPage);
}


void detServHtml(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0,uint8_t* mds,char* lib)
{
  if(buf!=nullptr && lb0!=0){

    Serial.print("detServHtml ");for(uint8_t i=0;i<MDSLEN;i++){if(memDetServ[i]<16){Serial.print('0');}Serial.print(memDetServ[i],HEX);Serial.print(' ');}Serial.println();

    formIntro(buf,jsbuf,"dsrv_init_",0,0);         // params pour retours navigateur (n° usr + time usr + pericur + locfonc pour inits)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit
    
    setFont(buf,"courier","10");
    scrDspText(buf,jsbuf,"<fieldset><legend>détecteurs serveur (n->0):</legend>\n",0,nullptr,0,0);
    strcat(buf,"\n");

    uint8_t ni=0;
    uint16_t lb1=0;      
        
        for(int k=NBDSRV-1;k>=0;k--){
            ni++;
            
            char libb[LENLIBDETSERV+40];libb[0]='\0';
            memcpy(libb,lib+k*LENLIBDETSERV,LENLIBDETSERV);
            if(libb[0]=='\0'){
              convIntToString(libb,k);
              strcat(libb,&(mdsSrc[sourceDetServ[k]/256]));
              if(sourceDetServ[k]/256!=0){
                concatn(libb,sourceDetServ[k]&0x00ff);
                strcat(libb," ");}
              else{strcat(libb,"-- ");}
            }
            subDSnBm(buf,jsbuf,"mem_dsrv__\0",mds,k,libb);
            
            strcat(buf,"\n");
            lb1=strlen(buf);if(lb0-lb1<(lb1/ni+100)){ethWrite(cli,buf);ni=0;}
        }
    scrGetButSub(buf,jsbuf,"Per Update",0);

    endFont(buf);      
    formEnd(buf,jsbuf,TITLE,0,0);
    //strcat(buf,"\n"); 
          
    ethWrite(cli,buf,lb);        
  }
}

void detServHtml(EthernetClient* cli,uint8_t* mds,char* lib)
{
  detServHtml(cli,nullptr,nullptr,0,0,mds,lib);
}

void testHtml(EthernetClient* cli)
{
            Serial.println(" page d'essais");
 htmlImg(cli,"sweeth.jpg");            
}           

