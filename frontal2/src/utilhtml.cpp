#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>
#include "ds3231.h"
#include <shconst2.h>
#include <shutil2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "pageshtml.h"
#include "utilhtml.h"
#include "utiljs.h"

extern Ds3231 ds3231;

extern char*     chexa;

extern char      periRec[PERIRECLEN];        // 1er buffer de l'enregistrement de périphérique
  
extern uint16_t  periCur;                    // Numéro du périphérique courant

extern int8_t    periMess;                   // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

extern uint16_t  perrefr;

extern char*     serverName;
extern char*     usrnames; 
extern char*     usrpass; 
extern unsigned long* usrtime;  

extern int       usernum;

extern uint8_t   memDetServ[];
extern uint8_t   mDSmaskbit[];

extern byte mask[];
extern char pkdate[7];

extern uint16_t ljs;

#define LENCOLOUR 8
char colour[LENCOLOUR+1];

int16_t valMin=-9999;
int16_t valMax=9999;

const char* courier={"Courier, sans-serif"};
bool borderparam=false;             // pour forcer l'init du navigateur
uint16_t styleTdWidth=0;            // forçage largeur des colonnes de table en px
const char* styleTdFont=nullptr;    // specific font inside <td> <\td>
uint16_t styleTdFSize=0;            // specific font size inside <td> <\td>

bool styleLoadedGeneral=false;
bool styleLoadedRemote =false;


/* ---- intro HTTP ---- */

const char* introHttp=
"HTTP/1.1 200 OK\n"
  //cli->println("Location: http://82.64.32.56:1789/");
  //cli->println("Cache-Control: private");
"CONTENT-Type: text/html; charset=UTF-8\n"
"Connection: close\n\n"
"<!DOCTYPE HTML ><html>\n";
//"<link rel=\"icon\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABUAAAAVCAYAAACpF6WWAAAABGdBTUEAALGPC/xhBQAAAAlwSFlzAAAOxAAADsQBlSsOGwAAATZJREFUOE+1lEtuwkAMhj3hIR67gGBDN4hK7KuuegWkHqTn6UHYcQcugASsqBBqU8SqqkpIOnZmAiE2GQn4JMvGM/7HnlFQsQZujGf8TXEW3T48mqgYJ1Er6CpcKHou5CSMDyXx3RuQ6Zc8WqmU5C4girKCJ3ZJmB3fjuh/LMhLiFdhxFOog1ot11V/PGI75zrWK0ekIhTsTl6dhdMvCkc5H3fbG8Dz+xAiHeM9WT99mwGEa/A3P/pXAu71V3OK2TuNqg3ovHQzgoj1mG89tQHKDZPJkoriKXgamvoM4Ot3R3n2VGS9gTgI0hrbJZKpwQU01awD/O1NViAMaZ+tOYVvJDyYoABhnzjdNciiniKnKl7OCLPOwYrii8MyGS3eRzkj9LqJcoj//ErJnViEUln0Gu7wUAD/NhVhzcOHSTAAAAAASUVORK5CYII\"/>";
//"<link rel=\"icon\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQBAMAAADt3eJSAAAAMFBMVEU0OkArMjhobHEoPUPFEBIuO0L+AAC2FBZ2JyuNICOfGx7xAwTjCAlCNTvVDA1aLzQ3COjMAAAAVUlEQVQI12NgwAaCDSA0888GCItjn0szWGBJTVoGSCjWs8TleQCQYV95evdxkFT8Kpe0PLDi5WfKd4LUsN5zS1sKFolt8bwAZrCaGqNYJAgFDEpQAAAzmxafI4vZWwAAAABJRU5ErkJggg==\"/>";
//"<link rel=\"shortcut icon\" href=\"data:\" />";

/* ---- Html Intro end ---- */

const char* introHtmlE=
"</style>\n"
"</head>\n"
"<body>\n";

/* ----  style general ---- */

/* table */
const char* generalSt=
"table {border-collapse: collapse;border-color: #dddddd;overflow: auto;"
"white-space:nowrap;}"
/* check box */
"#cb1{width:10px; padding:0px; margin:0px; text-align: center;}"
"#cb2{width:18px; text-align: center};"
/* button */
".button {background-color: #195B6A; border: none; color: white;"
" padding: 32px 80px:text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}"
".button2 {background-color: #77878A;}"
"#nt1{width:10px;}"
"#nt2{width:18px;}"
/* dialog box */
"#myDialog {border: 2px solid gray;border-radius: 20px;}";

/* ---- style remote ---- */

/* slider */
const char* remoteSt=
".switch {position: relative;display: inline-block;width: 220px;height: 100px; margin: 16px;}"
".switch input {opacity: 0;width: 0;hight: 0;}"
".slider {position: absolute;cursor: pointer;  top: 0;left: 0;right: 0;bottom: 0;background-color: #ccc;-webkit-transition: .4s;transition: .4s;}"
".slider:before {position: absolute;content: \"\";"
" height: 84px;width: 84px;left: 8px;bottom: 8px;background-color: white;-webkit-transition: .4s;transition: .4s;}"
" input:checked + .slider {background-color: #2196F3;}"
" input:focus + .slider {box-shadow: 0 0 1px #2196F3;}"
" input:checked + .slider:before {-webkit-transform: translateX(110px);-ms-transform: translateX(55px);transform: translateX(110px);}"
".slider.round {border-radius: 50px;}"
".slider.round:before {border-radius: 50%;}"
/* square button */
"@import url(\"https://fonts.googleapis.com/css?family=Roboto:400,400i,700\");"
".content {display:flex;flex-wrap:wrap;gap:1rem;justify-content:flex-start;}"
".content > div{flex-basis:200px;border:1px solid #ccc;padding:1rem;box-shadow: 0px 1px 1px rgba(0,0,0,0.2), 0px 1px 1px rgba(0,0,0,0.2);}"
" input[type=\"radio\"].sqbr {display: none;}"
" input[type=\"radio\"].sqbr + label {padding: 0.5rem 1rem;font-size: 1.50rem;line-height: 1.5;border-radius: 0.3rem;color: #fff;background-color: #6c757d;border: 1px solid transparent;transition: all 0.15s ease-in-out;}"
" input[type=\"radio\"].sqbr.br_off:hover + label { background-color: #28a745;border-color: #28a745;}"
" input[type=\"radio\"].sqbr.br_off:checked + label { background-color: #28a745;border-color: #28a745;}"
" input[type=\"radio\"].sqbr.br_on:hover + label { background-color: #338fff;border-color: #338fff;}"
" input[type=\"radio\"].sqbr.br_on:checked + label { background-color: #338fff;border-color: #338fff;}"
" input[type=\"radio\"].sqbr.br_for:hover + label { background-color: #dc3545;border-color: #dc3545;}"
" input[type=\"radio\"].sqbr.br_for:checked + label { background-color: #dc3545;border-color: #dc3545;}"
/* rond jaune */
"#rond_jaune {width: 40px;height: 40px;border-radius: 20px;background: yellow;}";



/* ---------------- utilitaires des fonctions Meta ------------------------- */

void bufcat(char* buf,const char* s){strcat(buf,s);}

void jscat(char* jsbuf,const char* s){
#ifndef NOJSBUF
  if(jsbuf!=nullptr){strcat(jsbuf,s);}
#endif // NOJSBUF
}

void jscat(char* jsbuf,const char* s,bool sep){
#ifndef NOJSBUF
  jscat(jsbuf,s);if(sep){strcat(jsbuf,JSSEP);}
#endif // NOJSBUF  
}

void jscatch(char* jsbuf,const char s){
#ifndef NOJSBUF
  if(jsbuf!=nullptr){char* pt=jsbuf+strlen(jsbuf);*pt=s;*(pt+1)=*(pt+2)=0x00;}
#endif // NOJSBUF
}

void jscatch(char* jsbuf,const char s,bool sep){
#ifndef NOJSBUF
  if(jsbuf!=nullptr){char* pt=jsbuf+strlen(jsbuf);*pt=s;*(pt+1)=*JSSEP;*(pt+2)=0x00;}
#endif // NOJSBUF
}

void fnJsIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl)   // construit les 2,3 ou 4 premiers caractères d'une fonction
{
#ifndef NOJSBUF
  if(jsbuf!=nullptr){
    char* jspt=jsbuf+strlen(jsbuf);
    if(*(jspt-1)!=LF){*jspt=LF;jspt++;}
    *jspt=*JSFON;jspt++;
    *jspt=*fonc;jspt++;
    
    if(*(fonc+1)==*JSCTL){
      *jspt=(ctl&~CONCAT)|CTLCH;
      jspt++;
      if(pol!=0){
        *jspt|=CTLPO;jspt++;
        *jspt=(pol&0x7F)|PMFNCHAR;jspt++;
      }
    }
    *(jspt)=0x00;
  }
#endif // NOJSBUF
}

void tdSet(char* jsbuf,uint8_t width,const char* font,uint8_t fsize,uint8_t ctl)
{
#ifndef NOJSBUF
  if(jsbuf!=nullptr){fnJsIntro(jsbuf,JSTDS,0,ctl);}
  jscatch(jsbuf,width+PMFNCVAL,SEP);
  jscat(jsbuf,font,SEP);
  jscatch(jsbuf,fsize+PMFNCVAL);
#endif // NOJSBUF

  if(width!=0){styleTdWidth=width;}
  if(font!=nullptr){styleTdFont=font;}
  if(fsize!=0){styleTdFSize=fsize;}

}

void tdReset()
{
  styleTdWidth=0;  
  styleTdFont=nullptr;   
  styleTdFSize=0; 
}

void tdcat(char* buf)
{
  strcat(buf,"<td");
      bool a=false;
      if(styleTdWidth!=0){a=true;strcat(buf," style=\"width: ");concatn(buf,styleTdWidth);strcat(buf,"px;");} 
      if(styleTdFont!=nullptr){if(!a){a=true;strcat(buf," style=\"");}
        strcat(buf," font:");strcat(buf,styleTdFont);strcat(buf,";");} 
      if(styleTdFSize!=0){if(!a){a=true;strcat(buf," style=\"");} 
        strcat(buf," font-size:");concatn(buf,styleTdFSize);strcat(buf,"px;");}
      if(a){strcat(buf,"\"");} 
      strcat(buf,">");
}

void fnHtmlIntro(char* buf,const char* font,uint8_t sizpol,uint8_t ctl,char* colour)
{ 
  if(buf!=nullptr){
    if((ctl&TRBEG)!=0 || (ctl&TRMASK)==TRBE){strcat(buf,"<tr>");}
    if((ctl&TDBEG)!=0 || (ctl&TDMASK)==TDBE){tdcat(buf);}
    if(*colour!=0x00){strcat(buf,"<font color=\"");strcat(buf,colour);strcat(buf,"\"> ");}
    //if(pol!=0){strcat(buf,"<font size=\"");concatn(buf,pol);strcat(buf,"\">");}
    if(font!=nullptr || sizpol!=0){strcat(buf,"<span style=\"");
      if(font!=nullptr){strcat(buf,"font-family:");strcat(buf,font);strcat(buf,";");}
      if(sizpol!=0){strcat(buf,"font-size:");concatn(buf,sizpol);strcat(buf,"px;");}    //strcat(buf,"px;");}
      strcat(buf,"\">");
    }
  }
}

void fnHtmlIntro(char* buf,uint8_t sizpol,uint8_t ctl,char* colour)
{
  fnHtmlIntro(buf,nullptr,sizpol,ctl,colour);
}

void fnHtmlIntro(char* buf,uint8_t sizpol,uint8_t ctl)
{
  char nocol='\0';
  fnHtmlIntro(buf,nullptr,sizpol,ctl,&nocol);
}

void fnHtmlEnd(char* buf,const char* font,uint8_t sizpol,uint8_t ctl)
{
  if(buf!=nullptr){
    //if(pol!=0){strcat(buf,"</font>");}
    if(font!=nullptr||sizpol!=0){strcat(buf,"</span>");}
    if((ctl&BRMASK)!=0){strcat(buf,"<br>");}
    if((ctl&TDEND)!=0 || (ctl&TDMASK)==TDBE){strcat(buf,"</td>");}
    if((ctl&TREND)!=0 || (ctl&TRMASK)==TRBE){strcat(buf,"</tr>");}
  }
}

void fnHtmlEnd(char* buf,uint8_t sizpol,uint8_t ctl)
{
  fnHtmlEnd(buf,nullptr,sizpol,ctl);
}


void concat1a(char* buf,char a)
{
  concat1a(buf,nullptr,a);
}

void concat1a(char* buf,char* jsbuf,char a)
{
  char b[2];b[1]='\0';
  b[0]=(char)(a);
  if(buf!=nullptr){strcat(buf,b);}
#ifndef NOJSBUF
  jscat(jsbuf,b);
#endif  
}

void concat1aH(char* buf,char a)
{
  char b[]="\0\0\0";
  if(a<16){b[0]='0';}
  else {b[0]=chexa[a>>4];}
  b[1]=chexa[a&0x0f];
  if(buf!=nullptr){strcat(buf,b);}
}

void concatn(char* buf,char* jsbuf,unsigned long val)               // concatene un entier dans buf (et termine avex 0x00)
                                                                    // si jsbuf valide, jsbuf recoit la même chaine avec ";\0" à la fin                                                                
{
  uint16_t s;
  char dm[10];
  s=sprintf(dm,"%lu",val);dm[s]='\0';
  if(jsbuf!=nullptr){       // jscat ne copie pas si NOJSBUF
    strcat(jsbuf,dm);
  }
  if(buf!=nullptr){strcat(buf,dm);} 
}

void concatn(char* buf,char* jsbuf,unsigned long val,bool sep)
{
  concatn(buf,jsbuf,val); 
  if(sep){jscat(jsbuf,JSSEP);}
}

void concatn(char* buf,unsigned long val)           // !!!!!!!!!!!!! ne fonctionne pas pour jsbuf utiliser la version surchargée !!!!!!!!!!!!!!
{
  concatn(buf,nullptr,val);
}

void concatns(char* buf,char* jsbuf,long val,bool sep)
{
  uint16_t s;
  #define LSPR 22
  char bb[LSPR];
  s=sprintf(bb,"%lu",val);
  if(s>LSPR){ledblink(BCODESYSERR,PULSEBLINK);}
  bb[s]='\0';
  if(jsbuf!=nullptr){       // jscat ne copie pas si NOJSBUF
    strcat(jsbuf,bb);if(sep){strcat(jsbuf,JSSEP);}
  }
  if(buf!=nullptr){strcat(buf,bb);}
}

void concatns(char* buf,long val)
{
  concatns(buf,nullptr,val,SEPNO);
}

void concatns(char* buf,char* jsbuf,long val)
{
  concatns(buf,jsbuf,val,SEPNO);
}

void concatnf(char* buf,float val)
{
  concatnf(buf,nullptr,val,2,NOBR);
}

void concatnf(char* buf,float val,uint8_t dec)
{
  concatnf(buf,nullptr,val,dec,NOBR);
}

void concatnf(char* buf,char* jsbuf,float val)
{
  concatnf(buf,jsbuf,val,2,NOBR);
}

void concatnf(char* buf,char* jsbuf,float val,uint8_t dec)
{
  concatnf(buf,jsbuf,val,dec,NOBR);
}

void concatnf(char* buf,char* jsbuf,float val,uint8_t dec,bool br)
{
  concatnf(buf,jsbuf,val,dec,br,SEPNO);
}

void concatnf(char* buf,char* jsbuf,float val,uint8_t dec,bool br,bool sep)
{
  #define LSPR 22
  char bb[LSPR];memset(bb,0x00,LSPR);
  float pdix[]={1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000};

  if(int(val)>pdix[9] || dec>9){        // 19+'-'+'.'=21
    Serial.print("concatnf ovf ");Serial.print(val);Serial.print(" ");Serial.println(dec);
    ledblink(BCODESYSERR,PULSEBLINK);
  } 

/*
  convIntToString(bb,int(val));
  if(dec!=0){
    bb[strlen(bb)]='.';
    convIntToString(bb+strlen(bb),int((val-int(val))*pdix[dec]),dec);}
*/

  char d[]="%.2f";d[2]=(char)(dec+0x30);
  int s=sprintf(bb,d,val);
  if(s<0 || s>LSPR){
    Serial.print("concatnf ovf ");Serial.print(val);Serial.print(" ");Serial.println(dec);
    ledblink(BCODESYSERR,PULSEBLINK);
  } 
  bb[s]='\0';

  if(jsbuf!=nullptr){       // jscat ne copie pas si NOJSBUF
    strcat(jsbuf,bb);if(sep){strcat(jsbuf,JSSEP);}
  }
  if(buf!=nullptr){strcat(buf,bb);if(br){strcat(buf,"<br>");}}
}

void concatDate(char* buf,char* jsbuf,char* periDate)
{
  char dateascii[LDATEASCII+2];

  unpackDate(dateascii+1,periDate); // 6 car peridate -> 12 car dateascii+1
  for(uint8_t i=0;i<6;i++){dateascii[i]=dateascii[i+1];}  // décalage date
  // memcpy(dateascii,dateascii+1,6); remplacé par for/next pour supprimer warning nucleo
  *(dateascii+6)=' ';*(dateascii+LDATEASCII+1)=0x00;      // insertion espace 
  
  //unpackDate(dateascii,periDate);dateascii[6]=' '
  strcat(buf,dateascii);
  //for(j=0;j<LDATEASCII;j++){concat1a(buf,dateascii[j]);if(j==5){strcat(buf," ");}}
#ifndef NOJSBUF
  jscat(jsbuf,dateascii);
#endif // NOJSBUF
}

void concatDate(char* buf,char* periDate)
{
  concatDate(buf,nullptr,periDate);
}

void bufPrintDateHeure(char* buf,char* pkdate)
{
  bufPrintDateHeure(buf,nullptr,pkdate);
}

void bufPrintDateHeure(char* buf,char* jsbuf,char* pkdate)
{
#ifndef NOJSBUF
  char* dm=buf+strlen(buf);
#endif // NOJSBUF

  char bufdate[LNOW];ds3231.alphaNow(bufdate);packDate(pkdate,bufdate+2); // skip siècle
  for(int zz=0;zz<14;zz++){concat1a(buf,bufdate[zz]);if(zz==7){strcat(buf,"-");}}
  strcat(buf,"(");concatn(buf,bufdate[14]);strcat(buf,")");strcat(buf," GMT ");

#ifndef NOJSBUF
  jscat(jsbuf,dm);
#endif // NOJSBUF
}

void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet,uint8_t st)
{
  // st =1 => \0 final sinon rien
  memset(recep,0x00,lenRecep);
  if(lenRecep<lenEmet){lenEmet=lenRecep;}
  if(lenRecep==lenEmet){lenEmet-=st;}
  memcpy(recep,emet,lenEmet);
  trailingSpaces(recep,lenRecep);
}

void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet)
{
  return alphaTfr(recep,lenRecep,emet,lenEmet,1);
}

/* ------------------- fonctions attributs --------------------- */

void setColourE(char* buf,char* jsbuf)
{
  strcat(buf,"</font>");
  fnJsIntro(jsbuf,JSCOE,0,0);
}

void setColourE(char* buf)
{
  setColourE(buf,nullptr);
}

void setColourB(char* buf,char* jsbuf,const char* textColour)
{
  if(buf!=nullptr){
    //memcpy(colour,textColour,LENCOLOUR);
    strcat(buf,"<font color=\"");strcat(buf,textColour);strcat(buf,"\"> ");
  }
#ifndef NOJSBUF  
  if(jsbuf!=nullptr){
    fnJsIntro(jsbuf,JSCOB,0,0);  
    jscat(jsbuf,textColour);
  }
#endif // NOJSBUF  
}

void polBeg(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl)
{
  fnJsIntro(jsbuf,JSFNB,pol,ctl);
  fnHtmlIntro(buf,pol,ctl);
}

void polEnd(char* buf,char* jsbuf,uint8_t ctl)
{
  fnJsIntro(jsbuf,JSFNE,0,ctl&(~TRBEG)&(~TDBEG));
  strcat(buf,"</font>");
  fnHtmlEnd(buf,0,ctl);
}

void setFont(char* buf,const char* font,const char* size)
{
  
  if(font!=nullptr || size!=nullptr){strcat(buf,"<span style=\"");
    if(font!=nullptr){strcat(buf,"font-family:");strcat(buf,font);strcat(buf,";");}
    if(size!=nullptr){strcat(buf,"font-size:");strcat(buf,size);strcat(buf,"px;");}
    strcat(buf,"\">");
  }
 /*
 strcat(buf,"<p><font ");
 if(font!=nullptr){strcat(buf,"face=\"");strcat(buf,font);strcat(buf,"\"");}
 if(size!=0){strcat(buf," size=\"");strcat(buf,size);strcat(buf,"\"");}
 strcat(buf,">");
  */
}

void endFont(char* buf)
{
  strcat(buf,"</span>");
  //strcat(buf,"</font></p>");
}

/* ------------------ affichages (scrDsp....) ---------------------- */

void scrDspText(char* buf,char* jsbuf,const char* txt,uint16_t tdWidth,const char* font,uint8_t sizpol,uint8_t ctl)
// mode STRING :  ne fonctionne que dans une ligne de table
//                le premier appel comporte TDBEG pour forcer le nom de la commande et BRYES (ignoré dans ctl) ou TDEND 
//                TDEND est forcé dans le ctl la commande
//                les suivants BRYES ou TDEND
//                le dernier rien : TDEND est déjà dans le ctl de la commande (pour le format html ctl==STRING indique </td> à la fin)
//                CONCAT pour ajouter du texte à la volée
{

  // --------------------- debut traitement js
#ifndef NOJSBUF
  bool flagWidth=false;   // true si fnJsIntro() est effectué et tdWidth!=0
  
  uint8_t ctlb=ctl&~STRING;if((ctl&STRING)!=0){ctlb&=~BRYES;if((ctl&TDBEG)!=0){ctlb|=TDEND;}}
  if(((ctl&STRING)==0)||(((ctl&STRING)!=0)&&((ctl&TDBEG)!=0)&&((ctl&CONCAT)==0))){
    fnJsIntro(jsbuf,JSST,pol,ctlb);
    if(tdWidth!=0){flagWidth=true;}
  } 

  if(flagWidth==true){
    #define LWPX 12
    char widthPx[LWPX];memset(widthPx,0x00,LWPX);
    concatn(widthPx,styleTdWidth);
    jscat(jsbuf,JSSEP);jscat(jsbuf,widthPx,SEP);
  }
#endif // NOJSBUF
  char* dm0=jsbuf+strlen(jsbuf); // début texte+ctl pour html

  if(jsbuf!=nullptr){  // jscat ne copie pas si NOFSBUF
    strcat(jsbuf,txt);
    if(((ctl&STRING)!=0)&&((ctl&BRYES)!=0)){strcat(jsbuf,JSSBR);}
    if(((ctl&STRING)!=0)&&((ctl&TDEND)!=0)){strcat(jsbuf,JSSCO);}
  }    

  // -------------------- fin traitement js

  // -------------------- début traitement html
  styleTdWidth=tdWidth;                     // pour fnHtmlIntro()
  fnHtmlIntro(buf,font,sizpol,ctl,0);          // ajoute [<tr>][<td style width="nnnpx"]>]
  
  buftxcat(buf,dm0);                        // ajout texte (<br> et </td><td> décodés)
  
  if(ctl==STRING){ctl|=TDEND;}              // pas de JSSCO en fin de dm0 possible dans ce cas
  char a[]={JSSCO};
  if(*(dm0+strlen(dm0)-1)==*a){ctl&=~TDEND;}
  if((ctl&STRING)!=0){ctl&=~BRYES;sizpol=0;}   // évite doublon avec JSSBR de dm0 ; bloque le </font> (ajouter fontEnd à la fin du texte)
  fnHtmlEnd(buf,font,sizpol,ctl);                   // ajoute [<br>][</td>][</tr>]
}

void scrDspText(char* buf,char* jsbuf,const char* txt,uint16_t tdWidth,uint8_t sizpol,uint8_t ctl)
{
  scrDspText(buf,jsbuf,txt,0,nullptr,sizpol,ctl);
}

void scrDspText(char* buf,char* jsbuf,const char* txt,uint8_t sizpol,uint8_t ctl)
{
  scrDspText(buf,jsbuf,txt,0,nullptr,sizpol,ctl);
}

void affSpace(char* buf,char* jsbuf,uint8_t ctl)
{
  fnHtmlIntro(buf,0,ctl);
  strcat(buf," ");
  fnHtmlEnd(buf,0,ctl);
  fnJsIntro(jsbuf,JSSP,0,ctl);
}

void affSpace(char* buf,char* jsbuf)
{
  affSpace(buf,jsbuf,0);
}

void affRondJaune(char* buf,char* jsbuf,uint8_t ctl)
{
  fnHtmlIntro(buf,0,ctl);
  strcat(buf,"<div id=\"rond_jaune\"></div>");
  fnHtmlEnd(buf,0,ctl);

  fnJsIntro(jsbuf,JSRJ,0,ctl);
}

void concNum(char* buf,char* jsbuf,char type,uint8_t dec,void* value,bool sep)
{
  unsigned long v;
  switch (type){
    case 'b':concatn(buf,jsbuf,*(byte*)value,sep);break; //strcat(buf,(char*)valfonct);break;
    case 'd':concatn(buf,jsbuf,*(uint16_t*)value,sep);break;
    case 'D':concatn(buf,jsbuf,*(uint8_t*)value,sep);break;
    case 's':if(*(uint8_t*)value==0xff){break;}v=*(uint8_t*)value;concatn(buf,jsbuf,v,sep);break;
    case 'i':concatns(buf,jsbuf,*(int*)value,sep);break;
    case 'I':concatns(buf,jsbuf,*(int16_t*)value,sep);break;
    case 'r':concatnf(buf,jsbuf,(float)(*(int16_t*)value)/100,2,BRNO,sep);break;
    case 'l':concatns(buf,jsbuf,*(long*)value,sep);break;
    case 'f':concatnf(buf,jsbuf,*(float*)value,2,BRNO,sep);break;
    case 'F':concatnf(buf,jsbuf,*(float*)value,dec,BRNO,sep);break;
    case 'g':concatn(buf,jsbuf,*(uint32_t*)value,sep);break;    
    default:break;
  }
}

void checkColour(char* buf,char* jsbuf,char type,const void* value,const void* valmin,const void* valmax)
{
    char colour[6+1];
    char colour1[]={"black"};
    char colour2[]={"red"};
    memcpy(colour,colour1,6);
    if(type=='b' || type=='D' || type=='s'){
      if(*(uint8_t*)value<=*(uint8_t*)valmin || *(uint8_t*)value>=*(uint8_t*)valmax){memcpy(colour,colour2,4);}
    }
    else if(type=='d' || type=='r'){
      if(*(uint16_t*)value<=*(uint16_t*)valmin || *(uint16_t*)value>=*(uint16_t*)valmax){memcpy(colour,colour2,4);}
    }
    else if(type=='f' || type=='F'){
      if(*(float*)value<=*(float*)valmin || *(float*)value>=*(float*)valmax){memcpy(colour,colour2,4);}
    }
    else if(type=='I'){
      if((*(int16_t*)value<=*(int16_t*)valmin) || (*(int16_t*)value>=*(int16_t*)valmax)){memcpy(colour,colour2,4);}
    }
    else if(type=='i'){
      if((*(int*)value<=*(int*)valmin) || (*(int*)value>=*(int*)valmax)){memcpy(colour,colour2,4);}
    }
    else if(type=='l'){
      if(*(long*)value<=*(long*)valmin || *(long*)value>=*(long*)valmax){memcpy(colour,colour2,4);}
    }
    setColourB(buf,jsbuf,colour);
}

void scrDspNum(char* buf,char* jsbuf,char type,void* value,const void* valmin,const void* valmax,bool minmax,uint8_t dec,uint8_t sizpol,uint8_t ctl)
// mode STRING :  ne fonctionne que dans une ligne de table
//                le premier appel comporte TDBEG pour forcer le nom de la commande et BRYES (ignoré dans ctl) ou TDEND 
//                TDEND est forcé dans le ctl de la commande
//                les suivants BRYES ou TDEND
//                le dernier rien : TDEND est déjà dans le ctl de la commande
{
  
  uint8_t ctlb=ctl&~STRING;
  if((ctl&STRING)!=0){ctlb&=~BRYES;
  if((ctl&TDBEG)!=0){ctlb|=TDEND;}}
  if(((ctl&STRING)==0)||(((ctl&STRING)!=0)&&((ctl&TDBEG)!=0))){fnJsIntro(jsbuf,JSNT,sizpol,ctlb);} 
  
  fnHtmlIntro(buf,nullptr,sizpol,ctl,0);

  if(minmax){checkColour(buf,jsbuf,type,value,valmin,valmax);} 

  char* dm;dm=jsbuf+strlen(jsbuf);
  
  concNum(nullptr,jsbuf,type,dec,value,0);
  if(jsbuf!=nullptr){       // jscat ne copie pas si NOJSBUF
    if(((ctl&STRING)!=0) && ((ctl&BRYES)!=0)){strcat(jsbuf,JSSBR);}
    if(((ctl&STRING)!=0) && ((ctl&TDEND)!=0)){strcat(jsbuf,JSSCO);}
  }

  buftxcat(buf,dm);
  if((ctl&STRING)!=0){ctl&=~TDEND;}   // évite le double emploi avec JSSCO déjà dans dm
  if((ctl&STRING)!=0){ctl&=~BRYES;}   // évite le double emploi avec JSSBR déjà dans dm
  fnHtmlEnd(buf,sizpol,ctl);

  if(minmax){setColourE(buf,jsbuf);}
}

void scrDspNum(char* buf,char* jsbuf,char type,void* value,uint8_t dec,uint8_t sizpol,uint8_t ctl)
{
  char valmin[8],valmax[8];
  scrDspNum(buf,jsbuf,type,value,&valmin,&valmax,false,dec,sizpol,ctl);
}

void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,bool check,uint8_t dec,uint8_t ctl)
{ 
  fnJsIntro(jsbuf,JSNTI,0,ctl);
  fnHtmlIntro(buf,0,ctl,nullptr);

  if(check){checkColour(buf,jsbuf,'I',valfonct,valmin,valmax);}

  if(dec>4){dec=4;}
  uint16_t vfnt[5]={1,10,100,1000,10000};
  float vf=((float)*valfonct)/vfnt[dec]; 
  concatnf(buf,jsbuf,vf,dec,NOBR,SEPNO);
  fnHtmlEnd(buf,0,ctl);
  setColourE(buf,jsbuf);
}

/*
void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,uint8_t dec,uint8_t ctl)
{
  scrDspNum(buf,jsbuf,valfonct,valmin,valmax,true,dec,ctl);
}

void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,uint8_t ctl)
{
  scrDspNum(buf,jsbuf,valfonct,valmin,valmax,2,ctl);
}
*/

/* ----------------- structures (formulaires/tables) ---------------------- 

Les formulaires nécessitent :

1) une fonction de début : formIntro(buf,jsbuf,[f]onction],pol,ctl) à placer avant toute saisie et avant le bouton submit (scrGetButSub())
    
    génère 1 ou 2 arguments lors du submit : 

    user_ref_n=ttttt...         identifie l'utilisateur 'n' et donne la milli de la génération de la page 'tttt...' 
    xxxxxxxx_x=A                fonction facultative d'init spécifique au formulaire ("constructeur") ; A = PMFNCHAR+periCur
                                
2) une fonction de fin pour délimiter le formulaire : formEnd()

3) un submit : scrGetButSub()

Pour changer de page : 

    une fonction qui génère 2 arguments (par exemple scrGetButFn(), boutRetour())

    user_ref_n=ttttt...         id. ci-dessus 
    xxxxxxxx__=[yyyyy...]       xxx...xx__ la fonction qui envoie la page demandée ; yyy... un paramètre éventuel

*/

void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,const char* title,uint8_t pol,uint8_t ctl)       // les valeurs à recevoir systématiquement du navigateur
{
    if(buf!=nullptr){
          fnHtmlIntro(buf,pol,ctl);
          strcat(buf,"<form method=\"GET \">");
          if(title!=nullptr){
            strcat(buf,"<fieldset><legend>");strcat(buf,title);strcat(buf," :</legend>\n");
          }
          strcat(buf,"<p hidden><input type=\"text\" name=\"user_ref_");
          concat1a(buf,(char)(usernum+PMFNCHAR));
          strcat(buf,"\" value=\"");
          concatn(buf,usrtime[usernum]);
          strcat(buf,"\">");
 
          if(locfonc!=nullptr){
            char fonc[LENNOM+1];*fonc=0x00;
            memcpy(fonc,locfonc,LENNOM);fonc[LENNOM-1]=(char)(ninp+PMFNCVAL);fonc[LENNOM]='\0';
            strcat(buf,"<input type=\"text\" name=\"");strcat(buf,fonc);strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCHAR+periCur));strcat(buf,"\">");
          }
          strcat(buf,"</p>\n");
    }
#ifndef NOJSBUF    
    if(jsbuf!=nullptr){
          fnJsIntro(jsbuf,JSFBH,pol,ctl);
          jscatch(jsbuf,periCur+PMFNCVAL);
          jscatch(jsbuf,usernum+PMFNCVAL);          
          concatn(jsbuf,usrtime[usernum]);
          jscat(jsbuf,JSSEP);
          jscat(jsbuf,title,SEP);
          jscat(jsbuf,fonc,SEP);
    }
#endif // NOJSBUF    
}

void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,uint8_t pol,uint8_t ctl)
{
  formIntro(buf,jsbuf,locfonc,ninp,nullptr,pol,ctl);
}

void formIntro(char* buf,char* jsbuf,const char* locfonc,const char* title,uint8_t pol,uint8_t ctl)
{
  formIntro(buf,jsbuf,locfonc,0,title,pol,ctl);
}

void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t pol,uint8_t ctl)
{
  formIntro(buf,jsbuf,locfonc,0,nullptr,pol,ctl);
}

void formIntro(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl)
{
  formIntro(buf,jsbuf,nullptr,0,nullptr,pol,ctl);
}

void formEnd(char* buf,char* jsbuf,bool title,uint8_t pol,uint8_t ctl)
{
    if(buf!=nullptr){
          if(title){strcat(buf,"</fieldset>");}
          strcat(buf,"</form>\n");
          fnHtmlEnd(buf,pol,ctl);
    }
#ifndef NOJSBUF    
    if(jsbuf!=nullptr){
          if(title){fnJsIntro(jsbuf,JSFFS,pol,ctl&(~TDBEG)&(~TRBEG));}
          fnJsIntro(jsbuf,JSFE,pol,ctl&(~TDBEG)&(~TRBEG));
    }
#endif // NOJSBUF    
}

void formEnd(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl)
{
  formEnd(buf,jsbuf,0,pol,ctl);
}

void tableBeg(char* buf,char* jsbuf,const char* police,const char* size,bool border,const char* height,uint8_t ctl)
{
  char tBorder[]={'0','\"',0x00,'1','\"',0x00};

  strcat(buf,"<table border=\"");
  strcat(buf,tBorder+border*3);

  if(*height!='0' && *height!='\0'){strcat(buf," height=\"");strcat(buf,height);strcat(buf,"\"");}

  if(police!=nullptr || size!=nullptr){strcat(buf," style=\"");

    if(police!=nullptr){
      strcat(buf,"font-family:");strcat(buf,police);strcat(buf,";");}
  
    if(size!=nullptr){
      strcat(buf,"font-size:");strcat(buf,size);strcat(buf,"px;");}
    
    strcat(buf,"\">");}
  
#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSTB,0,ctl);
  jscat(jsbuf,police,SEP);
  if(border){jscat(jsbuf,"B");}
#endif // NOJSBUF  
  //fnHtmlIntro(buf,0,ctl);
  //fnHtmlEnd(buf,0,ctl);
}

void tableBeg(char* buf,char* jsbuf,const char* police,bool border,const char* height,uint8_t ctl)
{
  tableBeg(buf,jsbuf,police,nullptr,border,height,ctl);
}

void tableBeg(char* buf,char* jsbuf,const char* police,bool border,uint8_t ctl)
{
  tableBeg(buf,jsbuf,police,nullptr,border,"0",ctl);
}

void tableBeg(char* buf,char* jsbuf,bool border,uint8_t ctl)
{
  tableBeg(buf,jsbuf,nullptr,nullptr,border,"0",ctl);
}

void tableBeg(char* buf,char* jsbuf,uint8_t ctl)
{
  tableBeg(buf,jsbuf,nullptr,nullptr,true,"0",ctl);
}

void tableEnd(char* buf,char* jsbuf,uint8_t ctl)
{
  fnJsIntro(jsbuf,JSTE,0,ctl&(~TRBEG)&(~TDBEG));
  fnHtmlEnd(buf,0,ctl);
  strcat(buf,"</table>\n");
}

/* ----------------- saisies (scrGet...) --------------------- */

void scrGetText0(const char* fn,const char* htmlType,char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t sizfnt,uint8_t pol,uint8_t ctl)
{
  #define NOSIZE 100
  // -------------------- début traitement js
#ifndef NOJSBUF
  fnJsIntro(jsbuf,fn,pol,ctl);
  jscat(jsbuf,nomfonct,SEP);
  jscat(jsbuf,valfonct,SEP);
  #define LTT 6
  char tt[LTT];
  if(size<NOSIZE){
    memset(tt,0x00,LTT);concatn(tt,size);jscat(jsbuf,tt,SEP);}
  if(len!=0){
    memset(tt,0x00,LTT);concatn(tt,size);jscat(jsbuf,tt);}
#endif // NOJSBUF    
  // -------------------- fin traitement js

  // -------------------- début traitement html
  fnHtmlIntro(buf,pol,ctl);
  strcat(buf,"<input type=\"");strcat(buf,htmlType);strcat(buf,"\" name=\"");
  strcat(buf,nomfonct);
#ifndef NOJSBUF  
  jscat(jsbuf,nomfonct,SEP);
#endif
  strcat(buf,"\" value=\"");
  strcat(buf,valfonct);
#ifndef NOJSBUF  
  jscat(jsbuf,valfonct,SEP);
#endif  
  if(size<NOSIZE){strcat(buf,"\" size=\"");if(size==0){strcat(buf,"2");}else concatn(buf,jsbuf,size);}
  strcat(buf,"\"");
  if(sizfnt!=0){strcat(buf," style=\"font-size: ");concatn(buf,sizfnt);strcat(buf,"pt\"");}
  if(len!=0){strcat(buf," maxlength=\"");concatn(buf,jsbuf,len);strcat(buf,"\" ");}
  strcat(buf,">");
  fnHtmlEnd(buf,pol,ctl);
}

void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len,uint8_t pol,uint8_t ctl)
{
  scrGetText(buf,jsbuf,valfonct,nomfonct,0,len,pol,ctl);
}

void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t sizfnt,uint8_t pol,uint8_t ctl)
{
  scrGetText0(JSATB,"text",buf,jsbuf,valfonct,nomfonct,size,len,sizfnt,pol,ctl);
}

void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t pol,uint8_t ctl)
{
  scrGetText0(JSATB,"text",buf,jsbuf,valfonct,nomfonct,size,len,0,pol,ctl);
}

void scrGetHidden(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t pol,uint8_t ctl)
{
  scrGetText0(JSHID,"hidden",buf,jsbuf,valfonct,nomfonct,100,0,0,pol,ctl);
}

void scrGetNum(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t dec,uint8_t pol,uint8_t ctl)
{                          
  fnJsIntro(jsbuf,JSNTB,pol,ctl);
  fnHtmlIntro(buf,pol,ctl);
  strcat(buf,"<input type=\"text\" name=\"");
  strcat(buf,nomfonct);jscat(jsbuf,nomfonct,SEP);                         // nom
  strcat(buf,"\" value=\"");
  concNum(buf,jsbuf,type,dec,valfonct,SEP);                               // valeur

  jscatch(jsbuf,type);                                                    // type

  char sizeHtml='1';
  if(size!=0){sizeHtml=PMFNCVAL+size;}
  else {if(len>=3){sizeHtml='2';}if(len>=6){sizeHtml='4';}if(len>=9){sizeHtml='6';}}
  strcat(buf,"\" size=\"");concat1a(buf,jsbuf,sizeHtml);                  // size

  jscatch(jsbuf,dec+PMFNCVAL);                                            // dec

//  if(len<=2){
    strcat(buf,"\" id=\"nt");
    concatn(buf,len);         
    concatn(buf,dec);
//  }

  strcat(buf,"\" pattern=\"[-]{0-1}[");
  if(dec!=0){strcat(buf,".,");}strcat(buf,"0-9]{1,");
  if(len!=0){concatn(buf,jsbuf,len);}else{strcat(buf,"9");}               // len
  strcat(buf,"}\">");
  
  fnHtmlEnd(buf,pol,ctl);
}

void scrGetNum(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t dec,uint8_t pol,uint8_t ctl)
{
  scrGetNum(buf,jsbuf,type,valfonct,nomfonct,0,len,dec,pol,ctl);
}

void scrGetSelect(char* buf,char* jsbuf,char* val,char* ft,int nbre,int len,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl)
{            // val=table des libellés ; ft=fonction ; nbre ds table ; len step table ; sel=n°actuel ; nuv=n°param ; n°inp
  char a;
  int i,j;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);   
  ft[LENNOM-1]=(char)(ninp+PMFNCVAL);   
  
  fnHtmlIntro(buf,0,ctl);
  
  strcat(buf,"<SELECT name=\"");strcat(buf,ft);strcat(buf,"\">");
  for(i=0;i<nbre;i++){
    strcat(buf,"<OPTION");if(i==sel){strcat(buf," selected");};strcat(buf,">");
    for(j=0;j<len;j++){
      a=(char)val[i*len+j];
      if(a!=' '){concat1a(buf,a);}
    }
  }
  strcat(buf,"</SELECT>");
  fnHtmlEnd(buf,0,ctl);
  strcat(buf,"\n");

#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSSTB,0,ctl);
  jscatch(jsbuf,sel+PMFNCVAL);
  jscat(jsbuf,ft,SEP);
#endif // NOJSBUF
}

void scrGetSelect(char* buf,char* jsbuf,char* val,char* name,char* ft,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl)
{
  int nbre=*val-PMFNCVAL,len=*(val+1)-PMFNCVAL;
  scrGetSelect(buf,jsbuf,val+2,ft,nbre,len,sel,nuv,ninp,pol,ctl);
  jscat(jsbuf,name);
}

void optSelHtml(char* jsbuf,char* val,char* name)
{
#ifndef NOJSBUF  
  fnJsIntro(jsbuf,JSSOP,0,0);
  jscat(jsbuf,name);
  jscat(jsbuf,val);
#endif // NOJSBUF
}

void buttonCfg(char* buf,const char* font,char* sizfnt,uint8_t round,uint8_t margin,uint8_t bgcolor,uint8_t butSize,const char* butType)
{
  
  strcat(buf,"<input type=\"");strcat(buf,butType);strcat(buf,"\"");

  switch(butSize){  
    case 0:strcat(buf," style=\"height:20px;width:80px;font-size:12px;");break;
    case 1:strcat(buf," style=\"height:20px;width:80px;font-size:12px;");break;
    case 2:strcat(buf," style=\"height:30px;width:100px;font-size:15px;");break;
    case 3:strcat(buf," style=\"height:40px;width:150px;font-size:18px;");break;
    case 4:strcat(buf," style=\"height:50px;width:200px;font-size:20px;");break;
    case 5:strcat(buf," style=\"height:55px;width:250px;font-size:30px;");break;
    case 6:strcat(buf," style=\"height:60px;width:300px;font-size:40px;");break;
    case 7:strcat(buf," style=\"height:80px;width:400px;font-size:50px;");break;
    default: strcat(buf," style=\"height:80px;width:400px;font-size:50px;");break;
  }
  
    if(round!=0){
      strcat(buf,"  border-radius: ");
      if(round==1){strcat(buf,"5");}
      if(round==2){strcat(buf,"50");}
      strcat(buf,"px;");
    }

    if(margin!=0){strcat(buf," margin: 16px;");}
    if(sizfnt!=nullptr){strcat(buf," font-size:");strcat(buf,sizfnt);strcat(buf,"px;");}
    if(font!=nullptr){strcat(buf," font-family:");strcat(buf,font);strcat(buf,";");}

    const char* colNames[COLNAMENB];
    if(bgcolor<LIGHTVALUE){
      //colNames[0]="#28a745";    // disj
      colNames[0]="ForestGreen";  // disj
      //colNames[1]="#338FFF";    // on
      colNames[1]="DodgerBlue";   // on
      //colNames[2]="#dc3545";    // forced
      colNames[2]="Red";          // forced
      colNames[3]="Gold";         // push
      colNames[4]="Grey";         // off
      colNames[5]="Goldenrod";    // selected
      colNames[6]="ForestGreen";  // disj
      colNames[7]="";             // gris par défaut
      colNames[8]="";
      colNames[9]="LightGrey";    // std button
    }
    else {
      colNames[0]="PaleGreen";    // disj
      colNames[1]="LightCyan";    // on
      colNames[2]="LightPink";    // forced
      colNames[3]="LightYellow";  // push
      colNames[4]="LightGrey";    // off
      colNames[5]="NavajoWhite";  // selected
      colNames[6]="PaleGreen";  // disj
    }
    bgcolor%=LIGHTVALUE;

    //Serial.print(" bgcolor=");Serial.println(bgcolor);

    if(bgcolor!=0){
      strcat(buf,"background-color:");strcat(buf,colNames[bgcolor]);strcat(buf,";");
      strcat(buf,"border-color:");strcat(buf,colNames[bgcolor]);strcat(buf,";");
    }
    strcat(buf,"\"");
}

void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,const char* font,char* sizfnt,uint8_t butsize,uint8_t bgcolor,uint8_t margin,uint8_t round,uint8_t ctl)
/* génère user_ref_x=nnnnnnn...?ffffffffff=zzzzzz... */
{
#ifndef NOJSBUF
    //fnJsIntro(jsbuf,JSAC,0,0);
    fnJsIntro(jsbuf,JSBFB,sizfnt,ctl);
    jscatch(jsbuf,aligncenter+PMFNCVAL);
    jscatch(jsbuf,usernum+PMFNCVAL);
    concatn(jsbuf,usrtime[usernum]);
    jscat(jsbuf,JSSEP);
    jscat(jsbuf,nomfonct,SEP);jscat(jsbuf,valfonct,SEP);jscat(jsbuf,lib);
#endif // NOJSBUF

    fnHtmlIntro(buf,nullptr,0,ctl,0);
    char b[]={(char)(usernum+PMFNCHAR),'\0'};
    strcat(buf,"<a href=\"?user_ref_");
    strcat(buf,b);strcat(buf,"=");concatn(buf,usrtime[usernum]);
    strcat(buf,"?");
    //strcat(buf," rel=\"noreferrer\"?");
    //char* dm=buf+strlen(buf);
    strcat(buf,nomfonct);strcat(buf,"=");strcat(buf,valfonct);
    
    strcat(buf,"\">");
    if(aligncenter){strcat(buf,"<p align=\"center\">");}
    //char sizbuf[6];sizbuf[0]='\0';concatn(sizbuf,sizfnt);

    buttonCfg(buf,font,sizfnt,round,margin,bgcolor,butsize,"button");
/*
    if(sizfnt!=0){
      strcat(buf,"font-size:25px;font-family:Courier,sans-serif;");
      if(fntcolor!=1){strcat(buf,"color:White;");}
    }
*/
    strcat(buf,"\"");
    strcat(buf," value=\"");strcat(buf,lib);strcat(buf,"\"");

    if(aligncenter){strcat(buf,"></p></a>");}
    else{strcat(buf,"></a>");}
    
    fnHtmlEnd(buf,0,ctl);
}
void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t butsize,uint8_t bgcolor,uint8_t fntcolor,uint8_t margin,uint8_t round,uint8_t ctl)
{
  scrGetButFn(buf,jsbuf,nomfonct,valfonct,lib,aligncenter,"arial",nullptr,butsize,bgcolor,margin,round,ctl);
}

void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t butsize,uint8_t ctl)
{
  scrGetButFn(buf,jsbuf,nomfonct,valfonct,lib,aligncenter,"arial",nullptr,butsize,0,0,0,ctl);
}

void scrGetButRet(char* buf,char* jsbuf,const char* lib,uint8_t ctl)
{
    fnHtmlIntro(buf,0,ctl);
    strcat(buf,"<a href=\"?user_ref_");
    concat1a(buf,(char)(usernum+PMFNCHAR));strcat(buf,"=");concatn(buf,usrtime[usernum]);
    strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
    strcat(buf,lib);
    strcat(buf,"\"></a>\n");
    fnHtmlEnd(buf,0,ctl);
#ifndef NOJSBUF
    fnJsIntro(jsbuf,JSBRB,0,ctl);
    jscat(jsbuf,lib);
#endif // NOJSBUF    
}

void scrGetButRef(char* buf,char* jsbuf,const char* nomfonct,const uint8_t suffn,uint8_t ctl)
{
    fnHtmlIntro(buf,0,ctl);
    strcat(buf,"<a href=\"?user_ref_");
    concat1a(buf,(char)(usernum+PMFNCHAR));strcat(buf,"=");concatn(buf,usrtime[usernum]);
    strcat(buf,"?");
    char suffix[2];suffix[0]=(char)(suffn+PMFNCHAR);suffix[1]='\0';
    strcat(buf,nomfonct);strcat(buf,suffix),strcat(buf,"=");
    
    //strcat(buf,"\">");
    strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
    strcat(buf,"refresh");
    strcat(buf,"\"></a>\n");
    fnHtmlEnd(buf,0,ctl);
#ifndef NOJSBUF
    fnJsIntro(jsbuf,JSBRB,0,ctl);
    jscat(jsbuf,lib);
#endif // NOJSBUF    
}


void scrGetButSub(char* buf,char* jsbuf,const char* lib,bool aligncenter,uint8_t butsize,uint8_t ctl)
{
#ifndef NOJSBUF  
  fnJsIntro(jsbuf,JSBMB,0,ctl);
  jscat(jsbuf,lib);
  if(aligncenter){jscat(jsbuf,JSSEP);jscat(jsbuf,"A");}
#endif // NOJSBUF

  fnHtmlIntro(buf,0,ctl);
  if(aligncenter){strcat(buf,"<p align=\"center\">");}
  buttonCfg(buf,"nullptr",nullptr,0,0,0,butsize,"submit");
  /*
  if(sizfnt==1){strcat(buf," text style=\"width:300px;height:60px;font-size:40px\"");}
  if(sizfnt==7){strcat(buf," style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\" ");}
  */
  strcat(buf," value=\"");
  strcat(buf,lib);
  strcat(buf,"\">");
  if(aligncenter){strcat(buf,"</p>");}
  fnHtmlEnd(buf,0,ctl);
}

void scrGetButSub(char* buf,char* jsbuf,const char* lib,uint8_t ctl)
{
  scrGetButSub(buf,jsbuf,lib,ALICNO,0,ctl);
}

void scrGetRadiobut(char* buf,char* jsbuf,byte valeur,char* nomfonct,uint8_t nbval,bool vert,char* lib,uint8_t pol,uint8_t ctl)        // nbval boutons radio
{
#ifndef NOJSBUF                                                                                             // valeur = checked (0-n) 
      fnJsIntro(jsbuf,JSRAD,pol,ctl);
      jscat(jsbuf,nomfonct,SEP);
      jscatch(jsbuf,nbval+PMFNCVAL);
      jscatch(jsbuf,valeur+PMFNCVAL);
#endif // NOJSBUF

      fnHtmlIntro(buf,pol,ctl);
      //concatns(buf,valeur);
      for(uint8_t j=0;j<nbval;j++){
          strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,nomfonct);
          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+j));strcat(buf,"\"");
          if(j==valeur){strcat(buf," checked");}strcat(buf,"/>");
          if(lib!=nullptr){strcat(buf,lib);lib+=strlen(lib)+1;}
          if(vert){strcat(buf,"<br>");}
      }
      fnHtmlEnd(buf,pol,ctl);
}

void scrGetRadiobut(char* buf,char* jsbuf,byte valeur,char* nomfonct,uint8_t nbval,uint8_t pol,uint8_t ctl)        // nbval boutons radio
{
  return scrGetRadiobut(buf,jsbuf,valeur,nomfonct,nbval,0,nullptr,pol,ctl);        // nbval boutons radio
}
void yscrGetRadiobut(char* buf,char* jsbuf,byte valeur,const char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t ctl)                    
{                        // sqbr square button // 1 line ; nb = page row  // nbval à traiter
  valeur&=0x03;                                                               

#ifndef NOJSBUF                                                                                                            
  fnJsIntro(jsbuf,JSRADS,0,ctl);                                        
  jscat(jsbuf,nomfonct,SEP);
  jscatch(jsbuf,nb+PMFNCVAL);
  jscatch(jsbuf,valeur+PMFNCVAL);
  if(vert){jscat(jsbuf,"V");}else{jscat(jsbuf,"H");}
#endif // NOJSBUF

  fnHtmlIntro(buf,0,ctl);
  strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_off\" id=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+0));strcat(buf,"\"");
  if(valeur==0){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">OFF</label>\n");
  
  if(vert){strcat(buf,"<br><br>\n");}
  
  strcat(buf," <input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_on\" id=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+1));strcat(buf,"\"");
  if(valeur==1){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">ON</label>\n");

  strcat(buf," <input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_for\" id=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+2));strcat(buf,"\"");
  if(valeur==2 || valeur==3){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">FOR</label>\n");

  fnHtmlEnd(buf,0,ctl);
}

void jpgIntro0(char* dm)
{
  char* dm0=dm;
  strcat(dm,"HTTP/1.1 200 OK\n");
  strcat(dm,"CONTENT-Type: image/png\n");
  if(strlen(dm0)>JPGINTROLEN){ledblink(BCODEPERIRECLEN,PULSEBLINK);}
}

void htmlEnd(char* buf,char* jsbuf)
{
  if(buf!=nullptr){strcat(buf,"</body></html>\n");}
  fnJsIntro(jsbuf,JSHE,0,0);
}

/*void htmlIntro0(char* dm)    // suffisant pour commande péripheriques
{
  strcat(dm,"HTTP/1.1 200 OK\n");
  //cli->println("Location: http://82.64.32.56:1789/");
  //cli->println("Cache-Control: private");
  strcat(dm,"CONTENT-Type: text/html; charset=UTF-8\n");
  strcat(dm,"Connection: close\n\n");
  strcat(dm,"<!DOCTYPE HTML ><html>\n");
}*/

/* ---------------------- styles --------------------- 

void htmlStyleCbBut(char* buf)
{
  if(buf!=nullptr){
            strcat(buf,"#nt1{width:10px;}\n");    // inutilisé ?
            strcat(buf,"#nt2{width:18px;}\n");    // dans jsPeriCur() seult ?

            // check box 
            strcat(buf,"#cb1{width:10px; padding:0px; margin:0px; text-align: center};\n");
            strcat(buf,"#cb2{width:20px; text-align: center};\n");

            // buttons 
            strcat(buf,".button {background-color: #195B6A; border: none; color: white; padding: 32px 80px:"); //16px 40px;");
            strcat(buf,"text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}\n");
            strcat(buf,".button2 {background-color: #77878A;}\n");
  }
}

void htmlStyleSliders(char* buf)
{
  if(buf!=nullptr){  
            // big sliders 
            strcat(buf,".switch {position: relative;display: inline-block;width: 220px;height: 100px; margin: 16px;}\n");
            strcat(buf,".switch input {opacity: 0;width: 0;hight: 0;}\n");
            strcat(buf,".slider {position: absolute;cursor: pointer;");
            strcat(buf,"  top: 0;left: 0;right: 0;bottom: 0;background-color: #ccc;-webkit-transition: .4s;transition: .4s;}\n");
            strcat(buf,".slider:before {position: absolute;content: \"\";");
            strcat(buf,"  height: 84px;width: 84px;left: 8px;bottom: 8px;background-color: white;-webkit-transition: .4s;transition: .4s;}\n");

            strcat(buf,"input:checked + .slider {background-color: #2196F3;}\n");
            strcat(buf,"input:focus + .slider {box-shadow: 0 0 1px #2196F3;}\n");
            strcat(buf,"input:checked + .slider:before {-webkit-transform: translateX(110px);-ms-transform: translateX(55px);transform: translateX(110px);}\n");
            // Rounded sliders 
            strcat(buf,".slider.round {border-radius: 50px;}\n");
            strcat(buf,".slider.round:before {border-radius: 50%;}\n");
  }
}

void  htmlStyleSqrBut(char* buf) 
{
  if(buf!=nullptr){  
            // pour bouton radio carrés 
            strcat(buf,"@import url(\"https://fonts.googleapis.com/css?family=Roboto:400,400i,700\");\n");
            //strcat(buf,":root {  --txt-color: #00B7E8;}\n");
            //strcat(buf,"body { margin: 2rem;font-family: Roboto, sans-serif;}\n");
            strcat(buf,".content {display:flex;flex-wrap:wrap;gap:1rem;justify-content:flex-start;}\n");
            strcat(buf,".content > div{flex-basis:200px;border:1px solid #ccc;padding:1rem;box-shadow: 0px 1px 1px rgba(0,0,0,0.2), 0px 1px 1px rgba(0,0,0,0.2);}\n");
            //strcat(buf,"h1{font-weight: normal; color: var(--txt-color);}\n");
            //strcat(buf,"h2{font-size: 1.1rem;color: var(--txt-color);font-weight: normal;text-transform: uppercase;margin:0 0 2rem;border-bottom: 1px solid #ccc;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr {display: none;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr + label {padding: 0.5rem 1rem;font-size: 1.50rem;line-height: 1.5;border-radius: 0.3rem;color: #fff;background-color: #6c757d;border: 1px solid transparent;transition: all 0.15s ease-in-out;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_off:hover + label { background-color: #28a745;border-color: #28a745;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_off:hover + label { background-color: #218838;border-color: #1e7e34;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_off:checked + label { background-color: #28a745;border-color: #28a745;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_on:hover + label { background-color: #338fff;border-color: #338fff;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_on:hover + label { background-color: #338fff;border-color: #bd2130;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_on:checked + label { background-color: #338fff;border-color: #338fff;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_for:hover + label { background-color: #dc3545;border-color: #dc3545;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_for:hover + label { background-color: #c82333;border-color: #bd2130;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_for:checked + label { background-color: #dc3545;border-color: #dc3545;}\n");
  
            // rond jaune 
            strcat(buf,"#rond_jaune {width: 40px;height: 40px;border-radius: 20px;background: yellow;}");
  }
}*/

/*void htmlBegE(char* buf,EthernetClient* cli)
{
  if(buf!=nullptr){
      strcat(buf,"</style>\n");  
      strcat(buf,"</head>\n");
      strcat(buf,"<body>\n");
      ethWrite(cli,buf);
      borderparam=NOBORDER;   // force l'init du navigateur
  }
}*/

void pageIntro0(char* buf,char* jsbuf)
{
#ifndef NOJSBUF  
  jscat(jsbuf,introHttp);
#endif // NOJSBUF
  strcat(buf,introHttp);
}

void pageIntro(char* buf,char* jsbuf,char* titre)
{
  if(buf!=nullptr){
#ifndef NOJSBUF      
      char* dm=buf+strlen(buf);
#endif // NOJSBUF
      strcat(buf,"<head>");
      char locbuf[10]={0};
      if(perrefr!=0){strcat(buf,"<meta HTTP-EQUIV=\"Refresh\" content=\"");sprintf(locbuf,"%d",perrefr);strcat(buf,locbuf);strcat(buf,"\">");}
      if(titre!=nullptr){strcat(buf,"<title>");strcat(buf,titre);strcat(buf,"</title>\n");}
      strcat(buf,"<style>");
#ifndef NOJSBUF      
      jscat(jsbuf,dm);
#endif // NOJSBUF      
  }
}

void htmlBeg(char* buf,char* jsbuf,char* titre,char rem)    // en-tête de page HTTP/HTML
{   
  pageIntro0(buf,jsbuf);                        // en-tête HTTP
  pageIntro(buf,jsbuf,titre);                   // en-tête HTML

#ifndef NOJSBUF  
  scrRecall(jsbuf,GENSTYLE);                    // style général
  if(rem=='R'){scrRecall(jsbuf,REMOTESTYLE);}    // style remote
  scrRecall(jsbuf,INTROHTMLE);                  // fin en-tête
#endif // NOJSBUF

  strcat(buf,generalSt);
  strcat(buf,"\n");
  if(rem=='R'){strcat(buf,remoteSt);strcat(buf,"\n");}
  strcat(buf,introHtmlE);

  borderparam=NOBORDER;   // force l'init du navigateur
}

void htmlBeg(char* buf,char* jsbuf,char* titre) //,EthernetClient* cli)
{
  htmlBeg(buf,jsbuf,titre,' ');
}

void pageLineOne(char* buf,char* jsbuf)         
{ 
  //Serial.println("pageLineOne ");
  float th;                                  // pour temp DS3231
  char dm0[120];*dm0=0x00;
  
  ds3231.readTemp(&th);
  
  strcat(dm0,VERSION);
  #ifdef DUE
  strcat(dm0," DUE ");
  #endif // DUE
  #ifndef DUE
  strcat(dm0," NUC ");
  #endif // DUE
  #ifdef _MODE_DEVT
  strcat(dm0," DEV ");
  #endif // DEVT
  #ifndef _MODE_DEVT
  strcat(dm0," RUN ");
  #endif // DEVT

  scrDspText(buf,jsbuf,serverName,0,nullptr,0,0);
  affSpace(buf,jsbuf);
  bufPrintDateHeure(dm0,nullptr,pkdate);
  scrDspText(buf,jsbuf,dm0,0,nullptr,0,0);
  
  *dm0=0x00;
  uint32_t bufIp=Ethernet.localIP();
  strcat(dm0," ; local IP ");
  charIp(dm0,(char*)&bufIp,nullptr);strcat(dm0," ");
  concatnf(dm0,nullptr,th,2,NOBR,SEPNO);strcat(dm0,"°C ");
  scrDspText(buf,jsbuf,dm0,10,BRYES);
}

void scrGetCheckbox(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,const char* lib,uint8_t pol,uint8_t ctl)
{
  if(etat!=NO_STATE && !(*val & 0x01)){etat=2;}

#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSDB,pol,ctl);
  jscat(jsbuf,nomfonct,SEP);
  if((*val & 0x01)!=0){jscat(jsbuf,JSCHK);}
  jscat(jsbuf,lib,SEP);
  jscatch(jsbuf,etat+PMFNCVAL);
#endif // NOJSBUF

  fnHtmlIntro(buf,pol,ctl);
  strcat(buf,"<input type=\"checkbox\" name=\"");
  strcat(buf,nomfonct);
  strcat(buf,"\" id=\"cb1\" value=\"1\"");
  if((*val & 0x01)!=0){strcat(buf," checked");}
  strcat(buf,">");
  if(lib!=nullptr){strcat(buf,lib);}
  
  if(etat!=NO_STATE){concatn(nullptr,jsbuf,(unsigned long)etat);}
  switch(etat){
    case 2 :strcat(buf,"___");break;
    case 1 :strcat(buf,"_ON");break;
    case 0 :strcat(buf,"OFF");break;
    default:break;
  }
  fnHtmlEnd(buf,pol,ctl);
}

void scrGetCheckbox(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib)
{
  scrGetCheckbox(buf,jsbuf,val,nomfonct,etat,lib,0,td);
}

void scrGetCheckbox(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib)
{
  scrGetCheckbox(buf,nullptr,val,nomfonct,etat,td,lib);
}

void sliderFHtml(char* buf,char* jsbuf,uint8_t* val,const char* nf,int sqr,uint8_t ctl)
{
  scrGetButFn(buf,jsbuf,nf,"","",ALICNO,4,TDBEG);
}

void sliderBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nf,int sqr,uint8_t ctl)
{
  //char nf[LENNOM+1];nf[LENNOM]='\0';
  //memcpy(nf,nomfonct,LENNOM);if(nb>=0){nf[LENNOM-1]=(char)(nb+PMFNCHAR);}

#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSSLD,0,ctl);
  jscat(jsbuf,nf,SEP);
  if((*val & 0x01)!=0){jscat(jsbuf,JSCHK);}
  if(sqr==0){strcat(jsbuf,"R");}
#endif // NOJSBUF

  fnHtmlIntro(buf,0,ctl);
  strcat(buf,"<label class=\"switch\"><input type=\"checkbox\" style=\"color:Khaki;\" name=\"");strcat(buf,nf);strcat(buf,"\" value=\"1\"");
  if((*val & 0x01)!=0){strcat(buf," checked");}
  strcat(buf," ><span class=\"slider ");if(sqr==0){strcat(buf," round");}
  strcat(buf,"\"></span></label>");
  
  fnHtmlEnd(buf,0,ctl);
}

uint8_t mDSval(uint8_t num)
{
  uint8_t ret=0;
  //for(uint8_t i=0;i<MDSLEN;i++){if(((memDetServ[i]) & (mDSmaskbit[num*MDSLEN+i])) !=0){ret=1;break;}}
  uint8_t mi=num>>3;if(((memDetServ[mi]) & (mDSmaskbit[num*MDSLEN+mi])) !=0){ret=1;}
  //Serial.print(">>>>> ctl mDSval ");dumpstr((char*)(memDetServ-4),9);
  return ret;
}

void subDSnBm(char* buf,char* jsbuf,const char* fnc,uint8_t* val,uint8_t num,char* lib) // checkbox transportant 1 bit 
                                                                    // num le numéro du bit dans le mot
                                                                    // le caractère LENNOM-1 est le numéro du bit(+PMFNCHAR) dans periDetServ 
{                                                                   // le numéro est codé 0 à 15 + 0x40 et 16->n + 0x50 !!!! (évite les car [\]^ )
  char fonc[LENNOM+1];
  memcpy(fonc,fnc,LENNOM+1);
  uint8_t numbyte=num>>3;
  uint8_t val0=(*(val+numbyte)>>(num-(numbyte<<3)))&0x01; // mDSval(num); //(val>>num)&0x01;
  //Serial.print(">>>>> subDSnBm ctl ");//dumpstr((char*)(val-4),MDSLEN);
  //Serial.print(num);Serial.print(":");Serial.print((*((uint32_t*)val)>>num)&0x01);Serial.print('/');Serial.println(val0);
  //if(num>=16){num+=16;}
  fonc[LENNOM-1]=(char)(PMFNCVAL+num);
  scrGetCheckbox(buf,jsbuf,&val0,fonc,NO_STATE,lib,0,0);
}

void scrStore(char* jsbuf,char name,const char* data)
{
#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSSCST,0,0);
  jscatch(jsbuf,name);
  jscat(jsbuf,data);
#endif // NOJSBUF  
}

void scrRecall(char* jsbuf,char name)
{
#ifndef NOJSBUF
  fnJsIntro(jsbuf,JSSCRC,0,0);
  jscatch(jsbuf,name);
#endif // NOJSBUF  
}

void cliPrintMac(EthernetClient* cli, byte* mac)
{
  char macBuff[18];
  unpackMac(macBuff,mac);
  cli->print(macBuff);
}

uint8_t bufPrintPeriDate(char* buf,char* periDate)
{
  char dateascii[LDATEASCII];
  int j;
  unpackDate(dateascii,periDate);for(j=0;j<LDATEASCII;j++){concat1a(buf,dateascii[j]);if(j==5){strcat(buf," ");}}strcat(buf,"<br>\n");
  return LDATEASCII+1;
}

void trailingSpaces(char* data,uint16_t len)
{
  for(int i=len-1;i>=0;i--){if(data[i]==' ' || data[i]=='\0'){data[i]='\0';}else break;} // erase trailing spaces
}

void bufLenShow(char* buf,char* jsbuf,uint16_t lb,unsigned long begTPage)
{
  Serial.print("  len buf=");Serial.print(lb);
  Serial.print("  len jsbuf=");Serial.print(strlen(jsbuf));
  Serial.print("  ms=");Serial.println(millis()-begTPage);
#ifdef DEBUG_ON
  delay(10);
#endif  
}