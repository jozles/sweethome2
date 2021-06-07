#ifndef _UTILHTML_H_
#define _UTILHTML_H_

#define ALIC   VRAI
#define ALICNO FAUX
#define NOBR   FAUX
#define BR     VRAI
#define NOFORM (bool)false
#define HID  (bool)true
#define CRLF (bool)true
#define LF     0x0A
#define CTLCH  0x20
#define BRMASK 0x10
#define BRYES  0x10
#define BRNO   0
#define TRMASK 0x0c
#define TRBEG 0x04
#define TREND 0x08
#define TRBE  0x0C
#define TRNO  0
#define TDMASK 0x03
#define TDEND 0x02
#define TDBE  0x03
#define TDBEG 0x01
#define TDNO  0
#define SEP   VRAI
#define SEPNO FAUX
#define PV    VRAI
#define NOPV  FAUX

#define JSB strcat(jsbuf,
#define JSE );

/* commandes interprétées par javascript pour produire du html 
   de la forme ~JSAAAA; tilde + 3 à 6 caractères et ";"
   0 à n arguments (éventuellement optionnels) suivent séparés par ";"
*/
#define JSSEP  "|"            // séparateur interne aux fonction
#define JSCHK  "^"            // checked

#define JSCOB  "~wxp"         // couleur                 JSCOBcouleur;
#define JSCOE  "~W"

#define JSFNE  "~X"           // police fin

#define JSHIDB "~hxp"         // hide 
#define JSHIDE "~H"   
#define JSAC   "~V"           // align center

#define JSBRB  "~rxp"         // bouton retour 
#define JSBRE  "~R"
#define JSBMB  "~mxp"         // bouton Maj 
#define JSBME  "~M"
#define JSBFB  "~bxp"         // bouton fonct            JSBFBnomfonct|valfonct|size|lib
#define JSBFE  "~B"
#define JSNTB  "~nxp"         // saisie numtf            JSNTBnomfonct|len|dec|typevaleur
#define JSNTX  "~Nxp"         // affichage num (min/max) JSNTXvalfonct/100
#define JSDB   "~dxp"         // saisie cb               JSDBnomfonct|lib|etat
#define JSDE   "~D;"  
#define JSATB  "~axp"         // saisie alphaTableHtmlB  JSATBnomfonct|valfonct|len
#define JSATE  "~A"         
#define JSST   "~sxp"         // affichage texte         JSSTtexte

#define JSFUB  "~u;"          // usrPeriCurB beg
#define JSFUE  "~U;\n"        // usrPeriCurB end
#define JSTB   "~t;"          // debut table
#define JSTE   "~T;"          // fin table (crlf manuel dans jscat)
#define JSTBL  "~q"    
#define JSLB   "~l"       
#define JSLE   "~L"
#define JSCB   "~c"
#define JSCE   "~C"  
#define JSCEB  "~K"
#define JSCEL  "~k"
#define JSCELT "~Q"
#define JSFB   "~f;"          // début formulaire [titre si encadrement]
#define JSFF   "~F;\n"        // fin formulaire
#define JSBR   "~p;\n"        // <br>
#define JS2BR  "~o;\n"        // <br><br>


void bufcat(char* buf,char* jsbuf,const char* s);
void jscat(char* jsbuf,const char* s,bool sep);
void jscat(char* jsbuf,const char* s);
void fnHtmlIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl);
void fnHtmlEnd(char* buf,uint8_t pol,uint8_t ctl);
void fnJsIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl);

void pageHeader(char* buf);
void pageHeader(char* buf,char* jsbuf);
void pageHeader(char* buf,bool form);
void pageHeader(char* buf,char* jsbuf,bool form);
void cliPrintMac(EthernetClient* cli, byte* mac);
void trailingSpaces(char* data,uint16_t len);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet);

void selectTableBHtml(char* buf,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t td);
void usrPeriCurB(char* buf,const char* fnct,uint8_t ninp,int len,uint8_t td);
void usrPeriCurB(char* buf,char* jsbuf,const char* fnct,uint8_t ninp,int len,uint8_t td);
void subDSnB(char* buf,const char* fnc,uint32_t val,uint8_t num,char* lib);
void checkboxTableBHtml(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void usrFormBHtml(char* buf,bool hid);
void usrFormBHtml(char* buf,char* jsbuf,bool hid);
void usrFormInitBHtml(char* buf,const char* nomfonct);

void fontBeg(char* buf,char* jsbuf,uint8_t fntSiz,uint8_t td);
void fontEnd(char* buf,char* jsbuf,uint8_t td);
void concatIp(char* buf,byte* ip);
void concat1a(char* buf,char a);
void concat1a(char* buf,char* jsbuf,char a);
void concat1aH(char* buf,char a);
void concatn(char* buf,unsigned long val);
void concatn(char* buf,char* jsbuf,unsigned long val);
void concatns(char* buf,long val);
void concatns(char* buf,char* jsbuf,long val);
void concatns(char* buf,char* jsbuf,long val,bool sep);
void concatnf(char* buf,char* jsbuf,float val);
void concatnf(char* buf,char* jsbuf,float val,uint8_t dec);
void concatnf(char* buf,float val);
void concatnf(char* buf,float val,uint8_t dec);
void concatnf(char* buf,char* jsbuf,float val,uint8_t dec,bool br);
void concatnf(char* buf,char* jsbuf,float val,uint8_t dec,bool br,bool sep);
void concatDate(char* buf,char* periDate);
void concatDate(char* buf,char* jsbuf,char* periDate);
void bufPrintPeriDate(char* buf,char* periDate);
void bufPrintDateHeure(char* buf,char* pkdate);
void bufPrintDateHeure(char* buf,char* jsbuf,char* pkdate);
void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol);
void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec,bool br);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t dec,uint8_t pol,uint8_t ctl);
//void textTbl(char* buf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t ctl);
//void textTbl(char* buf,char* jsbuf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t ctl);
void affText(char* buf,char* jsbuf,char* txt,uint8_t pol,uint8_t ctl);
void affNum(char* buf,char* jsbuf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t ctl);
void alphaTableHtmlB(char* buf,const char* valfonct,const char* nomfonct,int len);
//void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len);
void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len,uint8_t pol,uint8_t ctl);
//void setCol (char* buf,const char* textColour);
void boutRetourB(char* buf,const char* lib,uint8_t td,uint8_t br);
void boutRetourB(char* buf,char* jsbuf,const char* lib,uint8_t ctl);
void boutF(char* buf,const char* nomfonct,const char* valfonct,const char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter);
void boutF(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t sizfnt,uint8_t ctl);
void boutMaj(char* buf,char* jsbuf,const char* lib,uint8_t td);
void radioTableBHtml(char* buf,byte valeur,char* nomfonct,uint8_t nbval);
void yradioTableBHtml(char* buf,byte valeur,const char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t td);
void setColourB(char* buf,const char* textColour);
void setColourB(char* buf,char* jsbuf,const char* textColour);
void setColourE(char* buf);
void setColourE(char* buf,char* jsbuf);
void sliderBHtml(char* buf,uint8_t* val,const char* nomfonct,int nb,int sqr,uint8_t td);
void htmlIntro0B(char* buf);
void htmlIntroB(char* buf,char* titre,EthernetClient* cli);

#endif // _UTILHTML_H_
