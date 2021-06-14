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
#define CTLCH  0x40     
#define CTLPO  0x20     // byte police present
#define BRMASK 0x10
#define BRYES  0x10
#define BRNO   0
#define TRMASK 0x0c     
#define TRBEG 0x04      // délicat à modifier (voir les ~TRBEG)   
#define TREND 0x08      // délicat à modifier (voir les ~)   
#define TRBE  0x0C      // délicat à modifier (voir les ~)
#define TRNO  0
#define TDMASK 0x03
#define TDEND 0x02      // délicat à modifier (voir les ~)
#define TDBE  0x03      // délicat à modifier (voir les ~)
#define TDBEG 0x01      // délicat à modifier (voir les ~)
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
#define JSFON  "{"        // séparateur début fonction  
#define JSSEP  "}"        // séparateur interne aux fonction
#define JSSBR  "|"        // séparateur colonne dans les chaines texte
#define JSLF   "~"        // séparateur <br> dans les chaines texte 
#define JSCHK  "^"        // checked 
#define JSCTL  "x"        // troisieme car optionnel de fonction (reçoit ctl) 

#define JSCOB  "wx"       // couleur                 JSCOBcouleur;
#define JSCOE  "W "

#define JSFNE  "X "       // police fin

#define JSHIDB "hx"       // hide 
#define JSHIDE "H "   
#define JSAC   "V "       // align center

#define JSBRB  "rx"       // bouton retour 
#define JSBRE  "R "
#define JSBMB  "mx"       // bouton Maj 
#define JSBME  "M "
#define JSBFB  "bx"       // bouton fonct            JSBFBnomfonct|valfonct|size|lib
#define JSBFE  "B "
#define JSNTB  "nx"       // saisie numtf            JSNTBnomfonct|len|dec|typevaleur

#define JSDB   "dx"       // saisie cb               JSDBnomfonct}lib}[JSCHK][etat]
#define JSDE   "D "       // saisie cb fin  
#define JSATB  "ax"       // saisie alphaTableHtmlB  JSATBnomfonct|valfonct|len
#define JSSP   "S "       // affichage space  
#define JSST   "sx"       // affichage texte         JSSTtexte
#define JSNT   "Ux"
#define JSNTI  "ux"       // affichage num (min/max) JSNTXvalfonct/100

#define JSUSR  "r "       // usrPeriCurB
#define JSFUE  "U "      
#define JSTB   "t "       // debut table
#define JSTE   "T "       // fin table (crlf manuel dans jscat)
#define JSTBL  "q "    
#define JSLB   "l "       
#define JSLE   "L "
#define JSCB   "c "
#define JSCE   "C "  
#define JSSTB  "kx"       // selectTable         
#define JSCEL  "Kx"
#define JSCELT "Q "
#define JSFBH  "gx"       // header formulaire
#define JSFB   "fx"       // début formulaire [titre si encadrement]
#define JSFE   "Fx"       // fin formulaire


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
void formBeg(char* buf,char*jsbuf,const char* title);
void formBeg(char* buf,char*jsbuf);
void formHeader(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void formEnd(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void cliPrintMac(EthernetClient* cli, byte* mac);
void trailingSpaces(char* data,uint16_t len);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet);

void selectTableBHtml(char* buf,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t td);
void selectTableBHtml(char* buf,char* jsbuf,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t td);
void usrPeriCurB(char* buf,const char* fnct,uint8_t ninp,int len,uint8_t td);
void usrPeriCurB(char* buf,char* jsbuf,const char* fnct,uint8_t ninp,int len,uint8_t td);
void subDSnB(char* buf,const char* fnc,uint32_t val,uint8_t num,char* lib);
#define NO_STATE 9         // etat
void checkboxTableBHtml(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,const char* lib,uint8_t pol,uint8_t ctl);
void usrFormBHtml(char* buf,bool hid);
void usrFormBHtml(char* buf,char* jsbuf,bool hid);
void usrFormInitBHtml(char* buf,const char* nomfonct);

void fontEnd(char* buf,char* jsbuf,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,uint8_t ctl);
void tableEnd(char* buf,char* jsbuf,uint8_t ctl);
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
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec,bool br);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t dec,uint8_t pol,uint8_t ctl);
void affSpace(char* buf,char* jsbuf);
void affText(char* buf,char* jsbuf,const char* txt,uint8_t pol,uint8_t ctl);
void affNum(char* buf,char* jsbuf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t ctl);
void affNum(char* buf,char* jsbuf,char type,void* value,uint8_t dec,uint8_t pol,uint8_t ctl);
void alphaTableHtmlB(char* buf,const char* valfonct,const char* nomfonct,int len);
void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len,uint8_t pol,uint8_t ctl);
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
