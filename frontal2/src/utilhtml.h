#ifndef _UTILHTML_H_
#define _UTILHTML_H_

#include <Ethernet.h>

#define NOFORM (bool)false

#define BORDER VRAI
#define NOBORDER FAUX

void bufcat(char* buf,char* jsbuf,const char* s);
void jscat(char* jsbuf,const char* s,bool sep);
void jscat(char* jsbuf,const char* s);
void fnHtmlIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl);
void fnHtmlEnd(char* buf,uint8_t pol,uint8_t ctl);
void fnJsIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl);

//void pageHeader(char* buf);
void htmlEnd(char* buf,char* jsbuf);
void pageHeader(char* buf,char* jsbuf);
void pageHeader(char* buf,bool form);
void pageHeader(char* buf,char* jsbuf,bool form);
/*void formBeg(char* buf,char*jsbuf,const char* title);
void formBeg(char* buf,char*jsbuf);*/
void formIntro(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,const char* title,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,const char* title,uint8_t pol,uint8_t ctl);
/*void usrFormBHtml(char* buf,bool hid);
void usrFormBHtml(char* buf,char* jsbuf,bool hid);*/
void formEnd(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void formEnd(char* buf,char* jsbuf,bool title,uint8_t pol,uint8_t ctl);
void cliPrintMac(EthernetClient* cli, byte* mac);
void trailingSpaces(char* data,uint16_t len);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet);

void selectTableBHtml(char* buf,char* val,char* ft,int nbre,int len,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t td);
void selectTableBHtml(char* buf,char* jsbuf,char* val,char* ft,int nbre,int len,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl);
void selectTableBHtml(char* buf,char* jsbuf,char* val,char* name,char* ft,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl);
void optSelHtml(char* jsbuf,char* val,char* name);
/*void usrPeriCurB(char* buf,const char* fnct,uint8_t ninp,int len,uint8_t td);
void usrPeriCurB(char* buf,char* jsbuf,const char* fnct,uint8_t ninp,int len,uint8_t td);*/
void subDSnB(char* buf,const char* fnc,uint32_t val,uint8_t num,char* lib);
void subDSnB(char* buf,char* jsbuf,const char* fnc,uint32_t val,uint8_t num,char* lib);
#define NO_STATE 9         // etat
void checkboxTableBHtml(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,const char* lib,uint8_t pol,uint8_t ctl);

void fontBeg(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void fontEnd(char* buf,char* jsbuf,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,bool border,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,const char* police,bool border,uint8_t ctl);
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
uint8_t bufPrintPeriDate(char* buf,char* periDate);
void bufPrintDateHeure(char* buf,char* pkdate);
void bufPrintDateHeure(char* buf,char* jsbuf,char* pkdate);
void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol);
//void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec,bool br);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t dec,uint8_t pol,uint8_t ctl);
void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t dec,uint8_t pol,uint8_t ctl);
void affSpace(char* buf,char* jsbuf);
void affColonBeg(char* buf,char* jsbuf);
void affColonEnd(char* buf,char* jsbuf);
void affText(char* buf,char* jsbuf,const char* txt,uint8_t pol,uint8_t ctl);
void affText(char* buf,char* jsbuf,const char* txt,uint16_t tdWidth,uint8_t pol,uint8_t ctl);
void affNum(char* buf,char* jsbuf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t ctl);
void affNum(char* buf,char* jsbuf,char type,void* value,uint8_t dec,uint8_t pol,uint8_t ctl);
void alphaTableHtmlB(char* buf,const char* valfonct,const char* nomfonct,int len);
void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len,uint8_t pol,uint8_t ctl);
void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t pol,uint8_t ctl);
void boutRetourB(char* buf,const char* lib,uint8_t td,uint8_t br);
void boutRetourB(char* buf,char* jsbuf,const char* lib,uint8_t ctl);
void boutF(char* buf,const char* nomfonct,const char* valfonct,const char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter);
void boutF(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t sizfnt,uint8_t ctl);
void boutMaj(char* buf,char* jsbuf,const char* lib,uint8_t ctl);
void radioTableBHtml(char* buf,byte valeur,char* nomfonct,uint8_t nbval);
void radioTableBHtml(char* buf,char* jsbuf,byte valeur,char* nomfonct,uint8_t nbval,uint8_t pol,uint8_t ctl);
void yradioTableBHtml(char* buf,byte valeur,const char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t td);
void setColourB(char* buf,const char* textColour);
void setColourB(char* buf,char* jsbuf,const char* textColour);
void setColourE(char* buf);
void setColourE(char* buf,char* jsbuf);
void sliderBHtml(char* buf,uint8_t* val,const char* nomfonct,int nb,int sqr,uint8_t td);
void htmlIntro0B(char* buf);
void htmlIntroB(char* buf,char* titre,EthernetClient* cli);

void bufLenShow(char* buf,char* jsbuf,uint16_t lb,unsigned long begTPage);

#endif // _UTILHTML_H_
