#ifndef _UTILHTML_H_
#define _UTILHTML_H_

#include <Ethernet.h>

#define DISJCOLOR   0           // Green
#define ONCOLOR     1           // Blue
#define FORCEDCOLOR 2           // Red
#define PUSHCOLOR   3           // Yellow
#define OFFCOLOR    4           // Grey
#define CURCOLOR    5           // 
#define STDBUTTON   9           // Silver
#define COLNAMENB   10           
#define LIGHTVALUE  20          // light mode (light grey, light yellow etc)

#define JPGINTROLEN 50
#define HTMLENDPNGLEN 20

#define BORDER (bool)true
#define NOBORDER (bool)false


#define INTROHTTP 'H'
#define INTROHTMLE 'h'
#define GENSTYLE 'G'
#define REMOTESTYLE 'R'

#define NOJSBUF

void bufcat(char* buf,const char* s);
void jscat(char* jsbuf,const char* s,bool sep);
void jscat(char* jsbuf,const char* s);
void tdSet(char* jsbuf,uint8_t width,const char* font,uint8_t fsize,uint8_t ctl);
void tdReset();
void tdcat(char* buf);

void fnJsIntro(char* jsbuf,const char* fonc,uint8_t pol,const uint8_t ctl);

void fnHtmlIntro(char* buf,const char* font,uint8_t pol,uint8_t ctl,char* colour);
void fnHtmlIntro(char* buf,uint8_t pol,uint8_t ctl,char* colour);
void fnHtmlEnd(char* buf,const char* font,uint8_t pol,uint8_t ctl);
void fnHtmlEnd(char* buf,uint8_t pol,uint8_t ctl);

void scrStore(char* jsbuf,char name,const char* data);
void scrRecall(char* jsbuf,char name);

void jpgIntro0(char* dm);
//void htmlIntro0(char* buf,char* jsbuf);
void pageIntro(char* buf,char* jsbuf,char* titre);
void pageIntro0(char* buf,char* jsbuf);
//void htmlBeg0(char* buf,char* jsbuf);
void htmlBeg(char* buf,char* jsbuf,char* titre);
void htmlBeg(char* buf,char* jsbuf,char* titre,char rem);
//void htmlBegE(char* buf,EthernetClient* cli);
void htmlEnd(char* buf,char* jsbuf);

void htmlStyleTable(char* buf);
void htmlStyleCbBut(char* buf);
void htmlStyleSliders(char* buf);
void htmlStyleSqrBut(char* buf); 

void pageLineOne(char* buf,char* jsbuf);

void formIntro(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,const char* title,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,uint8_t pol,uint8_t ctl);
void formIntro(char* buf,char* jsbuf,const char* locfonc,uint8_t ninp,const char* title,uint8_t pol,uint8_t ctl);

void formEnd(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void formEnd(char* buf,char* jsbuf,bool title,uint8_t pol,uint8_t ctl);

void subDSnBm(char* buf,char* jsbuf,const char* fnc,uint8_t* val,uint8_t num,char* lib);
#define NO_STATE 9         // etat

uint8_t mDSval(uint8_t num);

void scrGetSelect(char* buf,char* jsbuf,char* val,char* ft,int nbre,int len,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl);
void scrGetSelect(char* buf,char* jsbuf,char* val,char* name,char* ft,uint8_t sel,uint8_t nuv,uint8_t ninp,uint8_t pol,uint8_t ctl);
void optSelHtml(char* jsbuf,char* val,char* name);

void scrGetCheckbox(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void scrGetCheckbox(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib);
void scrGetCheckbox(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,const char* lib,uint8_t pol,uint8_t ctl);
void sliderBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nf,int sqr,uint8_t ctl);

void polBeg(char* buf,char* jsbuf,uint8_t pol,uint8_t ctl);
void polEnd(char* buf,char* jsbuf,uint8_t ctl);

void setFont(char* buf,const char* font,const char* size);
void endFont(char* buf);

void setColourB(char* buf,const char* textColour);
void setColourB(char* buf,char* jsbuf,const char* textColour);
void setColourE(char* buf);
void setColourE(char* buf,char* jsbuf);

void tableBeg(char* buf,char* jsbuf,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,bool border,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,const char* police,bool border,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,const char* police,bool border,const char* height,uint8_t ctl);
void tableBeg(char* buf,char* jsbuf,const char* police,const char* size,bool border,const char* height,uint8_t ctl);
void tableEnd(char* buf,char* jsbuf,uint8_t ctl);

void scrGetNum(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t dec,uint8_t pol,uint8_t ctl);
void scrGetNum(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t dec,uint8_t pol,uint8_t ctl);

void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len,uint8_t pol,uint8_t ctl);
void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t sizfnt,uint8_t pol,uint8_t ctl);
void scrGetText(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t size,int len,uint8_t pol,uint8_t ctl);
void scrGetHidden(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,uint8_t pol,uint8_t ctl);

void scrGetButRet(char* buf,char* jsbuf,const char* lib,uint8_t ctl);
void scrGetButRef(char* buf,char* jsbuf,const char* nomfonct,const uint8_t suffn,uint8_t ctl);
void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t butsize,uint8_t ctl);
void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,uint8_t butsize,uint8_t bgcolor,uint8_t fntcolor,uint8_t margin,uint8_t round,uint8_t ctl);
void scrGetButFn(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,bool aligncenter,const char* font,char* sizfnt,uint8_t butsize,uint8_t bgcolor,uint8_t margin,uint8_t round,uint8_t ctl);
void scrGetButSub(char* buf,char* jsbuf,const char* lib,uint8_t ctl);
void scrGetButSub(char* buf,char* jsbuf,const char* lib,bool aligncenter,uint8_t sizfnt,uint8_t ctl);

void scrGetRadiobut(char* buf,char* jsbuf,byte valeur,char* nomfonct,uint8_t nbval,uint8_t pol,uint8_t ctl);
void scrGetRadiobut(char* buf,char* jsbuf,byte valeur,char* nomfonct,uint8_t nbval,bool vert,char* lib,uint8_t pol,uint8_t ctl);
void yscrGetRadiobut(char* buf,char* jsbuf,byte valeur,const char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t ctl);

void affSpace(char* buf,char* jsbuf);
void affSpace(char* buf,char* jsbuf,uint8_t ctl);

void affRondJaune(char* buf,char* jsbuf,uint8_t ctl);

void scrDspText(char* buf,char* jsbuf,const char* txt,uint8_t pol,uint8_t ctl);
void scrDspText(char* buf,char* jsbuf,const char* txt,uint16_t tdWidth,uint8_t pol,uint8_t ctl);
void scrDspText(char* buf,char* jsbuf,const char* txt,uint16_t tdWidth,const char* font,uint8_t pol,uint8_t ctl);

void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,uint8_t ctl);
void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,uint8_t dec,uint8_t ctl);
void scrDspNum(char* buf,char* jsbuf,int16_t* valfonct,const int16_t* valmin,const int16_t* valmax,bool check,uint8_t dec,uint8_t ctl);
void scrDspNum(char* buf,char* jsbuf,char type,void* value,uint8_t dec,uint8_t pol,uint8_t ctl);
void scrDspNum(char* buf,char* jsbuf,char type,void* value,const void* valmin,const void* valmax,bool nominmax,uint8_t dec,uint8_t pol,uint8_t ctl);

void bufLenShow(char* buf,char* jsbuf,uint16_t lb,unsigned long begTPage);

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
void cliPrintMac(EthernetClient* cli, byte* mac);
void trailingSpaces(char* data,uint16_t len);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet,uint8_t st);
#endif // _UTILHTML_H_
