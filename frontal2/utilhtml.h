#ifndef _UTILHTML_H_
#define _UTILHTML_H_


//void htmlIntro0(EthernetClient* cli);
//void htmlIntro(char* titre,EthernetClient* cli);
//void usrFormHtml(EthernetClient* cli,bool hid);
//void usrFormInitHtml(EthernetClient* cli,char* nomfonct,bool hid);
//void usrPeriCur(EthernetClient* cli,char* fnct,uint8_t ninp,int len,uint8_t td);
//void boutRetour(EthernetClient* cli,char* lib,uint8_t td,uint8_t br);
//void boutFonction(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter);
//void bouTableHtml(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br);
//void lnkTableHtml(EthernetClient* cli,char* nomfonct,char* lib);
//void numTableHtml(EthernetClient* cli,char type,void* valfonct,char* nomfonct,int len,uint8_t td,int pol);
//void radioTableHtml(EthernetClient* cli,byte valeur,char* nomfonct,uint8_t nbval);
//void yradioTableHtml(EthernetClient* cli,byte valeur,char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t td);
//void checkboxTableHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int etat,uint8_t td,char* title);
//void selectTableHtml(EthernetClient* cli,char* val,char* ft,int nbre,int len,int sel,uint8_t v0,uint8_t v1,uint8_t td);
//void subDSn(EthernetClient* cli,char* fnc,uint32_t val,uint8_t num,char* title);
//void boutonHtml(EthernetClient* cli,byte* valfonct,char* nomfonct,uint8_t sw,uint8_t td);
//void textTableHtml_(EthernetClient* cli,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t br,uint8_t td);
//void setColour(EthernetClient* cli,char* textColour);
//void sliderHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int nb,int sqr,uint8_t td);
//void printPeriDate(EthernetClient* cli,char* periDate);
//char* cliPrintDateHeure(EthernetClient* cli,char* pkdate);

void pageHeader(char* buf);
void cliPrintMac(EthernetClient* cli, byte* mac);
void trailingSpaces(char* data,uint16_t len);
void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet);

void selectTableBHtml(char* buf,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t td);
void usrPeriCurB(char* buf,char* fnct,uint8_t ninp,int len,uint8_t td);
void subDSnB(char* buf,char* fnc,uint32_t val,uint8_t num,char* lib);
void checkboxTableBHtml(char* buf,uint8_t* val,char* nomfonct,int etat,uint8_t td,char* lib);
void usrFormBHtml(char* buf,bool hid);
void usrFormInitBHtml(char* buf,char* nomfonct);
void concatIp(char* buf,byte* ip);
void concat1a(char* buf,char a);
void concat1aH(char* buf,char a);
void concatn(char* buf,unsigned long val);
void concatns(char* buf,long val);
void concatnf(char* buf,float val);
void concatnf(char* buf,float val,uint8_t dec);
void concatDate(char* buf,char* periDate);
void bufPrintPeriDate(char* buf,char* periDate);
void bufPrintDateHeure(char* buf,char* pkdate);
void numTf(char* buf,char type,void* valfonct,char* nomfonct,int len,uint8_t td,int pol);
void numTf(char* buf,char type,void* valfonct,char* nomfonct,int len,uint8_t td,int pol,uint8_t dec);
void textTbl(char* buf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t br,uint8_t td);
void alphaTableHtmlB(char* buf,char* valfonct,char* nomfonct,int len);
void setCol(char* buf,char* textColour);
void boutRetourB(char* buf,char* lib,uint8_t td,uint8_t br);
void boutF(char* buf,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter);
void radioTableBHtml(char* buf,byte valeur,char* nomfonct,uint8_t nbval);
void yradioTableBHtml(char* buf,byte valeur,char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t td);
void setColourB(char* buf,char* textColour);
void sliderBHtml(char* buf,uint8_t* val,char* nomfonct,int nb,int sqr,uint8_t td);
void htmlIntro0B(char* buf);
void htmlIntroB(char* buf,char* titre,EthernetClient* cli);

#endif // _UTILHTML_H_
