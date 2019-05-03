#ifndef _UTILHTML_H_
#define _UTILHTML_H_

void htmlIntro0(EthernetClient* cli);
void htmlIntro(char* titre,EthernetClient* cli);
void cliPrintMac(EthernetClient* cli, byte* mac);
void usrFormHtml(EthernetClient* cli,bool hid);
void boutRetour(EthernetClient* cli,char* lib,uint8_t td,uint8_t br);
void boutFonction(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter);
void bouTableHtml(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br);
void lnkTableHtml(EthernetClient* cli,char* nomfonct,char* lib);
void numTableHtml(EthernetClient* cli,char type,void* valfonct,char* nomfonct,int len,uint8_t td,int pol);
void xradioTableHtml(EthernetClient* cli,byte valeur,char* nomfonct,byte nbval,int nbli,byte type);
void checkboxTableHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int etat,uint8_t td);
void subDSn(EthernetClient* cli,char* fnc,uint32_t val,uint8_t num);
void boutonHtml(EthernetClient* cli,byte* valfonct,char* nomfonct,uint8_t sw,uint8_t td);
void textTableHtml(EthernetClient* cli,char type,float* valfonct,float* valmin,float* valmax,uint8_t br,uint8_t td);
void setColour(EthernetClient* cli,char* textColour);
void sliderHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int nb,int sqr,uint8_t td);
void printPeriDate(EthernetClient* cli,char* periDate);

#endif // _UTILHTML_H_
