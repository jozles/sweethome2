#ifndef _PAGESHTML_H_
#define _PAGESHTML_H_

void dumpHisto(EthernetClient* cli);
void dumpHisto0(EthernetClient* cli,long histoPos);
void shDateHist(char* dhasc,long* pos);
void cfgServerHtml(EthernetClient* cli);
void cfgRemoteHtml(EthernetClient* cli);
void cfgThermosHtml(EthernetClient* cli);
void accueilHtml(EthernetClient* cli);
void testHtml(EthernetClient* cli);
void htmlFavicon(EthernetClient* cli);
int  htmlImg(EthernetClient* cli,const char* fimgname);
void remoteHtml(EthernetClient* cli);
void thermoCfgHtml(EthernetClient* cli);
void thermoShowHtml(EthernetClient* cli);
void timersHtml(EthernetClient* cli);
void timersCtlHtml(EthernetClient* cli);
void detServHtml(EthernetClient* cli,uint8_t* mds,char* lib);
void detServHtml(EthernetClient* cli,char* buf,char* jsbuf,uint16_t* lb,uint16_t lb0,uint8_t* mds,char* lib);
void cfgDetServHtml(EthernetClient* cli);
int  scalcTh(int bd);


#endif // _PAGESHTML_H_
