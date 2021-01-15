#ifndef _PAGESHTML_H_
#define _PAGESHTML_H_

void dumpHisto(EthernetClient* cli);
int  dumpHisto0(EthernetClient* cli);
void intro(EthernetClient* cli);
void cfgServerHtml(EthernetClient* cli);
void cfgRemoteHtml(EthernetClient* cli);
void cfgThermosHtml(EthernetClient* cli);
void accueilHtml(EthernetClient* cli);
void testHtml(EthernetClient* cli);
void htmlFavicon(EthernetClient* cli);
int  htmlImg(EthernetClient* cli,char* fimgname);
void remoteHtml(EthernetClient* cli);
void thermoCfgHtml(EthernetClient* cli);
void thermoShowHtml(EthernetClient* cli);
void timersHtml(EthernetClient* cli);
void detServHtml(EthernetClient* cli,uint32_t* mds,char* lib);
void cfgDetServHtml(EthernetClient* cli);
int  scalcTh(int bd);


#endif // _PAGESHTML_H_
