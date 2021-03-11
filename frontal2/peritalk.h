#ifndef _PERITALK_H_
#define _PERITALK_H_


void assySet(char* message,int periCur,char* diag,char* date14);
int  periReq0(EthernetClient* cli,char* nfonct,char* msg);
int  periReq(EthernetClient* cli,uint16_t np,char* nfonct,char* msg);
int  periReq(EthernetClient* cli,uint16_t np,char* nfonct);
int  periAns(EthernetClient* cli,char* nfonct);
void periDataRead(char* valf);

#endif // _PERITALK_
