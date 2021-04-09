#ifndef _PERITALK_H_
#define _PERITALK_H_


void assySet(char* message,int periCur,const char* diag,char* date14);
int  periReq0(EthernetClient* cli,const char* nfonct,const char* msg);
int  periReq(EthernetClient* cli,uint16_t np,const char* nfonct,const char* msg);
int  periReq(EthernetClient* cli,uint16_t np,const char* nfonct);
int  periAns(EthernetClient* cli,const char* nfonct);
void periDataRead(char* valf);

#endif // _PERITALK_
