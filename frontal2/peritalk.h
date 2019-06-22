#ifndef _PERITALK_H_
#define _PERITALK_H_

void assySet(char* message,int periCur,char* diag,char* date14);
int  periReq(EthernetClient* cli,uint16_t np,char* nfonct);
int  periAns(EthernetClient* cli,char* nfonct);

#endif // _PERITALK_
