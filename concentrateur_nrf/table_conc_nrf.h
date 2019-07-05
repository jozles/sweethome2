#ifndef _TCONCNRF_H_INCLUDED
#define _TCONCNRF_H_INCLUDED

#if NRF_MODE == 'C'

void tableCPrint();
void tableCInit();
int  tableCLoad();
int  tableCSave();

void printAddr(char* addr,char n);

#endif NRF_MODE == 'C'

#endif // TCONCNRF
