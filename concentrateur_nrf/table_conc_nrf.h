#ifndef _TCONCNRF_H_
#define _TCONCNRF_H_

#if NRF_MODE == 'C'

void tableCPrint();
void tableCInit();
int  tableCLoad();
int  tableCSave();

#endif NRF_MODE == 'C'

#endif // TCONCNRF
