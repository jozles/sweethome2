#include "radio_const.h"


#if NRF_MODE =='C'

int get_radio_message(byte* messageIn,uint8_t* pipe,uint8_t* pldLength,int nbper);

uint8_t cRegister(char* message);
uint8_t macSearch(char* mac,int* numPer);
uint8_t extDataStore(uint8_t numPer,uint8_t numT,uint8_t offset,char* data,uint8_t len);
void tableCPrint();
void tableCInit();
int tableCLoad();
int tableCSave();


#endif // NRF_MODE=='C'