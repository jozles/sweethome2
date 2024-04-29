#ifndef NRFUSER_CONC_H_INCLUDED
#define NRFUSER_CONC_H_INCLUDED

#if NRF_MODE == 'C'

void exportData(uint8_t numT);
int  exportData(uint8_t numT,char* modelName);
int  exportData(uint8_t numT,char* modelName,char* mailData);
void concExport(const char* messName);
int  importData(uint32_t* tLast);
void userResetSetup(byte* serverIp);

#endif // NRF_MODE == 'C'

#endif // NRFUSER_CONC_H_INCLUDED
