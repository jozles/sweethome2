#ifndef NRFUSER_CONC_H_INCLUDED
#define NRFUSER_CONC_H_INCLUDED

#if MACHINE_CONCENTRATEUR

void exportData(uint8_t numT);
int  exportData(uint8_t numT,char* mailData);
void exportDataMail(const char* messName);
void exportDataMail(const char* messName,uint8_t maxRetry);
int  importData();
void userResetSetup(byte* serverIp,const char* uRSMessage);
void userResetSetup(byte* serverIp);
void blkCtl(uint8_t where);
void fillMess(byte* message);
void blkCtlInit(uint8_t blkPin1,uint8_t blkPin2,uint8_t blkPin3);

#endif // MACHINE == 'C'

#endif // NRFUSER_CONC_H_INCLUDED
