#ifndef NRFUSER_CONC_H_INCLUDED
#define NRFUSER_CONC_H_INCLUDED

#if NRF_MODE == 'C'

int  exportData(uint8_t numT);
int  importData(uint32_t* tLast);
void userResetSetup();

#endif // NRF_MODE == 'C'

#endif // NRFUSER_CONC_H_INCLUDED
