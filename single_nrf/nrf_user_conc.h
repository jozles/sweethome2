#ifndef NRFUSER_CONC_H_INCLUDED
#define NRFUSER_CONC_H_INCLUDED


#include "nrf24l01s_const.h"


#if NRF_MODE == 'C'

int  exportData(uint8_t numT);
int  importData();
void userResetSetup();

#endif // NRF_MODE == 'C'

#endif // NRFUSER_CONC_H_INCLUDED
