#ifndef NRFUSER_CONC_H_INCLUDED
#define NRFUSER_CONC_H_INCLUDED

#include <Arduino.h>

#if NRF_MODE == 'C'

int  exportData(uint8_t numT);
void userResetSetup();

#endif // NRF_MODE == 'C'

#endif // NRFUSER_CONC_H_INCLUDED
