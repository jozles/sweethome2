#ifndef NRFPWRSLP_H_INCLUDED
#define NRFPWRSLP_H_INCLUDED

#if NRF_MODE == 'P'

#ifdef ATMEGA8
#define WDTCSR WDTCR
#define EIFR   GIFR
#define TIMSK1 TIMSK
#endif


// prescaler WDT
#define T16   0b00000000
#define T32   0b00000001
#define T64   0b00000010
#define T125  0b00000011
#define T250  0b00000100
#define T500  0b00000101
#define T1000 0b00000110
#define T2000 0b00000111
#define T4000 0b00100000
#define T8000 0b00100001

/* --------------- @ env 2sec / sleep -------------------- */
#define  AWAKE_OK_VALUE       10    // nbre réveils entre chaque test de temp 
#define  AWAKE_MIN_VALUE      120   // nbre réveils maxi pour message minimum de présence
#define  AWAKE_KO_VALUE       120   // debug 1500 // nbre réveils avant prochain test si com HS
#define  AWAKE_RETRY_VALUE    3     // nbre de retry avant KO

void sleepPwrDown(uint8_t durat);
void lethalSleep();
void checkOn();
void checkOff();
void wd();
void getVolts();


#endif // NRF_MODE == 'P'

#endif // NRFPWRSLP_H_INCLUDED
