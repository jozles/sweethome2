#ifndef _RADIO_PWRSLP_H_INCLUDED
#define _RADIO_PWRSLP_H_INCLUDED

#if MACHINE_DET328

#ifdef ATMEGA8
#define WDTCSR WDTCR
#define EIFR   GIFR
#define TIMSK1 TIMSK
#endif //


// prescaler WDT
#define NB_PRESCALER_VALUES 10
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
#define  AWAKE_OK_VALUE       2     // nbre réveils entre chaque test de temp 
#define  AWAKE_MIN_VALUE      10    // nbre réveils maxi pour message minimum de présence
#define  AWAKE_KO_VALUE       120   // debug 1500 // nbre réveils avant prochain test si com HS
#define  AWAKE_RETRY_VALUE    3     // nbre de retry avant KO

void sleepPwrDown(uint8_t durat);
void lethalSleep();
void checkOn();
void checkOff();
void wd();
void getVolts();
uint16_t adcRead0(uint8_t admuxval,uint8_t dly);
float adcRead(uint8_t admuxval,float factor, uint16_t offset, uint8_t ref,uint8_t dly);

#endif // MACHINE_DET328

#endif // _RADIO_PWRSLP_H_INCLUDED
