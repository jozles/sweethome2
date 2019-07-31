#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"
#include "nrf_user.h"

#if NRF_MODE == 'P'

#include <avr/sleep.h>
#include <avr/power.h>

uint16_t wdtTime[]={16,32,64,125,500,1000,2000,4000,8000};   // durées WDT millis

extern Nrfp nrfp;

extern float   durT;
extern uint32_t nbS;

 
void hardwarePowerDown()
{
  userHardPowerDown();
  nrfp.powerDown();
  pinMode(LED,INPUT);
}

void wdtSetup(uint8_t durat)  // (0-9) durat>9 for external wdt on INT0 (à traiter)
{ 
// datasheet page 54, Watchdog Timer.
   
    noInterrupts();

/*  MCUSR MCU status register (reset sources)(every bit cleared by writing 0 in it)
 *   WDRF reset effectué par WDT
 *   BORF ------------------ brown out detector
 *   EXTRF ----------------- pin reset
 *   PORF ------------------ power ON
*/
  MCUSR &= ~(1<<WDRF);  // pour autoriser WDE=0
   
/*  WDTCSR watchdog timer control
 *   WDIF watchdog interrupt flag (set when int occurs with wdt configured for) (reset byu writing 1 or executing ISR(WDT_vect))
 *   WDIE watchdog interrupt enable 
 *   WDE  watchdog reset enable
 *        WDE  WDIE   Mode
 *         0    0     stop
 *         0    1     interrupt
 *         1    0     reset
 *         1    1     interrupt then reset (WDIE->0 lors de l'interruption, retour au mode reset)
 *       !!!! le fuse WDTON force le mode reset si 0 !!!!
 *   WDCE watchdog change enable (enable writing 0 to WDE and modif prescaler) (auto cleared after 4 cycles)
 *   WDP[3:0] prescaler 2^(0-9)*2048 divisions de l'oscillateur WDT (f=128KHz p*2048=16mS) 
 *   
 *   l'instrction wdr reset le timer (wdt_reset();)
 */



    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3);   // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s
                                  
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE doivent être 1 
                                      // pour écrire WDP[0-3] et WDE dans les 4 cycles suivants
    WDTCSR = (1<<WDIE) | durat;       // WDCE doit être 0 ; WDE=0 ; WDIE=1 mode interruption, 8s

  interrupts();

}

void sleepPwrDown(uint8_t durat)
{

    nbS++;
    durT+=wdtTime[durat]/10;
    hardwarePowerDown();

    wdtSetup(durat);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    power_adc_disable();
    sleep_enable();

//  Serial.print("sleep_enable ");Serial.println(durat);delay(5); 
    
    sleep_mode();
  
    sleep_disable();
    power_all_enable();
}

#endif // NRF_MODE == 'P'
