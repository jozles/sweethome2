
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/* mécanisme
 *  
 *  la fonction uint16_t sleepPwrDown(durat) gère et effectue la mise en sommeil profond
 *  SLEEP_MODE_PWR_DOWN du MCU, et débranche tout ce qui consomme pour assurer la veille 
 *  la plus sobre possible.
 *  Selon l'argument durat, le réveil provient du watchdog timer ou du pin d'interruption 1 si durat = 0.
 *    WDT :
 *    Les durées valides effectuées par le WDT sont définies dans la fonction wdtSetup()
 *    qui initialise le registre WDTCSR pour déclencher le timer.
 *    Au réveil, la fonction wdtDisable() inhibe le WDT. 
 *    La fonction ISR(WDT_vect) est exécutée lors de l'interruption issue du WDT
 *    INT1 :
 *    La fonction int1_ISR() est exécutée lors de l'interruption,
 *    issue du pin INT1
 *    Comme le WDT est stoppé, la consommation est moindre
 *    
 *  la fonction letalSleep() n'effectue pas wdtSetup ni ne branche d'interruption du pin INT1
 *  rien ne sortira le CPU du sommeil sauf un reset physique. (à utiliser si les batteries 
 *  sont épuisées par exemple).
*/


#define LED  4 // 13 test // 4 DETS
#define REED 3 // 5  test // 3 DETS   (PD3-INT1)

uint16_t wdtTime[]={16,32,64,125,250,500,1000,2000,4000,8000};   // WDT millis

uint8_t nbBlk;                    // blink nb : 1 is from timer, 5 from reed, 1 sec if reed on, 8 sec sleep if reed off

ISR(WDT_vect)                     // ISR interrupt service for MPU INT WDT vector
{
}

void int1_ISR()
{
  sleep_disable();
  detachInterrupt(1);
}

void blk(uint8_t nbblk)
{
  pinMode(LED,OUTPUT);
  for(int i=0;i<nbblk;i++){digitalWrite(LED,HIGH);delay(1);digitalWrite(LED,LOW);delay(450);}
  pinMode(LED,INPUT); 
}


void wdtSetup(uint8_t durat)  // (0-9) durat>9 for external wdt on INT0 (à traiter)
{
// datasheet page 51 and 54, Watchdog Timer.


/*  MCUSR MCU status register (reset sources)(every bit cleared by writing 0 in it)
 *   WDRF reset effectué par WDT
 *   BORF ------------------ brown out detector
 *   EXTRF ----------------- pin reset
 *   PORF ------------------ power ON
 *
 *  WDTCSR watchdog timer control
 *   WDIF watchdog interrupt flag (set when int occurs with wdt configured for) (reset byu writing 1 or executing ISR(WDT_vect))
 *   WDIE watchdog interrupt enable  -> counter ovf make interrupt
 *   WDE  watchdog reset enable      -> counter ovf make reset
 *        WDE  WDIE   Mode
 *         0    0     stop (no watchdog)
 *         0    1     interrupt
 *         1    0     reset
 *         1    1     interrupt then reset (WDIE->0 lors de l'interruption, retour au mode reset)
 *       !!!! fuse WDTON forces reset mode if 0 !!!!
 *   WDCE watchdog change enable (0 write enable to WDE and prescaler update ; auto cleared after 4 cycles)
 *   WDP[3:0] prescaler 2^(0-9)*2048 divide WDT oscillator (f=128KHz p*2048=16mS)
 *
 *   wdr instruction resets timer (wdt_reset();)
 *   
 *   power down supply current for ATMEGA 328P page 594 : typically 4,5 uA at 3,3V 25°C with watchdog enabled (about 6,5 at 5V)
 *   
 *   idle supply current for ATMEGA 328P page 591 : typically labout 0,7mA at 3,3V 8MHz 
 *   
 *   
 */

// WDT prescaler - WDP3-0 bits (msec)

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

  noInterrupts();
  wdt_reset();

    MCUSR &= ~(1<<WDRF);  // pour autoriser WDE=0

    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE must be 1
                                      // to write WDP[0-3] and WDE in the following 4 cycles
    WDTCSR = (1<<WDIE) | durat;       // WDCE must be 0 ; WDE=0, WDIE=1 interrupt mode, TXXX 
     
  interrupts();
}

void wdtDisable()
{
    noInterrupts();
    wdt_reset();
    MCUSR &= ~(1<<WDRF);  // pour autoriser WDE=0
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE must be 1 to write WDE in the following 4 cycles
    WDTCSR = 0;                       // WDE et WDIE disabled
    interrupts();                                                    
}

uint16_t sleepPwrDown(uint8_t durat)
{

/* to reduce ATMEGA power in sleep mode
 *  
 * disable ADC
 *    in ADCSRA register clr ADEN bit ->  ADCSRA = 0;
 *    in DIDR0 register set ADC5D...ADC0D bits 
 *    ADC must be disabled before PRR !
 * disable AC (analog comparator)
 *    in ACSR register set ACD bit
 *    in DIDR1 register set AIN1D, AIN0D bits
 * disable BOD -> sleep_bod_disable() before sleep_cpu()
 *    in MCUCR register set BODS and BODSE bits then set BODS and clr BODSE ; 3 clk cycles to sleep or BODS cleared
 * Voltage Reference is connected when either
 *    BOD is enable or bandgap ref connected to AC or ADC enable
 * disable modules clock in PRR -> power_all_disable() 
 *    for ACTIVE and IDLE mode only (automatic in other modes)
 * put every I/O pin in input mode
 *
 * Power Reduction Register (PRR)
 * 
 * Bit 7 - PRTWI    : Two Wire
 * Bit 6 - PRTIM2   : Timer/Counter2
 * Bit 5 - PRTIM0   : Timer/Counter0
 * Bit 4 - Res:
 * Bit 3 - PRTIM1   : Timer/Counter1
 * Bit 2 - PRSPI    : SPI
 * Bit 1 - PRUSART0 : USART0
 * Bit 0 - PRADC    : ADC
 *
 * Enabling :
 *
 * power_adc_enable();    // ADC converter
 * power_spi_enable();    // SPI
 * power_usart0_enable(); // Serial (USART)
 * power_timer0_enable(); // Timer 0
 * power_timer1_enable(); // Timer 1
 * power_timer2_enable(); // Timer 2
 * power_twi_enable();    // TWI (I2C)
 *
 * Disabling:
 *
 * power_all_disable ();   // turn off all modules
 *
 * power_adc_disable();   // ADC converter
 * power_spi_disable();   // SPI
 * power_usart0_disable();// Serial (USART)
 * power_timer0_disable();// Timer 0
 * power_timer1_disable();// Timer 1
 * power_timer2_disable();// Timer 2
 * power_twi_disable();   // TWI (I2C)
*/
    ADCSRA &= ~(1<<ADEN);                 // ADC shutdown
    
    power_all_disable();                  // set all bits in PRR register (modules clk halted)
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  

    if(durat!=0){wdtSetup(durat);}        // setup WDTCSR register for sleep with WDT int awake
    noInterrupts();                       // cli();

    if(durat==0){                         // awake by interrupt
      attachInterrupt(1,int1_ISR,FALLING);
      EIFR=bit(INTF1);                    // clr flag
    }
    
    sleep_enable();                       
    sleep_bod_disable();                  // BOD halted if sleep_cpu follow 
    interrupts();                         // sei();
    sleep_cpu();
    sleep_disable();
    wdtDisable();                         
    power_all_enable();
    ADCSRA |= (1<<ADEN);                  // ADC enable

    return wdtTime[durat]/10;
}

void letalSleep()
{
    noInterrupts();
    wdt_reset();
    wdtDisable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
    ADCSRA = 0;
    power_all_disable();
    cli();
    sleep_enable();
    sleep_bod_disable();
    sleep_cpu();            // neither interrupt nor wdt -> reset only to awake cpu
}

void setup() {
    
    Serial.begin(115200);
    Serial.println("test sleep ready");
   
  pinMode(REED,INPUT_PULLUP); // must stay pullup when reed mode active
    
    nbBlk=5;
}

void loop() 
{

  /* 5 blinks au reset puis 8 sec sleep suivi d'1 blink jusqu'à contact du reed
   * à chaque contact changement de mode avec 16 blinks de tempo pour retirer l'aimant
   */

  
  bool reed=digitalRead(REED);Serial.println(reed);delay(1);
  if(!reed){ nbBlk=3;}                 // if reed on (pullup low) chge mode

  
  switch(nbBlk){
    case 1:sleepPwrDown(T8000);break;     // >3uA     
    case 3:
      for(int i=0;i<10;i++){blk(2);}      // tempo armement pour retirer l'aimant
      sleepPwrDown(0);                    // >1uA
      for(int i=0;i<8;i++){blk(2);}       // tempo désarmement pour retirer l'aimant      
      break;              
    default:break;
  }

  blk(nbBlk);                     // 1 blink -> timer ; 3 -> reed ;
                                  // le reed passe d'un mode à l'autre
  nbBlk=1;
}
