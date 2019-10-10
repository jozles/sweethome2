#include <SPI.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/* mécanisme
 *  
 *  uint16_t sleepPwrDown(durat) deep sleep manager
 *  make MCU SLEEP_MODE_PWR_DOWN and disable peripheral devices
 *  
 *  uint8_t durat is sleep delay by watchdog timer or external awake by INT1 pin if 0.
 *    WDT :
 *    valid durations are defined in wdtSetup() wich initialize WDTCSR reg to allow timer start.
 *    When awaking wdtDisable() disable WDT. 
 *    ISR(WDT_vect) is WDT vector interrupt service routine 
 *    INT1 :
 *    int1_ISR() for INT1 interrupt
 *   
 *   SPI and NRF24L01 access added to test NRF24L01 powerDown mode 
*/

#define NRF

#define LED  4                  
#define REED 3                  // reed contact pin (connected to INT1)

#ifdef NRF
/* SPI and NRF pins */
#define CLK_PIN    13
#define MISO_PIN   12
#define MOSI_PIN   11
#define CSN_PIN    10
#define CE_PIN     9
#endif 

/* voltage pins */
#define VFACTOR 0.0047          // volts converting 10K+33K
#define VCHECK  A3              // volts check pin
#define VINPUT  7               // volts ADC input pin


uint16_t wdtTime[]={16,32,64,125,250,500,1000,2000,4000,8000};   // WDT millis

uint8_t nbBlk;                    // blink nb ... see loop()

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

float getVolts()                  // read unregulated power voltage (when dedicated resistors mounted... increase power)
{
  float volts;
  analogReference(INTERNAL); 
  pinMode(VCHECK,OUTPUT);digitalWrite(VCHECK,LOW);
  volts=analogRead(VINPUT)*VFACTOR;
  pinMode(VCHECK,INPUT);
  return volts;
}
 

void wdtSetup(uint8_t durat)  // (0-9) durat>9 for external wdt on INT0 (not implemented here)
{
// datasheet page 51 and 54, Watchdog Timer.


/*  MCUSR MCU status register (reset sources)(every bit cleared by writing 0 in it)
 *   WDRF reset effectué par WDT
 *   BORF ------------------ brown out detector
 *   EXTRF ----------------- pin reset
 *   PORF ------------------ power ON
 *
 *  WDTCSR watchdog timer control
 *   WDIF watchdog interrupt flag (set when int occurs with wdt configured for) (reset by writing 1 or executing ISR(WDT_vect))
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
    MCUSR &= ~(1<<WDRF);              // to enable WDE=0
    WDTCSR = (1<<WDCE) | (1<<WDE);    // WDCE ET WDE must be 1
                                      // to write WDP[0-3] and WDE in the following 4 cycles
    WDTCSR = (1<<WDIE) | durat;       // WDCE must be 0 ; WDE=0, WDIE=1 interrupt mode, TXXX  
    interrupts();
}

void wdtDisable()
{
    noInterrupts();
    wdt_reset();
    MCUSR &= ~(1<<WDRF);              // to enable WDE=0
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
 * power_adc_enable();      // ADC converter
 * power_spi_enable();      // SPI
 * power_usart0_enable();   // Serial (USART)
 * power_timer0_enable();   // Timer 0
 * power_timer1_enable();   // Timer 1
 * power_timer2_enable();   // Timer 2
 * power_twi_enable();      // TWI (I2C)
 *
 * Disabling:
 *
 * power_all_disable ();    // turn off all modules
 *
 * power_adc_disable();     // ADC converter
 * power_spi_disable();     // SPI
 * power_usart0_disable();  // Serial (USART)
 * power_timer0_disable();  // Timer 0
 * power_timer1_disable();  // Timer 1
 * power_timer2_disable();  // Timer 2
 * power_twi_disable();     // TWI (I2C)
*/
    ADCSRA &= ~(1<<ADEN);                 // ADC shutdown
    
    power_all_disable();                  // set PRR register bits (modules clk halted)
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  

    if(durat!=0){wdtSetup(durat);}        // setup WDTCSR register for sleep with WDT int awake
    noInterrupts();                       // cli();

    if(durat==0){                         // awake by INT1 interrupt
      attachInterrupt(1,int1_ISR,FALLING);
      EIFR=bit(INTF1);                    // clr flag
    }
    
    sleep_enable();                       
    sleep_bod_disable();                  // BOD halted if sleep_cpu follow 
    interrupts();                         // sei();
    
    sleep_cpu();                          // good night
    
    sleep_disable();
    wdtDisable();                         
    power_all_enable();                   // clr PRR reg bits
    ADCSRA |= (1<<ADEN);                  // ADC enable

    return wdtTime[durat]/10;             // not valid if ext interrupt awake
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
    sleep_cpu();                          // neither interrupt nor wdt -> reset only to awake cpu
}

void hardPwrDown()
{
    pinMode(LED,INPUT);
    pinMode(REED,INPUT);
#ifdef NRF
    nrf24l01PwrDown();
#endif
/*    pinMode(CSN_PIN,INPUT);     // 340 to 400 uA
    pinMode(CLK_PIN,INPUT);
    pinMode(CE_PIN,INPUT);*/

/* 3 pins low                        290 uA */

/*    
    digitalWrite(CSN_PIN,HIGH);     // 290 to 305 uA
    digitalWrite(CLK_PIN,LOW);
    digitalWrite(CE_PIN,LOW);

    digitalWrite(CSN_PIN,LOW);     // 111 uA
    digitalWrite(CLK_PIN,HIGH);
    digitalWrite(CE_PIN,LOW);

    digitalWrite(CSN_PIN,HIGH);     // 302 to 305 uA
    digitalWrite(CLK_PIN,HIGH);
    digitalWrite(CE_PIN,LOW);

    digitalWrite(CSN_PIN,LOW);     // 497 to 505 uA
    digitalWrite(CLK_PIN,LOW);
    digitalWrite(CE_PIN,HIGH);

    digitalWrite(CSN_PIN,HIGH);     // 444 to 446 uA
    digitalWrite(CLK_PIN,LOW);
    digitalWrite(CE_PIN,HIGH);

    digitalWrite(CSN_PIN,LOW);     // 503 to 561 uA
    digitalWrite(CLK_PIN,HIGH);
    digitalWrite(CE_PIN,HIGH);

    digitalWrite(CSN_PIN,HIGH);     // 490 to 494 uA
    digitalWrite(CLK_PIN,HIGH);
    digitalWrite(CE_PIN,HIGH);
*/
}    

#ifdef NRF
void nrf24l01PwrDown()
{
    SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));  // init SPI
    SPI.begin();

    digitalWrite(CE_PIN,LOW);
    pinMode(CE_PIN,OUTPUT);
    digitalWrite(CSN_PIN,HIGH);                                      // enable device access 
    pinMode(CSN_PIN,OUTPUT);

/* NRF CONF REG BITS */    
#define MASK_RX_DR_BIT 0x40
#define MASK_TX_DS_BIT 0x20
#define MASK_MAX_RT_BIT 0x10
#define EN_CRC_BIT  0x08
#define CRCO_BIT    0x04 
#define PWR_UP_BIT  0x02
#define PRIM_RX_BIT 0x01
     
#define CONFREG (0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) & ~(PWR_UP_BIT) | PRIM_RX_BIT )
    byte conf=CONFREG;                                                // PWR_UP_BIT = 0

    delay(100);                                                       // power on reset NRF
        
#define W_REGISTER    0x20                                            // write register command
    digitalWrite(CSN_PIN,LOW);
    SPI.transfer(W_REGISTER);                                         // config register nb is 0x00
    SPI.transfer(conf);
    digitalWrite(CSN_PIN,HIGH);

#define NOP           0xFF                                            // nop command
    digitalWrite(CSN_PIN,LOW);
    byte statu=SPI.transfer(NOP);
    digitalWrite(CSN_PIN,HIGH);    

Serial.print("status ");Serial.println(statu,HEX);

conf |= PWR_UP_BIT;                                                   // PWR_UP_BIT set -> idle 1 state (40uA=12board+28nrf)
    digitalWrite(CSN_PIN,LOW);
    SPI.transfer(W_REGISTER);                                         // config register nb is 0x00
    SPI.transfer(conf);
    digitalWrite(CSN_PIN,HIGH);

    delay(5);

#define NOP           0xFF                                            // nop command
    digitalWrite(CSN_PIN,LOW);
    statu=SPI.transfer(NOP);
    digitalWrite(CSN_PIN,HIGH);    

Serial.print("status ");Serial.println(statu,HEX);

conf &= ~PWR_UP_BIT;                                                  // PWR_UP_BIT clr -> power down state (13uA=12 board+1nrf)
    digitalWrite(CSN_PIN,LOW);
    SPI.transfer(W_REGISTER);                                         // config register nb is 0x00
    SPI.transfer(conf);
    digitalWrite(CSN_PIN,HIGH);

#define NOP           0xFF                                            // nop command
    digitalWrite(CSN_PIN,LOW);
    statu=SPI.transfer(NOP);
    digitalWrite(CSN_PIN,HIGH);    

Serial.print("status ");Serial.println(statu,HEX);

    SPI.end();                                                        // disable SPI and pins
    //pinMode(MOSI_PIN,INPUT);                               

    digitalWrite(MOSI_PIN,HIGH);                                      // SPI MISO pin
    digitalWrite(CLK_PIN,HIGH);                                        // SPI CK pin
}
#endif NRF

void setup()
{    
    Serial.begin(115200);
    Serial.println("test sleep ready ");
    Serial.print(getVolts());Serial.println("V");

    hardPwrDown();
    
    pinMode(REED,INPUT_PULLUP); // must stay pullup when reed mode active
    
    nbBlk=5;
}

void loop() 
{

  /* 5 blinks at reset then 8 sec sleep followed by 1 blink until reed contact
   * at every reed contact mode change with 16 blinks tempo to allow magnet removal
   * (when timed sleep mode, reed contact is only checked at awake)
   */

  bool reed=digitalRead(REED);Serial.println(reed);delay(1);
  if(!reed){ nbBlk=3;}                    // if reed on (pullup low) chge mode

  
  switch(nbBlk){
    case 1:sleepPwrDown(T8000);break;     // <4uA timed sleep (power increase when pullup low) (~5uA with NRF ; @3V 2,4uA)
    case 3:
      for(int i=0;i<10;i++){blk(2);}      // magnet removal tempo
      sleepPwrDown(0);                    // <2uA ext interrupt sleep (no timer) (power increase when pullup low) 
                                          // (~3uA with NRF ; @3V too small to be mesured)
      for(int i=0;i<8;i++){blk(2);}       // magnet removal tempo
      break;              
    default:break;
  }

  blk(nbBlk);                     // 1 blink -> timer ; 3 -> reed ;
                                  // le reed passe d'un mode à l'autre
  nbBlk=1;
}
