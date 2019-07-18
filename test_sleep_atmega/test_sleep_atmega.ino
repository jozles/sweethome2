

#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define INTPIN 2  // 2 ou 3 sur UNO, NANO ; 2,3,18,19,20,21 sur Mega, tous sur Due

bool reveil = false;

#define LED 5
uint16_t blkdelay=0;
long blktime=0;
uint8_t bcnt=0;
#define TBLK 1
#define DBLK 1000
#define IBLK 80

void delayBlk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb,long dly)
{
  long tt=millis();
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);}
}

void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb)
{
  if((millis()-blktime)>blkdelay){
    if(digitalRead(LED)==LOW){
      digitalWrite(LED,HIGH);blkdelay=dur;}
    else{digitalWrite(LED,LOW);
      if(bcnt<bnb){blkdelay=bint;bcnt++;}
      else{blkdelay=bdelay;bcnt=1;}}
    blktime=millis();
  }
}


void isrWakeCallback() {
    /* rien */
}

ISR(WDT_vect)                      // ISR interrupt service pour vecteurs d'IT du MPU (ici vecteur WDT)
{
  reveil = true;
  Serial.println("interruption wdt");
}


void sleepPwrDown() {
 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    power_adc_disable();
    sleep_enable();
 
    sleep_mode();
  
    sleep_disable();
    power_all_enable();    
}

void wdtSetup()
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

  interrupts();
}


void setup() {
    
    Serial.begin(115200);
    
    pinMode(LED,OUTPUT);

    Serial.print("ready WDTCSR=");Serial.print(WDTCSR,HEX);

    wdtSetup();

    Serial.print("->");Serial.println(WDTCSR,HEX);
}

void loop() {

  delayBlk(TBLK,DBLK-(TBLK+IBLK)*2-TBLK,IBLK,3,4000);
  //sleepPwrDown();
  wdt_reset();
}
