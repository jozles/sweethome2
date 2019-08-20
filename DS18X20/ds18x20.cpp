
/*
    New release 2019 : add setds/readrom function ; return ds18x20 model ('S'/'B')
    necessary to execute the following to setup the model.
      byte setds[]={0,0x7f,0x80,0x3f},readds[8];    // converting time 187mS 10 bits accu 0,25°
      int v=ds1820.setDs(WPIN,setds,readds);        // readds[0] = model

    Interrupt-free, small memory wasting ds1820 interface (less than 1K)
    this is working with any pin accepting digitalRead and digitalWrite.
    Interrupt suspend should not be longer than 85 uSec for each written bit and
    less than 10uSec for the other occurencys.

    2 steps : first converting (could be as long as ... mSec depending on accuracy) then getting value.

    convertDs(uint8_t pin) returns 1 or error codes (see below)
    readDs(uint8_t pin) returns float value between -55 and +125 or error codes (see below)

    getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein)
    return a status (codes below) and fill framein with data provided by Ds in accordance
    with the frameout data transmitted to Ds.
    (see datasheet)

    pin = number of Arduino pin used to connect Ds unit

    Les DS18S20 "bugués" toujours négatifs sont utilisables pour les temp positives en appliquant le masque 0x7F sur la valeur lue
*/

#include "Arduino.h"
#include "Ds18x20.h"

// possible return status for getDs()/readDs functions
// values between -55 to +125 are valid data
// romDs output family code 256+code

#define TOPRES  -100
#define CRC_ERR -101

#define T_WAIT_PRESENCE 240
#define FAUX 0
#define VRAI 1
bool bitmessage=FAUX; // VRAI Serial.print les bits du meesage

Ds1820::Ds1820()  // constructeur de la classe
{
}

byte calcBitCrc(byte shiftReg, byte data_bit);
static uint8_t maskbit[]={1,2,4,8,16,32,64,128};  //  128,64,32,16,8,4,2,1

void hexprint(byte hh)
{
  if(hh&0xf0==0x00){Serial.print("0");}Serial.print(hh,HEX);Serial.print(" ");
}

void writeDs(uint8_t pin,uint8_t nbbyteout,uint8_t* frameout)
//write command nbbyteout @ nbbit/byte
{
  boolean pinstat=HIGH;
  uint8_t i=0,j=0,delaylow,delayhigh;

  digitalWrite(pin,HIGH);
  pinMode(pin,OUTPUT);
  for(i=0;i<nbbyteout;i++){
    for(j=0;j<8;j++){
      //delaylow=60;delayhigh=0;pinstat=LOW;if((frameout[i] & maskbit[j])!=0){delaylow=0;delayhigh=60;pinstat=HIGH;}
      //pinstat=!(frameout[i] & maskbit[j]);
      pinstat=(frameout[i]>>j)&0x01;
      noInterrupts();
#define DELAYCELL 50
      digitalWrite(pin,LOW);
      // if bit =1 low state about 2,7 uSec long (UNO)
      //delayMicroseconds(delaylow);
      digitalWrite(pin,pinstat);
      delayMicroseconds(DELAYCELL);
      //delayMicroseconds(delayhigh);
      digitalWrite(pin,HIGH);
      interrupts();
    }
  }
  pinMode(pin,INPUT);
}

int Ds1820::romDs(uint8_t pin,uint8_t* framein)   // read rom command
                                                  // romDs should not be used without function command
                                                  // framein must be at least 8 bytes long
{                                                 // B = 0x28 ; S = 0x10 , '' = 0x10
  uint8_t cmd[1]={0x33};
  int v;

  v=getDs(pin,cmd,1,framein,8);

  if(v<=TOPRES){return v;}   // error
//Serial.print("readRom cd(0x33) ");for(int ds=0;ds<8;ds++){hexprint(framein[ds]);}Serial.println();
  dsmodel=framein[0];
  return framein[0]+256;
}

int Ds1820::setDs(uint8_t pin,uint8_t* frameout,uint8_t* framein)      // read rom and write scratchpad
                 // framein must be at least 8 bytes long
                 // frameout must be at least 4 bytes long : 0 function
                 //                                          1 High alarm byte (max=0x7f)
                 //                                          2 Low  alarm byte (min=0x80)
                 //                                          3 config byte (0xx11111) xx=11 12 bits conv
{
  frameout[0]=0x4E;
  int v=romDs(pin,framein);
  if(v<256){return v;}       // error

  writeDs(pin,4,frameout);
  return 1;
}


int Ds1820::convertDs(uint8_t pin)
{
  uint8_t h[2]={0xCC,0x44},r[2];
  return getDs(pin,h,2,r,0);
}

float Ds1820::readDs(uint8_t pin)
{
  uint8_t cmd[2]={0xCC,0xBE},rep[9];
  uint8_t r0,r1;
  uint16_t rp=0,rp1=0,rp2=0;
  int v;
  float th=0,th0=0;

  v=getDs(pin,cmd,2,rep,9);

  if(v<=TOPRES){if(bitmessage){for(int rr=0;rr<9;rr++){hexprint(rep[rr]);}Serial.println();}return (float)v;}   // erreur TO présence ou CRC (-100,-101 etc)
  else{

//   complément à 2 :
//     DS1820 & S20 : si rep[1] != 0 c'est négatif : th = -(complt rep[0] + 1)
//                    le bit 0 vaut 0,5° le traiter à part
//     DS18B20 : si bit 0x80 de rep[1]!=0 c'est négatif : th = -(complt rep + 1)
//               valeur : rep>>4+0,0625*rep[0]&0x0F

      r0=rep[1] & 0x80;

      if(dsmodel==MODEL_B){                                // DS18B20
      rp=rep[1];
      rp=(rp<<8) | rep[0];
      if(r0 != 0){rp=(rp^0xFFFF)+1;}
      rp1=rp>>4;
      rp2=rp&0x000F;
      th0=((float)rp2)*((float)0.0625);
      th=(float)rp1 + th0;
    }                                                 // DS1820 - DS18S20
    else{
      if(rep[1]!=0){
        th0=-((rep[0]^0xFF)+1);
        th0=(uint8_t)rep[0]>>1;
      }
      else {th0=(rep[0]&0x7f)>>1;}
      th=(float)(th0)+0.5*((uint8_t)(rep[0]&0x01));       // th précision 0.5
      th=(float)(th0)-0.25+(float)(rep[7]-rep[6])/rep[7]; // th précision améliorée
    }

    if(r0 != 0){th=-th;}
    //Serial.print("th=");Serial.print(th);Serial.print(" rep[1,0]=");Serial.print(rep[1],HEX);
    //Serial.print(" ");Serial.println(rep[0],HEX);
  return th;
  }
}

int Ds1820::getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein)
{

  uint8_t crc=0;
  uint8_t i=0,j=0,v=0;
  long micr=0;
  uint8_t delaylow=0,delayhigh=0;
  boolean pinstat=HIGH;
  uint8_t buff[128];for(i=0;i<128;i++){buff[i]=0;}

  //reset pulse
  noInterrupts();
  digitalWrite(pin,HIGH);
  pinMode(pin,OUTPUT);
  interrupts();
  delay(1);
  digitalWrite(pin,LOW);
  delayMicroseconds(500);
  pinMode(pin,INPUT);
  delayMicroseconds(65);

  //presence detect
  micr=micros();
  while(1){
    if(micros()>(micr+T_WAIT_PRESENCE)){return TOPRES;} // pas de réponse
    if(digitalRead(pin)==LOW){delayMicroseconds(400);break;}
  }
  // write command
  writeDs(pin,nbbyteout,frameout);

  //read response nbbytein @ 8 bits/byte

  if(nbbytein!=0){
    memset(framein,0,nbbytein);
    for (i=0;i<nbbytein;i++){
      for (j=0;j<8;j++){
        v=i*8+j;
        noInterrupts();
        digitalWrite(pin,LOW);
        pinMode(pin,OUTPUT);
        delayMicroseconds(0);
        // delay between low and read about 5,2 uSec (UNO) + delayMicroseconds
        // delayMicroseconds is n-1 micros ; pinMode about 3,1 uSec
        pinMode(pin,INPUT);
        delayMicroseconds(1);           // environ 1,5uS pour blocage bas par le DS
        buff[v]=digitalRead(pin);
        interrupts();
        if(buff[v]!=0){framein[i]+= maskbit[j];}
        delayMicroseconds(50);
      }
    }
  }
  // check CRC
  byte shift_reg=0;
  for(i=0;i<nbbytein;i++){
    for(j=0;j<8;j++){
      shift_reg=calcBitCrc(shift_reg,buff[i*8+j]);
      if(bitmessage){Serial.print(buff[i*8+j]);}
    }
    if(bitmessage){Serial.print(" ");if(i==(nbbytein-2)){hexprint(shift_reg);}}
  }
  if(bitmessage){Serial.print(" CRC ");hexprint(shift_reg);Serial.println();}
  if(shift_reg==0){return 1;}else{return CRC_ERR;}
}

byte Ds1820::calcBitCrc (byte shiftReg, byte data_bit)
{
  byte fb;

  fb = (shiftReg & 0x01) ^ data_bit;
   /* exclusive or least sig bit of current shift reg with the data bit */
   shiftReg = shiftReg >> 1;
   if (fb==1){shiftReg = shiftReg ^ 0x8C;}    /* CRC ^ binary 1000 1100 */
   return(shiftReg);
}

