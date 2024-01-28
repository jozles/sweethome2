#include <shconst2.h>
#include <shutil2.h>
#include "nrf_user_peri.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern uint16_t aw_ok;
extern uint16_t aw_min;
extern uint32_t nbS;
extern uint32_t nbL;
extern unsigned long t_on;

extern bool diags;

/* gestion user data du périphérique */

#if NRF_MODE == 'P'

/* user includes */


/* user fields */

#define RAD1 5
#define RAD2 6
#define RADSTEP 25

/* system fields */

extern char* chexa;

extern float   volts;
extern float   temp;
extern float   deltaTemp;
extern bool    thSta;
extern float   period;
extern uint16_t userData[2];
uint16_t       analOutput=0;
uint16_t       prevAnalOutput=0;
uint8_t        periCfg=0;        

/* cycle functions */

bool checkThings(uint8_t awakeCnt,uint8_t awakeMinCnt,uint8_t retryCnt)
{
  /* Here data acquisition/checking/treatment (if something to send -> return true) */
  /* powerUp equipments who are in userHardPowerDown() */

  return false; 
}

void messageBuild(char* message,uint8_t* messageLength)
{
  /* Here add user data >>> be carefull to not override 32 bytes <<< */
  
    //Serial.print(" ");Serial.print(volts);
    dtostrf(volts,4,2,(char*)(message+*messageLength));             //          - 4                    
    (*messageLength)+=4;                                            // power voltage

    //Serial.print("V ");Serial.print(temp);Serial.print("°");
    char s='+';if(temp<0){s='-';}
    message[*messageLength]=s;

    dtostrf(temp,5,2,message+*messageLength+1);                     // temp     - 6
    if(temp<10){message[(*messageLength)+1]='0';}
    if((strstr(message,"nan")!=0) || !thSta){memcpy((message+*messageLength),"+00.00",6);}

    (*messageLength)+=6;   
    message[*messageLength]='0'+bitRead(PORT_DIG1,BIT_DIG1)+bitRead(PORT_DIG2,BIT_DIG2)*2+bitRead(PORT_REED,BIT_REED)*4;
    (*messageLength)++;                                             // digits   - 1

    uint16_t adcVal=0;
    #define LENUINT16_T 2
    adcVal=adcRead0(A1ADMUXVAL,0);conv_htoa((char*)(message+*messageLength),(byte*)&adcVal,LENUINT16_T);
    (*messageLength)+=4;                                            // anal1    - 4
    if(diags){Serial.print(" adc1 ");Serial.print(adcVal,HEX);}
    adcVal=adcRead0(A2ADMUXVAL,0);conv_htoa((char*)(message+*messageLength),(byte*)&adcVal,LENUINT16_T);
    (*messageLength)+=4;                                            // anal2    - 4
    if(diags){Serial.print(" adc2 ");Serial.println(adcVal,HEX);}
    message[*messageLength]='\0';                                   //          - 1
}

/*
uint16_t packGet(char* data,uint8_t len){       // lecture depuis les poids faibles !!!
  uint16_t v=0;
  char cc[2];cc[1]='\0';
  uint8_t k[4];k[0]=2;k[1]=3;k[2]=0;k[3]=1;
  for(int8_t i=0;i<len;i++){
      cc[0]=*(data+k[i]);v<<=4;v+=strstr(chexa,cc)-chexa;}
return v;
}
*/

uint16_t packGet(char* data,uint8_t len){
  uint16_t v=0;
  char cc[2];cc[1]='\0';
  for(int8_t i=len-1;i>0;i-=2){
    //Serial.print(i);Serial.print(' ');Serial.print((char)*(data+i-1));Serial.print(' ');Serial.print(v);
    v<<=4;cc[0]=*(data+i-1);v+=strstr(chexa,cc)-chexa;
    //Serial.print(' ');Serial.print((char)*(data+i));Serial.print(' ');Serial.print(v);
    v<<=4;cc[0]=*(data+i);v+=strstr(chexa,cc)-chexa;
    //Serial.print(' ');Serial.print((uint8_t)(strstr(chexa,cc)-chexa));Serial.print(' ');Serial.println(v);
    }
  return v;
}

void importData(byte* data,uint8_t dataLength)
{
  /* Here received data to local fields transfer */
    
  uint16_t perRefr=0;
  uint16_t perTemp=0;
  int      sizeRead,srt=0;

  if(memcmp(VERSION,"1.c",3)<=0){                                                 // version <= 1.C
    unsigned long perRefr=(long)convStrToNum((char*)(data+NRF_ADDR_LENGTH+1),&sizeRead);          // per refresh server
    aw_min=perRefr/period;
    srt=sizeRead;
    perTemp=(uint16_t)convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead);  // per check température
    aw_ok=perTemp/period;
    srt+=sizeRead;
    deltaTemp=(convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead))/100;  // pitch mesure !!!!!!!!!!!!!!!!!!!!!! bug ??????? deltaTemp est float ; controler data
                                                                                  // devrait être convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead)/100;
                                                                                  // vérifier srt...   
    srt+=sizeRead;
    //Serial.print(" ");Serial.print(srt);
    *(data+NRF_ADDR_LENGTH+1+srt+8)='\0';
//dumpstr((char*)data,32);
    Serial.print(":::");Serial.print((char*)(data+NRF_ADDR_LENGTH+1+srt));
      
    userData[0]=packGet((char*)(data+NRF_ADDR_LENGTH+1+srt),4);                   // forme '_hhhhhhhh' //Serial.println(userData[0]);
    srt+=5;
    userData[1]=packGet((char*)(data+NRF_ADDR_LENGTH+srt),4);
    srt+=4;
    analOutput=(userData[0]&=0xf800)>>=11;analOutput+=(userData[1]&=0xf800)>>=6;
    Serial.print("/");Serial.print(userData[0]);Serial.print("-");Serial.print(userData[1]);Serial.print("/");Serial.print(analOutput);
    Serial.print('-');Serial.print(prevAnalOutput);
    if((NRF_ADDR_LENGTH+srt+1+2)<=MAX_PAYLOAD_LENGTH){periCfg=(uint8_t)packGet((char*)(data+NRF_ADDR_LENGTH+srt+1),2);} // forme '_hh'
    else Serial.print(" decap MAX_PAYLOAD_LENGTH ");
    if(prevAnalOutput!=analOutput){radUpdate(analOutput);prevAnalOutput=analOutput;} 
  }
  else{                                                                           // version > 1.c
    perRefr=0;conv_atob((char*)(data+NRF_ADDR_LENGTH+1),&perRefr,5);              // per refresh server
    aw_min=perRefr/period;
    srt=1+5;
    conv_atob((char*)(data+NRF_ADDR_LENGTH+srt),&perTemp,5);                      // per check température
    aw_ok=perTemp/period;
    srt+=5;
    uint16_t pitch=0;
    conv_atob((char*)(data+NRF_ADDR_LENGTH+srt),&pitch,4);                        // pitch mesure
    deltaTemp=((float) pitch)/100;
    srt+=4;
    userData[0]=packGet((char*)(data+NRF_ADDR_LENGTH+srt),4);                     // forme 'hhhhhhhh' //Serial.println(userData[0]);
    //Serial.print(" >> ");Serial.println(packGet((char*)(data+NRF_ADDR_LENGTH+srt),4));
    srt+=4;
    userData[1]=packGet((char*)(data+NRF_ADDR_LENGTH+srt),4);                     //Serial.println(userData[1]);
    //Serial.print(" >> ");Serial.println(packGet((char*)(data+NRF_ADDR_LENGTH+srt),4));
    srt+=4;
    
    analOutput=(userData[0]&0xf800)>>11;userData[0]&=0x7ff;analOutput+=(userData[1]&0xf800)>>6;userData[1]&=0x7ff;
    periCfg=(uint8_t)packGet((char*)(data+NRF_ADDR_LENGTH+srt),2);

    Serial.print("/");Serial.print(userData[0]);Serial.print("-");Serial.print(userData[1]);Serial.print("/");Serial.print(analOutput);
    Serial.print('-');Serial.print(prevAnalOutput);Serial.print(" cfg:");Serial.println(periCfg,HEX);delay(5);
    
    if(prevAnalOutput!=analOutput && (periCfg&PERI_RAD)!=0){radUpdate(analOutput);prevAnalOutput=analOutput;}                  
  
  }

    if(diags){
    Serial.print("\n£ ");
    Serial.print(nbS);Serial.print("/");Serial.print(nbL);Serial.print(" | ");     // nbS com nb ; nbL loop nb
    for(uint8_t ii=0;ii<dataLength;ii++){Serial.print((char)data[ii]);delayMicroseconds(100);}Serial.print(" ");
    Serial.print("per_s=");Serial.print(perRefr);Serial.print(" per_t=");Serial.print(perTemp);Serial.print(" period=");Serial.print(period);                                                                                   
    Serial.print(" aw_min=");Serial.print(aw_min);Serial.print(" aw_ok=");Serial.print(aw_ok);Serial.print(" pth=");Serial.print(deltaTemp);
    Serial.print(" usr=");Serial.print(userData[0]);Serial.print(" / ");Serial.print(userData[1]);
    Serial.print(" anOut=");Serial.print(analOutput);Serial.print(" / ");Serial.print(analOutput,HEX);
    Serial.print(" cfg=");Serial.print(periCfg);Serial.print(" / ");Serial.print(periCfg,HEX);
    delay(4);
    Serial.println(" £");                                                                                   
    }
}


void userResetSetup()
{
  /* initial setup after reset */
  radSetup();
}

void userHardPowerDown()
{ 
  /* materials to powerDown when sleep */
  
}

void radSetup(){
  Serial.print("rad init : ");
  analOutput=0;
  pinMode(RAD1,INPUT_PULLUP);
  pinMode(RAD2,INPUT_PULLUP);
  delay(10);
  Serial.print(digitalRead(RAD1));Serial.println(digitalRead(RAD2));delay(10);
  while((digitalRead(RAD1)+digitalRead(RAD2)!=2) && (millis()-t_on)<30000){blink(1);}

/* 
  uint16_t preRad,curRad;
  uint16_t cnt=0;
  preRad=0;//digitalRead(RAD1)+digitalRead(RAD2)<<8;
  while(1){
    delay(1);
    curRad=(digitalRead(RAD1))+(digitalRead(RAD2)<<8);
    if(preRad!=curRad){Serial.print(cnt);Serial.print(" ");Serial.print(curRad&0x01);Serial.print(" ");Serial.println((curRad>>8)&0x01);
      preRad=curRad;cnt=0;}
    else cnt++;
    if(cnt>99){cnt=99;}
  }
*/
/*
  char c;
  Serial.println("valeur ascii (0x20->0x7F)-0x20 saisir un car");
  while(1){
    if(Serial.available()){c=Serial.read();c-=0x20;Serial.println((uint8_t)c);radUpdate(c);}
  }

  uint16_t max=10;
  for(uint16_t consigne=0;consigne<max;consigne++){
    Serial.println(consigne);delay(2000);radUpdate(consigne);
    
    char c='*';
    while(c!=' ' && c!='q'){
      if(Serial.available()){c=Serial.read();}
    }
    if(c=='q'){Serial.println("end");delay(10);Serial.end();consigne=max;}
  }
*/
}

void radUpdate(uint16_t value)
{
  
  if(value!=0){
    if(value>42){value=42;}

    digitalWrite(RAD2,HIGH);pinMode(RAD1,OUTPUT); // setup mode DOWN
    digitalWrite(RAD1,LOW);pinMode(RAD2,OUTPUT);

    for(uint8_t i=0;i<22;i++){                    // RAZ
      digitalWrite(RAD2,LOW);delay(RADSTEP);
      digitalWrite(RAD1,HIGH);delay(RADSTEP);
      digitalWrite(RAD2,HIGH);delay(RADSTEP);
      digitalWrite(RAD1,LOW);delay(RADSTEP);      
    }

    digitalWrite(RAD2,!value&0x01);delay(RADSTEP);       // setup mode UP
    digitalWrite(RAD1,!value&0x01);delay(RADSTEP);
    for(uint16_t i=1;i<=value;i++){
      digitalWrite(RAD2,i&0x01);delay(RADSTEP);
      digitalWrite(RAD1,i&0x01);delay(RADSTEP);      
    }
  }

/*
    char(c);
    while(1){
      c=' ';
      while(c==' '){
        if(Serial.available()){c=Serial.read();Serial.print(c);digitalWrite(RAD2,c-48);}}
      c=' ';
      while(c==' '){
      if(Serial.available()){c=Serial.read();Serial.println(c);digitalWrite(RAD1,c-48);}}
    }
*/
    pinMode(RAD1,INPUT_PULLUP);
    pinMode(RAD2,INPUT_PULLUP);
    digitalWrite(RAD1,HIGH);
    digitalWrite(RAD2,HIGH);

}

#endif // NRF_MODE == 'P'