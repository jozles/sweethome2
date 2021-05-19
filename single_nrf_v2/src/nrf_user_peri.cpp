#include "nrf_user_peri.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern uint16_t aw_ok;
extern uint16_t aw_min;
extern uint32_t nbS;
extern uint32_t nbL;

extern bool diags;

/* gestion user data du périphérique */

#if NRF_MODE == 'P'

/* user includes */

#include "shconst2.h"
#include "shutil2.h"


/* user fields */


/* system fields */

extern float   volts;
extern float   temp;
extern float   deltaTemp;
extern bool    thSta;
extern float   period;


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
  
    Serial.print(" volts ");Serial.print(volts);
    dtostrf(volts,4,2,(char*)(message+*messageLength));             //          - 4                    
    (*messageLength)+=4;                                            // power voltage

    Serial.print(" temp ");Serial.print(temp);      
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
    Serial.print(" adc1 ");Serial.print(adcVal,HEX);                                            
    adcVal=adcRead0(A2ADMUXVAL,0);conv_htoa((char*)(message+*messageLength),(byte*)&adcVal,LENUINT16_T);
    (*messageLength)+=4;                                            // anal2    - 4
    Serial.print(" adc2 ");Serial.println(adcVal,HEX);
    message[*messageLength]='\0';                                   //          - 1
}

void importData(byte* data,uint8_t dataLength)
{
  /* Here received data to local fields transfer */
    
  unsigned long     perRefr=0;                
  uint16_t perTemp=0;
  int      sizeRead,srt=0;
  
    perRefr=(long)convStrToNum((char*)(data+ADDR_LENGTH+1),&sizeRead);          // per refresh server
    aw_min=perRefr/period;
    srt=sizeRead;
    perTemp=(uint16_t)convStrToNum((char*)(data+ADDR_LENGTH+1+srt),&sizeRead);  // per check température
    aw_ok=perTemp/period;
    srt+=sizeRead;
    deltaTemp=(convStrToNum((char*)(data+ADDR_LENGTH+1+srt),&sizeRead))/100;    // pitch mesure !!!!!!!!!!!!!!!!!!!!!! bug ??????? deltaTemp est float ; controler data
                                                                                // devrait être convStrToNum((char*)(data+ADDR_LENGTH+1+srt),&sizeRead)/100;
                                                                                // vérifier srt...   
    if(diags){
    Serial.println();Serial.print("£ ");
    Serial.print(nbS);Serial.print("/");Serial.print(nbL);Serial.print(" | ");     // nbS com nb ; nbL loop nb
    for(uint8_t ii=0;ii<dataLength;ii++){Serial.print((char)data[ii]);delayMicroseconds(100);}Serial.print(" ");
    Serial.print("per_s=");Serial.print(perRefr);Serial.print(" per_t=");Serial.print(perTemp);Serial.print(" period=");Serial.print(period);                                                                                   
    Serial.print(" aw_min=");Serial.print(aw_min);Serial.print(" aw_ok=");Serial.print(aw_ok);Serial.print(" pth=");Serial.print(deltaTemp);
    delay(4);
    Serial.println(" £");                                                                                   
    }
}


void userResetSetup()
{
  /* initial setup after reset */

}

void userHardPowerDown()
{ 
  /* materials to powerDown when sleep */
  
}

#endif // NRF_MODE == 'P'