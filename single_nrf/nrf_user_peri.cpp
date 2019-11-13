#include "nrf_user_peri.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern uint16_t aw_ok;
extern uint16_t aw_min;


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
  
    dtostrf(volts,4,2,(char*)(message+*messageLength));                      
    (*messageLength)+=4;                                            // power voltage
          
    char s='+';if(temp<0){s='-';}
    message[*messageLength]=s;
    dtostrf(temp,5,2,message+*messageLength+1);                     // temp
    if(temp<10){message[(*messageLength)+1]='0';}
    if((strstr(message,"nan")!=0) || !thSta){memcpy((message+*messageLength),"+00.00",6);}
    (*messageLength)+=6; 
    message[*messageLength]='\0';
}

void importData(byte* data,uint8_t dataLength)
{
  /* Here received data to local fields transfer */
    
  unsigned long     perRefr=0;                
  uint16_t perTemp=0;
  int      sizeRead,srt=0;
  
    perRefr=(long)convStrToNum((char*)(data+ADDR_LENGTH+1),&sizeRead);          // per refresh server
    aw_min=perRefr/STEP_VALUE;
    srt=sizeRead;
    perTemp=(uint16_t)convStrToNum((char*)(data+ADDR_LENGTH+1+srt),&sizeRead);  // per check température
    aw_ok=perTemp/STEP_VALUE;
    srt+=sizeRead;
    deltaTemp=(long)convStrToNum((char*)(data+ADDR_LENGTH+1+srt),&sizeRead);    // pitch mesure 
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
