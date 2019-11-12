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

#ifdef DS18X20
#define THERMO "DS18X20 "
#include <ds18x20.h>
#ifdef DETS
#define WPIN       5          // pin thermomètre
#endif
#ifndef DETS
#define WPIN       3          // pin thermomètre
#endif


#endif DS18X20

/* user fields */

/*** ds18x20 ***/
#ifdef DS18X20
Ds1820 ds1820;
float    previousTemp=-99;
byte     setds[]={0,0x7f,0x80,0x1f},readds[8];   // 1f=93mS 9 bits accu 0,5° ; 3f=187mS 10 bits accu 0,25°
uint32_t nbT=0;         // nbre lectures de temp
#define  TCONVDS1 T64   // sleep PwrDown mS !!
#define  TCONVDS2 T32   // sleep PwrDown mS !!
#endif DS18X20 
char     dsM=' ';
float    temp=-99.99;
bool     dsSta=false;
float    deltaTemp=0.25; 

/*** volts ***/
extern float   volts;


/* cycle functions */

bool checkThings(uint8_t awakeCnt,uint8_t awakeMinCnt,uint8_t retryCnt)
{
  /* Here data acquisition/checking/treatment (if something to send -> return true) */
  /* powerUp equipments who are in userHardPowerDown() */

#ifdef DS18X20

  if(retryCnt==0){            // pas de conversion si retry en cours

    dsSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
    
    ds1820.convertDs(WPIN);
    
    /* *********** mettre en mode 9 bits 0,5° avec sleep 64+32mS -> 96 mA/mS +10mA/mS = 106/60000=1,6uA moyen (1 lect/min) ************* */

  sleepPwrDown(TCONVDS1);
  sleepPwrDown(TCONVDS2);   

    nbT++;
    temp=ds1820.readDs(WPIN);

/*    Serial.print(volts);Serial.print(" ");Serial.print(nbT);Serial.print(" ");Serial.print(temp);
#ifdef DIAG
Serial.print("/");Serial.print(previousTemp);
#endif // DIAG
    delay(1);
*/       
    if( (temp>(previousTemp+deltaTemp)) || (temp<(previousTemp-deltaTemp)) ){
      previousTemp=temp;
      return true;}
  }                        
#endif DS18X20  

  return false; 
}

void messageBuild(char* message,uint8_t* messageLength)
{
  /* Here add user data >>> be carefull to not override 32 bytes <<< */
  
    dtostrf(volts,4,2,(char*)(message+*messageLength));                      
    (*messageLength)+=4;                                            // power voltage
    message[*messageLength]=dsM;                                    // version ds18x20
    (*messageLength)++;
          
    char s='+';if(temp<0){s='-';}
    message[*messageLength]=s;
    dtostrf(temp,5,2,message+*messageLength+1);                     // temp
    if(temp<10){message[(*messageLength)+1]='0';}
    if((strstr(message,"nan")!=0) || !dsSta){memcpy((message+*messageLength),"+00.00",6);}
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
  
#ifdef DS18X20  
  dsSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
  dsM='B';if(ds1820.dsmodel==MODEL_S){dsM='S';}
  ds1820.convertDs(WPIN);
  sleepPwrDown(TCONVDS1);
  sleepPwrDown(TCONVDS2);    
  temp=ds1820.readDs(WPIN);
#endif DS18X20
}

void userHardPowerDown()
{ 
  /* materials to powerDown when sleep */
  
}

#endif // NRF_MODE == 'P'
