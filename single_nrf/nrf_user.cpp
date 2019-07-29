#include "nrf_user.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"

extern uint16_t aw_ok;
extern uint16_t aw_min;

/* user includes */

#include "shconst2.h"
#include "shutil2.h"
#define DS18X20
#ifdef DS18X20
#include <ds18x20.h>
#define WPIN       3          // pin thermomètre
#endif DS18X20

/* user fields */

/*** ds18x20 ***/
#ifdef DS18X20
Ds1820 ds1820;
float    temp,previousTemp=-99;
float    deltaTemp=0.25; 
bool     dsSta=false;
byte     setds[]={0,0x7f,0x80,0x3f},readds[8];   // 187mS 10 bits accu 0,25°
char     dsM;
uint32_t nbT=0;         // nbre lectures de temp
#define  TCONVDS 200     // mS !!
#endif DS18X20 

/*** volts ***/
float   volts=0;



bool checkThings(uint8_t retryCnt)
{
  /* Here data acquisition/checking/treatment (if something to send -> return true) */
  /* powerUp equipments who are in userHardPowerDown() */
#ifdef DS18X20

  if(retryCnt==0){            // pas de conversion si retry en cours
    ds1820.convertDs(WPIN);
    
    #if TCONVDS != 200
    tconv // TCONVDS not 200 ... should adjust sleep time
    #endif

    sleepPwrDown(T250); 

    nbT++;
    temp=ds1820.readDs(WPIN);
    Serial.print(nbT);Serial.print(temp);Serial.print("/");Serial.println(previousTemp);
    if( (temp>(previousTemp+deltaTemp)) || (temp<(previousTemp-deltaTemp)) ){
      previousTemp=temp;
      return true;}
  }
  return false;  
#endif DS18X20  
 
}

void messageBuild(char* message,uint8_t* messageLength)
{
  /* Here add user data >>> be carefull to not override 32 bytes <<< */
    message[*messageLength+LENVERSION]=dsM;                                    // version ds18x20
    *messageLength++;
    message[*messageLength]='+';if(temp<0){message[*messageLength]='-';}      
    *messageLength+=1;
    dtostrf(temp,5,2,message+*messageLength);                                  // temp
    if((strstr(message,"nan")!=0) || !dsSta){strcpy((char*)(message+*messageLength),"+00.00\0");}
    *messageLength+=5;
    // get voltage 
    dtostrf(volts,4,2,message+*messageLength);
    *messageLength+=4;                                                         // power voltage
    message[*messageLength]='\0';
}

void importData(char* data,uint8_t dataLength)
{
  /* Here received data to local fields transfer */
    
  long     perRefr=0;                
  uint16_t perTemp=0;
  int      sizeRead;

    perRefr=(long)convStrToNum(data+ADDR_LENGTH+1+MPOSPERREFR-MPOSPERREFR,&sizeRead);     // per refresh server
    aw_ok=perTemp/STEP_VALUE;
    perTemp=(uint16_t)convStrToNum(data+ADDR_LENGTH+1+MPOSTEMPPER-MPOSPERREFR,&sizeRead); // per check température
    aw_min=perRefr/STEP_VALUE;
    deltaTemp=(long)convStrToNum(data+ADDR_LENGTH+1+MPOSPITCH-MPOSPERREFR,&sizeRead);     // pitch mesure 
}

void userHardSetup()
{ /* initial setup after reset */
#ifdef DS18X20  
  dsSta=ds1820.setDs(WPIN,setds,readds);    // setup ds18b20
  dsM='B';if(ds1820.dsmodel==MODEL_S){dsM='S';}
#endif DS18X20

}

void userHardPowerDown()
{ /* materials to powerDown when sleep */
  
}
