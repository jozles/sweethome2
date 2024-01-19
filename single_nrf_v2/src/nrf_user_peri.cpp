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

extern bool diags;

/* gestion user data du périphérique */

#if NRF_MODE == 'P'

/* user includes */


/* user fields */


/* system fields */

extern char* chexa;

extern float   volts;
extern float   temp;
extern float   deltaTemp;
extern bool    thSta;
extern float   period;
extern uint16_t userData[2];
uint16_t       analOutput;

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
uint32_t convStrToHex(char* str,uint8_t len)
{
  uint8_t v0=0;
  uint32_t v=0;
  int i=0;

  for(i=0;i<len;i++){
    v0=strstr(chexa,&str[i])-chexa;if(v0>15){v0-=6;}
    //if(str[i]>='0' && str[i]<='9'){v0=*(str+i)-48;}
    //else if(str[i]>='A' && str[i]<='F'){v0=*(str+i)-64+10;}
    //else if(str[i]>='a' && str[i]<='f'){v0=*(str+i)-64+10;}
    //else v0=0;
    v+=v0;
    v=v<<4;
  }
  return v;
  //Serial.print("s>n str,num=");Serial.print(string);Serial.print(" ");Serial.println(v*minus);
}
*/

uint16_t usdGet(char* data){
  uint16_t v=0;
  char cc[2];cc[1]='\0';
  uint8_t k[4];k[0]=2;k[1]=3;k[2]=0;k[3]=1;
  for(int8_t i=0;i<4;i++){
      cc[0]=*(data+k[i]);v<<=4;v+=strstr(chexa,cc)-chexa;}
return v;
}

void importData(byte* data,uint8_t dataLength)
{
  /* Here received data to local fields transfer */
    
  unsigned long     perRefr=0;                
  uint16_t perTemp=0;
  int      sizeRead,srt=0;
  
    perRefr=(long)convStrToNum((char*)(data+NRF_ADDR_LENGTH+1),&sizeRead);          // per refresh server
    aw_min=perRefr/period;
    srt=sizeRead;
    perTemp=(uint16_t)convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead);  // per check température
    aw_ok=perTemp/period;
    srt+=sizeRead;
    deltaTemp=(convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead))/100;    // pitch mesure !!!!!!!!!!!!!!!!!!!!!! bug ??????? deltaTemp est float ; controler data
                                                                                // devrait être convStrToNum((char*)(data+NRF_ADDR_LENGTH+1+srt),&sizeRead)/100;
                                                                                // vérifier srt...   
    srt+=sizeRead;
    Serial.print(" ");Serial.print(srt);Serial.print(":::");Serial.println((char*)(data+NRF_ADDR_LENGTH+1+srt)); 
      
    userData[0]=usdGet((char*)(data+NRF_ADDR_LENGTH+1+srt));//Serial.println(userData[0]);
    srt+=4;
    userData[1]=usdGet((char*)(data+NRF_ADDR_LENGTH+1+srt));//Serial.println(userData[1]);
    srt+=4;
    analOutput=(userData[0]&=0xf800)>>=11;analOutput+=(userData[1]&=0xf800)>>=6;
                  
    if(diags){
    Serial.print("\n£ ");
    Serial.print(nbS);Serial.print("/");Serial.print(nbL);Serial.print(" | ");     // nbS com nb ; nbL loop nb
    for(uint8_t ii=0;ii<dataLength;ii++){Serial.print((char)data[ii]);delayMicroseconds(100);}Serial.print(" ");
    Serial.print("per_s=");Serial.print(perRefr);Serial.print(" per_t=");Serial.print(perTemp);Serial.print(" period=");Serial.print(period);                                                                                   
    Serial.print(" aw_min=");Serial.print(aw_min);Serial.print(" aw_ok=");Serial.print(aw_ok);Serial.print(" pth=");Serial.print(deltaTemp);
    Serial.print(" usr=");Serial.print(userData[0]);Serial.print(" / ");Serial.print(userData[1]);
    Serial.print(" anOut=");Serial.print(analOutput);Serial.print(" / ");Serial.print(analOutput,HEX);
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