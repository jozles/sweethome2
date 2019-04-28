
#include "Arduino.h"
#include "shconst.h"

extern char  pass[];
extern char* chexa;

static unsigned long blinktime=0;
uint8_t nbreBlink=0;          // si nbreBlink impair   -> blocage
uint8_t cntBlink=0;

#define TIMEOVFSLOTNB 10
uint32_t timeOvfSlot[TIMEOVFSLOTNB];

extern char* usrnames;
extern char* usrpass;


  /* debug
int   cntdebug[NBDBPTS];
long  timedebug[NBDBPTS*NBDBOC];
int*  v0debug[NBDBPTS*NBDBOC];
int*  v1debug[NBDBPTS*NBDBOC];
char* v2debug[NBDBPTS*NBDBOC];
char* v3debug[NBDBPTS*NBDBOC];
*/

int   int00=0;
int*  int0=&int00;

int convIntToString(char* str,int num)
{
  int i=0,t=0,num0=num;
  while(num0!=0){num0/=10;i++;}                 // comptage nbre chiffres partie entière
  t=i;
  for (i;i>0;i--){num0=num%10;num/=10;str[i-1]=chexa[num0];}
  str[t]='\0';
  return t;
}

int convNumToString(char* str,float num)  // retour string terminée par '\0' ; return longueur totale '\0' inclus
{
  int i=0,v=0,t=0;

  t=convIntToString(str,(int)num);

  num=num-(int)num;
  str[t]='.';
  for(i=0;i<2;i++){num=num*10;v=(int)num;num=num-v;str[t+1+i]=chexa[v];}
  t+=3;
  str[t]='\0';

  return t;
}

void serialPrintIp(uint8_t* ip)
{
  for(int i=0;i<4;i++){Serial.print(ip[i]);if(i<3){Serial.print(".");}}
  Serial.print(" ");
}

void charIp(byte* nipadr,char* aipadr)
{
  char buf[8];
  for(int i=0;i<4;i++){
        sprintf(buf,"%d",nipadr[i]);strcat(aipadr,buf);if(i<3){strcat(aipadr,".");}
  }
}

void conv_atoh(char* ascii,byte* h)
{
    uint8_t c=0;
  c = (uint8_t)(strchr(chexa,ascii[0])-chexa)<<4 ;
  c |= (uint8_t)(strchr(chexa,ascii[1])-chexa) ;
  *h=c;
}

void conv_htoa(char* ascii,byte* h)
{
    uint8_t c=*h,d=c>>4,e=c&0x0f;
//    Serial.print(c,HEX);Serial.print(" ");Serial.print(d,HEX);Serial.print(" ");Serial.print(e,HEX);Serial.print(" ");
//    Serial.print(chexa[d],HEX);Serial.print(" ");Serial.println(chexa[e],HEX);
        ascii[0]=chexa[d];ascii[1]=chexa[e];
        // Serial.print(c,HEX);Serial.print(" ");Serial.print(ascii[0],HEX);Serial.print(" ");Serial.println(ascii[1],HEX);
}

void dumpstr0(char* data,uint8_t len)
{
    char a[]={0x00,0x00,0x00};
    uint8_t c;
    Serial.print((long)data,HEX);Serial.print("   ");
    for(int k=0;k<len;k++){conv_htoa(a,(byte*)&data[k]);Serial.print(a);Serial.print(" ");}
    Serial.print("    ");
    for(int k=0;k<len;k++){
            c=data[k];
            if(c<32 || c>127){c='.';}
            Serial.print((char)c);
    }
    Serial.println();
}

void dumpstr(char* data,uint16_t len)
{
    while(len>=16){len-=16;dumpstr0(data,16);data+=16;}
    if(len!=0){dumpstr0(data,len);}
}


byte calcBitCrc (byte shiftReg, byte data_bit)
{
  byte fb;

  fb = (shiftReg & 0x01) ^ data_bit;
   /* exclusive or least sig bit of current shift reg with the data bit */
   shiftReg = shiftReg >> 1;                  /* shift one place to the right */
   if (fb==1){shiftReg = shiftReg ^ 0x8C;}    /* CRC ^ binary 1000 1100 */
   return(shiftReg);
}

uint8_t calcCrc(char* buf,int len)
{
  char bitMask[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
  uint8_t c,crc;
  int i,j;

  crc=0;
  for(i=0;i<len;i++){//Serial.print(i);Serial.print(" ");
    for(j=0;j<8;j++){c=0;if((buf[i] & bitMask[j])!=0){c=1;}//Serial.print(c);Serial.print(" ");}
    crc=calcBitCrc(crc,c);}// Serial.println(crc);}
  }
  return crc;
}

byte setcrc(char* buf,int len)
{
  byte c=calcCrc(buf,len);
  conv_htoa(buf+len,&c);buf[len+2]='\0';
}


float convStrToNum(char* str,int* sizeRead)
{
  float v=0;
  uint8_t v0=0;
  float pd=1;
  int minu=1;
  int i=0;
#define MAXL 10

  for(i=0;i<MAXL;i++){
    if(i==0 && str[i]=='+'){i++;}
    if(i==0 && str[i]=='-'){i++;minu=-1;}
    if(str[i]=='.'){if(pd==1){pd=10;}i++;}
    *sizeRead=i+1;
    if(str[i]!='_' && str[i]!='\0' && str[i]>='0' && str[i]<='9'){
      v0=*(str+i)-48;
      if(pd==1){v=v*10+v0;}
      else{v+=v0/pd;pd*=10;}
    }
    else {i=MAXL;}
  }
  //Serial.print("s>n str,num=");Serial.print(string);Serial.print(" ");Serial.println(v*minus);
  return v*minu;
}

/*
float convStrToNum(char* str,int* sizeRead)
{
  float   pd=1,v=0;
  uint8_t i=0,v0=0,minu=1;

  while(str[i]==' '){i++;}

  if(str[i]=='+'){i++;}
  if(str[i]=='-'){i++;minu=-1;}

  while((str[i]!='_' && str[i]!='\0' && str[i]>='0' && str[i]<='9') || (str[i]=='.')){
    if(str[i]=='.'){if(pd==1){pd=10;}}
    else {v0=str[i]-48;
      if(pd==1){v=v*10+v0;}
      else{v+=v0/pd;pd*=10;}
    }
    i++;
  }
  return v*minu;
}
*/

boolean compMac(byte* mac1,byte* mac2)
{
  for(int i=0;i<6;i++){if(mac1[i] != mac2[i]){return FAUX;}}
  return VRAI;
}

void packMac(uint8_t* mac,char* ascMac)
{
  for(int i=0;i<6;i++){conv_atoh(ascMac+i*3,mac+i);
  //Serial.print(ascMac+i*3);Serial.print(" ");Serial.println((uint8_t)(mac+i),HEX);
  }
}

void unpackMac(char* buf,byte* mac) //uint8_t* mac)
{
  for(int i=0;i<6;i++){conv_htoa(buf+(i*3),mac+i);if(i<5){buf[i*3+2]='.';}}
    buf[17]='\0';
}

void serialPrintMac(byte* mac,uint8_t ln)
{
  char macBuff[18];
  unpackMac(macBuff,mac);
  Serial.print(macBuff);
  if(ln!=0){Serial.println();}
}


void packDate(char* dateout,char* datein)
{
    for(int i=0;i<6;i++){
        dateout[i]=datein[i*2] << 4 | (datein[i*2+1] & 0x0F);
    }
}

void unpackDate(char* dateout,char* datein)
{
    for(int i=0;i<6;i++){
        dateout[i*2]=(datein[i] >> 4)+48; dateout[i*2+1]=(datein[i] & 0x0F)+48;
    }
}

uint8_t dcb2b(byte val)
{
    return ((val>>4)&0x0f)*10+(val&0x0f);
}

uint32_t cvds(char* d14,uint8_t skip)   // conversion date packée (yyyymmddhhmmss 7 car) en sec
{
    uint32_t secDay=24*3600L;
    uint32_t secYear=365*secDay;
    uint32_t m28=secDay*28L,m30=m28+secDay+secDay,m31=m30+secDay;
    uint32_t monthSec[]={0,m31,monthSec[1]+m28,monthSec[2]+m31,monthSec[3]+m30,monthSec[4]+m31,monthSec[5]+m30,monthSec[6]+m31,monthSec[7]+m31,monthSec[8]+m30,monthSec[9]+m31,monthSec[10]+m30,monthSec[11]+m31};

    uint32_t aa=0;if(skip==0){aa=dcb2b(d14[0])*100L;};aa+=dcb2b(d14[1-skip]);
    uint32_t njb=aa/4L;       // nbre années bisextiles depuis année 0
    uint8_t  mm=dcb2b(d14[2-skip]);
                        if(mm>2 && aa%4==0){njb++;}
    uint32_t bisext=njb*secDay;

    return bisext+aa*secYear+monthSec[(mm-1)]+(dcb2b(d14[3-skip])-1)*secDay
            +dcb2b(d14[4-skip])*3600L+dcb2b(d14[5-skip])*60L+dcb2b(d14[6-skip]);
}

int  dateCmp(char* olddate,char* newdate,uint32_t offset,uint8_t skip1,uint8_t skip2)
{

    uint32_t oldds=cvds(olddate,skip1),newds=cvds(newdate,skip2);

    if((oldds+offset)<newds){return -1;}
    if((oldds+offset)>newds){return 1;}
    return 0;
}

void serialPrintDate(char* datein)
{
    for(int i=0;i<6;i++){
        Serial.print((char)((datein[i] >> 4)+48));Serial.print ((char)((datein[i] & 0x0F)+48));}
        Serial.println();
}


void lb0()
{
    if(millis()>blinktime){
       if(digitalRead(PINLED)==LEDON){digitalWrite(PINLED,LEDOFF);
          if(cntBlink<=1){blinktime=millis()+SLOWBLINK;}
          else{blinktime=millis()+FASTBLINK;}
          if(cntBlink>0){cntBlink--;}
       }
       else {digitalWrite(PINLED,LEDON);blinktime=millis()+PULSEBLINK;}
       if(cntBlink==0){cntBlink=nbreBlink;}      // recharge cntblink
    }
}

void ledblink(uint8_t nbBlk)    // nbre blinks rapides tous les PERBLINK
{
         if(nbreBlink==0 ){
                if(nbBlk!=BCODEONBLINK){nbreBlink=nbBlk;}                   // une fois nbreBlink chargé, la consigne est ignorée
                else digitalWrite(PINLED,LEDON);
         }
         if(nbBlk==100+nbreBlink){nbreBlink=0;}
         while(nbreBlink%2!=0){lb0();}                        // si nbreBlink impair blocage
         lb0();                                               // sinon blink 1 ou nbreBlink
}

void timeOvfSet(uint8_t slot)
{
    if(slot<=TIMEOVFSLOTNB){timeOvfSlot[slot-1]=micros();}
}

void timeOvfCtl(uint8_t slot)
{
#ifdef FLAGTIMEOVF
    if(slot<=TIMEOVFSLOTNB){
    Serial.print("tOvf[");Serial.print(slot);
    Serial.print("]=");Serial.println((micros()-timeOvfSlot[slot-1]));
    if((micros()-timeOvfSlot[slot-1])>2000){Serial.print("<<<<<<<<<<<<");}
    }
#endif
}


/* debug

void debug(int cas){
  if(cntdebug[cas]==0 || cas==1){
    Serial.print("debug");Serial.println(cas);
  }
   cntdebug[cas]++;
}

void setdebug(int cas,int* v0,int* v1,char* v2, char* v3)
{
    cntdebug[cas]++;if(cntdebug[cas]<NBDBOC){
    timedebug[cas*NBDBOC+cntdebug[cas]]=micros();
    v0debug[cas*NBDBOC+cntdebug[cas]]=v0;
    v1debug[cas*NBDBOC+cntdebug[cas]]=v1;
    v2debug[cas*NBDBOC+cntdebug[cas]]=v2;
    v3debug[cas*NBDBOC+cntdebug[cas]]=v3;
  }
}

void showdebug()
{
  for(int deb=0;deb<NBDBPTS;deb++){
    Serial.print("point=");Serial.println(deb);
    for(int occ=0;occ<cntdebug[deb];occ++){
      Serial.print("     occ=");Serial.print(occ);
      Serial.print(" ");Serial.print(timedebug[deb*NBDBOC+occ]);
      Serial.print(" ");Serial.print((int)v0debug[deb*NBDBOC+occ]);
      Serial.print(" ");Serial.print((int)v1debug[deb*NBDBOC+occ]);
      Serial.print(" ");Serial.print((char*)v2debug[deb*NBDBOC+occ]);
      Serial.print(" ");Serial.println((char*)v3debug[deb*NBDBOC+occ]);
    }
  }
}

void initdebug()
{
  Serial.print("   *** init debug ***");
  for(int dg=0;dg<NBDBPTS;dg++){
    cntdebug[dg]=0;for(int dp=0;dp<NBDBOC;dp++){timedebug[dg*NBDBOC+dp]=0;}
  }
}
*/


bool ctlpass(char* data,char* model)
{
  return !memcmp(model,data,strlen(model));
}

bool ctlto(long time,uint16_t to)
{
    //Serial.print("ctlto=");Serial.print(time);Serial.print(" to=");Serial.println(to);
 return (millis()-time)>(to*1000);
}

void startto(long* time,uint16_t* to,uint16_t valto)
{
  *to=valto;
  *time=millis();
        //Serial.print("startto=");Serial.print(*time);Serial.print(" to=");Serial.print(*to);Serial.print(" valto=");Serial.println(valto);
}

int searchusr(char* usrname)
{
    int nbu=-1,k=0;
    bool ok=FAUX;
//    Serial.print(usrname);Serial.print(" usernames=");Serial.println(usrnames);

    for (nbu=NBUSR-1;nbu>=0;nbu--){
//        Serial.print("nbu=");Serial.println(nbu);
        for(k=0;k<LENUSRNAME;k++){
//                Serial.print(" ");Serial.print(k);Serial.print(" ");Serial.print(usrname[k]);Serial.print(" ");Serial.println(usrnames[k+nbu*(LENUSRNAME+1)]);
            if(usrname[k]=='\0' && k==0){break;}
            else if(usrname[k]=='\0'){ok=VRAI;}
            else if(usrname[k]!=usrnames[k+nbu*(LENUSRNAME+1)]){break;}
        }
        if(k==LENUSRNAME || ok==VRAI){return nbu;}
    }
    return -1;
}
