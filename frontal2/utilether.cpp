
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SD.h>
#include "const.h"
#include "periph.h"
#include "shconst.h"

#ifdef UDPUSAGE

// udp DATA

unsigned int localUDPPort = 8888;         // local port to listen for UDP packets
char timeServer[] = "ntp-p1.obspm.fr\0";  //"ntp-p1.obspm.fr\0";      // 145.238.203.14  NTP.MIDWAY.OVH ntp.unice.fr ntp.uvsq.fr ntp-p1.obspm.fr
const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE];      // buffer to hold incoming and outgoing packets

// UDP instance
EthernetUDP Udp;

#endif UDPUSAGE

extern char months[36];
extern char days[21];

extern uint32_t amj,hms,amj2,hms2;
extern byte js;
extern char strdate[33];

extern uint8_t* chexa;

int sdOpen(char mode,File* fileS,char* fname)
{
  fileS->close();
  if(!(*fileS=SD.open(fname,mode))){Serial.print(fname);Serial.println(" inaccessible");return SDKO;}

  return SDOK;
}

void sdstore_textdh0(File* fhisto,char* val1,char* val2,char* val3)
{
        char text[32]={'\0'};
        sprintf(text,"%.8lu",amj);strcat(text," ");       // 9
        sprintf(text+9,"%.6lu",hms);strcat(text," ");     // +7
        strcat(text,val1);strcat(text," ");               // +3
        strcat(text,val2);strcat(text,'\0');               // +3

          //fhisto->print(text);fhisto->print(val3);
          int v=fhisto->write(text);int w=fhisto->write(val3);
          if(v==0 || w==0){ledblink(BCODEFHISTO);}
}

void sdstore_textdh(File* fhisto,char* val1,char* val2,char* val3)
{
  long tm=millis();getdate(&hms,&amj,&js);
  sdstore_textdh0(fhisto,val1,val2,val3);
  //Serial.print("tdate=");Serial.println(millis()-tm);
}


int htmlSave(File* fhtml,char* fname,char* buff)                           // enregistre 1 buffer html
{
  char inch;
  fhtml->close();
  if(sdOpen(FILE_WRITE,fhtml,fname)==SDKO){Serial.print("html ");Serial.print(fname);Serial.println("KO");return SDKO;}

  fhtml->print(buff);Serial.print(inch);
  return SDOK;
}

int htmlPrint(EthernetClient* cli,File* fhtml,char* fname)                 // lis une page html sur cli
{
  char inch;
  int brcrlf=0;
  if(sdOpen(FILE_READ,fhtml,fname)==SDKO){cli->print("</title></head><body>");cli->print(fname);cli->println(" inaccessible<br>");return SDKO;}

  long htmlSiz=fhtml->size();
  long ptr=0;
  while (ptr<htmlSiz || inch==-1)
    {
    inch=fhtml->read();ptr++;
        cli->print(inch);
        Serial.print(inch);
        switch(brcrlf){
          case 0:if(inch=='<'){brcrlf=1;}break;
          case 1:if(inch=='b'){brcrlf=2;}else {brcrlf=0;}break;
          case 2:if(inch=='r'){brcrlf=3;}else {brcrlf=0;}break;
          case 3:if(inch=='>'){brcrlf=4;cli->println("");}else {brcrlf=0;}break;
          case 4:brcrlf=0;break;
        default:break;
        }
    }
   fhtml->close();
   return SDOK;
}

/*
void getdate_google(EthernetClient* cligoogle)
{
  Serial.print("cligoogle connected ?...");
  Serial.println("cligoogle connected");

  cligoogle->println("GET /search?q=test  HTTP/1.1");
  cligoogle->println("Host: www.google.com");
  cligoogle->println("Connection: close");
  cligoogle->println();

  boolean termine=FAUX,fini=FAUX,fini2=FAUX;
  int ptr=0,cnt=0;
  int posdate=-1;
  Serial.println("*getdate google*");
  while (!fini || !fini2)
    {
    if (cligoogle->available())
      {
      char c = cligoogle->read();Serial.print(c);
      //debug(c,ptr,posdate,strdate,cnt);
      if(posdate==strdate){strdate[ptr]=c;strdate[ptr+1]={0};ptr++;if(c<' ' || ptr>=32){fini=VRAI;}}
      if(posdate!=strdate && termine==VRAI){fini=VRAI;}
      if (!termine)
          {
          cnt++;if(cnt>500){termine=VRAI;Serial.println("***terminé");}
          strdate[ptr]=c;posdate=strstr(strdate,"Date: ");
          if(posdate!=strdate){ptr++;if(ptr>=5){for(ptr=0;ptr<5;ptr++){strdate[ptr]=strdate[ptr+1];}ptr=5;}}
          else {termine=VRAI;ptr=0;}
          }
      }
    else {fini2=VRAI;}
    }
    cligoogle->stop();Serial.println("\n---- fin getdate google");
}
*/

void convertNTP(long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second)
{
    int feb29;
    unsigned long secDay=24*3600L;
    unsigned long secYear=365*secDay;
    unsigned long m28=secDay*28L,m30=m28+secDay+secDay,m31=m30+secDay;
    unsigned long monthSec[]={0,m31,monthSec[1]+m28,monthSec[2]+m31,monthSec[3]+m30,monthSec[4]+m31,monthSec[5]+m30,monthSec[6]+m31,monthSec[7]+m31,monthSec[8]+m30,monthSec[9]+m31,monthSec[10]+m30,monthSec[11]+m31};
    unsigned long sec1quart=0;
    unsigned long secLastQuart=0;
    unsigned long secLastYear=0;
    unsigned long secLastMonth=0;
    unsigned long secLastDay=0;

    if(*dateUnix<2*secYear){*year=(int)(*dateUnix/secYear)+1970;feb29=1;}
    else
      {sec1quart = 4*secYear+secDay;
      secLastQuart = (*dateUnix-(2*secYear))%sec1quart;                           // sec dans quartile courant (premier 1972 bisextile)
//      Serial.print(secLastQuart);Serial.print(" ");
      *year = (int)((((*dateUnix-(2*secYear))/sec1quart)*4)+1972L);               // 1ère année quartile courant
//      Serial.print(*year);Serial.print(" ");
      if(secLastQuart<=(monthSec[2]+secDay)){feb29=0;monthSec[2]+=secDay;}        // date dans les 29 premiers jours de la 1ère année du dernier quartile
      else{*year += ((secLastQuart-secDay)/secYear);feb29=1;}                     // sinon ...
      }
    secLastYear=((secLastQuart-feb29*secDay)%secYear);
    for(int i=0;i<12;i++){
//      Serial.print("monthSec[i+1]=");Serial.print(i+1);Serial.print("/");Serial.println(monthSec[i+1]);
      if(monthSec[i+1]>secLastYear){*month=i+1;i=12;}
    }
    secLastMonth=secLastYear-monthSec[*month-1];
    *day  = (int)(secLastMonth/secDay)+1;
    secLastDay = secLastMonth-((*day-1)*secDay);
//    Serial.print("secLastDay=");Serial.print(secLastDay);Serial.print("/");Serial.print(*day);Serial.print(" secLastMonth=");Serial.print(secLastMonth);Serial.print("/");Serial.print(*month);Serial.print(" secLastYear=");Serial.print(secLastYear);Serial.print("/");Serial.println(*year);
    *hour = (int)(secLastDay/3600L);
    *minute = (int)(secLastDay-*hour*3600L)/60;
    *second = (int)(secLastDay-*hour*3600L-*minute*60);

    int k=0;if(*month>=3){k=2;}
    *js=( (int)((23*(*month))/9) + *day + 4 + *year + (int)((*year-1)/4) - (int)((*year-1)/100) + (int)((*year-1)/400) - k )%7;

//char buf[4]={0};strncat(buf,days+(*js)*3,3);
//Serial.print(*js);Serial.print(" ");Serial.print(buf);Serial.print(" ");
//Serial.print(*year);Serial.print("/");Serial.print(*month);Serial.print("/");Serial.print(*day);Serial.print(" ");
//Serial.print(*hour);Serial.print(":");Serial.print(*minute);Serial.print(":");Serial.println(*second);
}

long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde)
{
    unsigned long secDay=24*3600L;
    unsigned long secYear=365*secDay;
    unsigned long m28=secDay*28L,m30=m28+secDay+secDay,m31=m30+secDay;
    unsigned long monthSec[]={0,m31,monthSec[1]+m28,monthSec[2]+m31,monthSec[3]+m30,monthSec[4]+m31,monthSec[5]+m30,monthSec[6]+m31,monthSec[7]+m31,monthSec[8]+m30,monthSec[9]+m31,monthSec[10]+m30,monthSec[11]+m31};

    long udate;
    
    int yy=*year;if(yy<1970){yy=1970;}
    yy-=1970;
    int nbybis=(yy+1)/4;
    int day29=0;
    if(((yy+2)%4)==0){
      if(*month>2 || (*month==2 && *day==29)){day29=1;}
    }
    udate=(long)yy*secYear+(long)monthSec[*month-1]+(long)(*day-1)*secDay+(long)*hour*3600L+(long)*minute*60L+(long)*seconde+(long)nbybis*secDay+(long)day29*secDay;

    return udate;
}

void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss)     // bd jours avant date*
{
  long udate=genUnixDate(yy,mm,dd,hh,mi,ss);
  Serial.print("udate1=");Serial.println(udate);Serial.print(" (");Serial.print(*yy);Serial.print("/");Serial.print(*mm);Serial.print("/");Serial.print(*dd);Serial.println(") ");
  udate-=bd*24*3600;
  Serial.print("udate2=");Serial.println(udate);
  byte js0;
  convertNTP(&udate,yy,mm,dd,&js0,hh,mi,ss);
  *js=js0;  
}


#ifdef UDPUSAGE

void sendNTPpacket(char* address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


int getUDPdate(uint32_t* hms,uint32_t* amj,byte* js)
{
  int returnStatus=0;
  int year=0,month=0,day=0,hour=0,minute=0,second=0;

  Udp.begin(localUDPPort);

  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {                    // packet received
    Udp.read(packetBuffer, NTP_PACKET_SIZE);  // get it                 // sec1900- 2208988800UL;
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    long secsSince1900 = highWord << 16 | lowWord;
    long secsSince1970 = secsSince1900 -2208988800UL;          //sec1970=1456790399UL;  // 29/02/2016

//Serial.print("packetBuffer 40-43 : ");Serial.print(packetBuffer[40]);Serial.print(" ");Serial.print(packetBuffer[41]);Serial.print(" ");Serial.print(packetBuffer[42]);Serial.print(" ");Serial.println(packetBuffer[43]);
//Serial.print(" sin 1900/1970 : ");Serial.print(secsSince1900,10);Serial.print("/");Serial.println(secsSince1970,10);
    convertNTP(&secsSince1970,&year,&month,&day,js,&hour,&minute,&second);
    *amj=year*10000L+month*100+day;*hms=hour*10000L+minute*100+second;
//Serial.print(*amj);Serial.print(" ");Serial.println(*hms);
    returnStatus=1;
  }
  Udp.stop();
  return returnStatus;
}

#endif UDPUSAGE



