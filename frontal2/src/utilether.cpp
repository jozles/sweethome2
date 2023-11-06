
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SdFat.h>
#include "ds3231.h"
#include "const.h"
#include <shconst2.h>
#include <shutil2.h>
#include "periph.h"
#include "peritalk.h"
#include "utilether.h"
#include "utilhtml.h"

extern Ds3231 ds3231;

extern bool mailEnable;

extern char  pass[];
extern char* usrnames;
extern char* usrpass;

const uint8_t SD_CS_PIN = 4;
#define SPI_CLOCK SD_SCK_MHZ(8)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
extern SdFat32 sd32;
#define error(s) sd32.errorHalt(&Serial, F(s))

#ifdef UDPUSAGE

// udp DATA

//unsigned int localUDPPort = PORTUDP;         // local port to listen for UDP packets
char timeServer[] = "ntp-p1.obspm.fr\0";  //"ntp-p1.obspm.fr\0";      // 145.238.203.14  NTP.MIDWAY.OVH ntp.unice.fr ntp.uvsq.fr ntp-p1.obspm.fr
const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE];      // buffer to hold incoming and outgoing packets

EthernetUDP Udp;

#endif // UDPUSAGE

extern char months[36];
extern char days[21];

uint32_t        amj2,hms2;
byte            js2;
extern uint32_t amj,hms;
extern byte     js;
extern char     strdate[LDATEB];

extern char*    chexa;

extern uint16_t*  perifPort;
extern char*      mailToAddr1;
extern uint16_t*  periMail1;

extern File32 fhisto;           // fichier histo sd card
extern long   fhsize;           // remplissage fhisto

extern EthernetClient cliext;

int writeEth0(EthernetClient* cli,char* buf, uint16_t len)
{
  #define ETHTO 1000
  unsigned long beg=millis();
  bool sta=0;

  while(millis()-beg < ETHTO){
    if(cli->availableForWrite()){cli->write(buf,len);sta=1;break;}
  }
  return sta;
}

int ethWrite(EthernetClient* cli,char* buf,uint16_t* lb,long len)
{
  unsigned long ethbeg=millis();
  long i=0;
  long j=strlen(buf);
  if(len!=0){j=len;}
  bool sta=1;

  if(lb!=nullptr){*lb+=strlen(buf);}

  trigwd();
  while (j>=2048){
    sta=writeEth0(cli,buf+i,2048);
    if(!sta){j=-1;break;}
    j-=2048;i+=2048;}
  if(j>0){sta=writeEth0(cli,buf+i,j);}
  buf[0]='\0';
  if(millis()-ethbeg>100){Serial.print(" ethw dur=");Serial.print(millis()-ethbeg);if(sta==1){Serial.print(" ok ");}else{Serial.print(" ko ");}}
  return sta;
}

int ethWrite(EthernetClient* cli,char* buf)
{
  return ethWrite(cli,buf,nullptr,strlen(buf));
}

int ethWrite(EthernetClient* cli,char* buf,long len)
{
  return ethWrite(cli,buf,nullptr,len);
}

int ethWrite(EthernetClient* cli,char* buf,uint16_t* lb)
{
  return ethWrite(cli,buf,lb,strlen(buf));
}

void mailInit(char* login,char* pass)
{
  if(mailEnable){
    
    trigwd();
    char ms[LMAILMESS];memset(ms,0x00,LMAILMESS);

    strcat(ms,login);strcat(ms,"==");strcat(ms,pass);
    periReq(&cliext,*periMail1,"mail_init_",ms);
  }
}

void mail(const char* a, const char* mm)
{
  
  if(mailEnable){
    
      trigwd();
      char ms[LMAILMESS];memset(ms,0x00,LMAILMESS);

      uint32_t bufIp=Ethernet.localIP();
      charIp(ms,(char*)&bufIp,nullptr);
      strcat(ms,"/");
      concatn(ms,*perifPort);
      strcat(ms," ");
      strcat(ms,a);strcat(ms,"==");
      strcat(ms,mailToAddr1);strcat(ms,"==");
      if(strlen(mm)+2<=(LMAILMESS-strlen(ms))){
        strcat(ms,mm);strcat(ms," ");}
      if(LDATEB+3<=(LMAILMESS-strlen(ms))){
        strcat(ms,alphaDate());strcat(ms," ");}
      strcat(ms,VERSION);
      //#define LP 4
      //char per[LP];memset(per,'\0',LP);sprintf(per,"%d",*periMail1);
      //strcat(ms," p=");strcat(ms,per);
      sprintf(ms+strlen(ms)," p=%d",*periMail1);
      periReq(&cliext,*periMail1,"mail______",ms);  
  }
}

void sdRemove(const char* fname,File32* file32)
{
  file32->remove(fname);
  mail("REMOVE ",fname);
}

int sdOpen(const char* fname,File32* file32)
{
  return sdOpen(fname,file32," ");
}

int sdOpen(const char* fname,File32* file32,const char* txt)
{
  if (!file32->open(fname, O_RDWR | O_CREAT)) {
    char mess[LMAILMESS/2];mess[0]='\0';
    strcat(mess,fname);
    if(strlen(txt)>=LMAILMESS/2-strlen(mess)){
      int ll=strlen(mess);
      memcpy(mess,txt,LMAILMESS/2-ll-1);
      mess[LMAILMESS/2-ll-1]='\0';
    }
    else {strcat(mess,txt);}
    mail("SD OPEN FAIL",mess);
    Serial.print(fname);Serial.println(" inaccessible");return SDKO;
  }
  return SDOK;
}

char* alphaDate()
{
  ds3231.getDate(&hms,&amj,&js,strdate);
  memset(strdate,'\0',LDATEB);
  
  sprintf(strdate,"%.8lu",amj);       // 9
  strcat(strdate," ");
  sprintf(strdate+9,"%.6lu",hms);     // +7

  return strdate;
}


void histoStore_textdh0(const char* val1,const char* val2,const char* val3)
{
  #define LT 40
  char text[LT];
  int v,w;

        sprintf(text,"%.8lu",amj);text[8]=' ';            // 9
        sprintf(text+9,"%.6lu",hms);text[15]=' ';         // +7
        text[16]='\0';                                    // +1
        if(strlen(val1)+strlen(text)+1<LT){strcat(text,val1);strcat(text," ");}
        if(strlen(val2)+strlen(text)+1<LT){strcat(text,val2);}      
          
        //fhisto.open("fdhisto.txt", O_RDWR | O_CREAT);
        if(sdOpen("fdhisto.txt",&fhisto)==SDKO){
          mail("HISTO STORE FAIL",text);
        }
        fhisto.seekEnd(0);
        v=fhisto.write(text);w=fhisto.write(val3);
        if(v==0 || w==0){ledblink(BCODEFHISTO);}
        fhisto.sync();
        fhsize=fhisto.size();
        fhisto.close();
}

void histoStore_textdh(const char* val1,const char* val2,const char* val3)
{
  ds3231.getDate(&hms,&amj,&js,strdate);
  histoStore_textdh0(val1,val2,val3);
}


void cidDmp() {
  cid_t cid;
  if (!sd32.card()->readCID(&cid)) {error("readCID failed");
  }
  Serial.print(" Manufacturer ID: ");Serial.print(int(cid.mid),HEX);
  Serial.print(" OEM ID: ");Serial.print(cid.oid[0]);Serial.println(cid.oid[1]);
  Serial.print(" Product: ");
  for (uint8_t i = 0; i < 5; i++) {Serial.print(cid.pnm[i]);}
  Serial.print(" Version: ");Serial.print(int(cid.prv_n));Serial.print(".");Serial.println(int(cid.prv_m));
  Serial.print(" Serial number: ");Serial.println(cid.psn,HEX);
  Serial.print(" Manufacturing date: ");Serial.print(int(cid.mdt_month));Serial.print('/');
  Serial.println((2000 + cid.mdt_year_low + 10 * cid.mdt_year_high));
  Serial.println();
}

void sdInit()
{
  if (!sd32.begin(SD_CONFIG)) {
    mail("SD_INIT_ERROR_HALT","");
    Serial.println("SD_INIT_ERROR_HALT");
    while(1){trigwd();}
    sd32.initErrorHalt(&Serial);
  }

  Serial.print("\nSD FAT");Serial.print((int)sd32.fatType());

  uint32_t size = sd32.card()->sectorCount();
  if (size == 0) {
    Serial.print("\nCan't determine the card size.\n");
    Serial.print("Try another SD card or reduce the SPI bus speed.\n");
    Serial.print("Edit SPI_SPEED in this program to change it.\n");
    mail("SD_INIT_SIZE_HALTED","");
    while(1){trigwd();delay(1000);}
  }

  uint32_t sizeMB = 0.000512 * size + 0.5;
  Serial.print(" Card size: ");Serial.print(sizeMB);
  Serial.println(" MB (MB = 1,000,000 bytes)");
  //Serial.print("Volume is FAT");Serial.print((int)(sd32.vol()->fatType()));
  Serial.print(" Cluster size (bytes): ");Serial.println(sd32.vol()->bytesPerCluster());

  cidDmp();
}

/*
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
*/

void convertNTP(unsigned long *dateUnix,int *year,int *month,int *day,byte *js,int *hour,int *minute,int *second)
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

unsigned long genUnixDate(int* year,int* month, int* day, int* hour,int* minute,int* seconde)
{
    unsigned long secDay=24*3600L;
    unsigned long secYear=365*secDay;
    unsigned long m28=secDay*28L,m30=m28+secDay+secDay,m31=m30+secDay;
    unsigned long monthSec[]={0,m31,monthSec[1]+m28,monthSec[2]+m31,monthSec[3]+m30,monthSec[4]+m31,monthSec[5]+m30,monthSec[6]+m31,monthSec[7]+m31,monthSec[8]+m30,monthSec[9]+m31,monthSec[10]+m30,monthSec[11]+m31};

    unsigned long udate;
    
    int yy=*year;if(yy<1970){yy=1970;}
    yy-=1970;
    int nbybis=(yy+1)/4;
    int day29=0;
    if(((yy+2)%4)==0){
      if(*month>2 || (*month==2 && *day==29)){day29=1;}
    }
    udate=(unsigned long)yy*secYear+(unsigned long)monthSec[*month-1]+(unsigned long)(*day-1)*secDay+(unsigned long)*hour*3600L+(unsigned long)*minute*60L+(unsigned long)*seconde+(unsigned long)nbybis*secDay+(unsigned long)day29*secDay;

    return udate;
}

void calcDate(int bd,int* yy,int*mm,int* dd,int* js,int*hh,int* mi,int* ss)     // bd jours avant date*
{
  unsigned long udate=genUnixDate(yy,mm,dd,hh,mi,ss);
  Serial.print("udate1=");Serial.println(udate);Serial.print(" (");Serial.print(*yy);Serial.print("/");Serial.print(*mm);Serial.print("/");Serial.print(*dd);Serial.println(") ");
  udate-=bd*24*3600;
  Serial.print("udate2=");Serial.println(udate);
  byte js0;
  convertNTP(&udate,yy,mm,dd,&js0,hh,mi,ss);
  *js=js0;  
}

#ifndef UDPUSAGE
void initDate()
{
  ds3231.getDate(&hms2,&amj2,&js2,strdate);histoStore_textdh0("ST","RE"," ");
  Serial.print(" DS3231 time ");Serial.print(js2);Serial.print(" ");Serial.print(amj2);Serial.print(" ");Serial.println(hms2);

#endif // UDPUSAGE


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
//IPAddress loc(82,64,32,56);
//Udp.beginPacket(loc,8888);

  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


int getUDPdate(uint32_t* hms,uint32_t* amj,byte* js)
{
  int returnStatus=0;
  int year=0,month=0,day=0,hour=0,minute=0,second=0;

 // Udp.begin(localUDPPort);

  sendNTPpacket(timeServer); 
  
  delay(1000);                                // wait to see if a reply is available
  if (Udp.parsePacket()) {                    // packet received
    Udp.read(packetBuffer, NTP_PACKET_SIZE);  // get it                 // sec1900- 2208988800UL;
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    unsigned long secsSince1970 = secsSince1900 -2208988800UL;          //sec1970=1456790399UL;  // 29/02/2016

//Serial.print("packetBuffer 40-43 : ");Serial.print(packetBuffer[40]);Serial.print(" ");Serial.print(packetBuffer[41]);Serial.print(" ");Serial.print(packetBuffer[42]);Serial.print(" ");Serial.println(packetBuffer[43]);
//Serial.print(" sin 1900/1970 : ");Serial.print(secsSince1900,10);Serial.print("/");Serial.println(secsSince1970,10);
    convertNTP(&secsSince1970,&year,&month,&day,js,&hour,&minute,&second);
    *amj=year*10000L+month*100+day;*hms=hour*10000L+minute*100+second;
//Serial.print(*amj);Serial.print(" ");Serial.println(*hms);
    returnStatus=1;
  }
  //Udp.stop();
  return returnStatus;
}

void initDate()
{
  Serial.print("date ");
  if(!getUDPdate(&hms,&amj,&js)){Serial.println("pb NTP");ledblink(BCODEPBNTP);} // pas de service date externe 
  else {
    Serial.print(js);Serial.print(" ");Serial.print(amj);Serial.print(" ");Serial.print(hms);Serial.println(" GMT");
    ds3231.getDate(&hms2,&amj2,&js2,strdate);               // read DS3231
    if(amj!=amj2 || hms!=hms2 || js!=js2){
      Serial.print(js2);Serial.print(" ");Serial.print(amj2);Serial.print(" ");Serial.print(hms2);Serial.print(" setup DS3231 ");
      ds3231.setTime((byte)(hms%100),(byte)((hms%10000)/100),(byte)(hms/10000),js,(byte)(amj%100),(byte)((amj%10000)/100),(byte)((amj/10000)-2000)); // SET GMT TIME      
      ds3231.getDate(&hms2,&amj2,&js2,strdate);
      if(amj!=amj2 || hms!=hms2 || js!=js2){Serial.println("failed");}
      else {Serial.println("ok");}
    }
  }
}  


int searchusr(char* usrname)
{
    int nbu=-1,k,pt;
    bool ok=FAUX;
//    Serial.print(usrname);Serial.print(" usernames=");Serial.println(usrnames);

    for (nbu=NBUSR-1;nbu>=0;nbu--){
//        Serial.print("nbu=");Serial.println(nbu);
        for(k=0;k<LENUSRNAME;k++){
//                Serial.print(" ");Serial.print(k);Serial.print(" ");Serial.print(usrname[k]);Serial.print(" ");Serial.println(usrnames[k+nbu*(LENUSRNAME+1)]);
            pt=nbu*(LENUSRNAME+1)+k;
            if(usrnames[pt]=='\0' && k==0){break;}
            else if(usrnames[pt]=='\0'){ok=VRAI;}
            else if(usrnames[pt]!=usrname[k]){break;}
        }
        if(k==LENUSRNAME || ok==VRAI){return nbu;}
    }
    return -1;
}

bool ctlpass(char* data,char* model)
{
  return !memcmp(model,data,strlen(model));
}

#endif // UDPUSAGE
