
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

extern char* mailFromAddr;
extern char* mailPass;
extern char  pass[];
extern char* usrnames;
extern char* usrpass;
extern uint16_t periCur;

//SdFat32 sd;
//SdFat sd;
SdFs sd;
#define error(s) sd.errorHalt(&Serial, F(s))

uint32_t sdopenFail=0;
const uint8_t SD_CS_PIN = 4;
#define SPI_CLOCK SD_SCK_MHZ(30)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)

#ifdef UDPUSAGE

// udp DATA

//unsigned int localUDPPort = PORTUDP;         // local port to listen for UDP packets
char timeServer[] = "ntp-p1.obspm.fr\0";  //"ntp-p1.obspm.fr\0";      // 145.238.203.14  NTP.MIDWAY.OVH ntp.unice.fr ntp.uvsq.fr ntp-p1.obspm.fr
const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE];      // buffer to hold incoming and outgoing packets

extern EthernetUDP* udp[2];

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

//File32 fhisto;           // fichier histo sd card
//SdFile fhisto;           // fichier histo sd card
FsFile fhisto;
extern long   fhsize;           // remplissage fhisto

extern "C" {
 #include "utility/w5100.h"
}

extern EthernetClient cliext;

//#define MAXSV 10
//char sss[MAX_SOCK_NUM];     //buffer sockets status numérique
//char sssVal[MAXSV]={SnSR::UDP,SnSR::CLOSED,SnSR::LAST_ACK,SnSR::TIME_WAIT,SnSR::FIN_WAIT,SnSR::CLOSING,SnSR::CLOSE_WAIT,SnSR::LISTEN,SnSR::ESTABLISHED,0};
//                            // valeurs utiles pour sockets status

#define LSSSP 6
#define NOLF true
char sssa[MAX_SOCK_NUM+2];    // valeurs alpha pour sockets status ; buffer affichage
char prevsssa[MAX_SOCK_NUM+2];
char sssP[MAX_SOCK_NUM*LSSSP+2];


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

void mailInit()
{
   if(mailEnable){
    Serial.print("Init-Mail: ");
    trigwd();
    char ms[LMAILMESS];memset(ms,0x00,LMAILMESS);

    strcat(ms,mailFromAddr);strcat(ms,"==");strcat(ms,mailPass);
    Serial.println(ms);
    periReq(&cliext,*periMail1,"mail_init_",ms);
   }
}

void mail(const char* a, const char* mm)
{

  if(mailEnable){

      uint16_t saveperiCur=periCur;
      mailInit();                                               // à supprimer pour version peripherique2 >=2.8
      trigwd();
      char ms[LMAILMESS];memset(ms,0x00,LMAILMESS);

      uint32_t bufIp=Ethernet.localIP();
      charIp(ms,(char*)&bufIp,nullptr);
      strcat(ms,"/");
      concatn(ms,*perifPort);
      strcat(ms," ");
      strcat(ms,a);strcat(ms,"==");
      strcat(ms,mailToAddr1);strcat(ms,"==");
      if((strlen(mm)+2<=(LMAILMESS-strlen(ms)))&& *mm!='\0'){
        strcat(ms,mm);strcat(ms," ");}
      if(LDATEB+3<=(LMAILMESS-strlen(ms))){
        strcat(ms,alphaDate());strcat(ms," ");}
      strcat(ms,VERSION);

      sprintf(ms+strlen(ms)," p=%d",*periMail1);
      strcat(ms," ##");strcat(ms,mailFromAddr);strcat(ms,"==");strcat(ms,mailPass);strcat(ms,"##");   // init values

      dumpstr(ms,LMAILMESS);
     
      periReq(&cliext,*periMail1,"mail______",ms);  
      periCur=saveperiCur;if(periCur!=0){periLoad(periCur);}
  }
}

/* --------------------------- SD card ------------------------------- */

/*void sdRemove(const char* fname,File32* file32)
{
  if(file32->remove(fname)){
  mail("REMOVE ",fname);}
  else {Serial.print("remove ");Serial.print(fname);Serial.println(" fail");}
}
*/
//void sdRemove(const char* fname,SdFile* file)
void sdRemove(const char* fname,FsFile* file)
{
  if(file->remove(fname)){
  mail("REMOVE ",fname);}
  else {Serial.print("remove ");Serial.print(fname);Serial.println(" fail");}
}

//int sdOpen(const char* fname,File32* file32)
//int sdOpen(const char* fname,SdFile* file)
int sdOpen(const char* fname,FsFile* file)
{
  return sdOpen(fname,file," ");
}

//int sdOpen(const char* fname,File32* file32,const char* txt)
//int sdOpen(const char* fname,SdFile* file,const char* txt)
int sdOpen(const char* fname,FsFile* file,const char* txt)
{
  sdopenFail++;
  //Serial.print(">====>");Serial.print(sdopenFail);Serial.print(" sdOpen ");Serial.print(fname);
  if (!file->open(fname, O_RDWR | O_CREAT)) {
    //Serial.println(" ko");
    char mess[LMAILMESS];
    sprintf(mess,"%.6lu",sdopenFail);
    mess[6]=' ';mess[7]='\0';
    strcat(mess,fname);
    int ll=strlen(txt);
    int mm=strlen(mess);
    if(ll>=LMAILMESS-mm-2){
      ll=LMAILMESS-mm-2;}
    memcpy(mess+mm,txt,ll);
    mess[mm+ll-1]='\0';
    
    mail("SD OPEN FAIL",mess);
    fhisto.close();
    return SDKO;
  }
  //Serial.println(" ok");
  return SDOK;
}

void cidDmp() {
  cid_t cid;
  if (!sd.card()->readCID(&cid)) {error("readCID failed");
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
  Serial.print("SDFAT_FILE_TYPE ");Serial.print(SDFAT_FILE_TYPE);Serial.print(" ");
  if (!sd.begin(SD_CONFIG)) {
    mail("SD_INIT_ERROR_HALT","");
    Serial.println("SD_INIT_ERROR_HALT");
    while(1){trigwd();}
    sd.initErrorHalt(&Serial);
  }

  //Serial.print("\nSD TYPE");Serial.print((int)sd.fatType());

  uint32_t size = sd.card()->sectorCount();
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
  
  Serial.print(" Cluster size (bytes): ");Serial.println(sd.vol()->bytesPerCluster());

  cidDmp();
}

//------------------------------ format ------------------------------------------------

void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}

void errorHalt() {
  sd.printSdError(&Serial);
  SysCall::halt();
}

void sdExfatFormat()
{
  // Force exFAT formatting for all SD cards larger than 512MB.
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
//#ifndef SDCARD_SS_PIN
//const uint8_t SD_CS_PIN = SS;
//#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
//const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
//#endif  // SDCARD_SS_PIN

// Select fastest interface.
//#if HAS_SDIO_CLASS
// SD config for Teensy 3.6 SDIO.
//#define SD_CONFIG SdioConfig(FIFO_SDIO)
//#define SD_CONFIG SdioConfig(DMA_SDIO)
//#elif ENABLE_DEDICATED_SPI
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
//#else  // HAS_SDIO_CLASS
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
//#endif  // HAS_SDIO_CLASS

//#define error(s) (Serial.println(F(s)),errorHalt())

//SdExFat sd;
//------------------------------------------------------------------------------
//void setup() {
  //Serial.begin(9600);
  while (!Serial) {}
  Serial.println(F("Type any character to begin formatting"));

  while (!Serial.available()) {
    yield();trigwd();
  }
  clearSerialInput();
  Serial.println();
  Serial.println(F(
    "Your SD will be formated exFAT.\r\n"
    "All data on the SD will be lost.\r\n"
    "Type 'Y' to continue.\r\n"));

  while (!Serial.available()) {
    yield();trigwd();
  }
  if (Serial.read() != 'Y') {
    Serial.println(F("Exiting, 'Y' not typed."));
    return;
  }
  if (!sd.cardBegin(SD_CONFIG)) {
    error("cardBegin failed");
  }
 // if(!sd.format(&Serial))    // utiliser SdExFat sd; 
  {
    error("format failed");
  }
  if (!sd.volumeBegin()) {
    error("volumeBegin failed");
  }
  Serial.print(F("Bytes per cluster: "));
  Serial.println(sd.bytesPerCluster());
  Serial.println(F("Done"));
//}
}

/* ---------------------------------------------------------------- */

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
  char text[LT];memset(text,0,LT);
  int v,w;

        sprintf(text,"%.8lu",amj);text[8]=' ';              // 9
        sprintf(text+9,"%.6lu",hms);text[15]=' ';           // +7
        sprintf(text+16,"%.6lu",sdopenFail);text[22]=' ';   // +7
        text[23]='\0';                                      // +1
        if(strlen(val1)+strlen(text)+1<LT){strcat(text,val1);strcat(text," ");}
        if(strlen(val2)+strlen(text)+1<LT){strcat(text,val2);}     

        //Serial.print("\n histo store ");Serial.print(text);Serial.print(" | ");Serial.println(val3);
          
        if(sdOpen("fdhisto.txt",&fhisto)){  // si ko mail dans sdOpen
          fhisto.seekEnd(0);
          v=fhisto.write((const void*)text,strlen(text));w=fhisto.write((const void*)val3,strlen(val3));
          /*uint16_t kk;
          v=1;kk=0;while(text[kk]!='\0' && kk<LT){v=fhisto.write(text[kk]);kk++;}
          w=1;kk=0;while(val3[kk]!='\0' && kk<1000){w=fhisto.write(val3[kk]);kk++;}
          if(val3[kk]!='\0'){w=fhisto.write('\0');}
          */
          if(v==0 || w==0){mail("fdhisto_store_ko"," ");}   // ledblink(BCODEFHISTO);}
          fhisto.sync();
          fhsize=fhisto.size();
          fhisto.close();
        }
}

void histoStore_textdh(const char* val1,const char* val2,const char* val3)
{
  ds3231.getDate(&hms,&amj,&js,strdate);
  histoStore_textdh0(val1,val2,val3);
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
  Serial.print("udate1=");Serial.print(udate);Serial.print(" (");Serial.print(*yy);Serial.print("/");Serial.print(*mm);Serial.print("/");Serial.print(*dd);Serial.print(") ");
  udate-=bd*24*3600;
  Serial.print("udate2=");Serial.println(udate);
  byte js0;
  convertNTP(&udate,yy,mm,dd,&js0,hh,mi,ss);
  *js=js0;  
}

void dateToStr(char* buff,int year,int month,int day,int hour,int minute,int second)
{
  sprintf(buff,"%.8lu",(long)(year)*10000+(long)month*100+(long)day);
  sprintf((buff+8),"%.2u",hour);sprintf((buff+10),"%.2u",minute);sprintf((buff+12),"%.2u",second);
  buff[14]=' ';
  buff[15]='\0';
}  

unsigned long alphaDateToUnix(const char* tim,bool onlyHours,bool print)
{
  int year,month,day,hour,minute,seconde;
  uint32_t buf;
  
  year=1970;month=1;day=1;
  if(!onlyHours){
    conv_atobl(tim,&buf,4);year=buf;if(year==0){year=1970;}
    conv_atobl(tim+4,&buf,2);month=buf;if(month==0){month=1;}
    conv_atobl(tim+6,&buf,2);day=buf;if(day==0){day=1;}
  }
  conv_atobl(tim+8,&buf,2);hour=buf;
  conv_atobl(tim+10,&buf,2);minute=buf;  
  conv_atobl(tim+12,&buf,2);seconde=buf;

  if(print){
    Serial.print(" alphaDateToUnix ");
    Serial.print(year);Serial.print("/");Serial.print(month);Serial.print("/");Serial.print(day);Serial.print(" ");
    Serial.print(hour);Serial.print(":");Serial.print(minute);Serial.print(":");Serial.println(seconde);}

  return genUnixDate(&year,&month,&day,&hour,&minute,&seconde);
}

unsigned long alphaDateToUnix(const char* tim,bool onlyHours)
{
 return alphaDateToUnix(tim,onlyHours,false);
}

void unixDateToStr(unsigned long uxd,char* date)
{
  int year,month,day,hour,minute,seconde;
  convertNTP(&uxd,&year,&month,&day,&js,&hour,&minute,&seconde);
  dateToStr(date,year,month,day,hour,minute,seconde);
}

void addTime(char* recep,const char* tim1,const char* tim2,bool onlyHours)
{
  /*
  int year,month,day,hour,minute,seconde;
  uint32_t buf;
  
  conv_atobl(tim1,&buf,4);year=buf;
  conv_atobl(tim1+4,&buf,2);month=buf;
  conv_atobl(tim1+6,&buf,2);day=buf;
  conv_atobl(tim1+8,&buf,2);hour=buf;
  conv_atobl(tim1+10,&buf,2);minute=buf;  
  conv_atobl(tim1+12,&buf,2);seconde=buf;

  unsigned long unixtim1=genUnixDate(&year,&month,&day,&hour,&minute,&seconde);
*/
  unsigned long unixtim1=alphaDateToUnix(tim1,false);

/*  
  year=1970;month=1;day=1;
  if(!onlyHours){
    conv_atobl(tim2,&buf,4);year=buf;
    conv_atobl(tim2+4,&buf,2);month=buf;
    conv_atobl(tim2+6,&buf,2);day=buf;
  }
  conv_atobl(tim2+8,&buf,2);hour=buf;
  conv_atobl(tim2+10,&buf,2);minute=buf;  
  conv_atobl(tim2+12,&buf,2);seconde=buf;

  unsigned long unixtim2=genUnixDate(&year,&month,&day,&hour,&minute,&seconde);
//Serial.print(tim1);Serial.print(" ");Serial.println(tim2);
//Serial.print(unixtim1);Serial.print(" ");Serial.println(unixtim2);
*/
  unsigned long unixtim2=alphaDateToUnix(tim2,onlyHours);

  unsigned long unixsum=unixtim1+unixtim2;

  unixDateToStr(unixsum,recep);
}

void subTime(char* recep,const char* endtime,const char* time,bool onlyHours)
{
  int year,month,day,hour,minute,seconde;
  uint32_t buf;

  conv_atobl(endtime,&buf,4);year=buf;
  conv_atobl(endtime+4,&buf,2);month=buf;
  conv_atobl(endtime+6,&buf,2);day=buf;
  conv_atobl(endtime+8,&buf,2);hour=buf;
  conv_atobl(endtime+10,&buf,2);minute=buf;  
  conv_atobl(endtime+12,&buf,2);seconde=buf;

  unsigned long unixtim1=genUnixDate(&year,&month,&day,&hour,&minute,&seconde);

  year=1970;month=1;day=1;
  if(!onlyHours){
  conv_atobl(time,&buf,4);year=buf;
  conv_atobl(time+4,&buf,2);month=buf;
  conv_atobl(time+6,&buf,2);day=buf;
  }
  conv_atobl(time+8,&buf,2);hour=buf;
  conv_atobl(time+10,&buf,2);minute=buf;  
  conv_atobl(time+12,&buf,2);seconde=buf;

  unsigned long unixtim2=genUnixDate(&year,&month,&day,&hour,&minute,&seconde);

  unsigned long unixdate=unixtim1-unixtim2;

  convertNTP(&unixdate,&year,&month,&day,&js,&hour,&minute,&seconde);
  dateToStr(recep,year,month,day,hour,minute,seconde);
//Serial.print(unixtim1);Serial.print(" ");Serial.println(unixtim2);
//Serial.print(unixdate);Serial.print(" ");Serial.println(recep);
}

#ifndef UDPUSAGE
void initDate()
{
  ds3231.getDate(&hms2,&amj2,&js2,strdate);histoStore_textdh0("ST","RE"," ");
  Serial.print(" DS3231 time ");Serial.print(js2);Serial.print(" ");Serial.print(amj2);Serial.print(" ");Serial.println(hms2);

#endif // UDPUSAGE


#ifdef UDPUSAGE

void sendUdpData(uint8_t udpChannel,IPAddress host,uint16_t hostPort,char* data)
{
  udp[udpChannel]->beginPacket(host,hostPort);
  udp[udpChannel]->write(data,strlen(data));
  udp[udpChannel]->endPacket();
}

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
  udp[0]->beginPacket(address, 123); //NTP requests are to port 123
//IPAddress loc(82,64,32,56);
//Udp.beginPacket(loc,8888);

  udp[0]->write(packetBuffer, NTP_PACKET_SIZE);
  udp[0]->endPacket();
}

int getUDPdate(uint32_t* hms,uint32_t* amj,byte* js)
{
  int returnStatus=0;
  int year=0,month=0,day=0,hour=0,minute=0,second=0;

 // Udp.begin(localUDPPort);

  sendNTPpacket(timeServer); 
  
  delay(1000);                                // wait to see if a reply is available
  if (udp[0]->parsePacket()) {                    // packet received
    udp[0]->read(packetBuffer, NTP_PACKET_SIZE);  // get it                 // sec1900- 2208988800UL;
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
  if(!getUDPdate(&hms,&amj,&js)){Serial.println("pb NTP");ledblink(BCODEPBNTP,PULSEBLINK);} // pas de service date externe 
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

void printSocketStatus(bool nolf,const char* mess)
{
#ifdef SOCK_DEBUG
  if(*mess!=0){Serial.print(mess);}
  Serial.print(sssa);
  Serial.print(sssP);
  if(!nolf){Serial.println();}
#endif // SOCK_DEBUG
}

void showSocketsStatus(bool close,bool nolf,bool print,const char* mess)
{
/*
  la lib ethernet gère les 8 sockets du W5500 
  cli.available valorise la variable cli.sockindex avec le socket qui a une requête pendante (ou pas)
  cli.connect cherche un socket libre (CLOSED) via lequel lancer une requête
  cli.connected renvoie le socketstatus 
  server.begin associe un numéro de port à un socket qui passe en mode LISTEN (attenet de requête)
  Dans tous les cas sockindex>=MAX_SOCK_NUM indique que l'instance est déconnectée (pas de socket)

  showSocketStatus montre le socketstatus des 8 sockets et ferme éventuellement
  un socket ouvert et inactif depuis "trop" longtemps  
*/

  memset(sssa,'_',MAX_SOCK_NUM+1);sssa[MAX_SOCK_NUM+1]=0;
	for (uint8_t s=0; s < MAX_SOCK_NUM; s++) {
    sssa[s]=' ';
		switch(W5100.readSnSR(s)){
      case SnSR::CLOSED:  sssa[s]='C';break; 
      case SnSR::UDP:     sssa[s]='U';break; 
      case SnSR::LISTEN:  sssa[s]='L';break; 
      case SnSR::ESTABLISHED: sssa[s]='E';break;
      case SnSR::LAST_ACK:    sssa[s]='A';break;
      case SnSR::TIME_WAIT:   sssa[s]='W';break;
      case SnSR::FIN_WAIT:    sssa[s]='F';break;
		  case SnSR::CLOSING:     sssa[s]='c';break;
      case SnSR::CLOSE_WAIT:  sssa[s]='w';break;
      case SnSR::INIT:        sssa[s]='I';break;      
      case SnSR::SYNRECV:     sssa[s]='R';break;
      case SnSR::SYNSENT:     sssa[s]='S';break;      

      default:break;

/*
constatations :

en tcp server
lors d'une demande de connexion un socket LISTEN passe en mode ESTABLISHED 

.available() a 2 fonctions :
1) détecter la présence de data dispo
2) passer un socket en mode LISTEN si ce port n'en a pas (EthernetServer::begin())
.stop() fait Ethernet.socketDisconnect() sur le port pour que le nombre de sockets utilisés reste constant.

problème : une connexion établie sans data ne sera jamais fermée 
(le retour de .available() toujours faux n'aboutira jamais à un .stop())
solution provisoire(?) : fermeture des sockets E après 3(?) secondes d'inactivité.

il y a un socket dédié au fonctionnement de l'udp (si Sock_CLOSE plantage)

en tcp de browser, il reste un status ESTABLISHED en fin de commonserver
il y aurait 2 demandes de connexion de la part du browser (pour favicon !?) 
ce qui génère 2 sockets E dont un sans data qui, de ce fait, reste ouvert

Il serait utile d'avoir un socket réservé pour l'appel aux serveurs externes (periReq()). Comment ?
*/      
    }
	}

  uint8_t prt;
  uint16_t prt0;
  for(uint8_t i=0;i<MAX_SOCK_NUM;i++){
    prt0=EthernetServer::server_port[i];prt=prt0-prt0/10*10;
    sssP[i*LSSSP]=i+'0';sssP[i*LSSSP+1]='_';sssP[i*LSSSP+2]=prt+'0';sssP[i*LSSSP+3]='=';sssP[i*LSSSP+4]=sssa[i];sssP[i*LSSSP+5]=' ';
  }
  sssP[MAX_SOCK_NUM*LSSSP]=' ';
  sssP[MAX_SOCK_NUM*LSSSP+1]=0;

  if(print){printSocketStatus(true,mess);}

  if(close && print){
    for(uint8_t s=0;s<MAX_SOCK_NUM;s++){
      if(sssa[s]=='E'){
        uint8_t b;while(Ethernet.socketRecv(s, &b, 1) > 0){Serial.print(b);}
         // fermeture des sockets fantomes... apparemment toujours sur serveur remotes
         // remotehtml() génèrerait une connexion supplémentaire ?
        Serial.print("sock close ");Serial.print(s);Serial.print(' ');         
        prt0=EthernetServer::server_port[s];prt=prt0-prt0/10*10;Serial.print(prt0);Serial.print(' ');
        W5100.execCmdSn(s, Sock_CLOSE);
      }
    }
  }
#ifdef SOCK_DEBUG
  if(print && !nolf){Serial.println();}
#endif // SOCK_DEBUG
}

void showSocketsStatus(bool close,bool nolf,bool print)
{
  showSocketsStatus(close,nolf,print,"");
}

void showSocketsStatus(bool close,bool nolf)
{
  showSocketsStatus(close,nolf,true);
}

void showSocketsStatus(bool close)
{
  showSocketsStatus(close,false,true);
}