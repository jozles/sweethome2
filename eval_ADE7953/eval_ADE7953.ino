#define VERSION "1.0 "

#include <SPI.h>
#include <Wire.h>

enum {FAUX, VRAI};

/*  I2C  */

//#define SLAVE 0x38 // ade I2C address
#define SLAVE 0x08 // SPI CS PIN 

/* ADE register */

#define LENRNOM 10

// 8 bits regs
#define SAGYC     0x0000
#define DISNOLOAD 0x0001
#define LCYCMODE  0x0004
#define PGA_V     0x0007
#define PGA_IA    0x0008
#define PGA_IB    0x0009
#define WR_PROT   0x0040
#define LAST_OP   0x00FD
#define ReservFE  0x00FE
#define LAST_RW8  0x00FF
#define ADE_VERS  0x0702
#define EX_REF    0x0800

#define NB8REG 12
uint16_t reg8ad[] ={(uint16_t)SAGYC,(uint16_t)DISNOLOAD,(uint16_t)LCYCMODE,(uint16_t)PGA_V,(uint16_t)PGA_IA,(uint16_t)PGA_IB,(uint16_t)WR_PROT,(uint16_t)LAST_OP,ReservFE,(uint16_t)LAST_RW8,(uint16_t)ADE_VERS,(uint16_t)EX_REF};
char reg8nam[NB8REG][LENRNOM]  ={"SAGYC    ","DISNOLOAD","LCYCMODE ","PGA_V    ","PGA_IA   ","PGA_IB   ","WR_PROT  ","LAST_OP  ","ReservFE ","LAST_RW8 ","ADE_VERS ","EX_REF   "};


// 16 bits regs
#define ZXTOUT    0x0100
#define LINECYC   0x0101
#define CONFIG    0x0102
#define CF1DE     0x0103
#define CF2DE     0x0104
#define CFMODE    0x0107
#define PHCALA    0x0108
#define PHCALB    0x0109
#define PFA       0x010A
#define PFB       0x010B
#define ANGLE_A   0x010C
#define ANGLE_B   0x010D
#define PERIOD    0x010E
#define ALT_OUTP  0x0110
#define Reserv120 0x0120
#define LAST_ADD  0x01FE
#define LAST_RW16 0x01FF

#define NB16REG 17
uint16_t reg16ad[]={(uint16_t)ZXTOUT,(uint16_t)LINECYC,(uint16_t)CONFIG,(uint16_t)CF1DE,(uint16_t)CF2DE,(uint16_t)CFMODE,(uint16_t)PHCALA,(uint16_t)PHCALB,(uint16_t)PFA,(uint16_t)PFB,(uint16_t)ANGLE_A,(uint16_t)ANGLE_B,(uint16_t)PERIOD,(uint16_t)ALT_OUTP,(uint16_t)Reserv120,(uint16_t)LAST_ADD,(uint16_t)LAST_RW16};
char     reg16nam[NB16REG][LENRNOM]={"ZXTOUT   ","LINECYC  ","CONFIG   ","CF1DE    ","CF2DE    ","CFMODE   ","PHCALA   ","PHCALB   ","PFA      ","PFB      ","ANGLE_A  ","ANGLE_B  ","PERIOD   ","ALT_OUTP ","Reserv120","LAST_ADD ","LAST_RW16"};

// 32 bits regs
#define SAGLVL    0x0300
#define ACCMODE   0x0301   
#define AP_NOLOAD 0x0303
#define VR_NOLOAD 0x0304
#define VA_NOLOAD 0x0305
#define AVA       0x0310
#define BVA       0x0311
#define AWATT     0x0312
#define BWATT     0x0313
#define AVAR      0x0314
#define BVAR      0x0315
#define IA        0x0316
#define IB        0x0317
#define V         0x0318
#define IRMSA     0x031A
#define IRMSB     0x031B
#define VRMS      0x031C
#define AENERGYA  0x031E
#define AENERGYB  0x031F
#define RENERGYA  0x0320
#define RENERGYB  0x0321
#define APENERGYA 0x0322
#define APENERGYB 0x0323
#define OVLVL     0x0324
#define OILVL     0x0325
#define VPEAK     0x0326
#define RSTVPEAK  0x0327
#define IAPEAK    0x0328
#define RSTIAPEAK 0x0329
#define IBPEAK    0x032A
#define RSTIBPEAK 0x032B
#define IRQENA    0x032C
#define IRQSTATA  0x032D
#define RIRQSTATA 0x032E
#define IRQENB    0x032F
#define IRQSTATB  0x0330
#define RIRQSTATB 0x0331
#define CRC       0x037F
#define AIGAIN    0x0380
#define AVGAIN    0x0381
#define AWGAIN    0x0382
#define AVARGAIN  0x0383
#define AVAGAIN   0x0384
#define Reserved  0x0385
#define AIRMSOS   0x0386
#define Reserved  0x0387
#define VRMSOS    0x0388
#define AWATTOS   0x0389
#define AVAROS    0x038A
#define AVAOS     0x038B
#define BIGAIN    0x038C
#define BVGAIN    0x038D
#define BWGAIN    0x038E
#define BVARGAIN  0x038F
#define BVAGAIN   0x0390
#define Reserved  0x0391
#define BIRMSOS   0x0392
#define Reserved  0x0393
#define Reserved  0x0394
#define BWATTOS   0x0395
#define BVAROS    0x0396
#define BVAOS     0x0397
#define LAST_RW32 0x03FF

#define NB32REG 63
uint16_t reg32ad[]={SAGLVL,ACCMODE,AP_NOLOAD,VR_NOLOAD,VA_NOLOAD,AVA,BVA,AWATT,BWATT,AVAR,BVAR,IA,IB,V,IRMSA,IRMSB,VRMS,AENERGYA,AENERGYB,RENERGYA,RENERGYB,APENERGYA,APENERGYB,OVLVL,OILVL,VPEAK,RSTVPEAK,IAPEAK,RSTIAPEAK,IBPEAK,RSTIBPEAK,IRQENA,IRQSTATA,RIRQSTATA,IRQENB,IRQSTATB,RIRQSTATB,CRC,AIGAIN,AVGAIN,AWGAIN,AVARGAIN,AVAGAIN,Reserved,AIRMSOS,Reserved,VRMSOS,AWATTOS,AVAROS,AVAOS,BIGAIN,BVGAIN,BWGAIN,BVARGAIN,BVAGAIN,Reserved,BIRMSOS,Reserved,Reserved,BWATTOS,BVAROS,BVAOS,LAST_RW32};
char     reg32nam[NB32REG][LENRNOM]={"SAGLVL   ","ACCMODE  ","AP_NOLOAD","VR_NOLOAD","VA_NOLOAD","AVA      ","BVA      ","AWATT    ","BWATT    ","AVAR     ","BVAR     ","IA       ","IB       ","V        ","IRMSA    ","IRMSB    ","VRMS     ","AENERGYA ","AENERGYB ","RENERGYA ","RENERGYB ","APENERGYA","APENERGYB","OVLVL    ","OILVL    ","VPEAK    ","RSTVPEAK ","IAPEAK   ","RSTIAPEAK","IBPEAK   ","RSTIBPEAK","IRQENA   ","IRQSTATA ","RIRQSTATA","IRQENB   ","IRQSTATB ","RIRQSTATB","CRC      ","AIGAIN   ","AVGAIN   ","AWGAIN   ","AVARGAIN ","AVAGAIN  ","Reserved ","AIRMSOS  ","Reserved ","VRMSOS   ","AWATTOS  ","AVAROS   ","AVAOS    ","BIGAIN   ","BVGAIN   ","BWGAIN   ","BVARGAIN ","BVAGAIN  ","Reserved ","BIRMSOS  ","Reserved ","Reserved ","BWATTOS  ","BVAROS   ","BVAOS    ","LAST_RW32"};

uint8_t slave = SLAVE;

/* SPI select pins */

#define SS2 10      // spi select courant
#define SS1 7       // spi select tension
uint8_t ss[2] = {SS1, SS2}; // ss[0] tension ss[1] courant


/* LED */

#define LED 13
#define ONLED HIGH
#define OFFLED LOW

long tmpBlink = millis();         // temps dernier blink
int  perBlink = 0;
uint8_t savled;
#define TBLINKON 40
#define TBLINKOFF 1960

#define ON HIGH
#define OFF LOW

int      i,j;
byte     data[5];  // 4 bytes maxi
uint16_t reg;
char     nam[LENRNOM];
bool     contVolt=0;
bool     contCurr=0;


/* prototypes */

uint8_t adeRead(uint8_t slave, uint8_t reg,byte* data, int len);
void adeWrite(uint8_t slave, uint8_t reg, byte* data, int len);
int getReg(int* len,char* nam);
void readade();

void hexPrint8(byte b);
void hexPrint16(uint16_t b);


void setup() {

  Serial.begin(115200);delay(1000);
  Serial.println("\n\nready");
  
  pinMode(LED, OUTPUT); digitalWrite(LED, OFFLED);

  /* SPI */

  pinMode(SLAVE, OUTPUT); digitalWrite(SLAVE,HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  /* Wire 
  Wire.begin();
*/
  //  init reg 0x0120 

int len;
reg=0x00FE;
data[0]=0xAD;
adeWrite(slave, reg, data,1); // unlock
//readRegN(&len,"LAST_OP  ");
readRegN(&len,"LAST_RW8 ");
Serial.println();


reg=0x00FE;
data[0]=0xAD;
adeWrite(slave, reg, data,1); // unlock
reg=0x0120;
data[0]=0x30;
data[1]=0x00;
adeWrite(slave, reg, data,2); // setup
readRegN(&len,"LAST_RW16");
Serial.println();

}

void loop() {

  


  /* LED */

  int len;

  if (millis() > (tmpBlink + perBlink)) {
    tmpBlink = millis();
    if (digitalRead(LED) == ONLED) {
      digitalWrite(LED, OFFLED);
      perBlink = TBLINKOFF;
    }
    else {
      digitalWrite(LED, ONLED);
      perBlink = TBLINKON;
      if(contVolt!=0){readRegN(&len,"V        ");if(contCurr==0){Serial.println();}}
      if(contCurr!=0){readRegN(&len,"IB       ");Serial.println();}
    } 
  }


  /* VOLTS/AMP/POW */

 


  /* commandes Serial */

  if (Serial.available()) {
    char a=Serial.read();Serial.print("\n");Serial.println((char)a);
    switch (a) {
      case 'A': readade();break;
      case 'B': readRegN(&len,"V        ");readRegN(&len,"VRMS     ");readRegN(&len,"IB       ");readRegN(&len,"IRMSB    ");Serial.println();
                readRegN(&len,"AENERGYB ");readRegN(&len,"RENERGYB ");readRegN(&len,"APENERGYB");Serial.println();break;
      case 'C': contVolt=!contVolt;
                break;
      case 'D': break;
      case 'E': contCurr=!contCurr;
                break;
      
      default: break;
    }
  }
}
/* --------------- fin loop ---------------------- */

void readRegN(int* len,char* nam)
{

  int reg=getReg(len,nam);
  if(reg>=0){

    char pnam[LENRNOM];
    for(int i=0;i<LENRNOM-1;i++){pnam[i]=nam[i];if(pnam[i]==' '){pnam[i]='\0';break;}}
    if(i>=LENRNOM-1){pnam[LENRNOM-1]='\0';}
    Serial.print((char*)pnam);
    Serial.print("=");

    char invdata[5];
    for(int k=0;k<4;k++){hexPrint8(data[k]);invdata[3-k]=data[k];}
    Serial.print("/");Serial.print((int32_t)*((int32_t*)invdata));
    Serial.print("  ");
  }
  else{Serial.print("nom invalide");}
}


void adeWrite(uint8_t slave, uint16_t reg, byte* data, int len)  // SPI
{
  digitalWrite(slave, LOW);

  SPI.transfer16(reg);   // registre
  SPI.transfer(0x00);    // write
  for(len;len>0;len--){SPI.transfer(data[len-1]);}

  digitalWrite(slave, HIGH);
}

void adeRead(uint8_t slave, uint16_t reg, byte* data, int len)
{
  digitalWrite(slave, LOW);

  SPI.transfer16(reg);   // registre
  SPI.transfer(0x80);    // read
  
  SPI.transfer(data,len);

  //for (int h=-4;h<4;h++){hexPrint8(*(data+h));Serial.println();}

  digitalWrite(slave, HIGH);
}

/*  I2C
 
void adeWrite(uint8_t slave, uint16_t reg, byte* data, int len)  // I2C
{
  Wire.beginTransmission(slave);
  Wire.write(reg / 256);
  Wire.write(reg % 256);
  for(len;len>0;len--){
    Wire.write(data[len-1]);
  }
  Wire.endTransmission();
}

uint8_t adeRead(uint8_t slave, uint16_t reg, byte* data, int len)
{
  Wire.beginTransmission(slave);
  Wire.write(reg / 256);
  Wire.write(reg % 256);  
  Wire.endTransmission();

  Wire.requestFrom(slave, len);
  uint8_t nb=0;
  while (Wire.available())     // slave may send less (or more) than requested
  {
    //Serial.print("(");
    nb++;
    len--;if(len<0){len=0;}
    data[len]=Wire.read();
    //Serial.print(len);Serial.print("=");Serial.print(data[len]);Serial.print(" ");
  }
  //Serial.print("#");hexPrint8(data[1]);hexPrint8(data[0]);Serial.print(" ");
  return nb;
}
*/

void hexPrint8(byte b)
{
  if((b&0xF0)==0){Serial.print("0");}
  Serial.print(b,HEX);
}

void hexPrint16(uint16_t b)
{
  char* pt;pt=(char*)&b;
  hexPrint8((uint8_t)*(pt+1));
  hexPrint8((uint8_t)*pt);
}



void readAdeR(uint8_t nb,uint8_t len,char* regxnam,uint16_t* regxadd)
{   
  char lb[9]={'8','\0',' ','1','6','\0','3','2','\0'};
  Serial.print((char*)(lb+(len/2)*3));Serial.println(" bits registers");
  for(i=0;i<nb;i++){
    for(j=0;j<LENRNOM;j++){                               // nom
      Serial.print((char)*(regxnam+(i*LENRNOM)+j));
    }
    Serial.print(" 0x");hexPrint16(regxadd[i]);        // adresse
    adeRead(slave,regxadd[i],data,len);
      for(int r=0;r<len;r++){                             // data
        Serial.print(" ");hexPrint8(data[r]);}
    Serial.println();
  }
  Serial.println();
}

void readade()
{ 
  readAdeR(NB8REG,1,&reg8nam[0][0],reg8ad);
  readAdeR(NB16REG,2,&reg16nam[0][0],reg16ad);  
  readAdeR(NB32REG,4,&reg32nam[0][0],reg32ad);    
}

int search(char* regnam,char* nam, int nb)
{
  char k;
  for(nb;nb>0;nb--){
    k=' ';
    for(j=0;j<LENRNOM-1;j++){
      if(regnam[(nb-1)*LENRNOM+j]!=nam[j]){k='k';break;}
    }
    if(k!='k'){return nb-1;}
  }
  return -1;
}

int getReg(int* len,char* nam)
{
  uint16_t reg;
  int v=-1;

  v=search(&reg8nam[0][0],nam,NB8REG);
//Serial.print(v);Serial.print(" ");Serial.println(&reg8nam[v][0]);
  if(v>=0){reg=reg8ad[v];*len=1;}
  else{
    v=search(&reg16nam[0][0],nam,NB16REG);
//Serial.print(v);Serial.print(" ");Serial.println(&reg16nam[v][0]);
    if(v>=0){reg=reg16ad[v]; *len=2;}
    else{
      v=search(&reg32nam[0][0],nam,NB32REG);
//Serial.print(v);Serial.print(" ");Serial.println(&reg32nam[v][0]);    
      if(v>=0){reg=reg32ad[v]; *len=4;}
    }
  }
  if(v>=0){
    hexPrint16(reg);Serial.print(" ");
    adeRead(slave,reg,data,*len);
  }
  return v;
}

