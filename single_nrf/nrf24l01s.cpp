#include <SPI.h>
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"

/* AP NFR24L01+ node single */


/*************** hardware ****************************
*
* setup et confcircuit doivent �tre effectu�s
* dans l'ordre à l'initialisation
*
******************************************************/

#define CE_INIT   pinMode(CE_PIN,OUTPUT);
#define CE_OFF    pinMode(CE_PIN,INPUT);
#define CSN_INIT  pinMode(CSN_PIN,OUTPUT);
#define CSN_OFF   pinMode(CSN_PIN,INPUT);
/*
#ifdef MEGA
#define CSN_HIGH  bitSet(PORTB,4);
#define CSN_LOW   bitClear(PORTB,4)
#endif  // MEGA
*/
#ifdef UNO        // idem for PRO MINI
#define CSN_HIGH  bitSet(PORTB,2);
#define CSN_LOW   bitClear(PORTB,2);
#define CE_HIGH   bitSet(PORTB,1);delayMicroseconds(10);
#define CE_LOW    bitClear(PORTB,1);
#endif  // UNO

/*#ifdef DUE
#define CSN_HIGH  bitSet(PORTC,29);
#define CSN_LOW   bitClear(PORTC,29);
#define CE_HIGH   bitSet(PORTC,21);delayMicroseconds(10);
#define CE_LOW    bitClear(PORTC,21);
#endif  // DUE */

#ifndef CSN_HIGH
#define CSN_HIGH  digitalWrite(CSN_PIN,HIGH);
#define CSN_LOW   digitalWrite(CSN_PIN,LOW);
#endif  // CSN_HIGH
#ifndef CE_HIGH
#define CE_HIGH   digitalWrite(CE_PIN,HIGH);delayMicroseconds(8);
#define CE_LOW    digitalWrite(CE_PIN,LOW);
#endif  // CE_HIGH

#define CLKPIN    13
#define MISOPIN   12
#define MOSIPIN   11
#define SPI_INIT  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
#define SPI_START SPI.begin();
#define SPI_OFF   SPI.end();pinMode(MOSIPIN,INPUT);pinMode(CLKPIN,INPUT);

//#define GET_STA   CSN_LOW statu=SPI.transfer(NOP);CSN_HIGH
#define GET_STA   digitalWrite(CSN_PIN,LOW);statu=SPI.transfer(NOP);CSN_HIGH //digitalWrite(CSN_PIN,HIGH);

#define CLR_TXDS_MAXRT  regw=TX_DS_BIT|MAX_RT_BIT;regWrite(STATUS,&regw);
#define CLR_RXDR        regw=RX_DR_BIT;regWrite(STATUS,&regw);


#define CONFREG (0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) | PWR_UP_BIT | PRIM_RX_BIT)

#define RFREG   RF_PWR_BITS

/***** config *****/

#if NRF_MODE == 'C'
struct NrfConTable tableC[NBPERIF];
#endif NRF_MODE == 'C'

/***** scratch *****/

bool prxMode=false;       // true=circuit en PRX (pwrUpRx(), CE_HIGH)

bool powerD=true;         // etat power (true=down)

uint8_t regw,statu,fstatu,conf;


Nrfp::Nrfp()    // constructeur
{
}

void Nrfp::setup()
{ 
    /* registers */

    regw=EN_DYN_ACK_BIT | EN_DPL_BIT;  // ack enable ; dyn pld length enable
    regWrite(FEATURE,&regw);

//    regw=(ADDR_LENGTH-2)<<AW;       // addresses width
//    regWrite(SETUP_AW,&regw);

//    regw=9;//MAX_PAYLOAD_LENGTH;        // set payload length
//    regWrite(RX_PW_P0,&regw);

//    regw=9;//MAX_PAYLOAD_LENGTH;        // set payload length
//    regWrite(RX_PW_P1,&regw);

//    regw=(ERX_P1_BIT|ERX_P0_BIT);   // R0,R1 seuls (ERX_P5_BIT|ERX_P4_BIT|ERX_P3_BIT|ERX_P2_BIT|ERX_P1_BIT|ERX_P0_BIT);
//    regWrite(EN_RXADDR,&regw);
//    regw=(ENAA_P1_BIT|ENAA_P0_BIT); // ACK //(ENAA_P5_BIT|ENAA_P4_BIT|ENAA_P3_BIT|ENAA_P2_BIT|ENAA_P1_BIT|ENAA_P0_BIT);
//    regWrite(EN_AA,&regw);          // ENAA nécessaire pour DPL
    regw=(DPL_P1_BIT|DPL_P0_BIT);   // (DPL_P5_BIT|DPL_P4_BIT|DPL_P3_BIT|DPL_P2_BIT|DPL_P1_BIT|DPL_P0_BIT);
    regWrite(DYNPD,&regw);          // dynamic payload length

#if NRF_MODE == 'P'
    addrWrite(RX_ADDR_P0,CC_ADDR);   // RXP0 pour réception ACK
    addrWrite(TX_ADDR,CC_ADDR);      // TX sur concentrateur
#endif // NRF_MODE == 'P'

    addrWrite(RX_ADDR_P1,(byte*)MAC_ADDR);  // RXP1 = macAddr du circuit

    regw=CHANNEL;
    regWrite(RF_CH,&regw);

    regw=RFREG | RF_SPEED;
    regWrite(RF_SETUP,&regw);

    regw=ARD_VALUE<<ARD+ARC_VALUE<<ARC;
    regWrite(SETUP_RETR,&regw);

    flushRx();
    flushTx();

    regw=TX_DS_BIT | MAX_RT_BIT | RX_DR_BIT; // clear bits TX_DS , MAX_RT , RX_DR
    regWrite(STATUS,&regw);

    prxMode=false;
}

/************ utilitary ***************/

void Nrfp::regRead(uint8_t reg,byte* data)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    *data=SPI.transfer(*data);
    CSN_HIGH
}

void Nrfp::regWrite(uint8_t reg,byte* data)
{
    CSN_LOW
    SPI.transfer((W_REGISTER | (REGISTER_MASK & reg)));
    SPI.transfer(*data);
    CSN_HIGH
}

void Nrfp::addrRead(uint8_t reg,byte* data)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    SPI.transfer(data,ADDR_LENGTH);
    CSN_HIGH
}

void Nrfp::addrWrite(uint8_t reg,byte* data)
{
    CSN_LOW
    SPI.transfer((W_REGISTER | (REGISTER_MASK & reg)));
    for(uint8_t i=0;i<ADDR_LENGTH;i++){SPI.transfer(data[i]);}
    CSN_HIGH
}

void Nrfp::powerUp()
{
    CSN_HIGH;
    CSN_INIT;
    CE_LOW;
    CE_INIT;
    PP4_INIT;
    PP4
    PP4

    SPI_INIT;
    SPI_START;

    conf=CONFREG;                         // powerUP/CRC 1 byte/PRX
    regWrite(CONFIG,&conf);
    regWrite(CONFIG,&conf);

    flushRx();
    CLR_RXDR

    powerD=false;

    delay(5);
}

void Nrfp::powerDown()
{
    if(!powerD){            // si power down, �a bloque de refaire...

        powerD=true;

        CE_LOW

        conf=CONFREG & ~PWR_UP_BIT;          // powerDown
        regWrite(CONFIG,&conf);

        CSN_OFF
        //CE_OFF    CE stay ON else go high (need a pulldown resistor?)

        SPI_OFF
    }
    PP4 PP4 PP4
}

void Nrfp::setTx()
{
    conf=CONFREG & ~(PRIM_RX_BIT);
    regWrite(CONFIG,&conf);
}

void Nrfp::setRx()
{
    conf=CONFREG | PRIM_RX_BIT;
    regWrite(CONFIG,&conf);}


void Nrfp::flushTx()
{
    CSN_LOW
    SPI.transfer(FLUSH_TX);
    CSN_HIGH
}

void Nrfp::flushRx()
{
    CSN_LOW
    SPI.transfer(FLUSH_RX);
    CSN_HIGH
}

bool Nrfp::letsPrx()      // goto PRX mode
{
    if(!prxMode){
      CE_LOW
      //flushRx();
      //CLR_RXDR
      setRx();
      prxMode=true;
    }
    CE_HIGH
}

/********** public *************/

void Nrfp::write(byte* data,bool ack,uint8_t len,uint8_t numP)  // write data,len to numP
{
    uint8_t llen=len; // MAX_PAYLOAD_LENGTH;

    prxMode=false;
    CE_LOW

#if NRF_MODE == 'C'
    addrWrite(TX_ADDR,tableC[numP].periMac);    // PER_ADDR);       
    addrWrite(RX_ADDR_P0,tableC[numP].periMac); // PER_ADDR);
#endif // NRF_MODE == 'C'

    setTx();
    flushTx();   // avant tx_pld !!!    
    CLR_TXDS_MAXRT

    CSN_LOW
    if(ack){SPI.transfer(W_TX_PAYLOAD);}          // with ACK
    else   {SPI.transfer(W_TX_PAYLOAD_NA);}       // without ACK
    for(uint8_t i=0;i<llen;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    CE_HIGH                                 // transmit (CE high->TX_DS 235uS @1MbpS)
    
// transmitting() should be checked now to wait for end of paquet transmission
// or ACK reception before turning CE low
}

int Nrfp::transmitting()         // busy -> 1 ; sent -> 0 -> Rx ; MAX_RT -> -1
{     // should be added : TO in case of out of order chip (trst=-2)
      // when sent or max retry, output in PRX mode with CE high

      int trst=1;
      
      GET_STA

      if((statu & (TX_DS_BIT | MAX_RT_BIT))!=0){

        trst=0;
        if(statu & MAX_RT_BIT){trst=-1;}

        CLR_TXDS_MAXRT
        letsPrx();
        PP4
      }
      
      return trst;
}

void Nrfp::rxError()
{
        CE_LOW
        flushRx();                         // if error flush all
        CLR_RXDR
        prxMode=false;
}

int Nrfp::available(uint8_t* pipe,uint8_t* pldLength)
{
/* available/read output errors codes (>=0 numP) */

    uint8_t maxLength=*pldLength; // MAX_PAYLOAD_LENGTH; //
    int err=0;

    if(!prxMode){letsPrx();}

    GET_STA
    if((statu & RX_DR_BIT)==0){             // RX empty ?
        regRead(FIFO_STATUS,&fstatu);
        if((fstatu & RX_EMPTY_BIT)!=0){     // FIFO empty ?
            return AV_EMPTY;               // --------------- empty
        }
        else{

          GET_STA

          *pipe=(statu & RX_P_NO_BIT)>>RX_P_NO;   // get pipe nb
          if(*pipe!=1){
            flushRx();
            //CLR_RXDR 
            return AV_EMPTY;}
        }
    }
    PP4    

    *pipe=(statu & RX_P_NO_BIT)>>RX_P_NO;   // get pipe nb
   
    if(*pipe==1){
        CSN_LOW
        SPI.transfer(R_RX_PL_WID);
        *pldLength=SPI.transfer(0xff);      // get pldLength (dynamic length)
        CSN_HIGH      
    }
    else {err=AV_NBPIP;}                    // ---------------- pipe nb error

    if(((*pldLength>maxLength) || (*pldLength<=0)) && err==0){
        err=AV_LMERR;}                      // ---------------- pldLength error

    if(err!=0){rxError();}
    return err;                             // =0 not empty ; !=0 error invalid pipe nb or length
}

int Nrfp::read(char* data,uint8_t* pipe,uint8_t* pldLength,int numP)
{   // see available() return codes
    // numP<NBPERIF means "available() allready done with result ok && *pipe set")

    //*pldLength=MAX_PAYLOAD_LENGTH;

    if(numP>=NBPERIF){                                      // allow to only execute available()
        numP=available(pipe,pldLength);                     // if necessary
    } 
    if(numP>=0){
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,*pldLength);                      // get pld
        CSN_HIGH

        CLR_RXDR

#if NRF_MODE == 'C'
        numP=data[ADDR_LENGTH]-48;                                        // numP
        if(numP!=0 && memcmp(data,tableC[numP].periMac,ADDR_LENGTH)!=0){  // macAddr ok ?
            numP=AV_MCADD;rxError();}                                     // si numP==0 inscription à faire
#endif // NRF_MODE == 'C'
    }

    return numP;                  // available() return codes ; PRX mode still true if no error
}

#if NRF_MODE == 'P'
int Nrfp::pRegister(char* message,uint8_t* pldLength)  // peripheral registration to get pipeAddr
{                      // ER_MAXRT ; AV_errors codes ; >=0 numP ok

    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    memcpy(message,MAC_ADDR,ADDR_LENGTH);
    message[ADDR_LENGTH]='0';
    write(message,NO_ACK,ADDR_LENGTH+1,0);     // send macAddr + numP=0 to cc_ADDR ; no ACK
    int trst=1;
    while(trst==1){trst=transmitting();}

    if(trst<0){return ER_MAXRT;}            // MAX_RT error should not happen (no ACK mode)

    unsigned long time_beg = millis();
    long readTo=0;
    uint8_t pipe=99;
    *pldLength=MAX_PAYLOAD_LENGTH;
    int numP=AV_EMPTY;
    while(numP==AV_EMPTY && (readTo>=0)){   // waiting for concentrator answer
        readTo=TO_REGISTER-millis()+time_beg;
        numP=read(message,&pipe,pldLength,NBPERIF);}

/*#ifdef DIAG
if(readTo<0 && numP>=0){numP=-99;}
  Serial.print((char*)message);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" numP=");Serial.println(numP);
#endif // DIAG
*/
    if(numP>=0 && (readTo>=0)){            // no TO && pld ok
        numP=message[ADDR_LENGTH]-48;      // numP
        return numP;}                      // PRX mode still true

    rxError();                             // CE low
    if(numP>=0){return ER_RDYTO;}          // else TO error
    return numP;                           // or AV error 
}
#endif // NRF_MODE == 'P'

#if NRF_MODE =='C'

uint8_t Nrfp::cRegister(char* message)      // search free line or existing macAddr
{         // retour NBPERIF -> full ; NBPERIF+2 -> MAX_RT ; else numP

          uint8_t i,freeLine=0;
          bool exist=false;

          for(i=1;i<NBPERIF;i++){
            if(memcmp(tableC[i].periMac,message,ADDR_LENGTH)==0){exist=true;break;}      // already existing
            else if(freeLine==0 && tableC[i].periMac[0]=='0'){freeLine=i;}        // store free line nb
          }

          if(!exist && freeLine!=0){
            i=freeLine;                                                           // i = free line
            exist=true;
            memcpy(tableC[i].periMac,message,ADDR_LENGTH);                        // record macAddr
            tableC[i].periMac[ADDR_LENGTH]=i+48;                                  // add numT as 6th char
          }

          if(exist){
            message[ADDR_LENGTH]=i+48;}
/*            write((byte*)message,NO_ACK,ADDR_LENGTH+1,i);       // send numP to peri(macAddr) ; no ACK

            int trst=1;
            while(trst==1){trst=transmitting();}

            if(trst<0){memset(tableC[i].periMac,'0',ADDR_LENGTH);i=NBPERIF+2;}    // MAX_RT -> effacement table ;
          }                                           // numP du perif sera effac� pour non r�ponse au premier TX
*/
          return i;
}

void Nrfp::tableCPrint()
{
  for(int i=0;i<NBPERIF;i++){
    if(i<10){Serial.print(" ");}
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");
    printAddr((char*)tableC[i].periMac,' ');Serial.print(" (");
    Serial.print(tableC[i].periBufLength);Serial.print("/");
    Serial.print(tableC[i].periBufSent);Serial.print(")");
    Serial.print(tableC[i].periBuf);Serial.print(" ");
    Serial.print(tableC[i].serverBuf);Serial.print(" ");
    Serial.println();
  }
}

void Nrfp::tableCInit()
{
  memcpy(tableC[0].periMac,CC_ADDR,ADDR_LENGTH);
  for(int i=1;i<NBPERIF;i++){
    tableC[i].numPeri=0;
    memcpy(tableC[i].periMac,"00000",ADDR_LENGTH);
    memcpy(tableC[i].serverBuf,"00120_00040_0.25",16);
    memset(tableC[i].periBuf,'\0',MAX_PAYLOAD_LENGTH+1);
    tableC[i].periBufLength=0;
    tableC[i].periBufSent=false;
  }
}

int Nrfp::tableCLoad()
{
}

int Nrfp::tableCSave()
{
}
#endif NRF_MODE=='C'

void Nrfp::printAddr(char* addr,char n)
{
  for(int j=0;j<ADDR_LENGTH;j++){Serial.print((char)addr[j]);}
  if(n=='n'){Serial.println();}
}
