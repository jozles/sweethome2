#include "nrf_user_conc.h"
#include <SPI.h>
#include "nrf24l01s.h"

/* AP NFR24L01+ node single */


/*************** hardware ****************************
*
* setup et confcircuit doivent être effectués
* dans l'ordre à l'initialisation
*
******************************************************/

/*
#ifdef MEGA
#define CSN_HIGH  bitSet(PORTB,4);
#define CSN_LOW   bitClear(PORTB,4)
#endif  // MEGA
*/
#ifdef UNO        // idem for PRO MINI
#define CSN_HIGH  bitSet(PORT_CSN,BIT_CSN);
#define CSN_LOW   bitClear(PORT_CSN,BIT_CSN);
#define CE_HIGH   bitSet(PORT_CE,BIT_CE);delayMicroseconds(10);
// 10uS is minimal pulse ; 4uS to CSN low
#define CE_LOW    bitClear(PORT_CE,BIT_CE);
#define CSN_INIT  bitSet(DDR_CSN,BIT_CSN);
#define CSN_OFF   bitClear(DDR_CSN,BIT_CSN);
#define CE_INIT   bitSet(DDR_CE,BIT_CE);
#define CE_OFF    bitClear(DDR_CE,BIT_CE);
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
#define CE_HIGH   digitalWrite(CE_PIN,HIGH);delayMicroseconds(8);
#define CE_LOW    digitalWrite(CE_PIN,LOW);
#define CE_INIT   pinMode(CE_PIN,OUTPUT);
#define CE_OFF    pinMode(CE_PIN,INPUT);
#define CSN_INIT  pinMode(CSN_PIN,OUTPUT);
#define CSN_OFF   pinMode(CSN_PIN,INPUT);
#endif // CSN_HIGH

#ifdef SPI_MODE
#if NRF_MODE == 'P'
#define SPI_INIT    SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
#endif
#if NRF_MODE == 'C'
#define SPI_INIT    SPI.beginTransaction(SPISettings(8000000,MSBFIRST,SPI_MODE0));
#endif
#define SPI_START   SPI.begin();
#define SPI_OFF     SPI.end();pinMode(MOSI_PIN,INPUT);pinMode(CLK_PIN,INPUT);
#endif SPI_MODE

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

void Nrfp::powerOn()
{
#if NRF_MODE == 'P'  
  bitClear(PORT_CLK,BIT_CLK);     //digitalWrite(CLK_PIN,LOW);
  bitSet(DDR_CLK,BIT_CLK);        //pinMode(CLK_PIN,OUTPUT);
      
  bitClear(PORT_CSN,BIT_CSN);     //digitalWrite(CSN_PIN,LOW);
  bitSet(DDR_CSN,BIT_CSN);        //pinMode(CSN_PIN,OUTPUT);
  
  bitClear(PORT_CE,BIT_CE);       //digitalWrite(CE_PIN,LOW);
  bitSet(DDR_CE,BIT_CE);          //pinMode(CE_PIN,OUTPUT);
  
  bitClear(PORT_MOSI,BIT_MOSI);   //digitalWrite(MOSI_PIN,LOW);
  bitSet(DDR_MOSI,BIT_MOSI);      //pinMode(MOSI_PIN,OUTPUT);
  
  digitalWrite(RPOW_PIN,LOW);     // power on
  pinMode(RPOW_PIN,OUTPUT);

  bitSet(PORT_CSN,BIT_CSN);       //digitalWrite(CSN_PIN,HIGH);
  
  delay(POWONDLY);                // powerOn delay ******************** mettre en sleep *********************

#endif NRF_MODE == 'P'
#if NRF_MODE == 'C'   
  digitalWrite(CLK_PIN,LOW);
  pinMode(CLK_PIN,OUTPUT);

  digitalWrite(CSN_PIN,HIGH);
  pinMode(CSN_PIN,OUTPUT);

  digitalWrite(CE_PIN,LOW);
  pinMode(CE_PIN,OUTPUT);

  digitalWrite(MOSI_PIN,LOW);
  pinMode(MOSI_PIN,OUTPUT);
  digitalWrite(MOSI_PIN,HIGH);  
  
#endif NRF_MODE == 'C'

  powerUp();
  setup();                        // registry inits 
}

void Nrfp::powerOff()
{
    /* all radio/SPI pins low */

#if NRF_MODE == 'P'
  bitClear(PORT_CLK,BIT_CLK);     //digitalWrite(CLK_PIN,LOW);
  bitSet(DDR_CLK,BIT_CLK);        //pinMode(CLK_PIN,OUTPUT);
      
  bitClear(PORT_CSN,BIT_CSN);     //digitalWrite(CSN_PIN,LOW);
  bitSet(DDR_CSN,BIT_CSN);        //pinMode(CSN_PIN,OUTPUT);
  
  bitClear(PORT_CE,BIT_CE);       //digitalWrite(CE_PIN,LOW);
  bitSet(DDR_CE,BIT_CE);          //pinMode(CE_PIN,OUTPUT);
  
  bitClear(PORT_MOSI,BIT_MOSI);   //digitalWrite(MOSI_PIN,LOW);
  bitSet(DDR_MOSI,BIT_MOSI);      //pinMode(MOSI_PIN,OUTPUT);

  digitalWrite(RPOW_PIN,HIGH);    // power off
  pinMode(RPOW_PIN,OUTPUT);
#endif NRF_MODE='P'  
}

void Nrfp::powerUp()
{   
#ifdef SPI_MODE
    SPI_INIT;
    SPI_START;
#endif SPI_MODE

    conf=CONFREG;                         // powerUP/CRC 1 byte/PRX
    regWrite(CONFIG,&conf);

    flushRx();
    flushTx();
    CLR_RXDR

    powerD=false;

    delay(POWUPDLY);       // powerUp delay
}

void Nrfp::powerDown()
{
    if(!powerD){            // si power down, ça bloque de refaire...        
        powerD=true;
     
        CE_LOW

        conf=CONFREG & ~PWR_UP_BIT;          // powerDown
        regWrite(CONFIG,&conf);

#ifdef SPI_MODE
        SPI_OFF
#endif SPI_MODE              
    }
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
      CE_LOW              // to change from TX to RX
      //flushRx();
      //CLR_RXDR
      setRx();
      prxMode=true;
    }
    CE_HIGH
//    PP4_LOW
}

void Nrfp::rxError()
{
        CE_LOW
        flushRx();                         // if error flush all
        CLR_RXDR
        prxMode=false;
}

/********** public *************/

void Nrfp::write(byte* data,bool ack,uint8_t len,uint8_t numP)  // write data,len to numP if 'C' mode or to CCADDR if 'P' mode
{
    uint8_t llen=len; // MAX_PAYLOAD_LENGTH;

    prxMode=false;
    CE_LOW

#if NRF_MODE == 'C'
    addrWrite(TX_ADDR,tableC[numP].periMac);    // PER_ADDR);       
    addrWrite(RX_ADDR_P0,tableC[numP].periMac); // PER_ADDR);
#endif // NRF_MODE == 'C'

    setTx();
    flushTx();
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

int Nrfp::transmitting(bool ack)         // busy -> 1 ; sent -> 0 -> Rx ; MAX_RT -> -1
{     // should be added : TO in case of out of order chip (trst=-2)
      // when sent or max retry, output in PRX mode with CE high

      int trst=1;
      
      GET_STA

      if((statu & (TX_DS_BIT | MAX_RT_BIT))!=0){

        trst=0;
        if(statu & MAX_RT_BIT){
          if(!ack){Serial.print("\nsyst err maxrt without ack ");Serial.println(statu,HEX);delay(2);}
          trst=-1;} 

        CLR_TXDS_MAXRT
        letsPrx();
      }
      
      return trst;
}

int Nrfp::available(uint8_t* pipe,uint8_t* pldLength)
{
/* returns ( 0 full (valid length) ; <0 empty, pipe err or length error) 
   only receiving in pipe 1 ...
*/

    uint8_t maxLength=*pldLength; // MAX_PAYLOAD_LENGTH; //
    int err=0;

    if(!prxMode){letsPrx();}

    GET_STA
    lastSta=statu;
    if((statu & RX_DR_BIT)==0){                   // RX empty ?

        regRead(FIFO_STATUS,&fstatu);
        if((fstatu & RX_EMPTY_BIT)!=0){           // FIFO empty ?
            return AV_EMPTY;                      // --------------- empty
        }
        else{                                     // FiFO not empty

          GET_STA

          *pipe=(statu & RX_P_NO_BIT)>>RX_P_NO;   // get pipe nb
          if(*pipe!=1){                           // concentrator talking
            flushRx();                            
            //CLR_RXDR 
            return AV_EMPTY;}
        }

    }
    // RX full : either (statu & RX_DR_BIT)!=0
    //           either (fstatu & RX_EMPTY_BIT)==0) && ((statu & RX_P_NO_BIT)>>RX_P_NO)==1

    *pipe=(statu & RX_P_NO_BIT)>>RX_P_NO;         // get pipe nb ... if not 1 there is trouble
   
    if(*pipe==1){
        CSN_LOW
        SPI.transfer(R_RX_PL_WID);
        *pldLength=SPI.transfer(0xff);            // get pldLength (dynamic length)
        CSN_HIGH      
    }
    else {err=AV_EMPTY;} //AV_NBPIP;}                          // ---------------- pipe nb error

    if(((*pldLength>maxLength) || (*pldLength<=0)) && err==0){
        err=AV_LMERR;}                            // ---------------- pldLength error

    if(err!=0){rxError();}
    return err;                                   // =0 not empty ; <0 error : invalid pipe nb or length
}

int Nrfp::read(byte* data,uint8_t* pipe,uint8_t* pldLength,int numP)
{
   
    /* mode 'C' returns  0  registration to do 
                        >0  valid data table entry nb 
                        <0  empty, pipe err, length err, mac addr table error
       mode 'P' returns =0  full
                        <0  empty, pipe err or length error
    */
    // numP<NBPERIF means "available() allready done with result ok && *pipe set")
    // PRX mode still true if no error

    if(numP>=NBPERIF){
        numP=available(pipe,pldLength);
    }

    if(numP>=0){

        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,*pldLength);                      // get pld
        CSN_HIGH

#if NRF_MODE == 'C'
        numP=data[ADDR_LENGTH]-'0';                                       // sender numP
        if(numP!=0 && memcmp(data,tableC[numP].periMac,ADDR_LENGTH)!=0){  // macAddr ko ?
            numP=AV_MCADD;rxError();}                                     // if numP==0 registration to do
#endif NRF_MODE == 'C'
    }

    if(numP!=AV_EMPTY){CLR_RXDR PP4}    // ne pas faire CLR_RXDR si numP==empty : un message a pu arriver depuis la lecture du status
    return numP;    // CE stay HIGH in case of subsequent packets 
                    // (so read() caller has to put CE low if rx completed)
}

void Nrfp::readStop()
{
  CE_LOW
  prxMode=false;
}

#if NRF_MODE == 'P'
int Nrfp::pRegister(byte* message,uint8_t* pldLength)  // peripheral registration to get pipeAddr
{                      // ER_MAXRT ; AV_errors codes ; >=0 numP ok

    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    memcpy(message,MAC_ADDR,ADDR_LENGTH);
    message[ADDR_LENGTH]='0';
    
    write(message,NO_ACK,ADDR_LENGTH+1,0);     // send macAddr + numP=0 to cc_ADDR ; no ACK
 
    int trst=1;
    while(trst==1){trst=transmitting(NO_ACK);}
    if(trst<0){return ER_MAXRT;}            // MAX_RT error should not happen (no ACK mode)
                                            // radio card HS or missing

    unsigned long time_beg = millis();
    long readTo=0;
    uint8_t pipe=99;
    *pldLength=MAX_PAYLOAD_LENGTH;
    int numP=AV_EMPTY;    
    while(numP==AV_EMPTY && (readTo>=0)){   // waiting for concentrator answer
        readTo=TO_REGISTER-millis()+time_beg;
        numP=read(message,&pipe,pldLength,NBPERIF);}

    PP4_HIGH
    CE_LOW
    if(numP>=0 && (readTo>=0)){             // no TO && pld ok
        numP=message[ADDR_LENGTH]-'0';      // numP
        PP4
        return numP;}                       // PRX mode still true

    if(numP>=0){
//        CE_LOW
        return ER_RDYTO;}                   // else TO error
    
    CE_LOW
    return numP;                            // or AV error 
}

int Nrfp::txRx(byte* message,uint8_t* pldLength)
{                      // ER_MAXRT ; AV_errors codes ; >=0 numP ok

    //memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    message[MAX_PAYLOAD_LENGTH]=0x00;
    memcpy(message,MAC_ADDR,ADDR_LENGTH);
    
    write(message,NO_ACK,MAX_PAYLOAD_LENGTH,0);     // send macAddr + numP=0 to cc_ADDR ; no ACK
 
    int trst=1;
    while(trst==1){trst=transmitting(NO_ACK);}
    if(trst<0){return ER_MAXRT;}              // MAX_RT error should not happen (no ACK mode)
                                              // radio card HS or missing

    unsigned long time_beg = millis();
    long readTo=0;
    uint8_t pipe=99;
    *pldLength=MAX_PAYLOAD_LENGTH;
    int numP=AV_EMPTY;    
    while(numP==AV_EMPTY && (readTo>=0)){     // waiting for concentrator answer
        readTo=TO_REGISTER-millis()+time_beg;
        numP=read(message,&pipe,pldLength,NBPERIF);}

    PP4_HIGH
    CE_LOW
    if(numP>=0 && (readTo>=0)){               // no TO && pld ok
        numP=message[ADDR_LENGTH]-'0';        // numP
        PP4
        return numP;}                         // PRX mode still true

    if(numP>=0){
        return ER_RDYTO;}                     // else TO error
    
    return numP;                              // or AV error 
}
#endif // NRF_MODE == 'P'

#if NRF_MODE =='C'

uint8_t Nrfp::cRegister(char* message)      // search free line or existing macAddr
{         // retour NBPERIF -> full ; NBPERIF+2 -> MAX_RT ; else numP

          uint8_t i,freeLine=0;
          bool exist=false;

          for(i=1;i<NBPERIF;i++){
            if(memcmp(tableC[i].periMac,message,ADDR_LENGTH)==0){exist=true;break;}      // already exist
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

uint8_t Nrfp::macSearch(char* mac,int* numPer)    // search mac in tableC ; out 1->n ; n<NBPERIF found
{
  int i,j;

  for(i=1;i<NBPERIF;i++){
    for(j=ADDR_LENGTH-1;j>=0;j--){
      if(mac[j]!=tableC[i].periMac[j]){j=-2;}
    }
    if(j>-2){*numPer=tableC[i].numPeri;break;}
  }
  return i;
}

uint8_t Nrfp::extDataStore(uint8_t numPer,uint8_t numT,char* data,uint8_t len)
{
  if(numT>NBPERIF){return EDS_STAT_PER;}
  if(len>BUF_SERVER_LENGTH || len>MAX_PAYLOAD_LENGTH){return EDS_STAT_LEN;}

  tableC[numT].numPeri=numPer;
  if(len>MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1){len=MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1;}
  if(len!=0){
    tableC[numT].servBufLength=len;
    memcpy(tableC[numT].servBuf,data,len);}
  else{
    tableC[numT].servBufLength=SBLINIT;
    memcpy(tableC[numT].servBuf,SBVINIT,SBLINIT);}

  return EDS_STAT_OK;
}

void Nrfp::tableCPrint()
{
  for(int i=0;i<NBPERIF;i++){
    if(i<10){Serial.print(" ");}
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");
    printAddr((char*)tableC[i].periMac,' ');Serial.print(" (");
    Serial.print(tableC[i].periBufLength);Serial.print("/");
    Serial.print(tableC[i].periBufSent);Serial.print(") ");
    Serial.print(tableC[i].periBuf);Serial.print(" (");
    Serial.print(tableC[i].servBufLength);Serial.print(")");
    Serial.print(tableC[i].servBuf);Serial.print(" ");
    Serial.println();
  }
}

void Nrfp::tableCInit()
{
  memcpy(tableC[0].periMac,CC_ADDR,ADDR_LENGTH);
  for(int i=1;i<NBPERIF;i++){
    tableC[i].numPeri=0;
    memcpy(tableC[i].periMac,"00000",ADDR_LENGTH);
    tableC[i].servBufLength=SBLINIT;
    memcpy(tableC[i].servBuf,SBVINIT,SBLINIT);
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
