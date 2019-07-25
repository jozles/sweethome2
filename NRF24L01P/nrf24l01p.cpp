#include <SPI.h>
#include "nrf24l01p.h"
#include "nrf24l01p_const.h"

/* AP NFR24L01+ node multi*multi */


/*************** hardware ****************************
*
* numNRF est le pointeur permanent de circuit
* il commande CE et CSN, donc il contrôle parfaitement
* l'accès aux circuits
*
* setup, setNum et confcircuit doivent être effectués
* dans l'ordre pour chaque circuit à l'initialisation
*
******************************************************/

#define CE_INIT   pinMode(cePin,OUTPUT);
#define CE_OFF    pinMode(cePin,INPUT);
#define CSN_INIT  pinMode(csnPin,OUTPUT);
#define CSN_OFF   pinMode(csnPin,INPUT);
#define CE_HIGH   digitalWrite(cePin,HIGH);
#define CE_LOW    digitalWrite(cePin,LOW);
#define CSN_HIGH  digitalWrite(csnPin,HIGH);
#define CSN_LOW   digitalWrite(csnPin,LOW);

#ifdef MEGA
#define CSN_HIGH  bitSet(PORTB,4);delayMicroseconds(1);
#define CSN_LOW   bitClear(PORTB,4);delayMicroseconds(1);
#endif MEGA
#ifdef UNO        // idem for PRO MINI
#define CSN_HIGH  bitSet(PORTB,2);
#define CSN_LOW   bitClear(PORTB,2);
#endif UNO

#define CLKPIN    13
#define MISOPIN   12
#define MOSIPIN   11
#define SPI_INIT  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
#define SPI_START SPI.begin();
#define SPI_OFF   SPI.end();pinMode(MOSIPIN,INPUT);pinMode(CLKPIN,INPUT);

#define GET_STA   CSN_LOW stat=SPI.transfer(NOP);CSN_HIGH

/*** debug pulse for logic analyzer  ***/
#define PP        4
#ifdef UNO        // idem for PRO MINI
#define PP4       bitClear(PORTB,PP);bitSet(PORTB,PP);
#endif UNO
#ifndef UNO
#define PP4       digitalWrite(PP,LOW);digitalWrite(PP,HIGH);
#endif
#define PP4_INIT  pinMode(PP,OUTPUT);PP4;

#define DEF_CHANNEL 1

#define RX 1
#define TX 0

#define CONFREG (0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) | PWR_UP_BIT | PRIM_RX_BIT)

#define DEF_RF_SPEED RF_SPD_1MB
#define RFREG   RF_PWR_BITS

/***** config *****/


uint8_t nbNrf=0;            // nbre circuits
uint8_t numNrf=0;           // n° du circuit courant
uint8_t numBalise=0;        // n° du circuit balise

uint8_t ceP[NB_CIRCUIT];       // pins pour CE
uint8_t cePin;
uint8_t csnP[NB_CIRCUIT];      // pins pour CSN
uint8_t csnPin;
uint8_t channel[NB_CIRCUIT];   // channels utilisés
byte    rfSpeed;
byte    setupRetry;

byte*   br_addr;
byte*   cc_addr;
byte*   r0_addr;
byte*   pi_addr;

#if NRF_MODE == 'C'
struct NrfConTable tableC[NBPERIF];
#endif NRF_MODE == 'C'

/***** scratch *****/

bool prxMode[NB_CIRCUIT];       // true=circuit en PRX (pwrUpRx(), RX0 chargé, CE_HIGH)
bool rxP0Set[NB_CIRCUIT];       // force rechargement RX_P0 si false
                            // après modification pour TX avec ACK
byte baseAddr[NB_CIRCUIT*ADDR_LENGTH];       // RX_P0 pour chaque circuit

uint8_t regw,stat,fstat,conf;

bool powerD=true;


Nrfp::Nrfp()    // constructeur
{
}

#if NRF_MODE == 'P'
void Nrfp::setup(byte* exPi_addr)
{
    nbNrf=1;
    pi_addr=exPi_addr;
    numNrf=0;
    numBalise=99;
    cePin=CE_PIN; // ceP[numNrf];
    csnPin=CSN_PIN; //csnP[numNrf];
    channel[0]=CHANNEL;
#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'
void Nrfp::setup()
{
    nbNrf=NB_CIRCUIT;
    numNrf=CIRCUIT;
    ceP[0]=CE_PIN;for(uint8_t i=1;i<nbNrf;i++){ceP[i]=ceP[0]+i;}
    csnP[0]=CSN_PIN;for(uint8_t i=1;i<nbNrf;i++){csnP[i]=csnP[0]+i;}
    channel[0]=CHANNEL;for(uint8_t i=1;i<nbNrf;i++){channel[i]=channel[0];}

#endif // NRF_MODE == 'C'

    rfSpeed=RF_SPEED;
    setupRetry=(ARD-1)*16+ARC;

    cc_addr=CC_ADDR;
    br_addr=BR_ADDR;
    r0_addr=R0_ADDR;

}

bool Nrfp::confCircuit()   // numNrf dependant
{

    /* registers */

    regw=EN_DYN_ACK_BIT | EN_DPL_BIT;  // no ack enable ; dyn pld length enable
    regWrite(FEATURE,&regw);

    regw=(ENAA_P5_BIT|ENAA_P4_BIT|ENAA_P3_BIT|ENAA_P2_BIT|ENAA_P1_BIT|ENAA_P0_BIT);
    regWrite(EN_AA,&regw);

    regw=(ERX_P5_BIT|ERX_P4_BIT|ERX_P3_BIT|ERX_P2_BIT|ERX_P1_BIT|ERX_P0_BIT);
    regWrite(EN_RXADDR,&regw);

    regw=(DPL_P5_BIT|DPL_P4_BIT|DPL_P3_BIT|DPL_P2_BIT|DPL_P1_BIT|DPL_P0_BIT);
    regWrite(DYNPD,&regw);                  // dynamic payload length

#if NRF_MODE == 'P'
        addrWrite(RX_ADDR_P1,r0_addr);      // macAddr
#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'
        byte pAddr[NB_PIPE];
        memcpy(pAddr,r0_addr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]+=numNrf*NB_PIPE;
        addrWrite(RX_ADDR_P0,pAddr);         // RX0 du circuit
        memcpy(baseAddr+numNrf*ADDR_LENGTH,pAddr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]++;
        addrWrite(RX_ADDR_P1,pAddr);         // RX1 du circuit

        for(uint8_t i=2;i<NB_PIPE;i++){      // fill pipes RX2-6
            regw=pAddr[ADDR_LENGTH-1]+i-1;
            regWrite((RX_ADDR_P0+i),&regw);
        }
#endif // NRF_MODE == 'C'

    prxMode[numNrf]=false;
    rxP0Set[numNrf]=false;    // pour forcer la restauration de RX0
                              // à l'entrée de available()
                              // brAddr si 'P' ; ccAddr si 'C' && balise
                              // baseAddr+numNrf*ADDR_LENGTH si 'C' "normal"

    regWrite(RF_CH,&channel[numNrf]);

    regw=RFREG | rfSpeed;
    regWrite(RF_SETUP,&regw);

    regWrite(SETUP_RETR,&setupRetry);

    flushRx();
    flushTx();

    regw=TX_DS_BIT | MAX_RT_BIT | RX_DR_BIT; // clear bits TX_DS , MAX_RT , RX_DR
    regWrite(STATUS,&regw);

#if NRF_MODE == 'C'
    letsPrx();
#endif NRF_MODE
}

/* ********************************************************** */

#if NRF_MODE == 'C'
bool Nrfp::setNum(uint8_t circuit,uint8_t balise)
{
    if(circuit>=nbNrf){return false;}
    numNrf=circuit;
    numBalise=balise;

    cePin=ceP[numNrf];
    csnPin=csnP[numNrf];
    return true;
}
#endif // NRF_MODE == 'C'

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

    SPI_INIT;
    SPI_START;

    conf=CONFREG;                         // powerUP
    regWrite(CONFIG,&conf);
    regWrite(CONFIG,&conf);

    powerD=false;

    delay(4);
}

void Nrfp::powerDown()
{
    if(!powerD){            // si power down, ça bloque de refaire...

        powerD=true;

        CE_LOW

        conf=CONFREG & ~PWR_UP_BIT;          // powerDown
        regWrite(CONFIG,&conf);

        CSN_OFF
        //CE_OFF    CE stay ON else go high (need a pulldown resistor?)

        SPI_OFF
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
    CE_HIGH

    if(!rxP0Set[numNrf]){                            // rechargement RX_ADDR_P0 après TX
      flushRx();
      regw=RX_DR_BIT;
      regWrite(STATUS,&regw);                       // clr RX_DR_BIT

#if NRF_MODE == 'C'
      if(numNrf==numBalise){                        // R0 registration ?
        addrWrite(RX_ADDR_P0,cc_addr);}             // addr pour inscriptions
      else{
        addrWrite(RX_ADDR_P0,baseAddr+numNrf*ADDR_LENGTH);} // addr normale
#endif //NRF_MODE == 'C'
#if NRF_MODE == 'P'
      addrWrite(RX_ADDR_P0,br_addr);                // addr pour broadcast
#endif // NRF_MODE == 'P'
      rxP0Set[numNrf]=true;
    }

    setRx();
    prxMode[numNrf]=true;
}

/********** public *************/

void Nrfp::write(byte* data,char na,uint8_t len,uint8_t numP)
{
    prxMode[numNrf]=false;
    CE_LOW

    flushTx();   // avant tx_pld !!!

    CSN_LOW
    if(na=='A'){SPI.transfer(W_TX_PAYLOAD);}        // with ACK
    else{       SPI.transfer(W_TX_PAYLOAD_NA);}     // without ACK
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    setTx();

    stat=TX_DS_BIT | MAX_RT_BIT;  // clear TX_DS & MAX_RT bits
    regWrite(STATUS,&stat);

    byte wrAddr[5];
#if NRF_MODE == 'C'
    memcpy(wrAddr,tableC[numP].periMac,ADDR_LENGTH);
    if(numP==0){memcpy(wrAddr,BR_ADDR,ADDR_LENGTH);}
#endif // NRF_MODE == 'C'
#if NRF_MODE == 'P'
    memcpy(wrAddr,pi_addr,ADDR_LENGTH);
    if(numP==0){memcpy(wrAddr,CC_ADDR,ADDR_LENGTH);}
#endif // NRF_MODE == 'P'

    addrWrite(TX_ADDR,wrAddr);              // l'adresse TX doit être chargée avec CE low sinon délai supplémentaire
    if(na=='A'){
        addrWrite(RX_ADDR_P0,wrAddr);
        rxP0Set[numNrf]=false;              // RX_ADDR_P0 restore to do
    }                                       // (cc_addr if 'C' ; br_addr if 'P' - see available())

    CE_HIGH                                 // transmit (CE high->TX_DS 235uS @1MbpS)

// transmitting() should be checked now to wait for end of paquet transmission
// or ACK reception before turning CE low
}

int Nrfp::transmitting()         // busy -> 1 ; sent -> 0 -> Rx ; MAX_RT -> -1
{     // should be added : TO in case of out of order chip (trst=-2)
      // when sent or max retry, output in PRX mode with CE high

      CE_LOW

      int trst=1;

      GET_STA

      if((stat & TX_DS_BIT)){trst=0;}           // data sent
      else if((stat & MAX_RT_BIT)){             // max retry
        trst=-1;
#if NRF_MODE == 'P'
        memset(pi_addr,0x00,ADDR_LENGTH);       // de-validate inscription
#endif // NRF_MODE
//        if(mode=='P'){memset(pi_addr,0x00,ADDR_LENGTH);} // de-validate inscription
      }
      if(trst<=0){                              // data sent or max retry
        flushTx();
        stat = TX_DS_BIT | MAX_RT_BIT;          // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat);
        letsPrx();
        PP4
      }

      return trst;
}

void Nrfp::rxError()
{
        CE_LOW
        flushRx();                         // if error flush all
        regw=RX_DR_BIT;
        regWrite(STATUS,&regw);
        prxMode[numNrf]=false;
}

int Nrfp::available(uint8_t* pipe,uint8_t* pldLength)
{
/* available/read output errors codes (>=0 numP) */

    uint8_t maxLength=*pldLength;
    int err=0;

    GET_STA
    if((stat & RX_DR_BIT)==0){             // RX empty ?
        regRead(FIFO_STATUS,&fstat);
        if((fstat & RX_EMPTY_BIT)!=0){     // FIFO empty ?
            if(!prxMode[numNrf]){letsPrx();}
            return AV_EMPTY;               // empty
        }
        GET_STA
    }

    *pipe=(stat & RX_P_NO_BIT)>>RX_P_NO;   // get pipe nb

    if(*pipe<NB_PIPE){
        CSN_LOW
        SPI.transfer(R_RX_PL_WID);
        *pldLength=SPI.transfer(0xff);      // get pldLength (dynamic length)
        CSN_HIGH
    }
    else {err=AV_NBPIP;}

    if((*pldLength<=maxLength) && (*pldLength>0) && err==0){
        return numNrf*NB_PIPE+*pipe;        // ok ; numP ; PRX mode still true
    }
    else if(err==0){err=AV_LMERR;}

    rxError();
    return err;                              // invalid pipe nb or length
}

int Nrfp::read(char* data,uint8_t* pipe,uint8_t* pldLength,uint8_t numP)
{   // see available() return codes
    // numP<NBPERIF means "available() allready done with result ok && *pipe set")
    int avSta=numP;
    if(numP>=NBPERIF){                                      // allow to only execute available()
        avSta=available(pipe,pldLength);                    // if necessary
    }
    if(avSta>=0){
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,*pldLength);                      // get pld
        CSN_HIGH

        regw=RX_DR_BIT;
        regWrite(STATUS,&regw);                             // clear RX_DR bit
#if NRF_MODE == 'C'
        if(avSta!=0 && memcmp(data,tableC[avSta].periMac,ADDR_LENGTH)!=0){      // macAddr ok ?
            avSta=AV_MCADD;rxError();}
#endif // NRF_MODE == 'C'
    }

    return avSta;               // available() return codes ; PRX mode still true if no error
}

#if NRF_MODE == 'P'
int Nrfp::pRegister()  // peripheral registration to get pipeAddr
{                       // ER_MAXRT ; AV_errors codes ; >=0 numP ok

    uint8_t pipe,pldLength=ADDR_LENGTH;

    write(r0_addr,'N',ADDR_LENGTH,0);       // send macAddr to cc_ADDR ; no ACK

    int trst=1;
    while(trst==1){trst=transmitting();}

    if(trst<0){return ER_MAXRT;}            // MAX_RT error

    long time_beg = millis();
    long readTo=0;
    int  gdSta=AV_EMPTY;

    while(gdSta==AV_EMPTY && (readTo>=0)){  // waiting for data
        readTo=TO_REGISTER-millis()+time_beg;
        gdSta=read(pi_addr,&pipe,&pldLength,NBPERIF);}

//Serial.print("gdSta=");Serial.print(gdSta);Serial.print(" readTo=");Serial.print(readTo);Serial.print(" pi_addr=");printAddr(pi_addr,'n');

    if(gdSta>=0 && (readTo>=0)){            // no TO pld ok
        addrWrite(TX_ADDR,pi_addr);         // PTX address update
        return gdSta;}                      // PRX mode still true ; numP value or error code

    rxError();                              // CE low
    if(gdSta>=0){return ER_RDYTO;}          // else TO error
    return gdSta;                           // or AV error ;
}
#endif // NRF_MODE

void Nrfp::printAddr(char* addr,char n)
{
  for(int j=0;j<ADDR_LENGTH;j++){Serial.print((char)addr[j]);}
  if(n=='n'){Serial.println();}
}

#if NRF_MODE =='C'

uint8_t Nrfp::cRegister(char* macAddr)      // search free line or existing macAddr
{         // retour NBPERIF -> full ; NBPERIF+2 -> MAX_RT ; else numP

          uint8_t i,freeLine=0;
          bool exist=false;

          for(i=1;i<NBPERIF;i++){
            if(memcmp(tableC[i].periMac,macAddr,ADDR_LENGTH)==0){exist=true;break;}      // already existing
            else if(freeLine==0 && tableC[i].periMac[0]=='0'){freeLine=i;}        // store free line nb
          }

          if(!exist && freeLine!=0){
            i=freeLine;                                                           // i = free line
            exist=true;
            memcpy(tableC[i].periMac,macAddr,ADDR_LENGTH);}                       // record macAddr

          if(exist){
            write(tableC[i].pipeAddr,'A',ADDR_LENGTH,i);                          // send pipeAddr to peri(macAddr)
            int trst=1;
            while(trst==1){trst=transmitting();}
            if(trst<0){memset(tableC[i].periMac,'0',ADDR_LENGTH);i=NBPERIF+2;}    // MAX_RT -> effacement table ;
          }                                           // la pi_addr sera effacée pour non réponse au premier TX

          return i;
}




void Nrfp::tableCPrint()
{
  for(int i=0;i<NBPERIF;i++){
    Serial.print(i);Serial.print(" ");
    Serial.print(tableC[i].numNrf24);Serial.print(" ");
    Serial.print(tableC[i].numCirc);Serial.print(" ");
    Serial.print(tableC[i].numPipe);Serial.print(" ");
    Serial.print(tableC[i].numPeri);Serial.print(" ");

    printAddr(tableC[i].periMac,' ');Serial.print(" ");
    printAddr(tableC[i].pipeAddr,'n');
  }
}

void Nrfp::tableCInit()
{
  for(int i=0;i<NBPERIF;i++){
    tableC[i].numNrf24=i;
    tableC[i].numCirc=i/NB_PIPE;      // (0->n)
    tableC[i].numPipe=i%NB_PIPE;      // (0->5)
    tableC[i].numPeri=0;
    memcpy(tableC[i].pipeAddr,R0_ADDR,ADDR_LENGTH-1);
    tableC[i].pipeAddr[ADDR_LENGTH-1]='0'+(tableC[i].numCirc*NB_PIPE)+tableC[i].numPipe;
    memcpy(tableC[i].periMac,"00000",ADDR_LENGTH);
  }
}

int Nrfp::tableCLoad()
{
}

int Nrfp::tableCSave()
{
}

#endif NRF_MODE=='C'
