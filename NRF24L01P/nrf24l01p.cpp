#include <SPI.h>
#include "nrf24l01p.h"


/*************** hardware *********************
* numNRF est le pointeur permanent de circuit
* il commande CE et CSN donc contrôle parfaitement l'accès aux circuits
***********************************************/

#define CE_INIT   pinMode(cePin[numNrf],OUTPUT);
#define CSN_INIT  pinMode(csnPin[numNrf],OUTPUT);
#define CE_HIGH   digitalWrite(cePin[numNrf],HIGH);
#define CE_LOW    digitalWrite(cePin[numNrf],LOW);
#define CSN_HIGH  digitalWrite(csnPin[numNrf],HIGH);
#define CSN_LOW   digitalWrite(csnPin[numNrf],LOW);
#define SPI_INIT  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
#define SPI_START SPI.begin();

#define PP4       pinMode(4,OUTPUT);digitalWrite(4,LOW);digitalWrite(4,HIGH);pinMode(4,INPUT);
                  // debug pulse for logic analyzer
#define DEF_CHANNEL 1

#define RX 1
#define TX 0

#define CONFREG 0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) | PWR_UP_BIT | PRIM_RX_BIT

#define RFREG   RF_DR_HIGH_BIT & ~(RF_DR_LOW_BIT) | RF_PWR_BITS

/***** config *****/

uint8_t nbNrf=0;            // nbre circuits
uint8_t numNrf=0;           // n° du circuit courant
uint8_t numBalise=0;        // n° du circuit balise
char    mode=' ';           // 'P' ou 'C'
uint8_t cePin[MAX_NRF];    // pins pour CE
uint8_t csnPin[MAX_NRF];   // pins pour CSN
uint8_t channel[MAX_NRF];   // channels utilisé

byte*   br_addr;
byte*   cc_addr;
byte*   r0_addr;
byte*   pi_addr;

/***** scratch *****/

bool prxMode[MAX_NRF];       // true=circuit en PRX (pwrUpRx(), RX0 chargé, CE_HIGH)
bool rxP0Set[MAX_NRF];       // force rechargement RX_P0 si false
                            // après modification pour TX avec ACK
byte baseAddr[MAX_NRF*ADDR_LENGTH];       // RX_P0 pour chaque circuit

uint8_t regw,stat,fstat,conf;

uint8_t bidcnt;


Nrfp::Nrfp()    // constructeur
{
}

void Nrfp::setup(char exMode,uint8_t exCePin,uint8_t exCsnPin,
                 uint8_t exNbNrf,uint8_t exChannel,byte* exBr_addr,
                 byte* exCc_addr,byte* exR0_addr,byte* exPi_addr)
{
    mode=exMode;
    nbNrf=exNbNrf;if(nbNrf==0 || nbNrf>MAX_NRF){nbNrf=1;}
    cePin[0]=exCePin;for(uint8_t i=1;i<nbNrf;i++){cePin[i]=cePin[0]+i;}
    csnPin[0]=exCsnPin;for(uint8_t i=1;i<nbNrf;i++){csnPin[i]=csnPin[0]+i;}
    channel[0]=exChannel;
    if(channel[0]==0){channel[0]=DEF_CHANNEL;}
    for(uint8_t i=1;i<nbNrf;i++){channel[i]=channel[0]+i;}

    cc_addr=exCc_addr;
    br_addr=exBr_addr;
    r0_addr=exR0_addr;
    pi_addr=exPi_addr;

    /* hardware */

    SPI_INIT;
    SPI_START;
}

void Nrfp::start()   // numNrf dependant
{

    CSN_HIGH;
    CSN_INIT;
    CE_LOW;
    CE_INIT;

    /* registers */

    regw=EN_DYN_ACK_BIT | EN_DPL_BIT;  // no ack enable ; dyn pld length enable
    regWrite(FEATURE,&regw);
    regWrite(FEATURE,&regw);

    regw=(ENAA_P5_BIT|ENAA_P4_BIT|ENAA_P3_BIT|ENAA_P2_BIT|ENAA_P1_BIT|ENAA_P0_BIT);
    regWrite(EN_AA,&regw);

    regw=(ERX_P5_BIT|ERX_P4_BIT|ERX_P3_BIT|ERX_P2_BIT|ERX_P1_BIT|ERX_P0_BIT);
    regWrite(EN_RXADDR,&regw);

    regw=(DPL_P5_BIT|DPL_P4_BIT|DPL_P3_BIT|DPL_P2_BIT|DPL_P1_BIT|DPL_P0_BIT);
    regWrite(DYNPD,&regw);                  // dynamic payload length

    if(mode=='P'){
        addrWrite(RX_ADDR_P1,r0_addr);      // macAddr
    }

    if(mode=='C'){

        byte pAddr[NB_PIPE];
        memcpy(pAddr,r0_addr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]+=numNrf*NB_PIPE;
        addrWrite(RX_ADDR_P0,pAddr);         // RX0 du circuit
        memcpy(baseAddr+numNrf*ADDR_LENGTH,pAddr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]+=numNrf*NB_PIPE+1;
        addrWrite(RX_ADDR_P1,pAddr);         // RX1 du circuit

        for(uint8_t i=2;i<NB_PIPE;i++){      // fill pipes RX2-6
            regw=pAddr[ADDR_LENGTH-1]+i-1;
            regWrite((RX_ADDR_P0+i),&regw);
        }
    }

    prxMode[numNrf]=false;
    rxP0Set[numNrf]=false;    // pour forcer la restauration de RX0
                              // à l'entrée de available()
                              // brAddr si 'P' ; ccAddr si 'C' && balise
                              // baseAddr+numNrf*ADDR_LENGTH si 'C' "normal"

    regWrite(RF_CH,&channel[numNrf]);

    regw=RFREG;
    regWrite(RF_SETUP,&regw);

    powerUp();

    delay(5);

    flushRx();
    flushTx();

    regw=TX_DS_BIT | MAX_RT_BIT | RX_DR_BIT; // clear bits TX_DS , MAX_RT , RX_DR
    regWrite(STATUS,&regw);

}


bool Nrfp::begin()
{
    if(mode=='P'){return pRegister();}
    if(mode=='C'){letsPrx();return true;}
}


/* ********************************************************** */

bool Nrfp::setNum(uint8_t circuit,uint8_t balise)
{
    if(mode=='P'){numNrf=0;numBalise=99;}
    if(mode=='C'){
        if(circuit<nbNrf){
            numNrf=circuit;
            numBalise=balise;
            return true;
        }
    }
    return false;
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
    CE_LOW

    conf=CONFREG;                         // powerUP
    regWrite(CONFIG,&conf);

}

void Nrfp::powerDown()
{
    CE_LOW

    conf=CONFREG & ~(PWR_UP_BIT);          // powerDown
    regWrite(CONFIG,&conf);

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

/************** **************/

bool Nrfp::letsPrx()      // passage en mode PRX
{
    CE_LOW

    if(!rxP0Set[numNrf]){                             // rechargement RX_ADDR_P0 après TX
      regw=RX_DR_BIT;
      regWrite(STATUS,&regw);                         // clr RX_DR_BIT

      if(mode=='C'){
         if(numNrf==numBalise){
           addrWrite(RX_ADDR_P0,cc_addr);}              // addr pour inscriptions
         else{
           addrWrite(RX_ADDR_P0,baseAddr+numNrf*ADDR_LENGTH);}} // addr normale

      if(mode=='P'){
           addrWrite(RX_ADDR_P0,br_addr);} // addr pour broadcast

      rxP0Set[numNrf]=true;
    }

    setRx();
    CE_HIGH
    prxMode[numNrf]=true;
}

bool Nrfp::available()      // keep CE high when false
{
    if(!prxMode[numNrf]){letsPrx();}

/*    regRead(FIFO_STATUS,&fstat);
    if((fstat & RX_EMPTY_BIT)!=0){      // FIFO empty ?
        regRead(STATUS,&stat,1);
        if((stat & RX_DR_BIT)==0){      // RX empty ?
            return false;               // stay in prxMode while waiting
        }
    }
*/
    regRead(STATUS,&stat);
    if((stat & RX_DR_BIT)==0){          // RX empty ?
        regRead(FIFO_STATUS,&fstat);
        if((fstat & RX_EMPTY_BIT)!=0){  // FIFO empty ?
            return false;               // stay in prxMode while waiting
        }
        Serial.print('F');
    }

    prxMode[numNrf]=false;
    CE_LOW
    PP4
    return true;  // dataRead should be done now
}

bool Nrfp::read(byte* data,uint8_t* pipe,uint8_t* pldLength)
{       // available() should have be done
    uint8_t maxLength=*pldLength;

    regRead(STATUS,&stat);
    *pipe=(stat & RX_P_NO_BIT)>>RX_P_NO;   // get pipe nb

    if(*pipe<=5){
        CSN_LOW
        SPI.transfer(R_RX_PL_WID);
        *pldLength=SPI.transfer(0xff);     // get pldLength (dynamic length)
        CSN_HIGH
        Serial.print(*pipe);Serial.print('<');Serial.println(*pldLength);
        }

    if((*pipe<=5) && (*pldLength<=maxLength) && (*pldLength>0)){
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,*pldLength);      // get pld
        CSN_HIGH

        regw=RX_DR_BIT;
        regWrite(STATUS,&regw);             // clear RX_DR bit
        return true;
    }
    else{
        flushRx();                          // if error flush all
        regw=RX_DR_BIT;
        regWrite(STATUS,&regw);
        return false;                       // invalid pipe nb or length
    }
}

void Nrfp::write(byte* data,char na,uint8_t len,byte* tx_addr)
{
    prxMode[numNrf]=false;

    //fstat=TX_FULL_BIT;
    //while((fstat & TX_FULL_BIT)){regRead(FIFO_STATUS,&fstat,1);}   // wait for free FIFO
                                                            // manage TO in case of out of order chip
    addrWrite(TX_ADDR,tx_addr);
    if(na=='A'){
        addrWrite(RX_ADDR_P0,tx_addr);
        rxP0Set[numNrf]=false;              // RX_ADDR_P0 restore to do
    }                                       // (cc_addr if 'C' ; br_addr if 'P' - see available())

    stat=TX_DS_BIT | MAX_RT_BIT;
    regWrite(STATUS,&stat);                 // clear TX_DS & MAX_RT bits
                                            // static length used
    CSN_LOW
    if(na=='A'){SPI.transfer(W_TX_PAYLOAD);}        // with ACK
    else{       SPI.transfer(W_TX_PAYLOAD_NA);}     // without ACK
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH
Serial.print('>');Serial.println(len);

    setTx();
    CE_HIGH                       // transmit

// transmitting() should be checked now to wait for end of paquet transmission
// or ACK reception before turning CE low
}

int Nrfp::transmitting()         // busy -> 1 ; sent -> 0 -> Rx ; MAX_RT -> -1
{     // should be added : TO in case of out of order chip (trst=-2)
      // when sent or max retry, output in PRX mode with CE high
      int trst=1;

      regRead(STATUS,&stat);
/*      bidcnt++;if(bidcnt<8){
        Serial.println(stat,HEX);
        PP4
      }*/

      if((stat & TX_DS_BIT)){trst=0;}           // data sent
      if((stat & MAX_RT_BIT)){                  // max retry
        trst=-1;
        if(mode=='P'){memset(pi_addr,0x00,ADDR_LENGTH);} // de-validate inscription
      }
      if(trst<=0){                              // data sent or max retry
        CE_LOW
        stat = TX_DS_BIT | MAX_RT_BIT;          // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat);
        letsPrx();
        PP4
      }

      return trst;
}

bool Nrfp::pRegister()  // inscription péri pour obtenir une pipeAddr
{                       // true OK ; false MAXRT or TO receiving data

    uint8_t pipe,pldLength=ADDR_LENGTH;

    write(r0_addr,'N',ADDR_LENGTH,cc_addr);     // send macAddr to cc_ADDR
    bidcnt=0;
    int trst=1;
    while(trst==1){trst=transmitting();}
Serial.println("transmitted");

    if(trst<0){return false;}                   // MAX_RT error

    long time_beg = millis();
    long readTo;

    while(!available() && (readTo>=0)){
        readTo=TO_REGISTER-millis()+time_beg;}

    CE_LOW

    if(readTo>=0){                              // no TO data ready
        read(pi_addr,&pipe,&pldLength);     // get pipeAddr
        addrWrite(TX_ADDR,pi_addr);             // PTX address update
        return true;}

    return false;                               // else TO error
}
