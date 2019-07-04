#include <SPI.h>
#include "nrf24l01p.h"


/*************** hardware *********************
* numNRF est le pointeur permanent de circuit
* il commande CE et CSN donc controle parfaitement l'accès aux circuits
*/
#define INIT_CE   pinMode(ce_pin[numNrf],OUTPUT);
#define INIT_CSN  pinMode(csn_pin[numNrf],OUTPUT);
#define CE_HIGH   digitalWrite(ce_pin[numNrf],HIGH);
#define CE_LOW    digitalWrite(ce_pin[numNrf],LOW);
#define CSN_HIGH  digitalWrite(csn_pin[numNrf],HIGH);
#define CSN_LOW   digitalWrite(csn_pin[numNrf],LOW);
#define INIT_SPI  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
#define START_SPI SPI.begin();


#define DEF_CHANNEL 1

#define RX 1
#define TX 0

#define CONFREG 0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) & ~(PWR_UP_BIT) | PRIM_RX_BIT

#define RFREG   RF_DR_HIGH_BIT & ~(RF_DR_LOW_BIT) | RF_PWR_BITS

uint8_t numNrf=0;           // n° du circuit courant
uint8_t numBalise=0;        // n° du circuit balise
char    mode=' ';           // mode 'P' ou 'C'

bool rxP0Set='false';       // rechargement RX_P0 nécessaire
                            // après TX

uint8_t regw,stat,fstat,conf;


Nrfp::Nrfp()    // constructeur
{
}

bool Nrfp::setNum(uint8_t circuit,uint8_t balise)
{
    if(mode='P'){numNrf=1;numBalise=99;}
    if(mode='C'){
        if(circuit<NB_NRF){
            numNrf=circuit;
            numBalise=balise;
            return true;
        }
    }
    return false;
}

void Nrfp::setMode(char exmode)
{
    mode=exmode;
}

void Nrfp::hardInit()
{
  CE_LOW
  INIT_CE
  CSN_HIGH
  INIT_CSN


  INIT_SPI
  START_SPI
}


/*
 * Principe :             (one to one)
 *
 *  le bit PXR_UP et CE_LOW place le chip en Standby en maxi 4,5mS
 *
 *  pour transmettre :
 *    charger le FIFO, bit PRIM_RX_low puis CE high (delay 130uS)
 *    attendre la fin de la transmission (TX_DS set ou MAX_RT set)
 *    retour en standby avec CE low
 *      (pwrUpRx() et PwrUpTx() font CE_LOW)
 *      (dataWrite() fait flushTx() et termine avec CE_HIGH)
 *      (transmitting() termine avec pwrUpRx() si false)
 *
 *  pour recevoir :
 *    bit PRIM_RX_high, CE_HIGH (delay 130uS)
 *    attendre un paquet (RX_DR set ou RX_EMPTY clr)
 *    retour en standby avec CE low
 *       (available() fait CE_HIGH
 *                    et termine avec CE_LOW si true)
 *
 */

bool Nrfp::config()           // power on config
{
  regWrite(RX_ADDR_P1,r1_addr,ADDR_LENGTH); // MAC si P ; base si C
  regWrite(RX_ADDR_P1,r1_addr,ADDR_LENGTH); // MAC si P ; base si C

  if(mode=='C'){
    regw=(ENAA_P5_BIT|ENAA_P4_BIT|ENAA_P3_BIT|ENAA_P2_BIT|ENAA_P1_BIT|ENAA_P0_BIT);
    regWrite((EN_AA),&regw,1);
    regw=(ERX_P5_BIT|ERX_P4_BIT|ERX_P3_BIT|ERX_P2_BIT|ERX_P1_BIT|ERX_P0_BIT);
    regWrite((EN_RXADDR),&regw,1);
    for(uint8_t i=1;i>=ADDR_LENGTH;i++){
      regw=r1_addr[ADDR_LENGTH-1]+i;
      regWrite((RX_ADDR_P1+i),&regw,1);
    }
    rxP0Set=false;
  }
  if(mode=='P'){regWrite(RX_ADDR_P2,br_addr,ADDR_LENGTH);} // broadcast

  if(channel==0){channel=DEF_CHANNEL;}
  regWrite(RF_CH,&channel,1);
  regw=MAX_PAYLOAD_LENGTH;
  regWrite(RX_PW_P0,&regw,1);
  regWrite(RX_PW_P1,&regw,1);
  regw=EN_DYN_ACK_BIT | EN_DPL_BIT;  // no ack enable ; dyn pld length enable
  regWrite(FEATURE,&regw,1);
  regw=RFREG;
  regWrite(RF_SETUP,&regw,1);

  pwrUpRx();

  delay(5);

  flushRx();
  flushTx();

  regw=TX_DS_BIT | MAX_RT_BIT | RX_DR_BIT; // clear bits TX_DS , MAX_RT , RX_DR
  regWrite(STATUS,&regw,1);

  if(mode=='P'){return inscript();}
}


/* ********************************************************** */

void Nrfp::regRead(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    for(uint8_t i=0;i<len;i++){data[i]=SPI.transfer(0xff);}
    CSN_HIGH
}

void Nrfp::regWrite(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((W_REGISTER | (REGISTER_MASK & reg)));
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH
}

void Nrfp::pwrUpRx()
{
    CE_LOW

    conf=CONFREG | PWR_UP_BIT | PRIM_RX_BIT;            // powerUP, Rx
    regWrite(CONFIG,&conf,1);

}

void Nrfp::pwrUpTx()
{
    CE_LOW

      conf=(CONFREG | PWR_UP_BIT) & ~(PRIM_RX_BIT);     // powerUP, Tx
      regWrite(CONFIG,&conf,1);
}

bool Nrfp::available()      // keep CE high when false
{
//Serial.print(" numNrf=");Serial.print(numNrf);Serial.print(" numBalise=");Serial.println(numBalise);
        // rechargement RX_ADDR_P0 après TX
        if((mode=='C') && (numNrf==numBalise) && !rxP0Set){
Serial.println("cc_addr->RX0 ; rxP0Set=true");
            regWrite(RX_ADDR_P0,cc_addr,ADDR_LENGTH); // addr pour inscriptions
            flushRx();                                // flush RX FIFO
            regw=RX_DR_BIT;
            regWrite(STATUS,&regw,1);                 // clr RX_DR_BIT
            rxP0Set=true;}

    CE_HIGH

    regRead(FIFO_STATUS,&fstat,1);
    if((fstat & RX_EMPTY_BIT)!=0){

        regRead(STATUS,&stat,1);
        if((stat & RX_DR_BIT)==0){
            return false;
        }
    }
    CE_LOW
    return true;  // dataRead should be done now
}

void Nrfp::dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength)
{
    regRead(STATUS,&stat,1);
    *pipe=(stat>>RX_P_NO)&0x03;                 // get pipe nb

    uint8_t maxLength=*pldLength;
    regRead((RX_PW_P0+pipe),*pldLength,1);      // get pldLength
    if(*pldLength>maxLength){
        *pldLength=0;
        flushRx();                              // if error flush all
        memset(data,0x00,*pldLength);
    }
    else{
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,pldLength);           // get pld
        CSN_HIGH
    }
    stat=RX_DR_BIT;
    regWrite(STATUS,&stat,1);                   // clear RX_DR bit
}

void Nrfp::dataWrite(uint8_t pipe,byte* data,char na,uint8_t len,byte* tx_addr)
{
    byte pipeAddr[ADDR_LENGTH];                // buffer TX_ADDR

    pwrUpTx();
    flushTx();

    if(mode=='C'){
        rxP0Set='false';        // rechargement RX_ADDR_P0 pour RX à faire
                                // (cc_addr)
        if(numNrf!=numBalise){  // addr de péri ou addr broadcast
            regWrite(TX_ADDR,tx_addr,ADDR_LENGTH);
            regWrite(RX_ADDR_P0,tx_addr,ADDR_LENGTH);}

        else{regWrite(TX_ADDR,br_addr,ADDR_LENGTH);
            na='N';}            // broadcast NO ACK
    }

    stat=TX_DS_BIT | MAX_RT_BIT;
    regWrite(STATUS,&stat,1);    // clear TX_DS & MAX_RT bits

    CSN_LOW
    if(na=='A'){SPI.transfer(W_TX_PAYLOAD);}        // avec ACK
    else{       SPI.transfer(W_TX_PAYLOAD_NA);}     // sans ACK
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    CE_HIGH                       // transmit

// transmitting() should be checked now to wait for end of paquet transmission
// or ACK reception before turning CE low
}

int Nrfp::transmitting()         // busy -> 1 ; empty -> 0 -> Rx ; MAX_RT -> -1
{
      int rtst=1;
      regRead(STATUS,&stat,1);
      if((stat & TX_DS_BIT)){rtst=0;pwrUpRx();}
      if((stat & MAX_RT_BIT)){
        rtst=-1;
        if(mode=='P'){memset(pi_addr,0x00,ADDR_LENGTH);} // refaire l'inscription
      }
      if(rtst<=0){
        stat = TX_DS_BIT | MAX_RT_BIT; // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat,1);
      }

      return rtst;
}

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

bool Nrfp::inscript()  // inscription péri pour obtenir une pipeAddr
{                      // true OK ; false MAXRT ou TO réception data


    regWrite(TX_ADDR,cc_addr,ADDR_LENGTH);    // adresse pour inscription
    regWrite(RX_ADDR_P0,cc_addr,ADDR_LENGTH);

    dataWrite(0,r1_addr,'A',ADDR_LENGTH,"00000");    // envoi macAddr
    int trst=1;
    while(trst==1){trst=transmitting();}

    if(trst<0){return false;}                // erreur MAX_RT

    long time_beg = micros();
    int readTo;
    while(!available()&& (readTo>=0)){
        readTo=TO_AVAILABLE-micros()+time_beg;}

    if(readTo>=0){
        uint8_t pipe,pldLength=ADDR_LENGTH;
        dataRead(pi_addr,&pipe,&pldLength);   // réception pipeAddr
        regWrite(TX_ADDR,pi_addr,ADDR_LENGTH); // adresse PTX update
        return true;}

    return false;                             // erreur TO
}
