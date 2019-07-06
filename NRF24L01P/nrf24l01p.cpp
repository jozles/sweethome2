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
#define INIT_SPI  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
#define START_SPI SPI.begin();

#define PP4       pinMode(4,OUTPUT);digitalWrite(4,LOW);digitalWrite(4,HIGH);pinMode(4,INPUT);

#define DEF_CHANNEL 1

#define RX 1
#define TX 0

#define CONFREG 0x00 & ~(MASK_RX_DR_BIT) & ~(MASK_TX_DS_BIT) & ~(MASK_MAX_RT_BIT) | EN_CRC_BIT & ~(CRCO_BIT) & ~(PWR_UP_BIT) | PRIM_RX_BIT

#define RFREG   RF_DR_HIGH_BIT & ~(RF_DR_LOW_BIT) | RF_PWR_BITS

uint8_t numNrf=0;           // n° du circuit courant
uint8_t numBalise=0;        // n° du circuit balise
char    mode=' ';           // mode 'P' ou 'C'

bool rxP0Set[NB_NRF];        // force rechargement RX_P0 si false
                            // après modification pour TX avec ACK
byte baseAddr[NB_NRF*ADDR_LENGTH];       // RX_P0 pour chaque circuit

uint8_t regw,stat,fstat,conf;


Nrfp::Nrfp()    // constructeur
{
}

bool Nrfp::setNum(uint8_t circuit,uint8_t balise)
{
    if(mode=='P'){numNrf=0;numBalise=99;}
    if(mode=='C'){
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
 * Principe :
 *
 *  au démarrage, le bit PXR_UP et CE_LOW
 *  place le chip en mode standby en maxi 4,5mS
 *
 *  utilisation statique half duplex (messages gérés un par un)
 *
 *  pour transmettre :
 *
 *    charger le FIFO, bit PRIM_RX_low
 *    charger TX_ADDR (et RX_ADDR_P0 si auto ACK)
 *    CE high lance la transmission après un delais de 130uS
 *    attendre la fin de la transmission (TX_DS set ou MAX_RT set
 *                                        effacer TX_DS et MAX_RT)
 *    retour en mode standby avec CE low.
 *
 *    fonctions :
 *    dataWrite() charge le FIFO, les adresses, fait pwrUpTx()
 *                et termine avec CE_HIGH
 *    transmitting() teste la fin de transmission et termine avec CE_LOW
 *
 *  pour recevoir :
 *
 *    bit PRIM_RX_high, CE_HIGH commence l'écoute après un delais de 130uS
 *    attendre un paquet (RX_DR set ou RX_EMPTY clr ; effacer RX_DR)
 *    retour en standby avec CE low
 *    charger le message reçu
 *
 *    fonctions :
 *    available() recharge RX_ADDR_P0 si nécessaire (vidage FIFO dans ce cas)
 *               puis fait CE_HIGH et termine avec CE_LOW si true
 *    dataRead() contrôle la longuer maxi, transfère le message
 *               et récupère la longueur reçue et le n° de pipe
 *
 *
 *             ***** utilisation des pipes *****
 *             modèle en étoile : 1 concentrateur
 *           n circuits vers (n*6)-1 périphériques
 *
 *  Données communes :
 *
 *      br_addr (broadcast) messages du concentrateur vers tous les périphériques
 *      cc_addr (concentrateur) guichet unique d'inscription
 *
 *  périphériques :
 *
 *      macAddr unique allouée au pipe1 (pour mode PRX)
 *      (br_addr sur pipe0 en PRX)
 *      regAddr(ccPipe)  adresse fournie par le concentrateur (pour mode PTX)
 *      pRegister() envoie la macAddr et reçoit une regAddr
 *
 *  concentrateur :
 *
 *      un pipe est réservé pour la cc_addr
 *      chaque pipe est muni d'une adresse fixe
 *      l'adresse de chaque périphérique est associée à chaque circuit/pipe
 *      en réception dataRead() fournit le numéro de pipe emetteur
 *      en émission l'adresse est fournie à datawrite()
 *      cRegister() reçoit la macAddr et fournit une regAddr
 *
 *      numPeri numéro de périphérique (1-n)
 *      numPipe numéro de pipe de circuit du concentrateur (1-5)
 *      numNrf  numéro du circuit courant numPeri=(numNrf*10)+numPipe
 *
 *
 *
 *
 *
 *    CE et CSN sont pilotés par une variable unique
 *
 *
 */

bool Nrfp::config()           // power on config
{


    regw=EN_DYN_ACK_BIT | EN_DPL_BIT;  // no ack enable ; dyn pld length enable
//regw=EN_DYN_ACK_BIT;      // static payload length
    regWrite(FEATURE,&regw,1);
    regWrite(FEATURE,&regw,1);

    regw=(ENAA_P5_BIT|ENAA_P4_BIT|ENAA_P3_BIT|ENAA_P2_BIT|ENAA_P1_BIT|ENAA_P0_BIT);
    regWrite(EN_AA,&regw,1);

    regw=(ERX_P5_BIT|ERX_P4_BIT|ERX_P3_BIT|ERX_P2_BIT|ERX_P1_BIT|ERX_P0_BIT);
    regWrite(EN_RXADDR,&regw,1);

    regw=(DPL_P5_BIT|DPL_P4_BIT|DPL_P3_BIT|DPL_P2_BIT|DPL_P1_BIT|DPL_P0_BIT);
    regWrite(DYNPD,&regw,1);                  // dynamic payload length

    if(mode=='P'){
        regWrite(RX_ADDR_P1,r0_addr,ADDR_LENGTH); // macAddr
    }
Serial.print("mode");Serial.println(mode);
    if(mode=='C'){

        byte pAddr[NB_PIPE];
        memcpy(pAddr,r0_addr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]+=numNrf*NB_PIPE;
        regWrite(RX_ADDR_P0,pAddr,ADDR_LENGTH);   // RX0 du circuit
        memcpy(baseAddr+numNrf*ADDR_LENGTH,pAddr,ADDR_LENGTH);
        pAddr[ADDR_LENGTH-1]+=numNrf*NB_PIPE+1;
        regWrite(RX_ADDR_P1,pAddr,ADDR_LENGTH);   // RX1 du circuit

        for(uint8_t i=2;i<NB_PIPE;i++){          // fill pipes RX2-6
            regw=pAddr[ADDR_LENGTH-1]+i-1;
            regWrite((RX_ADDR_P0+i),&regw,1);
        }
    }

    rxP0Set[numNrf]=false;    // pour forcer la restauration de RX0
                              // à l'entrée de available()
                              // brAddr si 'P' ; ccAddr si 'C' && balise
                              // baseAddr+numNrf*ADDR_LENGTH si 'C' "normal"

    if(channel==0){channel=DEF_CHANNEL;}
    regWrite(RF_CH,&channel,1);

//  regw=MAX_PAYLOAD_LENGTH;
//  for(uint8_t i=0;i<6;i++){regWrite(RX_PW_P0+i,&regw,1);}  // static payload length

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
  if(!rxP0Set[numNrf]){                                 // rechargement RX_ADDR_P0 après TX
      rxP0Set[numNrf]=true;
      flushRx();                                // flush RX FIFO
      regw=RX_DR_BIT;
      regWrite(STATUS,&regw,1);                 // clr RX_DR_BIT

      if(mode=='C'){
         if(numNrf==numBalise){
           regWrite(RX_ADDR_P0,cc_addr,ADDR_LENGTH); // addr pour inscriptions
         }
         else{
           regWrite(RX_ADDR_P0,baseAddr+numNrf*ADDR_LENGTH,ADDR_LENGTH);
         }                                           // addr normale
      }

      if(mode=='P'){
           regWrite(RX_ADDR_P0,br_addr,ADDR_LENGTH); // addr pour broadcast
      }
  }

  CE_HIGH

  regRead(FIFO_STATUS,&fstat,1);
  if((fstat & RX_EMPTY_BIT)!=0){      // FIFO empty ?
      regRead(STATUS,&stat,1);
      if((stat & RX_DR_BIT)==0){      // RX empty ?
          return false;               // CE stay high while waiting
      }
  }

  CE_LOW
  PP4
  return true;  // dataRead should be done now
}

bool Nrfp::dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength)
{       // available() should have be done
    uint8_t maxLength=*pldLength;

    regRead(STATUS,&stat,1);
    *pipe=(stat & RX_P_NO_BIT)>>RX_P_NO;       // get pipe nb

    if(*pipe<=5){
        regRead(R_RX_PL_WID,*pldLength,1);}  // get pldLength (dynamic length)
        //regRead((RX_PW_P0+*pipe),*pldLength,1);}    // get pldLength (static length)

    if((*pipe<=5) && (*pldLength<=maxLength) && (*pldLength>0)){
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,*pldLength);              // get pld
        CSN_HIGH

        regw=RX_DR_BIT;
        regWrite(STATUS,&regw,1);                   // clear RX_DR bit
        return true;
    }
    else{
        flushRx();                                  // if error flush all
        return false;                               // invalid pipe nb or length
    }
}

void Nrfp::dataWrite(byte* data,char na,uint8_t len,byte* tx_addr)
{
    //fstat=TX_FULL_BIT;
    //while((fstat & TX_FULL_BIT)){regRead(FIFO_STATUS,&fstat,1);}   // wait for free FIFO
                                                            // manage TO in case of out of order chip
    regWrite(TX_ADDR,tx_addr,ADDR_LENGTH);
    if(na=='A'){
        regWrite(RX_ADDR_P0,tx_addr,ADDR_LENGTH);
        rxP0Set[numNrf]=false;      // RX_ADDR_P0 restore to PRX to do
    }                       // (cc_addr if 'C' ; br_addr if 'P' - see available())

    stat=TX_DS_BIT | MAX_RT_BIT;
    regWrite(STATUS,&stat,1);   // clear TX_DS & MAX_RT bits
//len=MAX_PAYLOAD_LENGTH;       // static length used
    CSN_LOW
    if(na=='A'){SPI.transfer(W_TX_PAYLOAD);}        // with ACK
    else{       SPI.transfer(W_TX_PAYLOAD_NA);}     // without ACK
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    pwrUpTx();
    CE_HIGH                       // transmit

// transmitting() should be checked now to wait for end of paquet transmission
// or ACK reception before turning CE low
}

int Nrfp::transmitting()         // busy -> 1 ; sent -> 0 -> Rx ; MAX_RT -> -1
{     // manage TO in case of out of order chip (trst=-2)
      int trst=1;

      regRead(STATUS,&stat,1);

      if((stat & TX_DS_BIT)){trst=0;}             // data sent
      if((stat & MAX_RT_BIT)){                    // max retry
        trst=-1;
        if(mode=='P'){memset(pi_addr,0x00,ADDR_LENGTH);} // de-validate inscription
      }
      if(trst<=0){                                // data sent or max retry
        stat = TX_DS_BIT | MAX_RT_BIT;            // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat,1);
        CE_LOW
        PP4
      }

      return trst;
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
{                      // true OK ; false MAXRT or TO receiving data
    rxP0Set[numNrf]=false;

    dataWrite(r0_addr,'N',ADDR_LENGTH,cc_addr);    // send macAddr to cc_ADDR
    int trst=1;
    while(trst==1){trst=transmitting();}

    if(trst<0){
            CE_LOW
            return false;}                     // MAX_RT error

    long time_beg = micros();
    int readTo;

    while(!available() && (readTo>=0)){
        readTo=TO_AVAILABLE-micros()+time_beg;}

    CE_LOW

    if(readTo>=0){                             // no TO data ready
        uint8_t pipe,pldLength=ADDR_LENGTH;
        dataRead(pi_addr,&pipe,&pldLength);    // get pipeAddr
        regWrite(TX_ADDR,pi_addr,ADDR_LENGTH); // PTX address update
        return true;}

    return false;                              // else TO error
}
