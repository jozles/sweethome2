#ifndef NRF24L01P_H_INCLUDED
#define NRF24L01P_H_INCLUDED

/* AP NFR24L01+ node single */

/*
 * Fonctionnement du NRF24L01+ :
 *
 *  au démarrage, le bit PWR_UP et CE_LOW
 *  place le chip en mode standby en maxi 4,5mS
 *
 *  pour transmettre :
 *
 *    (a)charger le FIFO, bit PRIM_RX_low
 *    (b)charger TX_ADDR (et RX_ADDR_P0 si auto ACK)
 *    (c)CE high lance la transmission aprés un delais de 130uS
 *    (d)attendre la fin de la transmission (TX_DS set ou MAX_RT set
 *                                        effacer TX_DS et MAX_RT)
 *    (e)retour en mode standby avec CE low.
 *
 *    fonctions :
 *    Write()     (a)(b)(c)
 *                sort du mode prx
 *                charge le FIFO, les adresses P0 et TX (si 'C'), fait setTx() et flushTx()
 *                et termine avec CE_HIGH
 *    transmitting() (d)(e)
 *                teste la fin de transmission, et passe en mode prx via CE_LOW
 *
 *  pour recevoir :
 *
 *    (f)bit PRIM_RX_high, CE_HIGH commence l'écoute aprés un delais de 130uS
 *    (g)attendre un paquet (RX_DR set ou fifo RX_EMPTY clr ; effacer RX_DR)
 *    (h)controler n° pipe et longueur reçue
 *    (i)retour en standby avec CE low
 *    (j)charger le message reçu
 *
 *    fonctions :
 *    available() (f)(g)(h)
 *                recharge RX_ADDR_P0 si nécessaire (vidage FIFO dans ce cas)
 *                passe en mode prx (pwrUpTx+CE_HIGH) si nécessaire au premier test
 *                contrôle la longueur maxi, transf�re le message
 *                et récupère la longueur reçue et le n° de pipe
 *                termine avec CE_LOW si true (fin du mode prx)
 *    Read()      (f)(g)(h)(i)
 *                effectue available() à la demande
 *
 *
 *             ***** pas d'utilisation des pipes *****
 * RX_P1 du concentrateur reçoit tous les périphériques et emmet avec TX et RX_P0
 * RX_P2 du concentrateur reçoit les requêtes d'adresse de concentrateur des périphériques qui s'apparient
 * RX_P1 des périphériques reçoit les messages du concentrateurs ; TX/RX_P0 fixes sur concentrateur
 * 
 *
 *  périphériques :
 *
 *      macAddr unique allouée au pipe1 (pour mode PRX)
 *      pRegister() envoie la macAddr et reçoit le numT 
 *      n° d'entrée de la table où la macAddr du périphérique est stockée
 *      
 *  concentrateur :
 *
 *      cRegister() reoit la macAddr et renvoie le numT
 *
 *
 *
 *
 */
#include <Arduino.h>

#include "nRF24L01.h"       // mnemonics
#include "nrf24l01s_const.h"

#define POWONDLY 100       // millis()  - 100 -
#define POWUPDLY 5         // millis()

#define ACK     true
#define NO_ACK  false

#define NB_PIPE 2          // nombre pipes utilisées
#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5

#define TO_AVAILABLE 100   // millis()
#define TO_REGISTER  100   // millis()

#define RF_SPD_2MB RF_DR_HIGH_BIT
#define RF_SPD_1MB 0
#define RF_SPD_250K RF_DR_LOW_BIT

/*** return (error) codes ***/
#define AV_NBPIP -1
#define AV_LMERR -2
#define AV_MCADD -3 // read output (need message upload)
#define AV_EMPTY -4
#define AV_MAXAV AV_EMPTY
#define ER_MAXRT (AV_MAXAV)-1 // code erreur MAX_RT
#define ER_RDYTO (ER_MAXRT)-1 // code erreur Time Out attente réception
#define ER_MAXER (ER_RDYTO)
#define ER_TEXT "to\0rt\0em\0mc\0le\0pi\0--\0ok\0" // 2 char libs for codes

/*** logic analyzer debug pulse ***/

#ifdef  UNO        // idem for PRO MINI
  #define PP4       bitClear(PORT_PP,BIT_PP);bitSet(PORT_PP,BIT_PP);
  #define PP4_INIT  bitSet(PORT_PP,BIT_PP);bitSet(DDR_PP,BIT_PP);
  #define PP4_OFF   bitClear(DDR_PP,BIT_PP);
  #define PP4_HIGH  bitSet(DDR_PP,BIT_PP);bitSet(PORT_PP,BIT_PP);
  #define PP4_LOW   bitSet(DDR_PP,BIT_PP);bitClear(PORT_PP,BIT_PP);  
#endif UNO
#ifndef UNO
  #define PP4       digitalWrite(PP,LOW);digitalWrite(PP,HIGH);
  #define PP4_INIT  pinMode(PP,OUTPUT);
#endif
  


#if NRF_MODE == 'C'

// extDataStore() output status
#define EDS_STAT_OK 1
#define EDS_STAT_PER 2
#define EDS_STAT_LEN 3 

struct NrfConTable
{
  uint8_t numPeri;                  // numéro périphérique dans la table du serveur
  byte    periMac[ADDR_LENGTH+2];   // macAddr (ajout 1 car=num entrée de la table pour former une addr mac pour l'extérieur)
  char    servBuf[MAX_PAYLOAD_LENGTH+1];
  uint8_t servBufLength;
  bool    servBufSent;
  char    periBuf[MAX_PAYLOAD_LENGTH+1];
  uint8_t periBufLength;
  bool    periBufSent;
};

#define SBVINIT "00072_00024_0025"  // server buffer init value
#define SBLINIT 16                  // server buffer init length

#endif // NRF_MODE == 'C'


class Nrfp
{
  public:
    Nrfp();

    void setup();

    int  available(uint8_t* pipe,uint8_t* length);
    int  read(byte* data,uint8_t* pipe,uint8_t* length,int numP);
    void readStop();
    void write(byte* data,bool ack,uint8_t len,uint8_t numP);
    int  transmitting(bool ack);

    void powerOn();
    void powerOff();
    void powerUp();
    void powerDown();
    void regRead(uint8_t reg,byte* data);
    void addrWrite(uint8_t reg,byte* data);

    void printAddr(char* addr,char n);

    bool powerD=true;         // etat power (true=down)
    uint8_t lastSta=0;
    byte*   locAddr;          // local Addr        

#if NRF_MODE == 'P'
    byte*   ccAddr; 
    int     pRegister(byte* message,uint8_t* pldLength);
    int     txRx(byte* message,uint8_t* pldLength);
#endif // NRF_MODE == 'P'
#if NRF_MODE == 'C'
    void    tableCInit();
    void    tableCPrint();
    uint8_t cRegister(char* message);
    uint8_t macSearch(char* mac,int* numPer);
    uint8_t extDataStore(uint8_t numPer,uint8_t numT,char* data,uint8_t len);
#endif NRF_MODE == 'C'

  private:

    void regWrite(uint8_t reg,byte* data);
    void addrRead(uint8_t reg,byte* data);
//    void addrWrite(uint8_t reg,byte* data);
    bool letsPrx();

    void rxError();
    void flushRx();
    void flushTx();
    void setRx();
    void setTx();

    int tableCLoad();
    int tableCSave();

    
    bool prxMode=false;       // true=circuit en PRX (pwrUpRx(), CE_HIGH)

    uint8_t regw,statu,fstatu,conf;

#if NRF_MODE == 'P'
    void allPinsLow();
#endif NRF_MODE == 'P'
};

#endif // NRF24L01P INCLUDED
