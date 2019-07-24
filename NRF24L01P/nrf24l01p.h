#ifndef NRF24L01P_H_INCLUDED
#define NRF24L01P_H_INCLUDED

/* AP NFR24L01+ node multi*multi */

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
 *    (c)CE high lance la transmission après un delais de 130uS
 *    (d)attendre la fin de la transmission (TX_DS set ou MAX_RT set
 *                                        effacer TX_DS et MAX_RT)
 *    (e)retour en mode standby avec CE low.
 *
 *    fonctions :
 *    Write()     (a)(b)(c)
 *                sort du mode prx
 *                charge le FIFO, les adresses, fait pwrUpTx()
 *                et termine avec CE_HIGH
 *    transmitting() (d)(e)
 *                teste la fin de transmission, et passe en mode prx via CE_LOW
 *
 *  pour recevoir :
 *
 *    (f)bit PRIM_RX_high, CE_HIGH commence l'écoute après un delais de 130uS
 *    (g)attendre un paquet (RX_DR set ou RX_EMPTY clr ; effacer RX_DR)
 *    (h)retour en standby avec CE low
 *    (i)charger le message reçu
 *
 *    fonctions :
 *    available() (f)(g)(h)
 *                recharge RX_ADDR_P0 si nécessaire (vidage FIFO dans ce cas)
 *                passe en mode prx (pwrUpTx+CE_HIGH) si nécessaire au premier test
 *                contrôle la longueur maxi, transfère le message
 *                et récupère la longueur reçue et le n° de pipe
 *                termine avec CE_LOW si true (fin du mode prx)
 *    Read()      (f)(g)(h)(i)
 *                effectue available()
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
 */


#include <Arduino.h>

#include "nRF24L01.h"       // mnemonics
#include "nrf24l01p_const.h"

#define NB_PIPE 6           // nombre pipes par circuit
#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5

#define TO_AVAILABLE 1000   // millis()
#define TO_REGISTER  1000   // millis()

#define RF_SPD_2MB RF_DR_HIGH_BIT
#define RF_SPD_1MB 0
#define RF_SPD_250K RF_DR_LOW_BIT

/*** return (error) codes ***/
#define AV_NBPIP -1
#define AV_LMERR -2
#define AV_MCADD -3 // read output (need message upload)
#define AV_EMPTY -4
#define AV_MAXAV AV_EMPTY
#define ER_MAXRT AV_MAXAV-1 // code erreur MAX_RT
#define ER_RDYTO ER_MAXRT-1 // code erreur Time Out attente réception
#define ER_MAXER ER_RDYTO
#define ER_TEXT "to\0rt\0em\0mc\0le\0pi\0--\0ok\0" // 2 char libs for codes

#if NRF_MODE == 'C'
struct NrfConTable
{
  uint8_t numNrf24;                 // numéro circuit nrf24L01+
  uint8_t numCirc;                  // numéro circuit (1-n)
  uint8_t numPipe;                  // numéro pipe (1-5)
  byte    pipeAddr[ADDR_LENGTH+1];  // addr associée au pipe
  uint8_t numPeri;                  // numéro périphérique pour serveur
  byte    periMac[ADDR_LENGTH+1];   // macAddr
};
#endif // NRF_MODE == 'C'


class Nrfp
{
  public:
    Nrfp();
    bool confCircuit();

    int  available(uint8_t* pipe,uint8_t* length);
    int  read(char* data,uint8_t* pipe,uint8_t* length,uint8_t numP);
    void write(byte* data,char na,uint8_t len,uint8_t numP);
    int  transmitting();

    void powerUp();
    void powerDown();
    void regRead(uint8_t reg,byte* data);

    void printAddr(char* addr,char n);

#if NRF_MODE == 'P'
    void setup(byte* expi_addr);
    int  pRegister();
#endif // NRF_MODE == 'P'
#if NRF_MODE == 'C'
    void setup();
    bool setNum(uint8_t circuit,uint8_t balise);
    void tableCInit();
    void tableCPrint();
    uint8_t cRegister(char* message);
#endif NRF_MODE == 'C'

  private:

    void regWrite(uint8_t reg,byte* data);
    void addrRead(uint8_t reg,byte* data);
    void addrWrite(uint8_t reg,byte* data);
    bool letsPrx();

    void rxError();
    void flushRx();
    void flushTx();
    void setRx();
    void setTx();

    int Nrfp::tableCLoad();
    int Nrfp::tableCSave();
};

#endif // NRF24L01P INCLUDED

