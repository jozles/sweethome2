#ifndef NRF24L01P_H_INCLUDED
#define NRF24L01P_H_INCLUDED

/* AP NFR24L01+ node multi*multi */

/*
 * Fonctionnement du NRF24L01+ :
 *
 *  au d�marrage, le bit PWR_UP et CE_LOW
 *  place le chip en mode standby en maxi 4,5mS
 *
 *  pour transmettre :
 *
 *    (a)charger le FIFO, bit PRIM_RX_low
 *    (b)charger TX_ADDR (et RX_ADDR_P0 si auto ACK)
 *    (c)CE high lance la transmission apr�s un delais de 130uS
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
 *    (f)bit PRIM_RX_high, CE_HIGH commence l'�coute apr�s un delais de 130uS
 *    (g)attendre un paquet (RX_DR set ou RX_EMPTY clr ; effacer RX_DR)
 *    (h)retour en standby avec CE low
 *    (i)charger le message re�u
 *
 *    fonctions :
 *    available() (f)(g)(h)
 *                recharge RX_ADDR_P0 si n�cessaire (vidage FIFO dans ce cas)
 *                passe en mode prx (pwrUpTx+CE_HIGH) si n�cessaire au premier test
 *                contr�le la longueur maxi, transf�re le message
 *                et r�cup�re la longueur re�ue et le n� de pipe
 *                termine avec CE_LOW si true (fin du mode prx)
 *    Read()      (f)(g)(h)(i)
 *                effectue available()
 *
 *
 *             ***** utilisation des pipes *****
 *             mod�le en �toile : 1 concentrateur
 *           n circuits vers (n*6)-1 p�riph�riques
 *
 *  Donn�es communes :
 *
 *      br_addr (broadcast) messages du concentrateur vers tous les p�riph�riques
 *      cc_addr (concentrateur) guichet unique d'inscription
 *
 *  p�riph�riques :
 *
 *      macAddr unique allou�e au pipe1 (pour mode PRX)
 *      (br_addr sur pipe0 en PRX)
 *      regAddr(ccPipe)  adresse fournie par le concentrateur (pour mode PTX)
 *      pRegister() envoie la macAddr et re�oit une regAddr
 *
 *  concentrateur :
 *
 *      un pipe est r�serv� pour la cc_addr
 *      chaque pipe est muni d'une adresse fixe
 *      l'adresse de chaque p�riph�rique est associ�e � chaque circuit/pipe
 *      en r�ception dataRead() fournit le num�ro de pipe emetteur
 *      en �mission l'adresse est fournie � datawrite()
 *      cRegister() re�oit la macAddr et fournit une regAddr
 *
 *      numPeri num�ro de p�riph�rique (1-n)
 *      numPipe num�ro de pipe de circuit du concentrateur (1-5)
 *      numNrf  num�ro du circuit courant numPeri=(numNrf*10)+numPipe
 *
 *
 *
 */


#include <Arduino.h>

#include "nRF24L01.h"       // mnemonics

#define MAX_NRF 2           // nombre maxi circuits
#define NB_PIPE 6           // nombre pipes par circuit
#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5

#define TO_AVAILABLE 1000   // millis()
#define TO_REGISTER  1000   // millis()

#define RF_SPD_2MB RF_DR_HIGH_BIT
#define RF_SPD_1MB 0
#define RF_SPD_250K RF_DR_LOW_BIT



class Nrfp
{
  public:
    Nrfp();
    void setup(char exMode,uint8_t exCePin,uint8_t exCsnPin,
                 uint8_t exNbNrf,uint8_t exChannel,byte exSpeed,
                 byte ardarc,byte* exBr_addr,
                 byte* exCc_addr,byte* exR0_addr,byte* exPi_addr);
    void confCircuit();
    bool begin();
    bool setNum(uint8_t circuit,uint8_t balise);

    int  available(uint8_t* pipe,uint8_t* length);
    int  read(char* data,uint8_t* pipe,uint8_t* length);
    void write(byte* data,char na,uint8_t len,byte* tx_addr);
    int  transmitting();

    void powerUp();
    void powerDown();
    void regRead(uint8_t reg,byte* data);

  private:

    void regWrite(uint8_t reg,byte* data);
    void addrRead(uint8_t reg,byte* data);
    void addrWrite(uint8_t reg,byte* data);
    bool letsPrx();
    bool pRegister();
    void flushRx();
    void flushTx();
    void setRx();
    void setTx();
};

#endif // NRF24L01P INCLUDED

