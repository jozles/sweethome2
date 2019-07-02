#ifndef _CON_NRF_CONST_
#define _CON_NRF_CONST_

  /* périphérique */
/*
au démarrage PTX (inscription):
    RX1=mac
    RX0=cc_addr
    TX =cc_addr
    (concentrateur RX0=cc_addr)
    le péri envoie la macAddr à cc_addr
    le concentrateur enregistre la macAddr associée à un pipe
    dans la tableC[].vvvv 
    et envoie (PTX) la pipeAddr à la macAddr
    la pipeAddr est stockée dans une entrée unique de table
en fonctionnement péri PRX :
    RX1=mac réception de commandes
    RX0=br_addr
en fonctionnement péri PTX :
    RX1=mac réception de commandes
    RX0=pipeAddr
    TX =pipeAddr

    la lib assure le rechargement de br_addr à chaque retour en PRX -> available()

*/

  /* concentrateur */
  // gère n circuits nrf27L01+
  // la table des pipes organise les adresses
  //    n° de circuit (1 à n)
  //    n° de pipe (1 à 5)
  //    n° de périphérique (attribué par le serveur via dataRead/Set)
  //    macAddr du périphérique
  //
  // le circuit 0 est utilisé pour recevoir les demandes d'inscription et emettre la balide temporelle
  //

  #define NBPERIF 24

  #define NRF_MODE 'P'         // C concentrateur ; P périphérique

  #define PI_C 5             // pipes dispo / circuit

  #define BR_ADDR  "bcast"   // adresse fixe de broadcast
  #define CC_ADDR  "ccons"   // adresse fixe du concentrateur pour inscription

#if NRF_MODE == 'P'
  #define R1_ADDR "peri1"    // MAC_ADDR PERI
  #define NBPI 1             // une seule entrée de table
#endif
#if NRF_MODE == 'C'
  #define R1_ADDR "toto0"    // base des peri du concentrateur
  #define NBPI NBPERIF       // une entrée par périf
#endif

struct NrfConTable
{
  uint8_t numNrf24;         // numéro circuit nrf24L01+
  uint8_t numCirc;          // numéro circuit (1-n)
  uint8_t numPipe;          // numéro pipe (1-5) 
  uint8_t numPeri;          // numéro périphérique pour serveur
  byte    periMac[5];       // macAddr 
};





#endif _CON_NRF_CONST_
