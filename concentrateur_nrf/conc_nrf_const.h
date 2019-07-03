#ifndef _CON_NRF_CONST_
#define _CON_NRF_CONST_

#include "nrf24l01p.h"

/*
  *** périphérique ***

au démarrage péri en PTX (inscription):
    RX2=BR_ADDR
    RX1=macAddr
    RX0=CC_ADDR
    TX =CC_ADDR
    (concentrateur PRX RX0=cc_addr)
    le péri envoie la macAddr à cc_addr
    le concentrateur enregistre la macAddr associée à un pipe libre
    dans la tableC[] et envoie en PTX la pipeAddr à la macAddr
    la pipeAddr est stockée dans ccpipe 
en fonctionnement péri en PRX :
    RX2=BR_ADDR
    RX1=macAddr réception de commandes
en fonctionnement péri en PTX : (si échec MAX_RT ccPipe est effacé ; retour à l'incription)
    RX2=BR_ADDR
    RX1=macAddr réception de commandes
    RX0=ccPipe
    TX =ccPipe


  *** concentrateur ***
  
  il gère n circuits nrf27L01+
  
  la table des pipes stocke les adresses
      n° de circuit (1 à n)
      n° de pipe (1 à 5)
      adresse de pipe (aaann) (idem RX_ADDR_Rn)
      n° de périphérique (attribué par le serveur via dataRead/Set)
      macAddr du périphérique
  
le circuit BALISE pipe 0 est utilisé pour recevoir les demandes d'inscription et emettre la balide temporelle
les autres pipes du circuit BALISE fonctionne normalement
les pipes RX0 des autres circuits sont inutilisés (avec 6 circuits perte de 5/36 ... avec 2 circuits perte de 1/12)

évolution possible : channel indépendant par circuit avec channel fixe sur circuit BALISE et paramétrage du channel 
lors de l'inscription

concentrateur en PRX
   RX0=CC_ADDR     réception demandes d'inscriptions 
   RXn=adresses utilisées par les péri inscrits 
concentrateur en PTX d'inscription
   RX0=macAddr reçue
   TX =  idem
concentrateur en PTX de balise
   TX =BR_ADDR (pas d'ACK -> RX0 indifférent)
concentrateur en PTX "normal"
   RX0=macAddr associée au n°de périphérique adressé
   TX =  idem

*/  

  #define NBPERIF 24         // pour taille table

  #define NRF_MODE 'P'       // C concentrateur ; P périphérique

  #define PI_C 5             // nombre pipes dispo / circuit

  #define BR_ADDR  "bcast"   // adresse fixe de broadcast
  #define CC_ADDR  "ccons"   // adresse fixe du concentrateur pour inscription

  #define BALISE  1          // numéro du circuit du concentrateur pour inscriptions et balise 
                             // (inscriptions pipe0) (1 si concentrateur avec un seul circuit)
                             // sans effet en mode 'P'

#if NRF_MODE == 'P'
  #define BR_PIPE 0          // pipe pour réception broadcast
  #define R1_ADDR "peri1"    // MAC_ADDR PERI
  //#define NBPI 1           // une seule entrée de table
#endif
#if NRF_MODE == 'C'
  #define R1_ADDR "tot00"    // base des peri du concentrateur
  #define NBPI NBPERIF       // une entrée par périf
#endif

struct NrfConTable
{
  uint8_t numNrf24;         // numéro circuit nrf24L01+
  uint8_t numCirc;          // numéro circuit (1-n)
  uint8_t numPipe;          // numéro pipe (1-5) 
  byte    pipeAddr[ADDR_LENGTH];  // addr associée au pipe
  uint8_t numPeri;          // numéro périphérique pour serveur
  byte    periMac[5];       // macAddr 
};





#endif _CON_NRF_CONST_
