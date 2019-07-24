#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include "nrf24l01p.h"

#define VERSION "1.10"
#define LENVERSION 4

/*
  *** périphérique ***

Le circuit NRF n'autorise que 2 adresses en réception : RX0 et RX1 ;
les autres RX sont "dérivés" du RX1 (seul le LSB change)
au démarrage péri en PTX (inscription):
    RX1=macAddr
    RX0=CC_ADDR
    TX =CC_ADDR
    (concentrateur PRX RX0=cc_addr)
    le péri envoie la macAddr à CC_ADDR
    le concentrateur enregistre la macAddr associée à un pipe libre
    dans la tableC[] et envoie en PTX la pipeAddr à la macAddr
    la pipeAddr est stockée dans ccPipe
en fonctionnement péri en PRX :
    RX0=BR_ADDR (restauré à chaque entrée dans available() avant CE_HIGH)
    RX1=macAddr réception de commandes
en fonctionnement péri en PTX : (si échec MAX_RT ccPipe est effacé ; retour à l'incription)
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

concentrateur en PRX
   RX0=CC_ADDR     réception demandes d'inscriptions (restauré à chaque entrée dans available() avant CE_HIGH)
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


//****************************/
  #define NRF_MODE 'P'        //  C concentrateur ; P périphérique

#if NRF_MODE == 'P'
  #define R0_ADDR "peri2"     //  MAC_ADDR PERI
  #define UNO                 //  UNO ou MEGA ou DUE  (PRO MINI id UNO)
  #define BR_PIPE 0           //  pipe pour réception broadcast
  #define NB_CIRCUIT 1        //  nombre de circuits nrf
#endif
//************************/

#if NRF_MODE == 'C'
  #define NB_CIRCUIT 1         // nombre de circuits nrf
  #define R0_ADDR "tot00"      // adresse de base des peri du concentrateur
#endif

  #define LED        5          // pin pour Led (13 pris par le clk du SPI)

  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    10         // pin pour CS du SPI

  #define CHANNEL    110        // numéro canal radio
  #define RF_SPEED   RF_SPD_1MB // vitesse radio  RF_SPD_2MB // RF_SPD_1MB // RF_SPD_250K
  #define ARD        2          // (1-16) x 250uS delay before repeat
  #define ARC        15         // (0-15) repetitions
  #define BR_ADDR    "bcast"    // adresse fixe de broadcast
  #define CC_ADDR    "ccons"    // adresse fixe du concentrateur pour inscription

  #define BALISE  0             // toujours 0  (sans effet en mode 'P')
                                // numéro du circuit du concentrateur pour inscriptions et balise
                                // (inscriptions pipe0)
  #define CIRCUIT 0             // n° circuit initial

  #define NBPERIF 24            //  pour dim table

#endif _NRF_CONST_INCLUDED
