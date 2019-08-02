#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include "nrf24l01s.h"

#define VERSION "1.10"
#define LENVERSION 4

/*

Le circuit NRF n'autorise que 2 adresses en réception : RX0 et RX1 ;
les autres RX sont "dérivés" du RX1 (seul le LSB change)

Le système Auto ACK est inutilisé

L'adresse RX1 est utilisée pour recevoir les messages spécifiques au circuit
c'est la macAddr du circuit

L'adresse RX0 n'est utilisée que pour les périphériques pour recevoir
les messages de broadcast (

Sur les périphériques l'adresse TX est fixe sur la macAddr du concentrateur

Le concentrateur gère une table qui associe
    la macAddr du périphérique, son rang dans la table et son numéro de périphérique de sweetHome
    + un buffer de réception de sh et un buffer de réception du périphérique

Tous les messages commencent par macAddr/numP (numP 0x30 à 0xFF)
Le concentrateur contrôle et répond si ok ; sinon l'éventuelle macAddr est effacée de la table.
Si le numP est '0', le concentrateur le recherche et enregistre la macAddr si inex puis renvoie

*/

/*****************************/
  #define NRF_MODE 'P'            //  C concentrateur ; P périphérique
  #define UNO                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN

#if NRF_MODE == 'P'
  #define MAC_ADDR  PER_ADDR
#endif
#if NRF_MODE == 'C'
  #define MAC_ADDR  CC_ADDR
#endif

 // #define DIAG                    // affichages série

/****************************/

  #define PER_ADDR  "peri1"     // MAC_ADDR périphériques
  #define CC_ADDR   "toto_"     // MAC_ADDR concentrateur
  #define BR_ADDR   "bcast"     // adresse fixe de broadcast

  #define LED        5          // pin pour Led (13 pris par le clk du SPI)

  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    10         // pin pour CS du SPI

  #define CHANNEL    110        // numéro canal radio
  #define RF_SPEED   RF_SPD_1MB // vitesse radio  RF_SPD_2MB // RF_SPD_1MB // RF_SPD_250K
  #define ARD_VALUE  0          // ((0-15)+1) x 250uS delay before repeat
  #define ARC_VALUE  0          // (0-15) repetitions

  #ifdef UNO
  #define NBPERIF 8             //  pour dim table
  #endif
  #ifdef MEGA
  #define NBPERIF 24            //  pour dim table
  #endif  

#endif _NRF_CONST_INCLUDED
