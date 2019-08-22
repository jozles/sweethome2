#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include "nrf24l01s.h"

#define VERSION "1.11"
#define LENVERSION 4

/*

Le circuit NRF n'autorise que 2 adresses en réception : RX0 et RX1 ;
les autres RX sont "dérivés" du RX1 (seul le LSB change)

L'adresse RX1 est utilisée pour recevoir les messages spécifiques au circuit
c'est la macAddr du circuit

L'adresse RX0 n'est utilisée que par les périphériques pour recevoir
les messages de broadcast (à traiter)

Sur les périphériques l'adresse TX est fixe sur la macAddr du concentrateur

Le concentrateur gère une table qui associe
    la macAddr du périphérique, son rang dans la table et son numéro de périphérique de sweetHome
    + un buffer de réception de l'exterieur et un buffer de réception du périphérique
    (la macAddr est complétée du rang+48 dans la table pour avoir une longueur "normale" de 6 car) 

Tous les messages commencent par macAddr/numT (numT rang dans la table 0x30 à 0xFF)
Le concentrateur contrôle et répond si ok ; sinon l'éventuelle macAddr est effacée de la table.
Si le numT est '0', le concentrateur le recherche et enregistre la macAddr si inex puis renvoie

Tous les messages du concentrateur vers un périphériques sont de la forme :
  mmmmmTxxxxxx...xxxxx   mmmmm mac péri ; T rang dans table ; xxx...xxx buffer messages extérieur

*/

/*****************************/
  #define NRF_MODE 'C'            //  C concentrateur ; P périphérique
//  #define UNO                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc
  #define DUE                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc
//  #define MEGA                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc

#if NRF_MODE == 'P'
    //#define DETS
#endif NRF_MODE == 'P'    

  #define DIAG                    // affichages diags série

/****************************/

#if NRF_MODE == 'P'
  #define SPI_MODE            // SPI initialisé par la lib (ifndef -> lib externe)
  #define MAC_ADDR  PER_ADDR
  #define PER_ADDR  (byte*)"peri1"     // MAC_ADDR périphériques
#endif
#if NRF_MODE == 'C'
  #define MAC_ADDR  CC_ADDR
#endif

  #define CC_ADDR   (byte*)"toto_"     // MAC_ADDR concentrateur
  #define BR_ADDR   (byte*)"bcast"     // adresse fixe de broadcast

#if NRF_MODE == 'P'
#ifdef DETS
  #define LED        4
  #define CSN_PIN    10
  #define CE_PIN     9
#endif
#ifndef DETS
  #define LED        5
  #define CSN_PIN    10
  #define CE_PIN     9
#endif
#endif

#if NRF_MODE == 'C'
  #define LED        4
  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    8          // pin pour CS du SPI-nrf
#endif

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
  #ifdef DUE
  #define NBPERIF 24            //  pour dim table
  #endif  


#define VOLTMIN 3.5             // minimal value to run
//#define VFACTOR 0.0061          // volts conversion 2,2K+10K
#define VFACTOR 0.0047          // volts conversion 10K+33K
#define VCHECK  A3              // volts check pin
#ifdef  DETS
#define VINPUT  7               // volts ADC input pin
//#define VFACTOR 0.0061          // volts conversion 2,2K+10K
#define VFACTOR 0.0047          // volts conversion 10K+33K
#endif
#ifndef DETS                    // UNO d'essais
#define VFACTOR 0.009           // volts conversion 3,9K+33K
#define VINPUT  2               // volts ADC input pin
#endif



#endif _NRF_CONST_INCLUDED
