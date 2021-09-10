#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include <Arduino.h>
#include "shconst2.h"
#include "shutil2.h"

#define VERSION "1.9 "
#define LENVERSION 4

/*
Le serveur contient la config des concentrateurs ; (mac,ip pour forçage éventuel,adresse NRF,RF channel, RF speed) ;
MAXCONC concentrateurs.
Les périphériques sont attachés à un concentrateur (adresse NRF, RF Channel, RF speed chargés lors de la config série.)

Le circuit NRF n'autorise que 2 adresses en réception : RX0 et RX1 (NRF_ADDR_LENTH) ;
les autres RX sont "dérivés" du RX1 (seul le LSB change)
RX0 n'est utilisée que par les périphériques pour recevoir les messages de broadcast (à traiter)
RX1 des concentrateurs provient du serveur via la config série

>> à développer
>> L'adresse RX2 est utilisée par les concentrateurs avec une adresse générique. Ca permet l'appariemment périphérique/concentrateur.
>> Les demandes d'adresse de concentrateur sont faites en faible puissance par le périphérique au contact du concentrateur auquel le connecter.

Sur les périphériques l'adresse TX est fixe sur RX1 du concentrateur auquel ils sont attachés

Le concentrateur gère une table qui associe
    RX périphérique (+ un 6ème caractère : son rang dans la table) et son numéro de périphérique dans le serveur sweetHome
    + un buffer de réception du serveur et un buffer de réception du périphérique 
    (la dernière entrée de la table est utilisée pour les demandes d'adresse de concentrateur par les périphériques)

Tous les messages commencent par (RX1 du périphérique)/numT (numT rang dans la table 0x30 à 0xFF)
Le concentrateur contrôle et répond si ok ; sinon l'éventuelle RX1 est effacée de la table.
Si le numT est '0', le concentrateur recherche la RX1 dans la table, l'enregistre si inex puis répond numT.
Si c'est une demande d'appariement (pipe 2) le concentrateur répond avec son adresse générique en faible puissance. (à développer ?)

Tous les messages du concentrateur vers un périphériques sont de la forme :
  mmmmmTxxxxxx...xxxxx   mmmmm mac péri ; T rang dans table ; xxx...xxx buffer messages extérieur

v 1.3 La détection d'alim faible est effectuée lors de la lecture de la tension en début de boucle et ne bloque pas la boucle en cours.
La valeur utilisée comme seuil est la constante VOLTMIN (rien n'est passé depuis le serveur)
Le sixième byte de la macAddr des périfs pour le serveur prend la valeur du numéro de concentrateur + '0'
Sur les périfs, l'affichage des diags est controlé par la variable bool diags, true à la frappe d'une touche sur le terminal qui doit être branché avant le reset.
Sur le concentrateur diags est toujours true.
v1.31 Optimisation blinkDly pour diminuer la conso ; lethalSleep devrait être lethal (à vérifier) ; nrfp. devient radio. ; powerSleep nettoyé pour devenir une lib ; 
Ajout param PER_PO pour mode 'P' : 'P' if power Off/On radio ; 'N' if radio ever On (compatibilité avec proto) ; ajout VFACTOR et TFACTOR pour proto.
v1.4  EEprom stocke la config : macAddr peri, macAddr conc ; factor volts et thermo calibrés avec pgme de test ;
v1.5  transfert valeur entrée analogique + seuils (user_conc exportData()) le stockage des seuils est à faire dans importData() ; la lecture de l'entrée analogique aussi ;
v1.6  pas de retry si pas de réponse à txrx ou beginP() ; periode 10sec résistance 11.2K ; ajout calcul/affichage du temps de diag
v1.7  définition du numConc dans la config et paramétrage du canal et du nom de concentrateur selon numConc
v1.8  passage PC0(ADC0) PC6(ADC6) PD3(INT1-Rreed) PD5 PD6 au concentrateur
v1.9  config concentrateur en flash (eepr) chargée via Serial1 depuis server ; 
      numConc disparait remplacé par concNb qui provient du serveur (plus de config physique sur le concentrateur)
      En factory reset, concNb forme le 5ème caractère d'adresse radio des concentrateurs sur le serveur. 
      (l'adresse RX1 par défaut des concentrateurs est le param CC_NRF_ADDR (4 caractères) + le numéro d'entrée dans la table du serveur ;)
      Les périfs reçoivent une adresse de concentrateur complète et le n° (inutilisé) du concentrateur dans la table lors de la config série.
      L'adresse mac de périphérique utilisée par le serveur est l'adresse RX(mac) des périfs complétée par le concentrateur avec concNb.
      Adresse mac, ip, adresse NRF, channel, speed sont paramétrés dans la table des concentrateurs du serveur.
      
*/

/************* config ****************/
  
  #define NRF_MODE 'P'            //  P périphérique
  //#define NRF_MODE 'C'            //  C concentrateur  
/* !!!!!! changer de platformio.ini selon le NRF_MODE ('C'=due ou stm32 ; 'P' =328 !!!!! */

  #define TXRX_MODE 'U'           // TCP / UDP

  #define CB_ADDR   (byte*)"shcc0"      // adresse fixe de broadcast concentrateurs (recup MAC_ADDR concentrateurs) 0 nécessaire pour read()
  #define BR_ADDR   (byte*)"bcast"      // adresse fixe de broadcast

  #define CLK_PIN    13
  #define MISO_PIN   12
  #define MOSI_PIN   11

  #define NBPERIF 12                      // dim table
  #define BUF_SERVER_LENGTH LBUFSERVER    // to/from server buffer length

//  #define RF_SPEED   RF_SPD_1MB // vitesse radio  RF_SPD_2MB // RF_SPD_1MB // RF_SPD_250K
  #define ARD_VALUE  0          // ((0-15)+1) x 250uS delay before repeat
  #define ARC_VALUE  4          // (0-15) repetitions

#if NRF_MODE == 'C'
  #define DUE                   // DUE OU STM32...
  #define LED        PINLED     // 3 sur proto          
  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    8          // pin pour CS du SPI-nrf
  #define PORT_PP    7          // pin pour pulse de debug analyseur logique (macro PP4)
#endif // NRF_MODE == 'C'

#if NRF_MODE == 'P'             /* voltage and temp acquisition params */
   // param carte DETS (sinon ?) dans platformo.ini
  #define ATMEGA328                 // option ATMEGA8 ... manque de memoire programme (8K dispo et nécessite 17K)
  #define PER_PO    'P'           // 'N' no powoff 'P' powoff
  #define SPI_MODE                // SPI initialisé par la lib (ifndef -> lib externe)
  #define DEF_ADDR  "peri_"
  
  #define MCP9700                 //#define TMP36 //#define LM335 //#define DS18X20 // modèle thermomètre

#ifdef DETS
// led
  #define LED         PINLED
// NRF
  #define PORT_CSN    PORTB
  #define DDR_CSN     DDRB
  #define BIT_CSN     2
  #define CSN_PIN     10
  #define PORT_CE     PORTB
  #define DDR_CE      DDRB
  #define BIT_CE      1
  #define CE_PIN      9
// reed
  #define PORT_REED   PORTD
  #define DDR_REED    DDRD
  #define BIT_REED    3
  #define REED        3
// ports dispo
  #define PORT_DIG1   PORTD
  #define DDR_DIG1    DDRD
  #define BIT_DIG1    5
  #define DIG1        5
  #define PORT_DIG2   PORTD
  #define DDR_DIG2    DDRD
  #define BIT_DIG2    6
  #define DIG2        6
// done 5111
  #define PORT_DONE   PORTB
  #define DDR_DONE    DDRB
  #define BIT_DONE    0
  #define DONE        8
// spi
  #define PORT_MOSI   PORTB
  #define DDR_MOSI    DDRB
  #define BIT_MOSI    3
  #define PORT_CLK    PORTB
  #define DDR_CLK     DDRB
  #define BIT_CLK     5
// PP (debug pulse)
  #define PORT_PP     PORTD
  #define DDR_PP      DDRD
  #define BIT_PP      6
// volts
  #define PORT_VCHK   PORTC
  #define DDR_VCHK    DDRC
  #define BIT_VCHK    3
// nrf etc power ctl
  #define PORT_RPOW   PORTD
  #define DDR_RPOW    DDRD
  #define BIT_RPOW    7
  #define RPOW_PIN    7

  #define ISREDGE    RISING

  #define VCHECKADC 7             // VOLTS ADC pin Nb
  #define VCHECKHL HIGH           // command pin level for reading
  #define VADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | VCHECKADC     // internal 1,1V ref + ADC input for volts
  //#define VFACTOR 0.00810         // volts conversion 1K+6,8K Proto
  #define VFACTOR 0.00594         // volts conversion 1,5K+6,8K 
  #define TCHECKADC 1             // TEMP  ADC pin Nb (6 DETS1.0 ; 1 DETS2.0)
  #define TREF      25            // TEMP ref for TOFFSET 
  #define LTH       6             // len thermo name                                 
                                  // temp=(ADCreading/1024*ADCREF(mV)-TOFFSET(mV))/10+TREF                                
                                  // equivalent to // temp=(ADC*TFACTOR-(TOFFSET))+TREF (no dividing)
                                  // with
                                  // TFACTOR=1.1/10.24 or VCC/10.24 or AREF/10.24
                                  // TOFFSET voltage(mV)/10 @ TREF @ 10mV/°C
  #define A1CHECKADC 0            // user ADC1 
  #define A1ADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | A1CHECKADC     // internal 1,1V ref + ADC input for volts
  #define A2CHECKADC 6            // user ADC2
  #define A2ADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | A2CHECKADC     // internal 1,1V ref + ADC input for volts
#endif // def DETS
#ifndef DETS                      // params douteux
  #define LED        PINLED
  #define CSN_PIN    10
  #define CE_PIN     9
  #define VFACTOR 0.009           // volts conversion 3,9K+33K
  #define VCHECKADC 2             // volts ADC pin Nb
  #define VCHECK  A3              // volts arduino check pin
#endif // ndef DETS

// thermomètres
#ifdef LM335
  #define TADMUXVAL  0 | (0<<REFS1) | (1<<REFS0) | TCHECKADC     // ADVCC ref + ADC input for temp
  #define THERMO "LM335 "
  #define THN    'L'
  #define TFACTOR 0.806           // temp conversion pour LM335
  #define TOFFSET 750             // @25°
#endif // LM335
#ifdef TMP36
  #define THERMO "TMP36 "
  #define THN    'T'
  #define TADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | TCHECKADC     // internal 1,1V ref + ADC input for temp
  #define TFACTOR 1               // temp conversion pour TMP36
  #define TOFFSET 698             // @25°
#endif // TMP36
#ifdef MCP9700
  #define TADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | TCHECKADC     // internal 1,1V ref + ADC input for temp
  #define THERMO "MCP97 "
  #define THN    'M'
  #define TFACTOR 0.1074          // temp conversion pour MCP9700
  //#define TFACTOR 0.135          // temp conversion pour MCP9700 proto
  #define TOFFSET 75              // @25°
#endif // MCP9700
#ifdef DS18X20
  #define THERMO "DS18X "
  #define THN    'X'
  #define TFACTOR 1
  #define TOFFSET 0
  #define WPIN       5          // pin thermomètre
#endif // DS18X20

#define VOLTMIN 3.2             // minimal value to run

#endif // NRF_MODE == 'P'

#endif // _NRF_CONST_INCLUDED
