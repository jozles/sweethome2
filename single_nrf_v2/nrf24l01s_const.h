#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include <Arduino.h>
#include "shconst2.h"
#include "shutil2.h"

#define VERSION "1.3 "
#define LENVERSION 4

/*

Le circuit NRF n'autorise que 2 adresses en réception : RX0 et RX1 ;
les autres RX sont "dérivés" du RX1 (seul le LSB change)

L'adresse RX1 est utilisée pour recevoir les messages spécifiques au circuit ;
elle est utilisée comme macAddr du circuit ; sa longueur est de 5 bytes (ADDR_LENGTH) ;
Pour les communications entre le concentrateur et le serveur sweethome, un sixième byte (numConc) est ajouté ce qui forme une adresse unique pour le serveur.
Le numéro de concentrateur (numConc) provient des bits de port NUMC_BIT0 et NUMC_BIT1.
Ca fonctionne en laissant la même macAddr, la même IPAddr et le même numéro de port pour tous les concentrateurs. Si on ne change que la mac Addr le concentrateur ne reçoit plus les messages du serveur.
Le concentrateur et le périphérique utilisent le 6ème byte dans les messages : c'est le numéro d'entrée dans la table du concentrateur (voir plus loin)

L'adresse RX0 n'est utilisée que par les périphériques pour recevoir les messages de broadcast (à traiter)

Sur les périphériques l'adresse TX est fixe sur la macAddr du concentrateur

Le concentrateur gère une table qui associe
    la macAddr du périphérique (dont le 6ème caractère est son rang dans la table) et son numéro de périphérique dans le serveur sweetHome
    + un buffer de réception de l'exterieur et un buffer de réception du périphérique 

Tous les messages commencent par macAddr/numT (numT rang dans la table 0x30 à 0xFF)
Le concentrateur contrôle et répond si ok ; sinon l'éventuelle macAddr est effacée de la table.
Si le numT est '0', le concentrateur le recherche et enregistre la macAddr si inex puis renvoie

Tous les messages du concentrateur vers un périphériques sont de la forme :
  mmmmmTxxxxxx...xxxxx   mmmmm mac péri ; T rang dans table ; xxx...xxx buffer messages extérieur

v 1.3 La détection d'alim faible est effectuée lors de la lecture de la tension en début de boucle et ne bloque pas la boucle en cours.
La valeur utilisée comme seuil est la constante VOLTMIN (rien n'est passé depuis le serveur)
Le sixième byte de la macAddr des périfs pour le serveur prend la valeur du numéro de concentrateur + '0'
Sur les périfs, l'affichage des diags est controlé par la variable bool diags, true à la frappe d'une touche sur le terminal qui doit être branché avant le reset.
Sur le concentrateur diags est toujours true.
*/

#define ATMEGA328                 // option ATMEGA8 ... manque de memoire programme (8K dispo et nécessite 17K)

/************* config ****************/
  
  #define NRF_MODE 'P'            //  C concentrateur ; P périphérique
  
  #define UNO                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc
//  #define DUE                     //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc
//  #define MEGA                    //  UNO ou MEGA ou DUE  (PRO MINI id UNO) pour accélération CE/CSN / taille table etc

#if NRF_MODE == 'P'
    #define DETS                  // carte DETS (sinon UNO etc)
#endif NRF_MODE == 'C'    

  #define TXRX_MODE 'U'           // TCP / UDP

  #define MCP9700                 //#define TMP36 //#define LM335 //#define DS18X20 // modèle thermomètre

/**************************************/

#if NRF_MODE == 'P'
  #define SPI_MODE                // SPI initialisé par la lib (ifndef -> lib externe)
  #define MAC_ADDR  PER_ADDR
  #define PER_ADDR  "peri9"       // MAC_ADDR périphériques
#endif
#if NRF_MODE == 'C'
  #define MAC_ADDR  CC_ADDR
#endif

  #define CC_ADDR   (byte*)"ctest" //"toto_"      // MAC_ADDR concentrateur
  #define BR_ADDR   (byte*)"bcast"      // adresse fixe de broadcast

#define CLK_PIN    13
#define MISO_PIN   12
#define MOSI_PIN   11

#if NRF_MODE == 'P'
#ifdef DETS
  #define PORT_LED    PORTD
  #define DDR_LED     DDRD
  #define BIT_LED     4
  #define LED         4
  #define PORT_CSN    PORTB
  #define DDR_CSN     DDRB
  #define BIT_CSN     2
  #define CSN_PIN     10
  #define PORT_CE     PORTB
  #define DDR_CE      DDRB
  #define BIT_CE      1
  #define CE_PIN      9
  #define PORT_REED   PORTD
  #define DDR_REED    DDRD
  #define BIT_REED    3
  #define REED        3
  #define PORT_DONE   PORTB
  #define DDR_DONE    DDRB
  #define BIT_DONE    0
  #define DONE        8
  #define PORT_MOSI   PORTB
  #define DDR_MOSI    DDRB
  #define BIT_MOSI    3
  #define PORT_CLK    PORTB
  #define DDR_CLK     DDRB
  #define BIT_CLK     5
  #define PORT_PP     PORTD
  #define DDR_PP      DDRD
  #define BIT_PP      6
  #define PORT_VCHK   PORTC
  #define DDR_VCHK    DDRC
  #define BIT_VCHK    3
  #define PORT_RPOW   PORTD
  #define DDR_RPOW    DDRD
  #define BIT_RPOW    7
  #define RPOW_PIN    7

  #define ISREDGE    RISING
#endif
#ifndef DETS
  #define LED        5
  #define CSN_PIN    10
  #define CE_PIN     9
#endif
#endif

#if NRF_MODE == 'C'
  #define LED        2          // 3 sur proto          
  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    8          // pin pour CS du SPI-nrf
  #define PP         7
  #define NUMC_BIT0 14          // numConc low bit
  #define NUMC_BIT1 15          // numConc high bit
#endif

  #define CHANNEL    110        // numéro canal radio
  #define RF_SPEED   RF_SPD_1MB // vitesse radio  RF_SPD_2MB // RF_SPD_1MB // RF_SPD_250K
  #define ARD_VALUE  0          // ((0-15)+1) x 250uS delay before repeat
  #define ARC_VALUE  4          // (0-15) repetitions

  #ifdef UNO
  #define NBPERIF 8             //  pour dim table
  #endif
  #ifdef MEGA
  #define NBPERIF 12            //  pour dim table
  #endif  
  #ifdef DUE
  #define NBPERIF 12            //  pour dim table
  #endif  

#if NRF_MODE == 'P'             /* voltage and temp acquisition params */

#define VOLTMIN 3.2             // minimal value to run
#define VCHECK  A3              // volts arduino check pin
#define VCHECKHL HIGH           // command pin level for reading

#ifdef  DETS
#define VCHECKADC 7             // VOLTS ADC pin Nb
#define VADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | VCHECKADC     // internal 1,1V ref + ADC input for volts
//#define VFACTOR 0.00845         // volts conversion 1K+6,8K (MOSFET)
#define VFACTOR 0.00594         // volts conversion 1,5K+6,8K (MOSFET)
#define TCHECKADC 1             // TEMP  ADC pin Nb (6 DETS1.0 ; 1 DETS2.0)
#define TREF      25            // TEMP ref for TOFFSET 
#define LTH       6             // len thermo name                                 
                                // temp=(ADCreading/1024*ADCREF(mV)-TOFFSET(mV))/10+TREF                                
                                // equivalent to // temp=(ADC*TFACTOR-(TOFFSET))+TREF (no dividing)
                                // with
                                // TFACTOR=1.1/10.24 or VCC/10.24 or AREF/10.24
                                // TOFFSET voltage(mV)/10 @ TREF @ 10mV/°C


#ifdef LM335
#define TADMUXVAL  0 | (0<<REFS1) | (1<<REFS0) | TCHECKADC     // ADVCC ref + ADC input for temp
#define THERMO "LM335 "
#define THN    'L'
#define TFACTOR 0.806           // temp conversion pour LM335
#define TOFFSET 750             // @25°
#endif LM335
#ifdef TMP36
#define THERMO "TMP36 "
#define THN    'T'
#define TADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | TCHECKADC     // internal 1,1V ref + ADC input for temp
#define TFACTOR 1               // temp conversion pour TMP36
#define TOFFSET 698             // @25°
#endif TMP36
#ifdef MCP9700
#define TADMUXVAL  0 | (1<<REFS1) | (1<<REFS0) | TCHECKADC     // internal 1,1V ref + ADC input for temp
#define THERMO "MCP97 "
#define THN    'M'
#define TFACTOR 0.1074          // temp conversion pour MCP9700
#define TOFFSET 75              // @25°
#endif MCP9700
#ifdef DS18X20
#define THERMO "DS18X "
#define THN    'X'
#define TFACTOR 1
#define TOFFSET 0
#define WPIN       5          // pin thermomètre
#endif DS18X20

#endif // def DETS

#ifndef DETS                    // UNO d'essais
#define VFACTOR 0.009           // volts conversion 3,9K+33K
#define VCHECKADC 2             // ATMEGA ADC pin Nb
#endif // ndef DETS
#endif NRF_MODE == 'P'

#define BUF_SERVER_LENGTH LBUFSERVER    // to/from server buffer length

#endif _NRF_CONST_INCLUDED
