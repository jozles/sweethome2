#ifndef _NRF_CONST_INCLUDED
#define _NRF_CONST_INCLUDED

#include "radio_const.h"

//  #define RF_SPEED   RF_SPD_1MB // vitesse radio  RF_SPD_2MB // RF_SPD_1MB // RF_SPD_250K
  #define ARD_VALUE  0          // ((0-15)+1) x 250uS delay before repeat
  #define ARC_VALUE  4          // (0-15) repetitions

#if NRF_MODE == 'C'
         
  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    8          // pin pour CS du SPI-nrf
  #define PORT_PP    7          // pin pour pulse de debug analyseur logique (macro PP4)
  //#define REDV1                 // modèle carte red

  #define LRAMREM 16

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
  #define PLED         PINLED
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
  #define PLED        PINLED
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
