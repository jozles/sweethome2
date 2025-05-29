#ifndef _RADIO_CONST_INCLUDED
#define _RADIO_CONST_INCLUDED

#define VERSION "2.c "
#define LENVERSION 4

#define NRF // LORA //NRF


#include <Arduino.h>
#include <shconst2.h>
#include <nrf24l01s.h>

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
v1.9  configs concentrateurs et périfs nrf chargées via Serial1 depuis le server (getServerConfig); 
      numConc disparait remplacé par concNb qui provient du serveur (plus de config physique sur le concentrateur)
      En factory reset, le rang dans la table des concentrateurs forme le 5ème caractère d'adresse radio des concentrateurs sur le serveur. 
      (l'adresse RX1 par défaut des concentrateurs est le param CC_NRF_ADDR (4 caractères) + le numéro d'entrée dans la table du serveur ;)
      concNb indique le numéro de concentrateur pour la config série des concentrateurs et périphériques concentrés.
      Les paramètres des périfs sont chargés ou non dans la config du serveur selon keep/new.
      La totalité des paramètres de toutes les machines sont définis dans la config du serveur. v1.57 finale de frontal2
v1.a
v1.b  le concentrateur transfère les consignes analogiques et envoit des messages de présence au serveur (apparait dans periTable)
      dev en cours du controle des radiateurs
v1.c  la variable globale beginP_done indique que importData a été effectué dans beginP ; 
      donc inutile de faire radio.txRx( et importData ensuite ; 
v1.d  passage de consigne rad et exécution ; recup periCfg pour bit RAD ; ajout data_mail_ START
      pbs de transmission UDP : il semblerait que le concentrateur ait des difficultés avec W5500 à 30MHz 
      ça peut être un pb hard sur les cartes utilisées (DUE/W5500) les transmissions avec le concentrateur 1 semblent ok
      la spec du nrf dit "max sck freq=10MHz" !!! donc w5500 @16Mhz et nrf @8MHZ 
      (mais W5500 @30Mhz bloque tout ...???)
v2.9  prépa pour paramétrage du hard (LoRa is comming) ; ajout messageCnt ;
v2.a  utilisation lib ethernet2 ; delay après Udp.endPacket() ; reset hard du w5500 ; uRScnt
      à cette version, reboot si Udp ne démarre pas ou si exportDataMail reste sans réponse 
      après les retry définis dans userResetSetup, puis le reset hard du w5500 puis 2 essais
      entre 15/09 et 6/10 17 reboot avec journées de 7,3,2,2,1,1,1,0...0
      Des soucis avec l'écriture lecture de la config après power off sur Due... la carte marquée "TEST_CFG" fonctionne
v2.b  Compatibilté avec LoRa ; les codes erreur sont dans radio_const.h ; 
      dets modif beginP : retry + poweron permanent
      le port udp du server auquel s'adresse le concentrateur provient de configCreate ou de getServerConfig
      Le numéro d'UDP qui s'affiche dans periLine correspond à l'instance UDP qui a reçu le message (selon le port de la config du concentrateur)
v2.c  La structure du message vers le périf change pour faire de la place au temps absolu des cellules (voir radio_user_peri/conc)
      mise en place cellules temporelles
*/

/************* config ****************/

//#define NOCONFSER
  
  //#define MACHINE 'P'            //  P périphérique
  //#define MACHINE 'C'            //  C concentrateur  
/* !!!!!! changer de platformio.ini selon la MACHINE ('C'=due ou stm32 ; 'P' =328 !!!!! */

  #define TXRX_MODE 'U'           // TCP / UDP

  #define CB_ADDR   (byte*)"shcc0"      // adresse fixe de broadcast concentrateurs (recup MAC_ADDR concentrateurs) 0 nécessaire pour read()
  #define BR_ADDR   (byte*)"bcast"      // adresse fixe de broadcast

  #ifdef NRF
  #define RADIO_ADDR_LENGTH NRF_ADDR_LENGTH           // doit être == NRF_ADDR_LENGTH --- voir shconst
  #define MAX_PAYLOAD_LENGTH NRF_MAX_PAYLOAD_LENGTH   // doit être == NRF_MAX_PAYLOAD_LENGTH
  #define RADIO_TFR_DLY 4                             // mS délai entree tx conc et import sur perif après rx  (mesuré au scope)
  #endif

  #define ABSTIME_STEP 6                  // 6 bits par caractère
  #define ABSMASK 0x3f
  #define NBCELLPOWER 2                   
  #define NBCELLS 0x4                     // nre de cellules temporelles  !!!!puissance de 2!!!!
  #define CELLDURPOWER 7
  #define CELLDUR 0x80                    // durée cellule                !!!!puissance de 2!!!!
  #define ABSTIMEPOWER (NBCELLPOWER+CELLDURPOWER)
  #define ABSTIME (NBCELLS*CELLDUR)       // millis cells size

  #define DLYSTP (int)32

  #define NBPERIF 12                      // dim table
  #define BUF_SERVER_LENGTH LBUFSERVER    // to/from server buffer length

#if MACHINE_CONCENTRATEUR

  #define MARKER    A11

  #define MISO_PIN   12
  #define MOSI_PIN   11
         
  #define CE_PIN     9          // pin pour CE du nrf
  #define CSN_PIN    8          // pin pour CS du SPI-nrf
  #define PORT_PP    7          // pin pour pulse de debug analyseur logique (macro PP4)
  //#define REDV1                 // modèle carte red

  #define LRAMREM 16

#endif // MACHINE_CONCENTRATEUR

#if MACHINE_DET328             /* voltage and temp acquisition params */
   // param carte DETS (sinon ?) dans platformo.ini
  #define ATMEGA328               // option ATMEGA8 ... manque de memoire programme (8K dispo et nécessite 17K)

  #define MARKER     5

  #define CLK_PIN    13                 
  #define MISO_PIN   12
  #define MOSI_PIN   11

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
#endif // DETS
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

#endif // MACHINE_DET328  

#if MACHINE_CONCENTRATEUR
  //#define DUE                   // DUE OU STM32... provient de platformio.ini  

  #define MODEL "REDV2_"

  #define WDTRIG trigwd();blktime=millis();
  
  #define PLED        PINLED     // 3 sur proto 

/* table des périphériques radio */  

  // extDataStore() output status
  #define EDS_STAT_OK 1
  #define EDS_STAT_PER 2
  #define EDS_STAT_LEN 3 

  struct ConTable
  {
    uint8_t numPeri;                        // numéro périphérique dans la table du serveur
    byte    periMac[RADIO_ADDR_LENGTH+2];   // macAddr (ajout 1 car=num entrée de la table pour former une addr mac pour l'extérieur)
    char    servBuf[MAX_PAYLOAD_LENGTH+1];
    uint8_t servBufLength;
    bool    servBufSent;
    char    periBuf[MAX_PAYLOAD_LENGTH+1];
    uint8_t periBufLength;
    bool    periBufSent;  
  };

  #define SBVINIT "00040000400025000000000001"  // server buffer init value
  #define SBLINIT 26                            // server buffer init length (MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1)
         
#endif // MACHINE_CONCENTRATEUR



/*** return (error) codes ***/
#define AV_NBPIP -1
#define AV_LMERR -2
#define AV_MCADD -3 // read output (need message upload)
#define AV_EMPTY -4
#define AV_MAXAV AV_EMPTY
#define ER_MAXRT (AV_MAXAV)-1 // code erreur MAX_RT
#define ER_RDYTO (ER_MAXRT)-1 // code erreur Time Out attente réception
#define ER_CRC   (ER_RDYTO)-1
#define ER_OVF   (ER_CRC)-1
#define ER_EMPTY (ER_OVF)-1
#define ER_MAXER (ER_EMPTY)

#define ER_TEXT "ey\0ov\0cc\0to\0rt\0em\0mc\0le\0pi\0--\0ok\0" // 2 char libs for codes

#endif // _RADIO_CONST_INCLUDED
