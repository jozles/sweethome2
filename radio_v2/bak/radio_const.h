#ifndef _RADIO_CONST_INCLUDED
#define _RADIO_CONST_INCLUDED

#define VERSION "2.b "
#define LENVERSION 4

#include <Arduino.h>
#include <shconst2.h>

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
v1.b  le concentrateur transfère les consignes analogiques et envoie des messages de présence au serveur (apparait dans periTable)
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
v2.b  Pour la compatibilté avec LoRa les codes erreur sont dans radio_const.h
      Structure packets :
        Nrf   l'adresse de destination est stockée dans les données de gestion du block
        LoRa  l'adresse de destination est stockée dans les données du packet

*/

/************* config ****************/

//#define NOCONFSER
  
  //#define MACHINE 'P'            //  P périphérique
  //#define MACHINE 'C'            //  C concentrateur  
/* !!!!!! changer de platformio.ini selon la MACHINE ('C'=due ou stm32 ; 'P' =328 !!!!! */

  #define TXRX_MODE 'U'           // TCP / UDP

  #define CB_ADDR   (byte*)"shcc0"      // adresse fixe de broadcast concentrateurs (recup MAC_ADDR concentrateurs) 0 nécessaire pour read()
  #define BR_ADDR   (byte*)"bcast"      // adresse fixe de broadcast

  #define CLK_PIN    13
  #define MISO_PIN   12
  #define MOSI_PIN   11

  #define NBPERIF 12                      // dim table
  #define BUF_SERVER_LENGTH LBUFSERVER    // to/from server buffer length

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

  #define SBVINIT "00072_00024_0025_00000000"  // server buffer init value
  #define SBLINIT 25                           // server buffer init length (MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1)
         
#endif // MACHINE == 'C'

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
#define ER_MAXER (ER_OVF)
#define ER_TEXT "ov\0cc\0to\0rt\0em\0mc\0le\0pi\0--\0ok\0" // 2 char libs for codes

#endif // _RADIO_CONST_INCLUDED
