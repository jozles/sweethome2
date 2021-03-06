#ifndef CONST_H_INCLUDED
#define CONST_H_INCLUDED

#define VERSION "1.r_"
/* 1.1 allumage/extinction modem
 * 1.2 ajout voltage (n.nn) dans message ; modif unpackMac
 * 1.3 deep sleep (PERTEMP) ; gestion EEPROM ; conversion temp pendant sleep
 * 1.4 IP fixe ; system.option(4 )modem off ; RTCmem
 * 1.5 correction calcul temp (ds1820.cpp) ; 
 *     tempage remplacé par dateon pour mémo durée connexion (durée totale env 1,25 durée jusqu'à datasave)
 * 1.6 IP en ram RTC produit par le router à la mise sous tension 
 *     en cas de raz de numperiph, raz IP pour produire nouvelle demande au routeur.
 * 1.7 talkServer renvoie 0 ou NBRETRY si pas de connexion.
 *     prochaine lecture température (donc tentative cx) dans 2 heures + dataread
 *     installation ssid2/password2
 * 1.8 ajout PINSWA et B et PINDET + leur transfert dans dataread et datasave
 *     ajout compilation non _DS_MODE ; 
 *     ajout led sur pin 0 ;
 *     option S DS1820 (ds1820.cpp) ; 
 *     Mode server avec décodage, comptage, action et réponse pour messages OPEN et CLOSE
 *     gestion actionneur manuel sur interruptions
 *     longueur message en HEXA ASCII après GET / pour comm avec clients
 * 1.9 normalisation du format de communication
 *     automate pour talkServer
 * 1.a paramètres pour switchs, pulse etc ; 3 modes pour l'alim : DS PO NO    
 *     config hardware selon carte.
 *     réception commandes (ordreExt) opérationnelle (cdes testa_on__ testb_on__ testaoff__ testboff__)
 *     pilotage des switchs via on/off du serveur opérationnelle.
 * 1.b ajout des valeurs courantes de pulse et image des detecteurs dans cstRec
 * 1.d horloge séparée, détecteurs locaux/externes, variables d'état pulses, model, cstRec et read/write/initConstant revus
 * 1.e moteur pulse, détecteurs en poling, transfert modèle DS18x00
 *     pulse fonctionnel
 * 1.f forçage communication au démarrage suivant si alim bloquée allumée (ne fonctionne que pour PO_MODE) ; 
 *     gestion tconversion selon modèle DS18X ; 
 *     révision connexion wifi : talkServerWifiConnect() et wifiConnexion
 *     révision timings : ajout tempTime, cstRec.tempPer, fonctions trigtemp(startTo), chkTrigtemp(ctlTo), forceTrigTemp
 *     ajout utilisation params descde et actcde : onCde devient onCdeO (Off prioritaire) et offCde offCdeO, 
 *                                                 actCde onCdeI (On prioritaire) et desCde offCdeI
 *                                                 swAction corrigé
 *     extinction des PO_MODE et DS_MODE si alim insuffisante
 * 1.g ds18x00 passe en lib
 * 1.h ?
 * 1.j révision de la gestion des variables permanentes pour PO_MODE qui réinitialise à chaque démarrage
 *     l'enregistrement des variables permanentes commence sur l'adresse de cstlen et non plus sur la structure cstRec 
 *     (bugs possibles dans les autres modes)
 *     correction conversion/delai conversion 200mS pour 0.25°
 * 1.k correction retry cx wifi qui ne retournait pas le dépassement de nombre d'essais 
 *     lorsque la cx wifi a échoué tempo de x heures (PERSERVKO), le délai de conversion si PO, la lecture et lé controle de la température ne sont plus faits. 
 *     néanmoins en PO, le temps de démarrage (avant setup) est tel (350mS) que la conso reste très élévée (70mA pendant 350mS toutes les 165sec soit 150uAh - 4mAh par jour)
 * 1.m corrections durée conversion DS18X20 ; correction du codage des actions dans la boucle des règles (actions()/dynam.cpp)   
 * 1.n transfert lecture analogique et réception seuils ; à faire : lecture de l'entrée analogique.
 *     correction gestion connexion wifi (suppresseion talkServerWifiConnect() : les retrys sont dans wifiConnexion, fonction du temps WIFI_TO_CONNEXION )
 * 1.p réception ordreExt() <5mS suppression de l'attente de déco avec TO de 2mS sur cli.available()
 *     lecture valeur analogique et tfr en NO_MODE (les autres modes utilisent l'ADC pour l'alim)
 * 1.q les règles deviennent de vraies opérations logiques entre la valeur courante et la valeur source
 *     création des fonctions -0- et -1- pour forcer une valeur initiale
 *     les conditions appliquées à la source sont statique/flanc, montant/descendant si flanc ou direct/inversé si statique
 *     affichage des diags optionnels pour 'NO_MODE'
 *     dataSave après chaque réception via ordrext (cstRec.talkStep=6 et cstRec.serverTime=0 sinon blocage)
 * 1.r en mode NO_MODE pas de PERSERVKO pour ne pas risquer de déclencher le watchdog TCP/UDP du server     
 *     les fonctions de test sont opérationnelles ; incorporation des mails
 * 
Modifier : 

  en deepsleep 10uA+1uA ds18x20 = 11uA de consommation de fond ; 
  une mesure de 2mS*80mA toutes les 2 minutes (conversion pendant sleep)=1,3uA
  une transmission 50 fois par jour (@6sec connexion)=650uA ; tester si la connexion est plus rapide en deepsleep

  généraliser l'automate de séquencement pour la totalité du périphérique : aucune attente ni délai ; création d'une fonction de reprise qui
  stocke l'adresse de la fonction suite du traitement en cours et le délai et time out d'attente pour l'executet.
  
  (étudier le traitement/vidage d'une réception non traitée en mode serveur avant d'effectuer une commande read/save)
  
  gestion des alarmes avec message au serveur (tension, niveaux th et autres) ?
  
  Lorsque l'alim est bloquée allumée, enchainer sur une communication série : 
    toutes les 2 sec envoi RDY et attente réponse au format des messages http (avec longueur,fonctions,params et crc)
    sur le serveur : vidage réception série, attente 1 seconde, si vide attente RDY puis envoi, sinon vider et recommencer
  
  commande à créer qui utilise la config du serveur :
  SSID1nnnnn...,SSID2nnnn....,PWD1nnnn....,PWD2nnnn....,SERVIPPxxx.xxx.xxx.xxx/nnnn, (16 car pour SSID et 48 car pour PWD) à stocker séparément des constantes)
  créer un protocole usb pour charger ssid, password, IP et port du host sans reprogrammer le 8266  
  
*/

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <shconst2.h>

#define PERIF       // pour compatibilité avec shconst etc

/* Modes de fonctionnement  */
/*
   2 cycles emboités :
   
      1) timing de base = période de lecture de la sonde thermomètre ; 
            si DS_MODE ou PO_MODE c'est la période d'allumage ,
            sinon PERTEMP est la période par défaut en Sec (PERTEMP doit être chargée avec la période d'allumage si timer externe)
            cstRec.tempPer la période courante chargée depuis le serveur (directement utilisée dans le timer si deep sleep),
            tempPériod la période locale pour permettre forçage et retrig 
            et cstRec.tempTime le point de départ en millis() si loop.
            Donc, si millis()>temptime+tempPer lecture thermo et +tempPer à serverTime (ci-après)
            Si la valeur lue a plus de cstRec.pitch d'écart avec la valeur précédente -> accés serveur
      2) fréquence d'accés au serveur pour enregistrer une/des valeurs de sondes/détecteurs
            comptage du timing de base selon le mode d'alimentation lors du check température
            PERSERV est la période par défaut en secondes
            cstRec.serverPer la période courante en secondes, chargée depuis le serveur (set/ack) 
              prend la valeur PERSERVKO lorsqu'il n'y a pas de connexion WIFI afin de limiter les tentatives de cx pour économiser les batteries
              prend la valeur PERSERV si la connexion au wifi fonctionne (sera rechargée par le serveur lors de l'accès suivant)
            et cstREc.serverTime le compteur de "timing de base" (remis à 0 à chaque déclenchement)
            Donc, si serverTime>serverPer accès server (talkServer)
            
            Gestion totale dans readTemp().

    Stockage des constantes :

        en mode Power Off (PO_MODE) les variables sont stockées en EEPROM 
        dans les autres modes mémoire RTC 
        
        Dans tous les cas :
            la première variable est la longueur totale de la structure (1 byte len maxi 256)
            les 3 caractères suivants inutilisés
            les 4 caractères suivants la version qui a alimenté la structure
            la dernière variable (1 caractère) le crc de ce qui le précède
           
            Les accés se font via 4 fonctions : 
                readConstant() (ctle du crc), writeConstant() (géné crc), initConstant() et printConstant

     Pulse :

            

     Detecteurs physiques :

            MAXDET nombre maxi dans shconst.h
            NBDET  nombre de la carte courante dans les params de la carte dans const.h
            cstRec.memDetec[MAXDET] 1 byte par détecteur setup ds isr et initIntPin
               DETBITLH 1 bit état HIGH LOW
               DETBITST 2 bits état (dclenché(TRIG)/attente(WAIT)/int masquée(IDLE)/disable(DIS)) si DIS le bit d'état est invalide
                                                                  passage du mode WAIT   à TRIG après déclenchement dans isr
                                                                  passage de DIS ou IDLE à WAIT à l'armement dans initIntPin 
                                                                  passage du mode TRIG   à IDLE lorsque les traitements de flanc sont effectués
                                                                  en mode IDLE le bit d'état est mis à jour par la loop
                                                                  à la mise sous tension mode DIS


      Détecteurs logiques : tableau 


      Actions :

            2 actions possibles sur les switchs : ON/OFF 
            chaque action peut être déclenchées par 3 sources : un des détecteurs du tableau 


*/
/* >>> MODES d'ALIM <<< */

#define DS_MODE 'D' // deep Sleep     (pas de loop ; pas de server ; ESP12 reset-GPIO6 connected)
#define PO_MODE 'P' // power off Mode (pas de loop ; pas de server ; ESP01 GPIO3/RX-Done connected)
#define NO_MODE 'N' // No stop mode   (loop et server)

/* >>> CARTES <<< */

#define VR      'V'  // 2 triacs
#define VRR     'W'  // 2 relais
#define VRDEV   'D'  // carte dev avec 2 leds 
//#define RELAY   
#define THESP01 '1'
#define THESP12 '2'
/********************************** 3 config à faire ********************************/
//                                  1 -- modèle de carte
//                                  2 -- type alimentation (POWER_MODE)
//                                  3 -- adresse IP serveur + port serveur + l'ordre des SSID  (peripherique.ino)
//                                 
//                                 enlever le cable série pour que ça marche sur THESP01
//                                 updater la condition de pinMode dansle setup en cas de nouvelle carte
#define CARTE VRDEV             // <------------- modèle carte
#define POWER_MODE NO_MODE      // <------------- type d'alimentation 
//#define PININT_MODE             // <------------- avec/sans pin d'interruption

/* ds18x20 */
#define MODEL_S 0x10
#define MODEL_B 0x28

#define TCONVERSIONB       300    // millis délai conversion temp 187mS 10 bits accu 0,25°
#define TCONVERSIONS       800    // millis délai conversion temp
#define T12BITS            0x7F   // 12 bits 750mS 0.0625°
#define T11BITS            0x5F   // 11 bits 375mS 0.125°
#define T10BITS            0x3F   // 10 bits 187,5mS 0.25°
#define T9BITS             0x1F   // 9 bits 93,75mS 0.5°

// stockage deep sleep / power off  (EEPROM pour PO_MODE seul )

#define EEPROMSAVED 'E'
#define RTCSAVED    'R'

#if POWER_MODE==NO_MODE
  #define _SERVER_MODE          /* Mode server */
  #define TBITS T12BITS       // résolution DSX20
  #define CONSTANT RTCSAVED
  #define CONSTANTADDR 64     // adresse des constantes dans la mémoire RTC (mots 4 octets = 256)
#endif PM==NO_MODE

#if POWER_MODE==PO_MODE
  #define TBITS T10BITS       // résolution DSX20
  #define CONSTANT EEPROMSAVED
  #define CONSTANTADDR 0      // adresse des constantes dans l'EEPROM
#endif PM==PO_MODE

#if POWER_MODE==DS_MODE
  #define TBITS T10BITS       // résolution DSX20
  #define CONSTANT RTCSAVED
  #define CONSTANTADDR 64     // adresse des constantes dans la mémoire RTC (mots 4 octets = 256)
#endif PM==DS_MODE

// matériel

#if CARTE==VR

#ifndef RELAY
#define PINXB 5
#define PINXDT 13
#endif ndef RELAY

#ifdef  RELAY
#define PINXB 13
#define PINXDT 5
#endif  def RELAY

#define WPIN   4        // 1 wire ds1820
#define NBSW   2        // nbre switchs
#define PINSWA 2        // pin sortie switch A
#define CLOSA  LOW      // triac ON
#define OPENA  HIGH     // triac off
#define PINSWB PINXB    // pin sortie switch B (interrupteur)
#define CLOSB  HIGH     // triac ON sortie haute
#define OPENB  LOW      // triac OFF
#define PINSWC PINSWB   // pin sortie switch C
#define CLOSC  CLOSA    
#define OPENC  OPENA
#define PINSWD 2        // pin sortie switch D
#define CLOSD  CLOSB    
#define OPEND  OPENB
#define NBDET  4
#define PINDTA 12       // pin entrée détect bit 0 
#define PINDTB 14       // pin entrée détect bit 1 
#define PINDTC PINXDT   // pin entrée détect bit 2  sur carte VR 3 entrées donc bit 2 et 3
#define PINDTD PINXDT   // pin entrée détect bit 3  sur la même entrée.
#define PININTA 12      // in interupt
#define PININTB 14      // in interupt
#define PININTC PINXDT  // in interupt
#define MEMDINIT 0x1111 // bits enable
#define PINPOFF 3       // power off TPL5111 (RX ESP01)
#define PERTEMP 20      // secondes période par défaut lecture temp (en PO_MODE fixé par la résistance du 511x)
#endif CARTE==VR

#if CARTE==VRR

#define PINXDT 13
#define WPIN   2        // 1 wire ds1820
#define NBSW   2        // nbre switchs
#define PINSWA 4        // pin sortie switch A
#define CLOSA  HIGH     // relais ON
#define OPENA  LOW      // relais off
#define PINSWB 5        // pin sortie switch B
#define CLOSB  HIGH     // relais ON sortie haute
#define OPENB  LOW      // relais OFF
#define PINSWC PINSWA   // pin sortie switch C
#define CLOSC  CLOSA    
#define OPENC  OPENA
#define PINSWD PINSWB   // pin sortie switch D
#define CLOSD  CLOSB    
#define OPEND  OPENB
#define NBDET  4
#define PINDTA 12       // pin entrée détect bit 0 
#define PINDTB 14       // pin entrée détect bit 1 
#define PINDTC PINXDT   // pin entrée détect bit 2  sur carte VR 3 entrées donc bit 2 et 3
#define PINDTD PINXDT   // pin entrée détect bit 3  sur la même entrée.
//#define PININTA 12      // in interupt
//#define PININTB 14      // in interupt
//#define PININTC PINXDT  // in interupt
#define MEMDINIT 0x1111 // bits enable
//#define PINPOFF 3       // power off TPL5111 (RX ESP01)
#define PERTEMP 20      // secondes période par défaut lecture temp (en PO_MODE fixé par la résistance du 511x)
#endif CARTE==VRR


#if CARTE==THESP01
#define WPIN   2        // ESP01=GPIO2 ; ESP12=GPIO4 ... 1 wire ds1820
#define NBSW   0        // nbre switchs
#define PINSWA 5        // pin sortie switch A
#define CLOSA  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENA  0
#define PINSWB 5        // pin sortie switch B
#define CLOSB  1        // valeur pour fermer (ouvert=!CLOSB)
#define OPENB  0
#define PINSWC 5        // pin sortie switch C
#define CLOSC  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENC  0
#define PINSWD 5        // pin sortie switch D
#define CLOSD  1        // valeur pour fermer (ouvert=!CLOSB)
#define OPEND  0
#define NBDET  0
#define PINDTA 5        // pin entrée détect bit 0 
#define PINDTB 5        // pin entrée détect bit 1 
#define PINDTC 5        // pin entrée détect bit 2  sur carte VR 3 entrées donc bit 2 et 3
#define PINDTD 5        // pin entrée détect bit 3  sur la même entrée.
#define PININTA 5       // in interupt
#define PININTB 5       // in interupt
#define MEMDINIT 0x0000 // bits enable memDetec
#define PINPOFF 3       // power off TPL5111 (RX ESP01)
#define PERTEMP 165     // secondes période par défaut lecture temp (en PO_MODE fixé par la résistance du 511x)
#endif CARTE==THESP01

#if CARTE==THESP12
#define WPIN   4        // ESP01=GPIO2 ; ESP12=GPIO4 ... 1 wire ds1820
#define NBSW   2        // nbre switchs
#define PINSWA 5        // pin sortie switch A
#define CLOSA  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENA  0
#define PINSWB 2        // pin sortie switch B
#define CLOSB  0        // valeur pour fermer (ouvert=!CLOSB)
#define OPENB  1
#define PINSWC 5        // pin sortie switch C
#define CLOSC  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENC  0
#define PINSWD 2        // pin sortie switch D
#define CLOSD  0        // valeur pour fermer (ouvert=!CLOSB)
#define OPEND  1
#define NBDET  4
#define PINDTA 12       // pin entrée détect bit 0 
#define PINDTB 14       // pin entrée détect bit 1 
#define PINDTC 13       // pin entrée détect bit 2  sur carte VR 3 entrées donc bit 2 et 3
#define PINDTD 13       // pin entrée détect bit 3  sur la même entrée.
#define PININTA 12      // in interupt
#define PININTB 14      // in interupt
#define PINPOFF 3       // power off TPL5111 (RX ESP01)
#define PERTEMP 60      // secondes période par défaut lecture temp (en PO_MODE fixé par la résistance du 511x)
#endif CARTE==THESP12

#if CARTE==VRDEV
#define MAIL_SENDER
#define WPIN   4        // ESP01=GPIO2 ; ESP12=GPIO4 ... 1 wire ds1820
#define NBSW   2        // nbre switchs
#define PINSWA 5        // pin sortie switch A
#define CLOSA  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENA  0
#define PINSWB 2        // pin sortie switch B
#define CLOSB  1        // valeur pour fermer (ouvert=!CLOSB)
#define OPENB  0
#define PINSWC 5        // pin sortie switch C
#define CLOSC  1        // valeur pour fermer (ouvert=!CLOSA)
#define OPENC  0
#define PINSWD 2        // pin sortie switch D
#define CLOSD  1        // valeur pour fermer (ouvert=!CLOSB)
#define OPEND  0
#define NBDET  4
#define PINDTA 12       // pin entrée détect bit 0 
#define PINDTB 14       // pin entrée détect bit 1 
#define PINDTC 13       // pin entrée détect bit 2  sur carte VR 3 entrées donc bit 2 et 3
#define PINDTD 13       // pin entrée détect bit 3  sur la même entrée.
#define PININTA 12      // in interupt
#define PININTB 14      // in interupt
#define PINPOFF 3       // power off TPL5111 (RX ESP01)
#define PERTEMP 60      // secondes période par défaut lecture temp (en PO_MODE fixé par la résistance du 511x)
#endif CARTE==VRDEV


// timings

#define PERSERVKO 7200/PERTEMP    // secondes période par défaut accès serveur si connexion server ko
#define PERSERV   120/PERTEMP     // secondes période max entre 2 accès server (modifié par le serveur dès la prmeière connexion)g

#define TOINCHCLI         4000    // msec max attente car server
#define WIFI_TO_CONNEXION 8000    // msec
#define WIFINBRETRY          2    // wifiConnexion
#define TSERIALBEGIN       100
#define TDEBOUNCE           50    // msec
#define PERFASTCLK           5    // millis période automate rapide (ordreExt, talkServer, debounce, etc) 
#define DETIMP            1000    // millis trig cde impulsionnelle

typedef struct {
  uint8_t   cstlen;               //  1   doit être la 1ère variable (adresse utilisée pour le calcul de longueur)
  byte      swCde;                //  1   2 bits par sw cde/état (*periSwVal) bits 8(sw4), 6(sw3), 4(sw2), 2(sw1)
  char      cstVers[LENVERSION];  //  4  
  char      cstModel[LENMODEL];   //  6
  char      numPeriph[2];         //  2
  uint16_t  serverTime;           //  2   (sec) temps écoulé depuis la dernière cx au serveur
  uint16_t  serverPer;            //  2   période (sec) de cx au serveur (PERSERVKO si pas de connexion Wifi)
  uint16_t  tempPer;              //  2   période (sec) de mesure thermomètre (PERTEMP)
  uint8_t   tempPitch;            //  1   seuil de variation de la temp pour cx au serveur
  int16_t   oldtemp;              //  2
  uint8_t   talkStep;             //  1   pointeur pour l'automate talkServer()
  uint32_t  durPulseOne[NBPULSE]; // 16   durée pulse 1
  uint32_t  durPulseTwo[NBPULSE]; // 16   durée pulse 2
  uint32_t  cntPulseOne[NBPULSE]; // 16   temps debut pulse 1
  uint32_t  cntPulseTwo[NBPULSE]; // 16   temps debut pulse 2
  byte      pulseMode[PCTLLEN];   //  2   ctle pulse   
  byte      perInput[NBPERINPUT*PERINPLEN]; // 96  configuration (24*4)
  byte      memDetec[MAXDET];     //  4   image mem des détecteurs physiques (1 byte par détecteur)   
  uint32_t  extDetec;             //  4   1 bit par detecteur externe
  IPAddress IpLocal;              //  4
  uint16_t  analVal;              //  2   dernière valeur analogique lue
  uint16_t  analLow;              //  2   seuil analogique low
  uint16_t  analHigh;             //  2   seuil analogique high
  uint16_t  dispo;                //  2                               ---------- inutilisé  
  uint16_t  portServer;           //  2   port en mode serveur          
#define LENFILLERCST 31
  byte      filler[LENFILLERCST]; //  
  uint8_t   cstcrc;               //  1   doit toujours être le dernier : utilisé pour calculer la longueur
             // total 238 = 60 mots ; 256 maxi pour RTC
} constantValues;

#define STEPDATASAVE 6            // code pour talkstep de dataSave()

#define LENRTC 250



/*enum rst_reason {
 REASON_DEFAULT_RST      = 0,   // normal startup by power on 
 REASON_WDT_RST          = 1,   // hardware watch dog reset 
 REASON_EXCEPTION_RST    = 2,   // exception reset, GPIO status won't change 
 REASON_SOFT_WDT_RST     = 3,   // software watch dog reset, GPIO status won't change 
 REASON_SOFT_RESTART     = 4,   // software restart ,system_restart , GPIO status won't change 
 REASON_DEEP_SLEEP_AWAKE = 5,   // wake up from deep-sleep 
 REASON_EXT_SYS_RST      = 6    // external system reset 
};
*/

#endif // CONST_H_INCLUDED
