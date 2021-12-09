#ifndef _CONST_H_
#define _CONST_H_


#define VERSION "A.58"
/* 1.1 ajout voltage dans données data_read_ ; modif unpackMac
   1.1a ajout volts et version dans table
   1.1b suppression dht ; ajout periDetVal et periSwVal avec affichage/saisie dans la table ; gestion serveur dev
   1.1c ajout periIpAddr avec aff dans la table
   1.1d ajout periSwPulse, periSwNb, periDetSs, periDetOo avec affichage dans la table et tfr aux périphériques
        ajout longueur message après GET / pour com avec serveur périph (les modifs de messages ne perturbent plus le crc)
   1.1e modif gestion des données reçues avec GET ou POST : ajout tableau d'offsets dans valeurs[] (nvalf[]) pour ne plus
        perdre de l'espace avec une longueur de données fixe. Les données sont séparées par '\0';
        Rédaction des "principes" et mise en lib de shconst.h, shutil et messages :
              conversion du soft (version 1.9 pour périphériques).
        Peritable : N°,nom,temp,freq,pitch,nbre int*(etat,cde,timer,nbre det, état), mac addr,ip addr,last in time,last out time
                    ajout bouton "refresh" et config per refr.
        ntp cassé
   1.1f restructuration communications (dataread/save;set;ack)
        shmess reçoit assySet, inpSet (à venir assyRead/Save inpRead/Save)
   1.1g suppression accueil et fonctions associées, mise en place perisend(cliext)
   1.1h pulse opérationnel ; ajout variable (float) periThOffset dans periRec (offset sur température mesurée)
   1.1j correspond à la version 1.c de peripherique
   1.1k ajout alarmes th et volts + champs detecteurs du serveur
        gestion mots de passe et TO révisée (rémanence password via macMaster)
        fichier config sur SD, séparation pages switch dans peritable, multiples révisions des fonctions.
        ajout periPerTemp dans fichiers periphériques pour paramétrage période temp via péritable
        ajout datecmp(),fichier remote et fonctions associées cfgremote,load,save,print;sliderhtml()

   1.2  utilhtml regroupe les fonctions utiles aux pages html
        ajout serveur pilote (pilotserv()) et traitement dans pilotserver()       -> client cli_b
        l'acquisition/traitement des fonctions de periserv() devient periserver() -> client cli_a
        remoteHtml() traitement des requêtes sur pilotserv
   1.2a ajout dans fichier config de la table username/usrpass
        commonserver()
        nouveau système password et TO (voir commonserver()) (boutRetour,boutFonction,usrForm)
        ajout periPort et tfr via ack/set ; accés aux périph serveurs débuggé (fermeture connection "cli.stop")
   1.2b ajout timers ; periTableHtml() protège periCur ; periSend valorise periCur + periLoad
   1.3  gestion des switchs :
        32 det serveur  ; remotes sur det serveur
   1.3a pulses indépendants des switchs, disjoncteur en amont de tout
        24 inputs par sw (enable, lev, oldlev, edge/stat, srce (phy/ext/pulse/mem, num), dest (sw/ext/pulse/mem, num), action)
        form à part pour les détecteurs du serveur ds peritable ; 1 bouton "per update"(pertosend)
        bouton refresh des switchs envoie une demande d'état au périphérique
        libellé pour detecteurs serveur ; affichage/saisie lignes péritable disjoint
   1.3b periSend et periParamsHtml disparaissent ... periReq et periAns envoient des messages aux périphériques
        ds3231.h devient une lib ; création de la classe Ds3231 (instance ds3231)
   1.4  réaménagement setup ; udp ;
   1.4a extension thermomètres pour contrôle chaudière/clim ; ajout scanThermos()
        valeurs entières pour températures(x100) dans fichiers peri ; convStrToInt remplace convStrToNum là où c'est possible
        remotes avec 2 detecteurs par ligne
   1.4b correction affichage/saisie de pitch et offset (parenthèses manquantes dans codage en réception et ajout dans numTableHtml du type 'r' : float*100 stocké en int16)
        couleur "teal" pour les date/heure de dernières connexion dans showline si <10% de dépassement
   1.4c raz peri dans periline ; retrig hard watchdog : ledblink(0) dans dumpHisto(), blinks dans setup ; ajout soft watchdog : détection WDDELAY sans connexion
        limitation dumpHisto à 100000 car ;
   1.4d scandate (maj date quotidienne) ; le CS de la SDCard est uniformisé sur le pin 4
        le time out d'attente d'un serveur est ramené à 2 sec pour réduire le déclcht du wd : TOINCHCLI dans waitRefCli( (shmess2)
        fonction trigwd();
   1.4e dumpHisto modifié avec buffer 1000 char pour diminuer le nombre de requètes ; idem showline ; fhisto est ouvert/refermé à chaque usage ;
        ajout fonction dans la bouckle d'attente "stop request pour un arrêt 'propre' ... la led passe en blink régulier rapide 300mS.
        (nécessite un switch sur la pin STOPREQ)
   1.4f mode buffer pour memDetServ() ; ajout variables pour entrée analogique des périphériques (periRec,showLine,periLine)
   1.4g transfert datas analogique (val,low,high) avec périphériques : modif assyset() et periDataRead()
        TOINCHCLI augmenté à 4000 pour laisser le temps de répondre en cas de périReq (annule modif 1.4d) ; trigwd() avant d'appeler getHttpResp()
   1.4h ajout règles d'update des detserv depuis une valeur analogique et depuis les det des périphériques (modif periRec) + fonctions adhoc
   1.4k ajout memdet ref + opération logique sur règles analog + digital
   1.4m ajout source détecteurs, modif memDetServHtml + valorisations depuis periline, SwCtlTable, thermomètres, remotes et timers
   1.5_ sd devient le plus souvent histo dans les noms de paramètres, de variables et de fonctions ;
   1.50 la librairie sdFat remplace sd ; ajout actions pour règles périphériques ; 
        ajout peri et sw dans tables detecteurs des remotes ; branchement disjoncteur et maj croisée remotes<->periline + voyant ; 
        le disjoncteur a besoin d'un dét mémo pour être "réactif" dans les règles (sinon il n'est pas vu par poolPerif)
        les lectures et modifs de periSwVal deviennent les fonctions periSwLev et Cd / Update pour en simplifier l'accès.
        periSave devient LOCAL pour toutes les opérations de communication périf/serveur ; ajouter une sauvegarde horaire du cache
        ajout maxCxWt et maxCxWu (TO pour absence de cx TCP/UDP) dans config  
        ajout buffer pour les données de mail dans periReq + boutons de test dans periLine
        dataTransfer peut recevoir un message amputé ; gros caractères pour accueil ; envoi de mails via periReq -> START/BOOT/TEST/DATE...
        correction sync
   1.51 periSwVal, remotes, memdet sont en miroir ; modifier l'un modifie les 2 autres et met à jour le périphérique
        timers et thermos utilisent periDetecUpdate lors de la mise à jour des memDet ;
        yield() redefini avec trigwd() inside ; integration params mail dans config ; correction sur messages mails ; correction sur record histo ;
        corrections sur users/pwd ; periline et d'autres bufferisés ; ajout forceWd pour mettre la led off et attendre le reset ;
   1.52 alphaTfr() pour saisir les valeurs alpha sans risquer d'overflow ; mailEnable bloque les mails si la config et les péris ne sont pas chargés
        correction de la ligne de reset dans l'histo ; accélération imgHtml() ; affichage durée plusieurs fonctions ;
        favicon devient une fonction insérée dans numfonct par getnv() ; le cli.stop en fin de htmlImg est supprimé 
        remoteN[].enable peut prendre 3(4?) valeurs 0 OFF, 1 conjoncté, 2-3 forcé ON : le memDet qui suit remoteT[].deten est traité comme image du forçage
        màj de toute la chaine de synchro entre memDet,remote et periSw ; ajout d'un 3eme bouton radio dans remoteHtml() ; nombre périfs augmenté à 28 ;
   1.53 après tfr vscpio ; installation buffer json dans peritable.cpp : jsbuf ; traitement periline.
   1.54 instances multiples datées pour TCP avec stop reporté. getCde révisé/corrigé. 
        réponse datasave à set______ -> réaffichages (remote/preitable etc) reportés après periReq. corrections periRemoteUpdate.
   1.55 incorpore metaJS pour periline ; #define NOJSBUF accélère la sortie html ; révision initLed ; ajout red1.1 (powerOn)
        config serverName (confightml) ; création factoryReset() ;
   1.56 Le fichier config devient la source des données réseau et des données de config des périfs et concentrateurs ; 
        Le mécanisme d'initialisation est installé. 
        Les concentrateurs et périphériques demandent leurs données de config au serveur via Serial1 du serveur, Serial1 du concentrateur, Serial des périfs.
   1.57 toutes config séries ok (?) v.19 de single_nrf_v2
   1.58 ajout mode forçage pour periTableSave() / param FORCE ; sortie sur accueil.html si getnv rend 0 params ;
        corrections gestion cache périfs ;

   BUGS :

        periReq : cx qui rate ou périf qui ne reçoit rien comme si cli.write() n'envoyait pas ou que le contenu se perd.
        La connexion ratée sort avec le socket stat "CLOSED" ce qui indique que la tentative du W5500 a échoué ;
        Lorsque le périf est en attente de réponse à un dataRead ou dataSave, il n'écoute pas jusqu'au TO (TOFINCHCLI dans waitRefCli)
        C'est un cas de non-réponse "normal" ; soit attendre soit recommencer plus tard.
        Essai de long TO infructueux... le taux de ratage semble augmenter quand il y a plusieurs periReq dans la meme minute ;
     

   à faire :

      passer à 32 (64?) périfs et 64 (128?) memDet
   
      gérer le periLoad KO dans periTableLoad
      remplacer date14 par date unix sur 8 digits dans les messages ACK et SET

      ajouter nbre rules dans assyset

      détecter les changements à la réception des dataread/save pour effectuer un refresh de l'affichage de peritable

      timers : ajouter option "1 jour sur n" : dhdebcycle=1ère date... calculer si date courante ok (récupérer un bool inutile - cyclic? )
      pulses : option entrée clock depuis input

      dans fenetre switchs ; table rules avec pour chaque ligne boutons poubelle/validation, + (nouv ligne), xmit(pertosend)
                             ajouter visu diag/état des rules(?)

      ajouter crc sur fichiers config,periphériques,remotes,timers,etc

      ajouter détection "android" dans getnv

*/

/* mac,adressage,port
  
  (jusqu'à v1.55 PORTSERVER et PORTPILOT sont les port des serveurs (proviennent de shconst.h) ; "portserver" du fichier config est inutilisé.)
  
  Depuis v1.56 !!! MODIF structure fichier config !!!
  les variables du fichier config mac,serverName, serverPort, remotePort, serverUdpPort sont utilisées. 
  
  La carte W5500 recoit l'adresse mac du fichier config.
  Pour permettre la redirection de port et l'accès au serveur depuis les périphériques ou les navigateurs,
  l'adresse IP locale du serveur doit être fixe. 
  L'adresse IP locale est fournie par Ethernet.localIp() à chaque Ethernet.begin() 
  puis chargée dans localIp du fichier config pour assurer que l'adresse fournie aux périphériques soit valide.

  pour initialiser un serveur :
     sur le serveur, faire un "factory reset" qui modifie le fichier config : 
          (factory reset : appuyer RESET, appuyer HALT, relacher RESET, après environ 2 secondes
          la led jaune blink 1/0,5sec lâcher HALT : blink 2/seconde ; faire RESET)
          efface localIP, serverPort=55550, remotePort=55551, serverUdpPort=55552, mac=54.55.55.55.55.55, admin/admin pour l'accès
          initialise la table des concentrateurs mac ashco\0,b,c,d ; IP 0 ; ports 55556,7,8,9 channel 120/110/100/90 RfSpeed 2 
          (si un équipement local utilise un de ces port ou cette adresse mac, l'éteindre le temps de l'installation)
     sur le routeur, dans la liste des baux actifs du routeur on trouve l'adresse MAC 55.55... associée avec l'Ip fournie par le DHCP
          créer une redirection de port sur cette adresse IP ce qui permet d'accéder au serveur (adresseIP:port)   
     (éventuellement, sur le serveur, changer l'adresse MAC et/ou les ports et le redémarrer ... sur le routeur, le serveur est visible avec la bonne adresse MAC)
          associer une adresse IP fixe à cette adresse MAC (éventuellement l'adresse courante)
          créer les redirections des ports définis sur le serveur sur cette adresse IP 
          (si l'adresse mac a été changée, supprimer la redirection sur l'adresse mac 90.90...)
     redémarrer le serveur
*/


#define _MODE_DEVT    // change l'adresse Mac de la carte IP, l'adresse IP (via DHCP de la box) et le port (en accord avec redir de port de la box)

// valeurs pour factoryReset

#define DEFMACADDR "\x54\x55\x55\x55\x55\x55"   // def server mac addr
#define DEFSERVERPORT 55550                     // base ports server (+1 remote, +2Udp)
#define LNSERV  17

#ifdef _MODE_DEVT
#define MODE_EXEC "DEVT"
//#define LOCALSERVERIP {192,168,0,35}                   //adresse IP    ---- 34 service, 35 devt
//#define PORTSERVER 1790                       // 1790
//#define PORTPILOT  1792                       // 1792
//#define PORTUDP    8890                       // 8890
#define DEFNOMSERV "sweet_hdev\0"
#endif // _MODE_DEVT

#ifdef _MODE_RUN
#define MODE_EXEC "RUN"
//#define LOCALSERVERIP {192,168,0,36}                   //adresse IP    ---- 34 service, 35 devt
//#define PORTSERVER 1786                       // 1786
//#define PORTPILOT  1788                       // 1788
//#define PORTUDP    8886                       // 8886
#define DEFNOMSERV "sweet_home\0"
#endif // _MODE_RUN

#define UDPUSAGE

#define DS3231_I2C_ADDRESS 0x68              // adresse 3231 sur bus I2C

#define NBPERIF 28
#define PERINAMLEN 16+1                      // longueur nom perif
#define PERIRECLEN 310 // V1.4k              // longueur record périph

#define CONFIGRECLEN 993                     // longueur record config 

#define LBUF1000 1000
#define LBUF2000 2048
#define LBUF4000 4096                        // buf size html print 

#define LBEC 1000                            // longueur buffer pour export message config

#define MAXCXWT 120000                       // (defaut) time out delay if no TCP connection 
#define MAXCXWU 900000                       // (defaut) time out delay if no UDP connection 

#define LDATEA 17                            // len date alpha

#define TO_PASSWORD 600                      // (defaut) sec toPassword

#define MAXIMGLEN   4000                     // longueur maxi pour fichiers image

#define MAXSERVACCESS 120                    // (defaut) (sec) période maximale accés au serveur
#define TEMPERPREF   20                      // (defaut) (sec) période check température sur périphérique
#define DEFPITCH     25                      // (defaut) pitch pour periInitVar
#define MINTHCHGE    0.25                    // (defaut) changement minimal th pour save

#define SDOK 1
#define SDKO 0

#define CACHEISFILE 1
#define FORCE true
#define PERILOAD true

#define LENREMNAM 16    // remote name length
#define NBREMOTE 16     // remote entrys
#define MAXREMLI 16     // sw items total
#define REMOTENFNAME "noms_rem"
#define REMOTETFNAME "tablerem"
#define PERILINE    1     // une modif de periline a eu lieu -> update remotes et memDet 
#define MEMDET      2     // une modif de memDet a eu lieu   -> update remotes et perif 
#define REM_ENABLE  1     // une modif de remote a eu lieu sur enable -> maj memDet correspondant
#define REM_DETEC   2     // une modif de remote a eu lieu sur detec  -> maj memDet correspondant


struct SwRemote           // liste des détecteurs modifiables par les remotes
{                         // une remote peut agir sur plusieurs détecteurs on/off ou enable 
                          // elle peut donc exister plusieurs fois dans la table
                          // le détecteur 0 sert "d'élément neutre" et est donc inutilisable comme détecteur
                          
  uint8_t  num;           // remote number (numéro dans table des noms)
  uint8_t  detec;         // detecteur on/off
  uint8_t  deten;         // detecteur enable
  bool     enable;        // remote enable
  uint8_t  peri;          // périphérique dont un disjoncteur est sous controle de enable (0 pas de périphérique)
  uint8_t  sw;            // sw concerné du périphérique 
};

struct Remote             // liste des remotes
{
  char    nam[LENREMNAM]; // remote name
  uint8_t onoff;          // état on/off
  uint8_t newonoff;       // buffer pour reception et traitement cb par GET /
  uint8_t enable;         // état enable (recopié dans les disjoncteurs des switch/périphériques concernés)
  uint8_t newenable;      // buffer pour reception et traitement cb par GET /

};

#define NBTIMERS     8
#define LENTIMNAM    16
#define TIMERSNFNAME "NOMS_TIM"
#define NBCBTIM      4   // nbre check box (hors dw)   

struct Timers
{
  uint8_t detec;         // numéro détecteur associé --- plusieurs timers possibles pour un détecteur ; voir le forçage
  char    nom[LENTIMNAM];// nom
  char    hdeb[7];       // heure début
  char    hfin[7];       // heure fin
  bool    perm;          // permanent (pas de test sur deb/fin cycle ni cyclic)
  bool    cyclic;        // cyclique/one time (si one time disable en fin de cycle)
  bool    enable;        //
  bool    curstate;      // etat courant du timer
  bool    forceonoff;    // inutilisé devrait être "action" (OR/NOR/XOR/AND)
  byte    dw;            // jours semaine xyyyyyyyy ; x si tout
  char    dhdebcycle[16];
  char    dhfincycle[16];
};

#define NBTHERMOS  NBPERIF
#define LENTHNAME 16
#define THERMOSFNAME "THERMOS_"

struct Thermo
{
  char      nom[LENTHNAME + 1]; // nom
  uint8_t   peri;               // periph
  bool      lowenable;          // low level enable
  bool      highenable;         // high level enable
  bool      lowstate;           // low level enable
  bool      highstate;          // high level enable
  uint16_t  lowvalue;           // low level value
  uint16_t  highvalue;          // high level value
  uint16_t  lowoffset;          // low level offset
  uint16_t  highoffset;         // high level offset
  uint16_t  lowdetec;           // low level value
  uint16_t  highdetec;          // high level value
};

#define MEMDETFNAME "MEM_DETS"
#define LENLIBDETSERV 9
#define MDSPER 0x01
#define MDSREM 0x02
#define MDSTHE 0x03
#define MDSTIM 0x04
#define MDSNBSRC 0x3f


#define LMEMO 16
#define NBMEMOS 16
#define MEMOSFNAME "MEMOS___"


/*  PRINCIPES DE FONCTIONNEMENT

  Le frontal utilise le protocole http pour communiquer avec les périphériques wifi et l'utilisateur ; udp pour les périphériques radio via un concentrateur

  Il y a 2 types de connexions possibles :
    1) mode serveur (reception de commandes GET et POST et envoi de pages html)
    2) mode client des périphériques serveurs dont il a l'adresse IP (envoi de commandes et reception de pages html)

  Les périphériques ont au minimum un mode client (lorsqu'ils fonctionnent sur batteries en particulier).

  Le frontal et les périphériques échangent des "messages" encapsulés soit dans des commandes GET ou POST
  soit dans le <body></body> d'une page html selon le mode de connexion utilisé.

  La totalité des "messages" est imprimable. Un "message" peut contenir plusieurs fonctions concaténées séparées par "?"
  Les fonctions sont de de la forme :

      nom_fonction/longueur/[données]/crc

      Le nom de fonction fait 11 caractères ; il se termine par '=', il ne contient pas d'espaces.
      les caractères 9 et 10 sont utilisés pour "démultiplier" le codage (tableau libfonctions) :
        Une meme fonction peut ainsi coder plusieurs "sousfonction" ou transporter des paramètres.
        Lors de l'interprétaion de la fonction, libfonctions[n° fonct courante] contient les caractères 9 et 10.
      La longueur des données est au maximum est de 9999 caractères (4 digits complétés avec des 0 à gauche).
      Le crc est sur 2 caractères (1 octet hexadécimal représenté en ascii). Il ne prend pas en compte le nom de fonction.

      Une fonction a donc une longueur minimale de 17 caractères et maximale de 10016.

      Le frontal peut aussi recevoir via GET ou POST des fonctions en provenance des pages html qu'il a fourni à des navigateurs.
      Elles n'ont ni longueur ni crc.

  Une couche d'encryptage sera ajoutée lorsque l'ensemble sera stabilisé. l'HTTPS n'est pas disponible dans les libs ethernet actuelles.

  FONCTIONS

  (fonctions des périphériques serveurs : etat______ ; set_______ ; ack_______ ; reset_____ ; sleep_____ ; mail______ )

  Certaines n'ont pas d'arguments donc il n'y a pas de données dans les messages correspondants.
    (pour assurer la variabilité des encryptages ultérieurs, des données aléatoires seront insérées)
  Certaines ont une simple valeur numérique comme argument.
  Certaines ont une collection d'arguments (avec ou sans séparateurs).

  L'ordre des arguments est fixe. La liste se termine au premier absent.

  PRINCIPE DES ECHANGES

  Table des périphériques :

  Le frontal reçoit de façon asynchrone des messages avec les fonctions (GET)data_read (réponse (page.html)set_______) et (
  (GET)data_save_ (réponse (page.html)ack_______.)
  Les données contenues dans ces messages sont enregistrées dans le cache du fichier du périphérique en mémoire.
  La recopie du cache sur SD est liée à certai évènements (HALT etc) 
  Le message complet est enregistré dans l'historique.

  Sur action de l'utilisateur via un navigateur (et via les changements de detMem), la configuration d'un périphérique peut être modifiée
     ce qui génère l'envoi d'un message (GET)set_______ 
     (réponse non attendue : (GET)done______ ou (GET)data_save_ suivie de (page.html)ack_______ ).

  Lorsqu'un message sortant ou entrant se déroule anormalement, l'incident est noté et daté. 
     Les périphériques passent en mode "dégradé" : 
     Leur période de communication est augmentée pour éviter que les retrys ne plombent la consommation.

  Un périphérique sans serveur ne reçoit des messages que par page html.
  Un périphérique avec serveur reçoit des messages par les 2 modes.
  Le frontal envoie des messages par les 2 modes.

  Historique :

  Un fichier séquentiel accumule et date tous les échanges qui se produisent.

  MISE EN OEUVRE !!!! la suite N'EST PAS forcément à jour !!!!

  (le mot fonction désigne maintenant les fonctions du C)

  Pour l'envoi de message :

  int messToServer(*client,const ip,const port,const *données)  connecte à un serveur http et envoie un message
  (dans shmess)   renvoie  0 connexion pas réussie
                  ou 1 ok, la donnée est transmise, la connexion n'est pas fermée, le client est valide

  (dans frontal)
  periAns() envoie une page html minimale contenant une fonction (set_______/etat______...) dans <body></body>
  periReq() envoie une cde html via GET / fonction (set_______/etat______...)
  assyset() assemble le contenu du message
  //(void periSend() envoie une page html minimale contenant une fonction (set_______/etat______...) dans <body></body>)
  //(dans frontal)  le client doit être connecté ; pas de contrôle
  //               ( periSend utilise periParamsHtml(*client,*host,port) avec port!=0)
  //int periParamsHtml(client,char* host,int port) assemble et envoie un message set___
  //(dans frontal)  si port=0 dans page html en réponse à une requête GET ou POST ; sinon via une commande GET
                  utilise messToServer

  int buildReadSave(char* nomfonction,char* data)      assemble et envoie dataread/datasave
  (dans peripherique)                                  (utilise buildMess et messToServer

  Pour la réception de message :

  int getHttpResponse(*client,*données,*diag,lmax,*fonction)     attente d'une page html,
  (dans shmess)   reçoit et contrôle un message d'une seule fonction
                  le client doit être connecté ; renvoie un code MESSxx (voir shconst.h)
                                                 diag pointe une chaine imprimable selon le code.
                                                 le numéro de fonction reçue, la donnée (nnnn...cc)
                  contrôle du temps, de dépassement de capacité, de la longueur et du crc

  void fServer(uint8_t fwaited))    reçoit (via getHttpResponse) et des-assemble (via dataTransfer)
  (dans périphérique)               les données des fonction Set/Ack
                                    utilise bufServer espace permanent de bricolage
                                    renvoie periMess (MESSFON si inex ou invalide)

  le frontal comme les périphériques sont dans une boucle d'attente de connexion pour la réception de messages via GET OU POST

  Pour le controle de messages :

  int checkData(*données)    renvoie un code MESSxx (voir shconst.h)
  (dans shmess)

  int checkHttpData(*données, *fonction)   utilise checkData renvoie un code MESSxx et le numéro de la fonction
  (dans shmess)

  Pour l'assemblage des messages :

  int buildMess(*fonction, *données, *séparateur)      utilise bufServer espace permanent de bricolage
  (dans shmess)                                        renvoie 0 si décap (ctle préalable) longueur totale sinon
                                                       les messages sont concaténés et le séparateur inséré

  void assySet(char* message,int pericur,char* diag,char* date14);
  (dans shmess)                                        assemble les données des fonction Set/Ack


  STRUCTURE TABLE DES PERIPHERIQUES

      Un fichier dans la carte SD par ligne avec numéro de ligne dans le nom du fichier

        periSwPulseCtl (cstRec.pulseCtl dans périphérique)

                 (enable des 2 compteurs et bit free run/oneshot)


          détail d'une entrée de règle :  (000000 01 - 00000000 - 0000 1 0 0 0 - 000000 02  = ext 0 - nu - sta 000 - sw 0)

            type source   : (2 bits)
                            détecteur physique local au périphérique
                            détecteur du serveur
                            géné pulses
                            mémoire locale
            numéro srce   : (6 bits)
            type dest     : (2 bits)
                            switch
                            détecteur du serveur
                            géné pulses
                            mémoire locale
            numéro dest   : (6 bits)
            enable        : 1 bit
            niveau actif  : 1 bit
            niveau précédent : 1 bit
            edge/static   : 1 bit
            actions
                  0) reset :  l'action reset remet les 2 compteurs à 0 en mode idle selon mode/état/flanc programmé
                  1) raz :    l'action raz remet à 0 le compteur courant sans effet sur l'horloge selon mode/état/flanc programmé
                  2) stop :   l'action stop suspend l'horloge selon l'état/flanc programmé
                  3) start :  l'action start déclenche l'horloge selon l'état/flanc programmé
                  4) short :  l'action short termine le compteur courant sans changer la période totale (le compteur suivant est augmenté)
                  5) fin :    l'action fin termine le compteur courant.
                  6) stop impulsionnel : stop si le compteur depuis le début a moins de DETIMP (1,5sec) start sinon
                        (impDetTime=millis() si start, =0 si stop ou stop impulsionnel)
                  7) toggle : action sur switch
                  8) OR
                  9) NOR
                  10) XOR
                  11) AND

         les détecteurs physiques sont représentés dans une variable mémoire (débounce est intégré)





         générateur d'impulsion :

                (4 par périphérique)
                Le générateur d'impulsion est constitué de 2 compteurs animés par une horloge à 1Hz (10Hz pour les traitements pour assurer la précision)
                Au reset ils sont à 0. Un seul eet actif à la fois. Lorsqu'un compteur atteint sa valeur maxi il repasse à 0.
                Lorsque le compteur 1 atteint sa valeur maxi le compteur 2 se déclenche s'il est enable
                Lorsque le compteur 2 atteint sa valeur maxi le compteur 1 se déclenche s'il est enable en mode free run
                L'horloge et les compteurs sont commandés par les actions de la table des entrées.

         états du générateur stockés dans staPulse:

                  1) disable le premier compteur est bloqué.
                  2) idle l'horloge est arrêtée.
                  3) running1, le 1er compteur n'a pas atteint sa valeur maxi
                  4) fin 1, le 1er compteur a atteint sa valeur maxi et le second compteur est bloqué
                  5) running 2, le 2nd compteur n'a pas atteint sa valeur maxi.
                  6) fin 2, le 2nd compteur a atteint sa valeur maxi, mode free run, et le premier compteur est bloqué
                     en mode free run lorsque le second compteur atteint sa valeur maxi, le générateur passe à l'état running 1
                     en mode oneshoot lorsque le second compteur atteint sa valeur maxi, l'horloge s'arrête état idle valeur 0


   int8          numéro
   16 bytes      nom
   uint8         nbre switchs    maximum 4 (MAXSW)
   byte          etat/cde        2 bits par switch (etat/cde)
   16 bytes (4 / switch)
      1 byte        sources activation         2 bits N° detec
                                               1 bit détec enable
                                               1 bit HIGH/LOW active
                                               1 bit server cde enable
                                               1 bit H/L active
                                               1 bit pulse enable
                                               1 bit H/L active
      1 byte        sources désactivation
      1 byte        sources forçage ON
      1 byte        sources forçage OFF
   66 bytes (16+2 bytes / switch)
      4 bytes       durée ONE 4Giga (ou 99 999 999) * 1/100sec   (stop si durée=curr ; start si condition remplie))
      4 bytes       beg  tONE
      4 bytes       durée TWO
      4 bytes       beg  tTWO
      4 bytes       controle                   1 bit tONE enable
                                               1 bit tTWO enable
                                               1 bit stop serveur si 1 pulse stoppé
                                               1 bit détecteur de déclenchement enable
                                               2 bits numéro détecteur pour déclenchement pulseONE (parmi 4 détec)
                                               1 bit declenchement sur état ou front
                                               1 bit déclenchement sur H/L
                                               1 bit détecteur d'arrêt enable
                                               2 bits numéro détecteur d'arrêt pulse
                                               1 bit arrêt sur état ou front
                                               1 bit arrêt sur H/L
                                               2 bits cycle : (représentés par F et P dans periTable)
                                                  00 selon la config (undefined cycle)
                                                  01 free run (après déclenchement nécessite une condition stop pour s'arrêter ; reprend si la condition disparait)
                                                  10 one shot
                                                  11 one shot terminé
                                               1 bits dispo

                                               pour le générateur
                                               1 bit enable tOne
                                               1 bit enable tTwo
                                               1 bit free run/oneshot

                                               pour détecteurs et serveur : (4 fois)
                                               1 bit enable
                                               3 bits numéro
                                               1 bit local/externe
                                               1 bit mode
                                               1 bit H/L
                                               3 bits action


   uint8         nbre détecteurs (maximum 4 MAXDET)
   byte          1 bits etat ; 1 bit enable
   uint8         nbre sondes     maximum 256 (MAXSDE)
   6 bytes       MacAddr
   4 bytes       IpAddr
   6 bytes       temps dernier envoi (AAMMJJHHMMSS BCD)
   6 bytes       temps derniere réception
   6 bytes       temps dernière anomalie com
   1 byte        code dernière anomalie (MESSxxx)
   2 bytes       version         XX.XX BCD

  total xxx

    fichier sondes (contient numéro de périphérique et numéro de sonde) (à implanter)

    6 bytes     MacAddr périphérique
    uint8       numéro sonde
    byte        type            (1 temp, 2 tension, 3 courant, vitesse etc...
    float       valeur courante
    uint16      période         période d'appel au serveur (secondes)
    uint8       pitch           variation minimum significative
    6 bytes     temps dernière valeur
    1 byte      param           (bit 0 affichage O/N, bit 1 présente O/N, ...

*/


/* debug tips & tricks
    si une variable ne reste pas modifiée après le re-affichage de péritable, c'est qu'il n'y a pas de periSave après la modif
    apparition de 0D0A à la suite de la valeur dans GET ... -> filtrage des car < 0x21
*/


#endif // _CONST_H_
