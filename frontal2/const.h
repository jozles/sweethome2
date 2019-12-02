#ifndef _CONST_H_
#define _CONST_H_


#define VERSION "1.4a"
/* 1.1 ajout voltage dans données data_read_ ; modif unpackMac
 * 1.1a ajout volts et version dans table
 * 1.1b suppression dht ; ajout periDetVal et periSwVal avec affichage/saisie dans la table ; gestion serveur dev
 * 1.1c ajout periIpAddr avec aff dans la table        
 * 1.1d ajout periSwPulse, periSwNb, periDetSs, periDetOo avec affichage dans la table et tfr aux périphériques
 *      ajout longueur message après GET / pour com avec serveur périph (les modifs de messages ne perturbent plus le crc)
 * 1.1e modif gestion des données reçues avec GET ou POST : ajout tableau d'offsets dans valeurs[] (nvalf[]) pour ne plus 
 *      perdre de l'espace avec une longueur de données fixe. Les données sont séparées par '\0';
 *      Rédaction des "principes" et mise en lib de shconst.h, shutil et messages : 
 *            conversion du soft (version 1.9 pour périphériques).
 *      Peritable : N°,nom,temp,freq,pitch,nbre int*(etat,cde,timer,nbre det, état), mac addr,ip addr,last in time,last out time
 *                  ajout bouton "refresh" et config per refr.
 *      ntp cassé
 * 1.1f restructuration communications (dataread/save;set;ack)      
 *      shmess reçoit assySet, inpSet (à venir assyRead/Save inpRead/Save)
 * 1.1g suppression accueil et fonctions associées, mise en place perisend(cliext)      
 * 1.1h pulse opérationnel ; ajout variable (float) periThOffset dans periRec (offset sur température mesurée)
 * 1.1j corespond à la version 1.c de peripherique
 * 1.1k ajout alarmes th et volts + champs detecteurs du serveur
 *      gestion mots de passe et TO révisée (rémanence password via macMaster)
 *      fichier config sur SD, séparation pages switch dans peritable, multiples révisions des fonctions.
 *      ajout periPerTemp dans fichiers periphériques pour paramétrage période temp via péritable
 *      ajout datecmp(),fichier remote et fonctions associées cfgremote,load,save,print;sliderhtml() 
 *      
 * 1.2  utilhtml regroupe les fonctions utiles aux pages html 
 *      ajout serveur pilote (pilotserv()) et traitement dans pilotserver()       -> client cli_b
 *      l'acquisition/traitement des fonctions de periserv() devient periserver() -> client cli_a
 *      remoteHtml() traitement des requêtes sur pilotserv
 * 1.2a ajout dans fichier config de la table username/usrpass
 *      commonserver() 
 *      nouveau système password et TO (voir commonserver()) (boutRetour,boutFonction,usrForm)
 *      ajout periPort et tfr via ack/set ; accés aux périph serveurs débuggé (fermeture connection "cli.stop")
 * 1.2b ajout timers ; periTableHtml() protège periCur ; periSend valorise periCur + periLoad   
 * 1.3  gestion des switchs :      
 *      32 det serveur  ; remotes sur det serveur
 * 1.3a pulses indépendants des switchs, disjoncteur en amont de tout      
 *      24 inputs par sw (enable, lev, oldlev, edge/stat, srce (phy/ext/pulse/mem, num), dest (sw/ext/pulse/mem, num), action) *      
 *      form à part pour les détecteurs du serveur ds peritable ; 1 bouton "per update"(pertosend)
 *      bouton refresh des switchs envoie une demande d'état au périphérique
 *      libellé pour detecteurs serveur ; affichage/saisie lignes péritable disjoint
 * 1.3b periSend et periParamsHtml disparaissent ... periReq et periAns envoient des messages aux périphériques
 *      ds3231.h devient une lib ; création de la classe Ds3231 (instance ds3231)
 * 1.4  réaménagement setup ; udp ; 
 * 1.4a extension thermomètres pour contrôle chaudière/clim ; ajout scanThermos()     
 *      valeurs entières pour températures(x100) dans fichiers peri ; convStrToInt remplace convStrToNum là où c'est possible
 *      
 * BUGS : 
 *  
 * à faire :
 * 
 *    remplacer date14 par date unix sur 8 digits dans les messages ACK et SET
 *     
 *    ajouter nbre inputs dans assyset
 *     
 *    détecter les changements à la réception des dataread/save pour effectuer un refresh de l'affichage de peritable
 *    
 *    timers : ajouter option "1 jour sur n" : dhdebcycle=1ère date... calculer si date courante ok (récupérer un bool inutile - cyclic? )
 *    pulses : option entrée clock depuis input 
 *    
 *    ajouter alarmes dans peritable : alim+/- temp+/- detecteurs temps non communication et envoi mails
 *     
 *    dans fenetre switchs ; table inputs avec pour chaque ligne boutons poubelle/validation, + (nouv ligne), xmit(pertosend)
 *                           ajouter visu diag/état des inputs(?)
 *                           
 *    ajouter crc sur fichiers config,periphériques,remotes,timers 
 *    
 *    ajouter détection "android" dans getnv
 *    
 */

/* mac,adressage,port
  
  (MACADDR) mac du fichier config définit la carte ethernet.
  Le routeur doit la connaitre via la table DHCP pour lui attribuer une adresse IP fixe
  ce qui permet de démarrer le service avec "Ethernet.begin(mac)"
  Sinon utiliser "Ethernet.begin(mac,localIp)" pour forcer une adresse IP particulière (localIp provient du fichier de config)
  (si elle est déjà prise sur le routeur ça ne fonctionnera pas). 
  Pour configurer la table DHCP de la box, l'adresse MAC doit en être connue : 
    démarrer le serveur avec Ethernet.begin(mac), la box fourni une adresse Ip quelconque, puis configurer DHCP sur la box)
  localIp ne sert à rien en cas de bail fixe sur la box.
  localIp du fichier config est inutilisé (v1.2b)
  

  PORTSERVER et PORTPILOT sont les port des serveurs. 
  Pour permettre aux appels entrants sur l'adresse de la box (forme xxx.xxx.xxx.xxx/pppp) d'aboutir aux serveurs,
  la box doit avoir une redirection de port paramétrée vers l'IP de la carte ethernet (donc de préférence un bail fixe).
  (PORTPERISERVER et PORTPILOTSERVER proviennent de shconst.h)
  "portserver" du fichier config est inutilisé (v1.2b)

  pour initialiser un serveur : 
  configurer (MACADDR) mac de config + PORTSERVER et PORTPILOT qui doivent correspondre au DHCP et redirection de port de la box

*/

#define _MODE_DEVT2    // change l'adresse Mac de la carte IP, l'adresse IP (via DHCP de la box) et le port (en accord avec redir de port de la box)

#ifdef _MODE_DEVT2
#define MODE_EXEC "DEVT2" 
#define MACADDR "\x90\xA2\xDA\x0F\xDF\xAE"    
#define LOCALSERVERIP HOSTIPADDR2                       // 36          
#define PORTSERVER PORTPERISERVER2                      // 1786
#define PORTPILOT  PORTPILOTSERVER2                     // 1788
#define PORTUDP    PORTUDPSERVER2                       // 8886
#define NOMSERV "sweet dev2\0"
#define LNSERV  17
#endif _MODE_DEVT2

#ifdef _MODE_DEVT
#define MODE_EXEC "DEVT"
#define MACADDR "\x90\xA2\xDA\x0F\xDF\xAC"    //adresse mac carte ethernet AB service ; AC devt
#define LOCALSERVERIP {192,168,0,35}                   //adresse IP    ---- 34 service, 35 devt
#define PORTSERVER PORTPERISERVER                      // 1790
#define PORTPILOT  PORTPILOTSERVER                     // 1792
#define NOMSERV "sweet hdev\0"
#define LNSERV  17
#endif _MODE_DEVT

#ifdef _MODE_RUN
#define MODE_EXEC "RUN"
#define MACADDR "\x90\xA2\xDA\x0F\xDF\xAB"    //adresse mac carte ethernet AB service ; AC devt
#define LOCALSERVERIP {192,168,0,34}                   //adresse IP    ---- 34 service, 35 devt
#define PORTSERVER PORTPERISERVER                      // 1790
#define PORTPILOT  PORTPILOTSERVER                     // 1792
#define NOMSERV "sweet home\0"
#define LNSERV  17
#endif _MODE_RUN

#define UDPUSAGE

#define DS3231_I2C_ADDRESS 0x68               // adresse 3231 sur bus I2C


#define NBPERIF 20                           
#define PERINAMLEN 16+1                      // longueur nom perif
#define PERIRECLEN 258 // V1.4a              // longueur record périph

#define CONFIGRECLEN 865                     // longueur record config 

#define TO_PASSWORD 600                      // sec (pour initialiser toPassword à la mise sous tension

#define MAXIMGLEN   4000                     // longueur maxi pour fichiers image

#define MAXSERVACCESS 120                    // (sec) période maximale accés au serveur par défaut
#define TEMPERPREF   20                      // (sec) période check température par défaut sur périphérique
#define MINTHCHGE    0.25                    // changement minimal th pour save

#define SDOK 1
#define SDKO 0 

//enum {FAUX,VRAI};

#define LENREMNAM 16    // remote name length
#define NBREMOTE 8      // remote entrys
#define MAXREMLI 16     // sw items total
#define REMOTENFNAME "noms_rem"
#define REMOTETFNAME "tablerem"

struct SwRemote
{
  uint8_t  num;           // remote number
  uint8_t  detec;         // detecteur
  bool     enable;        // remote enable
};

struct Remote
{
  char    nam[LENREMNAM]; // remote name
  bool    onoff;          // remote on/off
  bool    enable;         // remote enable
  bool    newonoff;       // buffer pour reception et traitement cb par GET /
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
  char      nom[LENTHNAME+1];   // nom
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

/* codes fonctions (mettre à jour...)
00 per_temp__ valorisation période de la mesure et enregistrement de température du serveur
01 
02 peri_pass_ mot de passe periphériques 
03 dump_sd___ affichage page dump contenu SD
04 test2sw___ 
05 done______
06 per_refr__ valorisation periode rafraichissement de la page d'accueil
07 peri_tofs_ valorisation offset thermomètre
08 per_date__ valorisation periode de l'enregistrement date dans SD
09 reset_____ reset serveur
10 password__ affichage page saisie mdp
11 sd_pos____ valorisation pointeur pour dump sd
12 data_save_ données formatées pour maj table et enregistrement dans SD 
13 data_read_ demande de paramètres pour un périphérique au serveur 
14 peri_parm_ (rien... actuellement accueil... à recycler)
15 peri_cur__ valorisation numéro périphérique courant pour la table
16 peri_refr_ valorisation periode de mesure/action du périphérique courant
17 peri_nom__ valorisation nom du périphérique courant
18 peri_mac__ valorisation adresse Mac du périphérique courant
19 accueil___ affichage page d'accueil du serveur
20 peri_table affichage page table des périphériques
21 peri_prog_ valorisation flag "programmable" du périphérique courant
22 peri_sonde valorisation flag "sonde" du périphérique courant
23 peri_pitch valorisation valeur "pitch" (écart significatif de mesure) du périphérique courant
24 peri_pmo__ valorisation num de détecteur pour on/off et des bits associés de periSwPulseCtl
25 peri_detec valorisation nombre de détecteurs sur le périphérique
26 peri_intnb valorisation nombre d'interrupteurs sur le périphérique
27 peri_intv0 valorisation de l'état de l'interrupteur 0 dans *periSwVal (4 interrupteurs possibles, 2 bits par inter)
28 peri_intv1 valorisation de l'état de l'interrupteur 1 dans *periSwVal (4 interrupteurs possibles, 2 bits par inter)
29 peri_intv2 valorisation de l'état de l'interrupteur 2 dans *periSwVal (4 interrupteurs possibles, 2 bits par inter)
30 peri_intv3 valorisation de l'état de l'interrupteur 3 dans *periSwVal (4 interrupteurs possibles, 2 bits par inter)
31 
32 peri_sfp__ valorisation des bits OTSFP de periSwPulseCtl 
33 peri_imn__ valorisation detecteurs sources pour switchs
34 peri_imc__ valorisation bits sources pour switchs 
35 peri_pto__ valorisation durée de l'impulsion tOne si 0 pas d'impulsion sinon durée en secondes
36 peri_ptt__ valorisation durée de l'impulsion tTwo si 0 pas d'impulsion sinon durée en secondes
37 
38 
39 

  PRINCIPES DE FONCTIONNEMENT

  Le frontal utilise le protocole http pour communiquer avec les périphériques et l'utilisateur

  Il y a 2 types de connexions possibles :
    1) mode serveur capable de recevoir des commandes GET et POST et d'envoyer des pages html 
    2) mode client des périphériques serveurs dont il a l'adresse IP capable d'envoyer des commandes et recevoir des pages html

  Les périphériques ont au minimum un mode client (lorsqu'ils fonctionnent sur batteries en particulier).

  Le frontal et les périphériques échangent des "messages" encapsulés soit dans des commandes GET ou POST
  soit dans le <body></body> d'une page html selon le mode de connexion utilisé.
  
  La totalité des "messages" est imprimable. Un "message" peut contenir plusieurs fonctions concaténées séparées par "?"
  Les fonctions sont de de la forme :
      
      nom_fonction/longueur/[données]/crc 
          
      Le nom de fonction fait 11 caractères ; il se termine par '=', il ne contient pas d'espaces.
      les caractères 9 et 10 sont utilisés pour "démultiplier" le codage (tableau libfonctions) : 
            par ex le caractère 9 encode le numéro de switch avec le param PMFNCVAL, ou via frecupptr les car 9 et 10 encodent le type d'action et le numéro de bit
      La longueur des données est au maximum est de 9999 caractères (4 digits complétés avec des 0 à gauche).
      Le crc est sur 2 caractères (1 octet hexadécimal représenté en ascii). Il ne prend pas en compte le nom de fonction.

      Une fonction a donc une longueur minimale de 17 caractères et maximale de 10016.

      Le frontal peut aussi recevoir via GET ou POST des fonctions en provenance des pages html qu'il a fourni à des navigateurs. 
      Elles n'ont ni longueur ni crc.

  Une couche d'encryptage sera ajoutée lorsque l'ensemble sera stabilisé.

  FONCTIONS

  (voir liste des fonctions du serveur du frontal au-dessus)
  (fonctions des périphériques serveurs : etat______ ; set_______ ; ack_______ ; reset_____ ; sleep_____ )

  Certaines n'ont pas d'arguments donc il n'y a pas de données dans les messages correspondants.
    (pour assurer la variabilité des encryptages ultérieurs, des données aléatoires seront insérées)
  Certaines ont une simple valeur numérique comme argument.
  Certaines ont une collection d'arguments (avec ou sans séparateurs). 

  L'ordre des arguments est fixe. La liste se termine au premier absent.

  PRINCIPE DES ECHANGES

  Table des périphériques :
  
  Le frontal reçoit de façon asynchrone des messages avec les fonctions (GET)data_read (réponse (page.html)set_______) et (
  (GET)data_save_ (réponse (page.html)ack_______.)
  Les données contenues dans ces messages sont enregistrées dans la carte SD sous la forme d'une table des périphériques.
  La dernière réception est datée.
  
  Sur action de l'utilisateur via un navigateur, la configuration d'un périphérique peut être modifiée ce qui génère 
  l'envoi d'un message (GET)set_______ (réponse non attendue : (GET)done______ ou (GET)data_save_ suivie de (page.html)ack_______ ).
  Le dernier envoi de message est daté.
  Lorsqu'un message sortant ou entrant se déroule anormalement, l'incident est noté et daté. 
  En cas d'anomalie, les périphériques doublent la période de com au serveur jusqu'à 7200sec.

  Un périphérique sans serveur ne reçoit des messages que par page html.
  Un périphérique avec serveur reçoit des messages par les 2 modes.
  Le frontal envoie des messages par les 2 modes.

  Historique :

  Un fichier séquentiel accumule et date tous les échanges qui se produisent.

  MISE EN OEUVRE

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
                

*  int8          numéro
*  16 bytes      nom  
*  uint8         nbre switchs    maximum 4 (MAXSW)
*  byte          etat/cde        2 bits par switch (etat/cde)
*  16 bytes (4 / switch)
*     1 byte        sources activation         2 bits N° detec
*                                              1 bit détec enable 
*                                              1 bit HIGH/LOW active
*                                              1 bit server cde enable
*                                              1 bit H/L active
*                                              1 bit pulse enable
*                                              1 bit H/L active  
*     1 byte        sources désactivation 
*     1 byte        sources forçage ON
*     1 byte        sources forçage OFF
*  66 bytes (16+2 bytes / switch) 
*     4 bytes       durée ONE 4Giga (ou 99 999 999) * 1/100sec   (stop si durée=curr ; start si condition remplie))
*     4 bytes       beg  tONE
*     4 bytes       durée TWO 
*     4 bytes       beg  tTWO
*     4 bytes       controle                   1 bit tONE enable
*                                              1 bit tTWO enable
*                                              1 bit stop serveur si 1 pulse stoppé
*                                              1 bit détecteur de déclenchement enable
*                                              2 bits numéro détecteur pour déclenchement pulseONE (parmi 4 détec) 
*                                              1 bit declenchement sur état ou front
*                                              1 bit déclenchement sur H/L
*                                              1 bit détecteur d'arrêt enable
*                                              2 bits numéro détecteur d'arrêt pulse
*                                              1 bit arrêt sur état ou front
*                                              1 bit arrêt sur H/L
*                                              2 bits cycle : (représentés par F et P dans periTable)
*                                                 00 selon la config (undefined cycle) 
*                                                 01 free run (après déclenchement nécessite une condition stop pour s'arrêter ; reprend si la condition disparait)
*                                                 10 one shot
*                                                 11 one shot terminé
*                                              1 bits dispo
*  
*                                              pour le générateur
*                                              1 bit enable tOne
*                                              1 bit enable tTwo
*                                              1 bit free run/oneshot
*                                              
*                                              pour détecteurs et serveur : (4 fois)
*                                              1 bit enable
*                                              3 bits numéro 
*                                              1 bit local/externe
*                                              1 bit mode
*                                              1 bit H/L
*                                              3 bits action
*                                              
*  
*  uint8         nbre détecteurs (maximum 4 MAXDET) 
*  byte          1 bits etat ; 1 bit enable
*  uint8         nbre sondes     maximum 256 (MAXSDE)
*  6 bytes       MacAddr
*  4 bytes       IpAddr
*  6 bytes       temps dernier envoi (AAMMJJHHMMSS BCD)
*  6 bytes       temps derniere réception
*  6 bytes       temps dernière anomalie com
*  1 byte        code dernière anomalie (MESSxxx)
*  2 bytes       version         XX.XX BCD
*  
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
 *  si une variable ne reste pas modifiée après le re-affichage de péritable, c'est qu'il n'y a pas de periSave après la modif
 *  apparition de 0D0A à la suite de la valeur dans GET ... -> filtrage des car < 0x21
 */


#endif // _CONST_H_
