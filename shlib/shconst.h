#ifndef _SHCONST_H_
#define _SHCONST_H_


//#define PERIF

#define LENVERSION  4
#define LENMODEL    6
#define SIZEPULSE   4  // uint32_t
#define LENPERIDATE 6

///* 1er serveur
#define PORTPERISERVER  1790    // port du serveur pour périphériques et maintenance (RUN 1789)
#define PORTPILOTSERVER 1792    // port du serveur de remote
#define HOSTIPADDR "192.168.0.35"
//*/
///* 2nd serveur
#define PORTPERISERVER2  1786    // port du serveur pour périphériques et maintenance
#define PORTPILOTSERVER2 1788    // port du serveur de remote
#define HOSTIPADDR2 "192.168.0.36"
//*/




#ifdef PERIF
 #define PINLED 0                    //  0 = ESP-12  ; 2 = ESP-01
 #define LEDON LOW
 #define LEDOFF HIGH
#endif  defPERIF
#ifndef PERIF
 #define PINLED 13
 #define LEDON HIGH
 #define LEDOFF LOW
#endif ndef PERIF

    // messages

#define LENMESS     250             // longueur buffer message
#define MPOSFONC    0               // position fonction
#define MPOSLEN     11              // position longueur (longueur fonction excluse)
#define MPOSNUMPER  MPOSLEN+5       // position num périphérique
#define MPOSMAC     MPOSNUMPER+3    // position adr Mac
#define MPOSDH      MPOSMAC+18      // Date/Heure
#define MPOSPERREFR MPOSDH+15       // période refr
#define MPOSTEMPPER MPOSPERREFR+6   // période check température
#define MPOSPITCH   MPOSTEMPPER+6   // pitch
#define MPOSSWCDE   MPOSPITCH+5     // 4 commandes sw 0/1 (periSwVal)
#define MPOSINTPAR0 MPOSSWCDE+5     // paramétres sw (4*HH+1sep)*4
#define MPOSPULSONE MPOSINTPAR0+MAXSW*9 // Timers tOne (4bytes)*4
#define MPOSPULSTWO MPOSPULSONE+MAXSW*9 // Timers tTwo (4bytes+1sep)*4
#define MPOSPULSCTL MPOSPULSTWO+MAXSW*9 // paramètres timers (4*3 bits=2 bytes)
#define MPOSSWINPUT MPOSPULSCTL+PCTLLEN+1 // ext detec enable+level
#define MPOSEXTDEN  MPOSSWINPUT+MAXSW*(NBSWINPUT*SWINPLEN+1) // inputs (1 séparateur par sw)
#define MPOSPORTSRV MPOSEXTDEN+5    // port perif serveur
#define MPOSMDIAG   MPOSPORTSRV+3   // texte diag
#define MLMSET      MPOSMDIAG+5     // longueur message fonction incluse


    // fonctions

/*      message GET envoyé par periTable peut envoyer 251 fonctions (maxi version 1.1k)
        le buffer strSD de l'enregistrement d'historique doit pouvoir le contenir + entete et pied
        même pb pour valeurs, nvalf et libfonctions
        valeurs et strSD n'ont pas une longueurs fixe pour chaque fonction donc contrôle de décap

*/
#define LENNOM  10    // nbre car nom de fonction
#define NBVAL  256    // nbre maxi fonct/valeurs pour getnv() --- les noms de fonctions font 10 caractères + 2 séparateurs + la donnée associée = LENVAL
                      // taille macxi pour GET (IE)=2048  ; pour POST plusieurs mega
                      // avec un arduino Atmel la limitation vient de la taille de la ram
#define LENVAL 128    // nbre car maxi valeur (vérifier datasave)
#define MEANVAL  6    // pour donner un param de taille à valeurs[]
#define LENVALEURS NBVAL*MEANVAL+1   // la taille effective de valeurs[]
//#define LENPSW  16    // nbre car maxi pswd
#define RECHEAD 28                           // en-tete strSD date/heure/xx/yy + <br> + crlf
#define RECCHAR NBVAL*(MEANVAL+3)+RECHEAD+8  // longueur maxi record histo

#define LBUFSERVER LENMESS+16                // longueur bufserver (messages in/out periphériques)

/* positions fixées dans strSD (fhisto) */

#define SDPOSNUMPER 9   // position numéro périphérique / ';'
#define SDPOSMAC    12  // position addr mac    / ';'
#define SDPOSTEMP   30  // position température / ';'

/* >>>> mots de passe <<<< */

#define SRVPASS     "17515A" // pwd accès serveur pour péri
#define PERIPASS    SRVPASS
#define USRPASS     "17515A" // pwd accès serveur via browser
#define MODPASS     "17515A" // pwd modif serveur via browser
#define LPWD        8
#define LENUSRPASS  8
#define NBUSR       8
#define LENUSRNAME  16

/* >>>> WIFI <<<<< */

#define MAXSSID   8
#define LENSSID   16
#define LPWSSID   48
#define SSID1     "pinks"
#define PWDSSID1  "cain ne dormant pas songeait au pied des monts"
#define SSID2     "devolo-5d3"
#define PWDSSID2  "JNCJTRONJMGZEEQL"


#define TOINCHCLI 4000        // msec max attente car server
#define TO_HTTPCX 4000        // nbre maxi retry connexion serveur

#define SLOWBLINK 3000
#define FASTBLINK 500
#define PULSEBLINK 40
/* code blink  (valeurs impaires bloquantes) */
/* code courant+100 reset (sauf impairs ofcourse) */
#define BCODEONBLINK      98  // allume jusqu'au prochain blink
#define BCODEPERIRECLEN   3   // PERIRECLEN ou LENMESS trop petit -> blocage
#define BCODEPBNTP        2   // pas de service NTP
#define BCODEWAITWIFI     4   // attente WIFI
#define BCODESDCARDKO     5   // pas de SDCARD
#define BCODEFHISTO       6   // write 0 car (sdstore_textdh0)
#define BCODELENVAL       7   // LENVAL trop petit
#define BCODECONFIGRECLEN 9   // CONFIGRECLEN faux -> blocage
#define BCODEPERICACHEKO 11   // periSave et cache invalide
#define BCODESYSERR      13   // system error (fdatasave>99 STEPDATASAVE!=talkStep)
#define BCODENUMPER      15   // tentative de periLoad d'un perif invalide

enum {FAUX,VRAI};
enum {OFF,ON};


#define MAXSW   4       // nbre maxi switchs par périphérique
#define MAXDET  4       // nbre maxi détecteurs par périphérique
#define MAXDSP  4       // nbre maxi détecteurs spéciaux
#define MAXDEX  8       // nbre maxi détecteurs externes
#define MAXSDE  4       // nbre maxi sondes par périphérique
#define MAXTAC  4       // 4 types actions sur pulses (start/stop/mask/force)

// messages diag

#define MESSTO    0
#define MESSDEC  -1
#define MESSOK    1
#define MESSCRC  -2
#define MESSLEN  -3
#define MESSFON  -4
#define MESSFULL -5
#define MESSSYS  -6
#define MESSCX   -7
#define MESSNUMP -8
#define MESSMAC  -9

#define NBMESS   8  // OK ne compte pas

#define TEXTTO      "*TO_\0"
#define TEXTDEC     "*OVF\0"
#define TEXTCRC     "*CRC\0"
#define TEXTLEN     "*LEN\0"
#define TEXTFON     "*FON\0"
#define TEXTFULL    "*FUL\0"
#define TEXTSYS     "*SYS\0"
#define TEXTCX      "*CX_\0"

#define TEXTMESS "\0   \0*TO_\0*OVF\0*CRC\0*LEN\0*FON\0*FUL\0*SYS\0*CX_\0"

#define LPERIMESS  5   // len texte diag en réception de message

/* bits structures definitions */

/* SWITCHS CONTROL */

// PULSE MODE = PM
// time one = TO ; time two = TT ; current = C ; duration = D
// Server = SR ; free run = FR ; Phase H/L-LH = PH
// enable = E ; UP/DOWN = U ; H/L = H
// number = N ; detector on = DI ; off = DO ; ON = I ; OFF = O
// most significant = MS ; least = LS
//

/* nombre détecteurs externes */

#define NBDSRV 8
#define MAXDL  32      // valeur maxi pour num détecteur

/* description periSwPulseCtl (*/

#define PCTLLEN (MAXSW*PCTLBIT)/8+1 //  4*3 bits =12 -> 2 bytes/perif
#define PCTLBIT 3                   // 3 bits/sw

/* bits controle compteurs (periSwPulseCtl) */

#define PMTOE_PB 0x04           // time one enable numéro du bit
#define PMTOE_VB 0x02           //                 valeur du bit
#define PMTTE_PB 0x02           // time two enable
#define PMTTE_VB 0x01
#define PMFRO_PB 0x01           // freeRun/OneShoot
#define PMFRO_VB 0x00


/* bits détecteurs logiques */
/*
#define DLSWLEN   (DLNB*DLBITLEN/8+1) // nbre bytes par sw
#define DLNBSWCB   DLNB*DLNBCB        // nbre checkbox par switch

#define DLNB        4                 // nbre détecteurs logiques par switch

#define DLBITLEN   10                 // longueur (bits) desciption détecteur
#define DLBITMSK    0x03FF            // mask 10 bits

#define DLNBCB      4                 // nbre checkbox/détecteur
#define DLNULEN     3                 // nbre bits numéro détecteur
#define DLACLEN     3                 // nbre bits code action

/* codes mode */
/*
#define PMDM_STAT   0                  // statique
#define PMDM_TRANS  1                  // transitionnel
#define DLLOCAL     1                  // détecteur local

#define DLNMS_PB   DLNLS_PB+DLNULEN-1  // msb numéro det (3 bits)
#define DLNMS_VB   0x0200
#define DLNLS_PB   DLENA_PB+1          // lsb numéro
#define DLNLS_VB   0x0080
#define DLENA_PB   DLEL_PB+1           // enable (1 bit)
#define DLENA_VB   0x0040
#define DLEL_PB    DLMFE_PB+1          // local/externe (1 bit)
#define DLEL_VB    0x020
#define DLMFE_PB   DLMHL_PB+1          // mode flanc/état (1 bit)
#define DLMFE_VB   0x010
#define DLMHL_PB   DLACMS_PB+1         // H/L (1 bit)
#define DLMHL_VB   0x008
#define DLACMS_PB  DLACLS_PB+DLACLEN-1 // msb action sur pulse (3 bits)
#define DLACMS_VB  0x004
#define DLACLS_PB  0                   // lsb action
#define DLACLS_VB  0x001
*/

/* bits memDetec */

#define  DETBITLH_VB  0x01 // 1 bit état HIGH LOW
#define  DETBITLH_PB  0
#define  DETBITUD_VB  0x02 // 1 bit flanc/état du déclenchement
#define  DETBITUD_PB  1
#define  DETBITST_VB  0x0C // 2 bits état (déclenché(TRIG)/attente(WAIT)/disable(DIS)) si DIS le bit d'état est invalide
#define  DETBITST_PB  2
#define  DETTRIG      0x03 // valeur DETBIST si déclenché (LH=L falling =H rising)
#define  DETWAIT      0x02 // valeur         si armé      (LH=H falling =L rising)
#define  DETIDLE      0x01 // valeur         si pas d'interruption (LH valide)
#define  DETDIS       0x00 // valeur         si disable (LH invalide)


/* codes actions detecteurs logiques sur pulse */

#define PMDCA_RESET 0x00       // reset
#define PMDCA_RAZ   0x01       // raz
#define PMDCA_STOP  0x02       // stop  clk
#define PMDCA_START 0x03       // start clk
#define PMDCA_SHORT 0x04       // short pulse
#define PMDCA_END   0x05       // end pulse
#define PMDCA_IMP   0x06       // start/stop impulsionnel
#define PMDCA_TGL   0x07       // toggle switch

#define MAXACT 7

/* codes etats générateur (bit droite=valeur) */

// bit 1 0=stop      1=run
// bit 2 0=cpt1      1=cpt2
// bit 3 1=compteur  0=clk
// bit 4 1=disable   0=enable

#define PM_DISABLE  0x0C
#define PM_IDLE     0x00
#define PM_RUN1     0x05
#define PM_RUN2     0x07
#define PM_END1     0x04
#define PM_END2     0x06

#define PM_FREERUN  0
#define PM_ONESHOOT 1
#define PM_ACTIF    1

/* SWITCH MODE ON/OFF */
/*
#define SWMDLNUMS_PB 0x07  // position ms bit num détecteur
#define SWMDLNUMS_VB 0x80  // valeur
#define SWMDLNULS_PB 0x06  // position ms bit num détecteur
#define SWMDLNULS_VB 0x40
#define SWMDLEN_PB   0x05  // detecteur enable
#define SWMDLEN_VB   0x20
#define SWMDLHL_PB   0x04  // detecteur H/L
#define SWMDLHL_VB   0x10
#define SWMSEN_PB    0x03  // serveur enable
#define SWMSEN_VB    0x08
#define SWMSHL_PB    0x02  // serveur H/L
#define SWMSHL_VB    0x04
#define SWMPEN_PB    0x01  // pulse enable
#define SWMPEN_VB    0x02
#define SWMPHL_PB    0x00  // pulse H/L
#define SWMPHL_VB    0x01
*/


// codage car différenciation de fonction

#define PMFNCVAL    0X30    // car d'offset ('0') de la valeur de fonction (*valf)

#define PMFNCHAR    0x40    // NE PAS MODIFIER !! le codage des shifts
// de periSwPulseCtl est critique : utilisation des lettres maj et min seules
// car d'offset ('@') dans nom de fonction


// debug
#define NBDBPTS 4
#define NBDBOC  5

#define DLNB 4  // pour compiler... à virer


/* définition table inputs */

#define NBSWINPUT 8
#define SWINPLEN  3
// valeur
#define SWINPV_MS    0xFC
#define SWINPNVMS_VB 0x80          // value MSb
#define SWINPNVMS_PB 0x07
#define SWINPNVLS_VB 0x04          // value LSb
#define SWINPNVLS_PB 0x02
#define SWINPNT_MS   0x03
#define SWINPNTMS_VB 0x02          // type MSb
#define SWINPNTMS_PB 0x01
#define SWINPNTLS_VB 0x01          // type LSb
#define SWINPNTLS_PB 0x0
// params
#define SWINPDISE_VB 0x008000           // disjoncteur enable
#define SWINPDISE_PB 0x0F
#define SWINPDISL_VB 0x004000           // disjoncteur level
#define SWINPDISL_PB 0x0E
#define SWINPCONE_VB 0x002000           // conjoncteur enable
#define SWINPCONE_PB 0x0D
#define SWINPCONL_VB 0x001000           // conjoncteur level
#define SWINPCONL_PB 0x0C
#define SWINPINTE_VB 0x000800           // interrupteur enable
#define SWINPINTE_PB 0x0B
#define SWINPINTL_VB 0x000400           // interrupteur level
#define SWINPINTL_PB 0x0A
#define SWINPALLE_VB 0x000200           // allumeur enable
#define SWINPALLE_PB 0x09
#define SWINPALLL_VB 0x000100           // allumeur level
#define SWINPALLL_PB 0x08
#define SWINPACT_MS   0x00E0
#define SWINPACTMS_VB 0x000080          // action MSb
#define SWINPACTMS_PB 0x07
#define SWINPACTLS_VB 0x000020          // action LSb
#define SWINPACTLS_PB 0x05
#define SWINPEN_VB 0x000001             // enable
#define SWINPEN_PB 0x0

#endif  _SHCONST_H_
