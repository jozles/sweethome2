#ifndef _SHCONST_H_
#define _SHCONST_H_


#define PERIF

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

#define LENMESS     406+1               // longueur buffer message v1.3
#define MPOSFONC    0                   // position fonction
#define MPOSLEN     11                  // position longueur (longueur fonction excluse)
#define MPOSNUMPER  MPOSLEN+5           // position num périphérique
#define MPOSMAC     MPOSNUMPER+3        // position adr Mac
#define MPOSDH      MPOSMAC+18          // Date/Heure
#define MPOSPERREFR MPOSDH+15           // période refr
#define MPOSTEMPPER MPOSPERREFR+6       // période check température
#define MPOSPITCH   MPOSTEMPPER+6       // pitch
#define MPOSSWCDE   MPOSPITCH+5         // MAXSW commandes sw 0/1 + 1sep (periSwVal)
#define MPOSPULSONE MPOSSWCDE+MAXSW+1   // Timers tOne (4bytes)*4
#define MPOSPULSTWO MPOSPULSONE+NBPULSE*(LENVALPULSE+1)       // Timers tTwo (4bytes+1sep)*4
#define MPOSPULSCTL MPOSPULSTWO+NBPULSE*(LENVALPULSE+1)       // paramètres timers (4*3 bits=2 bytes)
#define MPOSPERINPUT MPOSPULSCTL+2*PCTLLEN+1                   // inputs
#define MPOSMDETSRV MPOSPERINPUT+NBPERINPUT*PERINPLEN*2+1        // detecteurs serveur
#define MPOSPORTSRV MPOSMDETSRV+MDSLEN*2+1                    // port perif serveur
#define MPOSMDIAG   MPOSPORTSRV+5                             // texte diag
#define MLMSET      MPOSMDIAG+5              // longueur message fonction incluse


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

#define LBUFSERVER LENMESS+LENNOM+1+4+1+2+1 // longueur bufserver (messages in/out periphériques)
                                            // + nom fonct+1+longueur+1+crc+1
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
#define BCODECONFIGRECLEN 9   // CONFIGRECLEN ou MLMSET/LENMESS faux -> blocage
#define BCODEPERICACHEKO 11   // periSave et cache invalide
#define BCODESYSERR      13   // system error (fdatasave>99 STEPDATASAVE!=talkStep)
#define BCODENUMPER      15   // tentative de periLoad d'un perif invalide

enum {FAUX,VRAI};
enum {OFF,ON};

/* description périphériques */

#define MAXSW   4       // nbre maxi switchs par périphérique
#define MAXDET  4       // nbre maxi détecteurs physiques par périphérique
//#define MAXDSP  4       // nbre maxi détecteurs spéciaux
//#define MAXDEX  8       // nbre maxi détecteurs externes
#define MAXSDE  4       // nbre maxi sondes par périphérique
#define MAXTAC  4       // 4 types actions sur pulses (start/stop/mask/force)
#define NBPULSE 4

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

#define NBDSRV 32               // nombre de det serveur (param "léger" memDetServ défini uint32_t !)
#define MDSLEN NBDSRV/8         // type memDetServ
//#define MAXDL  32               // valeur maxi pour num détecteur



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

/* description pulses */

#define LENVALPULSE 8


/* periSwPulseCtl (*/

#define PCTLLEN (((NBPULSE*PCTLBIT)/8)+1) //  4*3 bits =12 -> 2 bytes/perif
#define PCTLBIT 3                     // 3 bits/sw

/* bits controle compteurs (periSwPulseCtl) */

#define PMTOE_PB 0x04           // time one enable numéro du bit
#define PMTOE_VB 0x02           //                 valeur du bit
#define PMTTE_PB 0x02           // time two enable
#define PMTTE_VB 0x01
#define PMFRO_PB 0x01           // freeRun/OneShoot
#define PMFRO_VB 0x00


/* définition table inputs */

#define NBPERINPUT 24
#define PERINPLEN  4

// byte 0 & byte 3 -- num détecteur + type (source/dest)

#define PERINPV_MS    0xFC
#define PERINPNVMS_VB 0x80          // détec MSb
#define PERINPNVMS_PB 0x07
#define PERINPNVLS_VB 0x04          // détec LSb
#define PERINPNVLS_PB 0x02
#define PERINPNT_MS   0x03
#define PERINPNTMS_VB 0x02          // type MSb
#define PERINPNTMS_PB 0x01
#define PERINPNTLS_VB 0x01          // type LSb
#define PERINPNTLS_PB 0x00

// byte 1 -- règles

#define PERINPRULESLS_VB 0x01       // rules LSb
#define PERINPRULESLS_PB 0x00

// byte 2 -- act/usage/level/enable

#define PERINPACT_MS    0xF0
#define PERINPACTMS_VB  0x80        // action MSb (3 bits)
#define PERINPACTMS_PB  0x07
#define PERINPACTLS_VB  0x10        // action LSb
#define PERINPACTLS_PB  0x04
#define PERINPDETES_VB  0x08        // usage detec level (0=edge 1=static)
#define PERINPDETES_PB  0x03
#define PERINPVALID_VB  0x04        // actif level
#define PERINPVALID_PB  0x02
#define PERINPOLDLEV_VB 0x02        // prev detec level
#define PERINPOLDLEV_PB 0x01
#define PERINPEN_VB     0x01        // enable
#define PERINPEN_PB     0x00


/* types détecteurs */

#define NBDTYP 4

#define DETYEXT 1
#define DETYLOC 2
#define DETYPUL 3
#define DETYMEM 0


/* bits rules */

#define RULBITICVAL 0X80     // active level interrupteur/conjoncteur
#define RULBITIEN   0X40     // enable interrupteur
#define RULBITCEN   0X20     // enable conjoncteur
#define RULBITAVAL  0x10     // active level allumeur
#define RULBITAEN   0x08     // enable allumeur


/* codes actions */

//#define PMDCA_SW    0x00     // switch update
#define PMDCA_RAZ   0x01     // raz
#define PMDCA_STOP  0x02     // stop  clk
#define PMDCA_START 0x03     // start clk
#define PMDCA_SHORT 0x04     // short pulse
#define PMDCA_END   0x05     // end pulse
#define PMDCA_IMP   0x06     // start/stop impulsionnel
#define PMDCA_RESET 0x07     // reset
#define PMDCA_TGL   0x08     // toggle switch
#define PMDCA_SW0   0x10     // switch update 0
#define PMDCA_SW1   0x11     // switch update 1
#define PMDCA_LM0   0x12     // loc mem update 0
#define PMDCA_LM1   0x13     // loc mem update 1
#define PMDCA_EX0   0x14     // loc mem update 0
#define PMDCA_EX1   0x15     // loc mem update 1

#define MAXACT 16

#endif  _SHCONST_H_
