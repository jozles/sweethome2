#ifndef _SHCONST_H_
#define _SHCONST_H_

//#define PERIF

//#define SHDIAGS

#define LENVERSION  4
#define LENMODEL    6
//#define SIZEPULSE   4  // uint32_t
#define LENPERIDATE 6

///* 1er serveur
#define PORTPERISERVER  1790    // port du serveur pour périphériques et maintenance (RUN 1789)
#define PORTPILOTSERVER 1792    // port du serveur de remote
#define HOSTIPADDR "192.168.0.35"
//*/
///* 2nd serveur
#define PORTPERISERVER2  1786    // port du serveur pour périphériques et maintenance
#define PORTPILOTSERVER2 1788    // port du serveur de remote
#define PORTUDPSERVER2   8886
#define HOSTIPADDR2 "192.168.0.36"
//*/
///* Concentrateur NRF
#define PORTTCPCONC      1784
#define PORTUDPCONC      8887
#define CONCNRFIPADDR    {192,168,0,31}
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

    // protocoles

#define PROTOCHAR " TU"
#define PROTOCSTR "   \0TCP\0UDP\0"
#define LENPROSTR 4
#define NBPROTOC  2

#define UNIXDATELEN 4                   // uint32_t

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
#define LENVAL 256    // nbre car maxi valeur (vérifier datasave)
#define MEANVAL  6    // pour donner un param de taille à valeurs[]
#define LENVALEURS NBVAL*MEANVAL+1   // la taille effective de valeurs[]
//#define LENPSW  16    // nbre car maxi pswd
#define RECHEAD 28                           // en-tete strSD date/heure/xx/yy + <br> + crlf
#define RECCHAR NBVAL*(MEANVAL+3)+RECHEAD+8  // longueur maxi record histo
#define UDPBUFLEN NBVAL*(MEANVAL+12)+6+1 //LENCDEHTTP // longueur maxi buffer paquet UDP

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
#define PULSEBLINK 20
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

#define MESSOK    1
#define MESSTO    0
#define MESSDEC  -1
#define MESSCRC  -2
#define MESSLEN  -3
#define MESSFON  -4
#define MESSFULL -5
#define MESSSYS  -6
#define MESSCX   -7
#define MESSNUMP -8
#define MESSMAC  -9

#define NBMESS   10  // OK ne compte pas

#define TEXTOK      "*OK_"
#define TEXTTO      "*TO_"
#define TEXTDEC     "*OVF"
#define TEXTCRC     "*CRC"
#define TEXTLEN     "*LEN"
#define TEXTFON     "*FON"
#define TEXTFULL    "*FUL"
#define TEXTSYS     "*SYS"
#define TEXTCX      "*CX_"
#define TEXTNUMP    "*NUP"
#define TEXTMAC     "*MAC"

//#define TEXTMESS TEXTOK,TEXTTO,TEXTDEC,TEXTCRC,TEXTLEN,TEXTFON,TEXTFULL,TEXTSYS,TEXTCX,TEXTNUMP,TEXTMAC
#define TEXTMESS " OK \0*TO_\0*OVF\0*CRC\0*LEN\0*FON\0*FUL\0*SYS\0*CX_\0*NUP\0*MAC\0"

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



/* codes staPulse */

#define LENTSP 4

#define PM_DISABLE  0x06
#define PM6 'DISA'
#define PM_RUN2     0x05
#define PM5 'RUN2'
#define PM_RUN1     0x04
#define PM4 'RUN1'
#define PM_END2     0x03
#define PM3 'END2'
#define PM_END1     0x02
#define PM2 'END1'
#define PM_IDLE     0x01
#define PM1 'IDLE'

/* pulseMode */

#define PCTLLEN (((NBPULSE*PCTLBIT)/8)+1) //  4*3 bits =12 -> 2 bytes/perif
#define PCTLBIT 3                         // 3 bits/pulse

#define PMTOE_VB 0x04           // time one enable valeur du bit
#define PMTOE_PB 0x02           //                 position du bit (nombre shifts)
#define PMTTE_VB 0x02           // time two enable
#define PMTTE_PB 0x01
#define PMFRO_VB 0x01           // freeRun/OneShoot
#define PMFRO_PB 0x00

/* bits controle compteurs */

#define PM_FREERUN  0
#define PM_ONESHOOT 1
#define PM_ACTIF    1


/* définition table inputs */

#define NBPERINPUT 24
#define PERINPLEN  4

// byte 0 & byte 3 -- num détecteur + type (source/dest)

#define PERINPV_MS    0xFC          // mask n° det (6 bits - 64 possibles)
#define PERINPNVMS_VB 0x80          // détec MSb
#define PERINPNVMS_PB 0x07
#define PERINPNVLS_VB 0x04          // détec LSb
#define PERINPNVLS_PB 0x02
#define PERINPNT_MS   0x03          // mask type (2 bits - 4 possibles)
#define PERINPNTMS_VB 0x02          // type MSb
#define PERINPNTMS_PB 0x01
#define PERINPNTLS_VB 0x01          // type LSb
#define PERINPNTLS_PB 0x00

// byte 1 -- règles --- inutilisé

#define PERINPRULESLS_VB 0x01       // rules LSb
#define PERINPRULESLS_PB 0x00

// byte 2 -- act/usage/level/enable

#define PERINPACT_MS    0xF0        // mask action - 4 bits (16 possibles)
#define PERINPACTMS_VB  0x80        // action MSb (4 bits)
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
#define DETYPHY 2
#define DETYPUL 3
#define DETYSW  2
#define DETYMEM 0


/* codes et libellés actions */

#define LENTACT 5            // len libellé

#define PMDCA_VIDE  0x00     //
#define ACT0 '_____'
#define PMDCA_RAZ   0x01     // raz
#define ACT1 'R','A','Z',' ',' '
#define PMDCA_STOP  0x02     // stop  clk
#define ACT2 'STOP '
#define PMDCA_START 0x03     // start clk
#define ACT3 'START'
#define PMDCA_SHORT 0x04     // short pulse
#define ACT4 'SHORT'
#define PMDCA_END   0x05     // end pulse
#define ACT5 'END  '
#define PMDCA_IMP   0x06     // start/stop impulsionnel
#define ACT6 'IMP  '
#define PMDCA_RESET 0x07     // reset
#define ACT7 'RESET'
#define PMDCA_LXOR  0x08     // toggle dest
#define ACT8 'LXOR '
#define PMDCA_LOR   0x09     // logical or dest
#define ACT9 'OR   '
#define PMDCA_LAND  0x0A     // logical and dest
#define ACT10 'AND  '
#define PMDCA_LNOR 0x0B     //
#define ACT11 'N','O','R','_','_'
#define PMDCA_VIDE  0x0C     //
#define ACT12 '_____'
#define PMDCA_VIDE  0x0D     //
#define ACT13 '_____'
#define PMDCA_VIDE  0x0E     //
#define ACT14 '_____'
#define PMDCA_VIDE  0x0F     //
#define ACT15 '_____'

#define MAXACT 15

#endif  _SHCONST_H_
