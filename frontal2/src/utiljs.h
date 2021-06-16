#ifndef _UTIL_JS_H_
#define _UTIL_JS_H_

#define SEP   VRAI
#define SEPNO FAUX

#define ALIC   VRAI
#define ALICNO FAUX
#define NOBR   FAUX
#define BR     VRAI
#define HID  (bool)true
#define CRLF (bool)true
#define LF     0x0A

/* bits de ctl */
#define CTLCH  0x40     
#define CTLPO  0x20     // byte police present
#define BRMASK 0x10
#define BRYES  0x10
#define BRNO   0
#define TRMASK 0x0c     
#define TRBEG 0x04      // délicat à modifier (voir les ~TRBEG)   
#define TREND 0x08      // délicat à modifier (voir les ~)   
#define TRBE  0x0C      // délicat à modifier (voir les ~)
#define TRNO  0
#define TDMASK 0x03
#define TDEND 0x02      // délicat à modifier (voir les ~)
#define TDBE  0x03      // délicat à modifier (voir les ~)
#define TDBEG 0x01      // délicat à modifier (voir les ~)
#define TDNO  0


//#define PV    VRAI
//#define NOPV  FAUX
//#define JSB strcat(jsbuf,
//#define JSE );

/* commandes interprétées par javascript pour produire du html 
   de la forme JSAAAA codées '{' + 1 caractère code de la commande [+ 1 car ctl voir ci-dessus)][+1 car police][+n car paramètres[}...]]
*/
#define JSFON  "{"        // séparateur début fonction  
#define JSSEP  "}"        // séparateur interne aux fonction
#define JSSBR  "|"        // séparateur colonne dans les chaines texte
#define JSLF   "~"        // séparateur <br> dans les chaines texte 
#define JSCHK  "^"        // checked 
#define JSCTL  "x"        // troisieme car optionnel de fonction (reçoit ctl) 

/* commande Js (les commandes non commentées sont inutilisées) */
#define JSCOB  "wx"       // couleur                 JSCOBcouleur;
#define JSCOE  "W "
#define JSFNB  "xx"       // police beg 
#define JSFNE  "X "       // police fin

#define JSHIDB "hx"       // hide 
#define JSHIDE "H "   
#define JSAC   "V "       // align center

#define JSBRB  "rx"       // bouton retour 
#define JSBRE  "R "
#define JSBMB  "mx"       // bouton Maj 
#define JSBME  "M "
#define JSBFB  "bx"       // bouton fonct            *JSBFBnomfonct}valfonct}size}lib
#define JSBFE  "B "
#define JSNTB  "nx"       // saisie numtf            *JSNTBnomfonct}len}dec}typevaleur

#define JSDB   "dx"       // saisie cb               JSDBnomfonct}lib}[JSCHK][etat]
#define JSDE   "D "       // saisie cb fin  
#define JSATB  "ax"       // saisie alphaTableHtmlB  *JSATBnomfonct}valfonct}len
#define JSSP   "S "       // affichage space  
#define JSST   "sx"       // affichage texte         JSSTtexte
#define JSNT   "Ux"
#define JSNTI  "ux"       // affichage num (min/max) JSNTXvalfonct/100

#define JSUSR  "r "       // usrPeriCurB
#define JSTB   "t "       // debut table
#define JSTE   "T "       // fin table (crlf manuel dans jscat)
#define JSSTB  "kx"       // selectTable                JSSTBnfonc}nom_options n=n°sel+PMFNCVAL
#define JSSOP  "K "       // selectTable options table  JSCELvnloptions v=nom n=nbre+PMFNCVAL l=len+PMFNCVAL
#define JSFBH  "gx"       // header formulaire
#define JSFB   "fx"       // début formulaire [titre si encadrement]
#define JSFE   "Fx"       // fin formulaire

void buftxcat(char* buf,char* txt);

#endif // _UTIL_JS_H_
