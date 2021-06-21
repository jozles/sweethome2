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
#define JSSCO  "|"        // séparateur colonne dans les chaines texte  
#define JSSBR  "~"        // séparateur <br> dans les chaines texte     
#define JSCHK  "^"        // checked 
#define JSCTL  "x"        // troisieme car optionnel de fonction (reçoit ctl) 

/* commande Js (les commandes non commentées sont inutilisées) */
/* attributs */
#define JSCOB  "wx"       // couleur Beg             JSCOBcouleur;
#define JSCOE  "W "       // couleur End
#define JSFNB  "xx"       // police beg 
#define JSFNE  "X "       // police fin
#define JSHIDB "hx"       // hide beg
#define JSHIDE "H "       // hide end
#define JSAC   "v "       // align center beg
#define JSACE  "V "       // align center end
/* boutons */
#define JSBRB  "rx"       // bouton retour
#define JSBMB  "mx"       // bouton Maj 
#define JSBFB  "bx"       // bouton fonct            *JSBFBnomfonct}valfonct}size}lib
/* saisies */
#define JSNTB  "nx"       // saisie numtf            *JSNTBnomfonct}dec}typevaleur}size}maxlen
#define JSDB   "dx"       // saisie cb               JSDBnomfonct}lib}[JSCHK][etat]     
#define JSATB  "ax"       // saisie alphaTableHtmlB  *JSATBnomfonct}valfonct}len
#define JSSTB  "kx"       // saisie selectbox        JSSTBnnomfonc}nom_options n=n°sel+PMFNCVAL
#define JSSOP  "K "       // selectbox options table JSCELvnloptions v=nom n=nbre+PMFNCVAL l=len+PMFNCVAL
/* affichages */
#define JSSP   "S "       // affichage space
#define JSCLB  "( "       // parenthese ouverte
#define JSCLE  ") "       // parenthese fermée 
#define JSST   "sx"       // affichage texte         JSSTtexte
#define JSNT   "Ux"       // affichage num           JSNTtexte
#define JSNTI  "ux"       // affichage num (min/max) JSNTcouleur}valfonct/100 
/* structures */
#define JSTB   "t "       // debut table
#define JSTE   "T "       // fin table (crlf manuel dans jscat)
#define JSFBH  "gx"       // Intro formulaire
#define JSFB   "fx"       // début formulaire [titre si encadrement]
#define JSFE   "Fx"       // fin formulaire
#define JSUSR  "r "       // usrPeriCurB

void buftxcat(char* buf,char* txt);

#endif // _UTIL_JS_H_
