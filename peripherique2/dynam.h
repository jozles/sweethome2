#ifndef DYNAM_H_INCLUDED
#define DYNAM_H_INCLUDED

#if POWER_MODE==NO_MODE



void pulseClkisr();             // interrupt ou poling clk @10Hz



// gestion interruptions détecteurs
//void isrDet(uint8_t det);       // setup memDetec et staPulse après interruption sur det
//void isrD0();
//void isrD1();
//void isrD2();
//void isrD3();
//void initIntPin(uint8_t det);
//void initPolPin(uint8_t det);
void polDx(uint8_t det);
void polAllDet();
void swDebounce();
void memdetinit();
void swAction();

#endif NO_MODE


/*
Mode d'emploi de la peritable :

  pour chaque switch 3 colonnes : 
    1) les compteurs avec bit d'enable et de free-run
    2) le contrôle des détecteurs logiques : 
        4 lignes pour 4 détecteurs avec : 
        enable, local, n° physique (local) ou externe, transitionnel ou statiqiue, actif haut/bas, n° action éventuelle
        (un détecteur est actif si : il est enable, local (externe à dev), son niveau correspond)
                actions :
                  0) reset : l'action reset remet les 2 compteurs à 0 en mode idle selon mode/état/flanc programmé 
                  1) raz : l'action raz remet à 0 le compteur courant sans effet sur l'horloge selon mode/état/flanc programmé
                  2) stop : l'action stop suspend l'horloge selon l'état/flanc programmé
                  3) start : l'action start déclenche l'horloge selon l'état/flanc programmé
                  4) short : l'action short termine le compteur courant sans changer la période totale (le compteur suivant est augmenté)
                  5) fin : l'action fin termine le compteur courant. 
                  6) stop impulsionnel : stop si le compteur depuis le début a moins de DETIMP (1,5sec) start sinon 
                        (impDetTime=millis() si start, =0 si stop ou stop impulsionnel)
                  7) toggle switch change l'état du(ou des) switch(s) utilisant le détecteur (voir const.h pour les^pb de mise en oeuvre)
                        
    3) le contrôle des switchs :
        (les lignes A et D sont inutilisées et sans effet)
        ligne O pour OFF : 3 sources qui peuvent engendrer l'état OFF
        ligne I pour ON  : 3 sources qui peuvent engendrer l'état ON
        
        1ère source : un détecteur logique : numéro (0 à 3) parmi les 4 de la colonne de contrôle des dl, bit enable, bit H/L
        2nde source : le serveur : bit enable, bit H/L
        3ème source : le générateur d'impulsion : bit enable, bit H/L

Modification à faire : la source serveur devient la source remote : ajouter 4 bits remote pour 4 switchs (cstRec.remote et ack/set) ; 
                       les remotes fonctionnent en OU pour l'allumage (plusieurs remotes possibles sur le même switch)
                       periSwVal devient un disjoncteur qui coupe tout
                       ajout d'un écran de disjoncteurs et forçages d'allumages pour les switchs (1 ligne avec 2 inters par switch)
                       3 lignes deviennent valides dans le contrôle des switchs : 
                            I forçage ON 
                            O forçage OFF (si pas I)
                            X On (si pas O)
                            
exemple : positionner le switch 0 selon l'état du détecteur local 2 (P4 de la carte VR) et switch 1 à l'inverse
          (contrôle des détecteurs / conrôle des switchs)    (contrôle des détecteurs / conrôle des switchs)    (x coché, _ vide)
                  xx2_x0                I 1 xx____                  xx2__0                I 0 x_____                  
                  xx2__0                O 2 x_____                  xx2_x0                O 1 xx____
                  __0__0                                            __0__0
                  __0__0                                            __0__0   

exemple : positionner le switch 0 selon l'état du bit de commande serveur 
          (contrôle des détecteurs / conrôle des switchs)    (x coché, _ vide)
                  _____0                I 0 __xx__                  
                  _____0                O 0 __x___       

exemple : faire clignoter le switch 0 sur déclenchement du détecteur local 2 au passage haut
           compteurs  freerun          (contrôle des détecteurs / conrôle des switchs)    (x coché, _ vide)
             x  4        x                     xx2_x3                I 0 ____xx                  
             x  4                              __0__0                O 0 ____x_       
                                               __0__0
                                               __0__0
 
exemple : volets roulants avec détecteurs 0 et 1 (sur inter on off on P12 carte VR)
           compteurs  freerun          (contrôle des détecteurs / conrôle des switchs)    (x coché, _ vide)
             x  0        _                     xx0__3                I 0 __xxxx                  
             x  12                             xx0_x6                O 0 __x_x_       
                                               xx1__1
                                               __0__0

exemple : déclencher avec timer 2 (utilisation du détecteur serveur 3(0-7)) 
          timers
          n° nom     det hdeb   hfin    e p c f 7lmmjvsd dhdeb   dhfin
          2  exemple 3   hhmmss hhmmss  1 1     11111111 0000... 9999....   le bit e déclenche le détec au passage à 1
                                                                            mais est sans effet au passage à 0            

         (contrôle des détecteurs / conrôle des switchs)    (x coché, _ vide)  (utilisation dl 2)
                  __0__0                I 2 xx____                  
                  __0__0                O 2 x_____       
                  x_3__0
                  __0__0

            

*/         
 




#endif // DYNAM_H_INCLUDED
