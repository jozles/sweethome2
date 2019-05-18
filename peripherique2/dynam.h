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

/*
      pulseClkisr     coup d'horloge des pulses
 
      polAllDet fait le polling des détecteurs avec filtrage des débounce en cours et appelle polDx pour chaque détecteur
      polDx     si le détecteur a changé, maj memDetec + isrPul
      isrPul    recherche l'utilisation du détecteur dans les inputs et exécute l'action

      swDebounce maj des compteurs de debounce pour les détecteurs locaux
      memdetinit initialise détecteurs locaux et pulse à 0 à la mise sous tension
      pulsesinit initialise les générateurs à 0 mode IDLE

      swAction  fait le polling des inputs pour positionner les switchs selon les règles
 */



void polDx(uint8_t det);
void polAllDet();
void swDebounce();
void memdetinit();
void pulsesinit();
void actions();

#endif NO_MODE


/*
       stop impulsionnel : stop si le compteur depuis le début a moins de DETIMP (1,5sec) start sinon 
                        (impDetTime=millis() si start, =0 si stop ou stop impulsionnel)
                        
                            
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
