#ifndef DYNAM_H_INCLUDED
#define DYNAM_H_INCLUDED

#if POWER_MODE==NO_MODE



void setPulseChg(uint8_t npu,char timeOT);
void pulseClk();             // interrupt ou poling clk @10Hz



// gestion interruptions détecteurs
//void isrDet(uint8_t det);       // setup memDetec et staPulse après interruption sur det
//void isrD0();
//void isrD1();
//void isrD2();
//void isrD3();
//void initIntPin(uint8_t det);
//void initPolPin(uint8_t det);

/*
      pulseClk     coup d'horloge des pulses
 
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
void toogleBreaker(uint8_t sw);
void openBreaker(uint8_t sw);
void closeBreaker(uint8_t sw);
void memdetinit();
void pulsesinit();
void actions();
void actionsDebug();

#endif // NO_MODE






#endif // DYNAM_H_INCLUDED
