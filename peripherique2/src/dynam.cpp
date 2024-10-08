
#include "const.h"
#include "Arduino.h"
#include "shconst2.h"
#include "shutil2.h"
#include "dynam.h"
#include "util.h"
#include "peripherique2.h"

extern const char* chexa;
extern uint8_t bitMsk[];

#ifdef PWR_CSE7766
extern unsigned long powonDly[NBSW];
#endif // CSEPOWER

bool oneTime=true;
//#define DEBUG_ACTIONS       // construction d'une chaine qui décrit chaque action et son résultat
#ifndef DEBUG_ACTIONS
#define PUV2 
#endif 
#ifdef DEBUG_ACTIONS
#define PUV2 puv[2]=chexa[staPulse[ndest]];
#define LDA 1000
extern char inptyps[]; //="52meexphpu??";
extern char inptypd[]; //="52meexswpu??";
extern char inpact[];  //={"@5     RAZ  STOP STARTSHORTEND  IMP  RESETXOR  OR   AND  NOR  NAND -0-  -1-  SET  "};    // libellés options actions
char oldDebugAction[LDA];
char curDebugAction[LDA];
uint8_t cntDBA=0;
char cdga[3];
char typs[3];
char typd[3];
uint8_t dba=0;
char* cda(uint8_t val)
{
  memset(cdga,0x00,3);
  cdga[0]=val/10+0x30;cdga[1]=val-(val/10)*10+0x30;
  return cdga;
}
char* cds(uint8_t val)
{
  memset(cdga,0x00,3);
  cdga[0]=inptyps[val*2];cdga[1]=inptyps[val*2+1];
  return cdga;
}
char* cdd(uint8_t val)
{
  memset(cdga,0x00,3);
  cdga[0]=inptypd[val*2];cdga[1]=inptypd[val*2+1];
  return cdga;
}
char* cdx(uint8_t val)
{
  memset(cdga,0x00,3);
  cdga[0]=inpact[val*5];cdga[1]=inpact[val*5+1];
  return cdga;
}

#endif

#ifdef CAPATOUCH
#include <capaTouch.h>          // décommenter le path dans platformio.ini
extern Capat capaKeys;
#endif // CAPATOUCH

#if POWER_MODE==NO_MODE

extern uint32_t locmem;         // mémoire = valeurs locales pour partiels
extern uint8_t outSw;           // état à appliquer aux switchs (produit par "actions()")

//extern bool oneShow;
extern bool dataParFlag;

extern constantValues cstRec;

extern  uint8_t pinSw[MAXSW];                                  // les switchs
extern  uint8_t toogSw;                                        // n° du sw en toogle dans pinSw[]
extern  byte    staPulse[NBPULSE];                             // état clock pulses
uint32_t  cntPulseOne[NBPULSE]; // 16   temps debut pulse 1
uint32_t  cntPulseTwo[NBPULSE]; // 16   temps debut pulse 2
uint32_t  cntPulse[NBPULSE*2]; // 32   temps restant après STOP pour START
  
extern  uint8_t pinDet[MAXDET];

extern  unsigned long    detTime[MAXDET];                      // debounce détecteurs physiques
extern  unsigned long    impDetTime[NBPULSE];                  // timer pour gestion commandes impulsionnelles     
extern  unsigned long    isrTime;
extern  void    (*isrD[4])(void);                              // tableau des pointeurs des isr détecteurs
extern  byte    mask[];

//extern  int     cntdebug[];
//extern  unsigned long    timedebug[]={0,0,0,0};
extern  int*    int0;

extern uint32_t  locMaskbit[];

extern uint8_t openSw[],cloSw[],valSw[];

byte oldCstCde; // memo swCde pour debug



/* ------------------ généralités -------------------- 

  switchs :

  (pinSw[] est la table des pins associés aux switchs)
  la table des règles est pollée via actions() maxi NBPERRULES(48) règles
  (l'état du disjoncteur du switch (forcé,enable,coupé) est appliqué au résultat dans outputCtl())
  outSW reçoit les résultats de tous les switchs (1 bit par switch)
  (la variable mskSw limite à 4 le nombre de switchs)
  (le flag DEBUG_ACTIONS permet de suivre l'évolution et le résultat du polling)

  elles sont exécutées dans l'ordre ce qui a un effet sur le résultat !
  via la source/destination "mémoire" le résultat d'une action peut être la source d'une autre
  les positions de "mémoire" sont le résultat des opérations logiques effectuées par les actions

  Les règles comprennent :
    La source (type/n°) pour identifier un détecteur (physique, mémoire, externe, pulse) 
    Les paramètres de fonctionnement (enable/edge-static/active level/old level/action) 
    la destination (type N°) pour identifier ce sur quoi agir (switch, mémoire, détecteur externe, pulse)

  l'action indique ce qui doit être fait lorsque les paramètres appliqués à la source matchent.
  A partir de la version 1.q les nouvelles actions "0" et "1" sont disponibles qui forcent le résultat
  indépendament des conditions d'entrée. La valeur de destination devient le résultat de l'opération logique
  entre la valeur évaluée "en cours" et la valeur trouvée via la règle. 
  Les actions "0" et "1" permettent d'initialiser la valeur "en cours".
  Certaines actions concernent exclusivement les générateurs de pulses, 
  les autres actions pour switchs, mémoire et externes.
  
  détecteurs (sources) :
  4 types : 
    physiques (accédés via memDetec), 
    externes (accédés via cstRec.extDetec), 
    mémoire (variable locale lors du polling dans actions()), 
    pulses via staPulse
  
  les détecteurs physiques sont pollés et la table memDetec est mise à jour avec gestion du debounce.
  fonctions swDebounce(), polAllDet() et polDx()
    
  actions :
      (pulses)
      - start         active le cnt!=0 ou cnt1 si tout==0 ; stapulse devient run1 ou run2 
      - stop          suspend le comptage ; stapulse idle
      - raz           les 2 compteurs=0 ; stapulse idle
      - reset         les 2 compteurs et les 2 durées =0 ; stapulse idle
      - fin           compteur courant au max puis avance compteur ; si bloqué -> stapulse end1 ou 2 sinon suite normale
      - short         compteur courant au max ; stapulse inchangé
      - impulsion     si impDetTime < DETIMP effectue raz sinon sans effet
      (autres)
      - toogle        inverse la valeur "en cours" si la condition d'entrée est validée (anciennement - inverse l'état de la destination)
      - OR            effectue OR de l'entrée avec la valeur "en cours" (anciennement - force à 1 la destination)
      - NOR           effectue NOR de l'entrée avec la valeur "en cours" (anciennement - force à 0 la destination)
      - XOR           effectue XOR de l'entrée avec la valeur "en cours" 
      - AND           effectue AND de l'entrée avec la valeur "en cours"
      - NAND          effectue NAND de l'entrée avec la valeur "en cours"
      - -0-           force la valeur "en cours" à 0 si la source vaut 1
      - -1-           force la valeur "en cours" à 1 si la source vaut 1     
  
  variables des pulses et mécanismes :

  Chaque pulse est constitué d'un couple de compteurs (1 par phase), de bits de mode de fonctionnement et d'un état courant
  Les pulses sont déclenchés/arrêtés de façon asynchrone par les actions lues dans les règles (donc au rythme du polling des règles)
  Les pulses changent d'état lors des fins de comptage détectées par pulseClk() 

  cstRec.durPulseOne[NBPULSE] / cstRec.durPulseTwo[NBPULSE] consignes de durée issues du serveur (0 au reset)
  cstRec.pulseMode (NBPULSE fois 3 bits) (F=free run / One enable / Two enable) consignes de mode de fonctionnement issues du serveur
  cntPulseOne[NBPULSE] / cntPulseTwo[NBPULSE] mémorise millis() du start ; 0 si inactif (0 au reset)
  cntPulse[NBPULSE*2]  alternativement pour one et two ; mémoire de l'état du stop pour reprendre avec le restant de temps lors d'un start
  staPulse[NBPULSE] est l'état des couples de compteurs (IDLE au reset)
    débranché (DISABLE) en cas d'erreur système (action invalide d'un détecteur)
    suspendu  (IDLE) suite à une action STOP ou en fin de oneshot
    comptage  (RUN)  suite à une action START, RUN1 si compteur 1 en cours, RUN2 si compteur 2
    fin       (END)  lorsque le comptage d'une phase est terminé et que la phase suivante est débranchée 
  staPulse est mis à jour par isrPulse() et est modifiable par les actions

*/

void actionsDebug()
{
#ifdef DEBUG_ACTIONS
  memset(curDebugAction,0x00,LDA);
  Serial.println("(SS LL  S switchs ; L locmem ");
  Serial.println(" for each enabled rule h-ss:nn,dd:nn,AC_1.2=DcLL[,pP] ");
  Serial.println("      h=curinp+2 hexa ; ss srce ; dd dest ; AC action ; 1 detec1 ; ");
  Serial.println("      2 detec2 ; D detecState final ; c curValue ; LL locmem ;  p pulse srce ; P pulse dest ");
  Serial.println(" SS O)millis() ; O=outSw (req value for switchs)");
  Serial.println(" RUN1=4 RUN2=5 END1=2 END2=3 IDLE=1");
#endif //DEBUG_ACTIONS

}

void actionSysErr(uint8_t action,int inp)
{
  Serial.print(" syserr=");Serial.print(action,HEX);Serial.print("/");Serial.print(inp);
}

void actions()          // pour chaque input, test enable,
{                       //      récup valeur source (detecState) 
                        //      si action logique, exécution dest,cur=action(source,cur)
                        //      si modif pulse ou 0/1, exécution si detecState=1
  
  byte*   curinp;           // adresse cur input
  uint8_t detecState=0;     // valeur trouvée pour la source (type-n°) (0==OFF ; 1==ON)
  uint8_t detecFound=0;     // flag : valeur valide si !=0
  uint8_t nsrce;            // n° source
  uint8_t ndest;            // n° destination
  byte    tdest;            // type destination
  byte    tsrce;            // type source  
  uint8_t curValue=0;       // valeur en cours pour l'évaluation des règles (0==OFF ; 1==ON)
  uint32_t lmbit0;          // valeur locmem modifiée par 0
  uint32_t lmbit1;          // valeur locmem modifiée par 1  

  /* pour actions OR/NOR/AND/NAND */
  uint8_t curSw[MAXSW];memset(curSw,0x00,MAXSW);                                   // valeur courante des SW pendant la lecture des règles (0 au départ)
  for(uint8_t s=0;s<MAXSW;s++){if(digitalRead(pinSw[s])==cloSw[s]){curSw[s]=1;}}   // initialise l'état des switchs
  uint8_t usdSw[MAXSW];memset(usdSw,0x00,MAXSW);                                   // devient 1 si le SW est modifié par une règle
       
#ifdef DEBUG_ACTIONS
  char puv[4]={0x00,0x00,0x00,0x00};
  memset(curDebugAction,0x00,LDA);
  char lmb[2]={0x00,0x00};
  strcat(curDebugAction,"(");
  lmb[0]=chexa[cstRec.swCde>>4];strcat(curDebugAction,lmb);
  lmb[0]=chexa[cstRec.swCde&0x0F];strcat(curDebugAction,lmb);
  strcat(curDebugAction," ");
  lmb[0]=chexa[(locmem>>4)&0x0F];strcat(curDebugAction,lmb);
  lmb[0]=chexa[locmem&0x0F];strcat(curDebugAction,lmb);
#endif // DEBUG_ACTIONS

  for(int inp=0;inp<NBPERRULES;inp++){

    curinp=&cstRec.perInput[inp*PERINPLEN];                       // règle courante
    if(((*(curinp+2))&PERINPEN_VB)!=0){                           // enable
      
      nsrce=(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);              // numéro source
      tsrce=(*curinp)&PERINPNT_MS;                                // type source
      ndest=(((*(curinp+3))&PERINPV_MS)>>PERINPNVLS_PB);          // numéro destination
      tdest=(byte)((*(curinp+3))&PERINPNT_MS);                    // type destination
      
      byte action=(*(curinp+2))>>PERINPACTLS_PB;      
      
      // précalcul locmem[ndest] (au cas où tdest = mem)
      lmbit0=locmem & ~(locMaskbit[ndest]);      // locmem avec result 0 : ~(locMaskbit[ndest]) 11...101...11 masque du bit ndest
      lmbit1=locmem | locMaskbit[ndest];         // locmem avec result 1 :   locMaskbit[ndest]  00...010...00 masque du bit ndest   

      uint8_t openClose[]={openSw[ndest],cloSw[ndest]};           // open/close value for ndest switch

#ifdef DEBUG_ACTIONS
  puv[0]='\0';
  puv[1]='\0';
  puv[2]='\0';
  strcat(curDebugAction," ");
  lmb[0]=chexa[(*(curinp+2)&0x0F)];strcat(curDebugAction,lmb);
  strcat(curDebugAction,"-");
  strcat(curDebugAction,cds(tsrce));
  strcat(curDebugAction,":");
  strcat(curDebugAction,cda(nsrce));
#endif //DEBUG_ACTIONS

      /* évaluation source -> detecState (detecFound==1 if detecstate valid */
      switch(tsrce){                              // type source
        case DETYEXT:{
             uint8_t numbyte=nsrce>>3;
             uint8_t numbit=nsrce&0x07;
             detecState=(cstRec.extDetec[numbyte]>>numbit)&0x01;    // valeur détecteur externe 
             //detecState=(cstRec.extDetec>>nsrce)&0x01;    // valeur détecteur externe 
             detecFound=1;}break;
        case DETYPHY:detecState=(byte)(cstRec.memDetec[nsrce]>>DETBITLH_PB)&0x01;     // valeur détecteur physique
             detecFound=1;break;
        case DETYMEM:detecState=(locmem>>nsrce)&0x01;detecFound=1;break;              // valeur loc mem
        case DETYPUL:
#ifdef DEBUG_ACTIONS
  strcat(curDebugAction,"/");
  puv[0]=chexa[staPulse[nsrce]];
  strcat(curDebugAction,puv);
#endif //DEBUG_ACTIONS
              switch(staPulse[nsrce]){                     // pulse                
                 case PM_RUN1: detecState=1;detecFound=1;break;   // pulse run1=H     // état sortie MCU -> relais ON
                 case PM_RUN2: detecState=0;detecFound=1;break;   // pulse run2=L     // état sortie MCU -> relais OFF
                 case PM_END1: detecState=0;detecFound=1;break;   // pulse end1=L     // état sortie MCU -> relais OFF
                 case PM_END2: detecState=0;detecFound=1;break;   // pulse end2=L     // état sortie MCU -> relais OFF
                 case PM_IDLE: if((cntPulseOne[nsrce]!=0) || ((cntPulseOne[nsrce]+cntPulseTwo[nsrce])==0)){
                                 detecState=0;detecFound=1;}      // pulse idle,   cnt1 !=0 -> L ou cnt1+cnt2=0 -> L
                               else {detecState=1;detecFound=1;}  //               cnt2 !=0 -> H  
                               break;
                 default: break;
             }break;
        default:break;
      }

#ifdef DEBUG_ACTIONS
  //Serial.print(detecFound);     // devrait toujours être 1 (detecFound inutile) 
  strcat(curDebugAction,",");
  strcat(curDebugAction,cdd(tdest));
  strcat(curDebugAction,":");
  strcat(curDebugAction,cda(ndest));
  if(tdest==DETYPUL){
    strcat(curDebugAction,"/");
    puv[1]=chexa[staPulse[ndest]];
    strcat(curDebugAction,puv+1);}
  strcat(curDebugAction,",");
  strcat(curDebugAction,cdx(action));
  strcat(curDebugAction,"_");
  char cip[3];memset(cip,0x00,3);
  lmb[0]=(char)(detecState+0x30);strcat(curDebugAction,lmb);
  strcat(curDebugAction,".");
#endif // DEBUG_ACTIONS

      if(detecFound!=0){                                                          // if detecState valid

        /* update detecState (edge rise/fall ; static high/low) */
        if( (((*(curinp+2))&PERINPDETES_VB)!=0)                  
            &&((((*(curinp+2))>>(PERINPVALID_PB) )&0x01)==1)
           ){detecState^=0x01;}                                                   // if (static && inv) -> invert

        if( (((*(curinp+2))&PERINPDETES_VB)==0))                                  // if edge
          {

            /* if edge update detecState et curinp(oldlev) ; detecState ==1 if active edge only */
            if(oneTime){
              oneTime=false;
              Serial.print("state=");Serial.print(detecState);Serial.print(" curinp=");if(*(curinp+2)<16){Serial.print('0');}Serial.print(*(curinp+2),HEX);
              Serial.print(" ndest=");Serial.print(ndest);Serial.print(" tdest=");Serial.print(tdest);Serial.print(" action=");Serial.println(action);
              }
            if( (detecState!=(((*(curinp+2))>>(PERINPOLDLEV_PB))&0x01)) )         // if level chge (curr!=old)
              {
                *(curinp+2) &= ~PERINPOLDLEV_VB;                                  // raz bit oldlev
                *(curinp+2) |= (detecState << PERINPOLDLEV_PB);                   // màj bit oldlev

                if( (detecState!=(((*(curinp+2))>>(PERINPVALID_PB) )&0x01)) )     // if active edge (active=0 => rising donc detecState=1)
                  {detecState=1;}                                                 // active edge detected 
                else 
                  {detecState=0;}                                                 // wrong edge
                Serial.print("state=");Serial.print(detecState);Serial.print(" curinp=");if(*(curinp+2)<16){Serial.print('0');}Serial.print(*(curinp+2),HEX);
                Serial.print(" ndest=");Serial.print(ndest);Serial.print(" tdest=");Serial.print(tdest);Serial.print(" action=");Serial.println(action);
                oneTime=true;
              }
            else
              {detecState=0;}                                                     // no chge            
          }
          

#ifdef DEBUG_ACTIONS
    lmb[0]=(char)(detecState+0x30);strcat(curDebugAction,lmb);
#endif // DEBUG_ACTIONS

/* si action logique, exécution action(detecState,curValue) ; si modif pulse exécution si detecState=1 et màj curval selon état du pulse */
            switch(action){                                                       // action (compute curValue then store it depending of dest)
              case PMDCA_VIDE:break;
              case PMDCA_0:
                          if(detecState!=0){
                            curValue = 0;                           
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: locmem=lmbit0;                        // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[0];            // action '1' donc openClose(0)
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;
                            }
                          }
                          break;
              case PMDCA_1:
                          if(detecState!=0){
                            curValue=1;
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            cstRec.extDetec[numbyte]|=bitMsk[numbit];
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: locmem=lmbit1;                        // locmem=1
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[1];            // action '1' donc openClose(1)
                                            usdSw[ndest]=1;                                 
                                            break;                                         
                              default:break;
                            }
                          }
                          break;
              case PMDCA_LOR:
                          if(detecState!=0){
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if((curValue!=0) || (cstRec.extDetec[numbyte]&bitMsk[numbit])!=0){
                                              cstRec.extDetec[numbyte]|=bitMsk[numbit];
                                              curValue=1;}
                                            else {cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                              curValue=0;}
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if(curValue!=0 || (locmem&locMaskbit[ndest])!=0){
                                              locmem=lmbit1;curValue=1;}          // locmem=1
                                            else {locmem=lmbit0;curValue=0;}      // locmem=0
                                            break;              
                              case DETYSW:  curSw[ndest]=openClose[curValue];     // curValue -> curSw
                                            usdSw[ndest]=1;
                                            break;                                         
                              default:break;
                            }
                          }
                          break;
              case PMDCA_LNOR:
                          if(detecState!=0){
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if((curValue!=0) || (cstRec.extDetec[numbyte]&bitMsk[numbit])!=0){
                                              cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                              curValue=0;}
                                            else {cstRec.extDetec[numbyte]|=bitMsk[numbit];
                                              curValue=0;}
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if(curValue!=0 || (locmem&locMaskbit[ndest])!=0){
                                              locmem=lmbit0;curValue=0;}          // locmem=0
                                            else {locmem=lmbit1;curValue=1;}      // locmem=1
                                            break;              
                              case DETYSW:  curSw[ndest]=openClose[!curValue];    // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;              
                            }
                          }
                          break;          
              case PMDCA_LXOR:
                          if(detecState!=0){
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if(((curValue!=0) && (cstRec.extDetec[numbyte]&bitMsk[numbit])==0) ||
                                            ((curValue==0) && (cstRec.extDetec[numbyte]&bitMsk[numbit])!=0)){
                                              cstRec.extDetec[numbyte]|=bitMsk[numbit];
                                              curValue=1;}
                                            else {cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                              curValue=0;}
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if((curValue!=0 && (locmem&locMaskbit[ndest])==0) || 
                                            (curValue==0 && (locmem&locMaskbit[ndest])!=0)){
                                              locmem=lmbit1;curValue=1;}          // locmem=1
                                            else {locmem=lmbit0;curValue=0;}      // locmem=0
                                            break;              
                              case DETYSW:  curSw[ndest]=openClose[curValue];     // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;              
                            }
                          }
                          break;
              case PMDCA_LAND:
                          if(detecState!=0){
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if((curValue!=0) && (cstRec.extDetec[numbyte]&bitMsk[numbit])!=0){
                                              cstRec.extDetec[numbyte]|=bitMsk[numbit];
                                              curValue=1;}
                                            else {cstRec.extDetec[numbyte]&=~bitMsk[numbit];curValue=0;}
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if(curValue!=0 && (locmem & locMaskbit[ndest])!=0){
                                              locmem=lmbit1;curValue=1;}          // locmem=1
                                            else {locmem=lmbit0;curValue=0;}      // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];     // curValue -> curSw
                                            usdSw[ndest]=1; 
                                            break;
                              default:break;
                            }
                          }
                          break;                          
              case PMDCA_LNAND:
                          if(detecState!=0){
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if((curValue!=0) && (cstRec.extDetec[numbyte]&bitMsk[numbit])!=0){
                                              cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                              curValue=0;}
                                            else {cstRec.extDetec[numbyte]|=bitMsk[numbit];curValue=1;}
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if(curValue!=0 && (locmem & locMaskbit[ndest])!=0){
                                              locmem=lmbit0;curValue=0;}          // locmem=0
                                            else {locmem=lmbit1;curValue=1;}      // locmem=1
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[!curValue];    // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;
                              default:break;
                            }
                          }
                          break;
               case PMDCA_FORCE:     
                            curValue=detecState;
                            switch(tdest){                                        // type dest
                              case DETYEXT: {uint8_t numbyte=nsrce>>3,numbit=nsrce&0x07;
                                            if(detecState!=0 ){cstRec.extDetec[numbyte]|=bitMsk[numbit];}
                                            else cstRec.extDetec[numbyte]&=~bitMsk[numbit];
                                            }break;                               // transfert vers detServ à développer    
                              case DETYMEM: if(detecState!=0){locmem=lmbit1;}     // locmem=1
                                            else locmem=lmbit0;                   // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[detecState];   // detecState -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;
                            }
                            break;
               case PMDCA_STOP: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    if(staPulse[ndest]!=PM_END1 && staPulse[ndest]!=PM_END2){      // si arrêté ne rien toucher
                      if(staPulse[ndest]==PM_RUN1){
                        cntPulse[ndest*2]=millis()-cntPulseOne[ndest];}   // temps déjà écoulé pour repartir si un (re)start a lieu
                      if(staPulse[ndest]==PM_RUN2){
                        cntPulse[ndest*2+1]=millis()-cntPulseTwo[ndest];} // temps déjà écoulé pour repartir si un (re)start a lieu
                      staPulse[ndest]=PM_IDLE;
                      impDetTime[ndest]=0;
                    }
                    PUV2
                  }
                  break;
               case PMDCA_START: 
                  if(detecState==1)
                  {               
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    if(cntPulseOne[ndest]!=0){                      
                      cntPulseOne[ndest]=millis()-cntPulse[ndest*2]; // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN1;}                   
                    else if(cntPulseTwo[ndest]!=0){
                      cntPulseTwo[ndest]=millis()-cntPulse[ndest*2+1];  // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN2;}
                    else {staPulse[ndest]=PM_RUN1;cntPulseOne[ndest]=millis();}
                    impDetTime[ndest]=millis();
                    PUV2
                  }
                  break;
               case PMDCA_SHORT: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    if(staPulse[ndest]==PM_RUN1 || cntPulseOne[ndest]!=0){
                      cntPulseOne[ndest]=0;cntPulse[ndest*2]=0;}     // cstRec.durPulseOne[ndest]*10;}
                    else if(staPulse[ndest]==PM_RUN2 || cntPulseTwo[ndest]!=0){cntPulseTwo[ndest]=0;} // cstRec.durPulseTwo[ndest]*10;}
                    impDetTime[ndest]=0;
                    PUV2
                  }
                  break;                 
               case PMDCA_RAZ: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    cntPulseOne[ndest]=0;
                    cntPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    cntPulse[ndest*2]=0;
                    cntPulse[(ndest*2)+1]=0;
                    PUV2
                  }
                  break;               
               case PMDCA_RESET: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    cntPulseOne[ndest]=0;cstRec.durPulseOne[ndest]=0;
                    cntPulseTwo[ndest]=0;cstRec.durPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    cntPulse[ndest*2]=0;
                    cntPulse[(ndest*2)+1]=0;
                    PUV2
                  }
                  break;                 
               case PMDCA_IMP: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    if((millis()-impDetTime[ndest])<DETIMP){
                      staPulse[ndest]=PM_IDLE;
                      cntPulseOne[ndest]=0;
                      cntPulseTwo[ndest]=0;}
                    Serial.print("Time=");Serial.print(millis()-impDetTime[ndest]);Serial.print(" ");
                    impDetTime[ndest]=0;
                    PUV2
                  }
                  break;
               case PMDCA_END: 
                  if(detecState==1)
                  {
                    if(tdest!=DETYPUL){actionSysErr(action,inp);break;}
                    if(staPulse[ndest]==PM_RUN1 || cntPulseOne[ndest]!=0){
                      cntPulseOne[ndest]=0;
                      cntPulse[ndest*2]=0;
                      setPulseChg(ndest,'O');}
                    else if(staPulse[ndest]==PM_RUN2 || cntPulseTwo[ndest]!=0){
                      cntPulseTwo[ndest]=0;
                      cntPulse[(ndest*2)+1]=0;
                      setPulseChg(ndest,'T');}
                    impDetTime[ndest]=0;
                    PUV2
                  }
                  break;
               default:actionSysErr(action,inp);
                  if(tdest==DETYPUL){staPulse[ndest]=PM_DISABLE;}
                  break;                  
          }     // switch(action)
      }   // detecFound    

#ifdef DEBUG_ACTIONS
  strcat(curDebugAction,"=");
  lmb[0]=(char)(detecState+0x30);strcat(curDebugAction,lmb);
  lmb[0]=(char)(curValue+0x30);strcat(curDebugAction,lmb);
  lmb[0]=chexa[(locmem>>4)&0x0F];strcat(curDebugAction,lmb);
  lmb[0]=chexa[locmem&0x0F];strcat(curDebugAction,lmb);                
  if(puv[0]==0){puv[0]='0';}
  if(puv[1]==0 && puv[2]!=0){puv[1]='0';}
  strcat(curDebugAction,",");
  strcat(curDebugAction,puv);
#endif //DEBUG_ACTIONS
    }     // enable
  }       // next rule

  /* SW update */
  /*uint8_t mskSw[] = {0xfe,0xfb,0xef,0xbf};                           
  for(uint8_t i=0;i<NBSW;i++){                                    // 1 byte 4sw + 4disjoncteurs (voir const.h du frontal)
    if(usdSw[i]==1){
      cstRec.swCde &= mskSw[i];                                   // clear switch bit
      cstRec.swCde |= curSw[i]<<(2*i);                            // bit switchs (bits 7,5,3,1 pour switchs 3,2,1,0)  
      //if(cstRec.swCde!=oldCstCde){Serial.print("\n");Serial.print(millis());Serial.print("--->");Serial.print(oldCstCde,HEX);Serial.print(" ");Serial.println(cstRec.swCde,HEX);oldCstCde=cstRec.swCde;}
    }
  }
  */
  uint8_t mskSw[] = {0x01,0x02,0x04,0x08}; 
  outSw=0;
  // cursw résultat des actions
  for(uint8_t i=0;i<NBSW;i++){if(curSw[i]!=0){outSw|=mskSw[i];}}
 
//if(oneTime){Serial.print("curSw=");Serial.print(curSw[0],HEX);Serial.print("usdSw=");Serial.print(usdSw[0],HEX);Serial.print(" outSw=");Serial.println(outSw);}
#ifdef DEBUG_ACTIONS  
  strcat(curDebugAction," ");
  lmb[0]=chexa[cstRec.swCde>>4];strcat(curDebugAction,lmb);
  lmb[0]=chexa[cstRec.swCde&0x0F];strcat(curDebugAction,lmb);
  strcat(curDebugAction," ");
  lmb[0]=chexa[outSw&0x0f];strcat(curDebugAction,lmb);
  strcat(curDebugAction,")");
  if(memcmp(oldDebugAction,curDebugAction,LDA)!=0){
    Serial.println();Serial.print(curDebugAction);Serial.println(millis());
    memcpy(oldDebugAction,curDebugAction,LDA);
    memset(curDebugAction,0x00,LDA);
    //cntDBA++;if(cntDBA>10){while(1){yield();}}
  }
#endif //DEBUG_ACTIONS
}

/* ------------- gestion pulses ------------- */

void setPulseChg(uint8_t npu,char timeOT)     // traitement fin de temps 
                                          // timeOT ='O' fin timeOne ; ='T' fin timeTwo                                          
{
  uint16_t ctl;memcpy(&ctl,cstRec.pulseMode,PCTLLEN);ctl=ctl>>(npu*PCTLBIT);ctl&=0x0007;
  //Serial.print(" npu=");Serial.print(npu);Serial.print(" ctl=");dumpfield((char*)&ctl,2);Serial.print(' ');Serial.print(cntPulseOne[npu]);Serial.print(' ');Serial.print(cstRec.cntPulseTwo[npu]);
  if(timeOT=='O'){
        if((ctl&(uint16_t)PMTTE_VB)!=0){                                                           // cnt2 enable -> run2
          cntPulseOne[npu]=0;cntPulseTwo[npu]=(uint32_t)millis();
          cntPulse[npu*2]=0;
          staPulse[npu]=PM_RUN2;
          //Serial.print(' ');Serial.print(cstRec.cntPulseTwo[npu]);Serial.print(" PM_RUN2 ");
          }
        else {staPulse[npu]=PM_END1;//Serial.print(" PM_END1 ");
          }                                             // sinon fin1
  }
  else {                                                                                          // fin run2               
        cntPulse[(npu*2)+1]=0;                   
        if((ctl&(uint16_t)PMFRO_VB)==0){                                                          // oneshot -> idle
          cntPulseTwo[npu]=0;cntPulseOne[npu]=0;
          staPulse[npu]=PM_IDLE;
          //Serial.print(" PM_IDLE ");
          }
        else if((ctl&(uint16_t)PMTOE_VB)!=0){                                                     // free run && one enable -> run1
          cntPulseTwo[npu]=0;cntPulseOne[npu]=(uint32_t)millis();
          staPulse[npu]=PM_RUN1;
          //Serial.print(' ');Serial.print(cntPulseOne[npu]);Serial.print(" PM_RUN1 ");
          }
        else {staPulse[npu]=PM_END2;
          //Serial.print(" PM_END2 ");
        }                                   // free run bloqué -> fin2
  }
  //Serial.println();
}

void pulsesinit()                         // init pulses à la mise sous tension
{
  Serial.println("init pulses");
  
    memset(cntPulseOne,0x00,sizeof(cntPulseOne));
    memset(cntPulseTwo,0x00,sizeof(cntPulseTwo)); 
    memset(cstRec.durPulseOne,0x00,sizeof(cstRec.durPulseOne));
    memset(cstRec.durPulseTwo,0x00,sizeof(cstRec.durPulseTwo));
    memset(staPulse,PM_IDLE,sizeof(staPulse));
    memset(cstRec.pulseMode,0x00,PCTLLEN);
    memset(cntPulse,0x00,sizeof(cntPulse));
}

void pulseClk()                       // polling ou interruption ; contrôle de décap des compteurs : horloge des pulses
{  
  uint32_t currt;
  for(uint8_t npu=0;npu<NBPULSE;npu++){
    //Serial.print("pulseClk() npu=");Serial.print(npu);Serial.print( " staPulse=");Serial.println(staPulse[npu]);
    switch(staPulse[npu]){
      
      case PM_DISABLE: break;            // changement d'état quand le bit enable d'un compteur sera changé
      case PM_IDLE: break;               // changement d'état par une action
      case PM_RUN1: currt=((uint32_t)millis()-cntPulseOne[npu])/1000;
                    if(currt>cstRec.durPulseOne[npu]){   // (decap cnt1)
                      //Serial.print(currt);Serial.print(" ");Serial.print(cntPulseOne[npu]);Serial.print(" ");Serial.print(cstRec.durPulseOne[npu]);Serial.print(" ");
                      setPulseChg(npu,'O');
                    }break;
                    
      case PM_RUN2: currt=((uint32_t)millis()-cntPulseTwo[npu])/1000;
                    if(currt>cstRec.durPulseTwo[npu]){   // (decap cnt2)
                      //Serial.print(currt);Serial.print(" ");Serial.print(cstRec.cntPulseTwo[npu]);Serial.print(" ");Serial.print(cstRec.durPulseTwo[npu]);Serial.print(" ");
                      setPulseChg(npu,'T');
                    }break;
                    
      case PM_END1: cntPulseOne[npu]=0;break;  // changt d'état quand bit enable compteur 2 sera changé
      case PM_END2: cntPulseTwo[npu]=0;break;  // changt d'état quand bit enable compteur 1 sera changé ou changt freerun->oneshot
      default:break;
    }
  }
}

/* ------------- gestion détecteurs physiques ------------- */

void memdetinit()                         // init détecteurs physiques à la mise sous tension
{
  Serial.println("init détecteurs");
  
  PINCHK;
  for(uint8_t det=0;det<NBDET;det++){
    cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
    cstRec.memDetec[det] |= PINREAD(det)<<DETBITLH_PB;  // set bit LH 
    detTime[det]=millis();
  }
}

void toogleBreaker(uint8_t sw)
{
    const char* open  ="disj\0";
    const char* close ="force\0";
    const char* swState = open;
    cstRec.swCde&=0xfc<<(sw*2);         // si pas open -> disjoncté 
    if(digitalRead(pinSw[sw])==openSw[sw]){
      swState = close;
      cstRec.swCde|=0x02<<(sw*2);}      // force close
      #ifdef PWR_CSE7766
      powonDly[sw]=POWONDLY;
      #endif // CSEPOWER
    dataParFlag=true;
    Serial.print(" toogle sw ");Serial.print(sw);
    Serial.print('/');Serial.print(swState);
    Serial.print(" swCde ");Serial.print(cstRec.swCde,HEX);
}

void closeBreaker(uint8_t sw)
{
    if(digitalRead(pinSw[sw])==openSw[sw]){
      cstRec.swCde|=0x02<<(sw*2);       // force close (02)
      dataParFlag=true;
      #ifdef PWR_CSE7766
      powonDly[sw]=POWONDLY;
      #endif // CSEPOWER
      Serial.print(" close sw ");Serial.print(sw);
      Serial.print(" swCde ");Serial.print(cstRec.swCde,HEX);
    }
}

void openBreaker(uint8_t sw)
{
  //Serial.print("pinSw[");Serial.print(sw);Serial.print("]=");Serial.print(pinSw[sw]);Serial.print(" dR ");Serial.print(digitalRead(pinSw[sw]));Serial.print(' ');
    if(digitalRead(pinSw[sw])==cloSw[sw]){
      cstRec.swCde&=0xfc<<(sw*2);       // force open (00)
      dataParFlag=true;
      Serial.print(" open sw ");Serial.print(sw);
      Serial.print(" swCde ");Serial.println(cstRec.swCde,HEX);
    }
}

void polDx(uint8_t det)              // maj memDetec selon l'état du détecteur det (polDx masqué par tempo debounce) 
                                     // memDetec met le débounce en commun si plusieurs inputs
                                     // utilisent le même détecteur (seul bit utilisé : LH)
                                     // si le détecteur pilote un switch en toogle et flanc actif => maj outSw
{    
    byte lev=PINREAD(pinDet[det]); //digitalRead(pinDet[det]);
    if( ((byte)(cstRec.memDetec[det]>>DETBITLH_PB)&0x01) != lev ){    // niveau lu != niveau actuel de memDetec ?
      // level change -> update memDetec
      cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
      cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 
      detTime[det]=millis();                                          // arme debounce
      talkReq();                                                      // talkServer
      cstRec.serverTime=0;
      delay(1);
      Serial.print("  >>>>>>>>> det ");Serial.print(det);
      Serial.print(" pin ");Serial.print(pinDet[det]);
      Serial.print(" change to ");Serial.print(lev);
      delay(1);
      #ifdef TOOGBT
      if(pinDet[det]==TOOGBT && lev==TOOGLV){
        toogleBreaker(toogSw);
      }
      #endif // TOOGBT
      Serial.println();
    }
}

void polAllDet()                                        // maj de memDetec (via polDx) pour tous les détecteurs locaux
{                                                       // la tempo de debouce masque polDx
  PINCHK;
  for(uint8_t det=0;det<(NBDET);det++){if(detTime[det]==0){polDx(det);}}    // pas de debounce en cours  
}
 

void swDebounce()                     
{
  for(uint8_t det=0;det<NBDET;det++){
    if(detTime[det]!=0 && (millis()>(detTime[det]+TDEBOUNCE))){
      detTime[det]=0; 
    }
  }
}

/*/* --------------- interruptions détecteurs ----------------  

void isrDet(uint8_t det)      // setup memDetec et staPulse après interruption sur det
{
  unsigned long sht=micros();
  
  // setup memDetec 
  detTime[det]=millis();
  cstRec.memDetec[det] &= ~DETBITLH_VB;             // raz bit LH
  cstRec.memDetec[det] &= ~DETBITST_VB;             // raz bits ST
  cstRec.memDetec[det] |= DETTRIG<<DETBITST_PB;     // set bits ST (déclenché)
  if(pinDir[det]==(byte)HIGH){cstRec.memDetec[det] |= HIGH<<DETBITLH_PB;}
  else {cstRec.memDetec[det] |= LOW<<DETBITLH_PB;}
  pinDir[det]^=HIGH;                                // inversion pinDir pour prochain armement
  
  isrTime=micros()-sht;
}


void isrD0(){detachInterrupt(pinDet[0]);isrDet(0);}
void isrD1(){detachInterrupt(pinDet[1]);isrDet(1);}
void isrD2(){detachInterrupt(pinDet[2]);isrDet(2);}
void isrD3(){detachInterrupt(pinDet[3]);isrDet(3);}



void initIntPin(uint8_t det)              // enable interrupt du détecteur det ; flanc selon pinDir ; isr selon isrD
{                                         // setup memDetec

  Serial.print(det);Serial.println(" ********************* initIntPin");
  delay(1);
  cstRec.memDetec[det] &= ~DETBITLH_VB;            // raz bit LH
  cstRec.memDetec[det] &= ~DETBITST_VB;            // raz bits ST
  cstRec.memDetec[det] |= DETWAIT<<DETBITST_PB;    // set bits ST (armé)
  if(pinDir[det]==LOW){cstRec.memDetec[det] |= HIGH<<DETBITLH_PB;attachInterrupt(digitalPinToInterrupt(pinDet[det]),isrD[det], FALLING);}
  else {cstRec.memDetec[det] |= LOW<<DETBITLH_PB;attachInterrupt(digitalPinToInterrupt(pinDet[det]),isrD[det], RISING);}
}
*/



/*
void initPolPin(uint8_t det)         // setup détecteur det selon pulseCtl en fin de debounce ;
{                                     
  uint64_t swctl;

  Serial.print(det);Serial.print(" ********* initPolPin (");Serial.print((cstRec.memDetec[det]>>DETBITLH_PB)&0x01);Serial.print("->");

  cstRec.memDetec[det] &= ~DETBITST_VB;                                        // raz bits ST
  cstRec.memDetec[det] |= DETIDLE<<DETBITST_PB;                                // set bits ST (idle par défaut)
  byte lev=levdet(det);                                                        // levdet positionne DETBITST pour les det externes
                                                                               // donc après idle
  cstRec.memDetec[det] &= ~DETBITLH_VB;                                        // raz bits LH
  cstRec.memDetec[det] |= lev<<DETBITLH_PB;                                    // set bit  LH  = detec   

  Serial.print(lev);Serial.print(") time=");
   
  for(uint8_t sw=0;sw<NBSW;sw++){                                 // recherche des utilisations du détecteur dans le périphérique
    memcpy(&swctl,&cstRec.pulseCtl[sw*DLSWLEN],DLSWLEN);          // dans les bytes d'un switch
    for(uint8_t dd=0;dd<DLNB;dd++){                               // pour chaque dl       
      if(                                                                       // si...
           ((swctl>>(dd*DLBITLEN+DLEL_PB))&0x01==DLLOCAL)                       // local
        && ((swctl>>(dd*DLBITLEN+DLNLS_PB))&mask[DLNMS_PB-DLNLS_PB+1]==det)     // num ok
        && ((swctl>>(dd*DLBITLEN+DLENA_PB)&0x01)!=0)                            // enable
        )
        {
          cstRec.memDetec[det] &= ~DETBITST_VB;                                       // raz bits ST
          cstRec.memDetec[det] |= DETWAIT<<DETBITST_PB;                               // set bits ST (armé)
        }
    }
  }
  Serial.println(detTime[det]);  
  detTime[det]=0;
}
*/

#endif // NO_MODE
