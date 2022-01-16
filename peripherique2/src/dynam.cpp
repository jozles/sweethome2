
#include "const.h"
#include "Arduino.h"
#include "shconst2.h"
#include "shutil2.h"
#include "dynam.h"
#include "util.h"
#include "peripherique2.h"

//#define DEBUG_ACTIONS

#ifdef CAPATOUCH
#include <capaTouch.h>
extern Capat capaKeys;
#endif // CAPATOUCH

#if POWER_MODE==NO_MODE

extern uint32_t locmem;        // mémoire = valeurs locales pour partiels

extern constantValues cstRec;

extern  uint8_t pinSw[MAXSW];                                  // les switchs
extern  byte    staPulse[NBPULSE];                             // état clock pulses
extern  uint8_t pinDet[MAXDET];

extern  unsigned long    detTime[MAXDET];                      // debounce détecteurs physiques
extern  unsigned long    impDetTime[NBPULSE];                  // timer pour gestion commandes impulsionnelles     
extern  unsigned long    isrTime;
extern  void    (*isrD[4])(void);                              // tableau des pointeurs des isr détecteurs
extern  byte    mask[];

//extern  int     cntdebug[];
//extern  unsigned long    timedebug[]={0,0,0,0};
extern  int*    int0;

extern uint32_t  mDSmaskbit[];

extern uint8_t openSw[],cloSw[],valSw[];

byte oldCstCde; // memo swCde pour debug



/* ------------------ généralités -------------------- 

  switchs :

  les switchs sont actionnés par polling de la table des règles via actions()
  pinSw[] est la table les pins associés aux switchs

  dans la limite du nombre disponibles (24) une règle peut comporter autant d'actions que désiré
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
      - -0-           force la valeur "en cours" à 0
      - -1-           force la valeur "en cours" à 1      
  
  variables des pulses et mécanismes :

  Chaque pulse est constitué d'un couple de compteurs (1 par phase), de bits de mode de fonctionnement et d'un état courant
  Les pulses sont déclenchés/arrêtés de façon asynchrone par les actions lues dans les règles (donc au rythme du polling des règles)
  Les pulses changent d'état lors des fins de comptage détectées par pulseClkisr() 

  cstRec.durPulseOne[NBPULSE] / cstRec.durPulseTwo[NBPULSE] consignes de durée issues du serveur (0 au reset)
  cstRec.pulseMode (NBPULSE fois 3 bits) (F=free run / One enable / Two enable) consignes de mode de fonctionnement issues du serveur
  cstRec.cntPulseOne[NBPULSE] / cstRec.cntPulseTwo[NBPULSE] mémorise millis() du start ; 0 si inactif (0 au reset)
  cstRec.cntPulse[NBPULSE*2]  alternativement pour one et two ; mémoire de l'état du stop pour reprendre avec le restant de temps lors d'un start
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
  Serial.println("(n rules start,locmem");
  Serial.println("-01. next enabled rule,locmem if 0, if 1");
  Serial.println("[#] valid static rule");
  Serial.println("[*] valid edge rule");
  Serial.println("[!] invalid rule");
  Serial.println("a action nb");
  Serial.println("< OR action start");
  Serial.println("{ XOR action start");
  Serial.println("abcd detecState,srce,dest,curvalue ");
  Serial.println("> OR action end");
  Serial.println("} XOR action end");
  Serial.println("cl curvalue,locmem");
  Serial.println(") rules end");

#endif //DEBUG_ACTIONS

}

void actionSysErr(uint8_t action)
{
  Serial.print(" syserr=");Serial.print(action,HEX);Serial.print(" ");
}

void actions()          // pour chaque input, test enable,
{                       //      récup valeur détecteur
                        //      comparaison avec valeur demandée (et flanc éventuel)
                        //      maj destination selon résultat et màj oldlev
  
  byte*   curinp;           // adresse cur input
  uint8_t detecState=0;     // valeur trouvée pour le det source (type-n°) (0==OFF ; 1==ON)
  uint8_t detecFound=0;     // flag : valeur valide si !=0
  uint8_t nsrce;            // n° source
  uint8_t ndest;            // n° destination
  byte    tdest;            // type destination
  uint8_t curValue=0;       // valeur en cours pour l'évaluation des règles (0==OFF ; 1==ON)
  uint32_t lmbit0;          // valeur locmem modifiée par 0
  uint32_t lmbit1;          // valeur locmem modifiée par 1  

  /* pour actions OR/NOR/AND/NAND */
  uint8_t curSw[MAXSW];memset(curSw,0x00,MAXSW);     // valeur courante des SW pendant la lecture des règles (0 au départ)
  uint8_t usdSw[MAXSW];memset(usdSw,0x00,MAXSW);     // devient 1 si le SW est modifié par une règle

#ifdef DEBUG_ACTIONS
  Serial.print('(');Serial.print(locmem);
#endif //DEBUG_ACTIONS


  for(int inp=0;inp<NBPERINPUT;inp++){

    curinp=&cstRec.perInput[inp*PERINPLEN];                       // règle courante
    if(((*(curinp+2))&PERINPEN_VB)!=0){                           // enable
#ifdef DEBUG_ACTIONS
  Serial.print('-');
#endif //DEBUG_ACTIONS
      
      nsrce=(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);              // numéro source
      ndest=(((*(curinp+3))&PERINPV_MS)>>PERINPNVLS_PB);          // numéro destination
      tdest=(byte)((*(curinp+3))&PERINPNT_MS);                    // type destination
      
      lmbit0=locmem & ~(mDSmaskbit[ndest]);                      // locmem result 0
      lmbit1=locmem | mDSmaskbit[ndest];                         // locmem result 1    
#ifdef DEBUG_ACTIONS
  Serial.print(lmbit0);Serial.print(lmbit1);Serial.print('.');
#endif //DEBUG_ACTIONS


      /* évaluation source -> detecState (detecFound==1 if detecstate valid */
      switch((*curinp)&PERINPNT_MS){                              // type source
        case DETYEXT:detecState=(cstRec.extDetec>>nsrce)&0x01;    // valeur détecteur externe 
//             Serial.print("!!!!!!!!!!!!!!!!!! detecState=");Serial.print(detecState);Serial.print(" srce ");Serial.print(nsrce);
//             Serial.print("   curinp+2=");Serial.print(*(curinp+2),HEX);Serial.print("   curinp+2>>");Serial.println((((*(curinp+2))>>(PERINPVALID_PB) )&0x01),HEX);
             detecFound=1;break;
        case DETYPHY:detecState=(byte)(cstRec.memDetec[nsrce]>>DETBITLH_PB)&0x01;     // valeur détecteur physique
             detecFound=1;break;
        case DETYMEM:detecState=(locmem>>nsrce)&0x01;detecFound=1;break;              // valeur loc mem
        case DETYPUL:switch(staPulse[nsrce]){                     // pulse                
                 case PM_RUN1: detecState=1;detecFound=1;break;   // pulse run1=H     // état sortie MCU -> relais ON
                 case PM_RUN2: detecState=0;detecFound=1;break;   // pulse run2=L     // état sortie MCU -> relais OFF
                 case PM_END1: detecState=0;detecFound=1;break;   // pulse end1=L     // état sortie MCU -> relais OFF
                 case PM_END2: detecState=0;detecFound=1;break;   // pulse end2=L     // état sortie MCU -> relais OFF
                 case PM_IDLE: if((cstRec.cntPulseOne[nsrce]!=0) || ((cstRec.cntPulseOne[nsrce]+cstRec.cntPulseTwo[nsrce])==0)){
                                 detecState=0;detecFound=1;}      // pulse idle,   cnt1 !=0 -> L ou cnt1+cnt2=0 -> L
                               else {detecState=1;detecFound=1;}  //               cnt2 !=0 -> H  
                               break;
                 default: break;
             }break;
        default:break;
      }

      if(detecFound!=0){                                                          // if detecState valid (0==OFF ; 1==ON)

        if( (((*(curinp+2))&PERINPDETES_VB)!=0)                  
            &&((((*(curinp+2))>>(PERINPVALID_PB) )&0x01)==1)
           ){detecState^=0x01;}                                                   // static && inv -> invert          

        if(                                                                       // if( 
            (((*(curinp+2))&PERINPDETES_VB)==0)                                   // edge  
          )
          {
#ifdef DEBUG_ACTIONS
  Serial.print('[');Serial.print(((*(curinp+2))>>(PERINPOLDLEV_PB))&0x01);Serial.print(((*(curinp+2))>>(PERINPVALID_PB))&0x01);
  Serial.print(']');Serial.print(locmem,HEX);
#endif // DEBUG_ACTIONS
            if(                                                                   // if(
                (detecState!=(((*(curinp+2))>>(PERINPOLDLEV_PB))&0x01))           // level chge (curr!=old)
              )
              {
                *(curinp+2) &= ~PERINPOLDLEV_VB;                                  // raz bit oldlev
                *(curinp+2) |= (detecState << PERINPOLDLEV_PB);                   // màj bit oldlev

                if(                                                               // if(
                  (detecState==(((*(curinp+2))>>(PERINPVALID_PB) )&0x01))         // curr==active level  
                  )
                  {detecState=1;}                                                 // active edge detected 
                else 
                  {detecState=0;}                                                 // wrong edge
              }
            else
                  {detecState=0;}                                                 // no chge            

          }                                                                       // no edge


#ifdef DEBUG_ACTIONS
        if(                                                                       // if( 
            (
              (((*(curinp+2))&PERINPDETES_VB)==0)                                 // edge  
              &&(detecState==1)                                                   // valid
            )                                                                     
          ){Serial.print("*");}
        if(
            (((*(curinp+2))&PERINPDETES_VB)!=0)                                   // static
          ){Serial.print("#");}
#endif // DEBUG_ACTIONS

        if(                                                                       // if source ok (static or active edge)
            (
              (((*(curinp+2))&PERINPDETES_VB)==0)                                 // edge  
              &&(detecState==1)                                                   // valid
            )                                                                     
            ||                                                                    // or
            (
              (((*(curinp+2))&PERINPDETES_VB)!=0)                                 // static
            )                                                                     
          ){
            byte action=(*(curinp+2))>>PERINPACTLS_PB;
            uint8_t openClose[]={openSw[ndest],cloSw[ndest]};                     // open/close value for ndest switch
            
#ifdef DEBUG_ACTIONS
  Serial.print(action,HEX);Serial.print(locmem,HEX);
#endif // DEBUG_ACTIONS
            
            switch(action){                                                         // action (compute curValue then store it depending of dest)
              case PMDCA_0:
                            curValue = 0;
                            switch(tdest){                                          // type dest
                              case DETYEXT: break;                                  // transfert vers detServ à développer    
                              case DETYMEM: locmem=lmbit0;                          // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;
                            }
                            break;
              case PMDCA_1:
                            curValue = 1;
                            switch(tdest){                                          // type dest
                              case DETYEXT: break;                                  // transfert vers detServ à développer    
                              case DETYMEM: locmem=lmbit1;                          // locmem=1
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;                                 
                                            break;                                         
                              default:break;
                            }
                            break;
              case PMDCA_LOR:
#ifdef DEBUG_ACTIONS
  Serial.print('<');Serial.print(detecState);Serial.print(nsrce);Serial.print(ndest);Serial.print(curValue);Serial.print('>');
#endif // DEBUG_ACTIONS
                            curValue |= detecState;
                            switch(tdest){                                          // type dest
                              case DETYEXT: break;                                  // transfert vers detServ à développer    
                              case DETYMEM: if(curValue==1){locmem=lmbit1;}         // locmem=1
                                            else locmem=lmbit0;                     // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;
                            }
                            break;
              case PMDCA_LNOR:
                            curValue |= detecState;
                            switch(tdest){                                          // type dest
                              case DETYEXT: break;                                  // transfert vers detServ à développer    
                              case DETYMEM: if(curValue==1){locmem=lmbit0;}         // locmem=0
                                            else locmem=lmbit1;                     // locmem=1
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;              
                            }
                            break;                             
              case PMDCA_LXOR:
#ifdef DEBUG_ACTIONS
  Serial.print('{');Serial.print(detecState);Serial.print(nsrce);Serial.print(ndest);Serial.print(curValue);Serial.print('}');
#endif // DEBUG_ACTIONS
                            curValue ^= detecState;
                            switch(tdest){                                          // type dest
                              case DETYEXT: break;                                  // transfert vers detServ à développer    
                              case DETYMEM: if(curValue==1){locmem=lmbit1;}         // curValue to locmem tfr
                                            else locmem=lmbit0;                     
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;                                         
                              default:break;              
                            }
                            break;
              case PMDCA_LAND:
#ifdef DEBUG_ACTIONS
  Serial.print('|');Serial.print(detecState);Serial.print(nsrce);Serial.print(ndest);Serial.print(curValue);Serial.print('|');
#endif // DEBUG_ACTIONS
                            curValue &= detecState;
                            switch(tdest){                                          // type dest
                              case DETYEXT:break;                                   // transfert vers detServ à développer    
                              case DETYMEM: if(curValue==1 && (locmem & mDSmaskbit[ndest])!=0){locmem=lmbit1;}   // locmem=1
                                            else locmem=lmbit0;                     // locmem=0
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1; 
                                            break;
                              default:break;
                            }
                            break;
              case PMDCA_LNAND:
                            curValue &= detecState;
                            switch(tdest){                                          // type dest
                              case DETYEXT:break;                                   // transfert vers detServ à développer    
                              case DETYMEM: if(curValue==1 && (locmem & mDSmaskbit[ndest])!=0){locmem=lmbit0;}   // locmem=1
                                            else locmem=lmbit1;                     // locmem=1
                                            break;
                              case DETYSW:  curSw[ndest]=openClose[curValue];       // curValue -> curSw
                                            usdSw[ndest]=1;   
                                            break;
                              default:break;
                            }
                            break;
               case PMDCA_STOP: 
               //Serial.print(" stop(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]!=PM_END1 && staPulse[ndest]!=PM_END2){           // si arrêté ne rien toucher
                      if(staPulse[ndest]==PM_RUN1){
                        cstRec.cntPulse[ndest*2]=millis()-cstRec.cntPulseOne[ndest];}   // temps déjà écoulé pour repartir si un (re)start a lieu
                      if(staPulse[ndest]==PM_RUN2){
                        cstRec.cntPulse[ndest*2+1]=millis()-cstRec.cntPulseTwo[ndest];} // temps déjà écoulé pour repartir si un (re)start a lieu
                      staPulse[ndest]=PM_IDLE;
                      impDetTime[ndest]=0;
                    }
                    break;
               case PMDCA_START: 
               //Serial.print(" start (");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(") ");dumpfield((char*)curinp,4);Serial.print(detecState,HEX);Serial.print(" ");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(cstRec.cntPulseOne[ndest]!=0){
                      cstRec.cntPulseOne[ndest]=millis()-cstRec.cntPulse[ndest*2];    // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN1;}                   
                    else if(cstRec.cntPulseTwo[ndest]!=0){
                      cstRec.cntPulseTwo[ndest]=millis()-cstRec.cntPulse[ndest*2+1];  // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN2;}
                    else {staPulse[ndest]=PM_RUN1;cstRec.cntPulseOne[ndest]=millis();}
                    impDetTime[ndest]=millis();
                    break;
               case PMDCA_SHORT: 
               //Serial.print(" short(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]==PM_RUN1 || cstRec.cntPulseOne[ndest]!=0){
                      cstRec.cntPulseOne[ndest]=0;cstRec.cntPulse[ndest*2]=0;}     // cstRec.durPulseOne[ndest]*10;}
                    else if(staPulse[ndest]==PM_RUN2 || cstRec.cntPulseTwo[ndest]!=0){cstRec.cntPulseTwo[ndest]=0;} // cstRec.durPulseTwo[ndest]*10;}
                    impDetTime[ndest]=0;
                    break;                 
               case PMDCA_RAZ: 
               //Serial.print(" raz(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    cstRec.cntPulseOne[ndest]=0;
                    cstRec.cntPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    cstRec.cntPulse[ndest*2]=0;
                    cstRec.cntPulse[(ndest*2)+1]=0;
                    break;               
               case PMDCA_RESET: 
               //Serial.print(" reset(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    cstRec.cntPulseOne[ndest]=0;cstRec.durPulseOne[ndest]=0;
                    cstRec.cntPulseTwo[ndest]=0;cstRec.durPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    cstRec.cntPulse[ndest*2]=0;
                    cstRec.cntPulse[(ndest*2)+1]=0;
                    break;                 
               case PMDCA_IMP: 
               //Serial.print(" imp");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if((millis()-impDetTime[ndest])<DETIMP){
                      staPulse[ndest]=PM_IDLE;
                      cstRec.cntPulseOne[ndest]=0;
                      cstRec.cntPulseTwo[ndest]=0;}
                    Serial.print("Time=");Serial.print(millis()-impDetTime[ndest]);Serial.print(" ");
                    impDetTime[ndest]=0;
                    break;
               case PMDCA_END: 
               //Serial.print(" end(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]==PM_RUN1 || cstRec.cntPulseOne[ndest]!=0){
                      cstRec.cntPulseOne[ndest]=0;
                      cstRec.cntPulse[ndest*2]=0;
                      setPulseChg(ndest,'O');}
                    else if(staPulse[ndest]==PM_RUN2 || cstRec.cntPulseTwo[ndest]!=0){
                      cstRec.cntPulseTwo[ndest]=0;
                      cstRec.cntPulse[(ndest*2)+1]=0;
                      setPulseChg(ndest,'T');}
                    impDetTime[ndest]=0;
                    break;
               default:actionSysErr(action);
                    if(tdest==DETYPUL){staPulse[ndest]=PM_DISABLE;}
                    break;
          }     // switch(action)       
        }       // (valid static or active edge) curValue updated
        else{   // si condition non validée pas d'action : configurer explicitement pour l'inverse si nécessaire
#ifdef DEBUG_ACTIONS
  Serial.print('!');
#endif //DEBUG_ACTIONS        
        }

      }   // detecFound    
#ifdef DEBUG_ACTIONS
  Serial.print(curValue);Serial.print(locmem,HEX);
#endif //DEBUG_ACTIONS
    }     // enable
  }       // next input
#ifdef DEBUG_ACTIONS
  Serial.print(')');
#endif //DEBUG_ACTIONS
  /* SW update */
  uint8_t mskSw[] = {0xfe,0xfb,0xef,0xbf};                           
  for(uint8_t i=0;i<NBSW;i++){                                    // 1 byte 4sw + 4disjoncteurs (voir const.h du frontal)
    if(usdSw[i]==1){
      cstRec.swCde &= mskSw[i];                                   // clear switch bit
      cstRec.swCde |= curSw[i]<<(2*i);                            // bit switchs (bits 7,5,3,1 pour switchs 3,2,1,0)  
      //if(cstRec.swCde!=oldCstCde){Serial.print("\n");Serial.print(millis());Serial.print("--->");Serial.print(oldCstCde,HEX);Serial.print(" ");Serial.println(cstRec.swCde,HEX);oldCstCde=cstRec.swCde;}
    }  
  }
}

/* ------------- gestion pulses ------------- */

void setPulseChg(uint8_t npu,char timeOT)     // traitement fin de temps 
                                          // timeOT ='O' fin timeOne ; ='T' fin timeTwo                                          
{
  uint16_t ctl;memcpy(&ctl,cstRec.pulseMode,PCTLLEN);ctl=ctl>>(npu*PCTLBIT);ctl&=0x0007;
  //Serial.print(" npu=");Serial.print(npu);Serial.print(" ctl=");dumpfield((char*)&ctl,2);Serial.print(' ');Serial.print(cstRec.cntPulseOne[npu]);Serial.print(' ');Serial.print(cstRec.cntPulseTwo[npu]);
  if(timeOT=='O'){
        if((ctl&(uint16_t)PMTTE_VB)!=0){                                                           // cnt2 enable -> run2
          cstRec.cntPulseOne[npu]=0;cstRec.cntPulseTwo[npu]=(uint32_t)millis();
          cstRec.cntPulse[npu*2]=0;
          staPulse[npu]=PM_RUN2;
          //Serial.print(' ');Serial.print(cstRec.cntPulseTwo[npu]);Serial.print(" PM_RUN2 ");
          }
        else {staPulse[npu]=PM_END1;//Serial.print(" PM_END1 ");
          }                                             // sinon fin1
  }
  else {                                                                                          // fin run2               
        cstRec.cntPulse[(npu*2)+1]=0;                   
        if((ctl&(uint16_t)PMFRO_VB)==0){                                                          // oneshot -> idle
          cstRec.cntPulseTwo[npu]=0;cstRec.cntPulseOne[npu]=0;
          staPulse[npu]=PM_IDLE;
          //Serial.print(" PM_IDLE ");
          }
        else if((ctl&(uint16_t)PMTOE_VB)!=0){                                                     // free run && one enable -> run1
          cstRec.cntPulseTwo[npu]=0;cstRec.cntPulseOne[npu]=(uint32_t)millis();
          staPulse[npu]=PM_RUN1;
          //Serial.print(' ');Serial.print(cstRec.cntPulseOne[npu]);Serial.print(" PM_RUN1 ");
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
  
    memset(cstRec.cntPulseOne,0x00,sizeof(cstRec.cntPulseOne));
    memset(cstRec.cntPulseTwo,0x00,sizeof(cstRec.cntPulseTwo)); 
    memset(cstRec.durPulseOne,0x00,sizeof(cstRec.durPulseOne));
    memset(cstRec.durPulseTwo,0x00,sizeof(cstRec.durPulseTwo));
    memset(staPulse,PM_IDLE,sizeof(staPulse));
    memset(cstRec.pulseMode,0x00,PCTLLEN);
    memset(cstRec.cntPulse,0x00,sizeof(cstRec.cntPulse));
}

void pulseClkisr()                       // polling ou interruption ; contrôle de décap des compteurs : horloge des pulses
{  
  uint32_t currt;
  for(uint8_t npu=0;npu<NBPULSE;npu++){
    //Serial.print("pulseClkisr() npu=");Serial.print(npu);Serial.print( " staPulse=");Serial.println(staPulse[npu]);
    switch(staPulse[npu]){
      
      case PM_DISABLE: break;            // changement d'état quand le bit enable d'un compteur sera changé
      case PM_IDLE: break;               // changement d'état par une action
      case PM_RUN1: currt=((uint32_t)millis()-cstRec.cntPulseOne[npu])/1000;
                    if(currt>cstRec.durPulseOne[npu]){   // (decap cnt1)
                      //Serial.print(currt);Serial.print(" ");Serial.print(cstRec.cntPulseOne[npu]);Serial.print(" ");Serial.print(cstRec.durPulseOne[npu]);Serial.print(" ");
                      setPulseChg(npu,'O');
                    }break;
                    
      case PM_RUN2: currt=((uint32_t)millis()-cstRec.cntPulseTwo[npu])/1000;
                    if(currt>cstRec.durPulseTwo[npu]){   // (decap cnt2)
                      //Serial.print(currt);Serial.print(" ");Serial.print(cstRec.cntPulseTwo[npu]);Serial.print(" ");Serial.print(cstRec.durPulseTwo[npu]);Serial.print(" ");
                      setPulseChg(npu,'T');
                    }break;
                    
      case PM_END1: cstRec.cntPulseOne[npu]=0;break;  // changt d'état quand bit enable compteur 2 sera changé
      case PM_END2: cstRec.cntPulseTwo[npu]=0;break;  // changt d'état quand bit enable compteur 1 sera changé ou changt freerun->oneshot
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

void polDx(uint8_t det)              // maj memDetec selon l'état du détecteur det (polDx masqué par tempo debounce) 
                                     // memDetec met le débounce en commun si plusieurs inputs
                                     // utilisent le même détecteur (seul bit utilisé : LH)
{    
    byte lev=PINREAD(det); //digitalRead(pinDet[det]);
    if( ((byte)(cstRec.memDetec[det]>>DETBITLH_PB)&0x01) != lev ){    // niveau lu != niveau actuel de memDetec ?
      // level change -> update memDetec
      cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
      cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 
      detTime[det]=millis();                                          // arme debounce
      talkReq();                                                      // talkServer
      cstRec.serverTime=0;
      delay(1);Serial.print("  >>>>>>>>> det ");Serial.print(det);Serial.print(" change to ");Serial.println(lev);//Serial.print(" - ");
      delay(1);
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
