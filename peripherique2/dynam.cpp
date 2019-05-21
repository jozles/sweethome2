
#include "const.h"
#include "Arduino.h"
#include "shconst2.h"
#include "shutil2.h"
#include "dynam.h"
#include "util.h"

#if POWER_MODE==NO_MODE

extern constantValues cstRec;

extern  uint8_t pinSw[MAXSW];                                  // les switchs
extern  byte    staPulse[NBPULSE];                             // état clock pulses
extern  uint8_t pinDet[MAXDET];

extern  long    detTime[MAXDET];                               // debounce détecteurs physiques
extern  long    impDetTime[NBPULSE];                           // timer pour gestion commandes impulsionnelles     
extern  long    isrTime;
extern  void    (*isrD[4])(void);                              // tableau des pointeurs des isr détecteurs
extern  byte    mask[];

//extern  int     cntdebug[];
//extern  long    timedebug[]={0,0,0,0};
extern  int*    int0;

extern uint32_t  mDSmaskbit[];


/* ------------------ généralités -------------------- 

  switchs :

  les switchs sont actionnés par polling de la table des inputs (ou règles) via actions()
  pinSw[] est la table les pins associés aux switchs

  dans la limite du nombre d'inputs disponibles (24) une règle peut comporter autant d'actions que désiré
  elles sont exécutées dans l'ordre ce qui a un effet sur le résultat !
  via la source/destination "mémoire" le résultat d'une action peut être la source d'une autre
  les positions de "mémoire" sont le résultat des opérations logiques effectuées par les actions

  Les règles comprennent :
    La source (type/n°) pour identifier un détecteur (physique, mémoire, externe, pulse) 
    Les paramètres de fonctionnement (enable/edge-static/active level/old level/action) 
    la destination (type N°) pour identifier ce sur quoi agir (switch, mémoire, détecteur externe, pulse)

  l'action indique ce qui doit être fait lorsque les paramètres appliqués à la source matchent
  certaines actions concernent exclusivement les générateurs de pulses, 
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
      - stop          
      - raz           les 2 compteurs=0 ; stapulse idle
      - reset         les 2 compteurs et les 2 durées =0 ; stapulse idle
      - fin           compteur courant au max puis avance compteur ; si bloqué -> stapulse end1 ou 2 sinon suite normale
      - short         compteur courant au max ; stapulse inchangé
      - impulsion     si impDetTime < DETIMP effectue raz sinon sans effet
      (autres)
      - toogle        inverse l'état de la destination
      - OR            force à 1 la destination
      - NAND          force à 0 la destination 
  
  Les pulses sont commandés par les actions (donc au rythme du polling des inputs)
  et animés lors de l'appel de pulseClkisr()
  
  staPulse est l'état des générateurs de pulses
    en mode débranché (DISABLE) en cas d'erreur système (action invalide d'un détecteur)
    en mode suspendu  (IDLE) suite à une action STOP ou en fin de oneshot
    en mode comptage  (RUN)  suite à une action START
    en mode fin       (END)  lorsque le comptage d'une phase est terminé et que la phase suivante est débranchée 
  staPulse est mis à jour par isrPulse()

*/

void setPulseChg(int npu,char timeOT)     // traitement fin de temps 
                                          // timeOT ='O' fin timeOne ; ='T' fin timeTwo                                          
{
  uint16_t ctl;memcpy(&ctl,cstRec.pulseMode,PCTLLEN);ctl=ctl>>(npu*PCTLBIT);
  Serial.print(" npu=");Serial.print(npu);Serial.print(" ctl=");dumpfield((char*)&ctl,2);
  if(timeOT=='O'){
        if((ctl&(uint16_t)PMTTE_VB)!=0){                                                           // cnt2 enable -> run2
          cstRec.cntPulseOne[npu]=0;cstRec.cntPulseTwo[npu]=millis();staPulse[npu]=PM_RUN2;Serial.print(" PM_RUN2 ");}
        else {staPulse[npu]=PM_END1;Serial.print(" PM_END1 ");}                                             // sinon fin1
  }
  else {
        if((ctl&(uint16_t)PMFRO_VB)==0){                                                          // oneshot -> idle
          cstRec.cntPulseTwo[npu]=0;cstRec.cntPulseOne[npu]=0;staPulse[npu]=PM_IDLE;Serial.print(" PM_IDLE ");}
        else if((ctl&(uint16_t)PMTOE_VB)!=0){                                                     // free run -> run      
          cstRec.cntPulseTwo[npu]=0;cstRec.cntPulseOne[npu]=millis();staPulse[npu]=PM_RUN1;Serial.print(" PM_RUN1 ");}
        else {staPulse[npu]=PM_END2;Serial.print(" PM_END2 ");}                                   // free run bloqué -> fin2
  }
  Serial.print(" sec=");Serial.print(millis()/1000);Serial.println();
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
  uint8_t detecState=0;     // valeur trouvée pour le det source (type-n°)
  uint8_t detecFound=0;     // flag : valeur valide si !=0
  uint8_t nsrce;            // n° source
  uint8_t ndest;            // n° destination
  byte    tdest;            // type destination

  uint32_t locmem=0;        // mémoire = valeurs intermédiaires d'évaluation

  for(int inp=0;inp<NBPERINPUT;inp++){
    
    curinp=&cstRec.perInput[inp*PERINPLEN];
    nsrce=(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);
    ndest=(((*(curinp+3))&PERINPV_MS)>>PERINPNVLS_PB);
    tdest=(byte)((*(curinp+3))&PERINPNT_MS);
    
    if(((*(curinp+2))&PERINPEN_VB)!=0){                          // enable
      switch((*curinp)&PERINPNT_MS){                             // type
        case DETYEXT:detecState=(cstRec.extDetec>>nsrce)&0x01;   // valeur détecteur externe 
             detecFound=1;break;
        case DETYPHY:detecState=(byte)(cstRec.memDetec[nsrce]>>DETBITLH_PB)&0x01;     // valeur détecteur physique
             detecFound=1;break;
        case DETYMEM:detecState=(locmem>>nsrce)&0x01;            // valeur intermédiaire
        case DETYPUL:switch(staPulse[nsrce]){                    // pulse                
                 case PM_RUN1: detecState=0;detecFound=1;break;  // pulse run1=L
                 case PM_RUN2: detecState=1;detecFound=1;break;  // pulse run2=H
                 case PM_END1: detecState=0;detecFound=1;break;  // pulse end1=L
                 case PM_END2: detecState=1;detecFound=1;break;  // pulse run1=H
                 case PM_IDLE: if((cstRec.cntPulseOne[nsrce]!=0) || ((cstRec.cntPulseOne[nsrce]+cstRec.cntPulseTwo[nsrce])==0)){
                                 detecState=0;detecFound=1;}     // pulse idle,   cnt1 !=0 -> L ou cnt1+cnt2=0 -> L
                               else {detecState=1;detecFound=1;} //               cnt2 !=0 -> H  
                               break;
                 default: break;
             }break;
        default:break;
      }                                                         

    if(detecFound!=0){

      if(                                                                       // état actif ?
          (
            (((*(curinp+2))&PERINPDETES_VB)==0)                                 // edge
          &&(detecState!=(((*(curinp+2))>>PERINPOLDLEV_PB)&0x01))               // flanc si curr!=old
          &&(detecState==(((*(curinp+2))>>PERINPVALID_PB) &0x01))               // flanc ok
          )
          ||                                                                  // ou
          (
            (((*(curinp+2))&PERINPDETES_VB)!=0)                                 // static
          &&(detecState==(((*(curinp+2))>>PERINPVALID_PB) &0x01))               // état ok          
          )
        ){ 
//Serial.print("input=");Serial.print(inp);Serial.print(" enable=");Serial.print((*(curinp+2))&PERINPEN_VB);Serial.print(" oldlev=");Serial.print((*(curinp+2))&PERINPOLDLEV_VB);Serial.print(" e/s=");Serial.print((*(curinp+2))&PERINPDETES_VB);Serial.print(" active=");Serial.print((*(curinp+2))&PERINPVALID_VB);Serial.print(" types=");Serial.print((*curinp)&PERINPNT_MS);Serial.print(" num=");Serial.print(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);Serial.print(" detecFound=");Serial.print(detecFound);Serial.print(" detecState=");Serial.print(detecState);Serial.print(" disjonct=");Serial.print((cstRec.swCde>>(ndest*2+1))&0x01);Serial.print("/");Serial.print(cstRec.swCde,HEX);Serial.print("/");Serial.print(ndest,HEX);Serial.print(" typed=");Serial.print((*(curinp+3))&PERINPNT_MS);Serial.print(" action=");Serial.println((*(curinp+2))>>PERINPACTLS_PB);

          uint32_t lmbit;
          byte action=(*(curinp+2))>>PERINPACTLS_PB;
          switch(action){                                                        // exécution action
              case PMDCA_LOR:switch(tdest){                                               // type dest   *** 0->1 ; 1->1 ***
                             case DETYEXT:break;
                             case DETYMEM:locmem |= mDSmaskbit[ndest];break;              // or locmembit,1
                             case DETYSW:if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         digitalWrite(pinSw[ndest],(digitalRead(pinSw[ndest])|ON));}
                                         else{digitalWrite(pinSw[ndest],OFF);}
                                         break;
                             default:break;
                             }
                             break;
              case PMDCA_LAND:switch((byte)((*(curinp+3))&PERINPNT_MS)){                  // type dest   *** 0->0 ; 1->1 *** sans effet !
                             case DETYEXT:break;
                             case DETYMEM:lmbit=locmem & mDSmaskbit[ndest];               // get anded locmembit,1
                                          locmem &= ~mDSmaskbit[ndest];                   // raz locmem bit
                                          locmem |= lmbit;                                // store result
                                          break;
                             case DETYSW:if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         digitalWrite(pinSw[ndest],(digitalRead(pinSw[ndest])&ON));}
                                         else{digitalWrite(pinSw[ndest],OFF);}
                                         break;
                             default:break;
                             }
                             break;
              case PMDCA_LNOR:switch((byte)((*(curinp+3))&PERINPNT_MS)){                  // type dest   *** 0->0 ; 1->0 ***
                             case DETYEXT:break;
                             case DETYMEM:locmem &= ~mDSmaskbit[ndest];break;              // raz locmem bit
                             case DETYSW:digitalWrite(pinSw[ndest],OFF);break;             // no disjoncteur ?
                             default:break;
                             }
                             break;                             
              case PMDCA_LXOR:switch((byte)((*(curinp+3))&PERINPNT_MS)){                  // type dest   *** 0->1 ; 1->0 *** equiv NAND 
                             case DETYEXT:break;
                             case DETYMEM:lmbit=(locmem ^ 0xffffffff) & mDSmaskbit[ndest];// get eored locmem bit
                                          locmem &= ~mDSmaskbit[ndest];                   // raz locmem bit
                                          locmem |= lmbit;                                // store result
                                          break;
                             case DETYSW:if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         digitalWrite(pinSw[ndest],(digitalRead(pinSw[ndest])^ON));}
                                         else{digitalWrite(pinSw[ndest],OFF);}
                                         break;
                             default:break;
                             }
                             break;
               case PMDCA_STOP: Serial.print(" stop(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]==PM_RUN1){
                      cstRec.cntPulseOne[ndest]=millis()-cstRec.cntPulseOne[ndest];} // temps déjà écoulé pour repartir si un (re)start a lieu
                    if(staPulse[ndest]==PM_RUN2){
                      cstRec.cntPulseTwo[ndest]=millis()-cstRec.cntPulseTwo[ndest];} // temps déjà écoulé pour repartir si un (re)start a lieu
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    break;
               case PMDCA_START: Serial.print(" start (");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(") ");dumpfield((char*)curinp,4);Serial.print(detecState,HEX);Serial.print(" ");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(cstRec.cntPulseOne[ndest]!=0){
                      cstRec.cntPulseOne[ndest]=millis()-cstRec.cntPulseOne[ndest]; // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN1;}                   
                    else if(cstRec.cntPulseTwo[ndest]!=0){
                      cstRec.cntPulseTwo[ndest]=millis()-cstRec.cntPulseTwo[ndest]; // (re)start - temps déjà écoulé lors du stop
                      staPulse[ndest]=PM_RUN2;}
                    else {staPulse[ndest]=PM_RUN1;cstRec.cntPulseOne[ndest]=millis();}
                    impDetTime[ndest]=millis();
                    break;
               case PMDCA_SHORT: Serial.print(" short(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]==PM_RUN1 || cstRec.cntPulseOne[ndest]!=0){cstRec.cntPulseOne[ndest]=0;}      // cstRec.durPulseOne[ndest]*10;}
                    else if(staPulse[ndest]==PM_RUN2 || cstRec.cntPulseTwo[ndest]!=0){cstRec.cntPulseTwo[ndest]=0;} // cstRec.durPulseTwo[ndest]*10;}
                    impDetTime[ndest]=0;
                    break;                 
               case PMDCA_RAZ: Serial.print(" raz(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    cstRec.cntPulseOne[ndest]=0;
                    cstRec.cntPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    break;               
               case PMDCA_RESET: Serial.print(" reset(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    cstRec.cntPulseOne[ndest]=0;cstRec.durPulseOne[ndest]=0;
                    cstRec.cntPulseTwo[ndest]=0;cstRec.durPulseTwo[ndest]=0;
                    staPulse[ndest]=PM_IDLE;
                    impDetTime[ndest]=0;
                    break;                 
               case PMDCA_IMP: Serial.print(" imp");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if((millis()-impDetTime[ndest])<DETIMP){
                      staPulse[ndest]=PM_IDLE;
                      cstRec.cntPulseOne[ndest]=0;
                      cstRec.cntPulseTwo[ndest]=0;}
                    Serial.print("Time=");Serial.print(millis()-impDetTime[ndest]);Serial.print(" ");
                    impDetTime[ndest]=0;
                    break;
               case PMDCA_END: Serial.print(" end(");Serial.print((millis()-cstRec.cntPulseOne[ndest])/1000);Serial.print("/");Serial.print((millis()-cstRec.cntPulseTwo[ndest])/1000);Serial.print(")");
                    if(tdest!=DETYPUL){actionSysErr(action);break;}
                    if(staPulse[ndest]==PM_RUN1 || cstRec.cntPulseOne[ndest]!=0){cstRec.cntPulseOne[ndest]=0;setPulseChg(ndest,'O');}
                    else if(staPulse[ndest]==PM_RUN2 || cstRec.cntPulseTwo[ndest]!=0){cstRec.cntPulseTwo[ndest]=0;setPulseChg(ndest,'T');}
                    impDetTime[ndest]=0;
                    break;
               default:actionSysErr(action);
                    if(tdest==DETYPUL){staPulse[ndest]=PM_DISABLE;}
                    break;
          } 
      } else{} // si condition non validée pas d'action : configurer explicitement pour l'inverse si nécessaire
      
      *(curinp+2) &= ~PERINPOLDLEV_VB;                                          // raz bit oldlev
      *(curinp+2) |= (detecState << PERINPOLDLEV_PB);                           // màj bit oldlev

    }   // detecFound   
    }   // enable  
  }     // next input
}

/* ------------- gestion pulses ------------- */

void pulsesinit()                         // init pulses à la mise sous tension
{
  Serial.println("init pulses");
  
    memset(cstRec.cntPulseOne,0x00,sizeof(cstRec.cntPulseOne));
    memset(cstRec.cntPulseTwo,0x00,sizeof(cstRec.cntPulseTwo)); 
    memset(cstRec.durPulseOne,0x00,sizeof(cstRec.durPulseOne));
    memset(cstRec.durPulseTwo,0x00,sizeof(cstRec.durPulseTwo));
    memset(staPulse,PM_IDLE,sizeof(staPulse));
}

void pulseClkisr()                       // poling ou interruption ; horloge des pulses
{
  uint8_t npu;
  
  for(npu=0;npu<NBPULSE;npu++){
    //Serial.print("pulseClkisr() npu=");Serial.print(npu);Serial.print( " staPulse=");Serial.println(staPulse[npu]);
    switch(staPulse[npu]){
      
      case PM_DISABLE: break;            // changement d'état quand le bit enable d'un compteur sera changé
      case PM_IDLE: break;               // changement d'état par une action
      case PM_RUN1: /*cstRec.cntPulseOne[npu]++;
                    if(cstRec.cntPulseOne[npu]>=cstRec.durPulseOne[npu]*10){*/              // (decap cnt1)
                    if((millis()-cstRec.cntPulseOne[npu])>=cstRec.durPulseOne[npu]*1000){
                      setPulseChg(npu,'O');
                    }break;
                    
      case PM_RUN2: /*cstRec.cntPulseTwo[npu]++;
                    if(cstRec.cntPulseTwo[npu]>=cstRec.durPulseTwo[npu]*10){*/              // (decap cnt2)
                    if((millis()-cstRec.cntPulseTwo[npu])>=cstRec.durPulseTwo[npu]*1000){
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
  
  for(uint8_t det=0;det<MAXDET;det++){
    cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
    cstRec.memDetec[det] |= digitalRead(pinDet[det])<<DETBITLH_PB;  // set bit LH 
    detTime[det]=millis();
  }
}

void polDx(uint8_t det)              // maj memDetec selon l'état du détecteur det (polDx masqué par tempo debounce) 
                                     // memDetec met le débounce en commun si plusieurs inputs
                                     // utilisent le même détecteur (seul bit utilisé : LH)
{    
    byte lev=digitalRead(pinDet[det]);
    if( ((byte)(cstRec.memDetec[det]>>DETBITLH_PB)&0x01) != lev ){    // niveau lu != niveau actuel de memDetec ?
      // level change -> update memDetec
      cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
      cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 
      detTime[det]=millis();                                          // arme debounce
      cstRec.talkStep=1;                                              // talkServer
      cstRec.serverTime=0;
      Serial.print("  >>>>>>>>> det ");Serial.print(det);Serial.print(" change to ");Serial.println(lev);//Serial.print(" - ");
  }
}

void polAllDet()                                        // maj de memDetec (via polDx) pour tous les détecteurs locaux
{                                                       // la tempo de debouce masque polDx
   for(uint8_t det=0;det<(MAXDET);det++){if(detTime[det]==0){polDx(det);}}    // pas de debounce en cours  
}
 

void swDebounce()                     
{
  for(uint8_t det=0;det<MAXDET;det++){
    if(detTime[det]!=0 && (millis()>(detTime[det]+TDEBOUNCE))){
      detTime[det]=0; 
    }
  }
}

/*/* --------------- interruptions détecteurs ----------------  

void isrDet(uint8_t det)      // setup memDetec et staPulse après interruption sur det
{
  long sht=micros();
  
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

#endif NO_MODE
