
#include "const.h"
#include "Arduino.h"
#include "shconst2.h"
#include "shutil2.h"
#include "dynam.h"
#include "util.h"

#if POWER_MODE==NO_MODE

extern constantValues cstRec;

extern  uint8_t pinSw[MAXSW];                                  // les switchs
extern  byte    staPulse[MAXSW];                               // état clock pulses
extern  uint8_t pinDet[MAXDET];
extern  bool    pinLev[MAXDET];
extern  byte    pinDir[MAXDET];
extern  long    detTime[NBDSRV+MAXDET];
extern  long    impDetTime[MAXSW];                             // timer pour gestion commandes impulsionnelles     
extern  long    isrTime;
extern  void    (*isrD[4])(void);                              // tableau des pointeurs des isr détecteurs
extern  byte    mask[];

//extern  int     cntdebug[];
//extern  long    timedebug[]={0,0,0,0};
extern  int*    int0;

extern uint32_t  mDSmaskbit[];


/* ------------------ généralités -------------------- 

  switchs :

  les switchs sont actionnés par poling de la table des ccommandes de switch (inputs ou règles)
  via actions() qui utilise la table perInput des règles

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
  
  staPulse est l'état du générateur
    en mode débranché (DISABLE) en cas d'erreur système (action invalide d'un détecteur)
    en mode suspendu  (IDLE) suite à une action STOP d'un détecteur logique ou en fin de oneshot
    en mode comptage  (RUN)  suite à une action START d'un détecteur logique
    en mode fin       (END)  lorsque le comptage d'une phase est terminé et que la phase suivante est débranchée 
  staPulse est mis à jour par isrPulse

 
*/


  

/* -------------- gestion commande switchs ----------------- */

/*uint8_t rdy(byte modesw,int sw) // pour les 3 sources, check bit enable puis etat source ; retour numéro source valorisé si valide sinon 0
{
  modesw copie du mot de contrôle du switch (OffCdeO, OnCdeO, OnCdeI, OffCdeI) 2 bits valide par source ;
 *  pour chacune des sources, test du bit enable de la source SWxxEN_VB (xx=DL pour détecteur logique, MS pour serveur, MP pour pulse)
   si pas enable, source suivante ; 
*/
  
  /* ------ détecteur --------- 
   si enable=1 pour cette source, 
   recup n° detec logique 'det' pour isoler les bits d'info du det logique parmi 4 dans la description du switch
   (bits d'infos : enable det logique, n° det (physique si local, rang si externe), local si !=0 ou externe, état actif du détecteur)
   (le bit "état actif du détecteur" semble inutile et n'est pas traité pour cette source... en faire un inverseur ?)
   Enfin, test enable et comparaison avec la consigne : (si physique) l'image mémoire du détecteur physique dans memDetec
                                                        (si externe)  le bit correspondant dans la copie des bits externes (masqué par enable externe)
  */
  /*
  if((modesw & SWMDLEN_VB) !=0){                                      // det source bit enable 
    uint8_t det=(modesw>>SWMDLNULS_PB)&0x03;                          // numéro det logique
    uint64_t swctl;memcpy(&swctl,cstRec.pulseCtl+sw*DLSWLEN,DLSWLEN); // les 6 bytes de description du switch courant
    uint16_t dlctl=(uint16_t)(swctl>>(det*DLBITLEN));                 // calés en poids faibles les bits du détecteur logique
    if((uint16_t)(dlctl&DLENA_VB) != 0){                              // dl bit enable
      uint8_t lexdet=(dlctl>>DLNLS_PB)&0x07;                          // num det local ou externe
      if((uint16_t)(dlctl&DLEL_VB) != 0){                             // dl local
        uint8_t locdet=(cstRec.memDetec[lexdet]>>DETBITLH_PB)&0x01;   // det local
        if(
          ((modesw>>SWMDLHL_PB)&0x01)==locdet){
//         && ((dlctl>>DLMHL_PB)&0x01)==locdet                          // etat actif ?
          }{return 1;}
      }
      else {                                                           // dl externe ; extdetlev les 8 det (bits) 
        uint8_t extdet=(cstRec.extDetLev>>lexdet)&0x01;                // det externe
        //Serial.print("extdet=");Serial.print(extdet);Serial.print(" lexdet=");Serial.print(lexdet);Serial.print(" extDetLev=");Serial.print(cstRec.extDetLev/16,HEX);Serial.print(cstRec.extDetLev%16,HEX);Serial.print(" extDetEn=");Serial.print(cstRec.extDetEn/16,HEX);Serial.println(cstRec.extDetEn%16,HEX);
        if(
          ((cstRec.extDetEn>>lexdet)&0x01)!=0                          // ext det enable 
          && ((modesw>>SWMDLHL_PB)&0x01)==extdet                       // consigne ?
//          && ((dlctl>>DLMHL_PB)&0x01)==extdet                          // etat actif ?          
        ){return 1;}                                        
      }                                                                                                              
    }
  }
   --------- serveur ---------- */
//  if((modesw & SWMSEN_VB) != 0){                                       // server source bit enable
//    if (((modesw>>SWMSHL_PB)&0x01)==((cstRec.swCde>>((2*sw)+1))&0x01)){return 2;}    // comparaison bit et consigne
//  }

  /* --------- pulse gen --------- 
  si enable, selon staPulse et consigne détermination état du switch (ou pas) 
  (voir états staPulse dans const.h de frontal)
  */
//  if((modesw & SWMPEN_VB) != 0){                                       // pulse source bit enable
  //Serial.print(" sw=");Serial.print(sw);Serial.print(" stapulse=");Serial.print(staPulse[sw]);
//  bool lh=(modesw>>SWMPHL_PB)&0x01;
  //Serial.print(" LH=");Serial.print(lh);Serial.print(" cnt1=");Serial.print(cstRec.cntPulseOne[sw]);Serial.print(" cnt2=");Serial.print(cstRec.cntPulseTwo[sw]);
/*    switch(staPulse[sw]){
      case PM_RUN1: if(!lh){return 3;}break;     // pulse run1=L, demandé L ---> ok
      case PM_RUN2: if( lh){return 3;}break;     // pulse run2=H, demandé H ---> ok
      case PM_END1: if(!lh){return 3;}break;     // pulse end1=L, demandé L ---> ok
      case PM_END2: if( lh){return 3;}break;     // pulse run1=H, demandé H ---> ok
      case PM_IDLE: if(!lh){                     // pulse idle,   demandé L et cnt1 !=0 ---> ok
                                                 //                         et cnt1+cnt2=0 > ok
                                                 //               demandé H et cnt2 !=0 ---> ok
                      if((cstRec.cntPulseOne[sw]!=0) || ((cstRec.cntPulseOne[sw]+cstRec.cntPulseTwo[sw])==0))
                        {return 3;}break;}
                    else {if(cstRec.cntPulseTwo[sw]!=0){return 3;}break;}
      default: break;
    }
//  }
  return 0;                                      // disable 
}*/

void actions()           // pour chaque input, test enable,
{                       //      récup valeur détecteur
                        //      comparaison avec valeur demandée (et flanc éventuel) des règles actives (enable) dans l'ordre des priorités
                        //      (conjoncteur, interrupteur, allumeur)
                        //      maj destination selon résultat et archivage oldlev
  
  byte*   curinp;           // adresse cur input
  uint8_t detecState=0;     // valeur trouvée pour le det source (type-n°)
  uint8_t detecFound=0;     // flag : valeur valide si !=0
  uint8_t nsrce;            // n° source
  uint8_t ndest;            // n° destination

  uint32_t locmem=0;        // valeurs intermédiaires d'évaluation

  for(int inp=0;inp<NBPERINPUT;inp++){
    
    curinp=&cstRec.perInput[inp*PERINPLEN];
    nsrce=(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);
    ndest=(((*(curinp+3))&PERINPV_MS)>>PERINPNVLS_PB);
    
    if(((*(curinp+2))&PERINPEN_VB)!=0){                         // enable
      switch((*curinp)&PERINPNT_MS){                            // type
        case DETYEXT:detecState=(cstRec.extDetec>>nsrce)&0x01;  // valeur détecteur externe 
             detecFound=1;break;
        case DETYPHY:detecState=(byte)(cstRec.memDetec[nsrce]>>DETBITLH_PB)&0x01;     // valeur détecteur physique
             detecFound=1;break;
        case DETYMEM:detecState=(locmem>>nsrce)&0x01;           // valeur intermédiaire
        case DETYPUL:break;                                     // pulse 
        default:break;
      }                                                         

    if(detecFound!=0){

      if(
          (
            (((*(curinp+2))&PERINPDETES_VB)==0)                                 // edge
          &&(detecState!=(((*(curinp+2))>>PERINPOLDLEV_PB)&0x01))               // flanc
          &&(detecState==(((*(curinp+2))>>PERINPVALID_PB) &0x01))               // flanc ok
          )
          ||                                                                  // ou
          (
            (((*(curinp+2))&PERINPDETES_VB)!=0)                                 // static
          &&(detecState==(((*(curinp+2))>>PERINPVALID_PB) &0x01))               // état ok          
          )
        ){ 
//if(inp<3){Serial.print(" enable=");Serial.print((*(curinp+2))&PERINPEN_VB);Serial.print(" types=");Serial.print((*curinp)&PERINPNT_MS);Serial.print(" num=");Serial.print(((*curinp)&PERINPV_MS)>>PERINPNVLS_PB);Serial.print(" detecFound=");Serial.print(detecFound);Serial.print(" detecState=");Serial.print(detecState);Serial.print(" disjonct=");Serial.print((cstRec.swCde>>(ndest*2+1))&0x01);Serial.print("/");Serial.print(cstRec.swCde,HEX);Serial.print("/");Serial.print(ndest,HEX);Serial.print(" typed=");Serial.print((*(curinp+3))&PERINPNT_MS);Serial.print(" action=");Serial.println((*(curinp+2))>>PERINPACTLS_PB);}
            *(curinp+2) &= ~PERINPOLDLEV_VB;                                       // raz bit oldstate
            detecState << PERINPOLDLEV_PB;                                         
            *(curinp+2) |= detecState;                                             // setup oldstate

            uint32_t lmbit;
            switch((byte)((*(curinp+2))>>PERINPACTLS_PB)){                           // exécution action
              case PMDCA_LOR:switch((byte)((*(curinp+3))&PERINPNT_MS)){                     // type dest
                             case DETYEXT:break;
                             case DETYMEM:locmem |= mDSmaskbit[ndest];break;              // or locmembit,1
                             case DETYSW:if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         digitalWrite(pinSw[ndest],(digitalRead(pinSw[ndest])|ON));}
                                         else{digitalWrite(pinSw[ndest],OFF);}
                                         break;
                             default:break;
                             }
                             break;
              case PMDCA_LAND:switch((byte)((*(curinp+3))&PERINPNT_MS)){                    // type dest
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
              case PMDCA_LNAND:switch((byte)((*(curinp+3))&PERINPNT_MS)){                   // type dest
                             case DETYEXT:break;
                             case DETYMEM:locmem &= ~mDSmaskbit[ndest];break;             // raz locmem bit
                             case DETYSW://if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         //digitalWrite(pinSw[ndest],OFF);}
                                         //else{
                                         digitalWrite(pinSw[ndest],OFF);
                                         //}
                                         break;
                             default:break;
                             }
                             break;                             
              case PMDCA_TGL:switch((byte)((*(curinp+3))&PERINPNT_MS)){                     // type dest
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
              default:break;
            }
      }else{} // si condition non validée pas d'action : configurer explicitement pour l'inverse si nécessaire
       
    }   // detecFound   
    }   // enable  
  }     // next input
}

/* ------------- gestion pulses ------------- */


void setPulseChg(int sw ,uint64_t* spctl,char timeOT)     // traitement fin de temps 
                                                          // spctl les DLSWLEN bytes d'un switch dans pulseCtl
                                                          // timeOT ='O' fin timeOne ; ='T' fin timeTwo
{Serial.print(" sw=");Serial.print(sw);
  if(timeOT=='O'){
        if((*spctl&(uint64_t)PMTTE_VB)!=0){                                           // cnt2 enable -> run2
          cstRec.cntPulseOne[sw]=0;cstRec.cntPulseTwo[sw]=1;staPulse[sw]=PM_RUN2;Serial.print(" PM_RUN2 ");}
        else {staPulse[sw]=PM_END1;Serial.print(" PM_END1 ");}                                                  // sinon fin1
  }
  else {
        if((*spctl&(uint64_t)PMFRO_VB)==0){                                           // oneshot -> idle
          cstRec.cntPulseTwo[sw]=0;cstRec.cntPulseOne[sw]=0;staPulse[sw]=PM_IDLE;Serial.print(" PM_IDLE ");}
        else if((*spctl&(uint64_t)PMTOE_VB)!=0){                                      // free run -> run      
          cstRec.cntPulseTwo[sw]=0;cstRec.cntPulseOne[sw]=1;staPulse[sw]=PM_RUN1;Serial.print(" PM_RUN1 ");}
        else {staPulse[sw]=PM_END2;Serial.print(" PM_END2 ");}                                                  // free run bloqué -> fin2
  }
  Serial.println();
}

void pulseClkisr()     // poling ou interruption ; action horloge sur pulses tous switchs
{
/*
  uint8_t sw;
  uint64_t spctl=0;//memcpy(&spctl,cstRec.pulseCtl+sw*DLSWLEN,DLSWLEN);
  
  for(sw=0;sw<MAXSW;sw++){
    //Serial.print("pulseClkisr() sw=");Serial.print(sw);Serial.print( " staPulse=");Serial.println(staPulse[sw]);
    switch(staPulse[sw]){
      
      case PM_DISABLE: break;            // changement d'état quand le bit enable d'un compteur sera changé
      
      case PM_IDLE: break;               // changement d'état par l'action d'un détecteur logique

      case PM_RUN1: cstRec.cntPulseOne[sw]++;//printConstant();
                    if(cstRec.cntPulseOne[sw]>=cstRec.durPulseOne[sw]*10){              // (decap cnt1)
                      setPulseChg(sw,&spctl,'O');
                    }break;
                    
      case PM_RUN2: cstRec.cntPulseTwo[sw]++;//printConstant();
                    if(cstRec.cntPulseTwo[sw]>=cstRec.durPulseTwo[sw]*10){              // (decap cnt2)
                      setPulseChg(sw,&spctl,'T');
                    }break;
                    
      case PM_END1: break;               // changement d'état quand le bit enable du compteur 2 sera changé
      case PM_END2: break;               // changement d'état quand le bit enable du compteur 1 sera changé ou changement freerun->oneshot
      default:break;
    }
  }
*/  
}

void hspr16b(uint16_t hp)
{
  if(hp&0xF000==0){Serial.print('0');}Serial.print(hp>>8,HEX);
  if(hp&0x00F0==0){Serial.print('0');}Serial.print(hp,HEX);
}

void isrPul(uint8_t det)                        // maj staPulse (ou switch) au changement d'état d'un détecteur
{                                               // pooling switchs / input 
                                                //   si n° det ok       ... +1
                                                //   si enable          ... +2
                                                //   si local           ... +4
                                                //   si DLMHL==DETBITLH ... +8       (mode flanc descripteur / état memDetec)
                                                // résult = 15 DL ON => exécuter l'action
                                                
  Serial.print(det);Serial.print(" ");
/*
  for(int sw=0;sw<NBSW;sw++){                                                 // explo sw

    Serial.print(sw);Serial.print(":");
    for(int ninp=0;ninp<NBPERINPUT;ninp++){  
        Serial.print(ninp);Serial.print(" ");
        uint64_t spctl=0;//memcpy(&spctl,cstRec.pulseCtl+sw*DLSWLEN,DLSWLEN);     // les DL d'un switch        
        uint16_t spctlnb;//=(uint16_t)(spctl>>(nb*DLBITLEN))&DLBITMSK;           // spctnb les DLBITLEN bits du descripteur de detecteur logique numéro nb
        uint8_t test=0;                                                                                             // si ...
        if( (uint8_t)((spctlnb>>DLNLS_PB)&mask[(DLNMS_PB)-(DLNLS_PB)+1])==det ){ Serial.print("1");test+=1;}        // =det courant
        if( (spctlnb&DLENA_VB)!=0 ){  Serial.print("2");test+=2;}                                                   // enable
        if( (spctlnb&DLEL_VB)!=0  ){  Serial.print("3");test+=4;}                                                   // local
        if( (byte)((spctlnb>>DLMHL_PB)&0x01)==(byte)((cstRec.memDetec[det]>>DETBITLH_PB)&0x01) ){  Serial.print("4");test+=8;}    // LH ok
        
        if( test==15){                                                                                              // dl déclenché
          byte action;//=(byte)((spctlnb>>DLACLS_PB)&mask[DLACMS_PB-DLACLS_PB+1])+1;                                   // exécuter l'action
          byte actions[]={PMDCA_STOP+1,PMDCA_START+1,PMDCA_SHORT+1,PMDCA_RAZ+1,PMDCA_RESET+1,PMDCA_IMP+1,PMDCA_END+1,PMDCA_TGL,0x00};

          byte act=(byte)(strchr((char*)actions,action)-(char*)actions);
          Serial.print(" action=");Serial.print(action);
          switch(act){
            case 0: Serial.print(" stop");
                    staPulse[sw]=PM_IDLE;
                    impDetTime[sw]=0;
                    break;
            case 1: Serial.print(" start");
                    if(cstRec.cntPulseOne[sw]!=0){staPulse[sw]=PM_RUN1;}                   
                    else if(cstRec.cntPulseTwo[sw]!=0){staPulse[sw]=PM_RUN2;}
                    else {staPulse[sw]=PM_RUN1;}
                    impDetTime[sw]=millis();
                    break;
            case 2: Serial.print(" short");
                    if(staPulse[sw]==PM_RUN1 || cstRec.cntPulseOne[sw]!=0){cstRec.cntPulseOne[sw]=cstRec.durPulseOne[sw]*10;}
                    else if(staPulse[sw]==PM_RUN2 || cstRec.cntPulseTwo[sw]!=0){cstRec.cntPulseTwo[sw]=cstRec.durPulseTwo[sw]*10;}
                    impDetTime[sw]=0;
                    break;                 
            case 3: Serial.print(" raz");
                    cstRec.cntPulseOne[sw]=0;
                    cstRec.cntPulseTwo[sw]=0;
                    staPulse[sw]=PM_IDLE;
                    impDetTime[sw]=0;
                    break;               
            case 4: Serial.print(" reset");
                    cstRec.cntPulseOne[sw]=0;cstRec.durPulseOne[sw]=0;
                    cstRec.cntPulseTwo[sw]=0;cstRec.durPulseTwo[sw]=0;
                    staPulse[sw]=PM_IDLE;
                    impDetTime[sw]=0;
                    break;                 
            case 5: Serial.print(" imp");
                    if((millis()-impDetTime[sw])<DETIMP){
                      staPulse[sw]=PM_IDLE;
                      cstRec.cntPulseOne[sw]=0;
                      cstRec.cntPulseTwo[sw]=0;}
                    Serial.print("Time=");Serial.print(millis()-impDetTime[sw]);Serial.print(" ");
                    impDetTime[sw]=0;
                    break;
            case 6: Serial.print(" end");
                    if(staPulse[sw]==PM_RUN1 || cstRec.cntPulseOne[sw]!=0){setPulseChg(sw,&spctl,'O');}
                    else if(staPulse[sw]==PM_RUN2 || cstRec.cntPulseTwo[sw]!=0){setPulseChg(sw,&spctl,'T');}
                    impDetTime[sw]=0;
                    break;
            case 7: Serial.print(" toggle");
                    cstRec.swToggle[sw]=1;forceTrigTemp();
                    break;
            default:Serial.print(" syserr=");Serial.print(action,HEX);Serial.print(" ");
                    staPulse[sw]=PM_DISABLE;
                    break;
          }
        }
      Serial.print(" ");      
      
    }
  }*/
}


void memdetinit()                         // init détecteurs locaux et pulse à 0 à la mise sous tension
{
  Serial.println("init détecteurs");
  byte lev;
  
  for(uint8_t det=0;det<MAXDET;det++){
    lev=digitalRead(pinDet[det]);

    cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
    cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 

    detTime[det]=millis();

    cstRec.memDetec[det] &= ~DETBITST_VB;                           // raz bits ST
    cstRec.memDetec[det] |= DETWAIT<<DETBITST_PB;                   // set bits ST
  }

  Serial.println("init pulses");
  memset(cstRec.cntPulseOne,0x00,sizeof(cstRec.cntPulseOne));
  memset(cstRec.cntPulseTwo,0x00,sizeof(cstRec.cntPulseTwo));
}


void polDx(uint8_t det)              // maj memDetec selon l'état du détecteur det (polDx masqué par tempo debounce) 
                                     // memDetec sert uniquement à mettre le débounce en commun si plusieurs inputs
                                     // utilisent le même détecteur (seul bit utilisé : LH)
{    
    byte lev=digitalRead(pinDet[det]);
    if( ((byte)(cstRec.memDetec[det]>>DETBITLH_PB)&0x01) != lev ){    // niveau lu != niveau actuel de memDetec ?
      // level change -> update memDetec
      cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
      cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 
      detTime[det]=millis();                                          // arme debounce
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
