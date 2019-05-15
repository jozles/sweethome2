
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


/*extern  int   cntdebug[NBDBPTS];
extern  long  timedebug[NBDBPTS*NBDBOC];
extern  int*  v0debug[NBDBPTS*NBDBOC];
extern  int*  v1debug[NBDBPTS*NBDBOC];
extern  char* v2debug[NBDBPTS*NBDBOC];
extern  char* v3debug[NBDBPTS*NBDBOC];
extern  int*  int0=&(0x00);*/


/* ------------------ généralités -------------------- 

  switchs :

  les switchs sont actionnés par poling de la table des ccommandes de switch 
  via swAction qui utilise les tables pulseCtl, swCde et staPulse

  (actuellement) 2 types : ON et OFF ; OFF est prioritaire

  3 sources possibles pour chaque type : 1 détecteur logique choisi dans la table des détecteurs logiques
                                         le bit de commande du serveur
                                         le générateur d'impulsion du switch

  chaque source a un bit enable et un bit de niveau actif



  détecteurs :

  3 types : physiques, spéciaux, externes ; (spéciaux : bits serveur, alarmes etc ; externes positionnés par commande serveur)
  
  levdet() récupère le niveau de tous types

  les détecteurs (physiques, spéciaux et externes) sont polés et, lorsqu'un changement d'état se produit,
  leur image mémoire est mise à jour dans memDetec

  le croisement de memDetec et pulseCtl fournit l'état du générateur d'impulsion (staPulse)

  pulseCtl (devrait se nommer detecCtl) paramètre le fonctionnement de 4 détecteurs logiques (internes ou externes)
  - n° du détecteur
  - enable 
  - local/externe
  - statique/transitionnel (inutilisé)
  - L/H
  - n° action à exécuter quand : le détecteur en cours de traitement correspond, enable, local/externe ok, L/H ok
  actions :
  - start         active le cnt!=0 ou cnt1 si tout==0 ; stapulse devient run1 ou run2 
  - stop          
  - raz           les 2 compteurs=0 ; stapulse idle
  - reset         les 2 compteurs et les 2 durées =0 ; stapulse idle
  - fin           compteur courant au max puis avance compteur ; si bloqué -> stapulse end1 ou 2 sinon suite normale
  - short         compteur courant au max ; stapulse inchangé
  - impulsion     si impDetTime < DETIMP effectue raz sinon sans effet
  - toogle switch change l'état du(ou des) switch(s) utilisant le détecteur (voir const.h pour les^pb de mise en oeuvre)
  
  
  memDetec contient l'image mémoire des détecteurs locaux (physiques et spéciaux) et externes
  1 bit indique le niveau L/H (suit en continu le niveau du détecteur)
  (1 bit indique le flanc UP/DOWN actif -- inutilisé)
  (2 bits indiquent l'état (DIS/IDLE/WAIT/TRIG) -- inutilisé)
  polDx, à chaque transition, mets à jour le niveau du détecteur concerné et initialise le compteur de debounce
  pendant le debounce pas de poling. 
  le niveau de chaque détecteur est obtenu via levdet
   
---- inutilisé
  placé en mode suspendu  (IDLE) au reset et, si (TRIG) après traitement des pulses
  initialisé en mode armé (WAIT) en fin de débounce si le serveur a le détecteur enable
  placé en mode déclenché (TRIG) par la détection du flanc actif selon le serveur)
----
  
  staPulse est l'état du générateur
  placé en mode débranché (DISABLE) en cas d'erreur système (action invalide d'un détecteur)
  placé en mode suspendu  (IDLE) suite à une action STOP d'un détecteur logique ou en fin de oneshot
  placé en mode comptage  (RUN)  suite à une action START d'un détecteur logique
  placé en mode fin       (END)  lorsque le comptage d'une phase est terminé et que la phase suivante est débranchée 
  staPulse est mis à jour par isrPulse
  l'état des pulses est obtenu par croisement de memDetec et swPulseCtl qui contient le code de l'action à exécuter
 
*/


  

/* -------------- gestion commande switchs ----------------- */

uint8_t rdy(byte modesw,int sw) // pour les 3 sources, check bit enable puis etat source ; retour numéro source valorisé si valide sinon 0
{
/*  modesw copie du mot de contrôle du switch (OffCdeO, OnCdeO, OnCdeI, OffCdeI) 2 bits valide par source ;
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
    }*/
//  }
  return 0;                                      // disable 
}

/*
void swAction()         // poling des switchs pour maj selon règles et disjonteur
                        // pour chaque switch et chaque input, 
                        //      récup valeur détecteur
                        //      comparaison avec valeur demandée (et flanc éventuel) des règles actives (enable) dans l'ordre des priorités
                        //      (disjoncteur, conjoncteur, interrupteur, allumeur)
{ 
  uint8_t detecFound=0;     // indic que detecState est valide
  uint8_t detecState=0;     // valeur trouvée pour l'input (type-n°)
  uint8_t rulbitState=0;    // valeur active pour la règle conj/int
  uint8_t rules=8;          // byte des règles dans swInput 
  
  for(int ns=0;ns<NBSW;ns++){

    uint8_t rulesVal=0;                        // valeur finale de la règle (8 OFF par défaut ; 1 ON conj ; 0 OFF inter ; 2 ON allumeur
    if(((cstRec.swCde>>(ns*2+1))&0x01)!=0){                                          // no disjoncteur ?
      for(int ninp=0;ninp<NBSWINPUT;ninp++){

        uint16_t offs=ns*NBSWINPUT*SWINPLEN+ninp*SWINPLEN;
        uint8_t eni=((*(uint8_t*)(cstRec.swInput+2+offs)>>SWINPEN_PB)&0x01);         // enable
        uint8_t edgestat=((*(uint8_t*)(cstRec.swInput+2+offs)>>SWINPDETES_PB)&0x01); // edge/static
        uint8_t oldlev=((*(uint8_t*)(cstRec.swInput+2+offs)>>SWINPOLDLEV_PB)&0x01);  // oldlev        
        uint8_t typ=*(uint8_t*)(cstRec.swInput+offs)&SWINPNT_MS;                     // type
        uint8_t ndet=(*(uint8_t*)(cstRec.swInput+offs)&SWINPV_MS)>>SWINPNVLS_PB;     // n° det
        uint8_t curact=((*(uint8_t*)(cstRec.swInput+2+offs)>>SWINPACTLS_PB)&0x01);   // action 

        if(eni!=0){
          switch(typ){
            case DETYEXT:detecState=(cstRec.extDetec>>ndet)&0x01;                   // détecteur externe valide
                 detecFound=1;break;
            case DETYLOC:
                 if(cstRec.memDetec[ndet]>>DETBITST_PB != DETDIS) {                 // detecteur pas disable
                    detecState=(cstRec.memDetec[ndet]>>DETBITLH_PB)&0x01;           // détecteur local valide
                    detecFound=1;
                 }
                 break;
            case DETYPUL:break;                                                     // pulse valide ?
/*      uint8_t vPulse=0;
        bool lh=0; // valoriser avec etat actif demandé
        switch(staPulse[ndet]){
          case PM_RUN1: if(!lh){vPulse=1;}break;     // pulse run1=L, demandé L ---> ok
          case PM_RUN2: if( lh){vPulse=1;}break;     // pulse run2=H, demandé H ---> ok
          case PM_END1: if(!lh){vPulse=1;}break;     // pulse end1=L, demandé L ---> ok
          case PM_END2: if( lh){vPulse=1;}break;     // pulse run1=H, demandé H ---> ok
          case PM_IDLE: if(!lh){                     // pulse idle,   demandé L et cnt1 !=0 ---> ok
                                                     //                         et cnt1+cnt2=0 > ok
                                                     //               demandé H et cnt2 !=0 ---> ok
                          if((cstRec.cntPulseOne[ndet]!=0) || ((cstRec.cntPulseOne[ndet]+cstRec.cntPulseTwo[ndet])==0))
                            {vPulse=1;}break;}
                        else {if(cstRec.cntPulseTwo[ndet]!=0){vPulse=1;}break;}
          default: break;
        }
        if(vPulse!=0){          
*/
/*
            default:break;
          }
          if(detecFound!=0){                                                                   // detecState valorisé

            rules=(*(uint8_t*)(cstRec.swInput+1+offs)>>SWINPRULESLS_PB);        
            rulbitState=0;                                                                     
            if((rules&RULBITICVAL)!=0){rulbitState=1;}                                         // état actif pout inter et conj
            
            if(((rules&RULBITCEN)!=0 && (rulbitState==detecState) && edgestat!=0)                                // si conj et état actif et static
                 || ((rules&RULBITCEN)!=0 && (rulbitState==detecState) && oldlev!=detecState && edgestat==0)     // si conj et flanc actif et edge
                 ){         
                //delay(1000);Serial.print("ns=");Serial.print(ns);Serial.print(" ninp=");
                //Serial.print(ninp);Serial.println("; swon ");
                rulesVal=1;ninp=NBSWINPUT;}                                                    // conj ok -> rulesVal=1, next switch
                //digitalWrite(pinSw[ns],ON);ninp=NBSWINPUT;}                                    // ***** ON RULE ***** (next sw)
            else if(((rules&RULBITIEN)!=0 && (rulbitState==detecState) && edgestat!=0)                           // inter  et état actif et static
                 || ((rules&RULBITIEN)!=0 && (rulbitState==detecState) && oldlev!=detecState && edgestat==0)     // inter  et flanc actif et edge                                                                       
                 ){ 
                //delay(1000);Serial.print("ns=");Serial.print(ns);Serial.print(" ninp=");
                //Serial.print(ninp);Serial.println("; swoff ");
                rulesVal=0;}                                                                   // inter ok -> rulesVal=0, next input
                //digitalWrite(pinSw[ns],OFF);}                                                  // ***** OFF RULE *****
            else if (rulesVal>2){                                                              // switch ni conj ni int ni allumé
                rulbitState=0;
                if((rules&RULBITAVAL)!=0){rulbitState=1;}                                      // état actif pout allumeur
                if(((rules&RULBITAEN)!=0 && (rulbitState==detecState) && edgestat!=0)          // allumeur  et état actif et static
                     || ((rules&RULBITAEN)!=0 && (rulbitState==detecState) && oldlev!=detecState && edgestat==0)    // inter  et flanc actif et edge                                                                       
                     ){rulesVal=2;}
            }   // rulesVal>2 (input ni conj ni inter ni allumé)
          }     // found
        }       // input enable
        switch(curact){
          case 0:if(rulesVal==8 || rulesVal==0){digitalWrite(pinSw[ns],OFF);}
                 if(rulesVal==2 || rulesVal==1){digitalWrite(pinSw[ns],ON);}
                 break;
          default: break; 
        // rulesVal 8->OFF 2->ON 1->ON 0->OFF  
        }    
        *(uint8_t*)(cstRec.perInput+2+offs) &= ~(detecState<<PERINPOLDLEV_PB);                      // mise à jour oldlev ON ou OFF
        *(uint8_t*)(cstRec.perInput+2+offs) |= (detecState<<PERINPOLDLEV_PB);      
      }         // next input
    }           // no disjoncteur
    else {      // disjoncteur
      //delay(1000);Serial.print("ns=");Serial.print(ns);Serial.print(" ninp=");Serial.print(ninp);Serial.print("; swoff ");
      digitalWrite(pinSw[ns],OFF);}                                             // ***** OFF (disjoncteur) *****
    //Serial.println();
  }             // next switch
}
*/
void action()           // pour chaque input, test enable,
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
    ndest=(((*curinp+3)&PERINPV_MS)>>PERINPNVLS_PB);
    
    if((*(curinp+2)&PERINPEN_VB)!=0){                           // enable
      switch((*curinp)&PERINPNT_MS){                            // type
        case DETYEXT:detecState=(cstRec.extDetec>>nsrce)&0x01;  // valeur détecteur externe 
             detecFound=1;break;
        case DETYPHY:detecState=(byte)(cstRec.memDetec[nsrce]>>DETBITLH_PB)&0x01;     // valeur détecteur local
             detecFound=1;break;
        case DETYMEM:detecState=(locmem>>nsrce)&0x01;           // valeur intermédiaire
        case DETYPUL:break;                                     // pulse 
        default:break;
      }                                                         

    if(detecFound!=0){
/*
 
 (flanc ok : detecState!=oldState && detecState==req level)
 (level ok : detecState==active level)

  setup oldState

  exécution de l'action si 1
 
*/      
      if(
          (
            ((*(curinp+2)&PERINPDETES_VB)==0)                                 // edge
          &&(detecState!=((*(curinp+2)>>PERINPOLDLEV_VB)&0x01))               // flanc
          &&(detecState==((*(curinp+2)>>PERINPVALID_VB) &0x01))               // flanc ok
          )
          ||                                                                  // ou
          (
            ((*(curinp+2)&PERINPDETES_VB)!=0)                                 // static
          &&(detecState==((*(curinp+2)>>PERINPVALID_VB) &0x01))               // état ok          
          )
        ){ 
            *(curinp+2) &= ~PERINPOLDLEV_VB;                                       // raz bit oldstate
            detecState << PERINPOLDLEV_PB;                                         
            *(curinp+2) |= detecState;                                             // setup oldstate

            uint32_t lmbit;
            switch((byte)(*(curinp+2)>>PERINPACTLS_PB)){                           // exécution action
              case PMDCA_LOR:switch((byte)(*(curinp+2)&PERINPNT_MS)){                     // type dest
                             case DETYEXT:break;
                             case DETYMEM:locmem |= mDSmaskbit[ndest];break;              // or locmembit,1
                             case DETYSW:if(((cstRec.swCde>>(ndest*2+1))&0x01)!=0){       // no disjoncteur ?
                                         digitalWrite(pinSw[ndest],(digitalRead(pinSw[ndest])|ON));}
                                         else{digitalWrite(pinSw[ndest],OFF);}
                                         break;
                             default:break;
                             }
                             break;
              case PMDCA_LAND:switch((byte)(*(curinp+2)&PERINPNT_MS)){                    // type dest
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
              case PMDCA_TGL:switch((byte)(*(curinp+2)&PERINPNT_MS)){                     // type dest
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
{ Serial.print(" sw=");Serial.print(sw);
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
                                     // memDetec :
                                     //    bits DETBITST_  status (disable/idle/wait/trig
                                     //    bit  DETBITLH_  dernier état
                                     //    bit  DETBITUD_  prochain flanc valide 
                                     //                   (le détecteur passe en mode trig-déclenché pour la màj de staPulse puis revient à idle)
                                     
//  if(cstRec.memDetec[det]>>DETBITST_PB != DETDIS) {                     // detecteur pas disable
    
    byte lev=digitalRead(pinDet[det]);
    if( ((byte)(cstRec.memDetec[det]>>DETBITLH_PB)&0x01) != lev ){    // niveau lu != niveau actuel de memDetec ?
      // level change -> update memDetec
      cstRec.memDetec[det] &= ~DETBITLH_VB;                           // raz bits LH
      cstRec.memDetec[det] |= lev<<DETBITLH_PB;                       // set bit LH 
      detTime[det]=millis();                                          // arme debounce
      Serial.print("  >>>>>>>>> det ");Serial.print(det);Serial.print(" change to ");Serial.print(lev);Serial.print(" - ");
/*    
      if( ((byte)(cstRec.memDetec[det]>>DETBITUD_PB)&0x01) == lev ){
        // waited edge
        Serial.print(" edge=");Serial.print((byte)((cstRec.memDetec[det]>>DETBITUD_PB)&0x01),HEX);Serial.print(" ");
        cstRec.memDetec[det] &= ~DETBITST_VB;                           // raz bits ST
        cstRec.memDetec[det] |= DETTRIG<<DETBITST_PB;                   // set bits ST (déclenché)

        //isrPul(det);                                                    // staPulse setup : exploration si ce flanc est prévu dans un dl 

        cstRec.memDetec[det] &= ~DETBITST_VB;                           // raz bits ST    
        cstRec.memDetec[det] |= DETIDLE<<DETBITST_PB;                   // retour Idle

      Serial.println();printConstant();
    }
*/    
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
