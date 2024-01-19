#ifndef NRFUSER_H_INCLUDED
#define NRFUSER_H_INCLUDED

#include <Arduino.h>
#include <nrf24l01s_const.h>

#if NRF_MODE == 'P'

/**** insertion du code utilisateur ****
  
  Le module se réveille toutes les 8 secondes, incrémente 2 compteurs 
  et se rendort si l'un n'a pas atteint aw_min et l'autre aw_ok
  aw_min est le nombre maximum de réveils depuis la dernière communication avec le concentrateur
  aw_ok est le nombre de réveils pour déclencher checkTings()
  
  checkThings() reçoit les opérations à effectuer lors de chaque réveil "utile" du module
    si l'argument retryCnt est !=0 c'est qu'une répétition est en cours suite à un pb de transmission
    le retour de checkTings indique s'il faut effectuer une transmission (true=oui)
  
  messageBuild() permet de concaténer des données au champ message ; incrémenter *messageLength de la longueur ajoutee
    *messageLength ne doit pas dépasser 32
    Les 6 premiers caractères de message sont réservés pour le système 
    Les 10 suivants contiennent le numéro de version du système et le temps écoulé depuis le dernier reset 
    en secondes selon l'oscillateur local du module ; ils sont facultatifs et peuvent être écrasés

  importData() permet de récupérer les données à destination du module en provenance du système 
    avec lequel communique le concentrateur

  userResetSetup() le code à effectuer au reset du module

  userHardPowerDown() le code à effectuer avant chaque mise en sommeil après un réveil utile
  
*/


bool checkThings(uint8_t awakeCnt,uint8_t awakeMinCnt,uint8_t retryCnt);
void messageBuild(char* message,uint8_t* messageLength);
void importData(byte* data,uint8_t dataLength);
void userResetSetup();
void userHardPowerDown();

#endif // NRF_MODE == 'P'

#endif // NRFUSER_H_INCLUDED
