#ifndef NRF24L01P
#define NRF24L01P

#include <Arduino.h>

#include "nRF24L01.h"

#define NB_NRF 2        // nombre circuit (mini 2 : 1=balise/inscripteur, 2=concentrateur)
#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5
#define TO_AVAILABLE 4000

class Nrfp
{
  public:
    Nrfp();
    void hardInit();
    bool config();
    void regRead(uint8_t reg,byte* data,uint8_t len);
    void regWrite(uint8_t reg,byte* data,uint8_t len) ;
    void pwrUpRx();
    void pwrUpTx();
    int  transmitting(); // busy -> 1 ; empty -> 0 -> Rx ; MAX_RT -> -1
    bool available();
    void flushRx();
    void flushTx();
    void dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength);
    void dataWrite(uint8_t pipe,byte* data,char na,uint8_t len,byte* tx_addr);
    bool setNum(uint8_t circuit,uint8_t balise);
    void setMode(char mode);
    bool inscript();


    uint8_t  ce_pin[NB_NRF];
    uint8_t  csn_pin[NB_NRF];

    uint8_t channel;

    byte* r1_addr;    // macAddr si periph�rique ; base si concentrateur
    byte* pi_addr;    // p�ri -> addr d'inscription
                      // conc -> table macAddr associ�es
    byte* br_addr;
    byte* cc_addr;

};

#endif // NRF24L01P INCLUDED

