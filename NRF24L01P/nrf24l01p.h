#ifndef NRF24L01P_H_INCLUDED
#define NRF24L01P_H_INCLUDED

#include <Arduino.h>

#include "nRF24L01.h"

#define NB_NRF 2        // nombre circuit
#define NB_PIPE 6       // nombre pipes par circuit
#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5
#define TO_AVAILABLE 4000

class Nrfp
{
  public:
    Nrfp();
    void setMode(char mode);
    bool setNum(uint8_t circuit,uint8_t balise);
    void hardInit();
    bool config();
    void regRead(uint8_t reg,byte* data,uint8_t len);
    void regWrite(uint8_t reg,byte* data,uint8_t len) ;
    bool available();
    bool dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength);
    void dataWrite(byte* data,char na,uint8_t len,byte* tx_addr);
    int  transmitting(); // busy -> 1 ; empty -> 0 -> Rx ; MAX_RT -> -1

    uint8_t  ce_pin[NB_NRF];
    uint8_t  csn_pin[NB_NRF];

    uint8_t channel;

    byte* r0_addr;    // macAddr si periphérique ; base si concentrateur
    byte* pi_addr;    // péri -> addr d'inscription
                      // conc -> table macAddr associées
    byte* br_addr;
    byte* cc_addr;

  private:

    bool inscript();
    void flushRx();
    void flushTx();
    void pwrUpRx();
    void pwrUpTx();
};

#endif // NRF24L01P INCLUDED

