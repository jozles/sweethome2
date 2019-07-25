#ifndef nrfwords_H_INCLUDED
#define nrfwords_H_INCLUDED


/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* bits masks */
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_RX_DR_BIT BIT6
#define MASK_TX_DS  5
#define MASK_TX_DS_BIT BIT5
#define MASK_MAX_RT 4
#define MASK_MAX_RT_BIT BIT4
#define EN_CRC      3
#define EN_CRC_BIT  BIT3
#define CRCO        2
#define CRCO_BIT    BIT2
#define PWR_UP      1
#define PWR_UP_BIT  BIT1
#define PRIM_RX     0
#define PRIM_RX_BIT BIT0
#define ENAA_P5     5
#define ENAA_P5_BIT BIT5
#define ENAA_P4     4
#define ENAA_P4_BIT BIT4
#define ENAA_P3     3
#define ENAA_P3_BIT BIT3
#define ENAA_P2     2
#define ENAA_P2_BIT BIT2
#define ENAA_P1     1
#define ENAA_P1_BIT BIT1
#define ENAA_P0     0
#define ENAA_P0_BIT BIT0
#define ERX_P5      5
#define ERX_P5_BIT  BIT5
#define ERX_P4      4
#define ERX_P4_BIT  BIT4
#define ERX_P3      3
#define ERX_P3_BIT  BIT3
#define ERX_P2      2
#define ERX_P2_BIT  BIT2
#define ERX_P1      1
#define ERX_P1_BIT  BIT1
#define ERX_P0      0
#define ERX_P0_BIT  BIT0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0
#define RX_DR       6
#define RX_DR_BIT   BIT6
#define TX_DS       5
#define TX_DS_BIT   BIT5
#define MAX_RT      4
#define MAX_RT_BIT  BIT4
#define RX_P_NO     1
#define RX_P_NO_BIT 0x0E
#define TX_FULL     0
#define TX_FULL_BIT BIT0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define FIFO_FULL_BIT BIT5
#define TX_EMPTY    4
#define TX_EMPTY_BIT BIT4
#define RX_FULL     1
#define RX_FULL_BIT BIT1
#define RX_EMPTY    0
#define RX_EMPTY_BIT BIT0
#define EN_DPL      2
#define EN_DPL_BIT  BIT2
#define EN_DYN_ACK  0
#define EN_DYN_ACK_BIT BIT0
#define RF_DR_LOW   5
#define RF_DR_LOW_BIT BIT5
#define RF_DR_HIGH  3
#define RF_DR_HIGH_BIT BIT3
#define RF_PWR_BITS BIT1|BIT2
#define DPL_P5      5
#define DPL_P5_BIT  BIT5
#define DPL_P4      4
#define DPL_P4_BIT  BIT4
#define DPL_P3      3
#define DPL_P3_BIT  BIT3
#define DPL_P2      2
#define DPL_P2_BIT  BIT2
#define DPL_P1      1
#define DPL_P1_BIT  BIT1
#define DPL_P0      0
#define DPL_P0_BIT  BIT0


/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_TX_PAYLOAD_NA  0xB0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


#endif // nrfwords
