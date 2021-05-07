#ifndef PERIPHERIQUE2_H_INCLUDED
#define PERIPHERIQUE2_H_INCLUDED

  /* bits talkStep */
  #define TALKREQBIT 0x10   
  #define TALKGRTBIT 0x20
  #define TALKSTABIT 0xF0   // bits d'état
  #define TALKCNTBIT 0x0F   // bits de step   


void    talkReq();
uint8_t talkSta();


#endif // PERIPHERIQUE2_H_INCLUDED
