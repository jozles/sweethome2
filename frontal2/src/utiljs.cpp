#include <Arduino.h>
#include "utiljs.h"

void buftxcat(char* buf,char* txt)
{
  char c[2]={*txt,'\0'};
  while (*c!=0x00){
    switch (*c){
      case *JSSBR:strcat(buf,"</td><td>");break;
      case *JSLF:strcat(buf,"<br>");break;
      default: strcat(buf,c);
    }
    txt++;*c=*txt;
  }
}
