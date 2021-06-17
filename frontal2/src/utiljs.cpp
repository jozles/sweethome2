#include <Arduino.h>
#include <shconst2.h>
#include "const.h"
#include "utiljs.h"
#include "utilhtml.h"

int            jsUsrNum;
unsigned long* jsUsrTime;


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

uint8_t ctlJsB(char*buf,char*jsbuf,const char* jsCde)
{
    uint8_t a=*(jsCde+1);
    if(a==*JSCTL){
      a=*jsbuf++;
      if((a&TRBEG)!=0){strcat(buf,"<tr>");}
      if((a&TDBEG)!=0){strcat(buf,"<td>");}
      if((a&CTLPO)!=0){strcat(buf,"<font size=\"");concat1a(buf,*jsbuf++);strcat(buf,"\">");}
    }
    else {a=0x00;}
    return a;
}

void ctlJsE(char* buf,uint8_t ctl)
{
    if(ctl!=PMFNCVAL){
      if((ctl&TREND)!=0){strcat(buf,"</tr>");}
      if((ctl&TDBEG)!=0){strcat(buf,"</td>");}
      if((ctl&CTLPO)!=0){strcat(buf,"</font>");}
    }
    strcat(buf,"\n");
}

void cvJs2Html(char* jsbuf,char* buf)
{
    char* dm=jsbuf;
    char* dm1=jsbuf;
    *buf=0x00;
    char a=*jsbuf,ctlJs=0x00;
    while(a!=0x00){
        switch(a){
            case *JSCOB:ctlJs=ctlJsB(buf,jsbuf,JSCOB);                          // couleur
                        dm=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}
                        strcat(buf,"<font color=\"");strcat(buf,dm);strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNB:ctlJs=ctlJsB(buf,jsbuf,JSFNB);                          // font size
                        strcat(buf,"<font size=\"");concat1a(buf,*jsbuf++);strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNE:ctlJs=ctlJsB(buf,jsbuf,JSFNE);                          // font size end
                        strcat(buf,"</font>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSHIDB:ctlJs=ctlJsB(buf,jsbuf,JSHIDB);                        // hide beg
                        strcat(buf,"<p hidden>");
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSHIDE:ctlJs=ctlJsB(buf,jsbuf,JSHIDE);                        // hide end
                        strcat(buf,"</p>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSAC :ctlJs=ctlJsB(buf,jsbuf,JSAC);                           // align center beg
                        strcat(buf,"<p align=\"center\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSACE:ctlJs=ctlJsB(buf,jsbuf,JSACE);                          // align center end
                        strcat(buf,"</p>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBRB :ctlJs=ctlJsB(buf,jsbuf,JSBRB);                         // bouton Retour                        
                        strcat(buf,"<a href=\"?user_ref_");
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));strcat(buf,"=");concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
                        dm=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}
                        strcat(buf,dm);
                        strcat(buf,"\"></a>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBMB :ctlJs=ctlJsB(buf,jsbuf,JSBMB);                         // bouton MàJ
                        strcat(buf,"<input type=\"submit\" value=\"");
                        dm=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}
                        strcat(buf,dm);
                        strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBFB :ctlJs=ctlJsB(buf,jsbuf,JSBFB);                         // bouton fonction
                        {char b[]={(char)(jsUsrNum+PMFNCHAR),'\0'};
                        strcat(buf,"<a href=\"?user_ref_");
                        strcat(buf,b);strcat(buf,"=");concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"?");
                        dm=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}strcat(buf,dm);    // nomfonct
                        strcat(buf,"=");
                        dm=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}strcat(buf,dm);    // valfonct
                        dm=jsbuf;strcat(buf,"\">");
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}                   // size
                        dm1=jsbuf;
                        while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
                        if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}                   // lib
                        strcat(buf,"<input type=\"button\" value=\"");strcat(buf,dm1);strcat(buf,"\"");
                        if(*dm=='7'){strcat(buf," style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"");}
                        strcat(buf,"></a>");
                        }break;                        
                                                              
            default:break;
            a=*jsbuf++;if(a==LF){a=*jsbuf++;}    
        }
    }
}