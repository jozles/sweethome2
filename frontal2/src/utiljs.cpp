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
      if((ctl&TDEND)!=0){strcat(buf,"</td>");}
      if((ctl&CTLPO)!=0){strcat(buf,"</font>");}
    }
    strcat(buf,"\n");
}

char* jsGetArg(char* jsbuf)
{
    char* dm=jsbuf;
    while(*jsbuf!=0x00 && *jsbuf!=*JSSEP){jsbuf++;}
    if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;}
    return dm;
}

char* jsGetMemAddr(char optNam)
{
    return nullptr;
}

void jsPeriCur(char* buf,char* jsbuf)
{
    strcat(buf,"<input type=\"text\" name=\"peri_cur__@\" id=\"nt22\" value=\"");
    strcat(buf,jsGetArg(jsbuf));
    strcat(buf,"\" size=\"1\" maxlength=\"2\" >");    
}

void cvJs2Html(char* jsbuf,char* buf)
{
    *buf=0x00;
    char a=*jsbuf,ctlJs=0x00;
    while(a!=0x00){
        switch(a){
            case *JSCOB:ctlJs=ctlJsB(buf,jsbuf,JSCOB);                          // couleur
                        strcat(buf,"<font color=\"");strcat(buf,jsGetArg(jsbuf));strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNB:ctlJs=ctlJsB(buf,jsbuf,JSFNB);                          // font size
                        strcat(buf,"<font size=\"");concat1a(buf,*jsbuf++);strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNE:ctlJs=ctlJsB(buf,jsbuf,JSFNE);                          // font size/color end
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
            case *JSSP :ctlJs=ctlJsB(buf,jsbuf,JSSP);                           // space
                        strcat(buf," ");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSST :ctlJs=ctlJsB(buf,jsbuf,JSST);                           // texte 
                        buftxcat(buf,jsGetArg(jsbuf));
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSNT :ctlJs=ctlJsB(buf,jsbuf,JSNT);                           // nombre 
                        jsbuf++;    // skip type
                        strcat(buf,jsGetArg(jsbuf));
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSNTI:ctlJs=ctlJsB(buf,jsbuf,JSNTI);                          // nombre avec couleur
                        strcat(buf,"<font color=\"");strcat(buf,jsGetArg(jsbuf));strcat(buf,"\">");
                        strcat(buf,jsGetArg(jsbuf));
                        strcat(buf,"</font>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBRB :ctlJs=ctlJsB(buf,jsbuf,JSBRB);                         // bouton Retour                        
                        strcat(buf,"<a href=\"?user_ref_");
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));strcat(buf,"=");concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));
                        strcat(buf,"\"></a>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBMB :ctlJs=ctlJsB(buf,jsbuf,JSBMB);                         // bouton MàJ
                        strcat(buf,"<input type=\"submit\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));
                        strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBFB :ctlJs=ctlJsB(buf,jsbuf,JSBFB);                         // bouton fonction
                        {char b[]={(char)(jsUsrNum+PMFNCHAR),'\0'};
                        strcat(buf,"<a href=\"?user_ref_");
                        strcat(buf,b);strcat(buf,"=");concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"?");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        strcat(buf,"=");
                        strcat(buf,jsGetArg(jsbuf));    // valfonct
                        strcat(buf,"\">");
                        strcat(buf,"<input type=\"button\" value=\"");strcat(buf,jsGetArg(jsbuf)); // lib
                        strcat(buf,"\"");   
                        if(*jsGetArg(jsbuf)=='7'){strcat(buf," style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"");}
                        strcat(buf,"></a>");
                        ctlJsE(buf,ctlJs);
                        }break;                        
            case *JSNTB :ctlJs=ctlJsB(buf,jsbuf,JSNTB);                         // saisie numérique
                        strcat(buf,"<input type=\"text\" name=\"");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        strcat(buf,"\" id=\"nt");
                        strcat(buf,jsGetArg(jsbuf));    // len
                        strcat(buf,jsGetArg(jsbuf));    // dec
                        strcat(buf,"\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));   
                        strcat(buf,"\" size=\"");strcat(buf,jsGetArg(jsbuf));      
                        strcat(buf,"\" maxlength=\"");strcat(buf,jsGetArg(jsbuf)); 
                        strcat(buf,"\" >");
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSATB  :ctlJs=ctlJsB(buf,jsbuf,JSATB);                        // saisie alpha
                        strcat(buf,"<input type=\"text\" name=\"");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        strcat(buf,"\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));    // valfonct
                        strcat(buf,"\" size=\"12\" maxlength=\"");
                        strcat(buf,jsGetArg(jsbuf));    // maxlen
                        strcat(buf,"\" >");
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSDB  :ctlJs=ctlJsB(buf,jsbuf,JSDB);                          // saisie checkbox
                        strcat(buf,"<input type=\"checkbox\" name=\"");
                        jsGetArg(jsbuf);                // nomfonct
                        strcat(buf,"\" id=\"cb1\" value=\"1\"");
                        if(*jsbuf==*JSCHK){strcat(buf," checked");jsbuf++;}
                        strcat(buf,">");
                        strcat(buf,jsGetArg(jsbuf));    // etat
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSSTB :ctlJs=ctlJsB(buf,jsbuf,JSSTB);                          // select box
                        {strcat(buf,"<SELECT name=\"");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        strcat(buf,"\">");
                        uint8_t optSel=(*jsbuf++)-PMFNCVAL;
                        char optNam=*jsbuf++;
                        char* optAddr=jsGetMemAddr(optNam);
                        char a;
                        uint8_t optN=*optAddr-PMFNCVAL;
                        uint8_t optL=*(optAddr+1)-PMFNCVAL;
                        optAddr+=2;     // skip n,l
                        for(uint8_t i=0;i<optN;i++){
                            strcat(buf,"<OPTION");if(i==optSel){strcat(buf," selected");};strcat(buf,">");
                            for(uint8_t j=0;j<optL;j++){
                                a=optAddr[i*optL+j];
                                if(a!=' '){concat1a(buf,a);}
                            }
                        }
                        strcat(buf,"</SELECT>");
                        ctlJsE(buf,ctlJs);
                        }break;
            case *JSTB :ctlJs=ctlJsB(buf,jsbuf,JSTB);                          // table beg
                        strcat(buf,"<table>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSTE :ctlJs=ctlJsB(buf,jsbuf,JSTE);                          // table end
                        strcat(buf,"</table>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFB :ctlJs=ctlJsB(buf,jsbuf,JSFB);                          // form beg
                        strcat(buf,"<form>");
                        if(*jsbuf!=0x00 && *jsbuf!=*JSFON){
                            strcat(buf,"<fieldset><legend>");
                            strcat(buf,jsGetArg(jsbuf));      // titre cadre
                            strcat(buf," :</legend>");
                        }
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFE :ctlJs=ctlJsB(buf,jsbuf,JSFE);                           // form end
                        strcat(buf,"</form>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFBH:ctlJs=ctlJsB(buf,jsbuf,JSFBH);                          // form Intro
                        strcat(buf,"<form method=\"GET \">");
                        strcat(buf,"<p hidden><input type=\"text\" name=\"user_ref_");
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));
                        strcat(buf,"\" value=\"");
                        concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"\">");
                        jsPeriCur(buf,jsbuf);
                        strcat(buf,"/p");
                        break;
            default:strcat(buf,"<br><br> commande inconnue :");concat1a(buf,a);strcat(buf,"<br><br>");break;
            a=*jsbuf++;if(a==LF){a=*jsbuf++;}    
        }
    }
}