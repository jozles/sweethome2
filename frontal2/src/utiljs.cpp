#include <Arduino.h>
#include <shconst2.h>
#include "const.h"
#include "utiljs.h"
#include "utilhtml.h"

int            jsUsrNum;
unsigned long* jsUsrTime;

extern bool       borderparam;
extern uint16_t   styleTdWidth;
extern const char*  courier;
extern char*      styleTdFont;
extern uint16_t   styleTdFSize;

#define LCTDFNT 16
char currentTdFont[LCTDFNT];

void buftxcat(char* buf,char* txt)
{
  char c[2]={*txt,'\0'};
  while (*c!=0x00){
    switch (*c){
      case *JSSCO:strcat(buf,"</td>");tdcat(buf);break;
      case *JSSBR:strcat(buf,"<br>");break;
      case '\n':break;           // ignore LF !
      default: strcat(buf,c);
    }
    txt++;*c=*txt;
  }
}

uint8_t ctlJsB(char* buf,char* jsbuf,const char* jsCde)
/*    buf ptr début buffer sortie 
      jsbuf ptr sur caractère suivant le code commande
      jsCde ptr dans table des commandes

      décode le début des fonctions : x[s][{p...}]    x controle, s font size size, p... width pixels  
                          et génère : [<tr>][<td[ style="width:nnnpx"]>][<font size="n">]   
      retourne 0 ou controle ; buf inchangé, jsbuf sur le caractère suivant */
{
    uint8_t a=*(jsCde+1);
    
    if(a==*JSCTL){
      
      bool width=false;
      uint8_t lw=0;
      char* dm=jsbuf;
      a=*jsbuf++;
      if((a&CTLPO)!=0){dm++;}                   // search for JSSEP if width present
      if(*dm++==*JSSEP){                        // found ; now search end
          width=true;
          while(*(dm+lw++)!=*JSSEP){}           // lw char to skip ; next operand at dm+lw
          if(lw>0x05){lw=5;}                    // maxi 4+JSSEP (9999Px)
      }       

      if((a&TRBEG)!=0){strcat(buf,"<tr>");}
      if((a&TDBEG)!=0){
        strcat(buf,"<td");
        if(width){strcat(buf," style=\"width: ");memcpy(buf,dm,lw-1);strcat(buf,"px\"");}
        strcat(buf,">");
      }
      if((a&CTLPO)!=0){strcat(buf,"<font size=\"");concat1a(buf,*jsbuf++);strcat(buf,"\">");}
      jsbuf=dm+lw;
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

char* jsGetArg(char* jsbuf,uint8_t* sep)  // retourne la position de l'argument, efface le séparateur de fin éventuel pour mettre une fin 0x00 à l'argument
                                          // et avance le ptr sur la suite
                                          // si position==nullptr pas d'argument
                                          // retour sep==0 pas de séparateur (donc dernier argument)
{
    *sep=0;
    char* dm=jsbuf;
    if(*dm!=*JSFON && *dm!=LF && *dm!=0x00){
      while(*jsbuf!=*JSSEP && *jsbuf!=LF && *jsbuf!=0x00){jsbuf++;}
      if(*jsbuf==*JSSEP){*jsbuf=0;jsbuf++;*sep=1;}
    }
    return dm;
}

char* jsGetArg(char* jsbuf)
{
  uint8_t bid;
  return jsGetArg(jsbuf,&bid);
}

void jsGetEndArg(char* buf,char* jsbuf)   // remplace strcat(buf,jsGetArg(jsbuf)) pour le dernier argument qui n'a pas de séparateur de fin
{
  char* dm=jsGetArg(jsbuf);               // something ?
  if(dm!=nullptr){                  
    char* dm0=buf+strlen(buf);
    memcpy(dm0,dm,dm0-dm);}               // pas de séparateur de fin donc strcat ne fonctionne pas
}

char* jsGetMemAddr(char optNam)
{
    return nullptr;
}

void jsPeriCur(char* buf,char* jsbuf)       // (inutilisé?) génère une saisie de periCur 
{
    strcat(buf,"<input type=\"text\" name=\"peri_cur__@\" id=\"nt22\" value=\"");
    strcat(buf,jsGetArg(jsbuf));
    strcat(buf,"\" size=\"1\" maxlength=\"2\" >");    
}

void cvJs2Html(char* jsbuf,char* buf)
{
    uint8_t sep=0;      // indique la présence d'un séparateur de fin en retour de jsGetArg
    char* dm=nullptr;   // pour les retour de jsGetArg
    *buf=0x00;
    char a=*jsbuf,ctlJs=0x00;
    while(a!=0x00){
        switch(a){
            case *JSCOB:ctlJs=ctlJsB(buf,jsbuf,JSCOB);                  // couleur
                        strcat(buf,"<font color=\"");strcat(buf,jsGetArg(jsbuf));strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNB:ctlJs=ctlJsB(buf,jsbuf,JSFNB);                  // font size
                        strcat(buf,"<font size=\"");concat1a(buf,*jsbuf++);strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFNE:ctlJs=ctlJsB(buf,jsbuf,JSFNE);                  // font size/color end
                        strcat(buf,"</font>");
                        ctlJsE(buf,ctlJs);
                        break;
/*            case *JSHIDB:ctlJs=ctlJsB(buf,jsbuf,JSHIDB);                // hide beg
                        strcat(buf,"<p hidden>");
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSHIDE:ctlJs=ctlJsB(buf,jsbuf,JSHIDE);                // hide end
                        strcat(buf,"</p>");
                        ctlJsE(buf,ctlJs);
                        break;*/
            case *JSAC :ctlJs=ctlJsB(buf,jsbuf,JSAC);                   // align center beg
                        strcat(buf,"<p align=\"center\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSACE:ctlJs=ctlJsB(buf,jsbuf,JSACE);                  // align center end
                        strcat(buf,"</p>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSSP :ctlJs=ctlJsB(buf,jsbuf,JSSP);                   // space
                        strcat(buf," ");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSTDS :ctlJs=ctlJsB(buf,jsbuf,JSTDS);                 // tdSet JSTDSxwidth}font}font-size
                        styleTdWidth=*jsbuf-PMFNCVAL;jsbuf+=2;          // width
                        memcpy(currentTdFont,jsGetArg(jsbuf),LCTDFNT);  // police
                        styleTdFont=currentTdFont;
                        styleTdFSize=*jsbuf-PMFNCVAL;jsbuf++;           // font-size
                        ctlJsE(buf,ctlJs);
                        break;   
            case *JSTDR :ctlJs=ctlJsB(buf,jsbuf,JSTDR);                 // reset Td
                        styleTdWidth=0;
                        styleTdFont=nullptr;
                        styleTdFSize=0;
                        ctlJsE(buf,ctlJs);
                        break;                          
            case *JSST  :ctlJs=ctlJsB(buf,jsbuf,JSST);                   // texte JSSTx[police][}width}]texte
                        buftxcat(buf,jsGetArg(jsbuf));              //////////////////////////////////////////////////// jsGetarg de fin
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSNT  :ctlJs=ctlJsB(buf,jsbuf,JSNT);                   // nombre 
                        jsbuf++;    // skip type
                        jsGetEndArg(buf,jsbuf);
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSNTI:ctlJs=ctlJsB(buf,jsbuf,JSNTI);                  // nombre avec couleur
                        strcat(buf,"<font color=\"");strcat(buf,jsGetArg(jsbuf));strcat(buf,"\">");
                        jsGetEndArg(buf,jsbuf);
                        strcat(buf,"</font>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBRB :ctlJs=ctlJsB(buf,jsbuf,JSBRB);                 // bouton Retour                        
                        strcat(buf,"<a href=\"?user_ref_");
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));strcat(buf,"=");concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
                        jsGetEndArg(buf,jsbuf);
                        strcat(buf,"\"></a>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBMB :ctlJs=ctlJsB(buf,jsbuf,JSBMB);                 // bouton MàJ
                        strcat(buf,"<input type=\"submit\" value=\"");
                        jsGetEndArg(buf,jsbuf);
                        strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSBFB :ctlJs=ctlJsB(buf,jsbuf,JSBFB);                 // bouton fonction
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
            case *JSNTB :ctlJs=ctlJsB(buf,jsbuf,JSNTB);                 // saisie numérique
                        strcat(buf,"<input type=\"text\" name=\"");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        //strcat(buf,"\" id=\"nt");
                        
                        strcat(buf,jsGetArg(jsbuf));    // dec
                        strcat(buf,"\" value=\"");
                        jsbuf++;                        // skip type
                        strcat(buf,jsGetArg(jsbuf));   
                        strcat(buf,"\" size=\"");strcat(buf,jsGetArg(jsbuf));      
                        strcat(buf,"\" maxlength=\"");
                        jsGetEndArg(buf,jsbuf);
                        strcat(buf,"\" >");
                        ctlJsE(buf,ctlJs);
                        break;                        
            case *JSATB  :ctlJs=ctlJsB(buf,jsbuf,JSATB);                // saisie alpha JSATBx[size police]name}value}[size}][maxlen]
                        strcat(buf,"<input type=\"text\" name=\"");
                        strcat(buf,jsGetArg(jsbuf));      // nomfonct
                        strcat(buf,"\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));      // valfonct
                        strcat(buf,"\" size=\"");
                        dm=jsGetArg(jsbuf,&sep);
                        if(sep!=0){                       // size présent
                          strcat(buf,dm);
                          strcat(buf,"\"");
                          dm=jsGetArg(jsbuf);}            // len ?
                        else {strcat(buf,"12\"");}                      
                        if(dm!=nullptr){                  // len présent
                          strcat(buf,"maxlength=\"");
                          memcpy(buf,dm,jsbuf-dm);}       // pas de séparateur de fin donc strcat ne fonctionne pas
                        strcat(buf,"\" >");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSDB    :ctlJs=ctlJsB(buf,jsbuf,JSDB);                // saisie checkbox
                          strcat(buf,"<input type=\"checkbox\" name=\"");
                          jsGetArg(jsbuf);                  // nomfonct
                          strcat(buf,"\" id=\"cb1\" value=\"1\"");
                          if(*jsbuf==*JSCHK){strcat(buf," checked");jsbuf++;}
                          strcat(buf,">");
                          jsGetEndArg(buf,jsbuf);
                          ctlJsE(buf,ctlJs);
                          break;
            case *JSRADS  :ctlJs=ctlJsB(buf,jsbuf,JSRADS);              // saisie checkbox big
                          {dm=jsGetArg(jsbuf);               // nom fonction
                          char nb=*jsbuf++;
                          uint8_t vv=*jsbuf++;vv-=PMFNCVAL;
                          for(uint8_t i=0;i<3;i++){
                            strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,dm);concat1a(buf,nb);
                            strcat(buf,"\" class=\"sqbr br_off\" id=\"sqbrb");concat1a(buf,nb);strcat(buf,"\"");
                            strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+0));strcat(buf,"\"");
                            if(vv==i or (vv==3 && i==2)){strcat(buf," checked");}strcat(buf,">");
                            strcat(buf,"<label for=\"sqbrb");concat1a(buf,nb);strcat(buf,"\">OFF</label>\n");
                          }
                          }break;
            case *JSSLD :ctlJs=ctlJsB(buf,jsbuf,JSSLD);                 // saisie slider (cb) JSSLDx[size police]name}value}[^][R]
                          strcat(buf,"<label class=\"switch\"><input type=\"checkbox\" style=\"color:Khaki;\" name=\"");
                          jsGetArg(jsbuf);
                          strcat(buf,"\" value=\"1\""); 
                          if(*jsbuf==*JSCHK){strcat(buf," checked");jsbuf++;}
                          strcat(buf," ><span class=\"slider ");
                          if(*jsbuf=='R'){strcat(buf," round");jsbuf++;}
                          strcat(buf,"\"></span></label>");
                        break;
            case *JSSTB :ctlJs=ctlJsB(buf,jsbuf,JSSTB);                 // select box
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
            case *JSTB :ctlJs=ctlJsB(buf,jsbuf,JSTB);                   // table beg JSSTBx[pol]
                        strcat(buf,"<table>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSTE :ctlJs=ctlJsB(buf,jsbuf,JSTE);                   // table end
                        strcat(buf,"</table>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFB :ctlJs=ctlJsB(buf,jsbuf,JSFB);                   // form beg
                        strcat(buf,"<form>");
                        if(*jsbuf!=0x00 && *jsbuf!=*JSFON){
                            strcat(buf,"<fieldset><legend>");
                            jsGetEndArg(buf,jsbuf);                     // titre cadre
                            strcat(buf," :</legend>");
                        }
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFE :ctlJs=ctlJsB(buf,jsbuf,JSFE);                   // form end
                        strcat(buf,"</form>");
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSFBH:ctlJs=ctlJsB(buf,jsbuf,JSFBH);                  // form Intro
                        strcat(buf,"<form method=\"GET \">");
                        strcat(buf,"<p hidden><input type=\"text\" name=\"user_ref_");
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));
                        strcat(buf,"\" value=\"");
                        concatn(buf,jsUsrTime[jsUsrNum]);
                        strcat(buf,"\">");
                        strcat(buf,"/p");
                        break;
            default:strcat(buf,"<br><br> commande inconnue :");concat1a(buf,a);strcat(buf,"<br><br>");break;
            a=*jsbuf++;if(a==LF){a=*jsbuf++;}    
        }
    }
}