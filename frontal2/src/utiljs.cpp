#include <Arduino.h>
#include <shconst2.h>
#include "const.h"
#include "utiljs.h"
#include "utilhtml.h"

uint8_t        jsUsrNum;
unsigned long  jsUsrTime;
uint16_t       jsPeriCur;

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
  while ((*c!=0x00) && (*c!=*JSSEP) && (*c!=*JSFON)){
    switch (*c){
      case *JSSCO:strcat(buf,"</td>");tdcat(buf);break;
      case *JSSBR:strcat(buf,"<br>");break;
      case '\n':break;           // ignore LF !
      default: strcat(buf,c);
    }
    txt++;*c=*txt;
  }
}

uint8_t ctlJsB(char* buf,char* jsbuf,char* size,const char* jsCde)
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
      if((a&CTLPO)!=0){*size=*jsbuf++;strcat(buf,"<font size=\"");concat1a(buf,*size);strcat(buf,"\">");}
      jsbuf=dm+lw;
    }
    else {a=0x00;}
    return a;
}

uint8_t ctlJsB(char* buf,char* jsbuf,const char* jsCde)
{
  char bid;
  return ctlJsB(buf,jsbuf,&bid,jsCde);
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

char* jsGetArg(char* jsbuf,uint8_t* sep)  // retourne la position de l'argument, efface le séparateur de fin éventuel 
                                          // pour mettre une fin 0x00 à l'argument
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

char* jsGetEndArg(char* buf,char* jsbuf)   // remplace strcat(buf,jsGetArg(jsbuf)) pour le dernier argument qui n'a pas de séparateur de fin
{
  char* dm0=jsbuf;
  char* dm=jsGetArg(jsbuf);               // something ?
  if(dm!=nullptr && buf!=nullptr){                  
    char* dm1=buf+strlen(buf);
    memcpy(dm1,dm,dm0-dm);}               // pas de séparateur de fin donc strcat ne fonctionne pas
  return dm0;
}

char* jsGetMemAddr(char optNam)
{
    return nullptr;
}

/*void jsPeriCur(char* buf,char* jsbuf)       // (inutilisé?) génère une saisie de periCur 
{
    strcat(buf,"<input type=\"text\" name=\"peri_cur__@\" id=\"nt22\" value=\"");
    strcat(buf,jsGetArg(jsbuf));
    strcat(buf,"\" size=\"1\" maxlength=\"2\" >");    
}*/

void hidAlpha(char* buf,char* jsbuf,const char* type,uint8_t ctlJs)
                        {
                          strcat(buf,"<input type=\"");
                          strcat(buf,type);
                          strcat(buf,"\" name=\"");
                          strcat(buf,jsGetArg(jsbuf));      // nomfonct
                          strcat(buf,"\" value=\"");
                          strcat(buf,jsGetArg(jsbuf));      // valfonct
                          strcat(buf,"\" size=\"");
                          uint8_t sep=0;
                          char* dm=jsGetArg(jsbuf,&sep);
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
                        }

void cvJs2Html(char* jsbuf,char* buf)
{
    char* dm=nullptr;   // pour les retour de jsGetArg
    *buf=0x00;
    char a=*jsbuf;
    uint8_t ctlJs=0x00;
    while(a!=0x00){
        switch(a){
            case *JSCOB:ctlJs=ctlJsB(buf,jsbuf,JSCOB);                  // couleur (JSCOE inutile - pris en charge par ctlJsE)
                        strcat(buf,"<font color=\"");jsGetEndArg(buf,jsbuf);strcat(buf,"\">");
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
            case *JSRJ :ctlJs=ctlJsB(buf,jsbuf,JSRJ);                   // rond jaune
                        strcat(buf,"<div id=\"rond_jaune\"></div>");
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
            case *JSST  :ctlJs=ctlJsB(buf,jsbuf,JSST);                  // disp texte JSSTx[font-size][}width}]texte                        
                        buftxcat(buf,jsGetEndArg(nullptr,jsbuf));
                        ctlJsE(buf,ctlJs);
                        break;
            case *JSNT  :ctlJs=ctlJsB(buf,jsbuf,JSNT);                  // disp nombre JSNTx[pol]Ttt... T type inutilisé ttt... texte
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
                        concat1a(buf,(char)(jsUsrNum+PMFNCHAR));strcat(buf,"=");concatn(buf,jsUsrTime);
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
            case *JSBFB :{char size;
                        ctlJs=ctlJsB(buf,jsbuf,&size,JSBFB);            // bouton fonction JSBFBx[size]ANtt...tt}nomfonc}valfonc}lib
                        char aligncenter=*jsbuf++;
                        jsUsrNum=*jsbuf++;
                        char* jsUsrTime=jsGetArg(jsbuf);
                        strcat(buf,"<a href=\"?user_ref_");
                        concat1a(buf,jsUsrNum);strcat(buf,"=");strcat(buf,jsUsrTime);
                        strcat(buf,"?");
                        strcat(buf,jsGetArg(jsbuf));    // nomfonct
                        strcat(buf,"=");
                        strcat(buf,jsGetArg(jsbuf));    // valfonct
                        strcat(buf,"\">");
                        if(aligncenter=='A'){strcat(buf,"<p align=\"center\">");}
                        strcat(buf,"<input type=\"button\" value=\"");jsGetEndArg(buf,jsbuf); // lib
                        strcat(buf,"\"");
                        if(size=='7'){strcat(buf," style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"");}
                        if(aligncenter){strcat(buf,"></p></a>");}
                        strcat(buf,"></a>");
                        ctlJsE(buf,ctlJs);
                        }break;                  
            case *JSRAD :ctlJs=ctlJsB(buf,jsbuf,JSRAD);                 // saisie boutons radio
                        {dm=jsGetArg(jsbuf);              // nom fonction
                        char nb=*jsbuf++;nb-=PMFNCVAL;    // nombre boutons
                        uint8_t vv=*jsbuf++;vv-=PMFNCVAL; // n° checked
                        for(uint8_t j=0;j<nb;j++){
                          strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,dm);
                          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+j));strcat(buf,"\"");
                          if(j==vv){strcat(buf," checked");}strcat(buf,"/>");
                        }
                        ctlJsE(buf,ctlJs);
                        }break;           
            case *JSRADS :ctlJs=ctlJsB(buf,jsbuf,JSRADS);               // saisie bouton radio carrés spécial remote
                        {dm=jsGetArg(jsbuf);              // nom fonction
                        char nb=*jsbuf++;nb-=PMFNCVAL;    // nombre boutons  
                        uint8_t vv=*jsbuf++;vv-=PMFNCVAL; // n° checked
                        char dir=*jsbuf++;                // 'V' si vertical 'H' si horizontal

                          strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,dm);concat1a(buf,(char)(nb+PMFNCHAR));
                          strcat(buf,"\" class=\"sqbr br_off\" id=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
                          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+0));strcat(buf,"\"");
                          if(vv==0){strcat(buf," checked");}strcat(buf,">");
                          strcat(buf,"<label for=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">OFF</label>\n");

                          if(dir=='V'){strcat(buf,"<br><br>\n");}

                          strcat(buf," <input type=\"radio\" name=\"");strcat(buf,dm);concat1a(buf,(char)(nb+PMFNCHAR));
                          strcat(buf,"\" class=\"sqbr br_on\" id=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
                          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+1));strcat(buf,"\"");
                          if(vv==1){strcat(buf," checked");}strcat(buf,">");
                          strcat(buf,"<label for=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">ON</label>\n");

                          strcat(buf," <input type=\"radio\" name=\"");strcat(buf,dm);concat1a(buf,(char)(nb+PMFNCHAR));
                          strcat(buf,"\" class=\"sqbr br_for\" id=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
                          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+2));strcat(buf,"\"");
                          if(vv==2 || vv==3){strcat(buf," checked");}strcat(buf,">");
                          strcat(buf,"<label for=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">FOR</label>\n");

                        ctlJsE(buf,ctlJs);
                        }break;            
            case *JSNTB :ctlJs=ctlJsB(buf,jsbuf,JSNTB);                 // saisie num JSNTBx[pol]nom}val}TSDL
                        {strcat(buf,"<input type=\"text\" name=\"");
                        strcat(buf,jsGetArg(jsbuf));      // nomfonct
                        strcat(buf,"\" value=\"");
                        strcat(buf,jsGetArg(jsbuf));      // valeur
                        jsbuf++;                          // skip type
                        strcat(buf,"\" size=\"");
                        concat1a(buf,*jsbuf++);           // size
                        strcat(buf,"\"");
                        char dec=*jsbuf++;                // dec
                        char len=*jsbuf++;                // len
                        strcat(buf,"\"");
                        if(len<='2'){
                          strcat(buf,"\" id=\"nt");
                          concat1a(buf,len);
                          concat1a(buf,dec);
                        }
                        strcat(buf,"\" pattern=\"[");
                        if(dec!='0'){strcat(buf,".,");}strcat(buf,"0-9]{1,");
                        if(len!='0'){concat1a(buf,len);}else{concat1a(buf,'9');}
                        strcat(buf,"}\">");
                        ctlJsE(buf,ctlJs);
                        }break;              
            case *JSHID :ctlJs=ctlJsB(buf,jsbuf,JSHID);                 // saisie alpha JSHIDx[size police]name}value}[size}][maxlen]                      
                        hidAlpha(buf,jsbuf,"hidden",ctlJs);
                        break;
            case *JSATB :ctlJs=ctlJsB(buf,jsbuf,JSATB);                 // saisie alpha JSATBx[size police]name}value}[size}][maxlen]
                        hidAlpha(buf,jsbuf,"text",ctlJs);
                        break;
            case *JSDB  :ctlJs=ctlJsB(buf,jsbuf,JSDB);                  // saisie checkbox
                        strcat(buf,"<input type=\"checkbox\" name=\"");
                        jsGetArg(jsbuf);                  // nomfonct
                        strcat(buf,"\" id=\"cb1\" value=\"1\"");
                        if(*jsbuf==*JSCHK){strcat(buf," checked");jsbuf++;}
                        strcat(buf,">");
                        jsGetEndArg(buf,jsbuf);
                        ctlJsE(buf,ctlJs);
                        break;
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
            case *JSTB :ctlJs=ctlJsB(buf,jsbuf,JSTB);                   // table beg JSSTBx[size]police}[B]
                        {strcat(buf,"<table");
                        if(*jsbuf!=*JSSEP){
                          strcat(buf," style=\"font-family: ");
                          strcat(buf,jsGetArg(jsbuf));
                          strcat(buf,"\"");
                        }
                        char tBorder[]={'0','\"',0x00,'1','\"',0x00};
                        uint8_t border=0;
                        if(*jsbuf=='B'){border=1;}
                        strcat(buf," border=\"");
                        strcat(buf,tBorder+border*3);
                        strcat(buf,"\">");
                        ctlJsE(buf,ctlJs);
                        }break;
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
                        {jsPeriCur=(uint8_t)(*jsbuf++)-PMFNCVAL; // periCur
                        jsUsrNum=*jsbuf++;                      // usernum                       
                        char* jsUsrTime=jsGetArg(jsbuf);        // userTime[userNum]
                        char* title=jsGetArg(jsbuf);            // legend
                        char* fonc=jsGetArg(jsbuf);             // fonc
                        strcat(buf,"<form method=\"GET \">");
                        if(title!=nullptr){
                          strcat(buf,"<fieldset><legend>");strcat(buf,title);strcat(buf," :</legend>");} 
                        strcat(buf,"<p hidden><input type=\"text\" name=\"user_ref_");
                        concat1a(buf,jsUsrNum);
                        strcat(buf,"\" value=\"");
                        strcat(buf,jsUsrTime);
                        if(fonc!=nullptr){
                          strcat(buf,"<input type=\"text\" name=\"");strcat(buf,fonc);strcat(buf,"\" value=\"");
                          concat1a(buf,(char)(jsPeriCur+PMFNCVAL));strcat(buf,"\">");
                        }
                        strcat(buf,"</p>");
                        }break;
            default:strcat(buf,"<br><br> commande inconnue :");concat1a(buf,a);strcat(buf,"<br><br>");break;
            a=*jsbuf++;if(a==LF){a=*jsbuf++;}    
        }
    }
}