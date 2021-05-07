#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>
#include "ds3231.h"
#include <shconst2.h>
#include <shutil2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "pageshtml.h"
#include "utilhtml.h"

extern Ds3231 ds3231;

extern char*     chexa;

extern uint8_t   remote_IP[4],remote_IP_cur[4];

extern char      periRec[PERIRECLEN];        // 1er buffer de l'enregistrement de périphérique
  
extern uint16_t  periCur;                    // Numéro du périphérique courant

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

extern uint16_t  perrefr;

extern char*     usrnames; 
extern char*     usrpass; 
extern unsigned long* usrtime;  

extern int       usernum;
//extern int       chge_pwd;

extern byte mask[];
extern char pkdate[7];

#define LENCOLOUR 8
  char colour[LENCOLOUR+1];

void concat1a(char* buf,char a)
{
  concat1a(buf,nullptr,a);
}

void concat1a(char* buf,char* jsbuf,char a)
{
  char b[2];b[1]='\0';
  b[0]=(char)(a);strcat(buf,b);
  jscat(jsbuf,b);
}

void concat1aH(char* buf,char a)
{
  char b[]="\0\0\0";
  if(a<16){b[0]='0';}
  else {b[0]=chexa[a>>4];}
  b[1]=chexa[a&0x0f];
  strcat(buf,b);
}

void concatn(char* buf,unsigned long val)
{
  concatn(buf,nullptr,val);
}

void concatn(char* buf,char* jsbuf,unsigned long val)
{
  uint16_t b,s;
  char* a;
  char* dm=buf+strlen(buf);
  b=strlen(buf);a=buf+b;s=sprintf(a,"%lu",val);buf[b+s]='\0';
  jscat(jsbuf,dm);
}

void concatns(char* buf,long val)
{
  concatns(buf,nullptr,val);
}

void concatns(char* buf,char* jsbuf,long val)
{
  uint16_t b,s;
  char *dm=buf+strlen(buf);
  b=strlen(buf);s=sprintf(buf+b,"%lu",val);buf[b+s]='\0';
  jscat(jsbuf,dm);
}

void concatnf(char* buf,float val)
{
  concatnf(buf,nullptr,val,2);
}

void concatnf(char* buf,float val,uint8_t dec)
{
  concatnf(buf,nullptr,val,dec);
}

void concatnf(char* buf,char* jsbuf,float val)
{
  concatnf(buf,jsbuf,val,2);
}

void concatnf(char* buf,char* jsbuf,float val,uint8_t dec)
{
  uint16_t b,s;
  char* dm=buf+strlen(buf);
  char d[]="%.2f";
  d[2]=(char)(dec+0x30);
  b=strlen(buf);
  s=sprintf(buf+b,d,val);buf[b+s]='\0';
  jscat(jsbuf,dm);
}

void alphaTableHtmlB(char* buf,char* jsbuf,const char* valfonct,const char* nomfonct,int len)
{
  strcat(buf,"<td><input type=\"text\" name=\"");jscat(jsbuf,JSCB);
  strcat(buf,nomfonct);jscat(jsbuf,nomfonct);
  strcat(buf,"\" value=\"");
  strcat(buf,valfonct);jscat(jsbuf,valfonct);
  strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,jsbuf,len);
  strcat(buf,"\" ></td>\n");jscat(jsbuf,JSCE);
}

void alphaTableHtmlB(char* buf,const char* valfonct,const char* nomfonct,int len)
{
  alphaTableHtmlB(buf,nullptr,valfonct,nomfonct,len);
}

void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol)
{
  numTf(buf,nullptr,type,valfonct,nomfonct,len,td,pol,0);
}

void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol)
{
  numTf(buf,jsbuf,type,valfonct,nomfonct,len,td,pol,0);
}

void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec)
{
  numTf(buf,nullptr,type,valfonct,nomfonct,len,td,pol,dec);
}

void numTf(char* buf,char* jsbuf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec)
{                          
  if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSCB);}
  jscat(jsbuf,JSNTB);
  if(pol!=0){strcat(buf,"<font size=\"");jscat(jsbuf,JSPB);concatn(buf,jsbuf,pol);strcat(buf,"\">");jscat(jsbuf,JSPE);}
  strcat(buf,"<input type=\"text\" name=\"");strcat(buf,nomfonct);jscat(jsbuf,nomfonct);
  if(len<=2){strcat(buf,"\" id=\"nt");concatn(buf,jsbuf,len);}
  strcat(buf,"\" value=\"");
  switch (type){
    case 'b':concatn(buf,jsbuf,*(byte*)valfonct);break; //strcat(buf,(char*)valfonct);break;
    case 'd':concatn(buf,jsbuf,*(uint16_t*)valfonct);break;
    case 's':if(*(uint8_t*)valfonct==0xff){break;}concatn(buf,jsbuf,*(uint8_t*)valfonct);break;
    case 'i':concatns(buf,jsbuf,*(int*)valfonct);break;
    case 'I':concatns(buf,jsbuf,*(int16_t*)valfonct);break;
    case 'r':concatnf(buf,jsbuf,(float)(*(int16_t*)valfonct)/100);break;
    case 'l':concatns(buf,jsbuf,*(long*)valfonct);break;
    case 'f':concatnf(buf,jsbuf,*(float*)valfonct);break;
    case 'F':concatnf(buf,jsbuf,*(float*)valfonct,dec);break;
    case 'g':concatn(buf,jsbuf,*(uint32_t*)valfonct);break;    
    default:break;
  }
  int sizeHtml=1;if(len>=3){sizeHtml=2;}if(len>=6){sizeHtml=4;}if(len>=9){sizeHtml=6;}
  strcat(buf,"\" size=\"");concatn(buf,jsbuf,sizeHtml);strcat(buf,"\" maxlength=\"");concatn(buf,jsbuf,len);strcat(buf,"\" >");
  if(pol!=0){strcat(buf,"</font>");}
  jscat(jsbuf,JSNTE);
  if(td==1 || td==3){strcat(buf,"</td>\n");jscat(jsbuf,JSCE);}
}
/*
void numTf(char* buf,char type,void* valfonct,const char* nomfonct,int len,uint8_t td,int pol,uint8_t dec)
{                          
  if(td==1 || td==2){strcat(buf,"<td>");}
  if(pol!=0){strcat(buf,"<font size=\"");concatn(buf,pol);strcat(buf,"\">");}
  strcat(buf,"<input type=\"text\" name=\"");strcat(buf,nomfonct);
  if(len<=2){strcat(buf,"\" id=\"nt");concatn(buf,len);}
  strcat(buf,"\" value=\"");
  switch (type){
    case 'b':concatn(buf,*(byte*)valfonct);break; //strcat(buf,(char*)valfonct);break;
    case 'd':concatn(buf,*(uint16_t*)valfonct);break;
    case 's':if(*(uint8_t*)valfonct==0xff){break;}concatn(buf,*(uint8_t*)valfonct);break;
    case 'i':concatns(buf,*(int*)valfonct);break;
    case 'I':concatns(buf,*(int16_t*)valfonct);break;
    case 'r':concatnf(buf,(float)(*(int16_t*)valfonct)/100);break;
    case 'l':concatns(buf,*(long*)valfonct);break;
    case 'f':concatnf(buf,*(float*)valfonct);break;
    case 'F':concatnf(buf,*(float*)valfonct,dec);break;
    case 'g':concatn(buf,*(uint32_t*)valfonct);break;    
    default:break;
  }
  int sizeHtml=1;if(len>=3){sizeHtml=2;}if(len>=6){sizeHtml=4;}if(len>=9){sizeHtml=6;}
  strcat(buf,"\" size=\"");concatn(buf,sizeHtml);strcat(buf,"\" maxlength=\"");concatn(buf,len);strcat(buf,"\" >");
  if(pol!=0){strcat(buf,"</font>");}
  if(td==1 || td==3){strcat(buf,"</td>\n");}
}
*/
void usrFormBHtml(char* buf,bool hid)
{
  usrFormBHtml(buf,nullptr,hid);
}

void usrFormBHtml(char* buf,char* jsbuf,bool hid)                     // pour mettre en tête des formulaires ("<p hidden> .... </p>")
{
  jscat(jsbuf,JSFUB);
  if(hid){strcat(buf,"<p hidden>");jscat(jsbuf,JSHIDB);}
  strcat(buf,"<input type=\"text\" name=\"");
  char* dm=buf+strlen(buf);
  strcat(buf,"user_ref_");concat1a(buf,nullptr,(char)(usernum+PMFNCHAR));
  jscat(jsbuf,dm);
  strcat(buf,"\" value=\"");concatn(buf,jsbuf,usrtime[usernum]);strcat(buf,"\">");  
  if(hid){strcat(buf,"</p>");jscat(jsbuf,JSHIDE);}
  jscat(jsbuf,JSFUE);
}

void usrFormInitBHtml(char* buf,const char* nomfonct)            // pour mettre en tête des formulaires ("<p hidden> .... </p>")
{                                                          // ajoute une fonction invisible sans valeur associée pour faire des opérations préalables quand le bouton submit
                                                           // est appuyé (genre effacement de cb) 
    strcat(buf,"<p hidden>");
      usrFormBHtml(buf,0);
      strcat(buf,"<input type=\"text\" name=\"");strcat(buf,nomfonct);strcat(buf,"\">");
    strcat(buf,"</p>");
}

void usrPeriCurB(char* buf,const char* fnct,uint8_t ninp,int len,uint8_t td)
{
  usrPeriCurB(buf,nullptr,fnct,ninp,len,td);
}

void usrPeriCurB(char* buf,char* jsbuf,const char* fnct,uint8_t ninp,int len,uint8_t td)
{                                                         // pour mettre en tête des formulaires ("<p hidden> .... </p>")
                                                          // ajoute une fonction invisible pour faire des opérations préalables quand le bouton submit
                                                          // est appuyé sa associée valeur est periCur (genre effacement de cb)
                                                          // le n° de fonction ninp permet une seule fonction d'init pour plusieurs formulaires de même structure
    strcat(buf,"<p hidden>");jscat(jsbuf,JSHIDB);
      usrFormBHtml(buf,jsbuf,0);
      char fonc[LENNOM+1];memcpy(fonc,fnct,LENNOM);fonc[LENNOM-1]=(char)(ninp+PMFNCHAR);fonc[LENNOM]='\0';
      numTf(buf,jsbuf,'i',&periCur,fonc,len,td,0); // pericur n'est pas modifiable (fixation pericur, periload, cberase)
    strcat(buf,"</p>");jscat(jsbuf,JSHIDE);
}

void selectTableBHtml(char* buf,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t td)
{            // val=table des libellés ; ft=fonction ; nbre ds table ; len step table ; sel=n°actuel ; nuv=n°param ; n°inp
  char a;
  int i,j;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);   
  ft[LENNOM-1]=(char)(ninp+PMFNCHAR);   
  
  if(td==1 || td==2){strcat(buf,"<td>");}

  strcat(buf,"<SELECT name=\"");strcat(buf,ft);strcat(buf,"\">");
  for(i=0;i<nbre;i++){
    strcat(buf,"<OPTION");if(i==sel){strcat(buf," selected");};strcat(buf,">");
    for(j=0;j<len;j++){
      a=(char)val[i*len+j];
      if(a!=' '){concat1a(buf,a);}
    }
  }

  strcat(buf,"</SELECT>");

  if(td==1 || td==3){strcat(buf,"</td>");}
  strcat(buf,"\n");
}

void textTbl(char* buf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t br,uint8_t td)
{
  textTbl(buf,nullptr,valfonct,valmin,valmax,br,td);
}

void textTbl(char* buf,char* jsbuf,int16_t* valfonct,int16_t* valmin,int16_t* valmax,uint8_t br,uint8_t td)
{
  char colour[6+1];
  memcpy(colour,"black\0",6);if(*valfonct<*valmin || *valfonct>*valmax){memcpy(colour,"red\0",4);}
  if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSCB);}
  strcat(buf,"<font color=\"");strcat(buf,colour);strcat(buf,"\"> ");
  concatnf(buf,jsbuf,((float)*valfonct)/100);  
  strcat(buf,"</font>");
  if(br==1){strcat(buf,"<br>");jscat(jsbuf,JSBR);}
  if(td==2 || td==3){strcat(buf,"</td>");jscat(jsbuf,JSCE);}
}

void concatDate(char* buf,char* periDate)
{
  char dateascii[12];
  int j;
  unpackDate(dateascii,periDate);for(j=0;j<12;j++){concat1a(buf,dateascii[j]);if(j==5){strcat(buf," ");}}strcat(buf,"<br>");
}

void bufPrintDateHeure(char* buf,char* pkdate)
{
  bufPrintDateHeure(buf,nullptr,pkdate);
}

void bufPrintDateHeure(char* buf,char* jsbuf,char* pkdate)
{
  char* dm=buf+strlen(buf);
  
  char bufdate[LNOW];ds3231.alphaNow(bufdate);packDate(pkdate,bufdate+2); // skip siècle
  for(int zz=0;zz<14;zz++){concat1a(buf,bufdate[zz]);if(zz==7){strcat(buf,"-");}}
  strcat(buf,"(");concatn(buf,bufdate[14]);strcat(buf,")");strcat(buf," GMT ");

  jscat(jsbuf,dm);
}

void setCol(char* buf,const char* textColour)
{
  strcat(buf,"<font color=\"");strcat(buf,textColour);strcat(buf,"\"> ");
}

void boutF(char* buf,const char* nomfonct,const char* valfonct,const char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter)
{
  boutF(buf,nullptr,nomfonct,valfonct,lib,td,br,sizfnt,aligncenter);
}

void boutF(char* buf,char* jsbuf,const char* nomfonct,const char* valfonct,const char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter)
/* génère user_ref_x=nnnnnnn...?ffffffffff=zzzzzz... */
{
    jscat(jsbuf,JSBFB);
    if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSTB);}
    
    char b[]={(char)(usernum+PMFNCHAR),'\0'};
    strcat(buf,"<a href=\"?user_ref_");
    strcat(buf,b);strcat(buf,"=");concatn(buf,usrtime[usernum]);
    strcat(buf,"?");
    char* dm=buf+strlen(buf);
    strcat(buf,nomfonct);strcat(buf,"=");strcat(buf,valfonct);jscat(jsbuf,dm);
    strcat(buf,"\">");
    if(aligncenter){strcat(buf,"<p align=\"center\">");jscat(jsbuf,JSAC);}
    strcat(buf,"<input type=\"button\" value=\"");strcat(buf,lib);strcat(buf,"\"");jscat(jsbuf,lib);
    if(sizfnt==7){strcat(buf," style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"");}
    if(aligncenter){strcat(buf,"></p></a>");jscat(jsbuf,JSAC);}
    else{strcat(buf,"></a>");}

    jscat(jsbuf,JSBFE);
    if(br!=0){strcat(buf,"<br>");jscat(jsbuf,JSBR);}
    if(td==1 || td==3){strcat(buf,"</td>");jscat(jsbuf,JSCE);}
}

void boutRetourB(char* buf,const char* lib,uint8_t td,uint8_t br)
{
  boutRetourB(buf,nullptr,lib,td,br);
}

void boutRetourB(char* buf,char* jsbuf,const char* lib,uint8_t td,uint8_t br)
{
    if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSCB);}
    jscat(jsbuf,JSBRB);
    strcat(buf,"<a href=\"?user_ref_");
    concat1a(buf,(char)(usernum+PMFNCHAR));strcat(buf,"=");concatn(buf,usrtime[usernum]);
    strcat(buf,"\"><input type=\"button\" text style=\"width:300px;height:60px;font-size:40px\" value=\"");
    strcat(buf,lib);jscat(jsbuf,lib);
    strcat(buf,"\"></a>");
    jscat(jsbuf,JSBRE);
    if(td==1 || td==3){strcat(buf,"</td>");jscat(jsbuf,JSCE);}
    if(br!=0){strcat(buf,"<br>");jscat(jsbuf,JSBR);}
    strcat(buf,"\n");
}

void boutMaj(char* buf,char* jsbuf,const char* lib,uint8_t td)
{
  if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSCB);}
  jscat(jsbuf,JSBMB);
  strcat(buf,"<input type=\"submit\" value=\"");
  bufcat(buf,jsbuf,lib);
  strcat(buf," \">");
  jscat(jsbuf,JSBME);
  if(td==1 || td==3){strcat(buf,"</td>");jscat(jsbuf,JSCE);}
  strcat(buf,"\n");
}

void radioTableBHtml(char* buf,byte valeur,char* nomfonct,uint8_t nbval)        // nbval boutons radio
{                                                                               // valeur = checked (0-n) 
      concatns(buf,valeur);//strcat(buf,"<br>");
      for(uint8_t j=0;j<nbval;j++){
          strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,nomfonct);
          strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+j));strcat(buf,"\"");
          if(j==valeur){strcat(buf," checked");}strcat(buf,"/>");
      }
}

void yradioTableBHtml(char* buf,byte valeur,const char* nomfonct,uint8_t nbval,bool vert,uint8_t nb,uint8_t td)                    // 1 line ; nb = page row
{                                                                                                                                  // sqbr square button
                                                                                                                                   // nbval à traiter
  if(td==1 || td==2){strcat(buf,"<td>");}  
    valeur&=0x03;                                                               
  
  
  strcat(buf,"<input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_off\" id=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+0));strcat(buf,"\"");
  if(valeur==0){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbrb");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">OFF</label>\n");
  
  if(vert){strcat(buf,"<br><br>\n");}
  
  strcat(buf," <input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_on\" id=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+1));strcat(buf,"\"");
  if(valeur==1){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbra");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">ON</label>\n");

  strcat(buf," <input type=\"radio\" name=\"");strcat(buf,nomfonct);concat1a(buf,(char)(nb+PMFNCHAR));
  strcat(buf,"\" class=\"sqbr br_for\" id=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\"");
  strcat(buf,"\" value=\"");concat1a(buf,(char)(PMFNCVAL+2));strcat(buf,"\"");
  if(valeur==2 || valeur==3){strcat(buf," checked");}strcat(buf,">");
  strcat(buf,"<label for=\"sqbrc");concat1a(buf,(char)(nb+PMFNCHAR));strcat(buf,"\">FOR</label>\n");


  if(td==1 || td==3){strcat(buf,"</td>");}      
  strcat(buf,"<br>\n");
}

void sliderBHtml(char* buf,uint8_t* val,const char* nomfonct,int nb,int sqr,uint8_t td)
{
  if(td==1 || td==2){strcat(buf,"<td>");}

  char nf[LENNOM+1];nf[LENNOM]='\0';
  memcpy(nf,nomfonct,LENNOM);if(nb>=0){nf[LENNOM-1]=(char)(nb+PMFNCHAR);}
  strcat(buf,"<label class=\"switch\"><input type=\"checkbox\" style=\"color:Khaki;\" name=\"");strcat(buf,nf);strcat(buf,"\" value=\"1\"");
  if((*val & 0x01)!=0){strcat(buf," checked");}
  strcat(buf," ><span class=\"slider ");if(sqr==0){strcat(buf," round");}
  strcat(buf,"\"></span></label>");
  
  if(td==1 || td==3){strcat(buf,"</td>\n");}
}

void htmlIntro0B(char* buf)    // suffisant pour commande péripheriques
{
  strcat(buf,"HTTP/1.1 200 OK\n");
  //cli->println("Location: http://82.64.32.56:1789/");
  //cli->println("Cache-Control: private");
  strcat(buf,"CONTENT-Type: text/html; charset=UTF-8\n");
  strcat(buf,"Connection: close\n\n");
  strcat(buf,"<!DOCTYPE HTML ><html>\n");
}

void htmlIntroB(char* buf,char* titre,EthernetClient* cli)
{

  htmlIntro0B(buf);

  strcat(buf,"<head>");
  char locbuf[10]={0};
  if(perrefr!=0){strcat(buf,"<meta HTTP-EQUIV=\"Refresh\" content=\"");sprintf(locbuf,"%d",perrefr);strcat(buf,locbuf);strcat(buf,"\">");}
  strcat(buf,"<title>");strcat(buf,titre);strcat(buf,"</title>\n");
  
          strcat(buf,"<style>");         

            strcat(buf,"table {");
              strcat(buf,"font-family: Courier, sans-serif;");
              strcat(buf,"border-collapse: collapse;");
              //cli->println("width: 100%;");
              strcat(buf,"overflow: auto;\n");
              strcat(buf,"white-space:nowrap;"); 
            strcat(buf,"}\n");

  ethWrite(cli,buf);
  
            strcat(buf,"td, th {");
              strcat(buf,"font-family: Courier, sans-serif;\n");
              strcat(buf,"border: 1px solid #dddddd;\n");
              strcat(buf,"text-align: left;\n"); 
            strcat(buf,"}\n");

            strcat(buf,"#nt1{width:10px;}\n");
            strcat(buf,"#nt2{width:18px;}\n");
            strcat(buf,"#cb1{width:10px; padding:0px; margin:0px; text-align: center};\n");
            strcat(buf,"#cb2{width:20px; text-align: center};\n");

            strcat(buf,".button {background-color: #195B6A; border: none; color: white; padding: 32px 80px:"); //16px 40px;");
            strcat(buf,"text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}\n");
            strcat(buf,".button2 {background-color: #77878A;}\n");

  ethWrite(cli,buf);
  
            /* big sliders */
            strcat(buf,".switch {position: relative;display: inline-block;width: 220px;height: 100px; margin: 16px;}\n");
            strcat(buf,".switch input {opacity: 0;width: 0;hight: 0;}\n");
            strcat(buf,".slider {position: absolute;cursor: pointer;");
            strcat(buf,"  top: 0;left: 0;right: 0;bottom: 0;background-color: #ccc;-webkit-transition: .4s;transition: .4s;}\n");
            strcat(buf,".slider:before {position: absolute;content: \"\";");
            strcat(buf,"  height: 84px;width: 84px;left: 8px;bottom: 8px;background-color: white;-webkit-transition: .4s;transition: .4s;}\n");

            strcat(buf,"input:checked + .slider {background-color: #2196F3;}\n");
            strcat(buf,"input:focus + .slider {box-shadow: 0 0 1px #2196F3;}\n");
            strcat(buf,"input:checked + .slider:before {-webkit-transform: translateX(110px);-ms-transform: translateX(55px);transform: translateX(110px);}\n");
            /* Rounded sliders */
            strcat(buf,".slider.round {border-radius: 50px;}\n");
            strcat(buf,".slider.round:before {border-radius: 50%;}\n");

  ethWrite(cli,buf);            
  
            /* pour bouton radio carrés */     
            strcat(buf,"@import url(\"https://fonts.googleapis.com/css?family=Roboto:400,400i,700\");\n");
            //strcat(buf,":root {  --txt-color: #00B7E8;}\n");
            //strcat(buf,"body { margin: 2rem;font-family: Roboto, sans-serif;}\n");
            strcat(buf,".content {display:flex;flex-wrap:wrap;gap:1rem;justify-content:flex-start;}\n");
            strcat(buf,".content > div{flex-basis:200px;border:1px solid #ccc;padding:1rem;box-shadow: 0px 1px 1px rgba(0,0,0,0.2), 0px 1px 1px rgba(0,0,0,0.2);}\n");
            strcat(buf,"h1{font-weight: normal; color: var(--txt-color);}\n");
            strcat(buf,"h2 {font-size: 1.1rem;color: var(--txt-color);font-weight: normal;text-transform: uppercase;margin:0 0 2rem;border-bottom: 1px solid #ccc;}\n");
  
  ethWrite(cli,buf);            
  
            strcat(buf,"input[type=\"radio\"].sqbr {display: none;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr + label {padding: 0.5rem 1rem;font-size: 1.50rem;line-height: 1.5;border-radius: 0.3rem;color: #fff;background-color: #6c757d;border: 1px solid transparent;transition: all 0.15s ease-in-out;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_off:hover + label { background-color: #28a745;border-color: #28a745;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_off:hover + label { background-color: #218838;border-color: #1e7e34;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_off:checked + label { background-color: #28a745;border-color: #28a745;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_on:hover + label { background-color: #338fff;border-color: #338fff;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_on:hover + label { background-color: #338fff;border-color: #bd2130;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_on:checked + label { background-color: #338fff;border-color: #338fff;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_for:hover + label { background-color: #dc3545;border-color: #dc3545;}\n");
            //strcat(buf,"input[type=\"radio\"].sqbr.br_for:hover + label { background-color: #c82333;border-color: #bd2130;}\n");
            strcat(buf,"input[type=\"radio\"].sqbr.br_for:checked + label { background-color: #dc3545;border-color: #dc3545;}\n");
  
            /* rond jaune */
            strcat(buf,"#rond_jaune {width: 40px;height: 40px;border-radius: 20px;background: yellow;}");

          strcat(buf,"</style>\n");  
  strcat(buf,"</head>\n");
  ethWrite(cli,buf);
}

void bufcat(char* buf,char* jsbuf,const char* s){strcat(buf,s);if(jsbuf!=nullptr){strcat(jsbuf,s);strcat(jsbuf,";");}}

void jscat(char* jsbuf,const char* s,bool crlf)
{
  if(jsbuf!=nullptr){strcat(jsbuf,s);if(s[0]!='~'){strcat(jsbuf,";");}if(crlf){strcat(jsbuf,"\n\r");}}
}

void jscat(char* jsbuf,const char* s){jscat(jsbuf,s,false);}

void pageHeader(char* buf)
{
  pageHeader(buf,nullptr,true);
}

void pageHeader(char* buf,char* jsbuf)
{
  pageHeader(buf,jsbuf,true);
}

void pageHeader(char* buf,bool form)
{
  pageHeader(buf,nullptr,form);
}

void pageHeader(char* buf,char* jsbuf,bool form)
{ 
  float th;                                  // pour temp DS3231
  char* dm;
  ds3231.readTemp(&th);
  
  strcat(buf,"<body>");            
  if(form){strcat(buf,"<form method=\"get\" >");}
  dm=buf+strlen(buf);
  //Serial.println("---- beg record");
  strcat(buf,VERSION);strcat(buf," ");
  #ifdef _MODE_DEVT
  bufcat(buf,jsbuf,"MODE_DEVT ");
  #endif // _MODE_DEVT
  #ifdef _MODE_DEVT2
  strcat(buf,"MODE_DEVT2 ");
  #endif // _MODE_DEVT2

  bufPrintDateHeure(buf,nullptr,pkdate);
  uint32_t bufIp=Ethernet.localIP();
  jscat(jsbuf,dm);
  strcat(buf,"<font size=\"");bufcat(buf,jsbuf,"2");strcat(buf,"\">; local IP ");
  dm=buf+strlen(buf);
  charIp((byte*)&bufIp,buf,nullptr);strcat(buf," ");
  concatnf(buf,nullptr,th);strcat(buf,"°C");
  jscat(jsbuf,dm,CRLF);
  strcat(buf,"<br>\n");
}


void checkboxTableBHtml(char* buf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib)
{
  checkboxTableBHtml(buf,nullptr,val,nomfonct,etat,td,lib);
}

void checkboxTableBHtml(char* buf,char* jsbuf,uint8_t* val,const char* nomfonct,int etat,uint8_t td,const char* lib)
{
  if(td==1 || td==2){strcat(buf,"<td>");jscat(jsbuf,JSCB);}
    jscat(jsbuf,JSDB);
    strcat(buf,"<input type=\"checkbox\" name=\"");bufcat(buf,jsbuf,nomfonct);strcat(buf,"\" id=\"cb1\" value=\"1\"");
    if((*val & 0x01)!=0){bufcat(buf,jsbuf," checked");}
    strcat(buf,">");bufcat(buf,jsbuf,lib);
  if(etat>0 && !(*val & 0x01)){etat=2;}
      switch(etat){
        case 2 :bufcat(buf,jsbuf,"___");break;
        case 1 :bufcat(buf,jsbuf,"_ON");break;
        case 0 :bufcat(buf,jsbuf,"OFF");break;
        default:break;
      }
  jscat(jsbuf,JSDE);    
  if(td==1 || td==3){strcat(buf,"</td>");jscat(jsbuf,JSCE);}
  strcat(buf,"\n");jscat(jsbuf,"",CRLF);
}

void subDSnB(char* buf,const char* fnc,uint32_t val,uint8_t num,char* lib) // checkbox transportant 1 bit 
                                                                    // num le numéro du bit dans le mot
                                                                    // le caractère LENNOM-1 est le numéro du bit(+PMFNCHAR) dans periDetServ 
{                                                                   // le numéro est codé 0 à 15 + 0x40 et 16->n + 0x50 !!!! (évite les car [\]^ )
  char fonc[LENNOM+1];
  memcpy(fonc,fnc,LENNOM+1);
  uint8_t val0=(val>>num)&0x01;
  if(num>=16){num+=16;}
  fonc[LENNOM-1]=(char)(PMFNCHAR+num);
  checkboxTableBHtml(buf,&val0,fonc,-1,0,lib);
}

void setColourB(char* buf,const char* textColour)
{
  memcpy(colour,textColour,LENCOLOUR);strcat(buf,"<font color=\"");strcat(buf,colour);strcat(buf,"\"> ");
}

void cliPrintMac(EthernetClient* cli, byte* mac)
{
  char macBuff[18];
  unpackMac(macBuff,mac);
  cli->print(macBuff);
}

void bufPrintPeriDate(char* buf,char* periDate)
{
  char dateascii[12];
  int j;
  unpackDate(dateascii,periDate);for(j=0;j<12;j++){concat1a(buf,dateascii[j]);if(j==5){strcat(buf," ");}}strcat(buf,"<br>\n");
}

void trailingSpaces(char* data,uint16_t len)
{
  for(int i=len-1;i>=0;i--){if(data[i]==' ' || data[i]=='\0'){data[i]='\0';}else break;} // erase trailing spaces
}

void alphaTfr(char* recep,uint16_t lenRecep,char* emet,uint16_t lenEmet)
{
  memset(recep,0x00,lenRecep);
  if(lenEmet>=lenRecep-1){lenEmet=lenRecep-1;}
  memcpy(recep,emet,lenEmet);
  trailingSpaces(recep,lenRecep);
}
