#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet2.h>
#include <SD.h>
#include "ds3231.h"
#include <shutil2.h>
#include <shconst2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "pageshtml.h"

extern Ds3231 ds3231;

extern char*     chexa;

extern uint8_t   remote_IP[4],remote_IP_cur[4];

extern char      periRec[PERIRECLEN];        // 1er buffer de l'enregistrement de périphérique
  
extern uint16_t  periCur;                    // Numéro du périphérique courant

extern uint16_t* periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern long*     periPerRefr;                  // ptr ds buffer : période maximale accés au serveur
extern uint16_t* periPerTemp;                  // ptr ds buffer : période de lecture tempèrature
extern float*    periPitch;                    // ptr ds buffer : variation minimale de température pour datasave
extern float*    periLastVal;                  // ptr ds buffer : dernière valeur de température  
extern float*    periAlim;                     // ptr ds buffer : dernière tension d'alimentation
extern char*     periLastDateIn;               // ptr ds buffer : date/heure de dernière réception
extern char*     periLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
extern char*     periLastDateErr;              // ptr ds buffer : date/heure de derniere anomalie com
extern int8_t*   periErr;                      // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
extern char*     periNamer;                    // ptr ds buffer : description périphérique
extern char*     periVers;                     // ptr ds buffer : version logiciel du périphérique
extern char*     periModel;                    // ptr ds buffer : model du périphérique
extern byte*     periMacr;                     // ptr ds buffer : mac address 
extern byte*     periIpAddr;                   // ptr ds buffer : Ip address
extern byte*     periSwNb;                     // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*     periSwVal;                    // ptr ds buffer : état/cde des inter  
extern byte*     periSwMode;                   // ptr ds buffer : Mode fonctionnement inters (4 bytes par switch)           
extern uint32_t* periSwPulseOne;               // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t* periSwPulseTwo;               // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t* periSwPulseCurrOne;           // ptr ds buffer : temps courant pulses ON
extern uint32_t* periSwPulseCurrTwo;           // ptr ds buffer : temps courant pulses OFF
extern byte*     periSwPulseCtl;               // ptr ds buffer : mode pulses 
extern byte*     periSwPulseSta;               // ptr ds buffer : état clock pulses
extern uint8_t*  periSondeNb;                  // ptr ds buffer : nbre sonde
extern boolean*  periProg;                     // ptr ds buffer : flag "programmable" 
extern byte*     periDetNb;                    // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*     periDetVal;                   // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern float*    periThOffset;                 // ptr ds buffer : offset correctif sur mesure température
extern float*    periThmin;                    // ptr ds buffer : alarme mini th
extern float*    periThmax;                    // ptr ds buffer : alarme maxi th
extern float*    periVmin;                     // ptr ds buffer : alarme mini volts
extern float*    periVmax;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

extern uint16_t  perrefr;

extern char*     usrnames; 
extern char*     usrpass; 
extern unsigned long* usrtime;  

extern int       usernum;
//extern int       chge_pwd;

extern byte mask[];

#define LENCOLOUR 8
  char colour[LENCOLOUR+1];


void htmlIntro0(EthernetClient* cli)    // suffisant pour commande péripheriques
{
  cli->println("HTTP/1.1 200 OK");
  //cli->println("Location: http://82.64.32.56:1789/");
  //cli->println("Cache-Control: private");
  cli->println("CONTENT-Type: text/html; charset=UTF-8");
  cli->println("Connection: close\n");
  cli->println("<!DOCTYPE HTML ><html>");
}


void htmlIntro(char* titre,EthernetClient* cli)
{
  htmlIntro0(cli);

  cli->println("<head>");
  char buf[10]={0};
  if(perrefr!=0){cli->print("<meta HTTP-EQUIV=\"Refresh\" content=\"");sprintf(buf,"%d",perrefr);cli->print(buf);cli->print("\">");}
  cli->print("\<title>");cli->print(titre);cli->println("</title>");
  
          cli->println("<style>");

            cli->println("table {");
              cli->println("font-family: Courier, sans-serif;");
              cli->println("border-collapse: collapse;");
              //cli->println("width: 100%;");
              cli->println("overflow: auto;");
              cli->println("white-space:nowrap;"); 
            cli->println("}");

            cli->println("td, th {");
              cli->println("border: 1px solid #dddddd;");
              cli->println("text-align: left;"); 
            cli->println("}");

            cli->println("#nt1{width:10px;}");
            cli->println("#nt2{width:18px;}");
            cli->println("#cb1{width:10px; padding:0px; margin:0px; text-align: center};");
            cli->println("#cb2{width:20px; text-align: center};");

            cli->print(".button {background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            cli->println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            cli->println(".button2 {background-color: #77878A;}");

            /* big sliders */
            cli->println(".switch {position: relative;display: inline-block;width: 220px;height: 100px; margin: 16px;}");
            cli->println(".switch input {opacity: 0;width: 0;hight: 0;}");
            cli->print(".slider {position: absolute;cursor: pointer;");
            cli->println("  top: 0;left: 0;right: 0;bottom: 0;background-color: #ccc;-webkit-transition: .4s;transition: .4s;}");
            cli->print(".slider:before {position: absolute;content: \"\";");
            cli->println("  height: 84px;width: 84px;left: 8px;bottom: 8px;background-color: white;-webkit-transition: .4s;transition: .4s;}");
            cli->println("input:checked + .slider {background-color: #2196F3;}");
            cli->println("input:focus + .slider {box-shadow: 0 0 1px #2196F3;}");
            cli->println("input:checked + .slider:before {-webkit-transform: translateX(110px);-ms-transform: translateX(55px);transform: translateX(110px);}");
            /* Rounded sliders */
            cli->println(".slider.round {border-radius: 50px;}");
            cli->println(".slider.round:before {border-radius: 50%;}");

          cli->println("</style>");
  
  cli->println("</head>");
}


void setColour(EthernetClient* cli,char* textColour)
{
  memcpy(colour,textColour,LENCOLOUR);cli->print("<font color=\"");cli->print(colour);cli->print("\"> ");
}

void cliPrintMac(EthernetClient* cli, byte* mac)
{
  char macBuff[18];
  unpackMac(macBuff,mac);
  cli->print(macBuff);
}

void cliPrintDateHeure(EthernetClient* cli,char* pkdate)
{
  char bufdate[LNOW];ds3231.alphaNow(bufdate);packDate(pkdate,bufdate+2); // skip siècle
  for(int zz=0;zz<14;zz++){cli->print(bufdate[zz]);if(zz==7){cli->print("-");}}
  cli->print(" GMT ");
}

void boutonHtml(EthernetClient* cli,byte* valfonct,char* nomfonct,uint8_t sw,uint8_t td)      
{
  char tonoff[]={'O','F','F','\0'};

  nomfonct[LENNOM-2]=(char)(PMFNCHAR+sw);
  nomfonct[LENNOM-1]='1';

  if(td==1 || td==2){cli->print("<td>");}
  if ((*valfonct>>(sw*2))&0x01==0) {nomfonct[LENNOM-1]='0';memcpy(tonoff,"ON \0",4);}
  cli->print("<a href=\"");cli->print(nomfonct);cli->print("\"><button class=\"button\">");cli->print(tonoff);cli->println("</button></a>");
//  cli->print("<input type=\"button\" name=\"");cli->print(nomfonct);cli->print("\" <button class=\"button\">");cli->print(tonoff);cli->println("</button></a>");
  if(td==2 || td==3){cli->println("</td>");}
//              cli->println("<p><a href=\"/B/off\"><button class=\"button button2\">OFF</button></a></p>");

}

void numTableHtml(EthernetClient* cli,char type,void* valfonct,char* nomfonct,int len,uint8_t td,int pol)
{                          
  if(td==1 || td==2){cli->print("<td>");}
  if(pol!=0){cli->print("<font size=\"");cli->print(pol);cli->println("\">");}
  cli->print("<input type=\"text\" name=\"");cli->print(nomfonct);
  if(len<=2){cli->print("\" id=\"nt");cli->print(len);}
  cli->print("\" value=\"");
  switch (type){
    case 'b':cli->print(*(byte*)valfonct);break;
    case 'd':cli->print(*(uint16_t*)valfonct);break;
    case 'i':cli->print(*(int*)valfonct);break;
    case 'l':cli->print(*(long*)valfonct);break;
    case 'f':cli->print(*(float*)valfonct);break;
    case 'g':cli->print(*(uint32_t*)valfonct);break;    
    default:break;
  }
  int sizeHtml=1;if(len>=3){sizeHtml=2;}if(len>=6){sizeHtml=4;}if(len>=9){sizeHtml=6;}
  cli->print("\" size=\"");cli->print(sizeHtml);cli->print("\" maxlength=\"");cli->print(len);cli->print("\" >");
  if(pol!=0){cli->print("</font>");}
  if(td==1 || td==3){cli->println("</td>");}
}

void textTableHtml(EthernetClient* cli,char type,float* valfonct,float* valmin,float* valmax,uint8_t br,uint8_t td)
{
  memcpy(colour,"black\0",6);if(*valfonct<*valmin || *valfonct>*valmax){memcpy(colour,"red\0",4);}
  if(td==1 || td==2){cli->print("<td>");}
    cli->print("<font color=\"");cli->print(colour);cli->print("\"> ");
/*    switch (type){
      case 'b':cli->print(*(byte*)valfonct);break;
      case 'd':cli->print(*(uint16_t*)valfonct);break;
      case 'i':cli->print(*(int*)valfonct);break;
      case 'l':cli->print(*(long*)valfonct);break;
      case 'f':cli->print(*(float*)valfonct);break;
      case 'g':cli->print(*(uint32_t*)valfonct);break;    
      default:break;}
*/
  cli->print(*valfonct);  
  cli->print("</font>");
  if(br==1){cli->print("<br>");}
  if(td==2 || td==3){cli->println("</td>");}
}

void usrFormHtml(EthernetClient* cli,bool hid)                     // pour mettre en tête des formulaires ("<p hidden> .... </p>")
{
  if(hid){cli->print("<p hidden>");}
  cli->print("<input type=\"text\" name=\"user_ref_");cli->print((char)(usernum+PMFNCHAR));
  cli->print("\" value=\"");cli->print(usrtime[usernum]);cli->print("\">");  
  if(hid){cli->println("</p>");}
}

void usrFormInitHtml(EthernetClient* cli,char* nomfonct,bool hid)  // pour mettre en tête des formulaires ("<p hidden> .... </p>")
{
    cli->print("<p hidden>");
      usrFormHtml(cli,0);
//      char fonc[LENNOM+1];memcpy(fonc,fnct,LENNOM);fonc[LENNOM]='\0';   //fonc[LENNOM-1]=(char)(ninp+PMFNCHAR);
      cli->print("<input type=\"text\" name=\"");cli->print(nomfonct);cli->print("\">");
    cli->println("</p>");
}

void usrPeriCur(EthernetClient* cli,char* fnct,uint8_t ninp,int len,uint8_t td)
{                                                                  // pour mettre en tête des formulaires ("<p hidden> .... </p>")
    cli->print("<p hidden>");
      usrFormHtml(cli,0);
      char fonc[LENNOM+1];memcpy(fonc,fnct,LENNOM);fonc[LENNOM-1]=(char)(ninp+PMFNCHAR);fonc[LENNOM]='\0';
      numTableHtml(cli,'i',&periCur,fonc,len,td,0); // pericur n'est pas modifiable (fixation pericur, periload, cberase)
    cli->println("</p>");
}


void boutRetour(EthernetClient* cli,char* lib,uint8_t td,uint8_t br)
{
    if(td==1 || td==2){cli->print("<td>");}
    cli->print("<a href=\"?user_ref_");cli->print((char)(usernum+PMFNCHAR));cli->print("=");cli->print(usrtime[usernum]);
    cli->print("\"><input type=\"button\" value=\"");cli->print(lib);cli->print("\"></a>");
    if(br!=0){cli->print("<br>");}
    if(td==1 || td==3){cli->print("</td>");}
    cli->println();
}

void boutFonction(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br,uint8_t sizfnt,bool aligncenter)
/* génère user_ref_x=nnnnnnn...?ffffffffff=zzzzzz... */
{
    if(td==1 || td==2){cli->print("<td>");}

    cli->print("<a href=\"?user_ref_");cli->print((char)(usernum+PMFNCHAR));cli->print("=");cli->print(usrtime[usernum]);
    cli->print("?");cli->print(nomfonct);cli->print("=");cli->print(valfonct);
    cli->print("\">");
    if(aligncenter){cli->print("<p align=\"center\">");}
    cli->print("<input type=\"button\" value=\"");cli->print(lib);cli->print("\"");
    if(sizfnt==7){cli->print(" style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"");}
    if(aligncenter){cli->println("></p></a>");}
    else{cli->print("></a>");}

    if(br!=0){cli->print("<br>");}
    if(td==1 || td==3){cli->print("</td>");}
    cli->println();
}

void bouTableHtml(EthernetClient* cli,char* nomfonct,char* valfonct,char* lib,uint8_t td,uint8_t br)
{
    if(td==1 || td==2){cli->print("<td>");}

    cli->print("<a href=\"?");
    cli->print(nomfonct);cli->print("=");cli->print(valfonct);
    cli->print("\"><input type=\"button\" value=\"");
    cli->print(lib);
    cli->print("\"></a>");
    
/*    cli->print("<form><p hidden><input type=\"text\" name=\"");
    cli->print(nomfonct);
    cli->print("\" value=\"");
    cli->print(valfonct);
    cli->print("\" ></p><input type=\"submit\" value=\"");
    cli->print(lib);
    cli->print("\">");
    if(br!=0){cli->print("<br>");}
    cli->print("</form>");
*/

    if(br!=0){cli->print("<br>");}
    if(td==1 || td==3){cli->print("</td>");}
    cli->println();
}

void lnkTableHtml(EthernetClient* cli,char* nomfonct,char* lib)
{
  cli->print("<a href=?");cli->print(nomfonct);cli->print(": target=_self>");
  cli->print(lib);cli->println("</a>");
}

void xradioTableHtml(EthernetClient* cli,byte valeur,char* nomfonct,byte nbval,int nbli,byte type)
{
    for(int i=0;i<nbli;i++){
      char oi[]="OI";
      byte b,a=valeur; a=a >> i*2 ;b=a&0x01;a&=0x02;a=a>>1;          // mode periSwVal        a bit poids fort commande ; b bit poids faible état
      //Serial.print("swVal=");Serial.println(*periSwVal,HEX);
      for(int j=0;j<nbval;j++){
        if(type&0x02!=0){
          cli->print("<input type=\"radio\" name=\"");cli->print(nomfonct);cli->print((char)(i+48));cli->print("\" value=\"");cli->print((char)(PMFNCVAL+j));cli->print("\"");
          if(a==j){cli->print(" checked");}cli->print("/>");
        }
      }
      if(type&0x01!=0){cli->print(" ");cli->print(oi[b]);}
      cli->println("<br>");
    }
}

void selectTableHtml(EthernetClient* cli,char* val,char* ft,int nbre,int len,int sel,uint8_t nuv,uint8_t ninp,uint8_t td)
{            // val=table des libellés ; ft=fonction ; nbre ds table ; len step table ; sel=n°actuel ; nuv=n°param ; n°inp
  char a;
  int i,j;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);   
  ft[LENNOM-1]=(char)(ninp+PMFNCHAR);   
  
  if(td==1 || td==2){cli->print("<td>");}

  cli->print("<SELECT name=\"");cli->print(ft);cli->print("\">");
  for(i=0;i<nbre;i++){
    cli->print("<OPTION");if(i==sel){cli->print(" selected");};cli->print(">");
    for(j=0;j<len;j++){
      a=(char)val[i*len+j];
      if(a!=' '){cli->print(a);}
    }
  }
  cli->print("</SELECT>");

  if(td==1 || td==3){cli->print("</td>");}
  cli->println();
}

void checkboxTableHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int etat,uint8_t td,char* lib)
{
  if(td==1 || td==2){cli->print("<td>");}
  cli->print("<input type=\"checkbox\" name=\"");cli->print(nomfonct);cli->print("\" id=\"cb1\" value=\"1\"");
  if((*val & 0x01)!=0){cli->print(" checked");}
  cli->print(">");cli->print(lib);
  //cli->print("<label for=\"cb1\">");cli->print(lib);cli->print("</label>");
  if(etat>=0 && !(*val & 0x01)){etat=2;}
      switch(etat){
        case 2 :cli->print("___");break;
        case 1 :cli->print("_ON");break;
        case 0 :cli->print("OFF");break;
        default:break;
      }
  if(td==1 || td==3){cli->print("</td>");}
  cli->println();
}

void subDSn(EthernetClient* cli,char* fnc,uint32_t val,uint8_t num,char* lib) // checkbox transportant 1 bit 
                                                                    // num le numéro du bit dans le mot
                                                                    // le caractère LENNOM-1 est le numéro du bit(+PMFNCHAR) dans periDetServ 
{                                                                   // le numéro est codé 0 à 15 + 0x40 et 16->n + 0x50 !!!! (évite les car [\]^ )
  char fonc[LENNOM+1];
  memcpy(fonc,fnc,LENNOM+1);
  uint8_t val0=(val>>num)&0x01;
  if(num>=16){num+=16;}
  fonc[LENNOM-1]=(char)(PMFNCHAR+num);
  checkboxTableHtml(cli,&val0,fonc,-1,0,lib);
}

void sliderHtml(EthernetClient* cli,uint8_t* val,char* nomfonct,int nb,int sqr,uint8_t td)
{
  if(td==1 || td==2){cli->print("<td>");}

  char nf[LENNOM+1];nf[LENNOM]='\0';
  memcpy(nf,nomfonct,LENNOM);if(nb>=0){nf[LENNOM-1]=(char)(nb+PMFNCHAR);}
  cli->print("<label class=\"switch\"><input type=\"checkbox\" style=\"color:Khaki;\" name=\"");cli->print(nf);cli->print("\" value=\"1\"");
  if((*val & 0x01)!=0){cli->print(" checked");}
  cli->print(" ><span class=\"slider ");if(sqr==0){cli->print(" round");}
  cli->println("\"></span></label>");
  
  if(td==1 || td==3){cli->print("</td>");}
  cli->println();
}

void printPeriDate(EthernetClient* cli,char* periDate)
{
  char dateascii[12];
  int j;
  unpackDate(dateascii,periDate);for(j=0;j<12;j++){cli->print(dateascii[j]);if(j==5){cli->print(" ");}}cli->println("<br>");
}
