#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>
#include <SD.h>
#include <shutil2.h>
#include <shconst2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "utilhtml.h"
#include "pageshtml.h"

#ifndef WEMOS
//  #include <avr/wdt.h>  //biblio watchdog
#endif ndef WEMOS

extern File      fhisto;      // fichier histo sd card
extern long      sdpos;
extern char*     nomserver;
extern uint32_t  memDetServ;  // image mémoire NBDSRV détecteurs (8)
extern uint16_t  perrefr;

extern char*     userpass;            // mot de passe browser
extern char*     modpass;             // mot de passe modif
extern char*     peripass;            // mot de passe périphériques
extern char*     usrnames;            // usernames
extern char*     usrpass;             // userpass
extern long*     usrtime;
extern int       usernum;
extern char*     thermonames;
extern int16_t*  thermoperis;
extern uint16_t* toPassword;


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
extern uint16_t* periPort;                     // ptr ds buffer : port periph server
extern byte*     periSwNb;                     // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*     periSwVal;                    // ptr ds buffer : état/cde des inter  
extern byte*     periSwInput;                   // ptr ds buffer : Mode fonctionnement inters (4 bytes par switch)           
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


extern int  chge_pwd; //=FAUX;

extern byte mask[];



void subModePulseTime(EthernetClient* cli,uint8_t sw,uint32_t* pulse,uint32_t* dur,char* fonc1,char* fonc2,char onetwo)
{

  uint8_t pbit=PMTTE_VB;if(onetwo=='O'){pbit=PMTOE_VB;} pbit+=PCTLBIT*sw;
  uint8_t val=(((*(uint16_t*)periSwPulseCtl)>>pbit)&0x01)+PMFNCVAL;                                        
  cli->print("<font size=\"2\">");
  fonc1[LENNOM-1]=onetwo;
  //Serial.print(sw);Serial.print(" OT=");Serial.print(onetwo);Serial.print(" fonc1=");Serial.print(fonc1);Serial.print(" val=");Serial.println(val);
  checkboxTableHtml(cli,&val,fonc1,-1,0);                       // bit enable pulse
Serial.println(fonc2);
  if(*(pulse+sw)<0){*(pulse+sw)=0;}
  numTableHtml(cli,'l',(pulse+sw),fonc2,8,0,2);                 // durée pulse   
  char a[8];sprintf(a,"%06d",*(dur+sw));a[6]='\0';              // valeur courante
  cli->print("<br>(");cli->print(a);cli->println(")</font>");
}

void swinpfnc(EthernetClient* cli,uint8_t sw,uint8_t nuinp,uint16_t val,char type,uint8_t lmax,char* ft,uint8_t nuv)           // type='c' checkbox ; 'n' num / ft fonct transport / nuv num var
{                             // type input, num det, valeur, enable, action, 4*2 modes => 2 fonctions de transport avec 3 bits n°variable, 2 bits sw, 5 bits n°input                                                                           
  uint8_t vv=0;
  //char fnt[LENNOM+1];memcpy(fnt,ft,LENNOM+1);
  ft[LENNOM-2]=(char)(nuv+(sw<<3)+PMFNCHAR);
  ft[LENNOM-1]=(char)(nuinp+PMFNCVAL);
  
  switch (type){
    case 'c':if(val!=0){vv=1;};checkboxTableHtml(cli,&vv,ft,-1,0);break;
    case 'n':numTableHtml(cli,'d',&val,ft,lmax,0,2);break;
    default: break;
  }
}

void SwCtlTableHtml(EthernetClient* cli,int nbsw,int nbtypes)
{
  Serial.print("saisie switchs");Serial.print(" -- periCur=");Serial.println(periCur);
  htmlIntro(nomserver,cli);

  cli->println("<body>");            

  cli->println("<form method=\"get\" >");
    cli->print(VERSION);cli->println("  ");
    cli->print(periCur);cli->print("-");cli->print(periNamer);cli->println("<br>");

    cli->print("<p hidden>");
      usrFormHtml(cli,0);
      numTableHtml(cli,'i',&periCur,"peri_t_sw_",2,0,0); // pericur n'est pas modifiable (fixation pericur, periload, cberase)
    cli->println("</p>");

    boutRetour(cli,"retour",0,0);  
    cli->println("<input type=\"submit\" value=\"MàJ\">");
    
    cli->print("<br> détecteurs serveur ");
    char hl[]={"LH"};
    
    for(int k=NBDSRV-1;k>=0;k--){cli->print(hl[(memDetServ>>k)&0x01]);cli->print(" ");}
    cli->println("<br>");

    cli->println("<table>Pulses");                  // pulses
      cli->println("<tr><th></th><th>time One<br>time Two</th><th>f<br>r</th>");

      char pfonc[]="peri_pto__\0";            // transporte la valeur pulse time One
      char qfonc[]="peri_ptt__\0";            // transporte la valeur pulse time Two
      char rfonc[]="peri_otf__\0";            // transporte les bits freerun et enable pulse de periPulseMode (LENNOM-1= ,'F','O','T')

      cli->println("<tr>");

      for(int pu=0;pu<NBPULSE;pu++){          // boucle des pulses

        cli->print("<td>");cli->print(pu);cli->print("</td>");
        
        pfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        qfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        rfonc[LENNOM-2]=(char)(pu+PMFNCHAR);        
        
        cli->print("<td>");
        subModePulseTime(cli,pu,periSwPulseOne,periSwPulseCurrOne,rfonc,pfonc,'O');          // bit et valeur pulse one
        cli->print("<br>");
        subModePulseTime(cli,pu,periSwPulseTwo,periSwPulseCurrTwo,rfonc,qfonc,'T');          // bit et valeur pulse two
      
        cli->print("</td><td>");
        uint8_t val=(*(uint16_t*)periSwPulseCtl>>(PCTLBIT*pu+PMFRO_VB))&0x01;rfonc[LENNOM-1]='F';   // bit freerun
        checkboxTableHtml(cli,&val,rfonc,-1,0);                  
        cli->print("<br>");cli->print((char)periSwPulseSta[pu]);cli->print("</td>");         // staPulse 

      } // pulse suivant
  cli->print("</tr></table>");

    cli->println("<table>Règles");
      cli->println("<th></th><th>e.t_num a l<br>n.y_det c h O I</th></tr>");

      char xfonc1[]="swinp1____\0";
      char xfonc2[]="swinp2____\0";

      // offset dans periSwInput
      uint16_t offsetPeri=(periCur-1)*MAXSW*SWINPLEN*NBSWINPUT;
      uint16_t offsetSw=0;
      uint16_t offsetInp=0;
   
      for(int ns=0;ns<nbsw;ns++){              // ns n° de switch

        cli->print("<tr><td>");cli->print(ns);cli->print("</td>");
 
        offsetInp=0;
        cli->print("<td>");    
        for(int ninp=0;ninp<NBSWINPUT;ninp++){

            uint8_t vv;
            byte binp[SWINPLEN];memcpy(binp,periSwInput+offsetPeri+offsetSw+offsetInp,SWINPLEN);
            offsetInp+=SWINPLEN;
           
            vv=(binp[2]  & SWINPEN_VB);swinpfnc(cli,ns,ninp,vv,'c',1,xfonc1,1);                           // bit enable
            vv=(binp[0]  & (SWINPNTMS_VB | SWINPNTLS_VB));swinpfnc(cli,ns,ninp,vv,'n',1,xfonc1,2);        // type  
            vv=(binp[0]>>SWINPNVLS_PB);swinpfnc(cli,ns,ninp,vv,'n',2,xfonc1,3);                           // num detec
            cli->print(" ");
            vv=(binp[2]&SWINPACT_MS)>>SWINPACTLS_PB;swinpfnc(cli,ns,ninp,vv,'n',1,xfonc1,4);              // action
            cli->println();

            for(int mode=7;mode>=0;mode--){                                                               // 8 bits
                cli->print(" ");             
                vv=(binp[1]>>(SWINPRULESLS_PB+mode))&0x01;swinpfnc(cli,ns,ninp,vv,'c',1,xfonc2,mode);

            } // mode suivant    
            cli->print("<br>");
             
        } // input suivant
        cli->print("</td>"); 
        cli->print("</tr>");
        offsetSw+=SWINPLEN*NBSWINPUT; 

      } // switch suivant
  cli->print("</table></form></body></html>");
}

void periTableHtml(EthernetClient* cli)
{
  int i,j;
  int savePeriCur=periCur;   // save periCur et restore à la fin de periTable

Serial.print("début péritable ; remote_IP ");serialPrintIp(remote_IP_cur);Serial.println();

  htmlIntro(nomserver,cli);

  
  char bufdate[LNOW];alphaNow(bufdate);
  char pkdate[7];packDate(pkdate,bufdate+2); // skip siècle
  float th;                                  // pour temp DS3231
  readDS3231temp(&th);

        cli->println("<body>");
        cli->println("<form method=\"GET \">");

          cli->print(VERSION);
          #ifdef _MODE_DEVT
            cli->print(" _MODE_DEVT ");
          #endif _MODE_DEVT
          for(int zz=0;zz<14;zz++){cli->print(bufdate[zz]);if(zz==7){cli->print("-");}}
          cli->println(" GMT ; local IP ");cli->print(Ethernet.localIP());cli->println(" ");
          cli->print(th);cli->println("°C<br>");

          usrFormHtml(cli,1);

          boutRetour(cli,"refresh",0,0);
          numTableHtml(cli,'d',&perrefr,"per_refr__",4,0,0);cli->println("<input type=\"submit\" value=\"ok\">");          
          boutFonction(cli,"reset_____","","reset",0,0,0,0);
          boutFonction(cli,"cfgserv___","","config",0,0,0,0);
          boutFonction(cli,"remote____","","remote_cfg",0,0,0,0);
          boutFonction(cli,"remotehtml","","remotehtml",0,0,0,0);
          boutFonction(cli,"thermohtml","","thermohtml",0,0,0,0);
          boutFonction(cli,"timershtml","","timershtml",0,0,0,0);          

          cli->print("(");long sdsiz=fhisto.size();cli->print(sdsiz);cli->println(") ");
          numTableHtml(cli,'i',(uint32_t*)&sdpos,"sd_pos____",9,0,0);cli->println("<input type=\"submit\" value=\"ok\"> ");
          boutFonction(cli,"dump_sd___","","dump SD",0,0,0,0);
          
          cliPrintDetServ(cli,&memDetServ);
        
        cli->println("</form>");

          cli->println("<table>");
              cli->println("<tr>");
                cli->println("<th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>_O_I___</th><th></th><th>mac_addr<br>ip_addr</th><th>version DS18x<br>last out<br>last in</th><th></th>"); //<th>det<br>srv<br>en</th>"); //<th>time One<br>time Two</th><th>f<br>r</th><th>e.l _f_H.a<br>n.x _t_L.c</th><th>___det__srv._pul<br></th>");
              cli->println("</tr>");
 
              for(i=1;i<=NBPERIF;i++){
                // !!!!!!!!!!!!!!!!!! pericur doit étre le premier de la liste !!!!!!!!!!!!!!!!!!!!!!!!!
                // pour permettre periLoad préalablement aux mises à jour des données quand le navigateur 
                // envoie une commande GET/POST 
                // et pour assurer l'effacement des bits de checkbox : le navigateur ne renvoie que ceux "checkés"
                periInitVar();periLoad(i);periCur=i;
                if(*periSwNb>MAXSW){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}

                cli->println("<tr>");
                  cli->println("<form method=\"GET \">");
                      
                      cli->print("<td>");
                      cli->println(periCur);
                      cli->print("<p hidden>");
                        usrFormHtml(cli,0);
                        numTableHtml(cli,'i',&periCur,"peri_cur__",2,3,0);
                      cli->println("</p>");
                      cli->print("<td><input type=\"text\" name=\"peri_nom__\" value=\"");
                        cli->print(periNamer);cli->print("\" size=\"12\" maxlength=\"");cli->print(PERINAMLEN-1);cli->println("\" ></td>");
                      textTableHtml(cli,'f',periLastVal,periThmin,periThmax,1,1);
                      //numTableHtml(cli,'f',periThmin,"peri_thmin",5,0,0);cli->println("<br>");
                      //numTableHtml(cli,'f',periThmax,"peri_thmax",5,3,0);
                      cli->print(*periThmin);cli->println("<br>");
                      cli->print(*periThmax);
                      textTableHtml(cli,'f',periAlim,periVmin,periVmax,1,1);
                      numTableHtml(cli,'f',periVmin,"peri_vmin_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'f',periVmax,"peri_vmax_",5,3,0);
                      numTableHtml(cli,'d',(uint32_t*)periPerTemp,"peri_rtemp",5,2,0);cli->println("<br>");
                      numTableHtml(cli,'f',periPitch,"peri_pitch",5,0,0);cli->print("<br>");
                      numTableHtml(cli,'f',periThOffset,"peri_tofs_",5,3,0);
                      numTableHtml(cli,'l',(uint32_t*)periPerRefr,"peri_refr_",5,2,0);cli->println("<br>");
                      checkboxTableHtml(cli,(uint8_t*)periProg,"peri_prog_",-1,3);
                      numTableHtml(cli,'b',periSwNb,"peri_intnb",1,2,0);cli->println("<br>");
                      numTableHtml(cli,'b',periDetNb,"peri_detnb",1,3,0);
                      cli->println("<td>");
                      xradioTableHtml(cli,*periSwVal,"peri_vsw_\0",2,*periSwNb,3);
                      
                      cli->print("<td>");
                      for(uint8_t k=0;k<*periDetNb;k++){char oi[2]={'O','I'};cli->print(oi[(*periDetVal>>(k*2))&DETBITLH_VB]);if(k<*periDetNb-1){cli->print("<br>");}}
                      cli->println("</td>");
                      cli->print("<td><input type=\"text\" name=\"peri_mac__\" value=\"");for(int k=0;k<6;k++){cli->print(chexa[periMacr[k]/16]);cli->print(chexa[periMacr[k]%16]);}
                        cli->println("\" size=\"11\" maxlength=\"12\" ><br>");
                      if(*periProg!=0){cli->print("port=");numTableHtml(cli,'d',periPort,"peri_port_",4,0,0);}cli->println("<br>");
                      cli->print("<font size=\"2\">");for(j=0;j<4;j++){cli->print(periIpAddr[j]);if(j<3){cli->print(".");}}cli->println("</font></td>");
                      cli->print("<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){cli->print(periVers[j]);}cli->println("<br>");
                      
                      //char dateascii[12];
                      char colourbr[6];
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
                      //unpackDate(dateascii,periLastDateOut);for(j=0;j<12;j++){cli->print(dateascii[j]);if(j==5){cli->print(" ");}}cli->println("<br>");
                      printPeriDate(cli,periLastDateOut);
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
                      //unpackDate(dateascii,periLastDateIn);for(j=0;j<12;j++){cli->print(dateascii[j]);if(j==5){cli->print(" ");}}cli->println("<br>");
                      printPeriDate(cli,periLastDateIn);
                      setColour(cli,"black");
                      cli->println("</font></td>");
                      
                      cli->println("<td><input type=\"submit\" value=\"   MàJ   \"><br>");
 
                      if(*periSwNb!=0){
                        char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
                        boutFonction(cli,swf,"","Switchs",3,0,0,0);}

                  cli->print("</form>");
                cli->println("</tr>");
              }
          cli->println("</table>");
        cli->println("</body></html>");
periCur=savePeriCur;if(periCur!=0){periLoad(periCur);}
Serial.println("fin péritable");
}


