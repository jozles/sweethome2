#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>
#include <SD.h>
#include "ds3231.h"
#include <shutil2.h>
#include <shconst2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "utilhtml.h"
#include "pageshtml.h"


extern Ds3231 ds3231;

extern File      fhisto;      // fichier histo sd card
extern long      sdpos;
extern char*     nomserver;
extern uint32_t  memDetServ;  // image mémoire NBDSRV détecteurs
extern char      libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t  perrefr;

extern char*     userpass;            // mot de passe browser
extern char*     modpass;             // mot de passe modif
extern char*     peripass;            // mot de passe périphériques
extern char*     usrnames;            // usernames
extern char*     usrpass;             // userpass
extern unsigned long*     usrtime;
extern int       usernum;
extern char*     thermonames;
extern int16_t*  thermoperis;
extern uint16_t* toPassword;

extern unsigned long      cxtime;
extern char*     chexa;

extern uint8_t   remote_IP[4],remote_IP_cur[4];

extern char      periRec[PERIRECLEN];        // 1er buffer de l'enregistrement de périphérique
  
extern uint16_t  periCur;                    // Numéro du périphérique courant

extern uint16_t* periNum;                      // ptr ds buffer : Numéro du périphérique courant
extern long*     periPerRefr;                  // ptr ds buffer : période maximale accés au serveur
extern uint16_t* periPerTemp;                  // ptr ds buffer : période de lecture tempèrature
extern int16_t*  periPitch_;                    // ptr ds buffer : variation minimale de température pour datasave
extern int16_t*  periLastVal_;                  // ptr ds buffer : dernière valeur de température  
extern int16_t*  periAlim_;                     // ptr ds buffer : dernière tension d'alimentation
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
extern byte*     periInput;                    // ptr ds buffer : Mode fonctionnement inters (4 bytes par switch)           
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
extern int16_t*  periThOffset_;                 // ptr ds buffer : offset correctif sur mesure température
extern int16_t*  periThmin_;                    // ptr ds buffer : alarme mini th
extern int16_t*  periThmax_;                    // ptr ds buffer : alarme maxi th
extern int16_t*  periVmin_;                     // ptr ds buffer : alarme mini volts
extern int16_t*  periVmax_;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*     periProtocol;                 // ptr ds buffer : protocole ('T'CP/'U'DP)

extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

extern char     inptyps[];                     // libellés types sources inputs
extern char     inptypd[];                     // libellés types destinations inputs
extern char     inpact[];                      // libellés actions
extern char     psps[];                        // libellés staPulse

extern int  chge_pwd; //=FAUX;

extern byte mask[];

char protoChar[]=PROTOCHAR;
char protocStr[]=PROTOCSTR;

void showLine(EthernetClient* cli,int i,char* pkdate);

void subModePulseTime(EthernetClient* cli,uint8_t npu,uint32_t* pulse,uint32_t* dur,char* fonc1,char* fonc2,char onetwo)
{

  uint8_t pbit=PMTTE_PB;if(onetwo=='O'){pbit=PMTOE_PB;} pbit+=PCTLBIT*npu;
  uint8_t val=(((*(uint16_t*)periSwPulseCtl)>>pbit)&0x01)+PMFNCVAL;                                        
  cli->print("<font size=\"2\">");
  fonc1[LENNOM-1]=onetwo;
  checkboxTableHtml(cli,&val,fonc1,-1,0,"");                       // bit enable pulse
  if(*(pulse+npu)<0){*(pulse+npu)=0;}  
  numTableHtml(cli,'l',(pulse+npu),fonc2,8,0,2);                 // durée pulse   
//  char a[11];sprintf(a,"%06u",(uint32_t)*dur);a[10]='\0';              // valeur courante 32bits=4G soit 10 chiffres
  cli->print("<br>(");cli->print(*(dur+npu));cli->println(")</font>");

}

void perinpfnc(EthernetClient* cli,uint8_t nuinp,uint16_t val,char type,uint8_t lmax,char* ft,uint8_t nuv)      // type='c' checkbox ; 'n' num / ft fonct transport / nuv num var
{      // type input src, num det src,type input dest, num det dest, valeur, enable, action, 4*2 modes => 2 fonctions de transport avec 5 bits n°input (libf-2) et n°de paramètre (libf-1)                                                                          
  uint8_t vv=0;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);
  ft[LENNOM-1]=(char)(nuinp+PMFNCHAR);
  
  switch (type){
    case 'c':if(val!=0){vv=1;};checkboxTableHtml(cli,&vv,ft,-1,0,"");break;
    case 'n':numTableHtml(cli,'d',&val,ft,lmax,0,2);break;
    default: break;
  }
}

void SwCtlTableHtml(EthernetClient* cli)
{
  Serial.print("début SwCtlTableHtml -- periCur=");Serial.print(periCur);Serial.print("  cxtime=");Serial.println(millis()-cxtime);

  // periCur est transféré via les fonctions d'en-tête peri_inp__ et peri_t_sw

  htmlIntro(nomserver,cli);
  cli->println("<body>");            
  cli->println("<form method=\"get\" >");
    cli->print(VERSION);cli->print(" ");
    char pkdate[7]; // pour couleurs des temps des périphériques
    cliPrintDateHeure(cli,pkdate);cli->println();
    cli->print(periCur);cli->print("-");cli->print(periNamer);cli->print(" ");
    cli->print("<font size=\"2\">");for(int j=0;j<4;j++){cli->print(periIpAddr[j]);if(j<3){cli->print(".");}}
    if(*periProg!=0){cli->print("/port=");cli->print(*periPort);cli->print(" ");}
    for(int j=0;j<LENVERSION;j++){cli->print(periVers[j]);}
    cli->println("<br>");//cli->println("</font><br>");

  usrPeriCur(cli,"peri_t_sw_",0,2,0);    

    boutRetour(cli,"retour",0,0);cli->print(" ");  
    cli->println("<input type=\"submit\" value=\" MàJ \">");cli->print(" ");
    char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
    boutFonction(cli,swf,"","refresh",0,0,1,0);cli->print(" ");
    swf[LENNOM-2]='X';
    boutFonction(cli,swf,""," erase ",0,0,1,0);cli->print(" ");
    
    cli->print("<br> détecteurs serveur ");
    char hl[]={"LH"};
    
    for(int k=NBDSRV-1;k>=0;k--){cli->print(hl[(memDetServ>>k)&0x01]);cli->print(" ");}
    cli->println("<br>");

    cli->println("<table>Pulses");                  // pulses
      cli->println("<tr><th></th><th>time One<br>time Two</th><th>free<br>run</th>");

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
      
        cli->print("</td><td> ");
        uint8_t val=(*(uint16_t*)periSwPulseCtl>>(PCTLBIT*pu+PMFRO_PB))&0x01;rfonc[LENNOM-1]='F';   // bit freerun
        checkboxTableHtml(cli,&val,rfonc,-1,0,"");                  
        cli->print("<br>");for(int tsp=0;tsp<LENTSP;tsp++){cli->print(psps[periSwPulseSta[pu]*LENTSP+tsp]);}cli->print("</td>");         // staPulse 

      } // pulse suivant
  cli->print("</tr></table></form>");

    cli->println("<table>Règles en=enable, lv=active level, pr=?, es=edge/static ; 1101 OR to set when srce=1, 1001 NOR to clear when srce=0");
      cli->println("<tr><th></th><th>e.l p.e<br>n.v.r.s</th><th> source </th><th> destin.</th><th> action</th></tr>");

      char xfonc1[]="p_inp1____\0";
      char xfonc2[]="p_inp2____\0";

      uint16_t offsetInp=0;
 
      for(int ninp=0;ninp<NBPERINPUT;ninp++){     // boucle des inputs

            cli->print("<tr><form method=\"get\" >");    
            uint8_t vv;
            byte binp[PERINPLEN];memcpy(binp,periInput+offsetInp,PERINPLEN);
            offsetInp+=PERINPLEN;

           usrPeriCur(cli,"peri_inp__",ninp,2,0);

           cli->print("<td>");cli->print(ninp);cli->print("</td><td>");
            vv=(binp[2]  & PERINPEN_VB);perinpfnc(cli,ninp,vv,'c',1,xfonc1,1);                           // bit enable
            vv=(binp[2]  & PERINPVALID_VB);;perinpfnc(cli,ninp,vv,'c',1,xfonc1,9);                       // bit active level
            vv=(binp[2]  & PERINPOLDLEV_VB);perinpfnc(cli,ninp,vv,'c',1,xfonc1,2);                       // bit prev lev
            vv=(binp[2]  & PERINPDETES_VB);perinpfnc(cli,ninp,vv,'c',1,xfonc1,3);                        // bit edge/static            
           
            vv=(binp[0]  & PERINPNT_MS);                                                                 // type detec source
            selectTableHtml(cli,inptyps,xfonc1,4,2,vv,4,ninp,2);

            vv=(binp[0]>>PERINPNVLS_PB);perinpfnc(cli,ninp,vv,'n',2,xfonc1,5);                           // num detec source
           cli->println("</td>");            
           
            vv=(binp[3]  & PERINPNT_MS);                                                                 // type detec dest
            selectTableHtml(cli,inptypd,xfonc1,4,2,vv,6,ninp,2);

            vv=(binp[3]>>PERINPNVLS_PB);perinpfnc(cli,ninp,vv,'n',2,xfonc1,7);                           // num detec  dest
           cli->print("</td>");            
           //cli->print("<td>");
            vv=(binp[2]&PERINPACT_MS)>>PERINPACTLS_PB;                                                   // action 
            selectTableHtml(cli,inpact,xfonc1,12,5,vv,8,ninp,2);

           cli->println("</td>");

/*            for(int mode=7;mode>=0;mode--){                                                               // 8 bits
                cli->print(" ");             
                vv=(binp[1]>>(PERINPRULESLS_PB+mode))&0x01;perinpfnc(cli,ninp,vv,'c',1,xfonc2,mode);
            } // mode suivant    */
            cli->println("<td><input type=\"submit\" value=\"MàJ\"></td>");
            cli->print("</form></tr>");
                        
        } // input suivant
  cli->print("</table></body></html>");
  Serial.print("fin SwCtlTableHtml  cxtime=");Serial.println(millis()-cxtime);
}

void periTableHtml(EthernetClient* cli)
{
  int i,j;
  int savePeriCur=periCur;   // save periCur et restore à la fin de periTable


Serial.print("début péritable ; remote_IP ");serialPrintIp(remote_IP_cur);Serial.print(" cxtime=");Serial.println(millis()-cxtime); 

  htmlIntro(nomserver,cli);

  float th;                                  // pour temp DS3231
  ds3231.readTemp(&th);

        cli->println("<body>");
        cli->println("<form method=\"GET \">");

          cli->print(VERSION);cli->print(" ");
          #ifdef _MODE_DEVT
            cli->print("MODE_DEVT ");
          #endif _MODE_DEVT
          #ifdef _MODE_DEVT2
            cli->print("MODE_DEVT2 ");
          #endif _MODE_DEVT2

          char pkdate[7];cliPrintDateHeure(cli,pkdate);
          cli->println("; local IP ");cli->print(Ethernet.localIP());cli->println(" ");
          cli->print(th);cli->println("°C<br>");

          usrFormHtml(cli,1);

          boutRetour(cli,"refresh",0,0);
          
          numTableHtml(cli,'d',&perrefr,"per_refr__",4,0,0);

          cli->print("(");long sdsiz=fhisto.size();cli->print(sdsiz);cli->println(") ");
          numTableHtml(cli,'i',(uint32_t*)&sdpos,"sd_pos____",9,0,0);
          
          cli->println("<input type=\"submit\" value=\"ok\"> ");
          
          boutFonction(cli,"dump_sd___","","dump SD",0,0,0,0);    
          boutFonction(cli,"reset_____","","reset",0,0,0,0);
          boutFonction(cli,"cfgserv___","","config",0,0,0,0);
          cli->print("<br>");
          boutFonction(cli,"remote____","","remote_cfg",0,0,0,0);
          boutFonction(cli,"remotehtml","","remotehtml",0,0,0,0);
          boutFonction(cli,"thermoscfg","","thermo_cfg",0,0,0,0);
          boutFonction(cli,"thermoshow","","thermoshow",0,0,0,0);
          boutFonction(cli,"timershtml","","timershtml",0,0,0,0);
          boutFonction(cli,"dsrvhtml__","","detsrvhtml",0,0,0,0);                 
        
        cli->println("</form>");

          detServHtml(cli,&memDetServ,&libDetServ[0][0]);  // détecteurs serveur

          cli->println("<table>");
              cli->println("<tr>");
                cli->println("<th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>D_l<br>i_e<br>s_v</th><th></th><th>mac_addr<br>ip_addr</th><th>vers. prot<br>last out<br>last in</th><th></th>"); 
              cli->println("</tr>");
 
              for(i=1;i<=NBPERIF;i++){
                // !!!!!!!!!!!!!!!!!! pericur doit étre le premier de la liste !!!!!!!!!!!!!!!!!!!!!!!!!
                // pour permettre periLoad préalablement aux mises à jour des données quand le navigateur 
                // envoie une commande GET/POST 
                // et pour assurer l'effacement des bits de checkbox : le navigateur ne renvoie que ceux "checkés"
              showLine(cli,i,pkdate);
              }
          cli->println("</table>");
        cli->println("</body></html>");
periCur=savePeriCur;if(periCur!=0){periLoad(periCur);}
Serial.print("fin péritable - cxtime=");Serial.println(millis()-cxtime); 
}

void periLineHtml(EthernetClient* cli,int i)
{
  int j;

  Serial.print("début periLineHtml -- periCur=");Serial.print(periCur);Serial.print("/");Serial.println(i);
  // en principe periCur est à jour 

  htmlIntro(nomserver,cli);
  cli->println("<body>");            
  cli->println("<form method=\"get\" >");
    cli->print(VERSION);cli->print(" ");
    char pkdate[7]; // ppour couleurs des temps des périphériques
    cliPrintDateHeure(cli,pkdate);cli->println();
    cli->print(periCur);cli->print("-");cli->print(periNamer);cli->println("<br>");

    boutRetour(cli,"retour",0,0);cli->print(" ");  
    cli->println("<input type=\"submit\" value=\" MàJ \">");cli->print(" ");
    char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    boutFonction(cli,line,"","refresh",0,0,1,0);cli->print(" ");
    if(*periSwNb!=0){
      char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
      boutFonction(cli,swf,"","Switchs",3,0,0,0);}
                
                periInitVar();periLoad(i);periCur=i;
                if(*periSwNb>MAXSW){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}

                cli->println("<table><tr>");
                cli->println("<th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>__D__l<br>__i__e<br>__s__v</th><th></th><th>mac_addr<br>ip_addr</th><th>version DS18<br>last out<br>last in</th>"); //<th>det<br>srv<br>en</th>"); //<th>time One<br>time Two</th><th>f<br>r</th><th>e.l _f_H.a<br>n.x _t_L.c</th><th>___det__srv._pul<br></th>");
                cli->println("</tr>");

                cli->println("<tr>");
                  //cli->println("<form method=\"GET \">");
                      
                      cli->print("<td>");
                      cli->println(periCur);
                      usrPeriCur(cli,"peri_cur__",0,2,3);
                      cli->print("<td><input type=\"text\" name=\"peri_nom__\" value=\"");
                        cli->print(periNamer);cli->print("\" size=\"12\" maxlength=\"");cli->print(PERINAMLEN-1);cli->println("\" ></td>");
                      textTableHtml_(cli,periLastVal_,periThmin_,periThmax_,1,1);
                      //numTableHtml(cli,'I',periThmin_,"peri_thmin",5,0,0);cli->println("<br>");
                      //numTableHtml(cli,'I',periThmax_,"peri_thmax",5,3,0);
                      cli->print(*periThmin_);cli->println("<br>");
                      cli->print(*periThmax_);
                      textTableHtml_(cli,periAlim_,periVmin_,periVmax_,1,1);
                      numTableHtml(cli,'I',periVmin_,"peri_vmin_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'I',periVmax_,"peri_vmax_",5,3,0);
                      numTableHtml(cli,'d',(uint32_t*)periPerTemp,"peri_rtemp",5,2,0);cli->println("<br>");
                      numTableHtml(cli,'r',periPitch_,"peri_pitch",5,0,0);cli->print("<br>");
                      numTableHtml(cli,'r',periThOffset_,"peri_tofs_",5,3,0);
                      numTableHtml(cli,'l',(uint32_t*)periPerRefr,"peri_refr_",5,2,0);cli->println("<br>");
                      checkboxTableHtml(cli,(uint8_t*)periProg,"peri_prog_",-1,3,"");
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
                      cli->print("<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){cli->print(periVers[j]);}
                      cli->println(" ");cli->println((char)*periProtocol);cli->println("<br>");
                      
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
                      
                      //cli->println("<td><input type=\"submit\" value=\"   MàJ   \"><br>");

                cli->println("</tr></table></form>");
}

void showLine(EthernetClient* cli,int numline,char* pkdate)
{
  int j;
                periInitVar();periLoad(numline);periCur=numline;
                if(*periSwNb>MAXSW){periInitVar();periSave(numline,PERISAVESD);}     //periCheck(numline,"SwNb");periInitVar();periSave(numline,PERISAVESD);}
                if(*periDetNb>MAXDET){periInitVar();periSave(numline,PERISAVESD);}  //periCheck(numline,"detNb");periInitVar();periSave(numline,PERISAVESD);}

                cli->println("<tr>");
                  cli->println("<form method=\"GET \">");
                      
                      cli->print("<td>");
                      cli->println(periCur);
                      usrPeriCur(cli,"peri_cur__",0,2,3);
                      cli->print("<td>");cli->print(periNamer);cli->print("</td>");
                      textTableHtml_(cli,periLastVal_,periThmin_,periThmax_,1,1);
                      cli->print((float)*periThmin_/100);cli->println("<br>");
                      cli->print((float)*periThmax_/100);cli->print("</td>");
                      textTableHtml_(cli,periAlim_,periVmin_,periVmax_,1,1);
                      cli->print((float)*periVmin_/100);cli->println("<br>");
                      cli->print((float)*periVmax_/100);cli->print("</td>");
                      cli->print("<td>");cli->print(*periPerTemp);cli->print("<br>");
                      cli->print((float)*periPitch_/100);cli->print("<br>");
                      cli->print((float)*periThOffset_/100);cli->print("</td>");
                      cli->print("<td>");cli->print(*periPerRefr);cli->print("<br>");
                      if(*periProg!=0){cli->print("serv");}cli->print("</td>");
                      cli->print("<td>");cli->print(*periSwNb);cli->print("<br>");cli->print(*periDetNb);cli->print("</td>");
                      cli->println("<td>");
                      for(uint8_t k=0;k<*periSwNb;k++){char oi[2]={'O','I'};cli->print(oi[(*periSwVal>>((k*2)+1))&0x01]);
                      cli->print("_");
                      cli->print(oi[(*periSwVal>>((k*2)))&0x01]);if(k<*periSwNb-1){cli->print("<br>");}}
                      cli->print("</td>");                     
                      cli->print("<td>");
                      for(uint8_t k=0;k<*periDetNb;k++){char oi[2]={'O','I'};cli->print(oi[(*periDetVal>>(k*2))&DETBITLH_VB]);if(k<*periDetNb-1){cli->print("<br>");}}
                      cli->println("</td>");
                      cli->print("<td>");
                      for(int k=0;k<6;k++){cli->print(chexa[periMacr[k]/16]);cli->print(chexa[periMacr[k]%16]);}cli->print("<br>");
                      if(*periProg!=0 || *periProtocol=='U'){cli->print("port=");cli->print(*periPort);}cli->print("<br>");
                      cli->print("<font size=\"2\">");for(j=0;j<4;j++){cli->print(periIpAddr[j]);if(j<3){cli->print(".");}}cli->println("</font></td>");
                      cli->print("<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){cli->print(periVers[j]);}
                      cli->println(" ");long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
                      cli->print(protocStr+LENPROSTR*p);cli->println("<br>");
                      
                      char colourbr[6];
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
                      printPeriDate(cli,periLastDateOut);
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
                      printPeriDate(cli,periLastDateIn);
                      setColour(cli,"black");
                      cli->println("</font></td>");

                      cli->print("<td>");
                      char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
                      boutFonction(cli,line,"","Periph",0,1,0,0);
                      
                      if(*periSwNb!=0){
                        char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
                        boutFonction(cli,swf,"","Switchs",0,0,0,0);}
                      cli->print("</td>"); 

                  cli->print("</form>");
                cli->println("</tr>");
}
