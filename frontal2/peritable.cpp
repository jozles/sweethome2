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

#include <MemoryFree.h>;

extern Ds3231 ds3231;

extern File      fhisto;      // fichier histo sd card
extern long      sdpos;
extern long      fhsize;      // remplissage fhisto
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
extern char      memosTable[LMEMO*NBMEMOS];

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
extern int16_t*  periThOffset_;                // ptr ds buffer : offset correctif sur mesure température
extern int16_t*  periThmin_;                   // ptr ds buffer : alarme mini th
extern int16_t*  periThmax_;                   // ptr ds buffer : alarme maxi th
extern int16_t*  periVmin_;                    // ptr ds buffer : alarme mini volts
extern int16_t*  periVmax_;                    // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*     periProtocol;                 // ptr ds buffer : protocole ('T'CP/'U'DP)
extern uint16_t* periAnal;                     // ptr ds buffer : analog value
extern uint16_t* periAnalLow;                  // ptr ds buffer : low analog value 
extern uint16_t* periAnalHigh;                 // ptr ds buffer : high analog value 
extern uint16_t* periAnalOffset1;              // ptr ds buffer : offset on adc value
extern float*    periAnalFactor;               // ptr ds buffer : factor to float for analog value
extern float*    periAnalOffset2;              // ptr ds buffer : offset on float value
extern uint8_t*  periAnalCb;                   // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*  periAnalDet;                  // ptr ds buffer : 5 x n° détect serveur
extern int8_t*   periAnalMemo;                 // ptr ds buffer : 5 x n° mémo dans table mémos
extern uint8_t*  periDigitCb;                  // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*  periDigitDet;                 // ptr ds buffer : 5 x n° détect serveur
extern int8_t*   periDigitMemo;                // ptr ds buffer : 5 x n° mémo dans table mémos


extern int8_t    periMess;                     // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

extern char      rulop[];                       // libellés opérations logiques regles analog/digital inputs

extern char      inptyps[];                     // libellés types sources regles switchs
extern char      inptypd[];                     // libellés types destinations regles switchs
extern char      inpact[];                      // libellés actions
extern char      psps[];                        // libellés staPulse

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

void perinpBfnc(char* buf,uint8_t nuinp,uint16_t val,char type,uint8_t lmax,char* ft,uint8_t nuv)      // type='c' checkbox ; 'n' num / ft fonct transport / nuv num var
{      // type input src, num det src,type input dest, num det dest, valeur, enable, action, 4*2 modes => 2 fonctions de transport avec 5 bits n°input (libf-2) et n°de paramètre (libf-1)                                                                          
  uint8_t vv=0;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);
  ft[LENNOM-1]=(char)(nuinp+PMFNCHAR);
  
  switch (type){
    case 'c':if(val!=0){vv=1;};checkboxTableBHtml(buf,&vv,ft,-1,0,"");break;
    case 'n':numTf(buf,'d',&val,ft,lmax,0,2);break;
    default: break;
  }
}


void SwCtlTableHtml(EthernetClient* cli)
{
  char buf[2500];buf[0]='\0';
  
  Serial.print("début SwCtlTableHtml -- periCur=");Serial.print(periCur);Serial.print("  cxtime=");Serial.print(millis()-cxtime);
  Serial.print("  free=");Serial.println(freeMemory(), DEC);

  // periCur est transféré via les fonctions d'en-tête peri_inp__ et peri_t_sw

  htmlIntroB(buf,nomserver,cli);
  
  strcat(buf,"<body>");            
  strcat(buf,"<form method=\"get\" >");

/* en-tête (vers-dh-IP serv-modele) */
    strcat(buf,VERSION);strcat(buf," ");
    char pkdate[7]; // pour couleurs des temps des périphériques
    bufPrintDateHeure(buf,pkdate);strcat(buf,"\n");
    concatn(buf,periCur);strcat(buf,"-");strcat(buf,periNamer);strcat(buf," ");
    strcat(buf,"<font size=\"2\">");for(int j=0;j<4;j++){concatn(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}}
    if(*periProg!=0){strcat(buf,"/port=");concatn(buf,*periPort);strcat(buf," ");}
    for(int j=0;j<LENVERSION;j++){concat1a(buf,periVers[j]);}
    strcat(buf,"<br>\n");


  usrPeriCurB(buf,"peri_t_sw_",0,2,0);

/* boutons */
    boutRetourB(buf,"retour",0,0);
    strcat(buf," <input type=\"submit\" value=\" MàJ \"> ");
    char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
    boutF(buf,swf,"","refresh",0,0,1,0);strcat(buf," ");
    swf[LENNOM-2]='X';
    boutF(buf,swf,""," erase ",0,0,1,0);strcat(buf," ");

/* détecteurs */    
    strcat(buf,"<br> détecteurs serveur ");
    char hl[]={"LH"};
    
    for(int k=NBDSRV-1;k>=0;k--){concat1a(buf,hl[(memDetServ>>k)&0x01]);strcat(buf," ");}
    strcat(buf,"<br>");

/* pulses */
    strcat(buf,"<table>Pulses");                  // pulses
      strcat(buf,"<tr><th></th><th>time One<br>time Two</th><th>free<br>run</th>");

    cli->print(buf);buf[0]='\0';

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
  
/* affichage/saisie règles */
  strcat(buf,"</tr></table></form>");
  strcat(buf,"<table>Règles en=enable, lv=active level, pr=?, es=edge/static ; 1101 OR to set when srce=1, 1001 NOR to clear when srce=0");
  strcat(buf,"<tr><th></th><th>e.l p.e<br>n.v.r.s</th><th> source </th><th> destin.</th><th> action</th></tr>");

      char xfonc1[]="p_inp1____\0";
      char xfonc2[]="p_inp2____\0";

      uint16_t offsetInp=0;

      cli->print(buf);buf[0]='\0';
      for(int ninp=0;ninp<NBPERINPUT;ninp++){     // boucle des regles

            strcat(buf,"<tr>\n<form method=\"get\" >");    
            uint8_t vv;
            byte binp[PERINPLEN];memcpy(binp,periInput+offsetInp,PERINPLEN);
            offsetInp+=PERINPLEN;

           usrPeriCurB(buf,"peri_inp__",ninp,2,0);

           strcat(buf,"\n<td>");concatn(buf,ninp);strcat(buf,"</td><td>");
            vv=(binp[2]  & PERINPEN_VB);perinpBfnc(buf,ninp,vv,'c',1,xfonc1,1);                          // bit enable
            vv=(binp[2]  & PERINPVALID_VB);perinpBfnc(buf,ninp,vv,'c',1,xfonc1,9);                       // bit active level
            vv=(binp[2]  & PERINPOLDLEV_VB);perinpBfnc(buf,ninp,vv,'c',1,xfonc1,2);                      // bit prev lev
            vv=(binp[2]  & PERINPDETES_VB);perinpBfnc(buf,ninp,vv,'c',1,xfonc1,3);                       // bit edge/static            
           
            vv=(binp[0]  & PERINPNT_MS);                                                                 // type detec source
            selectTableBHtml(buf,inptyps,xfonc1,4,2,vv,4,ninp,2);

            vv=(binp[0]>>PERINPNVLS_PB);perinpBfnc(buf,ninp,vv,'n',2,xfonc1,5);                          // num detec source
           strcat(buf,"</td>\n");            
           
            vv=(binp[3]  & PERINPNT_MS);                                                                 // type detec dest
            selectTableBHtml(buf,inptypd,xfonc1,4,2,vv,6,ninp,2);

            vv=(binp[3]>>PERINPNVLS_PB);perinpBfnc(buf,ninp,vv,'n',2,xfonc1,7);                          // num detec  dest
           strcat(buf,"</td>\n");            
           
            vv=(binp[2]&PERINPACT_MS)>>PERINPACTLS_PB;                                                   // action 
            selectTableBHtml(buf,inpact,xfonc1,12,5,vv,8,ninp,2);

           strcat(buf,"</td>");
           strcat(buf,"<td><input type=\"submit\" value=\"MàJ\"></td>");
           strcat(buf,"</form></tr>\n");
           if(strlen(buf)>sizeof(buf)*0.4){cli->print(buf);buf[0]='\0';}
        } // input suivant

  strcat(buf,"</table></body></html>");
  cli->print(buf);
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
          cli->println("<font size=\"2\">");
          cli->println("; local IP ");cli->print(Ethernet.localIP());cli->println(" ");
          cli->print(th);cli->println("°C<br>");

          usrFormHtml(cli,1);

          boutRetour(cli,"refresh",0,0);
          
          numTableHtml(cli,'d',&perrefr,"per_refr__",4,0,0);

          cli->print("(");cli->print(fhsize);cli->println(") ");
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
        
        cli->println("</form>");        // le formulaire NE DOIT PAS intégrer detServHtml qui a son propre usrFormHtml pour gérer les mots de passe
                                        // sinon la fonction user_ref__ serait 2 fois dans la liste et la 2nde (fausse) enverrait à l'accueil        

          detServHtml(cli,&memDetServ,&libDetServ[0][0]);  // détecteurs serveur

          cli->println("<table>");
              cli->println("<tr>");              
                cli->println("<th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>D_l<br>i_e<br>s_v</th><th></th><th>Analog<br>_low<br>_high</th><th>mac_addr<br>ip_addr</th><th>vers. prot<br>last out<br>last in</th><th></th>"); 
              cli->println("</tr>");
 
              for(i=1;i<=NBPERIF;i++){
                // !!!!!!!!!!!!!!!!!! pericur doit étre le premier de la liste !!!!!!!!!!!!!!!!!!!!!!!!!
                // pour permettre periLoad préalablement aux mises à jour des données quand le navigateur 
                // envoie une commande GET/POST 
                // et pour assurer l'effacement des bits de checkbox : le navigateur ne renvoie que ceux "checkés"
              showLine(cli,i,pkdate);
              trigwd();
              }
          cli->println("</table>");
        cli->println("</body></html>");
periCur=savePeriCur;if(periCur!=0){periLoad(periCur);}
Serial.print("fin péritable - cxtime=");Serial.println(millis()-cxtime); 
}

void subCbdet(EthernetClient* cli,uint8_t nbfonc,char* title,char* nfonc,uint8_t nbLi,char* lib,uint8_t libsize,uint8_t* cb,uint8_t* det,int8_t* memo)
{                                 // le n° de fonction permet une seule fonction d'init pour plusieurs formulaires de même structure
  
  uint8_t k,op;
  char namfonct[LENNOM+1];memcpy(namfonct,nfonc,LENNOM);namfonct[LENNOM]='\0';
  char buf[5000];buf[0]='\0';  
  char a[]={"  \0"},colnb=PMFNCHAR;

  strcat(buf,"<form><fieldset><legend>");strcat(buf,title);strcat(buf," :</legend>\n");
  char inifonc[LENNOM];memcpy(inifonc,nfonc,4);memcpy(inifonc+4,"init__",LENNOM-4);
  usrPeriCurB(buf,inifonc,nbfonc,2,0);
  
  strcat(buf,"<table><th></th><th>e.l. p.e<br>n.v. r.s</th><th>det</th>");
  
  for(int i=0;i<nbLi;i++){    
    strcat(buf,"<tr>");    
    namfonct[LENNOM-1]=(char)(i+PMFNCHAR); // le dernier caractère est le n° de ligne ; l'avant dernier le n° de colonne
    strcat(buf,"<td>");a[0]=(char)(i+'1');strcat(buf,a);strcat(buf,(char*)(lib+i*libsize));strcat(buf,"</td>");
   
    strcat(buf,"<td>");
    colnb=PMFNCHAR;
    for(uint8_t j=0;j<4;j++){
      k=(*(cb+i)>>3-j)&0x01;
      namfonct[LENNOM-2]=(char)(colnb);checkboxTableBHtml(buf,&k,namfonct,-1,0,"");       // n° de colonne
      colnb++;}      
    strcat(buf,"</td><td>");
    op=(*(cb+i))>>4;
    selectTableBHtml(buf,rulop,namfonct,7,5,op,colnb-PMFNCHAR,i,0);
    colnb++;

    strcat(buf,"</td><td>");
    namfonct[LENNOM-2]=colnb;colnb++;numTf(buf,'s',&det[i],namfonct,2,0,0);
    namfonct[LENNOM-2]=colnb;   
    strcat(buf,"</td>\n");  

    strcat(buf,"<td><input type=\"text\" name=\"");strcat(buf,namfonct);strcat(buf,"\" value=\"");
    if(memo[i]>=0 && memo[i]<NBMEMOS){strcat(buf,&memosTable[memo[i]*LMEMO]);}
    strcat(buf,"\" size=\"12\" maxlength=\"");concatn(buf,LMEMO-1);strcat(buf,"\"></td>");

    strcat(buf,"</tr>\n");
    cli->print(buf);buf[0]='\0';    // max len pour cli-print() 2048 ???
  }
  strcat(buf,"</table><br>");
  strcat(buf,"<input type=\"submit\" value=\"MàJ\">");
  strcat(buf,"</fieldset></form>\n"); 

  cli->print(buf);buf[0]='\0';
}


void periLineHtml(EthernetClient* cli,int i)
{
  int j;

  Serial.print("début periLineHtml -- periCur=");Serial.print(periCur);Serial.print("/");Serial.println(i);
  // en principe periCur est à jour 

  htmlIntro(nomserver,cli);
  cli->println("<body>");            
  cli->println("<form method=\"get\" >");

/* en-tête (vers-dh-IP serv-modele) */    
    cli->print(VERSION);cli->print(" ");
    char pkdate[7]; // pour couleurs des temps des périphériques
    cliPrintDateHeure(cli,pkdate);cli->println();
    cli->print(periCur);cli->print("-");cli->print(periNamer);cli->println("<br>");
    cli->print("<font size=\"2\">");for(int j=0;j<4;j++){cli->print(periIpAddr[j]);if(j<3){cli->print(".");}}
    if(*periProg!=0){cli->print("/port=");cli->print(*periPort);cli->print(" ");}
    for(int j=0;j<LENVERSION;j++){cli->print(periVers[j]);}
    cli->print("<br>\n");

/* boutons */
    boutRetour(cli,"retour",0,0);cli->print(" ");  
    cli->println("<input type=\"submit\" value=\" MàJ \">");cli->print(" ");
    char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    boutFonction(cli,line,"","refresh",0,0,1,0);cli->print(" ");
    if(*periSwNb!=0){
      char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
      boutFonction(cli,swf,"","Switchs",3,0,0,0);}
    boutFonction(cli,"peri_raz___","","Raz",0,0,0,0);
    
/* ligne périphérique */                
                periInitVar();periLoad(i);periCur=i;
                if(*periSwNb>MAXSW){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}

                cli->println("<table><tr>");
                cli->println("<th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th>._D_ _l<br>._i_ _e<br>._s_ _v</th><th></th><th>mac_addr<br>ip_addr</th><th>version Th<br>last out<br>last in</th>"); //<th>det<br>srv<br>en</th>"); //<th>time One<br>time Two</th><th>f<br>r</th><th>e.l _f_H.a<br>n.x _t_L.c</th><th>___det__srv._pul<br></th>");
                cli->println("</tr>");

                cli->println("<tr>");
                      
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
                      cli->print("</td>");
 /*                   
                      cli->print("<td>");
                      cli->print(*periAnal);cli->print("<br>");
                      numTableHtml(cli,'I',periAnalLow,"peri_ana@_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'I',periAnalHigh,"peri_anaA_",5,0,0);
                      cli->print("</td><td>");
                      numTableHtml(cli,'I',periAnalOffset1,"peri_anaB_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'f',periAnalFactor,"peri_anaC_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'f',periAnalOffset2,"peri_anaD_",5,0,0);
                      cli->print("</td>");
*/                      
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

                cli->println("</tr></table>");

/* table analogique */
                cli->println("Analog Input<br><table>");
                //cli->println("<tr><th></th><th></th><th></th><th></th>");
                //cli->println("</tr>");
                
                cli->println("<tr>");
                    cli->print("<td>Val<br>Low<br>High</td>");
                    cli->print("<td>");
                      cli->print(*periAnal);cli->print("<br>");
                      numTableHtml(cli,'I',periAnalLow,"peri_ana@_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'I',periAnalHigh,"peri_anaA_",5,0,0);
                    cli->print("</td>");
                    cli->print("<td>Off1<br>Fact<br>Off2</td>");
                    cli->print("<td>");
                      numTableHtml(cli,'I',periAnalOffset1,"peri_anaB_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'f',periAnalFactor,"peri_anaC_",5,0,0);cli->println("<br>");
                      numTableHtml(cli,'f',periAnalOffset2,"peri_anaD_",5,0,0);
                    cli->print("</td>");
                cli->print("</tr></table><br></form>\n");
            
#define ANASIZLIB   3
    char aLibState[]={"> \0=>\0><\0=<\0< "};
                //Serial.print("          periLine()=========");periPrint(periCur);
                subCbdet(cli,0,"Analog Input Rules","rul_ana___",NBANST,aLibState,ANASIZLIB,periAnalCb,periAnalDet,periAnalMemo);

  /* table inputs */
#define DIGITSIZLIB 3
    char dLibState[MAXDET*DIGITSIZLIB];
    memset(dLibState,0x00,MAXDET*DIGITSIZLIB);
    
/*    for(uint8_t i=MAXDET-1;i>=0;i--){
      dLibState[i*INPUTSIZLIB]=(*periDetVal>>((MAXDET-i)*2))&0x01+'0';
      dLibState[i*INPUTSIZLIB+1]=' ';}
*/    
    for(uint8_t k=0;k<*periDetNb;k++){
      char oi[2]={'O','I'};
      dLibState[k*DIGITSIZLIB]=oi[(*periDetVal>>(k*2))&DETBITLH_VB];
      dLibState[i*DIGITSIZLIB+1]='_';}

                subCbdet(cli,1,"Digital Inputs Rules","rul_dig___",*periDetNb,dLibState,DIGITSIZLIB,periDigitCb,periDigitDet,periDigitMemo);
}


void showLineA(EthernetClient* cli,int numline,char* pkdate)
{
  unsigned long t0=micros();
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
                      long late;
                      late=*periPerRefr+*periPerRefr/10;
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateOut,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
                      printPeriDate(cli,periLastDateOut);
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColour(cli,colourbr);
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
Serial.print(" t=");Serial.println(micros()-t0);
}



void showLine(EthernetClient* cli,int numline,char* pkdate)
{
  unsigned long t0=micros();
  int j,s;
#define LBSHOWLINE 1000
  char buf[1000];buf[0]='\0';

                periInitVar();periLoad(numline);periCur=numline;
                if(*periSwNb>MAXSW){periInitVar();periSave(numline,PERISAVESD);}     //periCheck(numline,"SwNb");periInitVar();periSave(numline,PERISAVESD);}
                if(*periDetNb>MAXDET){periInitVar();periSave(numline,PERISAVESD);}  //periCheck(numline,"detNb");periInitVar();periSave(numline,PERISAVESD);}

                //cli->println("<tr>");
                  //cli->println("<form method=\"GET \">");
                      
                      //cli->print("<td>");
          strcat(buf,"<tr>\n<form method=\"GET \"><td>");
                      //cli->println(periCur);
          concatn(buf,periCur);
                      //usrPeriCur(cli,"peri_cur__",0,2,3);                      
          strcat(buf,"\n<p hidden><input type=\"text\" name=\"user_ref_");
          concat1a(buf,(char)(usernum+PMFNCHAR));
          strcat(buf,"\" value=\"");
          concatn(buf,usrtime[usernum]);
          strcat(buf,"\">");
          char fonc[]="peri_cur__\0\0";concat1a(fonc,(char)(PMFNCHAR));
          numTf(buf,'i',&periCur,fonc,2,3,0);
          strcat(buf,"</p>\n");
                      //cli->print("<td>");cli->print(periNamer);cli->print("</td>");
          strcat(buf,"<td>");strcat(buf,periNamer);strcat(buf,"</td>");
                      //textTableHtml_(cli,periLastVal_,periThmin_,periThmax_,1,1);
          textTbl(buf,periLastVal_,periThmin_,periThmax_,1,1);
                      //cli->print((float)*periThmin_/100);cli->println("<br>");                      
          concatnf(buf,(float)*periThmin_/100);strcat(buf,"<br>");
                      //cli->print((float)*periThmax_/100);cli->print("</td>");
          concatnf(buf,(float)*periThmax_/100);strcat(buf,"</td>\n");                                
                      //textTableHtml_(cli,periAlim_,periVmin_,periVmax_,1,1);                      
          textTbl(buf,periAlim_,periVmin_,periVmax_,1,1);          
                      //cli->print((float)*periVmin_/100);cli->println("<br>");
          concatnf(buf,(float)*periVmin_/100);strcat(buf,"<br>\n");
                      //cli->print((float)*periVmax_/100);cli->print("</td>");
          concatnf(buf,(float)*periVmax_/100);strcat(buf,"</td>");
                      //cli->print("<td>");cli->print(*periPerTemp);cli->print("<br>");                      
          strcat(buf,"<td>");concatn(buf,*periPerTemp);strcat(buf,"<br>");
                      //cli->print((float)*periPitch_/100);cli->print("<br>");
          concatnf(buf,(float)*periPitch_/100);strcat(buf,"<br>");
                      //cli->print((float)*periThOffset_/100);cli->print("</td>");
          concatnf(buf,(float)*periThOffset_/100);strcat(buf,"</td>");
                      //cli->print("<td>");cli->print(*periPerRefr);cli->print("<br>");
          strcat(buf,"<td>");concatn(buf,*periPerRefr);strcat(buf,"<br>");
                      //if(*periProg!=0){cli->print("serv");}cli->print("</td>");
          if(*periProg!=0){strcat(buf,"serv");}strcat(buf,"</td>");         
                      //cli->print("<td>");cli->print(*periSwNb);cli->print("<br>");cli->print(*periDetNb);cli->print("</td>");cli->println("<td>");
          strcat(buf,"<td>");concatn(buf,*periSwNb);strcat(buf,"<br>");concatn(buf,*periDetNb);strcat(buf,"</td><td>");
                      //for(uint8_t k=0;k<*periSwNb;k++){char oi[2]={'O','I'};cli->print(oi[(*periSwVal>>((k*2)+1))&0x01]);cli->print("_");
          for(uint8_t k=0;k<*periSwNb;k++){
                      char oi[2]={'O','I'};concat1a(buf,oi[(*periSwVal>>((k*2)+1))&0x01]);strcat(buf,"_");
                      //cli->print(oi[(*periSwVal>>((k*2)))&0x01]);if(k<*periSwNb-1){cli->print("<br>");}}
          concat1a(buf,oi[(*periSwVal>>((k*2)))&0x01]);if(k<*periSwNb-1){strcat(buf,"<br>");}}
                      //cli->print("</td>");cli->print("<td>");
          strcat(buf,"</td><td>");
                      //for(uint8_t k=0;k<*periDetNb;k++){char oi[2]={'O','I'};cli->print(oi[(*periDetVal>>(k*2))&DETBITLH_VB]);if(k<*periDetNb-1){cli->print("<br>");}}
          strcat(buf,"<font size=\"2\">");
          for(uint8_t k=0;k<*periDetNb;k++){char oi[2]={'O','I'};concat1a(buf,oi[(*periDetVal>>(k*2))&DETBITLH_VB]);if(k<*periDetNb-1){strcat(buf,"<br>");}}
                      //cli->println("</td>");cli->print("<td>");
          strcat(buf,"</font></td>\n");
          strcat(buf,"<td>");
          concatnf(buf,(float)(*periAnal+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2);strcat(buf,"<br>");
          concatnf(buf,(float)(*periAnalLow+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2);strcat(buf,"<br>");
          concatnf(buf,(float)(*periAnalHigh+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2);strcat(buf,"<br>");
          strcat(buf,"</td>");                    
          strcat(buf,"<td>");
                      //for(int k=0;k<6;k++){cli->print(chexa[periMacr[k]/16]);cli->print(chexa[periMacr[k]%16]);}cli->print("<br>");
          for(int k=0;k<6;k++){concat1a(buf,chexa[periMacr[k]/16]);concat1a(buf,chexa[periMacr[k]%16]);}strcat(buf,"<br>");
                      //if(*periProg!=0 || *periProtocol=='U'){cli->print("port=");cli->print(*periPort);}cli->print("<br>");
          if(*periProg!=0 || *periProtocol=='U'){strcat(buf,"port=");concatn(buf,*periPort);}strcat(buf,"<br>");
                      //cli->print("<font size=\"2\">");for(j=0;j<4;j++){cli->print(periIpAddr[j]);if(j<3){cli->print(".");}}cli->println("</font></td>");
          strcat(buf,"<font size=\"2\">");for(j=0;j<4;j++){concatn(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}}strcat(buf,"</font></td>\n");
                      //cli->print("<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){cli->print(periVers[j]);}
          strcat(buf,"<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){concat1a(buf,periVers[j]);}
                      //cli->println(" ");long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
          strcat(buf," ");long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
                      //cli->print(protocStr+LENPROSTR*p);cli->println("<br>");
          char* a=protocStr+LENPROSTR*p;strcat(buf,a);strcat(buf,"<br>\n");
   
                      char colourbr[6];
                      long late;
                      late=*periPerRefr+*periPerRefr/10;
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateOut,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setCol(buf,colourbr);
                      //printPeriDate(cli,periLastDateOut);
          concatDate(buf,periLastDateOut);
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setCol(buf,colourbr);
                      //printPeriDate(cli,periLastDateIn);
          concatDate(buf,periLastDateIn);
                      setCol(buf,"black");
                      //cli->println("</font></td>");cli->print("<td>");
          strcat(buf,"</font></td><td>");
                       
                      char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
                      //boutFonction(cli,line,"","Periph",0,1,0,0);
          boutF(buf,line,"","Periph",0,1,0,0);
                      
                      if(*periSwNb!=0){
                        char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
                        //boutFonction(cli,swf,"","Switchs",0,0,0,0);}
          boutF(buf,swf,"","Switchs",0,0,0,0);}              
                      //cli->print("</td>"); 

  
          strcat(buf,"</form></tr>");

if(strlen(buf)>=LBSHOWLINE){Serial.print("trop grand **************************");ledblink(BCODESHOWLINE);}

//Serial.print(" Strlen(showline)=");Serial.print(strlen(buf));Serial.print(" t=");Serial.print(micros()-t0);
          cli->print(buf);
//Serial.print(" tx=");Serial.println(micros()-t0);          

}
