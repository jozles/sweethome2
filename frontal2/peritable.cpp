#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>
//#include <SD.h>
#include "ds3231.h"
#include <shconst2.h>
#include <shutil2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "utilhtml.h"
#include "pageshtml.h"

#include <MemoryFree.h>;

extern Ds3231 ds3231;

extern File32    fhisto;      // fichier histo sd card
extern long      histoPos;
extern char      histoDh[LDATEA];
extern long      fhsize;      // remplissage fhisto
extern char*     nomserver;
extern uint32_t  memDetServ;  // image mémoire NBDSRV détecteurs
extern char      libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t  sourceDetServ();

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
extern uint8_t*  periAnalDestDet;              // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*  periAnalRefDet;               // ptr ds buffer : 5 x n° détect serveur pour op logique (0xff si rien)
extern int8_t*   periAnalMemo;                 // ptr ds buffer : 5 x n° mémo dans table mémos
extern uint8_t*  periDigitCb;                  // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*  periDigitDestDet;             // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*  periDigitRefDet;              // ptr ds buffer : 4 x n° détect serveur pour op logique (0xff si rien)
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

char pkdate[7];

void showLine(char* buf,EthernetClient* cli,int i,char* pkdate);


void perifHeader(char* buf)
{
    concatn(buf,periCur);strcat(buf,"-");strcat(buf,periNamer);strcat(buf," ");
    strcat(buf,"<font size=\"2\">");for(int j=0;j<4;j++){concatn(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}}
    if(*periProg!=0){strcat(buf," / port=");concatn(buf,*periPort);strcat(buf,"  v");}
    for(int j=0;j<LENVERSION;j++){concat1a(buf,periVers[j]);}
    strcat(buf,"<br>\n");
}

void subModePulseTime(char* buf,uint8_t npu,uint32_t* pulse,uint32_t* dur,char* fonc1,char* fonc2,char onetwo)
{

  uint8_t pbit=PMTTE_PB;if(onetwo=='O'){pbit=PMTOE_PB;} pbit+=PCTLBIT*npu;
  uint8_t val=(((*(uint16_t*)periSwPulseCtl)>>pbit)&0x01)+PMFNCVAL;                                        
  strcat(buf,"<font size=\"2\">");
  fonc1[LENNOM-1]=onetwo;
  checkboxTableBHtml(buf,&val,fonc1,-1,0,"");                       // bit enable pulse
  if(*(pulse+npu)<0){*(pulse+npu)=0;}  
  numTf(buf,'l',(pulse+npu),fonc2,8,0,2);                 // durée pulse   
//  char a[11];sprintf(a,"%06u",(uint32_t)*dur);a[10]='\0';              // valeur courante 32bits=4G soit 10 chiffres
  strcat(buf,"<br>(");concatn(buf,*(dur+npu));strcat(buf,")</font>");
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
  const uint16_t lb0=LBUF4000;
  char buf[lb0];buf[0]='\0';
  unsigned long begSwT=millis();
  
  Serial.print("début SwCtlTableHtml -- periCur=");Serial.print(periCur);
  Serial.print("  free=");Serial.println(freeMemory(), DEC);

  // periCur est transféré via les fonctions d'en-tête peri_inp__ et peri_t_sw

    htmlIntroB(buf,nomserver,cli);
    pageHeader(buf);
    perifHeader(buf);
    usrPeriCurB(buf,"peri_t_sw_",0,2,0);

/* boutons */
    boutRetourB(buf,"retour",0,0);
    strcat(buf," <input type=\"submit\" value=\" MàJ \"> ");
    char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
    boutF(buf,swf,"","refresh",0,0,1,0);strcat(buf," ");
    swf[LENNOM-2]='X';
    boutF(buf,swf,""," erase ",0,0,1,0);strcat(buf," <br>\n");

    ethWrite(cli,buf);

/* pulses */
    strcat(buf,"<table>Pulses");                  // pulses
    strcat(buf,"<tr><th></th><th>time One<br>time Two</th><th>free<br>run</th>");

      char pfonc[]="peri_pto__\0";            // transporte la valeur pulse time One
      char qfonc[]="peri_ptt__\0";            // transporte la valeur pulse time Two
      char rfonc[]="peri_otf__\0";            // transporte les bits freerun et enable pulse de periPulseMode (LENNOM-1= ,'F','O','T')

      strcat(buf,"<tr>");

      for(int pu=0;pu<NBPULSE;pu++){          // boucle des pulses

        strcat(buf,"<td>");concatn(buf,pu);strcat(buf,"</td>");
        
        pfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        qfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        rfonc[LENNOM-2]=(char)(pu+PMFNCHAR);        
      
        strcat(buf,"<td>");
        subModePulseTime(buf,pu,periSwPulseOne,periSwPulseCurrOne,rfonc,pfonc,'O');          // bit et valeur time one
        strcat(buf,"<br>");
        subModePulseTime(buf,pu,periSwPulseTwo,periSwPulseCurrTwo,rfonc,qfonc,'T');          // bit et valeur time two
      
        strcat(buf,"</td><td> ");
        uint8_t val=(*(uint16_t*)periSwPulseCtl>>(PCTLBIT*pu+PMFRO_PB))&0x01;rfonc[LENNOM-1]='F';   // bit freerun
        checkboxTableBHtml(buf,&val,rfonc,-1,0,"");                  
        strcat(buf,"<br>");for(int tsp=0;tsp<LENTSP;tsp++){concat1a(buf,psps[periSwPulseSta[pu]*LENTSP+tsp]);}strcat(buf,"</td>\n");         // staPulse 

        ethWrite(cli,buf);
      } // pulse suivant
      
      strcat(buf,"</tr></table></form>");
      ethWrite(cli,buf);

/* détecteurs */    
    detServHtml(cli,&memDetServ,&libDetServ[0][0]);
  
/* affichage/saisie règles */
  strcat(buf,"<table>Règles en=enable, rf=(rise/fall if edge)(direct/inv if static) , pr=prev, es=edge/static ; follow srce 1001 -0- 1001 -1-");
  strcat(buf,"<tr><th></th><th>e.r p.e<br>n.f.r.s</th><th> source </th><th> destin.</th><th> action</th></tr>");

      char xfonc1[]="p_inp1____\0";
      char xfonc2[]="p_inp2____\0";

      uint16_t offsetInp=0;
      uint8_t ni=0;                 // nbre lignes ds buffer
      uint16_t lb;

      ethWrite(cli,buf);
      
      for(int ninp=0;ninp<NBPERINPUT;ninp++){     // boucle des regles

            ni++;
            strcat(buf,"\n<form method=\"get\" >");    
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
            selectTableBHtml(buf,inpact,xfonc1,NBACT,5,vv,8,ninp,2);

           strcat(buf,"</td>");
           strcat(buf,"<td><input type=\"submit\" value=\"MàJ\"></td>");
                                                                                  //strcat(buf, ajouter nom de la source si det
           strcat(buf,"</form></tr>\n\n");

           lb=strlen(buf);
           //Serial.print("lb/lb0/ni/lb0-lb/lb_ni+100 ");Serial.print(lb);Serial.print(" ");Serial.print(lb0);Serial.print(" ");Serial.print(ni);Serial.print(" ");Serial.print(lb0-lb);Serial.print(" ");Serial.println(lb/ni+100);
           if((lb0-lb)<(lb/ni+100)){ethWrite(cli,buf);ni=0;}
           
        } // input suivant

  strcat(buf,"</table></body></html>");
  ethWrite(cli,buf);
  Serial.print("fin SwCtlTableHtml  dur=");Serial.println(millis()-begSwT);
}

void periTableHtml(EthernetClient* cli)
{
  const uint16_t lb0=LBUF4000;
  char buf[lb0];buf[0]='\0';
  int i,j;
  int savePeriCur=periCur;   // save periCur et restore à la fin de periTable

  unsigned long begPT=millis();

Serial.print("peritable ; remote_IP ");serialPrintIp(remote_IP_cur);

          htmlIntroB(buf,nomserver,cli);
          pageHeader(buf);
          usrFormBHtml(buf,1);
          boutRetourB(buf,"refresh",0,0);
          numTf(buf,'d',&perrefr,"per_refr__",4,0,0);
          ethWrite(cli,buf);
          
          strcat(buf,"(");concatn(buf,fhsize);strcat(buf,") ");
          numTf(buf,'i',(uint32_t*)&histoPos,"hist_sh___",9,0,0);
          alphaTableHtmlB(buf,histoDh,"hist_sh_D_",LDATEA-2);
          
          strcat(buf,"<input type=\"submit\" value=\"ok\"> ");
          boutF(buf,"dump_his__","","histo",0,0,0,0);strcat(buf,"<br>\n");      
          
          cli->print("<br>");
          boutF(buf,"deco______","","_  deco  _",0,0,0,0);
          boutF(buf,"deco_____B","","_ reboot _",0,0,0,0);          
          boutF(buf,"cfgserv___","","_ config _",0,0,0,0);
          boutF(buf,"remote____","","remote_cfg",0,0,0,0);
          boutF(buf,"remotehtml","","remotehtml",0,0,0,0);
          boutF(buf,"thermoscfg","","thermo_cfg",0,0,0,0);
          boutF(buf,"thermoshow","","thermoshow",0,0,0,0);
          boutF(buf,"timershtml","","timershtml",0,0,0,0);
          boutF(buf,"dsrvhtml__","","detsrvhtml",0,0,0,0);                 
        
          strcat(buf,"</form>\n");      // le formulaire NE DOIT PAS intégrer detServHtml qui a son propre usrFormHtml pour gérer les mots de passe
                                        // sinon la fonction user_ref__ serait 2 fois dans la liste et la 2nde (fausse) enverrait à l'accueil        
          ethWrite(cli,buf);
          
          detServHtml(cli,&memDetServ,&libDetServ[0][0]);  // détecteurs serveur
          
          strcat(buf,"<table><tr><th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>D_l<br>i_e<br>s_v</th><th></th><th>Analog<br>_low<br>_high</th><th>mac_addr<br>ip_addr</th><th>vers. prot<br>last out<br>last in</th><th></th></tr>");
 
          for(i=1;i<=NBPERIF;i++){
                // !!!!!!!!!!!!!!!!!! pericur doit étre le premier de la liste !!!!!!!!!!!!!!!!!!!!!!!!!
                // pour permettre periLoad préalablement aux mises à jour des données quand le navigateur 
                // envoie une commande GET/POST 
                // et pour assurer l'effacement des bits de checkbox : le navigateur ne renvoie que ceux "checkés"
          showLine(buf,cli,i,pkdate);
          trigwd();
          }
          strcat(buf,"</table></body></html>");
          periCur=savePeriCur;if(periCur!=0){periLoad(periCur);}
          Serial.print(" dur=");Serial.println(millis()-begPT); 
}

void subCbdet(char* buf,EthernetClient* cli,uint8_t nbfonc,char* title,char* nfonc,uint8_t nbLi,char* lib,uint8_t libsize,uint8_t nbOp,uint8_t lenOp,char* rulOp,uint8_t* cb,uint8_t* det,uint8_t* rdet,int8_t* memo)
{                                 // le n° de fonction permet une seule fonction d'init pour plusieurs formulaires de même structure
  
  uint8_t k,op;
  char namfonct[LENNOM+1];memcpy(namfonct,nfonc,LENNOM);namfonct[LENNOM]='\0';
  char a[]={"  \0"},colnb=PMFNCHAR;

  strcat(buf,"<form><fieldset><legend>");strcat(buf,title);strcat(buf," :</legend>\n");
  char inifonc[LENNOM];memcpy(inifonc,nfonc,4);memcpy(inifonc+4,"init__",LENNOM-4);
  usrPeriCurB(buf,inifonc,nbfonc,2,0);
  
  strcat(buf,"<table><th></th><th>e.l. p.e<br>n.v. r.s</th><th> op </th><th>rdet</th><th>det</th>\n");
  
  for(int i=0;i<nbLi;i++){    
    strcat(buf,"<tr>");    
    namfonct[LENNOM-1]=(char)(i+PMFNCHAR); // le dernier caractère est le n° de ligne ; l'avant dernier le n° de colonne

/* libellé ligne */
    strcat(buf,"<td>");a[0]=(char)(i+'1');strcat(buf,a);strcat(buf,(char*)(lib+i*libsize));strcat(buf,"</td>");
/* checkbox */
    strcat(buf,"<td>\n");
    colnb=PMFNCHAR;
    for(uint8_t j=0;j<4;j++){
      k=(*(cb+i)>>3-j)&0x01;
      namfonct[LENNOM-2]=(char)(colnb);checkboxTableBHtml(buf,&k,namfonct,-1,0,"");       // n° de colonne
      colnb++;}      
    strcat(buf,"</td><td>");
/* opé logique */
    op=(*(cb+i))>>4;
    selectTableBHtml(buf,rulOp,namfonct,nbOp,lenOp,op,colnb-PMFNCHAR,i,0);
    colnb++;
    strcat(buf,"</td><td>");
/* det ref */
    namfonct[LENNOM-2]=colnb;colnb++;numTf(buf,'s',&rdet[i],namfonct,2,0,0);
    namfonct[LENNOM-2]=colnb;   
    strcat(buf,"</td><td>\n");  
/* det dest */    
    namfonct[LENNOM-2]=colnb;colnb++;numTf(buf,'s',&det[i],namfonct,2,0,0);
    namfonct[LENNOM-2]=colnb;   
    strcat(buf,"</td>\n");  
/* mémo */
    char* mem="\0";
    if(memo[i]>=0 && memo[i]<NBMEMOS){mem=&memosTable[memo[i]*LMEMO];}
    alphaTableHtmlB(buf,mem,namfonct,LMEMO-1);

    strcat(buf,"</tr>\n");
    ethWrite(cli,buf);    // max len pour cli-print() 2048 ???
  }
  strcat(buf,"</table><br><input type=\"submit\" value=\"MàJ\"></fieldset></form>\n"); 

  ethWrite(cli,buf);
}


void periLineHtml(EthernetClient* cli,int i)
{
  char buf[2000];buf[0]='\0';
  int j;
  unsigned long begPL=millis();

  Serial.print("periLineHtml - periCur=");Serial.print(periCur);Serial.print("/");Serial.print(i);

  htmlIntroB(buf,nomserver,cli);
  pageHeader(buf);
  perifHeader(buf);
  usrPeriCurB(buf,"peri_cur__",0,2,3);
  ethWrite(cli,buf);

/* boutons */

    strcat(buf,"<table><tr><td>\n");

    boutRetourB(buf,"retour",0,0);strcat(buf," ");
    
    strcat(buf,"</td><td> <input type=\"submit\" value=\" MàJ \"> ");
    char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    boutF(buf,line,"","refresh",0,0,0,0);
    if(*periSwNb!=0){
      char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
      boutF(buf,swf,"","Switchs",0,0,0,0);};
    boutF(buf,"peri_raz___","","Raz",0,0,0,0);strcat(buf,"<br> \n");

    memcpy (line,"peri_tst__",LENNOM);line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    line[LENNOM-2]='0';boutF(buf,line,"","tst__SW0",0,0,0,0);
    line[LENNOM-2]='1';boutF(buf,line,"","tst__SW1",0,0,0,0);strcat(buf," ");
    line[LENNOM-2]='m';boutF(buf,line,"","tst_mail",0,0,0,0);strcat(buf,"\n");

    strcat(buf,"</td></tr></table>\n");
    
    ethWrite(cli,buf);

/* ligne périphérique */                

                periInitVar();periLoad(i);periCur=i;
                if(*periSwNb>MAXSW){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}

                strcat(buf,"<table><tr><th></th><th><br>periph_name</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th>._D_ _l<br>._i_ _e<br>._s_ _v</th><th>mac_addr<br>ip_addr</th><th>version Th<br>last out<br>last in</th></tr>\n<tr>\n<td>");
                      concatns(buf,periCur);
                      
                      alphaTableHtmlB(buf,periNamer,"peri_nom__",PERINAMLEN-1);
                      textTbl(buf,periLastVal_,periThmin_,periThmax_,1,1);
                      concatns(buf,*periThmin_);strcat(buf,"<br>");
                      concatns(buf,*periThmax_);
                      textTbl(buf,periAlim_,periVmin_,periVmax_,1,1);
                      numTf(buf,'I',periVmin_,"peri_vmin_",5,0,0);strcat(buf,"<br>\n");
                      numTf(buf,'I',periVmax_,"peri_vmax_",5,3,0);
                      numTf(buf,'d',(uint32_t*)periPerTemp,"peri_rtemp",5,2,0);strcat(buf,"<br>\n");
                      numTf(buf,'r',periPitch_,"peri_pitch",5,0,0);strcat(buf,"<br>\n");
                      numTf(buf,'r',periThOffset_,"peri_tofs_",5,3,0);
                      numTf(buf,'l',(uint32_t*)periPerRefr,"peri_refr_",5,2,0);strcat(buf,"<br>\n");
                      checkboxTableBHtml(buf,(uint8_t*)periProg,"peri_prog_",-1,3,"");
                      numTf(buf,'b',periSwNb,"peri_intnb",1,2,0);strcat(buf,"<br>\n");
                      numTf(buf,'b',periDetNb,"peri_detnb",1,3,0);
                      strcat(buf,"<td>");

                      char fonc[]={"peri_vsw__\0"},oi[]={"OI"};
                      for(int k=0;k<*periSwNb;k++){fonc[LENNOM-1]=PMFNCHAR+k;radioTableBHtml(buf,periSwCde(k),fonc,2);concat1a(buf,oi[periSwLev(k)]);if(k<*periSwNb-1){strcat(buf,"<br>");}strcat(buf,"\n");}
                      strcat(buf,"</td>");
                            
                      strcat(buf,"<td><input type=\"text\" name=\"peri_mac__\" value=\"");for(int k=0;k<6;k++){concat1a(buf,chexa[periMacr[k]/16]);concat1a(buf,chexa[periMacr[k]%16]);}
                      strcat(buf,"\" size=\"11\" maxlength=\"12\" ><br>\n");
                      if(*periProg!=0){strcat(buf,"port=");numTf(buf,'d',periPort,"peri_port_",4,0,0);}strcat(buf,"<br>\n");
                      strcat(buf,"<font size=\"2\">");for(j=0;j<4;j++){concatns(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}}strcat(buf,"</font></td>\n");
                      strcat(buf,"<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){concat1a(buf,periVers[j]);}
                      strcat(buf," \n");concat1a(buf,(char)*periProtocol);strcat(buf,"<br>\n");
                      
                      char colourbr[6];
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,colourbr);                      
                      concatDate(buf,periLastDateOut);
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,colourbr);                      
                      concatDate(buf,periLastDateIn);
                      setColourB(buf,"black");
                      strcat(buf,"</font></td>\n");
                      
                strcat(buf,"</tr></table>\n");
                ethWrite(cli,buf);

// table analogique
                
                strcat(buf,"Analog Input<br><table><tr><td>Val<br>Low<br>High</td><td>");
                      concatn(buf,*periAnal);strcat(buf,"<br>");
                      numTf(buf,'I',periAnalLow,"peri_ana@_",5,0,0);strcat(buf,"<br>");
                      numTf(buf,'I',periAnalHigh,"peri_anaA_",5,0,0);
                strcat(buf,"</td>\n<td>Off1<br>Fact<br>Off2</td>\n<td>");
                      numTf(buf,'I',periAnalOffset1,"peri_anaB_",5,0,0);strcat(buf,"<br>");                      
                      numTf(buf,'F',periAnalFactor,"peri_anaC_",7,0,0,4);strcat(buf,"<br>");
                      numTf(buf,'F',periAnalOffset2,"peri_anaD_",7,0,0,4);
                strcat(buf,"</td></tr></table><br></form>\n");
                ethWrite(cli,buf);
  
#define ANASIZLIB   3
                char aLibState[]={">H\0=H\0><\0=L\0-L\0"};
                subCbdet(buf,cli,0,"Analog Input Rules","rul_ana___",NBANST,aLibState,ANASIZLIB,NBRULOP,LENRULOP,rulop,periAnalCb,periAnalDestDet,periAnalRefDet,periAnalMemo);


#define DIGITSIZLIB 3
                char dLibState[MAXDET*DIGITSIZLIB];
                memset(dLibState,0x00,MAXDET*DIGITSIZLIB);    
                for(uint8_t k=0;k<*periDetNb;k++){
                  char oi[2]={'O','I'};
                  dLibState[k*DIGITSIZLIB]=oi[(*periDetVal>>(k*2))&DETBITLH_VB];
                  dLibState[i*DIGITSIZLIB+1]='_';}
                if(*periDetNb>0){
                  subCbdet(buf,cli,1,"Digital Inputs Rules","rul_dig___",*periDetNb,dLibState,DIGITSIZLIB,NBRULOP,LENRULOP,rulop,periDigitCb,periDigitDestDet,periDigitRefDet,periDigitMemo);
                }
                ethWrite(cli,buf);

                Serial.print(" dur=");Serial.println(millis()-begPL);
}


void showLine(char* buf,EthernetClient* cli,int numline,char* pkdate)
{
  unsigned long t0=micros();
  int j,s;
#define LBSHOWLINE 1000

      periInitVar();periLoad(numline);periCur=numline;
      if(*periSwNb>MAXSW){periInitVar();periSave(numline,PERISAVESD);}  
      if(*periDetNb>MAXDET){periInitVar();periSave(numline,PERISAVESD);}

          strcat(buf,"<tr>\n<form method=\"GET \"><td>");
          concatn(buf,periCur);
          strcat(buf,"\n<p hidden><input type=\"text\" name=\"user_ref_");
          concat1a(buf,(char)(usernum+PMFNCHAR));
          strcat(buf,"\" value=\"");
          concatn(buf,usrtime[usernum]);
          strcat(buf,"\">");
          char fonc[]="peri_cur__\0\0";concat1a(fonc,(char)(PMFNCHAR));
          numTf(buf,'i',&periCur,fonc,2,3,0);
          strcat(buf,"</p>\n");
/* th */          
          strcat(buf,"<td>");strcat(buf,periNamer);strcat(buf,"</td>");
          textTbl(buf,periLastVal_,periThmin_,periThmax_,1,1);
          concatnf(buf,(float)*periThmin_/100);strcat(buf,"<br>");
          concatnf(buf,(float)*periThmax_/100);strcat(buf,"</td>\n");                                
          textTbl(buf,periAlim_,periVmin_,periVmax_,1,1);          
          concatnf(buf,(float)*periVmin_/100);strcat(buf,"<br>\n");
          concatnf(buf,(float)*periVmax_/100);strcat(buf,"</td>");
/* pertemp/pitch/offset */         
          strcat(buf,"<td>");concatn(buf,*periPerTemp);strcat(buf,"<br>");
          concatnf(buf,(float)*periPitch_/100);strcat(buf,"<br>");
          concatnf(buf,(float)*periThOffset_/100);strcat(buf,"</td>");
          strcat(buf,"<td>");concatn(buf,*periPerRefr);strcat(buf,"<br>");
          if(*periProg!=0){strcat(buf,"serv");}strcat(buf,"</td>");         
          strcat(buf,"<td>");concatn(buf,*periSwNb);strcat(buf,"<br>");concatn(buf,*periDetNb);strcat(buf,"</td><td>");
          concat1aH(buf,(char)(*periSwVal));strcat(buf,"<br>");
          for(uint8_t k=0;k<*periSwNb;k++){
                      char oi[2]={'O','I'};concat1a(buf,oi[periSwCde(k)]);strcat(buf,"_");
                      concat1a(buf,oi[periSwLev(k)]);if(k<*periSwNb-1){strcat(buf,"<br>");}}
          strcat(buf,"</td><td>");
          strcat(buf,"<font size=\"2\">");
          for(uint8_t k=0;k<*periDetNb;k++){char oi[2]={'O','I'};concat1a(buf,oi[(*periDetVal>>(k*2))&DETBITLH_VB]);if(k<*periDetNb-1){strcat(buf,"<br>");}}
          strcat(buf,"</font></td>\n");
          strcat(buf,"<td>");
/* analog */          
          concatnf(buf,(float)(*periAnal+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2,4);strcat(buf,"<br>");
          concatnf(buf,(float)(*periAnalLow+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2,4);strcat(buf,"<br>");
          concatnf(buf,(float)(*periAnalHigh+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2,4);strcat(buf,"<br>");
          strcat(buf,"</td>");                    
/* mac */          
          strcat(buf,"<td>");                      
          for(int k=0;k<6;k++){concat1a(buf,chexa[periMacr[k]/16]);concat1a(buf,chexa[periMacr[k]%16]);}strcat(buf,"<br>");
          if(*periProg!=0 || *periProtocol=='U'){strcat(buf,"port=");concatn(buf,*periPort);}strcat(buf,"<br>");
          strcat(buf,"<font size=\"2\">");for(j=0;j<4;j++){concatn(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}}strcat(buf,"</font></td>\n");
          strcat(buf,"<td><font size=\"2\">");for(j=0;j<LENVERSION;j++){concat1a(buf,periVers[j]);}
          strcat(buf," ");long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
          char* a=protocStr+LENPROSTR*p;strcat(buf,a);strcat(buf,"<br>\n");
   
                      char colourbr[6];
                      long late;
                      late=*periPerRefr+*periPerRefr/10;
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateOut,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setCol(buf,colourbr);                      
          concatDate(buf,periLastDateOut);
                      memcpy(colourbr,"black\0",6);
                      if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
                      if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setCol(buf,colourbr);                      
          concatDate(buf,periLastDateIn);
                      setCol(buf,"black");                      
          strcat(buf,"</font></td><td>");                       
                      char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';                      
          boutF(buf,line,"","Periph",0,1,0,0);
                      
                      if(*periSwNb!=0){
                        char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
          boutF(buf,swf,"","Switchs",0,0,0,0);}                                    
          strcat(buf,"</form></tr>");

          ethWrite(cli,buf);
}
