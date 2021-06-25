#include <Arduino.h>
#include <SPI.h>      //bibliothéqe SPI pour W5100
#include <Ethernet.h>

#include "ds3231.h"
#include <shconst2.h>
#include <shutil2.h>
#include "const.h"
#include "periph.h"
#include "utilether.h"
#include "utilhtml.h"
#include "utiljs.h"
#include "pageshtml.h"

#include <MemoryFree.h>

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

extern uint16_t* periNum;                       // ptr ds buffer : Numéro du périphérique courant
extern long*     periPerRefr;                   // ptr ds buffer : période maximale accés au serveur
extern uint16_t* periPerTemp;                   // ptr ds buffer : période de lecture tempèrature
extern int16_t*  periPitch_;                    // ptr ds buffer : variation minimale de température pour datasave
extern int16_t*  periLastVal_;                  // ptr ds buffer : dernière valeur de température  
extern int16_t*  periAlim_;                     // ptr ds buffer : dernière tension d'alimentation
extern char*     periLastDateIn;                // ptr ds buffer : date/heure de dernière réception
extern char*     periLastDateOut;               // ptr ds buffer : date/heure de dernier envoi  
extern char*     periLastDateErr;               // ptr ds buffer : date/heure de derniere anomalie com
extern int8_t*   periErr;                       // ptr ds buffer : code diag anomalie com (voir MESSxxx shconst.h)
extern char*     periNamer;                     // ptr ds buffer : description périphérique
extern char*     periVers;                      // ptr ds buffer : version logiciel du périphérique
extern char*     periModel;                     // ptr ds buffer : model du périphérique
extern byte*     periMacr;                      // ptr ds buffer : mac address 
extern byte*     periIpAddr;                    // ptr ds buffer : Ip address
extern uint16_t* periPort;                      // ptr ds buffer : port periph server
extern byte*     periSwNb;                      // ptr ds buffer : Nbre d'interrupteurs (0 aucun ; maxi 4(MAXSW)            
extern byte*     periSwVal;                     // ptr ds buffer : état/cde des inter  
extern byte*     periInput;                     // ptr ds buffer : Mode fonctionnement inters (4 bytes par switch)           
extern uint32_t* periSwPulseOne;                // ptr ds buffer : durée pulses sec ON (0 pas de pulse)
extern uint32_t* periSwPulseTwo;                // ptr ds buffer : durée pulses sec OFF(mode astable)
extern uint32_t* periSwPulseCurrOne;            // ptr ds buffer : temps courant pulses ON
extern uint32_t* periSwPulseCurrTwo;            // ptr ds buffer : temps courant pulses OFF
extern byte*     periSwPulseCtl;                // ptr ds buffer : mode pulses 
extern byte*     periSwPulseSta;                // ptr ds buffer : état clock pulses
extern uint8_t*  periSondeNb;                   // ptr ds buffer : nbre sonde
extern boolean*  periProg;                      // ptr ds buffer : flag "programmable" 
extern byte*     periDetNb;                     // ptr ds buffer : Nbre de détecteurs maxi 4 (MAXDET)
extern byte*     periDetVal;                    // ptr ds buffer : flag "ON/OFF" si détecteur (2 bits par détec))
extern int16_t*  periThOffset_;                 // ptr ds buffer : offset correctif sur mesure température
extern int16_t*  periThmin_;                    // ptr ds buffer : alarme mini th
extern int16_t*  periThmax_;                    // ptr ds buffer : alarme maxi th
extern int16_t*  periVmin_;                     // ptr ds buffer : alarme mini volts
extern int16_t*  periVmax_;                     // ptr ds buffer : alarme maxi volts
extern byte*     periDetServEn;                 // ptr ds buffer : 1 byte 8*enable detecteurs serveur
extern byte*     periProtocol;                  // ptr ds buffer : protocole ('T'CP/'U'DP)
extern uint16_t* periAnal;                      // ptr ds buffer : analog value
extern uint16_t* periAnalLow;                   // ptr ds buffer : low analog value 
extern uint16_t* periAnalHigh;                  // ptr ds buffer : high analog value 
extern uint16_t* periAnalOffset1;               // ptr ds buffer : offset on adc value
extern float*    periAnalFactor;                // ptr ds buffer : factor to float for analog value
extern float*    periAnalOffset2;               // ptr ds buffer : offset on float value
extern uint8_t*  periAnalCb;                    // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*  periAnalDestDet;               // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*  periAnalRefDet;                // ptr ds buffer : 5 x n° détect serveur pour op logique (0xff si rien)
extern int8_t*   periAnalMemo;                  // ptr ds buffer : 5 x n° mémo dans table mémos
extern uint8_t*  periDigitCb;                   // ptr ds buffer : 5 x 4 bits pour checkbox
extern uint8_t*  periDigitDestDet;              // ptr ds buffer : 5 x n° détect serveur
extern uint8_t*  periDigitRefDet;               // ptr ds buffer : 4 x n° détect serveur pour op logique (0xff si rien)
extern int8_t*   periDigitMemo;                 // ptr ds buffer : 5 x n° mémo dans table mémos


extern int8_t    periMess;                      // code diag réception message (voir MESSxxx shconst.h)
extern byte      periMacBuf[6]; 

char inptyps[]="52meexphpu??";                  // libellés options types sources regles switchs
char optNam1[]={'S','\0'};                      // nom de la liste d'options
char inptypd[]="52meexswpu??";                  // libellés options types destinations regles switchs
char optNam2[]={'T','\0'};                      // nom de la liste d'options
char inpact[]={"@5     RAZ  STOP STARTSHORTEND  IMP  RESETXOR  OR   AND  NOR  NAND -0-  -1-       "};    // libellés options actions
char optNam3[]={'U','\0'};                      // nom de la liste d'options
char psps[]=  {"____IDLEEND1END2RUN1RUN2DISA"};                                                          // libellés staPulse

extern int  chge_pwd; //=FAUX;

extern int16_t valMin;
extern int16_t valMax;

extern byte mask[];

uint16_t ljs=0;

char rulop[]={"75     0    1    OR   AND  XOR  TO   "};  // options select ; libellés regles analog & digital 
char optNam0[]={'R','\0'};

char protoChar[]=PROTOCHAR;
char protocStr[]=PROTOCSTR;

char pkdate[7];

void showLine(char* buf,char* jsbuf,EthernetClient* cli,int i,char* pkdate,uint16_t* lb);

void perifHeader(char* buf,char* jsbuf)
{
  char* dm=buf+strlen(buf);

    concatn(buf,nullptr,periCur);strcat(buf,"-");
    strcat(buf,periNamer);strcat(buf," ");
    jscat(jsbuf,dm);
    strcat(buf,"<font size=\"2\">");
    dm=buf+strlen(buf);
    for(int j=0;j<4;j++){concatn(buf,periIpAddr[j]);if(j<3){strcat(buf,".");}};
    if(*periProg!=0){strcat(buf," / port=");concatn(buf,*periPort);strcat(buf,"  v");}
    char* db=buf+strlen(buf);
    db[0]=' ';db++;
    memcpy(db,periVers,LENVERSION);db[LENVERSION]='\0';
    jscat(jsbuf,dm);
    jscat(jsbuf,"\n");
    strcat(buf,"<br>\n");
}

void perifHeader(char* buf)
{
  perifHeader(buf,nullptr);
}

void subModePulseTime(char* buf,char* jsbuf,uint8_t npu,uint32_t* pulse,uint32_t* dur,char* fonc1,char* fonc2,char onetwo,uint8_t ctl)
{
  uint8_t pbit=PMTTE_PB;if(onetwo=='O'){pbit=PMTOE_PB;} pbit+=PCTLBIT*npu;
  uint8_t val=(((*(uint16_t*)periSwPulseCtl)>>pbit)&0x01)+PMFNCVAL;                                        
  //strcat(buf,"<font size=\"2\">");
  fonc1[LENNOM-1]=onetwo;
  checkboxTableBHtml(buf,jsbuf,&val,fonc1,NO_STATE,"",2,ctl&TDBEG);               // bit enable pulse
  if(*(pulse+npu)<0){*(pulse+npu)=0;}  
  numTf(buf,jsbuf,'l',(pulse+npu),fonc2,8,0,0,BRYES);                             // durée pulse   
//char a[11];sprintf(a,"%06u",(uint32_t)*dur);a[10]='\0';              // valeur courante 32bits=4G soit 10 chiffres
//strcat(buf,"<br>(");concatn(buf,*(dur+npu));strcat(buf,")</font>");
  uint32_t v=*(dur+npu);
  affNum(buf,jsbuf,'l',&v,0,2,(ctl&TDEND)|(ctl&BRYES));
}

void perinpBfnc(char* buf,char* jsbuf,uint8_t nuinp,uint16_t val,char type,uint8_t lmax,char* ft,uint8_t nuv,uint8_t ctl)      // type='c' checkbox ; 'n' num / ft fonct transport / nuv num var
{      // type input src, num det src,type input dest, num det dest, valeur, enable, action, 4*2 modes => 2 fonctions de transport avec 5 bits n°input (libf-2) et n°de paramètre (libf-1)                                                                          
  uint8_t vv=0;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);
  ft[LENNOM-1]=(char)(nuinp+PMFNCHAR);
  
  switch (type){
    case 'c':if(val!=0){vv=1;};checkboxTableBHtml(buf,jsbuf,&vv,ft,NO_STATE,"",0,ctl);break;
    case 'n':numTf(buf,jsbuf,'d',&val,ft,lmax,0,0,ctl);break;
    default: break;
  }
}


void swCtlTableHtml(EthernetClient* cli,int i)
{
  periCur=i;

  char jsbuf[8000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000,lb1=0;
  char buf[lb0];buf[0]='\0';

  unsigned long begTPage=millis();     // calcul durée envoi page

  Serial.print("swCtlTableHtml - periCur=");Serial.print(periCur);
  Serial.print("  free=");Serial.println(freeMemory(), DEC);

  htmlIntroB(buf,nomserver,cli);    // chargement CSS etc
  optSelHtml(jsbuf,inptyps,optNam1);
  optSelHtml(jsbuf,inptypd,optNam2);
  optSelHtml(jsbuf,inpact,optNam3);

  //usrPeriCurB(buf,jsbuf,"peri_t_sw_",0,2,0);
  formIntro(buf,jsbuf,"peri_t_sw_",0,0);         // params pour retours navigateur (n° usr + time usr + pericur + locfonc pour inits)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageHeader(buf,jsbuf);            // 1ère ligne page
  perifHeader(buf,jsbuf);           // 2nde ligne page
  
  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end 

/* boutons */
    boutRetourB(buf,jsbuf,"retour",TDBE);strcat(buf," ");
    formIntro(buf,jsbuf,nullptr,0,nullptr,0,0);
    //formBeg(buf,jsbuf);
    boutMaj(buf,jsbuf," MàJ ",0);
    //strcat(buf," <input type=\"submit\" value=\" MàJ \"> ");
    char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
    boutF(buf,jsbuf,swf,"","refresh",ALICNO,0,0);
    affSpace(buf,jsbuf);
    swf[LENNOM-2]='X';
    boutF(buf,jsbuf,swf,""," erase ",ALICNO,0,BRYES);

    ethWrite(cli,buf,&lb);

/* pulses */
    affText(buf,jsbuf,"Pulses",0,BRYES); 
    tableBeg(buf,jsbuf,TRBEG|TDBEG);
    //strcat(buf,"<tr><th></th><th>time One<br>time Two</th><th>free<br>run</th>");
    affText(buf,jsbuf," |time One~time Two|free~run",0,TDEND|TREND);

      char pfonc[]="peri_pto__\0";            // transporte la valeur pulse time One
      char qfonc[]="peri_ptt__\0";            // transporte la valeur pulse time Two
      char rfonc[]="peri_otfbv__\0";          // transporte les bits freerun et enable pulse de periPulseMode (LENNOM-1= ,'F','O','T')

      //strcat(buf,"<tr>");
      affText(buf,jsbuf,"",0,TRBEG);

      for(int pu=0;pu<NBPULSE;pu++){          // boucle des pulses

        //strcat(buf,"<td>");concatn(buf,pu);strcat(buf,"</td>");
        affNum(buf,jsbuf,'d',&pu,0,0,TDBE);

        pfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        qfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        rfonc[LENNOM-2]=(char)(pu+PMFNCHAR);        
      
        //strcat(buf,"<td>");
        subModePulseTime(buf,jsbuf,pu,periSwPulseOne,periSwPulseCurrOne,rfonc,pfonc,'O',TDBEG|BRYES);     // bit et valeur time one
        //strcat(buf,"<br>");
        subModePulseTime(buf,jsbuf,pu,periSwPulseTwo,periSwPulseCurrTwo,rfonc,qfonc,'T',TDEND);           // bit et valeur time two
      
        //strcat(buf,"</td><td> ");
        uint8_t val=(*(uint16_t*)periSwPulseCtl>>(PCTLBIT*pu+PMFRO_PB))&0x01;rfonc[LENNOM-1]='F';         // bit freerun
        checkboxTableBHtml(buf,jsbuf,&val,rfonc,NO_STATE,"",0,TDBEG|BRYES);                  
        //strcat(buf,"<br>");
        char ttsp[LENTSP+1];memcpy(ttsp,&(psps[periSwPulseSta[pu]*LENTSP]),LENTSP);ttsp[LENTSP]='\0';
        affText(buf,jsbuf,ttsp,0,TDEND);
        //for(int tsp=0;tsp<LENTSP;tsp++){concat1a(buf,psps[periSwPulseSta[pu]*LENTSP+tsp]);}strcat(buf,"</td>\n");         // staPulse 

        ethWrite(cli,buf,&lb);
      } // pulse suivant
      
      tableEnd(buf,jsbuf,TREND);
      formEnd(buf,jsbuf,0,0);
      ethWrite(cli,buf,&lb);

/* détecteurs */    
    detServHtml(cli,buf,jsbuf,&lb,lb0,&memDetServ,&libDetServ[0][0]);
    ethWrite(cli,buf,&lb);

/* affichage/saisie règles */
  affText(buf,jsbuf,"Règles en=enable, rf=(rise/fall if edge)(direct/inv if static) , pr=prev, es=edge/static ; follow srce 1001 -0- 1001 -1-",0,BRYES);
  tableBeg(buf,jsbuf,0);
  affText(buf,jsbuf,"|e.r p.e~n.f.r.s| source | destin.| action",0,TDBE|TRBE);
  ethWrite(cli,buf,&lb);

      char xfonc1[]="p_inp1____\0";

      uint16_t offsetInp=0;
      uint8_t ni=0;                 // nbre lignes ds buffer
      
      for(uint8_t ninp=0;ninp<NBPERINPUT;ninp++){     // boucle des regles

            ni++;
            
            char fnc[LENNOM+1];memcpy(fnc,"peri_inp__",LENNOM);fnc[LENNOM-1]=(char)(ninp+PMFNCHAR);fnc[LENNOM]='\0';
            formIntro(buf,jsbuf,fnc,0,0);

            uint8_t vv;
            byte binp[PERINPLEN];memcpy(binp,periInput+offsetInp,PERINPLEN);
            offsetInp+=PERINPLEN;

            affNum(buf,jsbuf,'s',&ninp,0,0,TRBEG|TDBE);
            //strcat(buf,"\n<tr><td>");concatn(buf,ninp);strcat(buf,"</td>");
            vv=(binp[2] & PERINPEN_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,1,TDBEG);              // bit enable
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPVALID_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,9,0);               // bit active level
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPOLDLEV_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,2,0);              // bit prev lev
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPDETES_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,3,TDEND);           // bit edge/static            
           
            vv=(binp[0]  & PERINPNT_MS);                                                                // type detec source
            selectTableBHtml(buf,jsbuf,inptyps,optNam1,xfonc1,vv,4,ninp,0,TDBEG);

            vv=(binp[0]>>PERINPNVLS_PB);perinpBfnc(buf,jsbuf,ninp,vv,'n',2,xfonc1,5,TDEND);              // num detec source
            strcat(buf,"\n");            

            vv=(binp[3]  & PERINPNT_MS);                                                                // type detec dest
            selectTableBHtml(buf,jsbuf,inptypd,optNam2,xfonc1,vv,6,ninp,0,TDBEG);

            vv=(binp[3]>>PERINPNVLS_PB);perinpBfnc(buf,jsbuf,ninp,vv,'n',2,xfonc1,7,TDEND);              // num detec  dest
            strcat(buf,"\n");            

            vv=(binp[2]&PERINPACT_MS)>>PERINPACTLS_PB;                                                  // action 
            selectTableBHtml(buf,jsbuf,inpact,optNam3,xfonc1,vv,8,ninp,0,TDBE);

            //strcat(buf,"<td><input type=\"submit\" value=\"MàJ\"></td>");
            boutMaj(buf,jsbuf,"Màj",TDBE);
                                                                                                        //strcat(buf, ajouter nom de la source si det
            //strcat(buf,"</form></tr>\n\n");
            formEnd(buf,jsbuf,0,TREND);

            lb1=strlen(buf);
            //Serial.print("lb/lb0/ni/lb0-lb/lb_ni+100 ");Serial.print(lb);Serial.print(" ");Serial.print(lb0);Serial.print(" ");Serial.print(ni);Serial.print(" ");Serial.print(lb0-lb);Serial.print(" ");Serial.println(lb/ni+100);
            if((lb0-lb1)<(lb1/ni+100)){ethWrite(cli,buf,&lb);ni=0;}
           
      }     // règle suivante

  tableEnd(buf,jsbuf,0);
  strcat(buf,"</body></html>");
  ethWrite(cli,buf,&lb);

bufLenShow(buf,jsbuf,lb,begTPage);
}

void periTableHtml(EthernetClient* cli)
{
  Serial.print("peritable ; remote_IP ");serialPrintIp(remote_IP_cur);Serial.println();

  char jsbuf[12000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  int i;
  unsigned long begTPage=millis();  // calcul durée envoi page

  int savePeriCur=periCur;          // restoré à la fin

  htmlIntroB(buf,nomserver,cli);    // chargement CSS etc

  //usrFormBHtml(buf,HID);
  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageHeader(buf,jsbuf);            // 1ère ligne page
  boutRetourB(buf,jsbuf,"refresh",0);  
  
  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end 

          fontBeg(buf,jsbuf,2,0);
          numTf(buf,jsbuf,'d',&perrefr,"per_refr__",4,0,0,0);
          
          affText(buf,jsbuf,"(",0,0);affNum(buf,jsbuf,'l',&fhsize,0,0,0);affText(buf,jsbuf,")",0,0);
          //strcat(buf,"(");concatn(buf,fhsize);strcat(buf,") ");
          
          numTf(buf,jsbuf,'i',(uint32_t*)&histoPos,"hist_sh___",9,0,0,0);
          alphaTableHtmlB(buf,jsbuf,histoDh,"hist_sh_D_",LDATEA-2,0,0);    
          boutMaj(buf,jsbuf,"ok",0);
      
          boutF(buf,jsbuf,"dump_his__","","histo",ALICNO,0,BRYES);

          //cli->print("<br>");
          boutF(buf,jsbuf,"deco______","","_  deco  _",ALICNO,0,0);
          boutF(buf,jsbuf,"deco_____B","","_ reboot _",ALICNO,0,0);          
          boutF(buf,jsbuf,"cfgserv___","","_ config _",ALICNO,0,0);
          boutF(buf,jsbuf,"remote____","","remote_cfg",ALICNO,0,0);
          boutF(buf,jsbuf,"remotehtml","","remotehtml",ALICNO,0,0);
          boutF(buf,jsbuf,"thermoscfg","","thermo_cfg",ALICNO,0,0);
          boutF(buf,jsbuf,"thermoshow","","thermoshow",ALICNO,0,0);
          boutF(buf,jsbuf,"timershtml","","timershtml",ALICNO,0,0);
          boutF(buf,"dsrvhtml__","","detsrvhtml",0,0,0,0);                 
        
          formEnd(buf,jsbuf,0,0);
          strcat(buf,"\n");

          detServHtml(cli,buf,jsbuf,&lb,lb0,&memDetServ,&libDetServ[0][0]);  // détecteurs serveur
          
          //strcat(buf,"<table><tr><th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>D_l<br>i_e<br>s_v</th><th></th><th>Analog<br>_low<br>_high</th><th>mac_addr<br>ip_addr</th><th>vers. prot<br>last out<br>last in</th><th></th></tr>");
          tableBeg(buf,jsbuf,TRBEG|TDBEG);
          affText(buf,jsbuf,"|~nom_periph|~TH|~  V |per_t~pth~ofs|per_s~ ~pg|nb~sw~det|D_l~i_e~s_v||Analog~_low~_high|mac_addr~ip_addr|vers. prot~last out~last in|",0,TREND|TDEND);

          ethWrite(cli,buf,&lb);

          for(i=1;i<=NBPERIF;i++){
                // !!!!!!!!!!!!!!!!!! pericur doit étre le premier de la liste !!!!!!!!!!!!!!!!!!!!!!!!!
                // pour permettre periLoad préalablement aux mises à jour des données quand le navigateur 
                // envoie une commande GET/POST 
                // et pour assurer l'effacement des bits de checkbox : le navigateur ne renvoie que ceux "checkés"
            showLine(buf,jsbuf,cli,i,pkdate,&lb);
            trigwd();
          }
          tableEnd(buf,jsbuf,0);
          htmlEnd(buf,jsbuf);
          ethWrite(cli,buf,&lb);
          
          periCur=savePeriCur;if(periCur>0){periLoad(periCur);}

bufLenShow(buf,jsbuf,lb,begTPage);
}

void subCbdet(char* buf,char* jsbuf,EthernetClient* cli,uint8_t nbfonc,const char* title,const char* nfonc,uint8_t nbLi,const char* lib,uint8_t libsize,uint8_t nbOp,uint8_t lenOp,char* rulOp,char* rulOptNam,uint8_t* cb,uint8_t* det,uint8_t* rdet,int8_t* memo,uint16_t* lb)
{                                 // le n° de fonction permet une seule fonction d'init pour plusieurs formulaires de même structure
  
  uint8_t k,op;
  char namfonct[LENNOM+1];memcpy(namfonct,nfonc,LENNOM);namfonct[LENNOM]='\0';
  char colnb=PMFNCHAR;

  //formBeg(buf,jsbuf,title);
  //strcat(buf,"<form><fieldset><legend>");strcat(buf,title);strcat(buf," :</legend>\n");
  //jscat(jsbuf,JSFB);strcat(jsbuf,title);strcat(jsbuf,";");
  char inifonc[LENNOM];memcpy(inifonc,nfonc,4);memcpy(inifonc+4,"init__",LENNOM-4);
  //usrPeriCurB(buf,inifonc,nbfonc,2,0);
  formIntro(buf,jsbuf,inifonc,0,title,2,0);
  
  tableBeg(buf,jsbuf,0);
  //strcat(buf,"<th></th><th>e.l. p.e<br>n.v. r.s</th><th> op </th><th>rdet</th><th>det</th></tr>\n");
  affText(buf,jsbuf,"|e.l. p.e~n.v. r.s| op |rdet|det",0,TRBE|TDBE);
  
  char bb[libsize+1];
  for(int i=0;i<nbLi;i++){    
    namfonct[LENNOM-1]=(char)(i+PMFNCHAR); // le dernier caractère est le n° de ligne ; l'avant dernier le n° de colonne
/* libellé ligne */
    *bb='\0';concatn(bb,i+1);strcat(bb," ");strcat(bb,(char*)(lib+i*libsize));
    affText(buf,jsbuf,bb,0,TRBEG|TDBE);

// checkbox 

    colnb=PMFNCHAR;
    #define CBNB 4
    for(uint8_t j=0;j<CBNB;j++){
      k=((*(cb+i)>>3)-j)&0x01;
      uint8_t cbtd=0;
      switch(j){
        case 0:cbtd=TDBEG;break;
        case CBNB-1:cbtd=TDEND;break;
        default: break;
      }
      namfonct[LENNOM-2]=(char)(colnb);checkboxTableBHtml(buf,jsbuf,&k,namfonct,NO_STATE,"",0,cbtd);
      if(j<(CBNB-1)){affSpace(buf,jsbuf);}
      colnb++;
    }      
    
// opé logique 
    op=(*(cb+i))>>4;
    selectTableBHtml(buf,jsbuf,rulOp,rulOptNam,namfonct,op,colnb-PMFNCHAR,i,0,TDBE);
    colnb++;
// det ref 
    namfonct[LENNOM-2]=colnb;colnb++;numTf(buf,jsbuf,'s',&rdet[i],namfonct,2,0,0,TDBE);
    namfonct[LENNOM-2]=colnb;   
    
// det dest 
    namfonct[LENNOM-2]=colnb;colnb++;numTf(buf,jsbuf,'s',&det[i],namfonct,2,0,0,TDBE);
    namfonct[LENNOM-2]=colnb;
// mémo 
    const char* mem="\0";
    if(memo[i]>=0 && memo[i]<NBMEMOS){mem=&memosTable[memo[i]*LMEMO];}
    alphaTableHtmlB(buf,jsbuf,mem,namfonct,LMEMO-1,0,TDBE|TREND);

    ethWrite(cli,buf,lb);
  }

  tableEnd(buf,jsbuf,BRYES);
  boutMaj(buf,jsbuf,"MàJ",0);
  formEnd(buf,jsbuf,TITLE,0,0);
  strcat(buf,"\n");
  //strcat(buf,"<input type=\"submit\" value=\"MàJ\"></fieldset></form>\n"); 

  ethWrite(cli,buf,lb);
}

void showDates(char* buf,char* jsbuf)
{
          char colourbr[6];
          #define LSTRD 14
          char strDate[LSTRD];
          long late;
          late=*periPerRefr+*periPerRefr/10;
          memcpy(colourbr,"black\0",6);
          if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
          if(dateCmp(periLastDateOut,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,jsbuf,colourbr);                      
          memset(strDate,0x00,LSTRD);
          concatDate(strDate,nullptr,periLastDateOut);
          affText(buf,jsbuf,strDate,0,BRYES);
          memcpy(colourbr,"black\0",6);
          if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
          if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,jsbuf,colourbr);                 
          memset(strDate,0x00,LSTRD);
          concatDate(strDate,nullptr,periLastDateIn);
          affText(buf,jsbuf,strDate,0,TDEND);
          setColourB(buf,jsbuf,"black");                      
          setColourE(buf,jsbuf);
}

void periLineHtml(EthernetClient* cli,int i)                // i=periCur
{
/* >>>>>>>>>>>>>>>>>>>>>>> modèle de page <<<<<<<<<<<<<<<<<<<<<<<<<<<<*/

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF2000;
  char buf[lb0];*buf=0x00;

  int j;
  unsigned long begTPage=millis();     // calcul durée envoi page

  Serial.print("periLineHtml - periCur=");Serial.print(periCur);Serial.print("/");Serial.print(i);

  htmlIntroB(buf,nomserver,cli);    // chargement CSS etc
  optSelHtml(jsbuf,rulop,optNam0);  // chargement tables params

  formIntro(buf,jsbuf,0,0);         // params pour retours navigateur (n° usr + time usr + pericur)
                                    // à charger au moins une fois par page ; pour les autres formulaires 
                                    // de la page formBeg() suffit

  pageHeader(buf,jsbuf);            // 1ère ligne page
  perifHeader(buf,jsbuf);           // 2nde ligne page
  
  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end 

/* boutons */

    //strcat(buf,"<table><tr>\n");
    tableBeg(buf,jsbuf,0);

    boutRetourB(buf,jsbuf,"retour",TDBE);strcat(buf," ");
    
    boutMaj(buf,jsbuf,"MàJ",TDBEG);
    char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    boutF(buf,jsbuf,line,"","refresh",ALICNO,0,0);
    if(*periSwNb!=0){
      char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
      boutF(buf,jsbuf,swf,"","Switchs",ALICNO,0,0);
    }
    affSpace(buf,jsbuf);
    char raz[]="peri_raz___";raz[LENNOM-1]=periCur+PMFNCHAR;
    boutF(buf,jsbuf,raz,"","Raz",ALICNO,0,BRYES);

    memcpy (line,"peri_tst__",LENNOM);line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    line[LENNOM-2]='0';boutF(buf,jsbuf,line,"","tst__SW0",ALICNO,0,0);
    line[LENNOM-2]='1';boutF(buf,jsbuf,line,"","tst__SW1",ALICNO,0,0);
    affSpace(buf,jsbuf);
    line[LENNOM-2]='m';boutF(buf,jsbuf,line,"","tst_mail",ALICNO,0,TREND|TDEND);
    tableEnd(buf,jsbuf,0);
    //strcat(buf,"</tr></table>\n");
    
    ethWrite(cli,buf,&lb);

/* ligne périphérique */                

                periInitVar();periLoad(i);periCur=i;
                if(*periSwNb>MAXSW){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(i,"perT");periInitVar();periSave(i,PERISAVESD);}

                tableBeg(buf,jsbuf,0);
                      //strcat(buf,"<tr><th></th><th><br>periph_name</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th>._D_ _l<br>._i_ _e<br>._s_ _v</th><th>mac_addr<br>ip_addr</th><th>version Th<br>last out<br>last in</th></tr><br>");   
                      affText(buf,jsbuf,"||~periph_name|~TH|~  V |per_t~pth~ofs|per_s~ ~pg|nb~sw~det|._D_ _l~._i_ _e~._s_ _v|mac_addr~ip_addr|version Th~last out~last in",0,TRBE);
                      affNum(buf,jsbuf,'d',&periCur,0,0,TDBE);
                      alphaTableHtmlB(buf,jsbuf,periNamer,"peri_nom__",PERINAMLEN-1,0,TRNO|TDBE|BRNO);                    
                      affNum(buf,jsbuf,periLastVal_,periThmin_,periThmax_,BRYES|TDBEG);            
                      float ff=((float)*periThmin_)/100;
                      affNum(buf,jsbuf,'f',&ff,2,0,BRYES);     
                      ff=((float)*periThmax_)/100;
                      affNum(buf,jsbuf,'f',&ff,2,0,TDEND);
                      affNum(buf,jsbuf,periAlim_,periVmin_,periVmax_,BRYES|TDBEG);
                      numTf(buf,jsbuf,'I',periVmin_,"peri_vmin_",5,0,0,BRYES);
                      numTf(buf,jsbuf,'I',periVmax_,"peri_vmax_",5,0,0,TDEND);
                      numTf(buf,jsbuf,'d',(uint32_t*)periPerTemp,"peri_rtemp",5,2,0,BRYES|TDBEG);
                      numTf(buf,jsbuf,'r',periPitch_,"peri_pitch",5,2,0,BRYES);
                      numTf(buf,jsbuf,'r',periThOffset_,"peri_tofs_",5,3,0,TDEND);
                      numTf(buf,jsbuf,'l',(uint32_t*)periPerRefr,"peri_refr_",5,2,0,TDBEG|BRYES);
                      checkboxTableBHtml(buf,jsbuf,(uint8_t*)periProg,"peri_prog_",NO_STATE,TDEND,"");
                      numTf(buf,jsbuf,'b',periSwNb,"peri_intnb",1,2,0,TDBEG|BRYES);
                      numTf(buf,jsbuf,'b',periDetNb,"peri_detnb",1,3,0,TDEND);
                      char fonc[]={"peri_vsw__\0"},oi[]={"OI"};
                      uint8_t lctl=TDBEG;
                      for(int k=0;k<*periSwNb;k++){fonc[LENNOM-1]=PMFNCHAR+k;radioTableBHtml(buf,jsbuf,periSwCde(k),fonc,2,0,lctl);
                        lctl=0;if(k<*periSwNb-1){lctl=BRYES;}
                        affText(buf,jsbuf,&oi[periSwLev(k)],1,0,lctl|STRING|CONCAT);
                        lctl=0;}
                      strcat(buf,"\n");

                      char bb[LENNOM];bb[0]='\0';
                      for(int k=0;k<6;k++){concat1a(bb,chexa[periMacr[k]/16]);concat1a(bb,chexa[periMacr[k]%16]);}
                      alphaTableHtmlB(buf,jsbuf,bb,"peri_mac__",12,0,TDBEG|BRYES);      
                      affText(buf,jsbuf,"port=",0,0);
                      numTf(buf,jsbuf,'d',periPort,"peri_port_",4,0,0,0,BRYES);
                      affText(buf,jsbuf,(char*)"",2,0);       // set police 2
                      #define LIP 16
                      char bc[LIP];memset(bc,'\0',LIP);
                      for(j=0;j<4;j++){concatns(bc,periIpAddr[j]);if(j<3){strcat(bc,".");}}
                      affText(buf,jsbuf,bc,0,TDEND);
                      fontEnd(buf,jsbuf,TDEND);
                      affText(buf,jsbuf,(char*)"",2,TDBEG);   // set police 2
                      char bd[LENVERSION+1];memcpy(bd,periVers,LENVERSION);bd[LENVERSION]='\0';
                      affText(buf,jsbuf,bd,0,0);
                      *bd=*periProtocol;*(bd+1)='\0';
                      affText(buf,jsbuf,bd,0,BRYES);
                      showDates(buf,jsbuf);
/*                    char colourbr[6];
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}
                      setColourB(buf,jsbuf,colourbr);                      
                      concatDate(buf,jsbuf,periLastDateOut);
                      affText(buf,jsbuf," ",0,BRYES);
                      memcpy(colourbr,"black\0",6);if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"red\0",4);}
                      setColourB(buf,jsbuf,colourbr);                      
                      concatDate(buf,jsbuf,periLastDateIn);
 */                     
                      fontEnd(buf,jsbuf,TDEND);
                      
                tableEnd(buf,jsbuf,TREND|BRYES);
                //setColourB(buf,jsbuf,"black");
                ethWrite(cli,buf,&lb);

// table analogique
                
                affText(buf,jsbuf,"Analog Input",0,BRYES);
                tableBeg(buf,jsbuf,TRBEG);
                affText(buf,jsbuf,"val~Low~High",0,TDBE);
                affNum(buf,jsbuf,(int16_t*)periAnal,&valMin,&valMax,BRYES|TDBEG);
                numTf(buf,jsbuf,'I',periAnalLow,"peri_ana@",5,0,0,BRYES);
                numTf(buf,jsbuf,'I',periAnalHigh,"peri_anaA",5,0,0,NOBR|TDEND);
                  //strcat(buf,"\n<td>Off1<br>Fact<br>Off2</td>\n<td>");
                affText(buf,jsbuf,"Off1~Fact~Off2",0,TDBE);
                numTf(buf,jsbuf,'I',periAnalOffset1,"peri_anaB_",5,0,0,TDBEG|BRYES);
                      //numTf(buf,'I',periAnalOffset1,"peri_anaB_",5,0,0);strcat(buf,"<br>");
                      //strcat(buf,"<br>");jscat(jsbuf,JSBR);                    
                numTf(buf,jsbuf,'F',periAnalFactor,"peri_anaC_",7,0,0,4,BRYES);
                      //numTf(buf,'F',periAnalFactor,"peri_anaC_",7,0,0,4);                        
                numTf(buf,jsbuf,'F',periAnalOffset2,"peri_anaD_",7,0,0,4,TDEND|TREND);
                      //numTf(buf,'F',periAnalOffset2,"peri_anaD_",7,0,0,4);                      
                tableEnd(buf,jsbuf,BRYES);
                formEnd(buf,jsbuf,0,0);
                
                ethWrite(cli,buf,&lb);
  
#define ANASIZLIB   3
                char aLibState[]={">H\0=H\0><\0=L\0-L\0"};
                subCbdet(buf,jsbuf,cli,0,"Analog Input Rules","rul_ana___",NBANST,aLibState,ANASIZLIB,NBRULOP,LENRULOP,rulop,optNam0,periAnalCb,periAnalDestDet,periAnalRefDet,periAnalMemo,&lb);

// table digitale

#define DIGITSIZLIB 3
                char dLibState[MAXDET*DIGITSIZLIB];
                memset(dLibState,0x00,MAXDET*DIGITSIZLIB);    
                for(uint8_t k=0;k<*periDetNb;k++){
                  char oi[2]={'O','I'};
                  dLibState[k*DIGITSIZLIB]=oi[(*periDetVal>>(k*2))&DETBITLH_VB];
                  dLibState[i*DIGITSIZLIB+1]='_';}
                if(*periDetNb>0){
                  subCbdet(buf,jsbuf,cli,1,"Digital Inputs Rules","rul_dig___",*periDetNb,dLibState,DIGITSIZLIB,NBRULOP,LENRULOP,rulop,optNam0,periDigitCb,periDigitDestDet,periDigitRefDet,periDigitMemo,&lb);
                }
                
                htmlEnd(buf,jsbuf);
                ethWrite(cli,buf,&lb);

                Serial.print("  len buf=");Serial.print(lb);
                Serial.print("  len jsbuf=");Serial.print(strlen(jsbuf));
                Serial.print("  ms=");Serial.println(millis()-begTPage);
}

void showLine(char* buf,char* jsbuf,EthernetClient* cli,int numline,char* pkdate,uint16_t* lb)
{
  //Serial.print("showLine ");Serial.println(numline);
  float vv=0;
//#define LBSHOWLINE 1000
      periInitVar();periLoad(numline);periCur=numline;
      if(*periSwNb>MAXSW){periInitVar();periSave(numline,PERISAVESD);}  
      if(*periDetNb>MAXDET){periInitVar();periSave(numline,PERISAVESD);}

/* line form header */
          formIntro(buf,jsbuf,0,TRBEG);
/* pericur - nom - th - volts */
          //char* dm;dm=jsbuf+strlen(jsbuf);
          uint8_t lctl=STRING|TRBEG|TDBE;

          affNum(buf,jsbuf,'d',&periCur,0,0,lctl);
          affText(buf,jsbuf,periNamer,0,STRING);
          affNum(buf,jsbuf,periLastVal_,periThmin_,periThmax_,TDBEG|BRYES);
          vv=*periThmin_/100;affNum(buf,jsbuf,'f',&vv,2,0,BRYES);
          vv=*periThmax_/100;affNum(buf,jsbuf,'f',&vv,2,0,TDEND);
          affNum(buf,jsbuf,periAlim_,periVmin_,periVmax_,TDBEG|BRYES);          
          vv=*periVmin_/100;affNum(buf,jsbuf,'f',&vv,2,0,BRYES);
          vv=*periVmax_/100;affNum(buf,jsbuf,'f',&vv,2,0,TDEND);               
// pertemp/pitch/offset 
          affNum(buf,jsbuf,'d',periPerTemp,0,0,STRING|TDBEG|BRYES);
          vv=*periPitch_/100;affNum(buf,jsbuf,'f',&vv,2,0,STRING|BRYES);
          vv=*periThOffset_/100;affNum(buf,jsbuf,'f',&vv,2,0,STRING|TDEND);
      
          char oi[2]={'O','I'};
          if(*periProg==0){lctl=STRING|TDEND;}
          else {lctl=STRING|BRYES;}
          affNum(buf,jsbuf,'d',(uint16_t*)periPerRefr,0,0,lctl);
          if(*periProg!=0){affText(buf,jsbuf,"serv",0,STRING|TDEND);}
            
          affNum(buf,jsbuf,'s',(uint8_t*)periSwNb,0,0,STRING|BRYES);
          affNum(buf,jsbuf,'s',(uint8_t*)periDetNb,0,0,STRING|TDEND);                                                 
          //concat1aH(buf,(char)(*periSwVal));strcat(buf,"<br>");
          
          char tt[3];
          for(uint8_t k=0;k<*periSwNb;k++){
                      tt[0]=oi[periSwCde(k)];tt[1]='_';tt[2]=oi[periSwLev(k)];
                      //affText(buf,jsbuf,&oi[periSwCde(k)],1,0,lctl);affText(buf,jsbuf,"_",1,0,0);
                      //lctl=0;
                      if(k<*periSwNb-1){lctl=STRING|BRYES;}
                      else {lctl=STRING|TDEND;}
                      //affText(buf,jsbuf,&oi[periSwLev(k)],1,0,lctl);
                      affText(buf,jsbuf,tt,3,0,lctl);
          }
          if(*periSwNb==0){affText(buf,jsbuf," ",0,STRING|TDEND);}          
          for(uint8_t k=0;k<*periDetNb;k++){
            if(k<*periDetNb-1){lctl=STRING|BRYES;}
            else lctl=STRING|TDEND;
            affText(buf,jsbuf,&oi[(*periDetVal>>(k*2))&DETBITLH_VB],1,0,lctl);
          }
          if(*periDetNb==0){affText(buf,jsbuf," ",0,STRING|TDEND);}
// analog 
          vv=(*periAnal+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;affNum(buf,jsbuf,'f',&vv,4,0,STRING|BRYES);
          vv=(*periAnalLow+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;affNum(buf,jsbuf,'f',&vv,4,0,STRING|BRYES);
          vv=(*periAnalHigh+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;affNum(buf,jsbuf,'f',&vv,4,0,STRING|TDEND);
          strcat(buf,"\n");
// mac                    
          lctl=TDBEG;
          char m[18];m[17]=0x00;
          for(int k=0;k<6;k++){
            m[k*3]=chexa[periMacr[k]/16];
            m[k*3+1]=chexa[periMacr[k]%16];
            if(k<5){m[k*3+2]='.';}
          }
          affText(buf,jsbuf,m,0,STRING|BRYES);

          if(*periProg!=0 || *periProtocol=='U'){
            affText(buf,jsbuf,"port=",0,STRING|CONCAT);affNum(buf,jsbuf,'d',periPort,0,0,STRING|BRYES);}
          else{affText(buf,jsbuf,"",0,STRING|BRYES);}
// IP addr
          char w[16];memset(w,0x00,16);
          uint8_t s=0;
          for(uint8_t j=0;j<4;j++){
            s+=sprintf(w+s,"%hu",(uint8_t)periIpAddr[j]);
            if(j<3){w[s]='.';s++;}
          }
          affText(buf,jsbuf,w,0,STRING|TDEND);
// version protocol
          char vers[LENVERSION+1];memcpy(vers,periVers,LENVERSION);vers[LENVERSION]='\0';
          affText(buf,jsbuf,vers,0,STRING|CONCAT);affText(buf,jsbuf," ",0,STRING|CONCAT);
          long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
          char* a=protocStr+LENPROSTR*p;affText(buf,jsbuf,a,0,STRING|CONCAT|BRYES);                
// date_heures
          showDates(buf,jsbuf);
/*          char colourbr[6];
          #define LSTRD 14
          char strDate[LSTRD];
          long late;
          late=*periPerRefr+*periPerRefr/10;
          memcpy(colourbr,"black\0",6);
          if(dateCmp(periLastDateOut,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
          if(dateCmp(periLastDateOut,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,jsbuf,colourbr);                      
          memset(strDate,0x00,LSTRD);
          concatDate(strDate,nullptr,periLastDateOut);
          affText(buf,jsbuf,strDate,0,BRYES);
          memcpy(colourbr,"black\0",6);
          if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
          if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,jsbuf,colourbr);                 
          memset(strDate,0x00,LSTRD);
          concatDate(strDate,nullptr,periLastDateIn);
          affText(buf,jsbuf,strDate,0,TDEND);
          setColourB(buf,jsbuf,"black");                      
          setColourE(buf,jsbuf);
*/              
          strcat(buf,"\n");
// boutons
          char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';                      
          boutF(buf,jsbuf,line,"","Periph",ALICNO,0,TDBEG|BRYES);                         // bouton periph
          if(*periSwNb!=0){char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
            boutF(buf,jsbuf,swf,"","Switchs",ALICNO,0,TDEND);}                            // bouton switchs
          strcat(buf,"\n");                                    
// fin
          formEnd(buf,jsbuf,0,TREND);
          strcat(buf,"\n");

          ethWrite(cli,buf,lb);         //Serial.print("lb=");Serial.println(*lb);
//if(*periDetNb!=0){Serial.print("\n====================");Serial.println(dm);}          
}
