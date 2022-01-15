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
extern char*     serverName;
extern uint32_t  memDetServ;  // image mémoire NBDSRV détecteurs
extern char      libDetServ[NBDSRV][LENLIBDETSERV];
extern uint16_t  sourceDetServ();

extern uint16_t  perrefr;

extern char*     peripass;            // mot de passe périphériques
extern char*     usrnames;            // usernames

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
extern bool*     periProg;                      // ptr ds buffer : flag "programmable" 
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
extern byte*     periSsidNb;                    // ptr ds buffer : n° dernier ssid utilisé

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
  scrGetCheckbox(buf,jsbuf,&val,fonc1,NO_STATE,"",2,ctl&TDBEG);               // bit enable pulse
  if(*(pulse+npu)<0){*(pulse+npu)=0;}  
  scrGetNum(buf,jsbuf,'g',(pulse+npu),fonc2,8,0,0,BRYES);                     // durée pulse   
  scrDspText(buf,jsbuf,"(",0,0);
  //uint32_t v=*(dur+npu);
  scrDspNum(buf,jsbuf,'g',(dur+npu),0,0,0);
  scrDspText(buf,jsbuf,")",0,(ctl&TDEND)|(ctl&BRYES));
  Serial.print(">>>>>>>>>>>>>>>>><");Serial.print(*(pulse+npu));Serial.print(" ");Serial.print(*(dur+npu));Serial.print(" ");Serial.println(npu);
}

void perinpBfnc(char* buf,char* jsbuf,uint8_t nuinp,uint16_t val,char type,uint8_t lmax,char* ft,uint8_t nuv,uint8_t ctl)      // type='c' checkbox ; 'n' num / ft fonct transport / nuv num var
{      // type input src, num det src,type input dest, num det dest, valeur, enable, action, 4*2 modes => 2 fonctions de transport avec 5 bits n°input (libf-2) et n°de paramètre (libf-1)                                                                          
  uint8_t vv=0;

  ft[LENNOM-2]=(char)(nuv+PMFNCHAR);
  ft[LENNOM-1]=(char)(nuinp+PMFNCHAR);
  
  switch (type){
    case 'c':if(val!=0){vv=1;};scrGetCheckbox(buf,jsbuf,&vv,ft,NO_STATE,"",0,ctl);break;
    case 'n':scrGetNum(buf,jsbuf,'d',&val,ft,lmax,0,0,ctl);break;
    default: break;
  }
}


void swCtlTableHtml(EthernetClient* cli)
{
  char jsbuf[10000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000,lb1=0;
  char buf[lb0];buf[0]='\0';

  unsigned long begTPage=millis();        // calcul durée envoi page

  Serial.print("swCtlTableHtml - periCur=");Serial.print(periCur);
  Serial.print("  free=");Serial.println(freeMemory(), DEC);

  htmlBeg(buf,jsbuf,serverName);          // chargement CSS etc

  optSelHtml(jsbuf,inptyps,optNam1);
  optSelHtml(jsbuf,inptypd,optNam2);
  optSelHtml(jsbuf,inpact,optNam3);

  formIntro(buf,jsbuf,"peri_t_sw_",0,0);  // n° usr + time usr + pericur + locfonc pour inits

  pageLineOne(buf,jsbuf);                 // 1ère ligne page
  perifHeader(buf,jsbuf);                 // 2nde ligne page
  
  ethWrite(cli,buf,&lb);                  // tfr -> navigateur
// ------------------------------------------------------------- header end 

/* boutons */
    scrGetButRet(buf,jsbuf,"retour",TDBE);
    affSpace(buf,jsbuf);
    scrGetButSub(buf,jsbuf," MàJ ",0);

    char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
    scrGetButFn(buf,jsbuf,swf,"","refresh",ALICNO,0,0);
    affSpace(buf,jsbuf);
    swf[LENNOM-2]='X';
    scrGetButFn(buf,jsbuf,swf,""," erase ",ALICNO,0,BRYES);

    ethWrite(cli,buf,&lb);

/* pulses */
    scrDspText(buf,jsbuf,"Pulses",0,BRYES); 
    tableBeg(buf,jsbuf,TRBEG|TDBEG);
    scrDspText(buf,jsbuf," |time One~time Two|free~run",0,TDEND|TREND);

      char pfonc[]="peri_pto__\0";            // transporte la valeur pulse time One
      char qfonc[]="peri_ptt__\0";            // transporte la valeur pulse time Two
      char rfonc[]="peri_otf__\0";            // transporte les bits freerun et enable pulse de periPulseMode (LENNOM-1= ,'F','O','T')

      scrDspText(buf,jsbuf,"",0,TRBEG);

      for(uint8_t pu=0;pu<NBPULSE;pu++){          // boucle des pulses
        scrDspNum(buf,jsbuf,'D',&pu,0,0,TDBE);

        pfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        qfonc[LENNOM-2]=(char)(pu+PMFNCHAR);
        rfonc[LENNOM-2]=(char)(pu+PMFNCHAR);        
      
        subModePulseTime(buf,jsbuf,pu,periSwPulseOne,periSwPulseCurrOne,rfonc,pfonc,'O',TDBEG|BRYES);     // bit et valeur time one
        subModePulseTime(buf,jsbuf,pu,periSwPulseTwo,periSwPulseCurrTwo,rfonc,qfonc,'T',TDEND);           // bit et valeur time two
        uint8_t val=(*(uint16_t*)periSwPulseCtl>>(PCTLBIT*pu+PMFRO_PB))&0x01;rfonc[LENNOM-1]='F';         // bit freerun
        scrGetCheckbox(buf,jsbuf,&val,rfonc,NO_STATE,"",0,TDBEG|BRYES);                  
        char ttsp[LENTSP+1];memcpy(ttsp,&(psps[periSwPulseSta[pu]*LENTSP]),LENTSP);ttsp[LENTSP]='\0';
        scrDspText(buf,jsbuf,ttsp,0,TDEND);

        ethWrite(cli,buf,&lb);
      } // pulse suivant
      
      tableEnd(buf,jsbuf,TREND);
      formEnd(buf,jsbuf,0,0);
      ethWrite(cli,buf,&lb);

/* détecteurs */    
    detServHtml(cli,buf,jsbuf,&lb,lb0,&memDetServ,&libDetServ[0][0]);
    ethWrite(cli,buf,&lb);

/* affichage/saisie règles */
        
  scrDspText(buf,jsbuf,"la fonction utilise la valeur courante et la source pour produire la destination et la nlle valeur courante",0,BRYES);
  scrDspText(buf,jsbuf,"Règles en=enable, rf=(fall/rise if edge)(direct/inv if static) , pr=prev, es=edge/static ; follow srce 1001 -0- 1001 -1-",0,BRYES);
  scrDspText(buf,jsbuf,";'static' retourne la valeur de la source ; 'edge' retourne 1 sur le flanc actif sinon 0",0,BRYES);
   
  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf,"|e...r...p...e~n...f...r...s| source | destin.| action",0,TDBE|TRBE);
  ethWrite(cli,buf,&lb);

      char xfonc1[]="p_inp1____\0";

      uint16_t offsetInp=0;
      uint8_t ni=0;                 // nbre lignes ds buffer
      
      for(uint8_t ninp=0;ninp<NBPERINPUT;ninp++){     // boucle des regles

            ni++;
            
            char fnc[LENNOM+1];memcpy(fnc,"peri_inp__",LENNOM);fnc[LENNOM-1]=(char)(ninp+PMFNCHAR);fnc[LENNOM]='\0';
            formIntro(buf,jsbuf,fnc,ninp,0,0);

            uint8_t vv;
            byte binp[PERINPLEN];memcpy(binp,periInput+offsetInp,PERINPLEN);
            offsetInp+=PERINPLEN;

            scrDspNum(buf,jsbuf,'s',&ninp,0,0,TRBEG|TDBE);
            vv=(binp[2] & PERINPEN_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,1,TDBEG);              // bit enable
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPVALID_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,9,0);               // bit active level
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPOLDLEV_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,2,0);              // bit prev lev
            affSpace(buf,jsbuf);
            vv=(binp[2] & PERINPDETES_VB);perinpBfnc(buf,jsbuf,ninp,vv,'c',1,xfonc1,3,TDEND);           // bit edge/static            
           
            vv=(binp[0]  & PERINPNT_MS);                                                                // type detec source
            scrGetSelect(buf,jsbuf,inptyps,optNam1,xfonc1,vv,4,ninp,0,TDBEG);

            vv=(binp[0]>>PERINPNVLS_PB);perinpBfnc(buf,jsbuf,ninp,vv,'n',2,xfonc1,5,TDEND);             // num detec source
            strcat(buf,"\n");            

            vv=(binp[3]  & PERINPNT_MS);                                                                // type detec dest
            scrGetSelect(buf,jsbuf,inptypd,optNam2,xfonc1,vv,6,ninp,0,TDBEG);

            vv=(binp[3]>>PERINPNVLS_PB);perinpBfnc(buf,jsbuf,ninp,vv,'n',2,xfonc1,7,TDEND);             // num detec  dest
            strcat(buf,"\n");            

            vv=(binp[2]&PERINPACT_MS)>>PERINPACTLS_PB;                                                  // action 
            scrGetSelect(buf,jsbuf,inpact,optNam3,xfonc1,vv,8,ninp,0,TDBE);

            scrGetButSub(buf,jsbuf,"Màj",TDBE);
                                                                                                        //strcat(buf, ajouter nom de la source si det
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
  Serial.print("peritable ; remote_IP ");serialPrintIp(remote_IP_cur);Serial.print(" fhsize=");Serial.println(fhsize);

  char jsbuf[12000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF4000;
  char buf[lb0];*buf=0x00;

  int i;
  //unsigned long begTPage=millis();  // calcul durée envoi page

  int savePeriCur=periCur;          // restoré à la fin

  htmlBeg(buf,jsbuf,serverName);     // chargement CSS etc

  //usrFormBHtml(buf,HID);
  formIntro(buf,jsbuf,0,0);         // (n° usr + time usr + pericur)

  pageLineOne(buf,jsbuf);            // 1ère ligne page
  scrGetButRet(buf,jsbuf,"refresh",0);  
  
  ethWrite(cli,buf,&lb);            // tfr -> navigateur
// ------------------------------------------------------------- header end 

          fontBeg(buf,jsbuf,2,0);
          scrGetNum(buf,jsbuf,'d',&perrefr,"per_refr__",4,0,0,0);
          scrDspText(buf,jsbuf,"(",0,0);scrDspNum(buf,jsbuf,'l',&fhsize,0,0,0);scrDspText(buf,jsbuf,")",0,0);
          
          scrGetNum(buf,jsbuf,'i',(uint32_t*)&histoPos,"hist_sh___",9,0,0,0);
          scrGetText(buf,jsbuf,histoDh,"hist_sh_D_",LDATEA-2,0,0);    
          scrGetButSub(buf,jsbuf,"ok",0);
      
          scrGetButFn(buf,jsbuf,"dump_his__","","histo",ALICNO,0,BRYES);

          //cli->print("<br>");
          scrGetButFn(buf,jsbuf,"deco______","","_  deco  _",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"deco_____B","","_ reboot _",ALICNO,0,0);          
          scrGetButFn(buf,jsbuf,"cfgserv___","","_ config _",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"remote____","","remote_cfg",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"remotehtml","","remotehtml",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"thermoscfg","","thermo_cfg",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"thermoshow","","thermoshow",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"timershtml","","timershtml",ALICNO,0,0);
          scrGetButFn(buf,jsbuf,"dsrvhtml__","","detsrvhtml",ALICNO,0,BRYES);                 
        
          formEnd(buf,jsbuf,0,0);
          strcat(buf,"\n");

          detServHtml(cli,buf,jsbuf,&lb,lb0,&memDetServ,&libDetServ[0][0]);  // détecteurs serveur
          
          //strcat(buf,"<table><tr><th></th><th><br>nom_periph</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th><br>D_l<br>i_e<br>s_v</th><th></th><th>Analog<br>_low<br>_high</th><th>mac_addr<br>ip_addr</th><th>vers. prot<br>last out<br>last in</th><th></th></tr>");
          tableBeg(buf,jsbuf,"Courier, sans-serif\"",BORDER,TRBEG|TDBEG);
          scrDspText(buf,jsbuf,"|~nom_periph|~TH|~  V |per_t~pth~ofs|per_s~ ~pg|nb~sw~det|D_l~i_e~s_v||Analog~_low~_high|mac_addr~ip_addr|ver ssid prot~last out~last in|",0,TREND|TDEND);
          
          strcat(buf,"\n\n");

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

//bufLenShow(buf,jsbuf,lb,begTPage);
}

void subCbdet(char* buf,char* jsbuf,EthernetClient* cli,uint8_t nbfonc,const char* title,const char* nfonc,uint8_t nbLi,const char* lib,uint8_t libsize,uint8_t nbOp,uint8_t lenOp,char* rulOp,char* rulOptNam,uint8_t* cb,uint8_t* det,uint8_t* rdet,int8_t* memo,uint16_t* lb)
{                                 // le n° de fonction permet une seule fonction d'init pour plusieurs formulaires de même structure
  
  uint8_t k,op;
  char namfonct[LENNOM+1];memcpy(namfonct,nfonc,LENNOM);namfonct[LENNOM]='\0';
  char colnb=PMFNCHAR;

  char inifonc[LENNOM];memcpy(inifonc,nfonc,4);memcpy(inifonc+4,"init__",LENNOM-4);

  formIntro(buf,jsbuf,inifonc,0,title,2,0);
  
  tableBeg(buf,jsbuf,0);
  scrDspText(buf,jsbuf,"|e.l. p.e~n.v. r.s| op |rdet|det",0,TRBE|TDBE);
  
  char bb[libsize+1];
  for(int i=0;i<nbLi;i++){    
    namfonct[LENNOM-1]=(char)(i+PMFNCHAR); // le dernier caractère est le n° de ligne ; l'avant dernier le n° de colonne
/* libellé ligne */
    *bb='\0';concatn(bb,i+1);strcat(bb," ");strcat(bb,(char*)(lib+i*libsize));
    scrDspText(buf,jsbuf,bb,0,TRBEG|TDBE);

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
      namfonct[LENNOM-2]=(char)(colnb);scrGetCheckbox(buf,jsbuf,&k,namfonct,NO_STATE,"",0,cbtd);
      if(j<(CBNB-1)){affSpace(buf,jsbuf);}
      colnb++;
    }      
    
// opé logique 
    op=(*(cb+i))>>4;
    scrGetSelect(buf,jsbuf,rulOp,rulOptNam,namfonct,op,colnb-PMFNCHAR,i,0,TDBE);
    colnb++;
// det ref 
    namfonct[LENNOM-2]=colnb;colnb++;scrGetNum(buf,jsbuf,'s',&rdet[i],namfonct,2,0,0,TDBE);
    namfonct[LENNOM-2]=colnb;   
    
// det dest 
    namfonct[LENNOM-2]=colnb;colnb++;scrGetNum(buf,jsbuf,'s',&det[i],namfonct,2,0,0,TDBE);
    namfonct[LENNOM-2]=colnb;
// mémo 
    const char* mem="\0";
    if(memo[i]>=0 && memo[i]<NBMEMOS){mem=&memosTable[memo[i]*LMEMO];}
    scrGetText(buf,jsbuf,mem,namfonct,LMEMO-1,0,TDBE|TREND);

    ethWrite(cli,buf,lb);
  }

  tableEnd(buf,jsbuf,BRYES);
  scrGetButSub(buf,jsbuf,"MàJ",0);
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
          scrDspText(buf,jsbuf,strDate,0,BRYES);
          memcpy(colourbr,"black\0",6);
          if(dateCmp(periLastDateIn,pkdate,*periPerRefr,1,1)<0){memcpy(colourbr,"teal\0",4);}
          if(dateCmp(periLastDateIn,pkdate,late,1,1)<0){memcpy(colourbr,"red\0",4);}setColourB(buf,jsbuf,colourbr);                 
          memset(strDate,0x00,LSTRD);
          concatDate(strDate,nullptr,periLastDateIn);
          scrDspText(buf,jsbuf,strDate,0,TDEND);
          setColourB(buf,jsbuf,"black");                      
          setColourE(buf,jsbuf);
}

void periLineHtml(EthernetClient* cli)              // periCur ok
{
/* ------------- modèle de page  ------------- */

  char jsbuf[LBUF4000];*jsbuf=LF;*(jsbuf+1)=0x00;   // jsbuf et buf init 
  uint16_t lb=0,lb0=LBUF2000;
  char buf[lb0];*buf=0x00;

  int j;
  unsigned long begTPage=millis();    // calcul durée envoi page

  Serial.print("periLineHtml - periCur=");Serial.print(periCur);

  htmlBeg(buf,jsbuf,serverName);       // chargement CSS etc
  
  optSelHtml(jsbuf,rulop,optNam0);    // chargement tables params

  formIntro(buf,jsbuf,"peri_cur__",0,0);  // n° usr + time usr + pericur + what=5 (faire periSave si MàJ)

  pageLineOne(buf,jsbuf);             // 1ère ligne page
  perifHeader(buf,jsbuf);             // 2nde ligne page

  

  ethWrite(cli,buf,&lb);              // tfr -> navigateur
// ------------------------------------------------------------- header end 

/* boutons */

    //strcat(buf,"<table><tr>\n");
    tableBeg(buf,jsbuf,false,0);

    scrGetButRet(buf,jsbuf,"retour",TRBEG|TDBE);strcat(buf," ");

    scrGetButSub(buf,jsbuf,"MàJ",TDBEG);
    char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    scrGetButFn(buf,jsbuf,line,"","refresh",ALICNO,0,0);
    
    if(*periSwNb!=0){
      char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
      scrGetButFn(buf,jsbuf,swf,"","Switchs",ALICNO,0,0);
    }
    affSpace(buf,jsbuf);
    char raz[]="peri_raz__";raz[LENNOM-1]=periCur+PMFNCHAR;
    scrGetButFn(buf,jsbuf,raz,"","Raz",ALICNO,0,BRYES);

    memcpy (line,"peri_tst__",LENNOM);line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';
    line[LENNOM-2]='0';scrGetButFn(buf,jsbuf,line,"","tst__SW0",ALICNO,0,0);
    line[LENNOM-2]='1';scrGetButFn(buf,jsbuf,line,"","tst__SW1",ALICNO,0,0);
    affSpace(buf,jsbuf);
    line[LENNOM-2]='m';scrGetButFn(buf,jsbuf,line,"","tst_mail",ALICNO,0,TREND|TDEND);
    tableEnd(buf,jsbuf,0);
    //strcat(buf,"</tr></table>\n");
    
    ethWrite(cli,buf,&lb);

/* ligne périphérique */                

                periInitVar();periLoad(periCur);
                if(*periSwNb>MAXSW){periCheck(periCur,"perT");periInitVar();periSave(periCur,PERISAVESD);}
                if(*periDetNb>MAXDET){periCheck(periCur,"perT");periInitVar();periSave(periCur,PERISAVESD);}

                tableBeg(buf,jsbuf,0);
                      //strcat(buf,"<tr><th></th><th><br>periph_name</th><th><br>TH</th><th><br>  V </th><th>per_t<br>pth<br>ofs</th><th>per_s<br> <br>pg</th><th>nb<br>sw<br>det</th><th>._D_ _l<br>._i_ _e<br>._s_ _v</th><th>mac_addr<br>ip_addr</th><th>version Th<br>last out<br>last in</th></tr><br>");   
                      scrDspText(buf,jsbuf,"||~periph_name|~TH|~  V |per_t~pth~ofs|per_s~ ~pg|nb~sw~det|._D_ _l~._i_ _e~._s_ _v|mac_addr~ip_addr|version Th~last out~last in",0,TRBE);
                      strcat(buf,"\n");
                      scrDspNum(buf,jsbuf,'d',&periCur,0,0,TDBE);
                      scrGetText(buf,jsbuf,periNamer,"peri_nom__",12,PERINAMLEN-1,0,TRNO|TDBE|BRNO);                    
                      scrDspNum(buf,jsbuf,periLastVal_,periThmin_,periThmax_,BRYES|TDBEG);            
                      float ff=((float)*periThmin_)/100;
                      scrDspNum(buf,jsbuf,'f',&ff,2,0,BRYES);     
                      ff=((float)*periThmax_)/100;
                      scrDspNum(buf,jsbuf,'f',&ff,2,0,TDEND);
                      strcat(buf,"\n");
                      scrDspNum(buf,jsbuf,periAlim_,periVmin_,periVmax_,BRYES|TDBEG);
                      scrGetNum(buf,jsbuf,'I',periVmin_,"peri_vmin_",1,5,0,0,BRYES);
                      scrGetNum(buf,jsbuf,'I',periVmax_,"peri_vmax_",1,5,0,0,TDEND);
                      scrGetNum(buf,jsbuf,'d',(uint32_t*)periPerTemp,"peri_rtemp",1,5,0,0,BRYES|TDBEG);
                      scrGetNum(buf,jsbuf,'r',periPitch_,"peri_pitch",1,4,2,0,BRYES);
                      scrGetNum(buf,jsbuf,'r',periThOffset_,"peri_tofs_",1,4,2,0,TDEND);
                      strcat(buf,"\n");
                      scrGetNum(buf,jsbuf,'l',(uint32_t*)periPerRefr,"peri_refr_",1,5,0,0,TDBEG|BRYES);
                      scrGetCheckbox(buf,jsbuf,(uint8_t*)periProg,"peri_prog_",NO_STATE,TDEND,"");
                      scrGetNum(buf,jsbuf,'b',periSwNb,"peri_intnb",1,1,0,0,TDBEG|BRYES);
                      scrGetNum(buf,jsbuf,'b',periDetNb,"peri_detnb",1,1,0,0,TDEND);
                      strcat(buf,"\n");
                      char fonc[]={"peri_vsw__\0"},oi[]={"OI"};
                      uint8_t lctl=TDBEG;
                      for(int k=0;k<*periSwNb;k++){
                        fonc[LENNOM-1]=PMFNCHAR+k;
                        scrGetRadiobut(buf,jsbuf,periSwCde(k),fonc,2,0,lctl);
                        lctl=0;if(k<*periSwNb-1){lctl=BRYES;}
                        char tt[2]={oi[periSwLev(k)],0x00};
                        scrDspText(buf,jsbuf,tt,0,lctl|STRING|CONCAT);
                        lctl=0;}
                      strcat(buf,"\n");

                      char bb[LENNOM];bb[0]='\0';
                      for(int k=0;k<6;k++){concat1a(bb,chexa[periMacr[k]/16]);concat1a(bb,chexa[periMacr[k]%16]);}
                      scrGetText(buf,jsbuf,bb,"peri_mac__",12,12,0,TDBEG|BRYES);      
                      scrDspText(buf,jsbuf,"port=",0,0);
                      scrGetNum(buf,jsbuf,'d',periPort,"peri_port_",1,4,0,0,BRYES);
                      scrDspText(buf,jsbuf,(char*)"",2,0);       // set police 2
                      #define LIP 16
                      char bc[LIP];memset(bc,'\0',LIP);
                      for(j=0;j<4;j++){concatns(bc,periIpAddr[j]);if(j<3){strcat(bc,".");}}
                      scrDspText(buf,jsbuf,bc,0,TDEND);
                      fontEnd(buf,jsbuf,TDEND);
                      scrDspText(buf,jsbuf,(char*)"",2,TDBEG);   // set police 2
                      char bd[LENVERSION+1];memcpy(bd,periVers,LENVERSION);bd[LENVERSION]='\0';
                      scrDspText(buf,jsbuf,bd,0,0);
                      *bd=*periProtocol;*(bd+1)='\0';
                      scrDspText(buf,jsbuf,bd,0,BRYES);
                      showDates(buf,jsbuf);
                      fontEnd(buf,jsbuf,TDEND);
                      
                tableEnd(buf,jsbuf,TREND|BRYES);
                strcat(buf,"\n\n");

                ethWrite(cli,buf,&lb);

// table analogique
                
                scrDspText(buf,jsbuf,"Analog Input",0,BRYES);
                tableBeg(buf,jsbuf,TRBEG);
                scrDspText(buf,jsbuf,"val~Low~High",0,TDBE);
                scrDspNum(buf,jsbuf,(int16_t*)periAnal,&valMin,&valMax,BRYES|TDBEG);
                scrGetNum(buf,jsbuf,'I',periAnalLow,"peri_ana@",5,0,0,BRYES);
                scrGetNum(buf,jsbuf,'I',periAnalHigh,"peri_anaA",5,0,0,NOBR|TDEND);
                scrDspText(buf,jsbuf,"Off1~Fact~Off2",0,TDBE);
                scrGetNum(buf,jsbuf,'I',periAnalOffset1,"peri_anaB_",5,0,0,TDBEG|BRYES);
                scrGetNum(buf,jsbuf,'F',periAnalFactor,"peri_anaC_",2,1,0,4,BRYES);               
                scrGetNum(buf,jsbuf,'F',periAnalOffset2,"peri_anaD_",2,1,0,4,TDEND|TREND);                   
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
                  dLibState[periCur*DIGITSIZLIB+1]='_';}
                if(*periDetNb>0){
                  subCbdet(buf,jsbuf,cli,1,"Digital Inputs Rules","rul_dig___",*periDetNb,dLibState,DIGITSIZLIB,NBRULOP,LENRULOP,rulop,optNam0,periDigitCb,periDigitDestDet,periDigitRefDet,periDigitMemo,&lb);
                }
                
            htmlEnd(buf,jsbuf);

            ethWrite(cli,buf,&lb);

            bufLenShow(buf,jsbuf,lb,begTPage);
}

void showLine(char* buf,char* jsbuf,EthernetClient* cli,int numline,char* pkdate,uint16_t* lb)
{
//Serial.print("showLine ");Serial.print(numline);
  float vv=0;
      periLoad(numline);
      periCur=numline;
      if(*periSwNb>MAXSW){periInitVar();periSave(numline,PERISAVESD);}  
      if(*periDetNb>MAXDET){periInitVar();periSave(numline,PERISAVESD);}

/* line form header */
          formIntro(buf,jsbuf,0,TRBEG);
/* pericur - nom - th - volts */
          uint8_t lctl=STRING|TRBEG|TDBE;
          scrDspNum(buf,jsbuf,'d',&periCur,0,0,lctl);
          scrDspText(buf,jsbuf,periNamer,0,STRING);
          //scrDspText(buf,jsbuf,"0.",0,TDBEG);scrDspNum(buf,jsbuf,'d',periLastVal_,0,0,BRYES);
          scrDspNum(buf,jsbuf,periLastVal_,periThmin_,periThmax_,TDBEG|BRYES);
          vv=(float)(*periThmin_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,BRYES);
          vv=(float)(*periThmax_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,TDEND);
          scrDspNum(buf,jsbuf,periAlim_,periVmin_,periVmax_,TDBEG|BRYES);          
          vv=(float)(*periVmin_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,BRYES);
          vv=(float)(*periVmax_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,TDEND);               
/* pertemp - pitch - offset */ 
          scrDspNum(buf,jsbuf,'d',periPerTemp,0,0,STRING|TDBEG|BRYES);
          vv=(float)(*periPitch_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,STRING|BRYES);
          vv=(float)(*periThOffset_)/100;scrDspNum(buf,jsbuf,'F',&vv,2,0,STRING|TDEND);          
      
          char oi[2]={'O','I'};
          if(*periProg==0){lctl=STRING|TDEND;}
          else {lctl=STRING|BRYES;}
          scrDspNum(buf,jsbuf,'d',(uint16_t*)periPerRefr,0,0,lctl);
          if(*periProg!=0){scrDspText(buf,jsbuf,"serv",0,STRING|TDEND);}
            
          scrDspNum(buf,jsbuf,'s',(uint8_t*)periSwNb,0,0,STRING|BRYES);
          scrDspNum(buf,jsbuf,'s',(uint8_t*)periDetNb,0,0,STRING|TDEND);                                                 
          
          char tt[4];tt[3]=0x00;
          for(uint8_t k=0;k<*periSwNb;k++){
                      tt[0]=oi[periSwCde(k)];tt[1]='_';tt[2]=oi[periSwLev(k)];
                      if(k<*periSwNb-1){lctl=STRING|BRYES;}
                      else {lctl=STRING|TDEND;}
                      scrDspText(buf,jsbuf,tt,0,lctl);
          }
          if(*periSwNb==0){scrDspText(buf,jsbuf," ",0,STRING|TDEND);}          
          for(uint8_t k=0;k<*periDetNb;k++){
            if(k<*periDetNb-1){lctl=STRING|BRYES;}
            else lctl=STRING|TDEND;
            char tt[2]={oi[(*periDetVal>>(k*2))&DETBITLH_VB],0x00};
            scrDspText(buf,jsbuf,tt,0,lctl);   
          }
          if(*periDetNb==0){scrDspText(buf,jsbuf," ",0,STRING|TDEND);}
// analog 
          vv=(*periAnal+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;scrDspNum(buf,jsbuf,'f',&vv,4,0,STRING|BRYES);
          vv=(*periAnalLow+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;scrDspNum(buf,jsbuf,'f',&vv,4,0,STRING|BRYES);
          vv=(*periAnalHigh+*periAnalOffset1)*(*periAnalFactor)+*periAnalOffset2;scrDspNum(buf,jsbuf,'f',&vv,4,0,STRING|TDEND);
          strcat(buf,"\n");
// mac                    
          lctl=TDBEG;
          char m[18];m[17]=0x00;
          for(int k=0;k<6;k++){
            m[k*3]=chexa[periMacr[k]/16];
            m[k*3+1]=chexa[periMacr[k]%16];
            if(k<5){m[k*3+2]='.';}
          }
          scrDspText(buf,jsbuf,m,0,STRING|BRYES);
          if(*periProg!=0 || *periProtocol=='U'){
            scrDspText(buf,jsbuf,"port=",0,STRING|CONCAT);
            scrDspNum(buf,jsbuf,'d',periPort,0,0,STRING|BRYES);}
          else{scrDspText(buf,jsbuf,"",0,STRING|BRYES);}
// IP addr
          char w[16];memset(w,0x00,16);
          uint8_t s=0;
          for(uint8_t j=0;j<4;j++){
            s+=sprintf(w+s,"%hu",(uint8_t)periIpAddr[j]);
            if(j<3){w[s]='.';s++;}
          }
          scrDspText(buf,jsbuf,w,0,STRING|TDEND);
// version ssid protocol
          char vers[LENVERSION+3];memcpy(vers,periVers,LENVERSION);
          vers[LENVERSION]=' ';
          char t=*periSsidNb;
          if(!((t>='0' && t<='9') || t=='?')){t=' ';}
          vers[LENVERSION+1]=t;
          vers[LENVERSION+2]='\0';
          scrDspText(buf,jsbuf,vers,0,STRING|CONCAT);scrDspText(buf,jsbuf," ",0,STRING|CONCAT);
          long p=strchr(protoChar,*periProtocol)-protoChar;if(p<0 || p>NBPROTOC){p=0;}
          char* a=protocStr+LENPROSTR*p;scrDspText(buf,jsbuf,a,0,STRING|CONCAT|BRYES);
// date_heures
          showDates(buf,jsbuf);
          strcat(buf,"\n");
// boutons
          char line[]="periline__";line[LENNOM-1]=periCur+PMFNCHAR;line[LENNOM]='\0';                      
          scrGetButFn(buf,jsbuf,line,"","Periph",ALICNO,0,TDBEG|BRYES);                         // bouton periph
          if(*periSwNb!=0){char swf[]="switchs___";swf[LENNOM-1]=periCur+PMFNCHAR;swf[LENNOM]='\0';
            scrGetButFn(buf,jsbuf,swf,"","Switchs",ALICNO,0,TDEND);}                            // bouton switchs
          strcat(buf,"\n");                                    
// fin
          formEnd(buf,jsbuf,0,TREND);
          strcat(buf,"\n");

          ethWrite(cli,buf,lb);
}
