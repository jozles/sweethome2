#include <Arduino.h>
#include <SD.h>
#include "ds3231.h"
#include "const.h"
//#include <Wire.h>
#include "utilether.h"
#include "utilhtml.h"
#include "shconst2.h"
#include "shutil2.h"
#include "periph.h"
#include "pageshtml.h"

extern Ds3231 ds3231;

extern char*      nomserver;
extern byte*      mac;              // adresse server
extern byte*      localIp;
extern uint16_t*  portserver;
extern char*      userpass;         // mot de passe browser
extern char*      modpass;          // mot de passe modif
extern char*      peripass;         // mot de passe périphériques

extern char*      chexa;
extern byte       maskbit[];

extern int        periCur;          // Numéro du périphérique courant

extern byte*      periMacr;                     // ptr ds buffer : mac address 
extern char*      periNamer;                    // ptr ds buffer : description périphérique
extern int16_t*   periLastVal_;                  // ptr ds buffer : dernière valeur de température  
extern int16_t*   periThmin_;                    // ptr ds buffer : alarme mini th
extern int16_t*   periThmax_;                    // ptr ds buffer : alarme maxi th
extern int16_t*   periThOffset_;                 // ptr ds buffer : offset correctif sur mesure température
extern char*      periLastDateIn;               // ptr ds buffer : date/heure de dernière réception
extern char*      periLastDateOut;              // ptr ds buffer : date/heure de dernier envoi  
extern char*      periVers;                     // ptr ds buffer : version logiciel du périphérique
extern char*      periModel;                    // ptr ds buffer : model du périphérique

extern byte       periMacBuf[6]; 

extern uint16_t   perrefr;
extern File       fhisto;           // fichier histo sd card
extern long       fhsize;           // remplissage fhisto
extern long       sdpos;
extern char       strSD[RECCHAR];


extern char*      ssid;
extern char*      passssid;
extern char*      usrnames;
extern char*      usrpass;
extern unsigned long*     usrtime;
extern uint16_t*  toPassword;

extern int        usernum;

extern byte*      periSwVal;                    // ptr ds buffer peri : état/cde des inter 

File fimg;     // fichier image

extern struct SwRemote remoteT[MAXREMLI];
extern struct Remote remoteN[NBREMOTE];

extern struct Timers timersN[NBTIMERS];

extern struct Thermo thermos[NBTHERMOS];

extern char*     periNamer;                    // ptr ds buffer : description périphérique

extern int       fdatasave;

extern uint32_t  memDetServ;  // image mémoire NBDSRV détecteurs
extern char      libDetServ[NBDSRV][LENLIBDETSERV];


   // Un formulaire ou une page html nécessite dans l'ordre :
   //      1) une fonction user_ref_ avec ses paramètres pour éviter le retour au mot de passe
   //      2) une fonction d'en tête hidden pour effectuer 
   //         les inits éventuels lors du submit de validation (posit what et effacements cb par ex)
   //         si pas d'inits à faire, elle n'est pas nécessaire
   //      3) n fonctions pour valoriser des champs
   //      ... n'importe où après (2) un bouton submit 
   //       
   // La fonction user_ref_ est fournie par userFormHtml()
   // S'il faut passer une fonction d'en-tête utiliser userFormInitHtml()
   // Si le passage de periCur est nécessaire utiliser usrPeriCur()
   //



int htmlImg(EthernetClient* cli,char* fimgname)    // suffisant pour commande péripheriques
{
        Serial.print(fimgname);
        File fimg; // = SD.open(fimgname,FILE_READ);
        if(sdOpen(FILE_READ,&fimg,fimgname)==SDKO){return SDKO;}
        else {
  
          cli->println("HTTP/1.1 200 OK");
          cli->println("CONTENT-Type: image/jpg");
          //cli->println("<link rel="icon" type="image/png" href="favicon.png\" sizes=\"64x64\">");
          cli->println();

          long fimgSiz=fimg.size();
          byte c;
          Serial.print(" size=");Serial.print(fimgSiz);
          while (fimgSiz>0){c=fimg.read();cli->write(&c,1);fimgSiz--;}
          fimg.close();
          delay(1);   
        }
        Serial.println(" terminé");
        cli->stop();
        return SDOK;
}

void htmlFavicon(EthernetClient* cli)
{
  htmlImg(cli,"sweeth.png");
}

void dumpsd(EthernetClient* cli)
{ 
  trigwd();
  htmlIntro(nomserver,cli);

  cli->print("<body>");cli->print(VERSION);cli->println("<br>");
  boutRetour(cli,"retour",0,1);
  if(dumpsd0(cli)!=SDOK){cli->println("SDKO");}
  boutRetour(cli,"retour",0,1);
  
  cli->println("</body></html>");
}

int dumpsd0(EthernetClient* cli)                 // liste le fichier de la carte sd
{
  char inch=0;
  
  if(sdOpen(FILE_READ,&fhisto,"fdhisto.txt")==SDKO){return SDKO;}

  trigwd();
  fhsize=fhisto.size();
  fhisto.seek(sdpos);

  cli->print("histoSD ");cli->print(sdpos);cli->print("/");cli->print(fhsize);cli->println("<br>");

  long ptr=sdpos;
  long ptr0=ptr;
  long ptra=ptr;
  long ptrb=ptr;
  
#define LBUF 1000  
  char buf[LBUF];

  while(ptr<fhsize){
    while((ptr-ptra)<(LBUF-2) && ptr<fhsize){           // -1 for end null char
      buf[ptr-ptra]=fhisto.read();ptr++;
    }
    buf[ptr-ptra]='\0';
    cli->print(buf);
    ptra=ptr;
    if((ptr-ptrb)>10000){ptrb=ptr;trigwd();}
    if((ptr-ptr0)>100000){break;}
  }
  
  fhisto.close();
  return SDOK;
}

void accueilHtml(EthernetClient* cli)
{
            Serial.println(" saisie pwd");
            htmlIntro(nomserver,cli);

            cli->println("<body><form method=\"get\" >");
            cli->println(VERSION);cli->println("<br>");

            cli->println("<p>user <input type=\"username\" placeholder=\"Username\" name=\"username__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");            
            cli->println("<p>pass <input type=\"password\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");
                        cli->println(" <input type=\"submit\" value=\"login\"><br>");
            cli->println("</form></body></html>");
}          

void sscb(EthernetClient* cli,bool val,char* nomfonct,int nuf,int etat,uint8_t td,uint8_t nb)
{                                                                               // saisie checkbox ; 
                                                                                // le nom de fonction reçoit 2 caractères
  char nf[LENNOM+1];
  memcpy(nf,nomfonct,LENNOM);
  nf[LENNOM]='\0';
  nf[LENNOM-1]=(char)(nb+PMFNCHAR);
  nf[LENNOM-2]=(char)(nuf+PMFNCHAR);
  checkboxTableHtml(cli,(uint8_t*)&val,nf,etat,td,"");
}


void sscfgt(EthernetClient* cli,char* nom,uint8_t nb,void* value,int len,uint8_t type)  // type=0 value ok ; type =1 (char)value modulo len*nt ; type =2 (uint16_t)value modulo nb
{
  int sizbx=len-3;if(sizbx<=0){sizbx=1;}
  cli->print("<td><input type=\"text\" name=\"");cli->print(nom);cli->print((char)(nb+PMFNCHAR));cli->print("\" value=\"");
  if(type==0){cli->print((char*)value);cli->print("\" size=\"");cli->print(sizbx);cli->print("\" maxlength=\"");cli->print(len);cli->println("\" ></td>");}
  if(type==2){cli->print(*((int16_t*)value+nb));cli->println("\" size=\"1\" maxlength=\"2\" ></td>");}
  if(type==1){cli->print((char*)(((char*)value+(nb*(len+1)))));cli->print("\" size=\"");cli->print(len);cli->print("\" maxlength=\"");cli->print(len);cli->println("\" ></td>");}
  if(type==3){cli->print(*((int8_t*)value));cli->println("\" size=\"1\" maxlength=\"2\" ></td>");}
}

void subcfgtable(EthernetClient* cli,char* titre,int nbl,char* nom1,char* value1,int len1,uint8_t type1,char* nom2,void* value2,int len2,char* titre2,uint8_t type2)
{
    cli->println("<table><col width=\"22\">");
    cli->println("<tr>");
    cli->print("<th></th><th>");cli->print(titre);cli->print("</th><th>");cli->print(titre2);cli->println("</th>");
    cli->println("</tr>");

    for(int nb=0;nb<nbl;nb++){
      cli->println("<tr>");
      cli->print("<td>");cli->print(nb);cli->print("</td>");

      sscfgt(cli,nom1,nb,value1,len1,type1);                      
      sscfgt(cli,nom2,nb,value2,len2,type2);

      if(len2==-1){
        int16_t peri=*((int16_t*)value2+nb);
        if(peri>0){Serial.print(peri);periLoad(peri);cli->println("<td>");cli->println(periNamer);cli->println("</td>");}
        if(nb==nbl-1){Serial.println();}
      }
      cli->println("</tr>");
    }
    cli->println("</table>");          
}

void cfgServerHtml(EthernetClient* cli)
{
            Serial.println(" config serveur");
            
            htmlIntro(nomserver,cli);
            
            cli->println("<body><form method=\"get\" >");
            cli->println(VERSION);cli->println("<br>");

            usrFormHtml(cli,1);
            
            boutRetour(cli,"retour",0,0);
            
            cli->println(" <input type=\"submit\" value=\"MàJ\"><br>");
            
            cli->print(" password <input type=\"text\" name=\"pwdcfg____\" value=\"");cli->print(userpass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");
            cli->print("  modpass <input type=\"text\" name=\"modpcfg___\" value=\"");cli->print(modpass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");            
            cli->print(" peripass <input type=\"text\" name=\"peripcfg__\" value=\"");cli->print(peripass);cli->print("\" size=\"5\" maxlength=\"");cli->print(LPWD);cli->println("\" >");            
            cli->print(" to password");numTableHtml(cli,'d',toPassword,"to_passwd_",6,0,0);cli->println("<br>");
            cli->print(" serverMac <input type=\"text\" name=\"ethcfg___m\" value=\"");for(int k=0;k<6;k++){cli->print(chexa[mac[k]/16]);cli->print(chexa[mac[k]%16]);}cli->print("\" size=\"11\" maxlength=\"");cli->print(12);cli->println("\" >");                        
            cli->print(" localIp <input type=\"text\" name=\"ethcfg___i\" value=\"");for(int k=0;k<4;k++){cli->print(localIp[k]);if(k!=3){cli->print(".");}}cli->print("\" size=\"11\" maxlength=\"");cli->print(15);cli->println("\" >");                        
            cli->print(" portserver ");numTableHtml(cli,'d',portserver,"ethcfg___p",4,0,0);cli->println("<br>");

            subcfgtable(cli,"SSID",MAXSSID,"ssid_____",ssid,LENSSID,1,"passssid_",passssid,LPWSSID,"password",1);
            subcfgtable(cli,"USERNAME",NBUSR,"usrname__",usrnames,LENUSRNAME,1,"usrpass__",usrpass,LENUSRPASS,"password",1);
          
            //subcfgtable(cli,"THERMO",NBTHERMO,"thername_",thermonames,LENTHNAME,1,"therperi_",thermoperis,-1,"peri",2);
            
            cli->println("</form></body></html>");
}

void cfgDetServHtml(EthernetClient* cli)
{
  
            Serial.println(" config detServ");
            htmlIntro(nomserver,cli);
            
            cli->println("<body><form method=\"get\" >");
            cli->println(VERSION);cli->println("<br>");
            
            usrFormHtml(cli,1);
            boutRetour(cli,"retour",0,0);

            cli->println(" <input type=\"submit\" value=\"MàJ\"><br>");

/* table libellés */

              cli->println("<table>");
              cli->println("<tr>");
              cli->println("<th>   </th><th>      Nom      </th>");
              cli->println("</tr>");

              for(int nb=0;nb<NBDSRV;nb++){
                uint8_t decal=0;if(nb>=16){decal=16;}
                cli->println("<tr>");
                
                cli->print("<td>");cli->print(nb);cli->print("</td>");                       // n° detserv
                cli->print("<td><input type=\"text\" name=\"libdsrv__");cli->print((char)(nb+decal+PMFNCHAR));cli->print("\" value=\"");
                        cli->print((char*)&libDetServ[nb][0]);cli->print("\" size=\"12\" maxlength=\"");cli->print(LENLIBDETSERV-1);cli->println("\" ></td>");
                   
                cli->println("</tr>");
              }
            cli->println("</table><br>");
            cli->println("</form></body></html>");
}


void cfgRemoteHtml(EthernetClient* cli)
{
  char nf[LENNOM+1];nf[LENNOM]='\0';
  uint8_t val;
  
            Serial.println(" config remote");
            htmlIntro(nomserver,cli);
            
            cli->println("<body><form method=\"get\" >");
            cli->println(VERSION);cli->println();
            
            usrFormHtml(cli,1);
            boutRetour(cli,"retour",0,0);
            cli->println(" <input type=\"submit\" value=\"MàJ\">");
            char pkdate[7];cliPrintDateHeure(cli,pkdate);cli->println();
            //detServHtml(cli,&memDetServ,&libDetServ[0][0]);            

/* table remotes */

              cli->println("<table>");
              cli->println("<tr>");
              cli->println("<th>   </th><th>      Nom      </th><th> on/off </th><th> en </th>");
              cli->println("</tr>");

              for(int nb=0;nb<NBREMOTE;nb++){
                cli->println("<tr>");
                
                cli->print("<td>");cli->print(nb+1);cli->print("</td>");                       // n° remote
                cli->print("<td><input type=\"text\" name=\"remotecfn");cli->print((char)(nb+PMFNCHAR));cli->print("\" value=\"");
                        cli->print(remoteN[nb].nam);cli->print("\" size=\"12\" maxlength=\"");cli->print(LENREMNAM-1);cli->println("\" ></td>");

                //sliderHtml(cli,(uint8_t*)(&remoteN[nb].onoff),"remotecfo_",nb,0,1);

                memcpy(nf,"remotecfo_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // état on/off
                val=(uint8_t)remoteN[nb].onoff;
                checkboxTableHtml(cli,&val,nf,-1,1,"");         
                
                memcpy(nf,"remotecfe_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // enable (inutilisé ?)
                val=(uint8_t)remoteN[nb].enable;
                checkboxTableHtml(cli,&val,nf,-1,1,"");         
                   
                cli->println("</tr>");
              }
            cli->println("</table><br>");

/* table détecteurs */

            cli->println("<table>");
              cli->println("<tr>");
              cli->println("<th>  </th><th>remote </th><th>detec on/off</th><th>detec en</th>");
              cli->println("</tr>");
              
              for(int nb=0;nb<MAXREMLI;nb++){
                cli->println("<tr>");
                cli->print("<td>");cli->print(nb+1);cli->print("</td>");                       // n° ligne de table
                
                memcpy(nf,"remotecfu_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° remote
                numTableHtml(cli,'b',&remoteT[nb].num,nf,1,2,0);
                if(remoteT[nb].num!=0){cli->print(remoteN[remoteT[nb].num-1].nam);}cli->println(" </td>");

                memcpy(nf,"remotecfd_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° detec on/off
                numTableHtml(cli,'b',&remoteT[nb].detec,nf,2,2,0);
                if(remoteT[nb].num!=0){
                  cli->print((char*)(&libDetServ[remoteT[nb].detec][0]));cli->print(" ");
                  cli->print((char)(((memDetServ>>remoteT[nb].detec)&0x01)+48));}
                cli->println(" </td>");

                memcpy(nf,"remotecfb_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // n° detec enable
                numTableHtml(cli,'b',&remoteT[nb].deten,nf,2,2,0);
                if(remoteT[nb].num!=0){
                  cli->print((char*)(&libDetServ[remoteT[nb].deten][0]));cli->print(" ");
                  cli->print((char)(((memDetServ>>remoteT[nb].deten)&0x01)+48));}
                cli->println(" </td>");
/*                
                memcpy(nf,"remotecfx_",LENNOM);nf[LENNOM-1]=(char)(nb+PMFNCHAR);               // enable (inutilisé ?)
                uint8_t ren=(uint8_t)remoteT[nb].enable;
                checkboxTableHtml(cli,&ren,nf,-1,1,"");         
*/                
                cli->println("</tr>");
              }
              
            cli->println("</table>");            
            cli->println("</form></body></html>");
}


void remoteHtml(EthernetClient* cli)
{  
            Serial.println("remote control");
            htmlIntro(nomserver,cli);
            
            cli->println("<body><form method=\"get\" >");
            cli->println(VERSION);cli->println(" ");

            usrFormHtml(cli,1);
            
            boutRetour(cli,"retour",0,0);cli->print(" ");          
            cli->println("<br>");

/* table remotes */

            cli->println("<table>");

            for(int nb=0;nb<NBREMOTE;nb++){
              if(remoteN[nb].nam[0]!='\0'){
                cli->println("<tr>");
                
                cli->print("<td>");cli->print(nb+1);cli->println("</td>");
                cli->print("<td>");
                cli->print(" <font size=\"7\">");cli->print(remoteN[nb].nam);cli->println("</font></td>");
                // slider on/off
                // chaque ligne de slider envoie 2 commandes : remote_cnx et remote_ctx (x n° de ligne)
                // l'input hidden remote_cnx assure la présence d'une fonction dans 'GET /' pour assurer l'effacement des cb
                // l'input remote_ctx renseigne le passage à 1 éventuel après l'effacement. La variable newonoff stocke le résultat
                // et la comparaison avec onoff permet de connaitre la transition (ou non transition) 
                // periRemoteUpdate détecte les transitions, positionne les détecteurs et déclenche poolperif si nécessaire 
                // pour la maj via PerToSend des périphériques concernés
                cli->print("<input type=\"hidden\" name=\"remote_cn");cli->print((char)(nb+PMFNCHAR));cli->println("\">");
                sliderHtml(cli,(uint8_t*)(&remoteN[nb].onoff),"remote_ct",nb,0,1);

                // slider enable idem slider on/off
                cli->print("<input type=\"hidden\" name=\"remote_cm");cli->print((char)(nb+PMFNCHAR));cli->println("\">");
                sliderHtml(cli,(uint8_t*)(&remoteN[nb].enable),"remote_cs",nb,0,1);                
                
/*                uint8_t ren=(uint8_t)remoteN[nb].enable;
                char nf[LENNOM+1]="remote_xe_";nf[LENNOM-1]=(char)(nb+PMFNCHAR);                
                checkboxTableHtml(cli,&ren,nf,-1,1,"");
*/                
                cli->println("</tr>");
              }
            }
            cli->println("</table>");
            
            cli->println("<p align=\"center\" ><input type=\"submit\" value=\"MàJ\" style=\"height:120px;width:400px;background-color:LightYellow;font-size:40px;font-family:Courier,sans-serif;\"><br>");
            boutFonction(cli,"thermoshow","","températures",0,0,7,0);cli->print(" ");
            boutFonction(cli,"remotehtml","","refresh",0,0,7,0);
            cli->print("</p>");
            
            cli->println("</form></body></html>");
}

int scalcTh(int bd)           // maj temp min/max des périphériques sur les bd derniers jours
{
  
/* --- calcul date début --- */
  
  int   ldate=15;

  int   yy,mm,dd,js,hh,mi,ss;
  byte  yb,mb,db,dsb,hb,ib,sb;
  ds3231.readTime(&sb,&ib,&hb,&dsb,&db,&mb,&yb);           // get date(now)
  yy=yb+2000;mm=mb;dd=db;hh=hb;mi=ib;ss=sb;
  calcDate(bd,&yy,&mm,&dd,&js,&hh,&mi,&ss);               // get new date
  uint32_t amj=yy*10000L+mm*100+dd;
  uint32_t hms=hh*10000L+mi*100+ss;
  char     dhasc[ldate+1];
  sprintf(dhasc,"%.8lu",amj);strcat(dhasc," ");
  sprintf(dhasc+9,"%.6lu",hms);dhasc[15]='\0';            // dhasc date/heure recherchée
//  Serial.print("dhasc=");Serial.println(dhasc);
  
  if(sdOpen(FILE_READ,&fhisto,"fdhisto.txt")==SDKO){return SDKO;}
  
  long sdsiz=fhisto.size();
  long searchStep=100000;
  long ptr,curpos=sdsiz;
  fhisto.seek(curpos);
  long pos=fhisto.position();  
  
  Serial.print("--- start search date at ");Serial.print(curpos-searchStep);Serial.print(" sdsiz=");Serial.print(sdsiz);Serial.print(" pos=");Serial.print(pos);Serial.print(" (millis=");Serial.print(millis());Serial.println(")");

/* --- recherche 1ère ligne --- */

  char inch1=0,inch2=0;
  char buf[RECCHAR];
  bool fini=FAUX;
  int pt;
    
  while(curpos>0 && !fini){
    curpos-=searchStep;if(curpos<0){curpos=0;}ptr=curpos;
    fhisto.seek(curpos);
    while(ptr<curpos+searchStep && inch1!='\n'){inch1=fhisto.read();ptr++;}
    for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                    // '\n' trouvé : get date
    if(memcmp(buf,dhasc,ldate)>0){ptr=curpos+searchStep;}                    // si la date trouvée est > reculer
    else {                                                                   // sinon chercher >
      
      while(!fini){
        while(ptr<pos && inch1!='\n'){inch1=fhisto.read();ptr++;}
        for(pt=0;pt<ldate;pt++){buf[pt]=fhisto.read();ptr++;}                // '\n' trouvé : get date
        if(memcmp(buf,dhasc,ldate)>0){                                       // si la date trouvé est > ok sinon continuer
          fini=VRAI;                                                         // ptr ok ; pt ok commencer l'acquisition
        }
      }
    }
  }
  unsigned long t0=millis();
  Serial.print("--- fin recherche ptr=");Serial.print(ptr);Serial.print(" millis=");Serial.print(millis());Serial.println("");

/* --- balayage et màj --- */
  
  char strfds[3];memset(strfds,0x00,3);
  if(convIntToString(strfds,fdatasave)>2){
    Serial.print("fdatasave>99!! ");Serial.print("fdatasave=");Serial.print(fdatasave);Serial.print(" strfds=");Serial.println(strfds);ledblink(BCODESYSERR);
  }
  char* pc;
  int16_t th_;
  uint8_t np_;
  int lnp=0,nbli=0,nbth=0;

  for(int pp=1;pp<=NBPERIF;pp++){periLoad(pp);*periThmin_=9900;*periThmax_=-9900;periSave(pp,PERISAVELOCAL);}
                                                                             
                                                                         // acquisition
  fhisto.seek(ptr-ldate);                                                // sur début enregistrement
  fini=FAUX;
  while(ptr<pos){

    pt=0;
    inch1='\0';      
    while(ptr<pos && inch1!='\n'){inch1=fhisto.read();buf[pt]=inch1;pt++;ptr++;}   // get record
    buf[pt]='\0';
    nbli++;
    pc=strchr(buf,';');
    if(memcmp(pc+1,strfds,2)==0){                                         // datasave (après ';' soit '\n' soit'<' soit num fonction)
      np_=(uint8_t)convStrToInt(pc+SDPOSNUMPER,&lnp);                     // num périphérique
      th_=(int16_t)(convStrToNum(pc+SDPOSTEMP,&lnp)*100);                 // temp périphérique
      periLoad(np_);
//    Serial.print(np_);Serial.print(" ");Serial.print(*periThmin_);Serial.print(" ");Serial.println(*periThmax_);
      packMac(periMacBuf,pc+SDPOSMAC);                       
      if(compMac(periMacBuf,periMacr) && th_<9900 && th_>-9900){                                  // contrôle mac
//        Serial.println(buf);Serial.print(" per=");Serial.print(np_);Serial.print(" th=");Serial.print(th_);Serial.print(" thmin=");Serial.print(*periThmin_);Serial.print(" thmax=");Serial.print(*periThmax_);Serial.print(" - ");
        if(*periThmin_>th_){*periThmin_=(int16_t)th_;periSave(np_,PERISAVELOCAL);nbth++;Serial.print(" maj ");}
        if(*periThmax_<th_){*periThmax_=(int16_t)th_;periSave(np_,PERISAVELOCAL);nbth++;Serial.print(" maj ");} 
//        Serial.println();
      }
    }
  }
  
  for(uint16_t pp=1;pp<=NBPERIF;pp++){periLoad(pp);if(periMacr[0]!=0x00){periSave(pp,PERISAVESD);}}   // écriture SD
  
  Serial.print("--- fin balayage ");Serial.print(nbli);Serial.print(" lignes ; ");Serial.print(nbth);Serial.print(" màj ; millis=");Serial.print(millis()-t0);Serial.println("");
 
  fhisto.seek(pos);
  return sdOpen(FILE_WRITE,&fhisto,"fdhisto.txt");
}

void intro(EthernetClient* cli)
{
  htmlIntro0(cli);
  cli->println("<head><title>sweet hdev</title>");
  
  cli->println("<style>");
  cli->println("table {");
  cli->println("font-family: Courier, sans-serif;");
  cli->println("border-collapse: collapse;");
  cli->println("overflow: auto;");
  cli->println("white-space:nowrap;");
  cli->println("}");
  cli->println("td, th {");
  cli->println("border: 1px solid #dddddd;");
  cli->println("text-align: left;");
  cli->println("}");
  cli->println("</style>");
  
  cli->println("</head>");
}

void thermoShowHtml(EthernetClient* cli)
{
          
            scalcTh(1);          // update periphériques
            intro(cli);
            
            cli->print("<body>");cli->print(VERSION);cli->print(" ");
            boutRetour(cli,"retour",0,0);cli->print(" ");


/* peritable températures */

         cli->println("<table>");
              cli->println("<tr>");
                cli->println("<th>peri</th><th></th><th>TH</th><th>min</th><th>max</th><th>last in</th>");
              cli->println("</tr>");

              for(int nuth=0;nuth<NBTHERMOS;nuth++){
                int16_t nuper=thermos[nuth].peri;
                if(nuper!=0){
                  periInitVar();periCur=nuper;periLoad(periCur);

                  if(periMacr[0]!=0x00){
                    cli->println("<tr>");
                      //cli->print("<td>");cli->print(nuth+1);cli->print("</td>");
                      cli->print("<td>");cli->print(periCur);cli->println("</td>");
                      cli->print("<td> <font size=\"7\">");cli->print(thermos[nuth].nom);cli->println("</font></td>");
                      cli->print("<td> <font size=\"7\">");cli->print((float)(*periLastVal_+*periThOffset_)/100);cli->println("</font></td>");
                      cli->print("<td> <div style='text-align:right; font-size:30px;'>");cli->print((float)*periThmin_/100);cli->println("</div></td>");
                      cli->print("<td> <div style='text-align:right; font-size:30px;'>");cli->print((float)*periThmax_/100);cli->println("</div></td>");
                      cli->print("<td>");printPeriDate(cli,periLastDateIn);cli->println("</td>");                      
                    cli->println("</tr>");
                  }
                }
              }
          cli->println("</table>");

        cli->print("<p align=\"center\">");
        boutFonction(cli,"remotehtml","","remote",0,0,7,0);cli->print(" ");                        
        boutFonction(cli,"thermoshow","","refresh",0,0,7,0);
        cli->print("</p>");
        cli->print("<br><br><br>");for(int d=0;d<NBDSRV;d++){cli->print((char)(((memDetServ>>d)&0x01)+48));cli->print(" ");}
        cli->println("/n</body></html>");
}

void subthd(EthernetClient* cli,char param,uint8_t nb,void* val,char type)
{
        uint8_t* vv;
        char nf[LENNOM+1];
        memcpy(nf,"thparams__\0",LENNOM+1);
        nf[LENNOM-2]=param;
        nf[LENNOM-1]=nb+PMFNCHAR;                              
        
        switch(type){
          case 'd': numTableHtml(cli,'b',val,nf,2,1,0);break;                           // n° detec/peri              
          case 'v': numTableHtml(cli,'I',val,nf,4,1,0);break;                           // value/offset
          case 'e': checkboxTableHtml(cli,(uint8_t*)val,nf,-1,1,"");break;              // enable/state
          default:break;
        }
}

void thermoCfgHtml(EthernetClient* cli)
{
  uint8_t val;
  
            Serial.println(" config thermos");
            htmlIntro(nomserver,cli);
            
            cli->println("<body>");
            cli->println(VERSION);cli->println();
            
            boutRetour(cli,"retour",0,0);
            boutFonction(cli,"thermoscfg","","refresh",0,0,1,0);cli->print(" ");
            
            char pkdate[7];cliPrintDateHeure(cli,pkdate);cli->println();
            //detServHtml(cli,&memDetServ,&libDetServ[0][0]);            

/* table thermos */

            cli->println("<table>");
              cli->println("<tr>");
                cli->println("<th>   </th><th>      Nom      </th><th> peri </th><th>  </th><th> low<br> en</th><th> low<br> state</th><th> low<br> value</th><th> low<br> pitch</th><th> low<br> det</th><th> </th><th> high<br> en</th><th> high<br> state</th><th> high<br> value</th><th> high<br> offset</th><th> high<br> det</th>");
              cli->println("</tr>");

              for(int nb=0;nb<NBTHERMOS;nb++){
                cli->println("<tr>");
                cli->println("<form method=\"get\" >");
                usrFormHtml(cli,1);
                
                cli->print("<td>");cli->print(nb+1);cli->print("</td>");                                      // n° thermo
                
                cli->print("<td><input type=\"text\" name=\"thparamsn");cli->print((char)(nb+PMFNCHAR));      // nom
                  cli->print("\" value=\"");
                  cli->print(thermos[nb].nom);cli->print("\" size=\"12\" maxlength=\"");cli->print(LENTHNAME-1);cli->println("\" ></td>");
    
                subthd(cli,'p',nb,&thermos[nb].peri,'d');                               // peri
                 periInitVar();periCur=thermos[nb].peri;if(periCur!=0){periLoad(periCur);}cli->print("<td>");cli->print(periNamer);
                 cli->println("|");cli->print((float)*periLastVal_/100);cli->println("</td>");
                subthd(cli,'e',nb,&thermos[nb].lowenable,'e');                          // low enable
                subthd(cli,'s',nb,&thermos[nb].lowstate,'e');                           // low state
                subthd(cli,'v',nb,&thermos[nb].lowvalue,'v');                           // low value
                subthd(cli,'o',nb,&thermos[nb].lowoffset,'v');                          // low offset
                subthd(cli,'d',nb,&thermos[nb].lowdetec,'d');                           // low det
                 uint8_t vv=(uint8_t)thermos[nb].lowdetec;cli->println(" <td>");if(vv!=0){cli->print((char*)(&libDetServ[vv][0]));cli->print(" ");cli->print((char)(((memDetServ>>vv)&0x01)+48));}cli->println(" </td>");
                

                cli->print("<td>");cli->print(" <input type=\"submit\" value=\"MàJ\"><br>");cli->println("</td>");
                cli->println("</form>");
                cli->println("</tr>");
              }
            cli->println("</table>");
            cli->println("</body></html>");
}


void timersHtml(EthernetClient* cli)
{
  int nucb;           
  char a;
  char bufdate[LNOW];ds3231.alphaNow(bufdate);
  
            Serial.println("saisie timers");
            htmlIntro(nomserver,cli);
            
            cli->print("<body>");cli->print(VERSION);cli->print(" ");
            
            boutRetour(cli,"retour",0,0);cli->print(" ");
            boutFonction(cli,"timershtml","","refresh",0,0,1,0);cli->print(" ");

            char pkdate[7];cliPrintDateHeure(cli,pkdate);cli->println();
            detServHtml(cli,&memDetServ,&libDetServ[0][0]);

         cli->println("<table>");
              cli->println("<tr>");
                cli->println("<th></th><th>nom</th><th>det</th><th>h_beg</th><th>h_end</th><th>OI det</th><th>e_p_c_</th><th>7_d_l_m_m_j_v_s</th><th>dh_beg_cycle</th><th>dh_end_cycle</th>");
              cli->println("</tr>");

              for(int nt=0;nt<NBTIMERS;nt++){

                    cli->println("<tr>");
                    cli->println("<form method=\"GET \">");
                      usrFormHtml(cli,1);
                      cli->print("<td>");cli->print(nt+1);cli->println("</td>");
                      
                      sscfgt(cli,"tim_name_",nt,timersN[nt].nom,LENTIMNAM,0);
                      sscfgt(cli,"tim_det__",nt,&timersN[nt].detec,2,3);                                            
                      sscfgt(cli,"tim_hdf_d",nt,timersN[nt].hdeb,6,0);                                            
                      sscfgt(cli,"tim_hdf_f",nt,timersN[nt].hfin,6,0);                   
                      
                      char oi[]="OI",md=memDetServ>>timersN[nt].detec;
                      cli->print("<td>");cli->print("_");
                      cli->print(oi[timersN[nt].curstate]);cli->print("__");
                      cli->print((memDetServ>>timersN[nt].detec)&0x01);
                      cli->println("</td><td>");
                      
                      nucb=0;sscb(cli,timersN[nt].enable,"tim_chkb__",nucb,-1,0,nt);
                      nucb++;sscb(cli,timersN[nt].perm,"tim_chkb__",nucb,-1,0,nt);
                      nucb++;sscb(cli,timersN[nt].cyclic,"tim_chkb__",nucb,-1,0,nt);   
                      //nucb++;sscb(cli,timersN[nt].forceonoff,"tim_chkb__",nucb,-1,0,nt);
                     
                      cli->println("</td><td>");
                      for(int nj=7;nj>=0;nj--){
                        bool vnj; 
                        vnj=(timersN[nt].dw>>nj)&0x01;
                        nucb++;sscb(cli,vnj,"tim_chkb__",nucb,-1,0,nt);
                      }
                      cli->println("</td>");
                      sscfgt(cli,"tim_hdf_b",nt,&timersN[nt].dhdebcycle,14,0);
                      sscfgt(cli,"tim_hdf_e",nt,&timersN[nt].dhfincycle,14,0); 
                      cli->print("<td>");cli->print(" <input type=\"submit\" value=\"MàJ\"><br>");cli->println("</td>");
                    cli->print("<td>");cli->print(timersN[nt].dw,HEX);cli->print(" ");cli->print(maskbit[1+bufdate[14]*2],HEX);cli->println("</td>");
                    cli->println("</form>");
                    cli->println("</tr>");
                  }

        cli->println("</table>");
        cli->println("</body></html>");
}

void detServHtml(EthernetClient* cli,uint32_t* mds,char* lib)
{
  char buf[1000];buf[0]='\0';
          strcat(buf,"<form>");
          usrFormInitBHtml(buf,"dsrv_init_");
          strcat(buf,"<fieldset><legend>détecteurs serveur (n->0):</legend>\n");
          for(int k=NBDSRV-1;k>=0;k--){
            char libb[LENLIBDETSERV];memcpy(libb,lib+k*LENLIBDETSERV,LENLIBDETSERV);
            if(libb[0]=='\0'){convIntToString(libb,k);}
            subDSnB(buf,"mem_dsrv__\0",*mds,k,libb);
            cli->print(buf);buf[0]='\0';}
          strcat(buf,"<input type=\"submit\" value=\"Per Update\"></fieldset></form>\n"); 
  cli->print(buf);        
}

void testHtml(EthernetClient* cli)
{
            Serial.println(" page d'essais");
 htmlImg(cli,"sweeth.jpg");
            
            
/*            
            
            htmlIntro(nomserver,cli);

            char pwd[32]="password__=\0";strcat(pwd,userpass);
            
            cli->println("<body><form method=\"get\" action=\"page.html\" >");

            cli->println("<a href=page.html?");cli->print(pwd);cli->print(":>retourner</a><br>");
            cli->println("<a href=page.html?password__=17515A:>retourner</a><br>");

            //cli->print(" <p hidden> ce que je ne veux plus voir <input type=\"text\" name=\"ma_fonct_1\" value=\"\"><br><p>");
            //cli->println(" ce que je veux toujour voir <input type=\"text\" name=\"ma_fonct_2\" value=\"\"><br>");

            //cli->print("<img id=\"img1\" alt=\"BOUTON\" fp-style=\"fp-btn: Border Left 1\" fp-title=\"BOUTON\" height=\"20\"  style=\"border: 0\" width=\"100\"> <br>");

            //cli->println(" <input type=\"submit\" value=\"MàJ\"><br>");

            cli->println("<a href=\"page.html?password__=17515A:\"><input type=\"button\" value=\"href+input button\"></a><br>"); 
            
            cli->println("<a href=page.html?");cli->print(pwd);cli->print(":><input type=\"submit\" value=\"retour\"></a><br>");
            cli->println("<a><input type=\"submit\" formaction=\"page.html?password__=17515A:\" value=\"formaction\"></a><br>");

            bouTableHtml(cli,"password__",userpass,"boutable",2,1);
            
            cli->print("<form><p hidden><input type=\"text\" name=\"password__\" value=\"");
            cli->print(userpass);
            cli->print("\" ><p><input type=\"submit\" value=\"submit\"><br></form>");
            
            
            cli->print("<a href=page.html?password__=17515A:><input type=\"submit\" value=\"retour\"></a><br>");

            cli->println("</form></body></html>");            
*/
}           
