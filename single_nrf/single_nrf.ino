#include "nRF24L01.h"
#include "nrf24l01s.h"
#include "nrf24l01s_const.h"
#include "nrf_powerSleep.h"
#include "shconst2.h"
#include "shutil2.h"
#include "nrf_user.h"


#if NRF_MODE == 'C'
extern struct NrfConTable tableC[NBPERIF];
bool menu=true;
#endif NRF_MODE == 'C'

Nrfp nrfp;

/*** LED ***/
uint16_t blkdelay=0;
long     blktime=0;
uint8_t bcnt=0;
#define TBLK 1
#define DBLK 2000
#define IBLK 80

/*** scratch ***/

#define TO_READ 1000
unsigned long readTo;                     // compteur TO read()
unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];    // buffer pour read()/write()
int     rdSta;                            // return status read() / available()
int     trSta;                            // return status write() / transmitting()
uint8_t pipe;
uint8_t pldLength;

uint8_t numT=0;                           // numéro périphérique dans table concentrateur

byte    testAd[]={"testx"};               // txaddr pour broadcast

#define LMERR 9           
char*   kk={"time out\0tx maxrt\0rx empty\0mac addr\0length  \0pipe nb \0--      \0ok      \0"};         // codes retour et erreur

#if NRF_MODE == 'P'

/*** gestion sleep ***/
bool    reveil;
bool    mustSend; 
int     awakeCnt=0;
int     awakeMinCnt=0;
int     retryCnt=0;
uint32_t nbS=0;                   // nbre sleeps
float   durT=0;                   // temps sleep cumulé (mS/10)
float   tBeg=0;                   // temps total depuis reset (oscillateur local)

uint16_t aw_ok=AWAKE_OK_VALUE;
uint16_t aw_min=AWAKE_MIN_VALUE;
uint16_t aw_ko=AWAKE_KO_VALUE;
uint8_t  aw_retry=AWAKE_RETRY_VALUE;

#endif NRF_MODE == 'P'

/* prototypes */

void showErr();
void showRx();
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
int  txMessage(bool ack,uint8_t len,uint8_t numP);
int  rxMessage();
void echo0(char* message,bool ack,uint8_t len,uint8_t numP);
#if NRF_MODE == 'C'
char getch();
void echo();
void broadcast();
#endif NRF_MODE == 'C'

void hardwarePowerUp()
{
  nrfp.powerUp();
  pinMode(LED,OUTPUT);
}
 

void setup() {
  
  Serial.begin(115200);

#if NRF_MODE == 'P'
  
  userHardSetup();
  hardwarePowerUp();
  nrfp.setup();
                                  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}        // 0,8sec (4 blink)

  numT=beginP();                          // registration
  
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  hardwarePowerUp();                                
  nrfp.setup();
  nrfp.tableCInit();//nrfp.tableCPrint();
  memcpy(tableC[1].periMac,testAd,ADDR_LENGTH);     // pour broadcast
  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}          // 0,8sec (4 blink)
  
#endif NRF_MODE == 'C'

  Serial.println("end setup");delay(1);
}

void loop() {

#if NRF_MODE == 'P'

  /* timing to usefull awake */
  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){           
    awakeCnt--;
    awakeMinCnt--;
    pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delayMicroseconds(500);digitalWrite(LED,LOW); // 1mA rc=0,5/8000 -> 62nA
    sleepPwrDown(T8000);
  }

  /* usefull awake */
  awakeCnt=aw_ok;
  mustSend=false;           
  mustSend=checkThings(retryCnt);           // user staff

  tBeg=((float)millis()/1000)+(durT/100);
  Serial.print(nbS);Serial.print("(");Serial.print(tBeg);Serial.print(") ");
  Serial.print(awakeCnt);Serial.print(" ");
  Serial.print(awakeMinCnt);Serial.print(" ");Serial.println(retryCnt);

  /* data ready to send or presence message to send -> send */
  if( (mustSend==true) || (awakeMinCnt<0) || (retryCnt!=0)){

    hardwarePowerUp();                   // 5mS delay inside

    /* building message MMMMMPssssssssVVVV....... MMMMMP should not be changed */
    uint8_t outLength=0;
    memcpy(message+outLength,MAC_ADDR,ADDR_LENGTH);                   // macAddr
    outLength+=ADDR_LENGTH;
    message[outLength]=numT+48;                                       // numéro du périphérique
    outLength++;
    memcpy(message+outLength,VERSION,LENVERSION);                     // version
    sprintf(message+outLength,"%08d",(uint32_t)tBeg);                 // seconds since last reset 
    outLength+=8;
    messageBuild(message,&outLength);                                 // add user data

    /* send message */
    if(txMessage(true,outLength,0)<0){      // tx error
      // si le message ne part pas, essai aux (aw_retry-1) prochains réveils 
      // puis après aw_ko réveils
      Serial.print((char*)message);Serial.println(" ********** maxRT ");
      switch(retryCnt){
        case 0:retryCnt=aw_retry;break;
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;awakeCnt=0;break;
        default:retryCnt--;break;
      }
    }
    else{                                  // tx ok 
      /* every counters reset */
      retryCnt=0;
      awakeCnt=aw_ok;
      awakeMinCnt=aw_min;
      
      /* get response */
      if(rxMessage()>=0){                  // rx ok
        showRx();
        importData(message,pldLength);     // user data
      }
      else{showErr();}                // rx error
    }
  }
  delay(1);  // serial

#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'

  if(menu){
    nrfp.printAddr(MAC_ADDR,0);
    Serial.println(" (e)cho (b)roadcast (t)ableC (q)uit");
    menu=false;
  }

  ledblk(TBLK,2000,IBLK,1);

  pldLength=MAX_PAYLOAD_LENGTH;                     // max length
  rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);
  if(rdSta>=0){                                     // no error
    showRx();
    numT=rdSta;                                     // rdSta >=0 -> numT du périphérique       
    if(numT==0){                                    // registration request
      byte pAd[ADDR_LENGTH];             
      uint8_t numT=nrfp.cRegister(message);
      if(numT<(NBPERIF)){                           // numT valid  
        memcpy(pAd,tableC[numT].periMac,ADDR_LENGTH);
        Serial.print(" registred on addr (");Serial.print(numT);Serial.print(")");nrfp.printAddr(pAd,'n');}
      else if(numT==(NBPERIF+2)){Serial.print(" MAX_RT ... deleted");}
      else {Serial.print(" full");}
      Serial.println();
    }
        
    else {                                        // already registred -> send config
      Serial.println();
      /* store incoming message */
      memcpy(tableC[numT].periBuf,message,pldLength);   
      tableC[numT].periBufLength=pldLength;
      /* build config */
      memcpy(message+ADDR_LENGTH+1,tableC[numT].serverBuf,MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1);
      /* send config message */
      
      if(txMessage(true,MAX_PAYLOAD_LENGTH,numT)==0){tableC[numT].periBufSent=true;}
      // reformater et transmettre au serveur le message reçu 
    }
  }

  else if(rdSta!=AV_EMPTY){
    showRx();
    showErr();}    // error... à traiter
    
  char a=getch();
  switch(a){
    case 'e':echo(a);menu=true;break;
    case 'b':broadcast(a);menu=true;break;
    case 't':Serial.println((char)a);nrfp.tableCPrint();menu=true;break;
    default:break;
  }
#endif // NRF_MODE == 'C'

} /********** loop end  *********/

#if NRF_MODE == 'C'

void echo(char a)
{
  int cnt=0;
  int cntko=0;

  Serial.println((char)a);
  
  while (getch()!='q'){

    uint8_t numP=1; // 1er perif de la table
    cnt++;
    sprintf(message,"%05d",cnt);
    echo0(message,false,5,numP);
                                                // si periMac ne répond plus ... à voir

    delay(2000);ledblk(TBLK,DBLK,IBLK,2);
  } 
}

char getch()
{
  if(Serial.available()){
    return Serial.read();
  }
  return 0;
}

void broadcast(char a)
{
  bool ack=true,titre=true;
  char b;
  Serial.println((char)a);
  while(1){
    if(titre==true){
      titre=false;
      if(!ack){Serial.print('!');}
      Serial.print("ACK ... saisir l'adresse (rien=");
      nrfp.printAddr(tableC[1].periMac,0);Serial.println(" ; !=toogle ack ; q=exit)");}
    b=getch();
    switch (b){
      case 0:break;
      case '!':ack=!ack;titre=true;break;
      case 'q':return;
      default:
        if(b!=' '){
          testAd[0]=b;for(int i=1;i<ADDR_LENGTH;i++){testAd[i]=getch();}
          memcpy(tableC[1].periMac,testAd,ADDR_LENGTH);}
        sprintf(message,"%08lu",millis());
        echo0(message,ack,8,1);
        titre=true;
        break;
    }
  }
}

void echo0(char* message,bool ack,uint8_t len,uint8_t numP)
{                       // txMessage + read réponse (1er perif de la table)
  message[len]='+';
  message[len+1]='\0';

  int trSta=-1;
  
  if(txMessage(ack,len+1,numP)<0){Serial.print("********** maxRT ");}
  else{
    if(rxMessage()>=0){showRx();}         // data ready, no error
    else{showErr();} 
  }
}

#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'

uint8_t beginP()
{
  int confSta=-1;
  while(confSta<0){
    confSta=nrfp.pRegister(); // -4 empty -3 max RT ; -2 len ; -1 pipe ; 0 na ; 1 ok
    Serial.print("start ");
    Serial.print((char*)(kk+(confSta+6)*LMERR));
    Serial.print(" numP=");Serial.println(confSta);
    
    delayBlk(TBLK,DBLK,IBLK,1,1000);
  }
  return confSta;                // le périphérique est inscrit
}

ISR(WDT_vect)                      // ISR interrupt service pour vecteurs d'IT du MPU (ici vecteur WDT)
{
  reveil = true;
}
 
#endif NRF_MODE == 'P'

int txMessage(bool ack,uint8_t len,uint8_t numP)
{
#if NRF_MODE=='P'  
  if(numT==0){numT=beginP();}
#endif NRF_MODE=='P'
  
  Serial.print((char*)message);delay(1);
  
  nrfp.write(message,ack,len,numP);
  trSta=1;
  time_beg = micros();
  while(trSta==1){
    trSta=nrfp.transmitting();}

  time_end=micros();  
  Serial.print(" tx to ");Serial.print(numP);
  Serial.print(" trSta=");Serial.print(trSta);
  Serial.print(" in:");Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");
delay(2);
  return trSta;
}

int rxMessage()
{
  Serial.print("rx ");
  memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
  pldLength=MAX_PAYLOAD_LENGTH;                    // max length
  rdSta=AV_EMPTY;
  time_beg = micros();
  while((rdSta==AV_EMPTY)&& (readTo>=0)){
    rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);
    readTo=TO_READ-millis()+time_beg;}
    if(readTo<0){rdSta==ER_RDYTO;}
  time_end=micros();
  Serial.print((char*)message);  
  Serial.print(" rx from ");Serial.print((int)(message[ADDR_LENGTH]-48));
  Serial.print("rdsta=");Serial.print(rdSta);  
  Serial.print(" in:");Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");
delay(2);
  return rdSta;
}

void showRx()
{  
  Serial.print((char*)message);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" rdSta=");Serial.print(rdSta);
}

void showErr()
{
  Serial.print(" err ");Serial.println((char*)kk+(rdSta+6)*LMERR);
}

void delayBlk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb,long dly)
{
  long tt=millis();
  while((millis()-tt)<dly){ledblk(dur,bdelay,bint,bnb);}
}

void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb)
{
  if((millis()-blktime)>blkdelay){
    if(digitalRead(LED)==LOW){
      digitalWrite(LED,HIGH);blkdelay=dur;}
    else{digitalWrite(LED,LOW);
      if(bcnt<bnb){blkdelay=bint;bcnt++;}
      else{blkdelay=bdelay;bcnt=1;}}
    blktime=millis();
  }
}

/*
void dumpstr0(char* data,uint8_t len)
{
    char* chexa="0123456789ABCDEFabcdef\0";
    char a[]={0x00,0x00,0x00};
    uint8_t c;
    Serial.print("   ");Serial.print((long)data,HEX);Serial.print("   ");
    for(int k=0;k<len;k++){Serial.print(chexa[(data[k]>>4)&0x0f]);Serial.print(chexa[data[k]&0x0f]);Serial.print(" ");}
    Serial.print("    ");
    for(int k=0;k<len;k++){
            c=data[k];
            if(c<32 || c>127){c='.';}
            Serial.print((char)c);
    }
    Serial.println();
}

void dumpstr(char* data,uint16_t len)
{
    while(len>=16){len-=16;dumpstr0(data,16);data+=16;}
    if(len!=0){dumpstr0(data,len);}
}

void dumpfield(char* fd,uint8_t ll)
{
    for(int ff=ll-1;ff>=0;ff--){if((fd[ff]&0xF0)==0){Serial.print("0");}Serial.print(fd[ff],HEX);}
    Serial.print(" ");
}
*/
