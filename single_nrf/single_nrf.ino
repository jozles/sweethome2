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
unsigned long read_beg;
unsigned long readTo;       // compteur TO read()

unsigned long time_beg=millis();
unsigned long time_end;
byte    message[MAX_PAYLOAD_LENGTH+1];   // buffer général pour nrfp.read()
int     numP;
uint8_t pipe;
uint8_t pldLength;

int     confSta=0;          // pour 'P' numP si inscription ok
byte    sta;                // status reg 

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

void showRx();
void ledblk(uint8_t dur,uint16_t bdelay,uint8_t bint,uint8_t bnb);
int  txMessage(char* message,char ack,uint8_t len,uint8_t numP);
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

  beginP();                                 // pRegister()
  
#endif NRF_MODE == 'P'

#if NRF_MODE == 'C'

  hardwarePowerUp();                                
  nrfp.setup();
  
  while((millis()-time_beg)<800){ledblk(TBLK,1000,80,4);}          // 0,8sec (4 blink)

  nrfp.tableCInit();//nrfp.tableCPrint();
  
#endif NRF_MODE == 'C'

  Serial.println("end setup");delay(1);
}

void loop() {

#if NRF_MODE == 'P'

  while((awakeMinCnt>=0)&&(awakeCnt>=0)&&(retryCnt==0)){
    awakeCnt--;
    awakeMinCnt--;
    pinMode(LED,OUTPUT);digitalWrite(LED,HIGH);delayMicroseconds(500);digitalWrite(LED,LOW); // 1mA rc=0,5/8000 -> 62nA
    sleepPwrDown(T8000);
  }

  awakeCnt=aw_ok;
  mustSend=false;           

  /* Here data acquisition/checking/treatment (if something to send put mustSend true) */

  mustSend=checkThings(retryCnt);

  /* End of data acquisition/checking/treatment */

  /***** now sleep and com staff ****/

    tBeg=((float)millis()/1000)+(durT/100);
    Serial.print(nbS);Serial.print("(");Serial.print(tBeg);Serial.print(") ");
    Serial.print(awakeCnt);Serial.print(" ");
    Serial.print(awakeMinCnt);Serial.print(" ");Serial.println(retryCnt);

  /* txdata ready to send or presence message to send -> send */
  if( (mustSend==true) || (awakeMinCnt<0) || (retryCnt!=0)){

    hardwarePowerUp();                   // 5mS delay inside

    uint8_t outLength=0;
    char    outMessage[MAX_PAYLOAD_LENGTH+1];
    int     trst;     // status retour txMessage
    int     gdSt=0;   // status retour read

    /* building message MMMMMPssssssssVVVV....... MMMMMP should not be changed */
    memcpy(outMessage+outLength,MAC_ADDR,ADDR_LENGTH);                     // macAddr
    outLength+=ADDR_LENGTH;
    outMessage[outLength]=numP+48;                                         // numP du périphérique
    outLength++;
    memcpy(outMessage+outLength,VERSION,LENVERSION);                      // version
    sprintf(outMessage+outLength,"%08d",(uint32_t)tBeg);                  // seconds since last reset 
    outLength+=8;

    /* Here add user data (be carefull to not override 32 bytes) */
    messageBuild(message,&outLength);
    /* End of add user data (be carefull to not override 32 bytes) */

    /* send message */
    trst=txMessage(outMessage,'N',outLength,0);  
    if(trst<0){
      // si le message ne part pas, essai aux (aw_retry-1) prochains réveils 
      // puis après aw_ko réveils
      Serial.print((char*)outMessage);Serial.println(" ********** maxRT ");
      switch(retryCnt){
        case 0:retryCnt=aw_retry;break;
        case 1:awakeCnt=aw_ko;awakeMinCnt=aw_ko;awakeCnt=0;break;
        default:retryCnt--;break;
      }
    }
    else{                 // transmit ok 
      /* every counters reset */
      retryCnt=0;
      awakeCnt=aw_ok;
      awakeMinCnt=aw_min;
      
      /* get response */
      pldLength=MAX_PAYLOAD_LENGTH;                    // max length
      int rdSta=AV_EMPTY;
      read_beg=millis();
      while((rdSta==AV_EMPTY)&& (readTo>=0)){
        rdSta=nrfp.read(message,&pipe,&pldLength,NBPERIF);
        readTo=TO_READ-millis()+read_beg;}
      Serial.print("rdsta=");Serial.println(rdSta);  
      if(rdSta>=0 && readTo>0){                        // no error
        showRx();

        /* Here received data to local fields transfer */
        importData(message,pldLength);
        /* End of received data to local fields transfer */
      }
      else{                                            // rx error
        if(rdSta>=0){rdSta=ER_RDYTO;}
        Serial.print(" response err ");Serial.println((char*)kk+(rdSta+6)*LMERR);}    // error
    }
  }
  delay(1);  // serial

#endif // NRF_MODE == 'P'

#if NRF_MODE == 'C'

  if(menu){
    Serial.println("(e)cho (b)roadcast (t)ableC (q)uit");
    menu=false;
  }

  ledblk(TBLK,2000,IBLK,1);

  pldLength=MAX_PAYLOAD_LENGTH;                   // max length
  numP=nrfp.read(message,&pipe,&pldLength,NBPERIF);
  if(numP>=0){                                    // no error
    showRx();
    if(numP==0){                                  // register request
      byte pAd[ADDR_LENGTH];             
      uint8_t numP=nrfp.cRegister(message);
      if(numP<(NBPERIF)){
        memcpy(pAd,tableC[numP].periMac,ADDR_LENGTH);
        Serial.print(" registred on addr (");Serial.print(numP);Serial.print(")");nrfp.printAddr(pAd,'n');}
      else if(numP==(NBPERIF+2)){Serial.print(" MAX_RT ... deleted");}
      else {Serial.print(" full");}
      Serial.println();
    }
        
    else {                                        // pas demande d'inscription... send config
      Serial.println();
      /* store incoming message */
      memcpy(tableC[numP].periBuf,message,pldLength);   
      tableC[numP].periBufLength=pldLength;
      /* build config */
      memcpy(message+ADDR_LENGTH+1,tableC[numP].serverBuf,MAX_PAYLOAD_LENGTH-ADDR_LENGTH-1);
      /* send config message */
      int trst=txMessage(message,'N',MAX_PAYLOAD_LENGTH,numP);
      if(trst==0){tableC[numP].periBufSent=true;}
      // reformater et transmettre au serveur le message reçu 
    }
  }

  else if(numP!=AV_EMPTY){
    showRx();
    Serial.print(" err ");Serial.println((char*)kk+(numP+6)*LMERR);}    // error... à traiter
    
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
    echo0(message,5,numP);
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

void showSta(uint8_t nb)
{
  Serial.print(nb);
  Serial.print(" >>> sta ");
  nrfp.regRead(7,&sta);
  Serial.println(sta,HEX);
}

void broadcast(char a)
{           
  Serial.println((char)a);
 
  sprintf(message,"%08lu",millis());

  echo0(message,8,1);
}

void echo0(char* message,uint8_t len,uint8_t numP)
{                       // txMessage + read réponse (1er perif de la table)
  message[len]='+';
  message[len+1]='\0';

  int trSta=-1;
  trSta=txMessage(message,'N',len,numP);
  
  if(trSta<0){Serial.print("********** maxRT ");
  }
  else{
    memset(message,0x00,MAX_PAYLOAD_LENGTH+1);
    pldLength=MAX_PAYLOAD_LENGTH;                   // max length
    readTo=0;
    numP=AV_EMPTY;
    long read_beg=millis();
    while((numP==AV_EMPTY)&& (readTo>=0)){
      numP=nrfp.read(message,&pipe,&pldLength,NBPERIF);
      readTo=TO_READ-millis()+read_beg;}    
    time_end=micros();
    if(numP>=0){         // data ready, no error
      showRx();
    }
    else{Serial.print("error ");Serial.print((char*)kk+(numP+ER_MAXER)*LMERR);} 
  }
  Serial.print(" in:");Serial.print((long)(time_end - time_beg));Serial.println("us");
}

#endif NRF_MODE == 'C'

#if NRF_MODE == 'P'

void beginP()
{
  confSta=-1;
  while(confSta<0){
    confSta=nrfp.pRegister(); // -4 empty -3 max RT ; -2 len ; -1 pipe ; 0 na ; 1 ok
    Serial.print("start ");
    Serial.print((char*)(kk+(confSta+6)*LMERR));//Serial.print((confSta-ER_MAXER)*3);
    Serial.print(" numP=");Serial.println(confSta);
    
    delayBlk(TBLK,DBLK,IBLK,1,1000);
  }
  numP=confSta;
}

ISR(WDT_vect)                      // ISR interrupt service pour vecteurs d'IT du MPU (ici vecteur WDT)
{
  reveil = true;
}
 
#endif NRF_MODE == 'P'

int txMessage(char* message,char ack,uint8_t len,uint8_t numP)
{
#if NRF_MODE=='P'  
  if(numP==0){beginP();}
#endif NRF_MODE=='P'
  
  Serial.print((char*)message);delay(1);
  
  nrfp.write(message,ack,len,numP);
  int trSta=1;
  time_beg = micros();
  while(trSta==1){
    trSta=nrfp.transmitting();}

  time_end=micros();  
  Serial.print(" tx to ");  
  Serial.print(numP);Serial.print(" in:");
  Serial.print((long)(time_end - time_beg)); 
  Serial.println("us");
delay(1);
  return trSta;
}

void showRx()
{  
  Serial.print((char*)message);
  Serial.print(" l=");Serial.print(pldLength);
  Serial.print(" p=");Serial.print(pipe);
  Serial.print(" numP=");Serial.print(numP);
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
