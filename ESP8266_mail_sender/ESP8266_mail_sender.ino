#include <ESP8266WiFi.h>

#define SMTP 'S'

#if SMTP == 'S'
WiFiClient espClient;
char server[] = "mail.smtp2go.com";
uint16_t port=2525;
#endif

#if SMTP == 'G'
#include <WiFiClientSecure.h>

WiFiClientSecure espClient;
char server[] = "smtp.gmail.com";
uint16_t port=465;
#endif

const char* ssid = "pinks"; 
const char* password = "cain ne dormant pas songeait au pied des monts";

String smtpRx;
String sujet;
String mess;
String from;
String dest;

uint8_t WiFiConnect(const char* ssid, const char* password)
{
    static uint16_t attempt = 0;
    Serial.print("Connecting to ");Serial.print(ssid);

    WiFi.begin(ssid, password);
    
    uint8_t i = 0;
    while(WiFi.status()!= WL_CONNECTED && i++ < 50){
        delay(200);
        Serial.print(".");
    }
    
    if(i == 51) {Serial.print("failed ");return false;}       
    else {    
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    }
}


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("8266 - email");
  Serial.print(server);Serial.print(" ");Serial.println(port);

    if(WiFiConnect(ssid,password)){

      sujet="test mail 8266";
      dest="lucieliu66@gmail.com";
      from="lucieliu66@gmail.com";
      mess="message de test";

      #define B64ADDRESS  "bHVjaWVsaXU2NkBnbWFpbC5jb20="
      #define B64PASSWORD "ZWljdWw2NjY="

      if(cxServer(server,port) && sendEmail(sujet,dest,from,mess)){             
        Serial.println("Message sent");
      } 
    espClient.stop();
    Serial.println("disconnected");
    }
}


void loop()
{
}

bool cxServer(char* server,uint16_t port)
{
  Serial.print("Connecting to :");
  Serial.println(server); 
  
  if (espClient.connect(server, port)) {Serial.println(" connected");}
  else {Serial.println(" failed");return false;}
}

bool getServer(char* code,char* rxCode)
{
  long tobeg=millis();

  #define TIMEOUT 5000

  while (!espClient.available()){
    if((millis()-tobeg)>TIMEOUT){
      espClient.stop();
      Serial.println(F(" smtp timeout"));
      return false;
    }
  }

  smtpRx=espClient.readStringUntil('\n');
  Serial.println(smtpRx);
  
  if(code[0]!=0){
    int ixc=smtpRx.indexOf(code);
    for(uint8_t i=0;i<3;i++){rxCode[i]=smtpRx[i];}
    rxCode[3]='\0';
    if(ixc == -1){
      char codeList[]="220250235354221";
      int nbc=(strstr(codeList,code)-codeList)/3;
      switch(nbc){
        case 0:Serial.print("cx");break;
        case 1:Serial.print("ident");break;
        case 2:Serial.print("smtp AUTH");break;
        case 3:Serial.print("smtp DATA");break;
        case 4:Serial.print("smtp QUIT");break;
        default:break;
      }
      Serial.print(" error ");Serial.print(code);Serial.print(" / ");Serial.println(rxCode);
      return false;
    }
  }
  return true;
}

bool sendEmail(String sujet,String destAddress,String fromAddress,String message)
{
  char rxCode[4];
  if(!getServer("220",rxCode)){return false;}

  Serial.println("HELO");
  espClient.println("HELO");
  if(!getServer("250",rxCode)){return false;}

/*  espClient.println("STARTTLS");
  if (!emailResp())
  return 0;*/

  Serial.println("AUTH LOGIN");
  espClient.println("AUTH LOGIN");
  getServer("",rxCode);

  Serial.println("B64 ADDRESS");
  espClient.println(B64ADDRESS);
  getServer("",rxCode);
  
  Serial.println("B64 PASSWORD");
  espClient.println(B64PASSWORD);
  if(!getServer("235",rxCode)){return false;}

  Serial.print("MAIL From:");
  Serial.println(fromAddress);
  espClient.print("MAIL From:");
  espClient.println(fromAddress);
  getServer("",rxCode);

  Serial.print("RCPT To:");
  Serial.println(destAddress);
  espClient.print("RCPT TO:");
  espClient.println(destAddress);
  getServer("",rxCode);

  Serial.println("DATA:");
  espClient.print("DATA");
  if(!getServer("354",rxCode)){return false;};

  Serial.println(F("Sending email"));
  espClient.println("To: ");espClient.println(destAddress);
  espClient.println("From: ");espClient.println(fromAddress);
  espClient.println("Subject: ");espClient.println(sujet);
  espClient.println(message);
  espClient.println(".");
  if(!getServer("250",rxCode)){return false;}
  
  Serial.println("QUIT");
  espClient.println("QUIT");
  if(!getServer("221",rxCode)){return false;}
  
  return true;
}
