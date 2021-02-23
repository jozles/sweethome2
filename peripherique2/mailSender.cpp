#include <ESP8266WiFi.h>
#include "utilWifi.h"

#define SMTP 'S'

#if SMTP == 'S'
WiFiClient mailClient;
char mailServer[] = "mail.smtp2go.com";
uint16_t mailPort=2525;
#endif

#if SMTP == 'G'
#include <WiFiClientSecure.h>

WiFiClientSecure mailClient;
char mailServer[] = "smtp.gmail.com";
uint16_t mailPort=465;
#endif

extern const char* ssid;
extern const char* password;


/* sender */
#define B64ADDRESS  "bHVjaWVsaXU2NkBnbWFpbC5jb20="
#define B64PASSWORD "ZWljdWw2NjY="
String from="lucieliu66@gmail.com";

String smtpRx;


/*
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("8266 - email");
  Serial.print(mailServer);Serial.print(" ");Serial.println(mailPort);

    if(wifiConnexion(ssid,password)){

      sujet="test mail 8266";
      dest="lucieliu66@gmail.com";
      from="lucieliu66@gmail.com";
      mess="message de test";

      if(cxServer(mailServer,mailPort) && sendEmail(sujet,dest,from,mess)){             
        Serial.println("Message sent");
      } 
    mailClient.stop();
    Serial.println("disconnected");
    }
}
*/


bool cxServer(char* server,uint16_t port)
{
  Serial.print("Connecting to :");
  Serial.println(server); 
  
  if (mailClient.connect(server, port)) {Serial.println(" connected");}
  else {Serial.println(" failed");return false;}
}

bool getServer(char* code,char* rxCode)
{
  long tobeg=millis();

  #define TIMEOUT 5000

  while (!mailClient.available()){
    if((millis()-tobeg)>TIMEOUT){
      mailClient.stop();
      Serial.println(F(" smtp timeout"));
      return false;
    }
  }

  smtpRx=mailClient.readStringUntil('\n');
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
  mailClient.println("HELO");
  if(!getServer("250",rxCode)){return false;}

/*  mailClient.println("STARTTLS");
  if (!emailResp())
  return 0;*/

  Serial.println("AUTH LOGIN");
  mailClient.println("AUTH LOGIN");
  getServer("",rxCode);

  Serial.println("B64 ADDRESS");
  mailClient.println(B64ADDRESS);
  getServer("",rxCode);
  
  Serial.println("B64 PASSWORD");
  mailClient.println(B64PASSWORD);
  if(!getServer("235",rxCode)){return false;}

  Serial.print("MAIL From:");
  Serial.println(fromAddress);
  mailClient.print("MAIL From:");
  mailClient.println(fromAddress);
  getServer("",rxCode);

  Serial.print("RCPT To:");
  Serial.println(destAddress);
  mailClient.print("RCPT TO:");
  mailClient.println(destAddress);
  getServer("",rxCode);

  Serial.println("DATA:");
  mailClient.print("DATA");
  if(!getServer("354",rxCode)){return false;};

  Serial.println(F("Sending email"));
  mailClient.println("To: ");mailClient.println(destAddress);
  mailClient.println("From: ");mailClient.println(fromAddress);
  mailClient.println("Subject: ");mailClient.println(sujet);
  mailClient.println(message);
  mailClient.println(".");
  if(!getServer("250",rxCode)){return false;}
  
  Serial.println("QUIT");
  mailClient.println("QUIT");
  if(!getServer("221",rxCode)){return false;}
  
  return true;
}

bool mail(String sujet,String destAddress,String message)
{
    if(wifiConnexion(ssid,password) && cxServer(mailServer,mailPort) && sendEmail(sujet,destAddress,from,message)){             
        Serial.print("Mail sent ");
      } 
    mailClient.stop();
    Serial.println("mailClient stopped");
}  
