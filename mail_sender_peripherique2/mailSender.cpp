#include <ESP8266WiFi.h>
#include "utilWifi.h"

#define SMTP 'G'

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


#if SMTP == 'F'
#include <WiFiClientSecure.h>
WiFiClientSecure mailClient;
char mailServer[] = "smtp.free.fr";
uint16_t mailPort=465;
#endif

extern const char* ssid;
extern const char* password;


/* sender */

String from=        "lucieliu66@gmail.com";
#define B64ADDRESS  "bHVjaWVsaXU2NkBnbWFpbC5jb20="
#define B64PASSWORD "ZWljdWw2NjY="

/*
String from=        "jozles@hotmail.fr";
#define B64ADDRESS  "am96bGVzQGhvdG1haWwuZnI="
*/
/*
String from =       "shpinks@free.fr";
#define B64ADDRESS  "c2hwaW5rc0BmcmVlLmZy"
#define B64PASSWORD "c3NTZWxhem5vZzQ0NEA="
*/

String smtpRx;

bool cxServer(char* server,uint16_t port)
{
  Serial.print("Connecting to :");Serial.print(server); 
  
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

  Serial.println("EHLO combox.fr");
  mailClient.println("EHLO combox.fr");
  if(!getServer("250",rxCode)){return false;}

/*  mailClient.println("STARTTLS");
  if (!emailResp())
  return 0;*/

#if SMTP != 'S'
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}
  if(!getServer("250",rxCode)){return false;}

  Serial.println("AUTH LOGIN");
  mailClient.println("AUTH LOGIN");
  getServer("",rxCode);

  Serial.println(B64ADDRESS);
  mailClient.println(B64ADDRESS);
  getServer("",rxCode);
  
  Serial.println(B64PASSWORD);
  mailClient.println(B64PASSWORD);
  if(!getServer("235",rxCode)){return false;}
#endif
  
  String mailFrom = "MAIL FROM: <" + String(fromAddress) + '>';
  Serial.println(mailFrom);
  mailClient.println(mailFrom);
  if(!getServer("250",rxCode)){return false;}

  String rcpt = "RCPT TO: <" + String(destAddress) + '>';
  Serial.println(rcpt);
  mailClient.println(rcpt);
  if(!getServer("250",rxCode)){return false;}

  Serial.println("DATA:");
  mailClient.print("DATA");
  if(!getServer("250",rxCode)){return false;};

  rcpt = "To: <" + destAddress + '>';
  Serial.println(rcpt);
  mailClient.println(rcpt);
  
  mailFrom = "From: <" + String(fromAddress) + '>';
  Serial.println(mailFrom);
  mailClient.println(mailFrom);
  
  String subj = "Subject: " + sujet + "\r\n";
  Serial.println(subj);
  mailClient.println(subj);

  Serial.println(message);
  mailClient.print(message);
  mailClient.print("\r\n");
  
  Serial.println(".");
  mailClient.print(".\r\n");
  if(!getServer("250",rxCode)){return false;}

  Serial.println("QUIT");
  mailClient.println("QUIT");
  if(!getServer("221",rxCode)){return false;}
  
/*  
  Serial.println("QUIT");
  mailClient.println("QUIT");
  if(getServer("221",rxCode)){return true;}
  if(memcmp("250",rxCode,3)!=0){return false;}

  Serial.println("AUTH LOGIN");
  mailClient.println("AUTH LOGIN");
  getServer("",rxCode);

  Serial.println(B64ADDRESS);
  mailClient.println(B64ADDRESS);
  getServer("",rxCode);

  Serial.println(B64PASSWORD);
  mailClient.println(B64PASSWORD);
  if(!getServer("235",rxCode)){return false;}

  Serial.println("QUIT");
  mailClient.println("QUIT");
  if(!getServer("221",rxCode)){return false;}
*/
  
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
