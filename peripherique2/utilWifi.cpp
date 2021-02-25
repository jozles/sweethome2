
#include "const.h"
#include <ESP8266WiFi.h>
#include <shconst2.h>
#include <shutil2.h>

extern constantValues cstRec;

extern uint8_t  nbreBlink;

extern byte     mac[6];

void wifiStatusValues()
{
  Serial.print(WL_CONNECTED);Serial.println(" WL_CONNECTED ");
  Serial.print(WL_CONNECT_FAILED);Serial.println(" WL_CONNECT_FAILED ");
  Serial.print(WL_DISCONNECTED);Serial.println(" WL_DISCONNECTED ");
  Serial.print(WL_IDLE_STATUS);Serial.println(" WL_IDLE_STATUS ");
  Serial.print(WL_NO_SSID_AVAIL);Serial.println(" WL_NO_SSID_AVAIL ");
}

int printWifiStatus()
{
  char* wifiSta="WL_IDLE_STATUS   \0WL_NO_SSID_AVAIL \0WL_UKN           \0WL_CONNECTED     \0WL_CONNECT_FAILED\0WL_UKN           \0WL_DISCONNECTED  \0";
  int ws=WiFi.status();
  Serial.println();Serial.print(ws);Serial.print(" WiFiStatus=");Serial.print((char*)(wifiSta+18*ws));
  return ws;
}

bool wifiConnexion(const char* ssid,const char* password)
{

  int i=0;
  unsigned long beg=micros();
  bool cxstatus=VRAI;

    ledblink(BCODEONBLINK);

    //WiFi.forceSleepWake();delay(1);
    //WiFi.forceSleepEnd();       // réveil modem
    
    int wifistatus=printWifiStatus();
    if(wifistatus!=WL_CONNECTED){    
/*
      WL_CONNECTED after successful connection is established
      WL_NO_SSID_AVAIL in case configured SSID cannot be reached
      WL_CONNECT_FAILED if password is incorrect
      WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
      WL_DISCONNECTED if module is not configured in station mode
*/
      unsigned long startcx=millis();  
      Serial.print(" WIFI connecting to ");Serial.print(ssid);
      WiFi.begin(ssid,password);
      
      //wifistatus=printWifiStatus();
      //while(wifistatus!=WL_CONNECTED){
      while(WiFi.status() != WL_CONNECTED){
        if((millis()-beg/1000)>WIFI_TO_CONNEXION){cxstatus=FAUX;break;}
        delay(500);wifistatus=printWifiStatus();
      }
    }
    if(cxstatus){
      if(nbreBlink==BCODEWAITWIFI){ledblink(BCODEWAITWIFI+100);}
      Serial.print(" local IP : ");Serial.print(WiFi.localIP());
      cstRec.IpLocal=WiFi.localIP();        
      WiFi.macAddress(mac);
      Serial.print(" ");serialPrintMac(mac,1);
      cstRec.serverPer=PERSERV;
      }
    else {Serial.print(" failed");if(nbreBlink==0){ledblink(BCODEWAITWIFI);}}
    Serial.print(" cxtime(micros)=");Serial.println(micros()-beg);
    return cxstatus;
}

void modemsleep()
{
  WiFi.disconnect();
  WiFi.forceSleepBegin();
  delay(100);
}



/*uint8_t WiFiConnect(const char* ssid, const char* password)
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

*/
