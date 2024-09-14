
#include <shconst2.h>
#include <shutil2.h>
#ifndef ESP32
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <Wifi.h>
#endif
#include "const.h"

extern constantValues cstRec;

extern uint8_t  nbreBlink;

extern byte     mac[6];

#define LWIFIST 18
const char* wifiSta="WL_IDLE_STATUS\0  \0WL_NO_SSID_AVAIL \0WL_UKN\0          \0WL_CONNECTED\0    \0WL_CONNECT_FAILED\0WL_UKN\0          \0WL_DISCONNECTED  \0";
const char* wifiSta255="NO_SHIELD\0";

void wifiStatusValues()
{
  Serial.print(WL_CONNECTED);Serial.println(" WL_CONNECTED ");
  Serial.print(WL_CONNECT_FAILED);Serial.println(" WL_CONNECT_FAILED ");
  Serial.print(WL_DISCONNECTED);Serial.println(" WL_DISCONNECTED ");
  Serial.print(WL_IDLE_STATUS);Serial.println(" WL_IDLE_STATUS ");
  Serial.print(WL_NO_SSID_AVAIL);Serial.println(" WL_NO_SSID_AVAIL ");
}

int printWifiStatus(const char* ssid,bool print)
{
  int ws=WiFi.status();       // (ESP32)~120uS
  
  if(print){
    //Serial.println();
    Serial.print(millis());
    //Serial.print(" WiFiStatus=");
    Serial.print(" ");
    Serial.print(ws);Serial.print(" ");
    if(ws!=255){Serial.print((char*)(wifiSta+LWIFIST*ws));}
    else Serial.print((char*)wifiSta255);
    if(ssid!=nullptr){Serial.print(" to ");Serial.print(ssid);}
  }
  return ws;
}

int printWifiStatus(const char* ssid)
{
  return printWifiStatus(ssid,PRINT);
}

int printWifiStatus()
{
  return printWifiStatus(nullptr,PRINT);
}

bool wifiConnexion(const char* ssid,const char* password,bool print)
{

  unsigned long beg=micros();
  //unsigned long bg0,bg1,bg2,bg3;
  bool cxstatus=VRAI;

    //WiFi.forceSleepWake();delay(1);
    //WiFi.forceSleepEnd();       // réveil modem
    
    if(ssid[0]==' '){return false;}
    
    int wifistatus=printWifiStatus(ssid,print);

    if(wifistatus!=WL_CONNECTED){    
/*    
      NOSHIELD par défaut au reset
      WL_CONNECTED after successful connection is established
      WL_NO_SSID_AVAIL in case configured SSID cannot be reached
      WL_CONNECT_FAILED if password is incorrect
      WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
      WL_DISCONNECTED if module is not configured in station mode
*/
      ledblink(BCODEONBLINK,PULSEBLINK);
      
      Serial.print("\n WIFI connecting to ");Serial.println(ssid);
      yield();
      WiFi.begin(ssid,password);
      
      while(WiFi.status() != WL_CONNECTED){     // try to connect
        if((millis()-beg/1000)>WIFI_TO_CONNEXION){cxstatus=FAUX;break;}
        delay(500);wifistatus=printWifiStatus();
      }                                         // cxstatus ok or ko
    }
    if(cxstatus){                               // ok
      ledblink(-1,PULSEBLINK);
      cstRec.IpLocal=WiFi.localIP();
      WiFi.macAddress(mac);
      if(print){
        //Serial.print(" local IP : ");delay(10);
        Serial.print(" ");
        //bg0=micros();
        Serial.print(cstRec.IpLocal);
        //bg1=micros();
        Serial.print(" ");
        //bg2=micros();
        serialPrintMac(mac,0);
        //bg3=micros();
      }
    }
    else {Serial.print(" failed");
    #if POWER_MODE==NO_MODE
    delay(10000);while(1){};
    #endif  // NO_MODE
    #if POWER_MODE!=NO_MODE
    if(nbreBlink==0){ledblink(BCODEWAITWIFI,PULSEBLINK);}
    #endif // != NO_MODE
    }
    if(print){
      unsigned long v=micros()-beg;
      //unsigned long v0=micros()-bg0;
      //unsigned long v1=micros()-bg1;
      //unsigned long v2=micros()-bg2;
      //unsigned long v3=micros()-bg3;
      Serial.print(" cxst=");Serial.print(cxstatus);
      Serial.print(" uS=");Serial.print(v);
      //Serial.print(' ');Serial.print(v0);Serial.print(' ');Serial.print(v1);Serial.print(' ');Serial.print(v2);Serial.print(' ');Serial.print(v3);Serial.print(' ');
    }
  
    return cxstatus;
}

bool wifiConnexion(const char* ssid,const char* password)
{
  return wifiConnexion(ssid,password,NOPRINT);
}


void modemsleep()
{
  WiFi.disconnect();
#ifndef ESP32  
  WiFi.forceSleepBegin();
#endif
  delay(100);
}

void htmlImg(WiFiClient* cli,char* data,int dataLen)
{
        cli->write("HTTP/1.1 200 OK\n");
        cli->write("CONTENT-Type: image/jpg\n\n");
        
        //for(int i=0;i<dataLen;i++){cli->write(data[i]);}
        data[dataLen]='\0';cli->write(data);

        Serial.println(" terminé");
        cli->stop();
        return;
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
