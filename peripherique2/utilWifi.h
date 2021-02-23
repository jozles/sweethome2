#ifndef UTIL_WIFI_H_INCLUDED
#define UTIL_WIFI_H_INCLUDED


bool  wifiConnexion(const char* ssid,const char* password);
int   printWifiStatus();
void  wifiStatusValues();
void  modemsleep();


#endif // UTIL_WIFI_H_INCLUDED
