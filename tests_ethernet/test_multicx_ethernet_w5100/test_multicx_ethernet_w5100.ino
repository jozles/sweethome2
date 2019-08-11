
#include <SPI.h>      // W5100 SPI
#include <Ethernet.h> // W5100 Ethernet

extern "C" {
 #include "utility/w5100.h"
}

  
EthernetClient cli_a;
EthernetClient cli_b;
    
/* extra server */
//IPAddress host(74,125,232,128);
//int port=80;
IPAddress host(192,168,0,38);
int port=1791;

/* local server */
byte mac[]="\x90\xA2\xDA\x0F\xDF\xAB";
IPAddress localIp(192,168,0,35);
EthernetServer localServer(1790);


void getremote_IP(EthernetClient *client,uint8_t* remote_IP,byte* remote_MAC)
{
    W5100.readSnDHAR(client->getSocketNumber(), remote_MAC);
    W5100.readSnDIPR(client->getSocketNumber(), remote_IP);
}


void setup() {

Serial.begin(115200);
Serial.println("ready");

Ethernet.begin (mac, localIp);
Serial.println(Ethernet.localIP());

localServer.begin();

}

void loop() {

uint8_t remote_MAC[6]={0,0,0,0,0,0}; 
uint8_t remote_IP[4]={0,0,0,0};       

      if(cli_a = localServer.available()){
        
        if(cli_a.connected()){
          Serial.println("cli_a.connected");
          getremote_IP(&cli_a,remote_IP,remote_MAC);
          for(int i=0;i<=3;i++){Serial.print(remote_IP[i]);
            if(i<3){Serial.print(".");}}
          Serial.println();
      
cli_a.stop();
cli_b.stop();
          Serial.print("connecting ");Serial.print(host);
          Serial.print("/");Serial.println(port);
         
          if(cli_b.connect(host,port)){
            Serial.println("cli_b.connected(1)");
          }
cli_b.stop();          
          if(cli_b.connect(host,port)){
            Serial.println("cli_b.connected(2)");
          }
          else Serial.println("fail");
      
          while(1){};
        }
      }

}
