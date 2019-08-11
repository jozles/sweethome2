
#include <SD.h>
#include <SPI.h>
#include <Ethernet2.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // mac address
//byte ip[]  = {192,168,0,34};                  // ip W5500

IPAddress ip(192, 168, 0,34);

EthernetServer server(1789);                  // port server


//EthernetClient cli;

#define BUFSIZ 250
#define LED    13

long blinktime=millis();
long  blinkwait = 2000;
long  blinkon   = 50;
long  blinktmp  = 0;

int len=0;
char buf[BUFSIZ];

void setup(){

  Serial.begin(115200);delay(1000);Serial.println("+");

  pinMode(LED,OUTPUT);digitalWrite(LED,LOW);delay(1000);digitalWrite(LED,HIGH);delay(1000);digitalWrite(LED,LOW);

/*  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  Serial.print("Starting SD..");
  if(!SD.begin(4)) Serial.println("failed");
  else Serial.println("ok");
*/

  Ethernet.begin(mac,ip);
  
  Serial.println(Ethernet.localIP());
  server.begin();Serial.println("server.begin ");
  Serial.println("Ready");


}


void loop(){

/*  if((millis()-blinktime)>blinktmp && blinktmp==blinkwait){blinktmp=blinkon;blinktime=millis();digitalWrite(LED,!digitalRead(LED));}
  if((millis()-blinktime)>blinktmp && blinktmp==blinkon){blinktmp=blinkwait;blinktime=millis();digitalWrite(LED,!digitalRead(LED));}
*/

if((millis()-blinktime)>blinktmp){
  if(blinktmp==blinkon){blinktmp=blinkwait;}
  else {blinktmp=blinkon;}
  blinktime=millis();digitalWrite(LED,!digitalRead(LED));}
  
  EthernetClient cli = server.available();
    
  if (cli) {
    
    Serial.println("client ");
    memset(buf,0x00,BUFSIZ);
    len=0;
    while (cli.connected()) {
      if (cli.available()) {
        
        char c = cli.read();Serial.write(c);
        if (len < 100) {buf[len]= c;len++;} 
        if (c == '\n') {
        
          cli.println("HTTP/1.1 200 OK");
          cli.println("Content-Type: text/html");
          cli.println("Connection: close");
          cli.println("Refresh: 5");
          cli.println();
          cli.println("<!DOCTYPE HTML ><html>");
          cli.println("<head><title>test server</title></head>");
          cli.println("<body><br>");
          cli.print(millis());
          cli.println("</body>");
          
          cli.println("</html>");
          
          cli.stop();
        }
      }
    }
  } 
}


