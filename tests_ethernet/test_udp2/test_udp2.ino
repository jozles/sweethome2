/*
  UDPSendReceive.pde:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender
 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.
 created 21 Aug 2010
 by Michael Margolis
 This code is in the public domain.
 */


#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

IPAddress masterIp(192, 168, 1, 30);
unsigned int masterPort = 8887;

#define SLAVE

#ifndef SLAVE
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
#define IP masterIp
#define PORT masterPort
#endif

#ifdef SLAVE
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
IPAddress slaveIp(192, 168, 1, 31);
#define IP slaveIp
#define PORT 8888
#endif

#define MAX_LENGTH 1000

EthernetUDP Udp;

int i;

char* chexa="0123456789ABCDEFabcdef\0";


void conv_htoa(char* ascii,byte* h)
{
    uint8_t c=*h,d=c>>4,e=c&0x0f;
    ascii[0]=chexa[d];ascii[1]=chexa[e];
}


void dumpstr0(char* data,uint8_t len)
{
    char a[]={0x00,0x00,0x00};
    uint8_t c;
    Serial.print("   ");Serial.print((long)data,HEX);Serial.print("   ");
    for(int k=0;k<len;k++){conv_htoa(a,(byte*)&data[k]);Serial.print(a);Serial.print(" ");}
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




void setup() {

  
  Serial.begin(115200);

  Serial.println("ready ");

  for(i=0;i<6;i++){Serial.print(mac[i],HEX);if(i!=5){Serial.print(":");}}
  Serial.print("  ");
  for(i=0;i<4;i++){Serial.print(IP[i]);if(i!=3){Serial.print(".");}}
  Serial.print("  ");
  Serial.println(PORT);
  Ethernet.begin(mac, IP);
  Udp.begin(PORT);

}

int rxUdp(IPAddress* ipAddr,unsigned int* rxPort,char* data)
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize){
    Serial.print("Received packet of size ");Serial.println(packetSize);
    Serial.print("From ");
    *ipAddr = Udp.remoteIP();
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i<4; i++){Serial.print(remote[i], DEC);if(i<3){Serial.print(".");}}
    *rxPort = Udp.remotePort();
    Serial.print(", port ");Serial.println(*rxPort);

    // read the packet
    Udp.read(data, MAX_LENGTH);
    Serial.println("Contents:");Serial.println(data);
  }
  return packetSize;
}

void txUdp(IPAddress ipAddr,unsigned int port,char* data)
{
  dumpstr((char*)&ipAddr,16);
    Udp.beginPacket(ipAddr,port);
  Serial.println("tx0");
    Udp.write(data);
  Serial.println("tx1");    
    Udp.endPacket();
}

void loop() 
{
  
  IPAddress rxIp;
  unsigned int rxPort;
  char data[MAX_LENGTH];

#ifdef SLAVE
  txUdp(masterIp,masterPort,"hello master !");
  Serial.println("tx");
  while(rxUdp(&rxIp,&rxPort,data)==0){}
  while(1){}
#endif

#ifndef SLAVE
  while(rxUdp(&rxIp,&rxPort,data)==0){}
  txUdp(rxIp,rxPort,"hi slave!");
#endif

}


/*
  Processing sketch to run with this example
 =====================================================
 // Processing UDP example to send and receive string data from Arduino
 // press any key to send the "Hello Arduino" message
 import hypermedia.net.*;
 UDP udp;  // define the UDP object
 void setup() {
 udp = new UDP( this, 6000 );  // create a new datagram connection on port 6000
 //udp.log( true );     // <-- printout the connection activity
 udp.listen( true );           // and wait for incoming message
 }
 void draw()
 {
 }
 void keyPressed() {
 String ip       = "192.168.1.177"; // the remote IP address
 int port        = 8888;    // the destination port
 udp.send("Hello World", ip, port );   // the message to send
 }
 void receive( byte[] data ) {      // <-- default handler
 //void receive( byte[] data, String ip, int port ) { // <-- extended handler
 for(int i=0; i < data.length; i++)
 print(char(data[i]));
 println();
 }
 */
