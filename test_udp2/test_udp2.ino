
#include <SPI.h>        
#include <Ethernet.h>
#include <EthernetUdp.h>

IPAddress masterIp(192, 168, 0, 36);
unsigned int masterPort = 8886;

#define SLAVE

#ifndef SLAVE
//byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0xDF, 0xAE};
#define IP masterIp
#define PORT masterPort
#endif

#ifdef SLAVE
//byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress slaveIp(192, 168, 0, 31);
#define IP slaveIp
#define PORT 8888
#endif

#define MAX_LENGTH 1000

EthernetUDP Udp;

int i,cnt=0;

void setup() {
  
  Serial.begin(115200);

// show mac,ip,port of current machine on serial
  for(i=0;i<6;i++){Serial.print(mac[i],HEX);if(i!=5){Serial.print(":");}}
  Serial.print(" ready on ");
  for(i=0;i<4;i++){Serial.print(IP[i]);if(i!=3){Serial.print(".");}}
  Serial.print("  ");
  Serial.println(PORT);

// start ethernet,udp  
  Ethernet.begin(mac, IP);
  if(!Udp.begin(PORT)){Serial.println("Udp.begin ko");while(1){};}

}

// receive an udp message
int rxUdp(IPAddress* ipAddr,unsigned int* rxPort,char* data)
{
  int packetSize = Udp.parsePacket();
 
  if (packetSize){
    *ipAddr = (uint32_t) Udp.remoteIP();
    Serial.print("Received packet of size ");Serial.println(packetSize);
    Serial.print("From ");Serial.print(*ipAddr);Serial.print(" ");
    
    *rxPort = (unsigned int) Udp.remotePort();
    Serial.print(", port ");Serial.println((int)*rxPort);

    Udp.read(data, packetSize);
    Serial.println("Contents:");Serial.println(data);
  }
  
  return packetSize;
}

// transmit an udp message
void txUdp(IPAddress ipAddr,unsigned int port,char* data)
{
  Udp.beginPacket(ipAddr,port);
  
  Serial.print("sending (");Serial.print(strlen(data));Serial.print(")>");Serial.print(data);
  Serial.print("< to ");Serial.print(ipAddr);Serial.print(":");Serial.println(port);
  
    Udp.write(data,strlen(data));
    Udp.endPacket();
}

void loop() 
{
  
  IPAddress rxIp;
  unsigned int rxPort;
  char data[MAX_LENGTH];
  memset(data,0x00,MAX_LENGTH);

#ifdef SLAVE
Serial.print(masterIp);Serial.print(" ");Serial.println(masterPort);
//  txUdp(masterIp,masterPort,"hello master !");
  txUdp(masterIp,masterPort,"GET /cx?peri_pass_=0011_17515A29?");
  Serial.println("tx");
  while(rxUdp(&rxIp,&rxPort,data)==0){}
  cnt++;
  Serial.println(cnt);delay(1000);
#endif

#ifndef SLAVE
  while(rxUdp(&rxIp,&rxPort,data)==0){}
  txUdp(rxIp,8888,"hi slave!");
#endif

}


 void conv_htoa(char* ascii,byte* h)
{
  char* chexa="0123456789ABCDEFabcdef\0";
  
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
