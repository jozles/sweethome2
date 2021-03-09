
/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

#include <arduino.h>
#include <ESP8266WiFi.h>

#define PINA 4
#define PINB 5

#define PINL 0
#define PERLEDON 2000
#define PERLEDOFF 50
long tmpLed=millis();
int perled=0;

const char* ssid     = "devolo-5d3";
const char* password = "JNCJTRONJMGZEEQL";

WiFiServer server(1796);
WiFiClient client;

String header;

char icon[1024];
//char f0[128]={0x42,0x4d,0x76,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x00,0x00,0x00,0x28,0x00,
//              0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x01,0x00,0x04,0x00,0x00,0x00,
//              0x00,0x00,0x00,0x02,0x00,0x00,0xc4,0x0e,0x00,0x00,0xc4,0x0e,0x00,0x00,0x00,0x00,
//              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x80,
//              0x00,0x00,0x00,0x80,0x80,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x80,
//              0x00,0x00,0x80,0x80,0x80,0x00,0xc0,0xc0,0xc0,0x00,0x00,0x00,0xff,0x00,0x00,0xff,
//              0x00,0x00,0x00,0xff,0xff,0x00,0xff,0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0xff,
char f0[16]={0x00,0x00,0xff,0xff,0xff,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

char f1[128]={0x66,0x0F,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0x66,0x66,0x66,0x66,0x66,
              0x66,0x0F,0xff,0xff,0xff,0xff,0xff,0xff,0xf9,0x99,0xf0,0x66,0x66,0x66,0x66,0x66,
              0x66,0x0F,0x99,0x9f,0xff,0xff,0xff,0xff,0xf9,0x99,0x90,0x66,0x66,0x66,0x66,0x66,
              0x66,0x09,0x99,0x99,0xff,0xff,0xff,0xff,0x99,0x99,0x99,0x96,0x66,0x66,0x66,0x66,
              0x66,0x99,0x99,0x99,0xff,0xff,0xff,0xff,0x99,0x99,0x99,0x99,0x66,0x66,0x66,0x66,
              0x69,0x99,0x99,0x99,0xff,0xff,0xff,0xff,0xf9,0x99,0x99,0x99,0x96,0x66,0x66,0x66,
              0x99,0x99,0x99,0x99,0xff,0xff,0xff,0xff,0xff,0x99,0x99,0x99,0x99,0x66,0x66,0x99,
              0x99,0x99,0x99,0x9f,0xff,0xff,0xff,0xff,0xff,0xff,0x99,0x99,0x99,0x99,0x69,0x99};


String outputBState = "off";
String outputAState = "off";

const int outputB = PINB;
const int outputA = PINA;

unsigned long cxtime=0;
#define CXTO 60000

int getcde();
void buttonsHtml();
void accueilHtml();
void introHttp();
void cxEnd();
void favicon();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nmini server 8266 v3 ready");
  delay(100);

  pinMode(outputB, OUTPUT);
  pinMode(outputA, OUTPUT);
  digitalWrite(outputB, LOW);
  digitalWrite(outputA, LOW);

  // 1 LED blink
  pinMode(PINL,OUTPUT);
  digitalWrite(PINL, HIGH);delay(PERLEDON);
  digitalWrite(PINL, LOW);tmpLed=millis();

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("mac: ");
  for(uint8_t k=0;k<6;k++){
    if(mac[k]<16){Serial.print("0");}
    Serial.print(mac[k],HEX);
    if(k<5){Serial.print(".");}}
  Serial.println();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  memset(icon,0xff,1024);
  icon[1023]=0x00;
  memcpy(icon,f0,16);
  memcpy(icon+256,f1,128);
}

void loop(){
  
  if(millis()>tmpLed+perled){
    tmpLed=millis();
    bool led=digitalRead(PINL);
    digitalWrite(PINL,!led);
    if(!led){perled=PERLEDON;}
    else{perled=PERLEDOFF;}
  }

  if(millis()-cxtime>CXTO){cxtime=0;}

  
  int i;
  int cde=getcde();if(cde!=2){Serial.print("cde=");Serial.println(cde);
                              Serial.print("header=");
                              char c=header[0];int i=0;
                              while(c!='\0' && i<20){c=header[i];Serial.print(c);i++;}                              
                              Serial.println("...");
                              Serial.print("<<< ix=");Serial.print(header.indexOf("GET /?username__=admin&password__=17515A"));
                              Serial.print(" cxtime=");Serial.println(cxtime);}
//if(cde!=2){accueilHtml();}
    switch(cde){
    case 0: accueilHtml();break;
    case 1: if (header.indexOf("GET /?username__=admin&password__=17515A") >= 0) {
              buttonsHtml();
            } else if (header.indexOf("GET /favicon.ico") >=0) {
                favicon();                
            } else if(cxtime!=0){
              if (header.indexOf("GET /B/on") >= 0) {
                Serial.println("GPIO B on");
                outputBState = "on";
                digitalWrite(outputB, HIGH);
                buttonsHtml();
              } else if (header.indexOf("GET /B/off") >= 0) {
                Serial.println("GPIO B off");
                outputBState = "off";
                digitalWrite(outputB, LOW);
                buttonsHtml();
              } else if (header.indexOf("GET /A/on") >= 0) {
                Serial.println("GPIO A on");
                outputAState = "on";
                digitalWrite(outputA, HIGH);
                buttonsHtml();
              } else if (header.indexOf("GET /A/off") >= 0) {
                Serial.println("GPIO A off");
                outputAState = "off";
                digitalWrite(outputA, LOW);
                buttonsHtml();
              } else accueilHtml(); 
            } else accueilHtml();
            break;
    case 2:break;
    default:break;
  }
  
}

void cxEnd()
{
    delay(10);
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
}

void introHttp()
{
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
}


void accueilHtml()
{            
              Serial.println("accueil");
              cxtime=0;
              introHttp();
              
              client.println("<!DOCTYPE html><html>");  
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<body><form method=\"get\" >");

              client.println("<form>");                     
              
              client.println("<p>user <input type=\"username\" placeholder=\"Username\" name=\"username__\" value=\"\" size=\"6\" maxlength=\"8\" >");            
              client.println("<p>pass <input type=\"password\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");
              //client.println(" <input type=\"submit\" value=\"login\"><br>");
              client.println("<a href=\"\"><input type=\"submit\" value=\"login\"></a>");            
              client.println("</form></body></html>");                  

              cxEnd();
}

void buttonsHtml()
{
            Serial.println("buttons");
            cxtime=millis();
            
            introHttp();
              
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");

            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            client.println("<body><h1>ESP8266 mini-Web Server  v4.0</h1>");
            
            client.print("<p>GPIO B (");client.println(PINB);client.println(") - State " + outputBState + "</p>");
            client.print("<p> server ctl </p>");
            //client.print("<form>");
            // If the outputBState is off, it displays the ON button       
            if (outputBState=="off") {
              client.println("<p><a href=\"/B/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/B/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
            //client.println("</form>");
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            client.print("<p>GPIO A (");client.println(PINA);client.println( ") - State " + outputAState + "</p>"); 
            client.print("<form>");
            // If the outputAState is off, it displays the ON button       
            if (outputAState=="off") {
              client.println("<a href=\"/A/on\">"); //<button class=\"button\">ON</button></a></p>");
              client.println("<input type=\"button\" value=\"ON\"></a>");
            } else {
              client.println("<a href=\"/A/off\">"); //<button class=\"button button2\">OFF</button></a>");
              client.println("<input type=\"button\" value=\"OFF\"></a>");
            }
            client.print("</form>");
            client.println("</body></html>");

            cxEnd();
}

void favicon()
{
  Serial.println("send icon");
  client.println("HTTP/1.1 200 OK");
  client.println("CONTENT-Type: image/jpg");
  client.println();
  client.write(icon);
  cxEnd();
}

int getcde()
{
  
  client = server.available();              // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          
    String currentLine="";
    header="";
    while (client.connected()) {            
      if (client.available()) {             
        char c = client.read();             
        Serial.write(c);                  
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            while(client.available()){c = client.read();Serial.write(c);}
            return 1;}      // two newline, response to send
          else {currentLine = "";}                       
        }
        else if (c != '\r') {currentLine += c;}           // if you got anything else but a carriage return character add it      
      }
    }
    return 0;                                             // got something but not ended by 2 nexline                               
  }
  return 2;                                               // no client
}
