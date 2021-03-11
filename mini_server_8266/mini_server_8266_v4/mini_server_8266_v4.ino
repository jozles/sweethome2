
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

char icon[1024]={0x00,0x00,0x00,0x15,0x00,0x00,0x00,0x15,0x08,0x06,0x00,0x00,0x00,0xA9,0x17,0xA5,
                 0x96,0x00,0x00,0x00,0x04,0x67,0x41,0x4D,0x41,0x00,0x00,0xB1,0x8F,0x0B,0xFC,0x61,     .....gAMA......a
                 0x05,0x00,0x00,0x00,0x09,0x70,0x48,0x59,0x73,0x00,0x00,0x0E,0xC4,0x00,0x00,0x0E,
/*   200876F0   C4 01 95 2B 0E 1B 00 00 01 36 49 44 41 54 38 4F     ...+.....6IDAT8O
   20087700   B5 94 4B 6E C2 40 0C 86 3D E1 21 1E BB 80 60 43     ..Kn.@..=.!...`C
   20087710   37 88 4A EC AB AE 7A 05 A4 1E A4 E7 E9 41 D8 71     7.J...z......A.q
   20087720   07 2E 80 04 AC A8 10 6A 53 C4 AA AA 4A 48 3A 76     .......jS...JH:v
   20087730   66 02 21 36 19 09 F8 24 CB C6 33 FE C7 9E 51 50     f.!6...$..3...QP
   20087740   B1 06 6E 8C 67 FC 4D 71 16 DD 3E 3C 9A A8 18 27     ..n.g.Mq..><...'
   20087750   51 2B E8 2A 5C 28 7A 2E E4 24 8C 0F 25 F1 DD 1B     Q+.*\(z..$..%...
   20087760   90 E9 97 3C 5A A9 94 E4 2E 20 8A B2 82 27 76 49     ...<Z.... ...'vI
   20087770   98 1D DF 8E E8 7F 2C C8 4B 88 57 61 C4 53 A8 83     .....,.K.Wa.S..
   20087780   5A 2D D7 55 7F 3C 62 3B E7 3A D6 2B 47 A4 22 14     Z-.U<b;.:.+G.".
   20087790   EC 4E 5E 9D 85 D3 2F 0A 47 39 1F 77 DB 1B C0 F3     .N^.../.G9.w....
   200877A0   FB 10 22 1D E3 3D 59 3F 7D 9B 01 84 6B F0 37 3F     .."..=Y?}...k.7?
   200877B0   FA 57 02 EE F5 57 73 8A D9 3B 8D AA 0D E8 BC 74     .W...Ws..;.....t
   200877C0   33 82 88 F5 98 6F 3D B5 01 CA 0D 93 C9 92 8A E2     3....o=.........
   200877D0   29 78 1A 9A FA 0C E0 EB 77 47 79 F6 54 64 BD 81     )x......wGy.Td..
   200877E0   38 08 D2 1A DB 25 92 A9 C1 05 34 D5 AC 03 FC ED     8....%....4.....
   200877F0   4D 56 20 0C 69 9F AD 39 85 6F 24 3C 98 A0 00 61     MV .i..9.o$<...a
   20087800   9F 38 DD 35 C8 A2 9E 22 A7 2A 5E CE 08 B3 CE C1     .8.5...".*^.....
   20087810   8A E2 8B C3 32 19 2D DE 47 39 23 F4 BA 89 72 88     ....2.-.G9#...r.
   20087820   FF FC 4A C9 9D 58 84 52 59 F4 1A EE F0 50 00 FF     ..J..X.RY....P..
   20087830   36 15 61 CD C3 87 49 30 00 00 00 00 49 45 4E 44     6.a...I0....IEND
   20087840   AE 42 60 82 00 00 00 00 00 00 00 00 00 00 00 00 
*/
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
