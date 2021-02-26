
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
String currentLine;

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

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("mini server 8266 v3 ready");
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

  int cde=getcde();if(cde!=2){Serial.print("cde=");Serial.println(cde);Serial.print("header=");Serial.print(header);
                              Serial.print("<<< ix=");Serial.print(header.indexOf("GET /?username__=admin&password__=17515A"));
                              Serial.print(" cxtime=");Serial.println(cxtime);}
  switch(cde){
    case 0: accueilHtml();break;
    case 1: if (header.indexOf("GET /?username__=admin&password__=17515A") >= 0) {
              buttonsHtml();}
            else if(cxtime!=0){
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
    default:break;
  }
}

void cxEnd()
{
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
}

void introHttp()
{
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
}


void accueilHtml()
{            
              cxtime=0;
              introHttp();
              
              client.println("<!DOCTYPE html><html>");  
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<body><form method=\"get\" >");

              client.println("<p>user <input type=\"username\" placeholder=\"Username\" name=\"username__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");            
              client.println("<p>pass <input type=\"password\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");
              client.println(" <input type=\"submit\" value=\"login\"><br>");
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
            
            client.println("<body><h1>ESP8266 mini-Web Server  v3.0</hHeadi1>");
            
            client.print("<p>GPIO B (");client.println(PINB);client.println(") - State " + outputBState + "</p>");
            client.print("<p> server ctl </p>");
            // If the outputBState is off, it displays the ON button       
            if (outputBState=="off") {
              client.println("<p><a href=\"/B/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/B/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            client.print("<p>GPIO A (");client.println(PINA);client.println( ") - State " + outputAState + "</p>"); 
            // If the outputAState is off, it displays the ON button       
            if (outputAState=="off") {
              client.println("<p><a href=\"/A/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/A/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");

            cxEnd();

}

int getcde()
{
  int state=0;
  
  client = server.available();              // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    currentLine = "";                       // make a String to hold incoming data from the client
    header="";
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             
        //Serial.write(c);                  
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {return 1;}      
          else {currentLine = "";}                        // if you got a newline, then clear currentLine
        }
        else if (c != '\r') {currentLine += c;}           // if you got anything else but a carriage return character add it      
      }
    }
    return 0;                                             // got something                               
  }
  return 2;                                               // pas de client
}
