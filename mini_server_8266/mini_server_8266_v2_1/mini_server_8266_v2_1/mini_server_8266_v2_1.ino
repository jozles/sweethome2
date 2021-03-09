
/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
#include <ESP8266WiFi.h>

#define PINA 4
#define PINB 5

#define PINL 0
#define PERLEDON 2000
#define PERLEDOFF 50
long tmpLed=millis();
int perled=0;

unsigned long cxTime=0;
#define CXDLY 60000

unsigned long activeTime=0;
#define ACTDLY 300000

// Replace with your network credentials
//const char* ssid     = "pinks";
//const char* password = "cain ne dormant pas songeait au pied des monts";
const char* ssid     = "devolo-5d3";
const char* password = "JNCJTRONJMGZEEQL";

// Set web server port
#define PORT 1796
WiFiServer server(PORT);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String outputBState = "off";
String outputAState = "off";

// Assign output variables to GPIO pins
const int outputB = PINB;
const int outputA = PINA;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("mini server 8266 v2 ready");
  delay(100);

  // Initialize the output variables as outputs
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
  Serial.println();Serial.println();
  delay(2);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
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

  if ( cxTime+CXDLY<millis() && cxTime!=0 ){cxTime=0;Serial.println("cxTime=0");}

//  if ( activeTime+ACTDLY<millis()){x=cliext.connect(hostExt,portExt);cliext.stop();}

  delay(500);
  int count=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    count++;
  }
  if(count>2){Serial.println(count);}
  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            // Web Page ng
            client.println("<body><h1>ESP8266 Web Server v2.1</h1>");

            if (header.indexOf("GET / HTTP")>=0){cxTime=0;}         // pas de commande
            
            if (cxTime!=0 || header.indexOf("?username__=admin&password__=17515A") >= 0){

            /* GPIO command waited */

              Serial.println("buttons");
              cxTime=millis();
              
              // turns the GPIOs on and off
              if (header.indexOf("GET /B/on") >= 0) {
                Serial.println("GPIO B on");
                outputBState = "on";
                digitalWrite(outputB, HIGH);
              } else if (header.indexOf("GET /B/off") >= 0) {
                Serial.println("GPIO B off");
                outputBState = "off";
                digitalWrite(outputB, LOW);
              } else if (header.indexOf("GET /A/on") >= 0) {
                Serial.println("GPIO A on");
                outputAState = "on";
                digitalWrite(outputA, HIGH);
              } else if (header.indexOf("GET /A/off") >= 0) {
                Serial.println("GPIO A off");
                outputAState = "off";
                digitalWrite(outputA, LOW);
              }
              
              // Display current state, and ON/OFF buttons for GPIO 5  
              client.print("<p>GPIO B (");client.println(PINB);client.println(") - State " + outputBState + "</p>");
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
            }             // cxtime !=0 and CXDLY not reached => end of buttons

            else {

            /* accueil */              
                          
              Serial.println("accueil");
              cxTime=0;
                            
              client.println("<form>");                                   
              client.println("<p>user <input type=\"username\" placeholder=\"Username\" name=\"username__\" value=\"\" size=\"6\" maxlength=\"8\" >");        
              client.println("<p>pass <input type=\"password\" placeholder=\"Password\" name=\"password__\" value=\"\" size=\"6\" maxlength=\"8\" ></p>");
              client.println("<input type=\"submit\" value=\"login\">");
              client.println("</form>");         
            }
            
            // html ends
            client.println("</body></html>");            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
        
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }     // client.available()
    }       // client.connected()
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
