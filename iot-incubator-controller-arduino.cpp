/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 Circuit:
 * WiFi shield attached

 created 30 December 2012
 by dlf (Metodo2 srl)

 */

#include <DHT.h>;
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

//Constants for DHT
#define DHTPIN 10     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// Constants for heatbed
#define HEATBED_PIN 9

// Constants for NTC thermistors
float R1 = 10000;
float logR2, R2, T;
int Vo; 
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int NTC_Pin = A3;

int status = WL_IDLE_STATUS;
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char sendBuffer[255]; //buffer to hold packet to be sent
char packetBuffer[255]; //buffer to hold incoming packet
char ReplyBuffer[] = "acknowledged";       // a string to send back

// Define server IP and UDP port
IPAddress serverIP(10, 50, 0, 127);
unsigned int serverPort = 6868;

WiFiUDP Udp;

// Default target temprature (in String format)
char currTargetTemp[] = "28.5";

void setup() {
  WiFi.setPins(8,7,4,2);
  dht.begin();
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {
  delay(2000); // Loop every 2s

  // Read temperature from dht
  float hum = dht.readHumidity();
  float temp= dht.readTemperature();
  
  // Read NTC temp from analog
  float realAnalogTemp = calcNTCTemp();
  
  //Print temp and humidity values to serial monitor
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" %, Temp: ");
  Serial.print(temp);
  Serial.print(" Celsius");
  Serial.print(", Analog Temp: ");
  Serial.println(realAnalogTemp);
  
  // Send udp packet to server
  sprintf(sendBuffer, "Hum:%f,Temp:%f,Analog:%f,TargetTemp:%s\n", hum, temp, realAnalogTemp, currTargetTemp);
  Udp.beginPacket(serverIP, serverPort);
  Udp.write(sendBuffer);
  if(Udp.endPacket() == 0) {
    Serial.println("UDP packet send failed");
  }

  // Compare with current target temperature and turn on/off heatbed
  if(temp > atof(currTargetTemp)) {
    // Stop heatbed
    digitalWrite(HEATBED_PIN, LOW);  
  }
  else {
    // Start heatbed
    digitalWrite(HEATBED_PIN, HIGH);  
  }

  
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
}


//Function to read NTC temp and perform the fancy math of the Steinhart-Hart equation
float calcNTCTemp() {
    Vo = analogRead(NTC_Pin);
    Vo = float(Vo);
    R2 = (1023.0 - Vo) * R1 / Vo;
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    return T - 273.15; // Celsius;
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}