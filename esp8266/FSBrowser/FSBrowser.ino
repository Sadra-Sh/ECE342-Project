#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "SadrasIphone"
#define STAPSK "Helloooo"
#endif

#define PORT 4210

WiFiUDP Udp;  // Correct usage
char packet[255];

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  WiFi.begin(STASSID, STAPSK);
  Serial.print("Connecting to ");
  Serial.print(STASSID);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  
  Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  Udp.begin(PORT);
  Serial.print("Listening on UDP port: ");
  Serial.println(PORT);
}

void loop() {

  int packetSize = Udp.parsePacket();  // Corrected Udp object reference
  if (packetSize > 0) {
    int len = Udp.read(packet, sizeof(packet) - 1); // Prevent overflow
    if(Serial.available() > 0){
        Serial.println(packet);
    }
  }
}
