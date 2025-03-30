#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "SadrasIphone"
#define STAPSK "Helloooo"
#endif

#define PORT 4210
IPAddress remoteIP(172, 20, 10, 2);
WiFiUDP Udp;
char packet[255];
bool readyToReceive = false;

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(STASSID, STAPSK);
  // Serial.print("Connecting to ");
  Serial.print(STASSID);

  while (WiFi.status() != WL_CONNECTED) {
    // Serial.print('.');
    delay(500);
  }

  // Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(PORT);
  // Serial.print("Listening on UDP port: ");
  Serial.println(PORT);
}

void loop() {
  while(Udp.parsePacket() > 0){
    int len = Udp.read(packet, sizeof(packet) - 1);
    if (len > 0) {
      packet[len] = '\0'; // Null-terminate string
    }
  }
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // remove any trailing newline/whitespace
    if (input == "ready") {
      Serial.println(packet);
    }
  }
}