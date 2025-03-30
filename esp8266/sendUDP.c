#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "SadrasIphone"
#define STAPSK "Helloooo"
#endif

#define PORT 4210

// Target IP Address
IPAddress carIP(172, 20, 10, 4);

WiFiUDP Udp;
char packet[255];
char reply[] = "Waiting for Data";

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
  
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  Udp.begin(PORT);
  Serial.print("Listening on UDP port: ");
  Serial.println(PORT);
}

void loop() {
  String str = "";
  if (Serial.available() > 0) {
    str = Serial.readStringUntil('\n');  // Read input from Serial

    // Send UDP packet
    Udp.beginPacket(carIP, PORT);
    Udp.write(str.c_str(), str.length());  // Convert String to char array
    Udp.endPacket();

    Serial.println("UDP Packet Sent!");
    delay(20);  // Optional: Small delay to prevent flooding
  }
}
