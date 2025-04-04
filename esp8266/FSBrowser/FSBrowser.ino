#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "SadrasIphone";
const char* password = "Helloooo";

// Remote IP and port (where data is sent)
IPAddress remoteIP(172, 20, 10, 3); // Replace with target IP
const uint16_t remotePort = 4210;   // Replace with target port

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Ready to send UDP packets...");
  Serial.println("Type a message and press Enter to send.");
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    message.trim(); // Remove extra whitespace/newline

    if (message.length() > 0) {
      Serial.print("Sending: ");
      Serial.println(message);

      // Send UDP packet
      Udp.beginPacket(remoteIP, remotePort);
      Udp.write(message.c_str());
      Udp.endPacket();

      Serial.println("Message sent!");
    }
  }
  delay(10); // Small delay to prevent watchdog issues
}