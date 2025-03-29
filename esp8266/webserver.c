#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "SadrasIphone";
const char* password = "Helloooo";
ESP8266WebServer server(80);

#define IMG_WIDTH 144
#define IMG_HEIGHT 174

uint8_t image[IMG_HEIGHT][IMG_WIDTH];
bool imageReady = false;
unsigned long lastSerialActivity = 0;
unsigned long bytesReceived = 0;

void handleRoot() {
  String html = "<html><head><title>ESP8266 Image Display</title>"
                "<style>"
                "body {font-family: Arial; text-align: center;}"
                "img {image-rendering: pixelated; width: 288px; height: 348px; border: 1px solid #000;}"
                "#status {color: #666; margin-top: 10px;}"
                "</style></head>"
                "<body>"
                "<h1>ESP8266 Image Display</h1>"
                "<img id='liveImage' src='/image'/>"
                "<div id='status'>Waiting for image data...</div>"
                "<script>"
                "function updateImage() {"
                "  fetch('/status').then(r=>r.text()).then(s=>{"
                "    document.getElementById('status').textContent = s;"
                "    document.getElementById('liveImage').src = '/image?' + Date.now();"
                "    setTimeout(updateImage, 1000);"
                "  });"
                "}"
                "updateImage();"
                "</script>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleImageRequest() {
  uint8_t bmpHeader[54] = {
    0x42, 0x4D, 0x36, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xAE, 0x00, 
    0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00
  };
  
  server.setContentLength(54 + IMG_WIDTH * IMG_HEIGHT);
  server.send(200, "image/bmp");
  server.sendContent((const char*)bmpHeader, 54);
  server.sendContent((const char*)image, IMG_WIDTH * IMG_HEIGHT);
}

void handleStatus() {
  String status;
  if (bytesReceived == 0) {
    status = "No data received yet";
  } else if (millis() - lastSerialActivity > 2000) {
    status = "Last data received " + String((millis() - lastSerialActivity)/1000) + "s ago";
  } else {
    status = "Receiving data (" + String(bytesReceived) + " bytes)";
  }
  server.send(200, "text/plain", status);
}

void decodeRLE() {
  static int x = 0, y = 0;
  
  while (Serial.available()) {
    uint8_t encoded = Serial.read();
    bytesReceived++;
    lastSerialActivity = millis();
    
    uint8_t pixelValue = (encoded >> 4) * 17;
    uint8_t runLength = encoded & 0x0F;
    
    for (int i = 0; i < runLength; i++) {
      if (x >= IMG_WIDTH) {
        x = 0;
        if (++y >= IMG_HEIGHT) {
          y = 0;
          imageReady = true;
          Serial.println("\nImage fully received!");
        }
      }
      image[y][x++] = pixelValue;
    }
    
    // Print progress every 512 bytes
    if (bytesReceived % 512 == 0) {
      Serial.printf("Received %d bytes (%.1f%%)\n", 
                   bytesReceived, 
                   100.0 * (y * IMG_WIDTH + x) / (IMG_WIDTH * IMG_HEIGHT));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting RLE Image Receiver");
  
  // Initialize with gradient pattern
  for(int y=0; y<IMG_HEIGHT; y++) {
    for(int x=0; x<IMG_WIDTH; x++) {
      image[y][x] = (x + y) % 256;
    }
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  server.on("/", handleRoot);
  server.on("/image", handleImageRequest);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Waiting for RLE image data...");
}

void loop() {
  if (Serial.available()) {
    decodeRLE();
  }
  server.handleClient();
  
  // Print warning if no data received for 5 seconds
  if (bytesReceived > 0 && millis() - lastSerialActivity > 5000) {
    Serial.println("Warning: No serial data for 5 seconds");
    lastSerialActivity = millis(); // Reset timer
  }
  
  delay(1);
}