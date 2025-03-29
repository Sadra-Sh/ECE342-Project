#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define IMG_WIDTH 174
#define IMG_HEIGHT 144
#define SQUARE_SIZE 24  // Larger squares for better visibility

#ifndef STASSID
#define STASSID "SadrasIphone"
#define STAPSK "Helloooo"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

ESP8266WebServer server(80);

uint8_t imageData[IMG_WIDTH * IMG_HEIGHT];
bool showInverted = false;
unsigned long lastToggleTime = 0;

void generateCheckerboard() {
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            // Determine if this pixel should be black or white
            bool isBlack = (x / SQUARE_SIZE + y / SQUARE_SIZE) % 2 == 0;
            imageData[y * IMG_WIDTH + x] = isBlack ? 0x00 : 0xFF;
        }
    }
}

void invertCheckerboard() {
    // Invert the entire image in memory by flipping each byte
    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
        imageData[i] = ~imageData[i];  // Bitwise NOT flips all bits
    }
}

void handleRoot() {
    String html = "<html><head><title>ESP8266 Checkerboard</title>"
                  "<style>"
                  "body { font-family: Arial, sans-serif; text-align: center; }"
                  "img { image-rendering: pixelated; border: 2px solid #333; }"
                  "h1 { color: #444; }"
                  "</style></head>"
                  "<body><h1>ESP8266 Checkerboard</h1>"
                  "<img id='liveImage' src='/image' "
                  "style='width: 348px; height: 288px;'/>"
                  "<p>Pattern inverts every second automatically</p>"
                  "<script>"
                  "setInterval(function() {"
                  "  document.getElementById('liveImage').src = '/image?' + Date.now();"
                  "}, 1000);"
                  "</script>"
                  "</body></html>";
    server.send(200, "text/html", html);
}

void handleImageRequest() {
    // BMP header for 8-bit grayscale image
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
    server.sendContent((const char*)imageData, IMG_WIDTH * IMG_HEIGHT);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Checkerboard Demo");
    
    // Generate initial checkerboard (only once)
    generateCheckerboard();
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

    // Set up web server
    server.on("/", HTTP_GET, handleRoot);
    server.on("/image", HTTP_GET, handleImageRequest);
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    // Toggle pattern every second by inverting in memory
    if (millis() - lastToggleTime >= 1000) {
        invertCheckerboard();
        lastToggleTime = millis();
        Serial.println("Pattern inverted in memory");
    }
    
    server.handleClient();
}