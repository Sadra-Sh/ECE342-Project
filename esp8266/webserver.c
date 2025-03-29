#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define IMG_WIDTH 174
#define IMG_HEIGHT 144

ESP8266WebServer server(80);
uint8_t imageData[IMG_WIDTH * IMG_HEIGHT];
bool showInverted = false;
unsigned long lastToggleTime = 0;

void generateCheckerboard(bool inverted) {
    const int squareSize = 8; // Size of each checkerboard square
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            // Determine if this pixel should be black or white
            bool isBlack = ((x / squareSize + y / squareSize) % 2 == 0);
            if (inverted) isBlack = !isBlack;
            
            // Grayscale value (0 = black, 255 = white)
            imageData[y * IMG_WIDTH + x] = isBlack ? 0x00 : 0xFF;
        }
    }
}

void handleRoot() {
    String html = "<html><head><title>ESP8266 Checkerboard</title>"
                  "<style>img { image-rendering: pixelated; }</style></head>"
                  "<body><h1>ESP8266 Checkerboard</h1>"
                  "<img id='liveImage' src='/image' "
                  "style='border: 1px solid #000; width: 174px; height: 144px;'/>"
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
        // File header (14 bytes)
        0x42, 0x4D,             // 'BM'
        0x36, 0x84, 0x03, 0x00, // File size (54 + 174*144 = 0x38436)
        0x00, 0x00,             // Reserved
        0x00, 0x00,             // Reserved
        0x36, 0x00, 0x00, 0x00, // Pixel data offset (54 bytes)
        
        // DIB header (40 bytes)
        0x28, 0x00, 0x00, 0x00, // Header size (40 bytes)
        0xAE, 0x00, 0x00, 0x00, // Image width (174)
        0x90, 0x00, 0x00, 0x00, // Image height (144)
        0x01, 0x00,             // Planes (1)
        0x08, 0x00,             // Bits per pixel (8)
        0x00, 0x00, 0x00, 0x00, // Compression (none)
        0x00, 0x84, 0x03, 0x00, // Image size (174*144 = 0x38400)
        0x00, 0x00, 0x00, 0x00, // X pixels per meter
        0x00, 0x00, 0x00, 0x00, // Y pixels per meter
        0x00, 0x00, 0x00, 0x00, // Colors in palette
        0x00, 0x00, 0x00, 0x00  // Important colors
    };
    
    // Send the image
    server.setContentLength(54 + IMG_WIDTH * IMG_HEIGHT);
    server.send(200, "image/bmp");
    server.sendContent((const char*)bmpHeader, 54);
    server.sendContent((const char*)imageData, IMG_WIDTH * IMG_HEIGHT);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Checkerboard Demo");
    
    // Generate initial checkerboard
    generateCheckerboard(false);
    
    // Connect to WiFi
    WiFi.begin(ssid, passPhrase);
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
    // Toggle pattern every second
    if (millis() - lastToggleTime >= 1000) {
        showInverted = !showInverted;
        generateCheckerboard(showInverted);
        lastToggleTime = millis();
        Serial.println("Pattern toggled: " + String(showInverted ? "Inverted" : "Normal"));
    }
    
    server.handleClient();
}