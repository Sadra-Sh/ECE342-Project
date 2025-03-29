#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "SadrasIphone";
const char* password = "Helloooo";
ESP8266WebServer server(80);

// Starting image dimensions
int img_w = 10;
int img_h = 10;
bool serverRunning = false;
unsigned long lastSizeIncrease = 0;
bool sendFailed = false;

const char page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {text-align:center; font-family:Arial;}
    img {image-rendering:pixelated; width:256px; height:256px;}
    .info {margin:20px; font-size:1.2em;}
  </style>
</head>
<body>
  <h1>ESP8266 Image Size Test</h1>
  <div class="info">Current size: <span id="size">10x10</span></div>
  <img id="img" src="/image">
  <script>
    setInterval(function() {
      fetch('/size').then(r=>r.text()).then(s=>{
        document.getElementById('size').textContent = s;
        document.getElementById('img').src = '/image?' + Date.now();
      });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", page);
}

void handleSize() {
  server.send(200, "text/plain", String(img_w) + "x" + String(img_h));
}

void handleImage() {
  if (sendFailed) {
    server.send(500, "text/plain", "Image too large");
    return;
  }

  // Calculate BMP header
  uint32_t fileSize = 54 + img_w * img_h;
  uint8_t bmpHeader[54] = {
    0x42, 0x4D,                         // 'BM'
    fileSize & 0xFF, (fileSize >> 8) & 0xFF, (fileSize >> 16) & 0xFF, (fileSize >> 24) & 0xFF,
    0, 0, 0, 0,                         // Reserved
    54, 0, 0, 0,                        // Pixel data offset
    40, 0, 0, 0,                        // DIB header size
    img_w & 0xFF, (img_w >> 8) & 0xFF, 0, 0,  // Width
    img_h & 0xFF, (img_h >> 8) & 0xFF, 0, 0,  // Height
    1, 0,                               // Planes
    8, 0,                               // Bits per pixel
    0, 0, 0, 0,                         // Compression
    (img_w * img_h) & 0xFF, ((img_w * img_h) >> 8) & 0xFF, ((img_w * img_h) >> 16) & 0xFF, ((img_w * img_h) >> 24) & 0xFF,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  // Try to send image
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "image/bmp");
  server.sendContent_P((const char*)bmpHeader, 54);
  
  uint8_t row[img_w];
  for (int y = 0; y < img_h; y++) {
    for (int x = 0; x < img_w; x++) {
      row[x] = random(256);
    }
    server.sendContent((const char*)row, img_w);
    
    // Check if client is still connected
    if (!server.client().connected()) {
      sendFailed = true;
      Serial.println("Client disconnected at size " + String(img_w) + "x" + String(img_h));
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  randomSeed(analogRead(A0));
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    server.on("/", handleRoot);
    server.on("/size", handleSize);
    server.on("/image", handleImage);
    server.begin();
    serverRunning = true;
    Serial.println("HTTP server started");
    Serial.println("Testing image sizes...");
    Serial.println("Format: [width]x[height] (pixels)");
  } else {
    Serial.println("\nFailed to connect to WiFi");
    ESP.restart();
  }
}

void loop() {
  if (serverRunning && millis() - lastSizeIncrease >= 500 && !sendFailed) {
    // Increase size
    img_w += 5;
    img_h += 5;
    
    Serial.print("Testing: ");
    Serial.print(img_w);
    Serial.print("x");
    Serial.println(img_h);
    
    lastSizeIncrease = millis();
  }
  
  server.handleClient();
  delay(2);
}