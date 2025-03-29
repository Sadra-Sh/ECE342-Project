int cmdx = 0;      // Initialize the count variable
int cmdy = 10000;
unsigned long lastMillis = 0;  // To store the time when the delay was last updated
int delayTime = 10; // Initial delay time in ms
char cmd[20];

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);  // Reduce default timeout for readString
}

void loop() {
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    str.trim();
    if (str.length() > 0) {
      // Safely format into buffer (snprintf prevents overflow)
      int len = snprintf(cmd, sizeof(cmd), "%d,%d\n", cmdx++, cmdy--);
      if (len > 0 && len < sizeof(cmd)) {
        Serial.print(cmd);  // Use print(), not println() (since cmd already has \n)
      }
    }
  }
}
