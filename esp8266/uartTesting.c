int count = 0;      // Initialize the count variable
unsigned long lastMillis = 0;  // To store the time when the delay was last updated
int delayTime = 10; // Initial delay time in ms

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);  // Reduce default timeout for readString
}

void loop() {
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');  // Read until newline
    str.trim();  // Remove any whitespace/newlines
    if (str.length() > 0) {  // Only process if we got data
      count++;
      Serial.println(count);  // Send response with newline
    }
  }
}
