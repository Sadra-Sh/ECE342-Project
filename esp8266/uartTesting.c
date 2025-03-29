int count = 0;      // Initialize the count variable
unsigned long lastMillis = 0;  // To store the time when the delay was last updated
int delayTime = 10; // Initial delay time in ms

void setup() {
  Serial.begin(115200);  // Start Serial communication
}

void loop() {
  // Increment the count
  count++;
  Serial.println(count);

  // Check if 1 second has passed
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) { // 1000 ms = 1 second
    lastMillis = currentMillis; // Update the lastMillis time
    // Increase the delay time (up to 20ms)
    if (delayTime < 20) {
      delayTime++;
    }
  }

  // Apply the delay
  delay(delayTime);
}
