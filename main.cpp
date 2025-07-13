#include <Arduino.h>

void setup() {
  Serial.begin(9600);         // Start serial communication
  while (!Serial);            // Wait for serial monitor (optional)
  Serial.println("✅ ESP Ready");  // Confirm boot
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // Read until newline
    cmd.trim();  // Remove trailing \r or spaces
    Serial.print("📥 Received: ");
    Serial.println(cmd);  // Print what was received
  }
}
