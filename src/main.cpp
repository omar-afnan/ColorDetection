#include <Arduino.h>
#define LED_LEFT 21
#define LED_RIGHT 22
#define LED_UP 23
#define LED_DOWN 25
#define LED_CENTER 26

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); // Serial to communicate with Raspberry Pi
  Serial.println("ESP32 Ready!");
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n'); // Read until newline
    command.trim();                                // Remove extra whitespace

    // Turn off all LEDs first (reset state)
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, LOW);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_CENTER, LOW);

    if (command == "LEFT")
    {
      Serial.println("⬅️ Move Left received.");
      // Add your motor or LED control logic here
    }
    else if (command == "RIGHT")
    {
      Serial.println("➡️ Move Right received.");
    }
    else if (command == "UP")
    {
      Serial.println("⬆️ Move Up received.");
    }
    else if (command == "DOWN")
    {
      Serial.println("⬇️ Move Down received.");
    }
    else if (command == "CENTERED" || command == "STAY")
    {
      Serial.println("✅ Stay Centered.");
    }
    else
    {
      Serial.println("❓ Unknown command: " + command);
    }
  }
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}
