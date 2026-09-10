
#include <Arduino.h>

#define RELAY2_PIN 1

void setup() {
  Serial.begin(115200);

  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);

  Serial.println("Relay 2 test starting...");
}

void loop() {

  Serial.println("RELAY 2 ON - GPIO14 HIGH");
  digitalWrite(RELAY2_PIN, HIGH);
  delay(3000);

  Serial.println("RELAY 2 OFF - GPIO14 LOW");
  digitalWrite(RELAY2_PIN, LOW);
  delay(3000);
}