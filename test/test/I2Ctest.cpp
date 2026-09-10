#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 23

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  delay(1000);

  Serial.println();
  Serial.println("I2C Scanner Starting...");
}

void loop() {
  byte error;
  byte address;
  int deviceCount = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);

      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(deviceCount);
  }

  Serial.println("--------------------------");

  delay(2000);
}