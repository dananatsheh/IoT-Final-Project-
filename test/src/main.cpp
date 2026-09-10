#include <Arduino.h>
#include <WiFi.h>

const char* WIFI_SSID = "CYBER_EXT";
const char* WIFI_PASSWORD = "cyberap2025";

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("Connecting to WiFi...");

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");

    Serial.print("ESP32 IP Address: ");
    Serial.println(
        WiFi.localIP()
    );

    Serial.print("Gateway: ");
    Serial.println(
        WiFi.gatewayIP()
    );

    Serial.print("Subnet Mask: ");
    Serial.println(
        WiFi.subnetMask()
    );

    Serial.print("RSSI: ");
    Serial.print(
        WiFi.RSSI()
    );
    Serial.println(" dBm");
}

void loop()
{
}