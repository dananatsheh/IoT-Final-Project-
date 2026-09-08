#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

// Globals
TwoWire I2C_A = TwoWire(0);   
TwoWire I2C_B = TwoWire(1);   

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

Adafruit_BME280 bmeX;
Adafruit_BME280 bmeY;         
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &I2C_B, -1);

bool bmeY_present = false;

enum JobState { STATE_IDLE, STATE_RUNNING, STATE_PAUSED, STATE_FAULTED, STATE_DONE, STATE_UNKNOWN };
JobState currentState = STATE_UNKNOWN;
JobState previousState = STATE_UNKNOWN;

bool estopLatched = false;
String estopReason = "";

// Vibration rolling window
float vibSamples[VIBRATION_WINDOW_SIZE];
int vibIndex = 0;
bool vibBufferFull = false;
unsigned long lastVibSampleMs = 0;

unsigned long lastPublishMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastMqttAttemptMs = 0;

// WiFi / MQTT
void connectWiFi() {
  Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi connect FAILED — will retry in loop()");
  }
}

void mqttReconnect() {
  if (millis() - lastMqttAttemptMs < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttAttemptMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("Attempting MQTT connection...");
  bool ok;
  if (strlen(MQTT_USERNAME) > 0) {
    ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
  } else {
    ok = mqtt.connect(MQTT_CLIENT_ID);
  }
  if (ok) {
    Serial.println(" connected.");
  } else {
    Serial.printf(" failed, rc=%d — will retry\n", mqtt.state());
  }
}

void mqttPublish(const char* topic, const String& payload) {
  if (mqtt.connected()) {
    mqtt.publish(topic, payload.c_str());
  }
}

// Sensors
bool initSensors() {
  bool ok = true;

  if (!bmeX.begin(BME280_ADDR_X, &I2C_A)) {
    Serial.println("ERROR: BME280 (X) not found on Bus A — check wiring/address");
    ok = false;
  }

  if (ENABLE_SECOND_BME280) {
    bmeY_present = bmeY.begin(BME280_ADDR_Y, &I2C_A);
    if (!bmeY_present) {
      Serial.println("ERROR: BME280 (Y) not found on Bus A — check wiring/address");
      ok = false;
    }
  }

  if (!mpu1.begin(MPU_ADDR_1, &I2C_A)) {
    Serial.println("ERROR: MPU6050 #1 not found on Bus A");
    ok = false;
  }
  if (!mpu2.begin(MPU_ADDR_2, &I2C_A)) {
    Serial.println("ERROR: MPU6050 #2 not found on Bus A — check AD0 tied to 3.3V");
    ok = false;
  }

  if (mpu1.begin(MPU_ADDR_1, &I2C_A)) {
    mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  return ok;
}

bool initOled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERROR: OLED not found on Bus B");
    return false;
  }
  display.clearDisplay();
  display.display();
  return true;
}


void sampleVibration() {
  if (millis() - lastVibSampleMs < VIBRATION_SAMPLE_INTERVAL_MS) return;
  lastVibSampleMs = millis();

  sensors_event_t a, g, temp;
  mpu1.getEvent(&a, &g, &temp);

  float mag = sqrt(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z) / 9.80665f;

  vibSamples[vibIndex] = mag;
  vibIndex = (vibIndex + 1) % VIBRATION_WINDOW_SIZE;
  if (vibIndex == 0) vibBufferFull = true;
}

float computeVibrationRMS() {
  int count = vibBufferFull ? VIBRATION_WINDOW_SIZE : vibIndex;
  if (count == 0) return 0.0f;

  float mean = 0;
  for (int i = 0; i < count; i++) mean += vibSamples[i];
  mean /= count;

  float sumSq = 0;
  for (int i = 0; i < count; i++) {
    float dev = vibSamples[i] - mean;
    sumSq += dev * dev;
  }
  return sqrt(sumSq / count);
}

bool readDriverTelemetry(float& outTempC, bool& outFault) {
  return false; 
}

// CN1 job-status derivation
JobState deriveJobState() {
  if (DI_IN1_PIN < 0 || DI_IN2_PIN < 0 || DI_IN3_PIN < 0) {
    return STATE_UNKNOWN;
  }

  bool out2Fault = (digitalRead(DI_IN3_PIN) == DI_ACTIVE_LEVEL);
  bool statusRunning = (digitalRead(DI_IN2_PIN) == DI_ACTIVE_LEVEL);
  bool out1Idle = (digitalRead(DI_IN1_PIN) == DI_ACTIVE_LEVEL);

  if (out2Fault) return STATE_FAULTED;
  if (statusRunning) return STATE_RUNNING; 
  if (out1Idle) {
    if (previousState == STATE_RUNNING) return STATE_DONE;
    return STATE_IDLE;
  }
  return STATE_UNKNOWN;
}

const char* stateToString(JobState s) {
  switch (s) {
    case STATE_IDLE: return "idle";
    case STATE_RUNNING: return "running";
    case STATE_PAUSED: return "paused";
    case STATE_FAULTED: return "faulted";
    case STATE_DONE: return "done";
    default: return "unknown";
  }
}

// E-stop logic
void triggerEstop(const String& reason) {
  if (estopLatched) return; 

  estopLatched = true;
  estopReason = reason;

  if (DO_RELAY_ESTOP_PIN >= 0) {
    digitalWrite(DO_RELAY_ESTOP_PIN, LOW); 
  } else {
    Serial.println("WARNING: DO_RELAY_ESTOP_PIN not configured — relay NOT driven. "
                    "Confirm ES32C14 relay channel mapping before relying on this.");
  }

  Serial.printf("*** E-STOP TRIGGERED (%s) ***\n", reason.c_str());
  mqttPublish(TOPIC_ESTOP_EVENT, reason);
}


void resetEstopLatch() {
  estopLatched = false;
  estopReason = "";
  if (DO_RELAY_ESTOP_PIN >= 0) {
    digitalWrite(DO_RELAY_ESTOP_PIN, HIGH); 
  }
  Serial.println("E-stop latch reset — normal operation resumed by deliberate action.");
}

void checkEstopConditions(float vibRms) {
  if (estopLatched) return;

  if (DI_IN4_PIN >= 0 && digitalRead(DI_IN4_PIN) == DI_ACTIVE_LEVEL) {
    triggerEstop("manual");
    return;
  }

  if (VIBRATION_THRESHOLD_G > 0 && vibRms > VIBRATION_THRESHOLD_G) {
    triggerEstop("vibration");
    return;
  }
}

// OLED
void updateOled(float tempX, float vibRms) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart CNC Station");
  display.println("-----------------");
  display.printf("State: %s\n", stateToString(currentState));
  display.printf("TempX: %.1f C\n", tempX);
  display.printf("Vib:   %.3f g\n", vibRms);
  if (estopLatched) {
    display.printf("E-STOP: %s\n", estopReason.c_str());
  }
  display.display();
}

// setup / loop
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Smart CNC Station booting ===");

  I2C_A.begin(I2C_A_SDA, I2C_A_SCL, 400000);
  I2C_B.begin(I2C_B_SDA, I2C_B_SCL, 400000);

  if (!initSensors()) {
    Serial.println("WARNING: one or more sensors failed to init — check wiring before trusting data.");
  }
  if (!initOled()) {
    Serial.println("WARNING: OLED failed to init.");
  }

  if (DI_IN1_PIN >= 0) pinMode(DI_IN1_PIN, INPUT);
  if (DI_IN2_PIN >= 0) pinMode(DI_IN2_PIN, INPUT);
  if (DI_IN3_PIN >= 0) pinMode(DI_IN3_PIN, INPUT);
  if (DI_IN4_PIN >= 0) pinMode(DI_IN4_PIN, INPUT);
  if (DO_RELAY_ESTOP_PIN >= 0) {
    pinMode(DO_RELAY_ESTOP_PIN, OUTPUT);
    digitalWrite(DO_RELAY_ESTOP_PIN, HIGH); 
  } else {
    Serial.println("NOTE: DI/DO pins are placeholders (see config.h). "
                    "Job-status and E-stop relay logic will run in a "
                    "safe no-op mode until real GPIOs are confirmed.");
  }

  connectWiFi();
  mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqtt.connected()) {
    mqttReconnect();
  }
  mqtt.loop();

  sampleVibration();
  float vibRms = computeVibrationRMS();

  previousState = currentState;
  currentState = deriveJobState();

  checkEstopConditions(vibRms);

  if (millis() - lastPublishMs >= SENSOR_PUBLISH_INTERVAL_MS) {
    lastPublishMs = millis();

    float tempX = bmeX.readTemperature();
    mqttPublish(TOPIC_TEMP_X, String(tempX, 2));

    if (bmeY_present) {
      float tempY = bmeY.readTemperature();
      mqttPublish(TOPIC_TEMP_Y, String(tempY, 2));
    }

    mqttPublish(TOPIC_VIBRATION, String(vibRms, 4));

    float driverTemp;
    bool driverFault;
    if (readDriverTelemetry(driverTemp, driverFault)) {
      mqttPublish(TOPIC_DRIVER_TEMP, String(driverTemp, 1));
      mqttPublish(TOPIC_DRIVER_FAULT, driverFault ? "1" : "0");
    }

    if (currentState != previousState) {
      mqttPublish(TOPIC_STATE, stateToString(currentState));
      if (currentState == STATE_RUNNING && previousState != STATE_RUNNING) {
        mqttPublish(TOPIC_JOB_START, String(millis()));
      }
      if ((currentState == STATE_DONE || currentState == STATE_FAULTED) &&
          previousState == STATE_RUNNING) {
        String outcome = (currentState == STATE_DONE) ? "completed" : "faulted";
        String reason = estopLatched ? estopReason : "none";
        mqttPublish(TOPIC_JOB_END, "{\"outcome\":\"" + outcome + "\",\"stop_reason\":\"" + reason + "\"}");
      }
    } else {
      mqttPublish(TOPIC_STATE, stateToString(currentState));
    }
  }

  if (millis() - lastOledMs >= OLED_UPDATE_INTERVAL_MS) {
    lastOledMs = millis();
    updateOled(bmeX.readTemperature(), vibRms);
  }
}