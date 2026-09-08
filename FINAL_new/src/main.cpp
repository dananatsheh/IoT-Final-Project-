#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>
#include <time.h>
#include "config.h"

TwoWire &I2C_BUS1 = Wire;
TwoWire &I2C_BUS2 = Wire1;

Adafruit_BME280 bmeX;
Adafruit_BME280 bmeY;
Adafruit_MPU6050 mpu;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

enum JobState {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_FAULTED,
    STATE_DONE,
    STATE_UNKNOWN
};

JobState currentState = STATE_UNKNOWN;

bool bmeXOK = false;
bool bmeYOK = false;
bool mpuOK = false;
bool oledOK = false;

bool estopLatched = false;
String estopReason = "";

bool jobActive = false;
bool jobEndSent = false;

float axSamples[VIBRATION_WINDOW_SIZE];
float aySamples[VIBRATION_WINDOW_SIZE];
float azSamples[VIBRATION_WINDOW_SIZE];

int vibrationIndex = 0;
int vibrationCount = 0;

float tempX = NAN;
float tempY = NAN;
float vibrationRMS = NAN;

unsigned long lastVibrationSample = 0;
unsigned long lastSensorPublish = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;

bool devicePresent(TwoWire &bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

const char *stateToString(JobState state) {
    switch (state) {
        case STATE_IDLE: return "idle";
        case STATE_RUNNING: return "running";
        case STATE_FAULTED: return "faulted";
        case STATE_DONE: return "done";
        default: return "unknown";
    }
}

String getTimestamp() {
    time_t now;
    time(&now);

    if (now > 1700000000) {
        return String((unsigned long)now);
    }

    return "uptime_ms:" + String(millis());
}

void mqttPublish(const char *topic, const String &payload, bool retained = false) {
    if (mqtt.connected()) {
        mqtt.publish(topic, payload.c_str(), retained);
    }
}

void setEstopRelay(bool active) {
    digitalWrite(ESTOP_RELAY_PIN, active ? ESTOP_RELAY_ACTIVE_LEVEL : ESTOP_RELAY_INACTIVE_LEVEL);
}

void triggerEstop(const String &reason) {
    if (estopLatched) {
        return;
    }

    estopLatched = true;
    estopReason = reason;

    setEstopRelay(true);

    Serial.print("E-STOP TRIGGERED: ");
    Serial.println(reason);

    mqttPublish(TOPIC_ESTOP_EVENT, reason);
}

void resetEstop() {
    if (digitalRead(DI_IN4_PIN) == DI_ACTIVE_LEVEL) {
        Serial.println("E-STOP RESET BLOCKED");
        return;
    }

    estopLatched = false;
    estopReason = "";

    setEstopRelay(false);

    Serial.println("E-STOP RESET");

    mqttPublish(TOPIC_ESTOP_EVENT, "clear");
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    String message;

    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    message.trim();

    if (String(topic) == TOPIC_ESTOP_COMMAND) {
        if (message == "manual") {
            triggerEstop("manual");
        }

        if (message == "reset") {
            resetEstop();
        }
    }
}

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    if (millis() - lastWiFiAttempt < WIFI_RECONNECT_INTERVAL_MS) {
        return;
    }

    lastWiFiAttempt = millis();

    Serial.print("Connecting WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (mqtt.connected()) {
        mqtt.loop();
        return;
    }

    if (millis() - lastMQTTAttempt < MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }

    lastMQTTAttempt = millis();

    bool connected = false;

    if (strlen(MQTT_USERNAME) > 0) {
        connected = mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, TOPIC_DEVICE_STATUS, 0, true, "offline");
    } else {
        connected = mqtt.connect(MQTT_CLIENT_ID, TOPIC_DEVICE_STATUS, 0, true, "offline");
    }

    if (connected) {
        Serial.println("MQTT CONNECTED");

        mqtt.publish(TOPIC_DEVICE_STATUS, "online", true);
        mqtt.subscribe(TOPIC_ESTOP_COMMAND);

        mqttPublish(TOPIC_STATE, stateToString(currentState), true);
    } else {
        Serial.print("MQTT ERROR: ");
        Serial.println(mqtt.state());
    }
}

bool initSensors() {
    bmeXOK = devicePresent(I2C_BUS1, BME_X_ADDRESS);

    if (bmeXOK) {
        bmeXOK = bmeX.begin(BME_X_ADDRESS, &I2C_BUS1);
    }

    bmeYOK = devicePresent(I2C_BUS2, BME_Y_ADDRESS);

    if (bmeYOK) {
        bmeYOK = bmeY.begin(BME_Y_ADDRESS, &I2C_BUS2);
    }

    mpuOK = devicePresent(I2C_BUS2, MPU_ADDRESS);

    if (mpuOK) {
        mpuOK = mpu.begin(MPU_ADDRESS, &I2C_BUS2);
    }

    if (mpuOK) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    }

    Serial.print("BME280 X: ");
    Serial.println(bmeXOK ? "OK" : "ERROR");

    Serial.print("BME280 Y: ");
    Serial.println(bmeYOK ? "OK" : "ERROR");

    Serial.print("MPU6050: ");
    Serial.println(mpuOK ? "OK" : "ERROR");

    return bmeXOK && bmeYOK && mpuOK;
}

void initOLED() {
    oled.setI2CAddress(OLED_ADDRESS << 1);
    oled.begin();

    oledOK = true;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.setCursor(0, 12);
    oled.print("SMART CNC STATION");
    oled.setCursor(0, 28);
    oled.print("Starting...");
    oled.sendBuffer();
}

void sampleVibration() {
    if (!mpuOK) {
        return;
    }

    if (millis() - lastVibrationSample < VIBRATION_SAMPLE_INTERVAL_MS) {
        return;
    }

    lastVibrationSample = millis();

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    bool success = mpu.getEvent(&accel, &gyro, &temperature);

    if (!success) {
        return;
    }

    if (!isfinite(accel.acceleration.x) ||
        !isfinite(accel.acceleration.y) ||
        !isfinite(accel.acceleration.z)) {
        return;
    }

    axSamples[vibrationIndex] = accel.acceleration.x;
    aySamples[vibrationIndex] = accel.acceleration.y;
    azSamples[vibrationIndex] = accel.acceleration.z;

    vibrationIndex++;

    if (vibrationIndex >= VIBRATION_WINDOW_SIZE) {
        vibrationIndex = 0;
    }

    if (vibrationCount < VIBRATION_WINDOW_SIZE) {
        vibrationCount++;
    }
}

float calculateVibrationRMS() {
    if (vibrationCount < VIBRATION_WINDOW_SIZE) {
        return NAN;
    }

    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;

    for (int i = 0; i < vibrationCount; i++) {
        sumX += axSamples[i];
        sumY += aySamples[i];
        sumZ += azSamples[i];
    }

    float meanX = sumX / vibrationCount;
    float meanY = sumY / vibrationCount;
    float meanZ = sumZ / vibrationCount;

    float sumSquares = 0.0;

    for (int i = 0; i < vibrationCount; i++) {
        float dx = axSamples[i] - meanX;
        float dy = aySamples[i] - meanY;
        float dz = azSamples[i] - meanZ;

        sumSquares += dx * dx + dy * dy + dz * dz;
    }

    float vibrationMS2 = sqrt(sumSquares / vibrationCount);

    return vibrationMS2 / 9.80665f;
}

JobState deriveJobState() {
    if (estopLatched) {
        return STATE_FAULTED;
    }

    bool idleSignal = digitalRead(DI_IN1_PIN) == DI_ACTIVE_LEVEL;
    bool runningSignal = digitalRead(DI_IN2_PIN) == DI_ACTIVE_LEVEL;
    bool faultSignal = digitalRead(DI_IN3_PIN) == DI_ACTIVE_LEVEL;

    if (faultSignal) {
        return STATE_FAULTED;
    }

    if (runningSignal) {
        return STATE_RUNNING;
    }

    if (idleSignal) {
        if (currentState == STATE_RUNNING || currentState == STATE_DONE) {
            return STATE_DONE;
        }

        return STATE_IDLE;
    }

    return STATE_UNKNOWN;
}

void publishJobStart() {
    jobActive = true;
    jobEndSent = false;

    mqttPublish(TOPIC_JOB_START, getTimestamp());
}

void publishJobEnd(const String &outcome, const String &reason) {
    if (!jobActive || jobEndSent) {
        return;
    }

    jobActive = false;
    jobEndSent = true;

    String payload = "{\"outcome\":\"" + outcome +
                     "\",\"stop_reason\":\"" + reason +
                     "\",\"timestamp\":\"" + getTimestamp() + "\"}";

    mqttPublish(TOPIC_JOB_END, payload);
}

void processJobState(JobState newState) {
    if (newState == currentState) {
        return;
    }

    JobState previousState = currentState;
    currentState = newState;

    mqttPublish(TOPIC_STATE, stateToString(currentState), true);

    Serial.print("JOB STATE: ");
    Serial.println(stateToString(currentState));

    if (currentState == STATE_RUNNING && previousState != STATE_RUNNING) {
        publishJobStart();
    }

    if (currentState == STATE_DONE && jobActive) {
        publishJobEnd("completed", "none");
    }

    if (currentState == STATE_FAULTED && jobActive) {
        String reason = estopLatched ? estopReason : "machine_fault";
        publishJobEnd("stopped", reason);
    }
}

void checkSafety() {
    if (estopLatched) {
        return;
    }

    if (digitalRead(DI_IN4_PIN) == DI_ACTIVE_LEVEL) {
        triggerEstop("manual");
        return;
    }

    if (currentState == STATE_RUNNING &&
        isfinite(vibrationRMS) &&
        vibrationRMS > VIBRATION_THRESHOLD_G) {
        triggerEstop("vibration");
    }
}

void readTemperatures() {
    tempX = bmeXOK ? bmeX.readTemperature() : NAN;
    tempY = bmeYOK ? bmeY.readTemperature() : NAN;

    if (!isfinite(tempX) || tempX < -40.0 || tempX > 85.0) {
        tempX = NAN;
    }

    if (!isfinite(tempY) || tempY < -40.0 || tempY > 85.0) {
        tempY = NAN;
    }
}

void publishSensors() {
    if (isfinite(tempX)) {
        mqttPublish(TOPIC_TEMP_X, String(tempX, 2));
    }

    if (isfinite(tempY)) {
        mqttPublish(TOPIC_TEMP_Y, String(tempY, 2));
    }

    if (isfinite(vibrationRMS)) {
        mqttPublish(TOPIC_VIBRATION, String(vibrationRMS, 5));
    }
}

void updateOLED() {
    if (!oledOK) {
        return;
    }

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);

    oled.setCursor(0, 9);
    oled.print("SMART CNC STATION");

    oled.setCursor(0, 19);
    oled.print("State: ");
    oled.print(stateToString(currentState));

    oled.setCursor(0, 29);
    oled.print("X:");

    if (isfinite(tempX)) {
        oled.print(tempX, 1);
    } else {
        oled.print("ERR");
    }

    oled.print(" Y:");

    if (isfinite(tempY)) {
        oled.print(tempY, 1);
    } else {
        oled.print("ERR");
    }

    oled.print(" C");

    oled.setCursor(0, 39);
    oled.print("Vib: ");

    if (isfinite(vibrationRMS)) {
        oled.print(vibrationRMS, 4);
    } else {
        oled.print("----");
    }

    oled.print(" g");

    oled.setCursor(0, 49);
    oled.print("Limit: ");
    oled.print(VIBRATION_THRESHOLD_G, 4);

    oled.setCursor(0, 59);
    oled.print("EStop: ");

    if (estopLatched) {
        oled.print(estopReason);
    } else {
        oled.print("clear");
    }

    oled.sendBuffer();
}

void printStatus() {
    Serial.println();
    Serial.println("------------------------");

    Serial.print("State: ");
    Serial.println(stateToString(currentState));

    Serial.print("Temperature X: ");

    if (isfinite(tempX)) {
        Serial.print(tempX, 2);
        Serial.println(" C");
    } else {
        Serial.println("ERROR");
    }

    Serial.print("Temperature Y: ");

    if (isfinite(tempY)) {
        Serial.print(tempY, 2);
        Serial.println(" C");
    } else {
        Serial.println("ERROR");
    }

    Serial.print("Vibration RMS: ");

    if (isfinite(vibrationRMS)) {
        Serial.print(vibrationRMS, 5);
        Serial.println(" g");
    } else {
        Serial.println("WAITING");
    }

    Serial.print("Threshold: ");
    Serial.print(VIBRATION_THRESHOLD_G, 5);
    Serial.println(" g");

    Serial.print("E-Stop: ");
    Serial.println(estopLatched ? estopReason : "clear");

    Serial.print("WiFi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "offline");

    Serial.print("MQTT: ");
    Serial.println(mqtt.connected() ? "connected" : "offline");

    Serial.println("------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("==============================");
    Serial.println("SMART CNC STATION");
    Serial.println("==============================");

    I2C_BUS1.begin(SDA_BUS1, SCL_BUS1, I2C_CLOCK_HZ);
    I2C_BUS2.begin(SDA_BUS2, SCL_BUS2, I2C_CLOCK_HZ);

    pinMode(DI_IN1_PIN, INPUT);
    pinMode(DI_IN2_PIN, INPUT);
    pinMode(DI_IN3_PIN, INPUT);
    pinMode(DI_IN4_PIN, INPUT);

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    setEstopRelay(false);

    initSensors();
    initOLED();

    WiFi.mode(WIFI_STA);

    mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    mqtt.setCallback(mqttCallback);

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    currentState = deriveJobState();

    Serial.println();
    Serial.print("Vibration threshold: ");
    Serial.print(VIBRATION_THRESHOLD_G, 5);
    Serial.println(" g");

    Serial.println("SYSTEM READY");
}

void loop() {
    connectWiFi();
    connectMQTT();

    sampleVibration();
    vibrationRMS = calculateVibrationRMS();

    JobState newState = deriveJobState();
    processJobState(newState);

    checkSafety();

    if (estopLatched && currentState != STATE_FAULTED) {
        processJobState(STATE_FAULTED);
    }

    if (millis() - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL_MS) {
        lastSensorPublish = millis();

        readTemperatures();
        publishSensors();
        mqttPublish(TOPIC_STATE, stateToString(currentState), true);
        printStatus();
    }

    if (millis() - lastOLEDUpdate >= OLED_UPDATE_INTERVAL_MS) {
        lastOLEDUpdate = millis();
        updateOLED();
    }

    delay(1);
}