#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

/* ======================================================
   WIFI
   ====================================================== */

#define WIFI_SSID "CYBER_EXT"
#define WIFI_PASSWORD "cyberap2025"

/* ======================================================
   MQTT
   SAME BROKER AS FIRST ESP32
   ====================================================== */

#define MQTT_BROKER_HOST "192.168.1.18"
#define MQTT_BROKER_PORT 1883

/*
   IMPORTANT:
   Must be DIFFERENT from first ESP32 client ID.
*/
#define MQTT_CLIENT_ID "smart-cnc-vibration-esp32"

#define MQTT_USERNAME "smartcnc"
#define MQTT_PASSWORD "awesomproj"

/* ======================================================
   MQTT TOPIC
   Keep this exact topic because Node-RED uses it.
   ====================================================== */

#define TOPIC_VIBRATION "sensor/mpu6050/vibration"

/* ======================================================
   MPU CONFIGURATION
   ====================================================== */

#define MPU_ADDRESS 0x68

#define MPU_SDA 21
#define MPU_SCL 23

#define I2C_CLOCK_HZ 100000

/*
   Accelerometer configured to +/-4 g.

   Scale:
   +/-2 g = 16384 LSB/g
   +/-4 g = 8192 LSB/g
*/

const float ACCEL_SCALE = 8192.0f;

/* ======================================================
   VIBRATION SETTINGS
   ====================================================== */

/*
   Threshold learned from your real CNC test.
*/

const float VIBRATION_THRESHOLD_G = 1.0f;

/*
   Sampling frequency = 200 Hz

   5000 us = 5 ms
*/

#define VIBRATION_SAMPLE_INTERVAL_US 5000

/*
   100 samples at 200 Hz
   = 0.5 second RMS window
*/

#define VIBRATION_WINDOW_SIZE 100

/*
   Require two consecutive high RMS windows.

   2 x 0.5 sec = approximately 1 second.
*/

#define FAULT_WINDOWS_REQUIRED 2

/*
   High-pass filter.

   Reduces slower CNC commanded motion
   while keeping higher-frequency vibration.
*/

const float HP_ALPHA = 0.864f;

/* ======================================================
   STATIONARY CALIBRATION
   ====================================================== */

#define CALIBRATION_SAMPLES 1000
#define MAX_CALIBRATION_FAILURES 200

float baselineX = 0.0f;
float baselineY = 0.0f;
float baselineZ = 0.0f;

/* ======================================================
   FILTER VARIABLES
   ====================================================== */

float previousX = 0.0f;
float previousY = 0.0f;
float previousZ = 0.0f;

float hpX = 0.0f;
float hpY = 0.0f;
float hpZ = 0.0f;

/* ======================================================
   RMS VARIABLES
   ====================================================== */

float rmsSumSquares = 0.0f;

int rmsSampleCount = 0;

float vibrationRMS = 0.0f;

int faultCounter = 0;

bool vibrationFault = false;

/* ======================================================
   TIMERS
   ====================================================== */

unsigned long lastMPUSample = 0;

unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;

#define WIFI_RECONNECT_INTERVAL_MS 10000
#define MQTT_RECONNECT_INTERVAL_MS 3000

/* ======================================================
   NETWORK OBJECTS
   ====================================================== */

WiFiClient wifiClient;

PubSubClient mqttClient(
    wifiClient
);

/* ======================================================
   FUNCTION DECLARATIONS
   ====================================================== */

bool readRegister(
    uint8_t reg,
    uint8_t &value
);

bool writeRegister(
    uint8_t reg,
    uint8_t value
);

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
);

bool calibrateMPU();

void updateVibration();

void processVibration(
    float rms
);

void publishVibration(
    float rms
);

void connectWiFi();

void connectMQTT();

/* ======================================================
   READ MPU REGISTER
   ====================================================== */

bool readRegister(
    uint8_t reg,
    uint8_t &value
)
{
    Wire.beginTransmission(
        MPU_ADDRESS
    );

    Wire.write(
        reg
    );

    if (
        Wire.endTransmission(false)
        != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDRESS,
            (uint8_t)1,
            (uint8_t)true
        );

    if (received != 1)
    {
        while (Wire.available())
        {
            Wire.read();
        }

        return false;
    }

    value =
        Wire.read();

    return true;
}

/* ======================================================
   WRITE MPU REGISTER
   ====================================================== */

bool writeRegister(
    uint8_t reg,
    uint8_t value
)
{
    Wire.beginTransmission(
        MPU_ADDRESS
    );

    Wire.write(
        reg
    );

    Wire.write(
        value
    );

    return (
        Wire.endTransmission()
        == 0
    );
}

/* ======================================================
   READ ACCELERATION
   ====================================================== */

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
)
{
    Wire.beginTransmission(
        MPU_ADDRESS
    );

    /*
       Accelerometer starts at register 0x3B.
    */

    Wire.write(
        0x3B
    );

    if (
        Wire.endTransmission(false)
        != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDRESS,
            (uint8_t)6,
            (uint8_t)true
        );

    if (received != 6)
    {
        while (Wire.available())
        {
            Wire.read();
        }

        return false;
    }

    ax =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    ay =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    az =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    return true;
}

/* ======================================================
   MPU STATIONARY CALIBRATION
   ====================================================== */

bool calibrateMPU()
{
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    int validSamples = 0;
    int failedSamples = 0;

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "MPU STATIONARY CALIBRATION"
    );

    Serial.println(
        "================================"
    );

    Serial.println();

    Serial.println(
        "KEEP CNC COMPLETELY STILL"
    );

    Serial.println(
        "Calibration starts in 3 seconds..."
    );

    delay(
        3000
    );

    while (
        validSamples <
        CALIBRATION_SAMPLES
    )
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        if (
            readAcceleration(
                ax,
                ay,
                az
            )
        )
        {
            sumX +=
                (float)ax /
                ACCEL_SCALE;

            sumY +=
                (float)ay /
                ACCEL_SCALE;

            sumZ +=
                (float)az /
                ACCEL_SCALE;

            validSamples++;

            if (
                validSamples %
                    200 ==
                0
            )
            {
                Serial.print(
                    "Calibration: "
                );

                Serial.print(
                    validSamples
                );

                Serial.print(
                    " / "
                );

                Serial.println(
                    CALIBRATION_SAMPLES
                );
            }
        }

        else
        {
            failedSamples++;

            if (
                failedSamples %
                    10 ==
                0
            )
            {
                Serial.print(
                    "I2C failures: "
                );

                Serial.println(
                    failedSamples
                );
            }

            if (
                failedSamples >=
                MAX_CALIBRATION_FAILURES
            )
            {
                Serial.println(
                    "CALIBRATION FAILED"
                );

                return false;
            }
        }

        delay(
            3
        );
    }

    baselineX =
        sumX /
        validSamples;

    baselineY =
        sumY /
        validSamples;

    baselineZ =
        sumZ /
        validSamples;

    /*
       Initialize high-pass filter.
    */

    previousX =
        baselineX;

    previousY =
        baselineY;

    previousZ =
        baselineZ;

    hpX = 0.0f;
    hpY = 0.0f;
    hpZ = 0.0f;

    Serial.println();

    Serial.println(
        "CALIBRATION COMPLETE"
    );

    Serial.print(
        "Baseline X: "
    );

    Serial.println(
        baselineX,
        4
    );

    Serial.print(
        "Baseline Y: "
    );

    Serial.println(
        baselineY,
        4
    );

    Serial.print(
        "Baseline Z: "
    );

    Serial.println(
        baselineZ,
        4
    );

    return true;
}

/* ======================================================
   WIFI
   ====================================================== */

void connectWiFi()
{
    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        return;
    }

    Serial.println();

    Serial.println(
        "Connecting to WiFi..."
    );

    WiFi.mode(
        WIFI_STA
    );

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    unsigned long start =
        millis();

    while (
        WiFi.status() !=
            WL_CONNECTED &&
        millis() - start <
            15000
    )
    {
        delay(
            500
        );

        Serial.print(
            "."
        );
    }

    Serial.println();

    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        Serial.println(
            "WiFi CONNECTED"
        );

        Serial.print(
            "Vibration ESP32 IP: "
        );

        Serial.println(
            WiFi.localIP()
        );

        Serial.print(
            "MQTT Broker: "
        );

        Serial.print(
            MQTT_BROKER_HOST
        );

        Serial.print(
            ":"
        );

        Serial.println(
            MQTT_BROKER_PORT
        );
    }

    else
    {
        Serial.println(
            "WiFi CONNECTION FAILED"
        );
    }
}

/* ======================================================
   MQTT CONNECTION
   ====================================================== */

void connectMQTT()
{
    if (
        WiFi.status() !=
        WL_CONNECTED
    )
    {
        return;
    }

    if (
        mqttClient.connected()
    )
    {
        return;
    }

    Serial.println();

    Serial.print(
        "Connecting to MQTT broker "
    );

    Serial.print(
        MQTT_BROKER_HOST
    );

    Serial.print(
        ":"
    );

    Serial.println(
        MQTT_BROKER_PORT
    );

    bool connected =
        mqttClient.connect(
            MQTT_CLIENT_ID,
            MQTT_USERNAME,
            MQTT_PASSWORD
        );

    if (connected)
    {
        Serial.println(
            "MQTT CONNECTED"
        );

        Serial.print(
            "Publishing vibration to: "
        );

        Serial.println(
            TOPIC_VIBRATION
        );
    }

    else
    {
        Serial.print(
            "MQTT FAILED | State = "
        );

        Serial.println(
            mqttClient.state()
        );
    }
}

/* ======================================================
   PUBLISH VIBRATION
   ====================================================== */

void publishVibration(
    float rms
)
{
    if (
        !mqttClient.connected()
    )
    {
        return;
    }

    char payload[16];

    /*
       Publish ONLY the number.

       Example:
       0.347

       This is ideal for the Node-RED
       gauge and line chart.
    */

    snprintf(
        payload,
        sizeof(payload),
        "%.3f",
        rms
    );

    mqttClient.publish(
        TOPIC_VIBRATION,
        payload
    );
}

/* ======================================================
   PROCESS RMS VALUE
   ====================================================== */

void processVibration(
    float rms
)
{
    /*
       Publish EVERY calculated RMS window.
    */

    publishVibration(
        rms
    );

    Serial.print(
        "Vibration RMS: "
    );

    Serial.print(
        rms,
        3
    );

    Serial.print(
        " g | Threshold: "
    );

    Serial.print(
        VIBRATION_THRESHOLD_G,
        3
    );

    Serial.print(
        " g"
    );

    /* ==================================================
       HIGH VIBRATION
       ================================================== */

    if (
        rms >
        VIBRATION_THRESHOLD_G
    )
    {
        faultCounter++;

        Serial.print(
            " | HIGH "
        );

        Serial.print(
            faultCounter
        );

        Serial.print(
            "/"
        );

        Serial.print(
            FAULT_WINDOWS_REQUIRED
        );

        if (
            faultCounter >=
            FAULT_WINDOWS_REQUIRED
        )
        {
            vibrationFault =
                true;

            Serial.print(
                " | ABNORMAL VIBRATION"
            );
        }
    }

    /* ==================================================
       NORMAL
       ================================================== */

    else
    {
        faultCounter = 0;

        if (!vibrationFault)
        {
            Serial.print(
                " | NORMAL"
            );
        }

        else
        {
            Serial.print(
                " | FAULT LATCHED"
            );
        }
    }

    Serial.println();
}

/* ======================================================
   UPDATE VIBRATION
   ====================================================== */

void updateVibration()
{
    unsigned long now =
        micros();

    if (
        now - lastMPUSample <
        VIBRATION_SAMPLE_INTERVAL_US
    )
    {
        return;
    }

    lastMPUSample +=
        VIBRATION_SAMPLE_INTERVAL_US;

    int16_t rawAX;
    int16_t rawAY;
    int16_t rawAZ;

    if (
        !readAcceleration(
            rawAX,
            rawAY,
            rawAZ
        )
    )
    {
        Serial.println(
            "MPU I2C READ FAILED - IGNORED"
        );

        return;
    }

    /* ==================================================
       RAW -> g
       ================================================== */

    float ax =
        (float)rawAX /
        ACCEL_SCALE;

    float ay =
        (float)rawAY /
        ACCEL_SCALE;

    float az =
        (float)rawAZ /
        ACCEL_SCALE;

    /* ==================================================
       HIGH-PASS FILTER
       ================================================== */

    hpX =
        HP_ALPHA *
        (
            hpX +
            ax -
            previousX
        );

    hpY =
        HP_ALPHA *
        (
            hpY +
            ay -
            previousY
        );

    hpZ =
        HP_ALPHA *
        (
            hpZ +
            az -
            previousZ
        );

    previousX =
        ax;

    previousY =
        ay;

    previousZ =
        az;

    /* ==================================================
       INSTANTANEOUS VIBRATION
       ================================================== */

    float vibration =
        sqrt(
            hpX * hpX +
            hpY * hpY +
            hpZ * hpZ
        );

    /* ==================================================
       RMS
       ================================================== */

    rmsSumSquares +=
        vibration *
        vibration;

    rmsSampleCount++;

    if (
        rmsSampleCount >=
        VIBRATION_WINDOW_SIZE
    )
    {
        vibrationRMS =
            sqrt(
                rmsSumSquares /
                VIBRATION_WINDOW_SIZE
            );

        rmsSumSquares =
            0.0f;

        rmsSampleCount =
            0;

        processVibration(
            vibrationRMS
        );
    }
}

/* ======================================================
   SETUP
   ====================================================== */

void setup()
{
    Serial.begin(
        115200
    );

    delay(
        1000
    );

    Serial.println();

    Serial.println(
        "======================================"
    );

    Serial.println(
        "SMART CNC VIBRATION ESP32"
    );

    Serial.println(
        "======================================"
    );

    /* ==================================================
       MPU I2C
       ================================================== */

    Wire.begin(
        MPU_SDA,
        MPU_SCL
    );

    Wire.setClock(
        I2C_CLOCK_HZ
    );

    delay(
        500
    );

    /* ==================================================
       CHECK MPU
       ================================================== */

    uint8_t whoAmI =
        0;

    if (
        !readRegister(
            0x75,
            whoAmI
        )
    )
    {
        Serial.println(
            "ERROR: MPU NOT FOUND"
        );

        while (true)
        {
            delay(
                1000
            );
        }
    }

    Serial.print(
        "WHO_AM_I = 0x"
    );

    Serial.println(
        whoAmI,
        HEX
    );

    /*
       Your sensor previously reported 0x70.
       Accept 0x70 or 0x71.
    */

    if (
        whoAmI == 0x70 ||
        whoAmI == 0x71
    )
    {
        Serial.println(
            "MPU DETECTED"
        );
    }

    else
    {
        Serial.println(
            "WARNING: Unexpected MPU ID"
        );
    }

    /* ==================================================
       WAKE SENSOR
       ================================================== */

    writeRegister(
        0x6B,
        0x00
    );

    delay(
        100
    );

    /* ==================================================
       ACCELEROMETER +/-4g
       ================================================== */

    writeRegister(
        0x1C,
        0x08
    );

    /* ==================================================
       MPU DIGITAL LOW PASS FILTER
       ================================================== */

    writeRegister(
        0x1A,
        0x03
    );

    delay(
        500
    );

    /* ==================================================
       STATIONARY CALIBRATION
       ================================================== */

    if (!calibrateMPU())
    {
        Serial.println(
            "MPU CALIBRATION FAILED"
        );

        while (true)
        {
            delay(
                1000
            );
        }
    }

    /* ==================================================
       WIFI
       ================================================== */

    connectWiFi();

    /* ==================================================
       MQTT
       ================================================== */

    mqttClient.setServer(
        MQTT_BROKER_HOST,
        MQTT_BROKER_PORT
    );

    connectMQTT();

    lastMPUSample =
        micros();

    Serial.println();

    Serial.println(
        "======================================"
    );

    Serial.println(
        "VIBRATION MONITORING STARTED"
    );

    Serial.println(
        "======================================"
    );

    Serial.print(
        "MQTT Topic: "
    );

    Serial.println(
        TOPIC_VIBRATION
    );

    Serial.print(
        "Threshold: "
    );

    Serial.print(
        VIBRATION_THRESHOLD_G,
        3
    );

    Serial.println(
        " g RMS"
    );
}

/* ======================================================
   LOOP
   ====================================================== */

void loop()
{
    /* ==================================================
       WIFI RECONNECT
       ================================================== */

    if (
        WiFi.status() !=
        WL_CONNECTED
    )
    {
        if (
            millis() -
                lastWiFiAttempt >=
            WIFI_RECONNECT_INTERVAL_MS
        )
        {
            lastWiFiAttempt =
                millis();

            connectWiFi();
        }
    }

    /* ==================================================
       MQTT RECONNECT
       ================================================== */

    if (
        WiFi.status() ==
            WL_CONNECTED &&
        !mqttClient.connected()
    )
    {
        if (
            millis() -
                lastMQTTAttempt >=
            MQTT_RECONNECT_INTERVAL_MS
        )
        {
            lastMQTTAttempt =
                millis();

            connectMQTT();
        }
    }

    /* ==================================================
       MQTT
       ================================================== */

    if (
        mqttClient.connected()
    )
    {
        mqttClient.loop();
    }

    /* ==================================================
       VIBRATION
       ================================================== */

    updateVibration();

    delay(
        1
    );
}