#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>

/* ======================================================
   WIFI CONFIGURATION
   ====================================================== */

#define WIFI_SSID "CYBER_EXT"
#define WIFI_PASSWORD "cyberap2025"

/* ======================================================
   MQTT CONFIGURATION
   Mosquitto Broker = Windows Laptop
   ====================================================== */

#define MQTT_BROKER_HOST "192.168.1.18"
#define MQTT_BROKER_PORT 1883

#define MQTT_CLIENT_ID "smart-cnc-esp32"

#define MQTT_USERNAME "smartcnc"
#define MQTT_PASSWORD "awesomproj"

/* ======================================================
   MQTT TOPICS
   ====================================================== */

// ESP32 -> Node-RED
#define TOPIC_STATE         "job/status/state"
#define TOPIC_JOB_START     "job/status/start"
#define TOPIC_JOB_END       "job/status/end"

#define TOPIC_DEVICE_STATUS "device/esp32/status"

#define TOPIC_TEMP_X        "sensor/bme280_x/temperature"
#define TOPIC_TEMP_Y        "sensor/bme280_y/temperature"

#define TOPIC_ESTOP_EVENT   "estop/status/triggered"

// Node-RED -> ESP32
#define TOPIC_ESTOP_COMMAND "estop/command"

/* ======================================================
   BME280 PINS
   ====================================================== */

// BME280 X
#define SDA_BUS1 21
#define SCL_BUS1 23

// BME280 Y
#define SDA_BUS2 32
#define SCL_BUS2 33

/* ======================================================
   OLED PINS
   ====================================================== */

#define OLED_SDA 4
#define OLED_SCL 16

/* ======================================================
   RDC6445S INPUTS
   ES32C14
   ====================================================== */

#define RDC_OUT1_PIN 19
#define RDC_OUT2_PIN 18

/* ======================================================
   RELAY
   ES32C14 Relay 1 = GPIO27
   ====================================================== */

#define RELAY_PIN 27

#define RELAY_ACTIVE_LEVEL HIGH
#define RELAY_INACTIVE_LEVEL LOW

/* ======================================================
   I2C ADDRESSES
   ====================================================== */

// Both sensors can use 0x76 because
// they are on different I2C buses.

#define BME_X_ADDRESS 0x76
#define BME_Y_ADDRESS 0x76

#define OLED_ADDRESS 0x3C

#define I2C_CLOCK_HZ 100000

/* ======================================================
   TEMPERATURE SAFETY
   ====================================================== */

// Trip relay at 50 C
#define TEMP_HIGH_THRESHOLD_C 50.0f

// Release thermal trip below 48 C
#define TEMP_RESET_THRESHOLD_C 48.0f

/* ======================================================
   TIMING
   ====================================================== */

#define SENSOR_INTERVAL_MS 1000
#define OLED_INTERVAL_MS 250
#define SERIAL_INTERVAL_MS 1000
#define MQTT_PUBLISH_INTERVAL_MS 1000

#define WIFI_RECONNECT_INTERVAL_MS 10000
#define MQTT_RECONNECT_INTERVAL_MS 3000

#define DONE_DISPLAY_TIME_MS 2000

/* ======================================================
   I2C BUSES
   ====================================================== */

TwoWire BME_X_BUS = TwoWire(0);
TwoWire BME_Y_BUS = TwoWire(1);

/* ======================================================
   BME280 OBJECTS
   ====================================================== */

Adafruit_BME280 bmeX;
Adafruit_BME280 bmeY;

bool bmeXOK = false;
bool bmeYOK = false;

float tempX = NAN;
float tempY = NAN;

/* ======================================================
   OLED
   Software I2C
   ====================================================== */

U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(
    U8G2_R0,
    OLED_SCL,
    OLED_SDA,
    U8X8_PIN_NONE
);

bool oledOK = false;

/* ======================================================
   RELAY / SAFETY STATE
   ====================================================== */

bool relayActive = false;

bool thermalTripActive = false;
bool manualStopActive = false;

/* ======================================================
   CNC STATE
   ====================================================== */

enum CNCState
{
    CNC_IDLE,
    CNC_RUNNING,
    CNC_DONE,
    CNC_FAULT
};

CNCState currentState = CNC_IDLE;
CNCState previousState = CNC_IDLE;

bool wasRunning = false;

unsigned long doneStartTime = 0;

/* ======================================================
   WIFI / MQTT
   ====================================================== */

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/* ======================================================
   TIMERS
   ====================================================== */

unsigned long lastSensorRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastMQTTPublish = 0;

unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;

/* ======================================================
   FUNCTION DECLARATIONS
   ====================================================== */

const char *stateToString(CNCState state);

void setRelay(bool active);
void applyRelayState();

void readTemperatures();
void checkTemperatureSafety();

void updateCNCState();
void handleStateChange();

void updateOLED();
void printStatus();

void handleSerialCommands();

void connectWiFi();
void connectMQTT();

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length
);

void publishTemperatures();
void publishState();

void publishJobEnd(
    const char *outcome,
    const char *reason
);

void triggerManualStop();
void resetManualStop();

/* ======================================================
   CNC STATE -> TEXT
   ====================================================== */

const char *stateToString(CNCState state)
{
    switch (state)
    {
        case CNC_IDLE:
            return "IDLE";

        case CNC_RUNNING:
            return "RUNNING";

        case CNC_DONE:
            return "DONE";

        case CNC_FAULT:
            return "FAULTED";

        default:
            return "UNKNOWN";
    }
}

/* ======================================================
   RELAY CONTROL
   ====================================================== */

void setRelay(bool active)
{
    relayActive = active;

    digitalWrite(
        RELAY_PIN,
        active
            ? RELAY_ACTIVE_LEVEL
            : RELAY_INACTIVE_LEVEL
    );
}

/* ======================================================
   APPLY SAFETY RELAY STATE
   ====================================================== */

void applyRelayState()
{
    /*
       Relay energizes if:

       1. Temperature >= 50 C
       OR
       2. Manual E-stop requested
    */

    bool shouldActivate =
        thermalTripActive ||
        manualStopActive;

    setRelay(
        shouldActivate
    );
}

/* ======================================================
   READ TEMPERATURES
   ====================================================== */

void readTemperatures()
{
    /* --------------------------
       X MOTOR SENSOR
       -------------------------- */

    if (bmeXOK)
    {
        tempX =
            bmeX.readTemperature();

        if (!isfinite(tempX))
        {
            tempX = NAN;
        }
    }

    /* --------------------------
       Y MOTOR SENSOR
       -------------------------- */

    if (bmeYOK)
    {
        tempY =
            bmeY.readTemperature();

        if (!isfinite(tempY))
        {
            tempY = NAN;
        }
    }
}

/* ======================================================
   TEMPERATURE SAFETY
   ====================================================== */

void checkTemperatureSafety()
{
    bool xHigh =
        isfinite(tempX) &&
        tempX >= TEMP_HIGH_THRESHOLD_C;

    bool yHigh =
        isfinite(tempY) &&
        tempY >= TEMP_HIGH_THRESHOLD_C;

    /* ==================================================
       NEW THERMAL TRIP
       ================================================== */

    if (
        !thermalTripActive &&
        (xHigh || yHigh)
    )
    {
        thermalTripActive = true;

        applyRelayState();

        Serial.println();
        Serial.println(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );

        Serial.println(
            "HIGH TEMPERATURE DETECTED"
        );

        if (xHigh)
        {
            Serial.print(
                "X Temperature: "
            );

            Serial.print(
                tempX,
                2
            );

            Serial.println(
                " C"
            );
        }

        if (yHigh)
        {
            Serial.print(
                "Y Temperature: "
            );

            Serial.print(
                tempY,
                2
            );

            Serial.println(
                " C"
            );
        }

        Serial.println(
            "RELAY ENERGIZED"
        );

        Serial.println(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );

        /* ----------------------------------------------
           MQTT EVENT
           ---------------------------------------------- */

        if (mqttClient.connected())
        {
            if (xHigh && yHigh)
            {
                mqttClient.publish(
                    TOPIC_ESTOP_EVENT,
                    "high_temperature_x_y"
                );

                if (
                    currentState ==
                    CNC_RUNNING
                )
                {
                    publishJobEnd(
                        "stopped",
                        "high_temperature_x_y"
                    );
                }
            }

            else if (xHigh)
            {
                mqttClient.publish(
                    TOPIC_ESTOP_EVENT,
                    "high_temperature_x"
                );

                if (
                    currentState ==
                    CNC_RUNNING
                )
                {
                    publishJobEnd(
                        "stopped",
                        "high_temperature_x"
                    );
                }
            }

            else
            {
                mqttClient.publish(
                    TOPIC_ESTOP_EVENT,
                    "high_temperature_y"
                );

                if (
                    currentState ==
                    CNC_RUNNING
                )
                {
                    publishJobEnd(
                        "stopped",
                        "high_temperature_y"
                    );
                }
            }
        }

        /*
           Do not allow this trip to later
           become a normal DONE event.
        */

        wasRunning = false;
    }

    /* ==================================================
       TEMPERATURE RESET
       ================================================== */

    if (thermalTripActive)
    {
        bool xSafe =
            !isfinite(tempX) ||
            tempX <= TEMP_RESET_THRESHOLD_C;

        bool ySafe =
            !isfinite(tempY) ||
            tempY <= TEMP_RESET_THRESHOLD_C;

        if (
            xSafe &&
            ySafe
        )
        {
            thermalTripActive = false;

            applyRelayState();

            Serial.println();
            Serial.println(
                "TEMPERATURE RETURNED TO SAFE LEVEL"
            );

            Serial.println(
                "THERMAL TRIP CLEARED"
            );
        }
    }
}

/* ======================================================
   RDC6445S CNC STATE
   ====================================================== */

void updateCNCState()
{
    int out1 =
        digitalRead(
            RDC_OUT1_PIN
        );

    int out2 =
        digitalRead(
            RDC_OUT2_PIN
        );

    /*
       OUT1:
       LOW  = Idle
       HIGH = Running / Active

       OUT2:
       LOW  = RDC Fault
       HIGH = No Fault
    */

    bool idleSignal =
        (out1 == LOW);

    bool rdcFaultSignal =
        (out2 == LOW);

    /* ==================================================
       LOCAL SAFETY TRIP
       ================================================== */

    if (
        thermalTripActive ||
        manualStopActive
    )
    {
        currentState =
            CNC_FAULT;

        wasRunning =
            false;

        return;
    }

    /* ==================================================
       RDC FAULT
       ================================================== */

    if (rdcFaultSignal)
    {
        currentState =
            CNC_FAULT;

        wasRunning =
            false;

        return;
    }

    /* ==================================================
       DONE TIMER
       ================================================== */

    if (
        currentState ==
        CNC_DONE
    )
    {
        if (
            millis() -
                doneStartTime <
            DONE_DISPLAY_TIME_MS
        )
        {
            return;
        }

        currentState =
            CNC_IDLE;
    }

    /* ==================================================
       IDLE / DONE
       ================================================== */

    if (idleSignal)
    {
        if (wasRunning)
        {
            currentState =
                CNC_DONE;

            doneStartTime =
                millis();

            wasRunning =
                false;
        }

        else
        {
            currentState =
                CNC_IDLE;
        }
    }

    /* ==================================================
       RUNNING
       ================================================== */

    else
    {
        currentState =
            CNC_RUNNING;

        wasRunning =
            true;
    }
}

/* ======================================================
   HANDLE CNC STATE CHANGE
   ====================================================== */

void handleStateChange()
{
    if (
        currentState ==
        previousState
    )
    {
        return;
    }

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.print(
        "CNC STATE: "
    );

    Serial.println(
        stateToString(
            currentState
        )
    );

    Serial.println(
        "=============================="
    );

    /* ==================================================
       MQTT STATE
       ================================================== */

    if (mqttClient.connected())
    {
        mqttClient.publish(
            TOPIC_STATE,
            stateToString(
                currentState
            ),
            true
        );

        /* ----------------------------------------------
           JOB START
           ---------------------------------------------- */

        if (
            currentState ==
            CNC_RUNNING
        )
        {
            mqttClient.publish(
                TOPIC_JOB_START,
                "started"
            );
        }

        /* ----------------------------------------------
           NORMAL JOB END
           ---------------------------------------------- */

        else if (
            currentState ==
            CNC_DONE
        )
        {
            publishJobEnd(
                "completed",
                "normal_completion"
            );
        }

        /* ----------------------------------------------
           RDC FAULT
           ---------------------------------------------- */

        else if (
            currentState ==
                CNC_FAULT &&
            !thermalTripActive &&
            !manualStopActive
        )
        {
            publishJobEnd(
                "faulted",
                "rdc_fault"
            );
        }
    }

    previousState =
        currentState;
}

/* ======================================================
   MANUAL E-STOP
   ====================================================== */

void triggerManualStop()
{
    if (manualStopActive)
    {
        return;
    }

    manualStopActive =
        true;

    /*
       Check whether job was running
       BEFORE state changes to FAULT.
    */

    bool jobWasRunning =
        (
            currentState ==
            CNC_RUNNING
        );

    wasRunning =
        false;

    applyRelayState();

    Serial.println();
    Serial.println(
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );

    Serial.println(
        "MANUAL E-STOP TRIGGERED"
    );

    Serial.println(
        "RELAY ENERGIZED"
    );

    Serial.println(
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );

    if (mqttClient.connected())
    {
        mqttClient.publish(
            TOPIC_ESTOP_EVENT,
            "manual"
        );

        if (jobWasRunning)
        {
            publishJobEnd(
                "stopped",
                "manual_estop"
            );
        }
    }
}

/* ======================================================
   MANUAL E-STOP RESET
   ====================================================== */

void resetManualStop()
{
    if (thermalTripActive)
    {
        Serial.println(
            "RESET BLOCKED:"
        );

        Serial.println(
            "TEMPERATURE TRIP IS ACTIVE"
        );

        return;
    }

    manualStopActive =
        false;

    applyRelayState();

    Serial.println(
        "MANUAL E-STOP RESET"
    );
}

/* ======================================================
   JOB END JSON
   ====================================================== */

void publishJobEnd(
    const char *outcome,
    const char *reason
)
{
    if (!mqttClient.connected())
    {
        return;
    }

    char payload[160];

    snprintf(
        payload,
        sizeof(payload),
        "{\"outcome\":\"%s\",\"reason\":\"%s\"}",
        outcome,
        reason
    );

    mqttClient.publish(
        TOPIC_JOB_END,
        payload
    );
}

/* ======================================================
   MQTT CALLBACK
   ====================================================== */

void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length
)
{
    String message;

    for (
        unsigned int i = 0;
        i < length;
        i++
    )
    {
        message +=
            (char)payload[i];
    }

    message.trim();

    Serial.println();

    Serial.print(
        "MQTT RX ["
    );

    Serial.print(
        topic
    );

    Serial.print(
        "] "
    );

    Serial.println(
        message
    );

    /* ==================================================
       NODE-RED E-STOP COMMAND
       ================================================== */

    if (
        String(topic) ==
        TOPIC_ESTOP_COMMAND
    )
    {
        /* ----------------------------------------------
           TRIGGER
           ---------------------------------------------- */

        if (
            message.equalsIgnoreCase(
                "STOP"
            ) ||
            message.equalsIgnoreCase(
                "ON"
            ) ||
            message.equalsIgnoreCase(
                "TRIGGER"
            ) ||
            message == "1"
        )
        {
            triggerManualStop();
        }

        /* ----------------------------------------------
           RESET
           ---------------------------------------------- */

        else if (
            message.equalsIgnoreCase(
                "RESET"
            ) ||
            message.equalsIgnoreCase(
                "OFF"
            ) ||
            message == "0"
        )
        {
            resetManualStop();
        }
    }
}

/* ======================================================
   WIFI CONNECTION
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

    unsigned long startTime =
        millis();

    while (
        WiFi.status() !=
            WL_CONNECTED &&
        millis() -
            startTime <
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
            "ESP32 IP: "
        );

        Serial.println(
            WiFi.localIP()
        );

        Serial.print(
            "Broker IP: "
        );

        Serial.println(
            MQTT_BROKER_HOST
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
            MQTT_PASSWORD,

            // Last Will topic
            TOPIC_DEVICE_STATUS,

            // QoS
            0,

            // Retained
            true,

            // Last Will message
            "offline"
        );

    if (connected)
    {
        Serial.println(
            "MQTT CONNECTED"
        );

        /* ----------------------------------------------
           ONLINE STATUS
           ---------------------------------------------- */

        mqttClient.publish(
            TOPIC_DEVICE_STATUS,
            "online",
            true
        );

        /* ----------------------------------------------
           SUBSCRIBE TO NODE-RED
           ---------------------------------------------- */

        mqttClient.subscribe(
            TOPIC_ESTOP_COMMAND
        );

        Serial.print(
            "Subscribed: "
        );

        Serial.println(
            TOPIC_ESTOP_COMMAND
        );

        /* ----------------------------------------------
           INITIAL DATA
           ---------------------------------------------- */

        publishState();

        publishTemperatures();
    }

    else
    {
        Serial.print(
            "MQTT CONNECTION FAILED | State = "
        );

        Serial.println(
            mqttClient.state()
        );
    }
}

/* ======================================================
   MQTT TEMPERATURE PUBLISH
   ====================================================== */

void publishTemperatures()
{
    if (!mqttClient.connected())
    {
        return;
    }

    char value[16];

    /* ==================================================
       TEMP X
       ================================================== */

    if (isfinite(tempX))
    {
        snprintf(
            value,
            sizeof(value),
            "%.2f",
            tempX
        );

        mqttClient.publish(
            TOPIC_TEMP_X,
            value
        );
    }

    /* ==================================================
       TEMP Y
       ================================================== */

    if (isfinite(tempY))
    {
        snprintf(
            value,
            sizeof(value),
            "%.2f",
            tempY
        );

        mqttClient.publish(
            TOPIC_TEMP_Y,
            value
        );
    }
}

/* ======================================================
   MQTT STATE PUBLISH
   ====================================================== */

void publishState()
{
    if (!mqttClient.connected())
    {
        return;
    }

    mqttClient.publish(
        TOPIC_STATE,
        stateToString(
            currentState
        ),
        true
    );
}

/* ======================================================
   OLED DISPLAY
   ====================================================== */

void updateOLED()
{
    if (!oledOK)
    {
        return;
    }

    oled.clearBuffer();

    oled.setFont(
        u8g2_font_6x10_tf
    );

    /* ==================================================
       TITLE
       ================================================== */

    oled.setCursor(
        0,
        10
    );

    oled.print(
        "SMART CNC STATION"
    );

    /* ==================================================
       TEMPERATURE X
       ================================================== */

    oled.setCursor(
        0,
        25
    );

    oled.print(
        "X:"
    );

    if (isfinite(tempX))
    {
        oled.print(
            tempX,
            1
        );

        oled.print(
            "C"
        );
    }

    else
    {
        oled.print(
            "ERR"
        );
    }

    /* ==================================================
       TEMPERATURE Y
       ================================================== */

    oled.setCursor(
        65,
        25
    );

    oled.print(
        "Y:"
    );

    if (isfinite(tempY))
    {
        oled.print(
            tempY,
            1
        );

        oled.print(
            "C"
        );
    }

    else
    {
        oled.print(
            "ERR"
        );
    }

    /* ==================================================
       CNC STATE
       ================================================== */

    oled.setCursor(
        0,
        41
    );

    oled.print(
        "CNC: "
    );

    oled.print(
        stateToString(
            currentState
        )
    );

    /* ==================================================
       BOTTOM STATUS
       ================================================== */

    oled.setCursor(
        0,
        58
    );

    if (thermalTripActive)
    {
        oled.print(
            "TEMP HIGH - STOP"
        );
    }

    else if (manualStopActive)
    {
        oled.print(
            "MANUAL E-STOP"
        );
    }

    else
    {
        oled.print(
            relayActive
                ? "RELAY ON "
                : "RELAY OFF "
        );

        oled.print(
            mqttClient.connected()
                ? "MQTT"
                : "NO MQTT"
        );
    }

    oled.sendBuffer();
}

/* ======================================================
   SERIAL STATUS
   ====================================================== */

void printStatus()
{
    Serial.println();

    Serial.println(
        "--------------------------------"
    );

    /* ==================================================
       NETWORK
       ================================================== */

    Serial.print(
        "ESP32 IP: "
    );

    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        Serial.println(
            WiFi.localIP()
        );
    }

    else
    {
        Serial.println(
            "OFFLINE"
        );
    }

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

    Serial.print(
        "MQTT: "
    );

    Serial.println(
        mqttClient.connected()
            ? "CONNECTED"
            : "DISCONNECTED"
    );

    /* ==================================================
       TEMPERATURE
       ================================================== */

    Serial.print(
        "Temp X: "
    );

    if (isfinite(tempX))
    {
        Serial.print(
            tempX,
            2
        );

        Serial.println(
            " C"
        );
    }

    else
    {
        Serial.println(
            "ERROR"
        );
    }

    Serial.print(
        "Temp Y: "
    );

    if (isfinite(tempY))
    {
        Serial.print(
            tempY,
            2
        );

        Serial.println(
            " C"
        );
    }

    else
    {
        Serial.println(
            "ERROR"
        );
    }

    /* ==================================================
       RDC INPUTS
       ================================================== */

    Serial.print(
        "RDC OUT1 GPIO19: "
    );

    Serial.println(
        digitalRead(
            RDC_OUT1_PIN
        )
    );

    Serial.print(
        "RDC OUT2 GPIO18: "
    );

    Serial.println(
        digitalRead(
            RDC_OUT2_PIN
        )
    );

    /* ==================================================
       CNC STATE
       ================================================== */

    Serial.print(
        "CNC State: "
    );

    Serial.println(
        stateToString(
            currentState
        )
    );

    /* ==================================================
       SAFETY
       ================================================== */

    Serial.print(
        "Thermal Trip: "
    );

    Serial.println(
        thermalTripActive
            ? "ACTIVE"
            : "NORMAL"
    );

    Serial.print(
        "Manual E-Stop: "
    );

    Serial.println(
        manualStopActive
            ? "ACTIVE"
            : "OFF"
    );

    Serial.print(
        "Relay: "
    );

    Serial.println(
        relayActive
            ? "ENERGIZED"
            : "OFF"
    );

    Serial.println(
        "--------------------------------"
    );
}

/* ======================================================
   SERIAL COMMANDS
   ====================================================== */

void handleSerialCommands()
{
    if (!Serial.available())
    {
        return;
    }

    char command =
        Serial.read();

    /*
       O = Manual E-stop / Relay ON
    */

    if (
        command == 'O' ||
        command == 'o'
    )
    {
        triggerManualStop();
    }

    /*
       F = Reset / Relay OFF
    */

    else if (
        command == 'F' ||
        command == 'f'
    )
    {
        resetManualStop();
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
        "================================"
    );

    Serial.println(
        "SMART CNC STATION"
    );

    Serial.println(
        "================================"
    );

    /* ==================================================
       RDC INPUTS
       ================================================== */

    pinMode(
        RDC_OUT1_PIN,
        INPUT
    );

    pinMode(
        RDC_OUT2_PIN,
        INPUT
    );

    /* ==================================================
       RELAY
       ================================================== */

    pinMode(
        RELAY_PIN,
        OUTPUT
    );

    setRelay(
        false
    );

    /* ==================================================
       BME280 X
       SDA21 / SCL23
       ================================================== */

    BME_X_BUS.begin(
        SDA_BUS1,
        SCL_BUS1,
        I2C_CLOCK_HZ
    );

    bmeXOK =
        bmeX.begin(
            BME_X_ADDRESS,
            &BME_X_BUS
        );

    Serial.print(
        "BME X [21/23]: "
    );

    Serial.println(
        bmeXOK
            ? "FOUND"
            : "NOT FOUND"
    );

    /* ==================================================
       BME280 Y
       SDA32 / SCL33
       ================================================== */

    BME_Y_BUS.begin(
        SDA_BUS2,
        SCL_BUS2,
        I2C_CLOCK_HZ
    );

    bmeYOK =
        bmeY.begin(
            BME_Y_ADDRESS,
            &BME_Y_BUS
        );

    Serial.print(
        "BME Y [32/33]: "
    );

    Serial.println(
        bmeYOK
            ? "FOUND"
            : "NOT FOUND"
    );

    /* ==================================================
       OLED
       SDA4 / SCL16
       ================================================== */

    oled.setI2CAddress(
        OLED_ADDRESS << 1
    );

    oled.begin();

    oledOK =
        true;

    oled.clearBuffer();

    oled.setFont(
        u8g2_font_6x10_tf
    );

    oled.setCursor(
        0,
        15
    );

    oled.print(
        "SMART CNC STATION"
    );

    oled.setCursor(
        0,
        35
    );

    oled.print(
        "Starting..."
    );

    oled.sendBuffer();

    /* ==================================================
       INITIAL SENSOR READ
       ================================================== */

    readTemperatures();

    checkTemperatureSafety();

    updateCNCState();

    previousState =
        currentState;

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

    mqttClient.setCallback(
        mqttCallback
    );

    mqttClient.setBufferSize(
        512
    );

    connectMQTT();

    /* ==================================================
       OLED INITIAL UPDATE
       ================================================== */

    updateOLED();

    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "SYSTEM READY"
    );

    Serial.println(
        "================================"
    );

    Serial.println();

    Serial.println(
        "Connections:"
    );

    Serial.println(
        "BME X -> SDA21 SCL23"
    );

    Serial.println(
        "BME Y -> SDA32 SCL33"
    );

    Serial.println(
        "OLED  -> SDA4 SCL16"
    );

    Serial.println(
        "OUT1  -> GPIO19"
    );

    Serial.println(
        "OUT2  -> GPIO18"
    );

    Serial.println(
        "Relay -> GPIO27"
    );

    Serial.println();

    Serial.println(
        "ESP32 expected IP: 192.168.1.108"
    );

    Serial.println(
        "MQTT Broker IP: 192.168.1.18"
    );

    Serial.println();

    Serial.println(
        "Temperature trip: 50 C"
    );

    Serial.println(
        "Temperature reset: 48 C"
    );

    Serial.println();

    Serial.println(
        "Serial commands:"
    );

    Serial.println(
        "O = Manual E-Stop"
    );

    Serial.println(
        "F = Reset E-Stop"
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
       MQTT PROCESSING
       ================================================== */

    if (mqttClient.connected())
    {
        mqttClient.loop();
    }

    /* ==================================================
       CNC STATE
       ================================================== */

    updateCNCState();

    handleStateChange();

    /* ==================================================
       TEMPERATURE
       ================================================== */

    if (
        millis() -
            lastSensorRead >=
        SENSOR_INTERVAL_MS
    )
    {
        lastSensorRead =
            millis();

        readTemperatures();

        checkTemperatureSafety();
    }

    /* ==================================================
       MQTT TEMPERATURE PUBLISH
       ================================================== */

    if (
        millis() -
            lastMQTTPublish >=
        MQTT_PUBLISH_INTERVAL_MS
    )
    {
        lastMQTTPublish =
            millis();

        publishTemperatures();
    }

    /* ==================================================
       OLED
       ================================================== */

    if (
        millis() -
            lastOLEDUpdate >=
        OLED_INTERVAL_MS
    )
    {
        lastOLEDUpdate =
            millis();

        updateOLED();
    }

    /* ==================================================
       SERIAL
       ================================================== */

    if (
        millis() -
            lastSerialPrint >=
        SERIAL_INTERVAL_MS
    )
    {
        lastSerialPrint =
            millis();

        printStatus();
    }

    /* ==================================================
       SERIAL TEST COMMANDS
       ================================================== */

    handleSerialCommands();

    delay(
        20
    );
}