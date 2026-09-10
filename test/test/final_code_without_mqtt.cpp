#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>

/* ======================================================
   PIN CONFIGURATION
   ====================================================== */

// BME280 X
#define SDA_BUS1 21
#define SCL_BUS1 23

// BME280 Y
#define SDA_BUS2 32
#define SCL_BUS2 33

// OLED
#define OLED_SDA 4
#define OLED_SCL 16

// ES32C14 Relay 1
#define RELAY_PIN 12

// RDC6445S signals through ES32C14 inputs
#define RDC_OUT1_PIN 19
#define RDC_OUT2_PIN 18

/* ======================================================
   I2C ADDRESSES
   ====================================================== */

#define BME_X_ADDRESS 0x76
#define BME_Y_ADDRESS 0x76

#define OLED_ADDRESS 0x3C

#define I2C_CLOCK_HZ 100000

/* ======================================================
   TEMPERATURE SAFETY
   ====================================================== */

#define TEMP_HIGH_THRESHOLD_C 50.0f
#define TEMP_RESET_THRESHOLD_C 48.0f

bool thermalTripActive = false;

/* ======================================================
   RELAY CONFIGURATION
   ====================================================== */

#define RELAY_ACTIVE_LEVEL HIGH
#define RELAY_INACTIVE_LEVEL LOW

bool relayActive = false;

// Manual relay command
bool manualRelayRequested = false;

/* ======================================================
   I2C BUSES
   ====================================================== */

// BME X -> hardware I2C bus 0
TwoWire BME_X_BUS = TwoWire(0);

// BME Y -> hardware I2C bus 1
TwoWire BME_Y_BUS = TwoWire(1);

/* ======================================================
   BME280
   ====================================================== */

Adafruit_BME280 bmeX;
Adafruit_BME280 bmeY;

bool bmeXOK = false;
bool bmeYOK = false;

float tempX = NAN;
float tempY = NAN;

/* ======================================================
   OLED
   ====================================================== */

U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(
    U8G2_R0,
    OLED_SCL,
    OLED_SDA,
    U8X8_PIN_NONE
);

bool oledOK = false;

/* ======================================================
   CNC STATES
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

/*
   RDC6445S SIGNAL LOGIC

   OUT1:
   LOW  = Idle / Standby
   HIGH = Not Idle / Running

   OUT2:
   LOW  = Fault
   HIGH = No Fault
*/

bool wasRunning = false;

/*
   After:
   RUNNING -> IDLE

   display DONE for 2 seconds.
*/

unsigned long doneStartTime = 0;

const unsigned long DONE_DISPLAY_TIME = 2000;

/* ======================================================
   TIMERS
   ====================================================== */

unsigned long lastSensorRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSerialPrint = 0;

const unsigned long SENSOR_INTERVAL_MS = 1000;
const unsigned long OLED_INTERVAL_MS = 250;
const unsigned long SERIAL_INTERVAL_MS = 1000;

/* ======================================================
   FUNCTION DECLARATIONS
   ====================================================== */

void setRelay(bool active);
void applyRelayState();

void readTemperatures();
void checkTemperatureSafety();

void updateCNCState();

const char *stateToString(CNCState state);

void printStateIfChanged();

void updateOLED();

void printStatus();

void handleSerialCommands();

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
   APPLY RELAY STATE
   ====================================================== */

void applyRelayState()
{
    /*
       Thermal protection has priority.

       Relay will be energized if:

       1. Temperature trip is active
          OR
       2. Manual relay ON was requested
    */

    bool shouldActivateRelay =
        thermalTripActive ||
        manualRelayRequested;

    setRelay(
        shouldActivateRelay
    );
}

/* ======================================================
   READ TEMPERATURES
   ====================================================== */

void readTemperatures()
{
    /* -------------------------
       BME X
       ------------------------- */

    if (bmeXOK)
    {
        tempX =
            bmeX.readTemperature();

        if (!isfinite(tempX))
        {
            tempX = NAN;
        }
    }

    /* -------------------------
       BME Y
       ------------------------- */

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
    bool xHigh = false;
    bool yHigh = false;

    /* ==================================================
       CHECK 50 C LIMIT
       ================================================== */

    if (
        isfinite(tempX) &&
        tempX >= TEMP_HIGH_THRESHOLD_C
    )
    {
        xHigh = true;
    }

    if (
        isfinite(tempY) &&
        tempY >= TEMP_HIGH_THRESHOLD_C
    )
    {
        yHigh = true;
    }

    /* ==================================================
       ACTIVATE TRIP
       ================================================== */

    if (
        !thermalTripActive &&
        (xHigh || yHigh)
    )
    {
        thermalTripActive = true;

        Serial.println();
        Serial.println(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );

        Serial.println(
            "HIGH TEMPERATURE DETECTED"
        );

        if (xHigh)
        {
            Serial.print(
                "BME X = "
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
                "BME Y = "
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
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );
    }

    /* ==================================================
       RESET BELOW 48 C
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

            Serial.println();
            Serial.println(
                "TEMPERATURE SAFE AGAIN"
            );

            Serial.println(
                "Thermal trip cleared."
            );
        }
    }

    applyRelayState();
}

/* ======================================================
   CNC STATE NAME
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
   READ RDC6445S CNC STATE
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

    /* ==================================================
       SIGNAL INTERPRETATION
       ================================================== */

    bool idleSignal =
        (out1 == LOW);

    bool faultSignal =
        (out2 == LOW);

    /* ==================================================
       FAULT
       Highest priority
       ================================================== */

    if (faultSignal)
    {
        currentState =
            CNC_FAULT;

        /*
           Prevent FAULT -> IDLE from
           being interpreted as DONE.
        */

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
            DONE_DISPLAY_TIME
        )
        {
            return;
        }

        currentState =
            CNC_IDLE;
    }

    /* ==================================================
       IDLE
       ================================================== */

    if (idleSignal)
    {
        /*
           Machine was running,
           and has now returned to idle.
        */

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
   PRINT CNC STATE CHANGE
   ====================================================== */

void printStateIfChanged()
{
    if (
        currentState !=
        previousState
    )
    {
        Serial.println();

        Serial.println(
            "=========================="
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
            "=========================="
        );

        previousState =
            currentState;
    }
}

/* ======================================================
   OLED
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

    /* -------------------------
       TITLE
       ------------------------- */

    oled.setCursor(
        0,
        10
    );

    oled.print(
        "SMART CNC STATION"
    );

    /* -------------------------
       TEMP X
       ------------------------- */

    oled.setCursor(
        0,
        24
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

    /* -------------------------
       TEMP Y
       ------------------------- */

    oled.setCursor(
        64,
        24
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

    /* -------------------------
       CNC STATE
       ------------------------- */

    oled.setCursor(
        0,
        40
    );

    oled.print(
        "CNC: "
    );

    oled.print(
        stateToString(
            currentState
        )
    );

    /* -------------------------
       SAFETY / RELAY
       ------------------------- */

    oled.setCursor(
        0,
        57
    );

    if (thermalTripActive)
    {
        oled.print(
            "TEMP HIGH RELAY ON"
        );
    }

    else
    {
        oled.print(
            "Relay:"
        );

        oled.print(
            relayActive
                ? "ON "
                : "OFF"
        );

        oled.print(
            " Temp OK"
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
        "-----------------------------"
    );

    /* ==================================================
       TEMPERATURE X
       ================================================== */

    Serial.print(
        "BME X Temperature: "
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

    /* ==================================================
       TEMPERATURE Y
       ================================================== */

    Serial.print(
        "BME Y Temperature: "
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
       TEMPERATURE STATUS
       ================================================== */

    Serial.print(
        "Temperature Status: "
    );

    Serial.println(
        thermalTripActive
            ? "HIGH / TRIPPED"
            : "NORMAL"
    );

    /* ==================================================
       CNC
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
       RAW RDC INPUTS
       ================================================== */

    Serial.print(
        "RDC OUT1 / GPIO19: "
    );

    Serial.println(
        digitalRead(
            RDC_OUT1_PIN
        )
    );

    Serial.print(
        "RDC OUT2 / GPIO18: "
    );

    Serial.println(
        digitalRead(
            RDC_OUT2_PIN
        )
    );

    /* ==================================================
       RELAY
       ================================================== */

    Serial.print(
        "Relay: "
    );

    Serial.println(
        relayActive
            ? "ON"
            : "OFF"
    );

    Serial.println(
        "-----------------------------"
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

    /* ==================================================
       O = RELAY ON
       ================================================== */

    if (
        command == 'O' ||
        command == 'o'
    )
    {
        manualRelayRequested =
            true;

        applyRelayState();

        Serial.println(
            "MANUAL RELAY ON"
        );
    }

    /* ==================================================
       F = RELAY OFF
       ================================================== */

    else if (
        command == 'F' ||
        command == 'f'
    )
    {
        manualRelayRequested =
            false;

        applyRelayState();

        if (thermalTripActive)
        {
            Serial.println(
                "Cannot turn relay OFF:"
            );

            Serial.println(
                "TEMPERATURE TRIP ACTIVE"
            );
        }

        else
        {
            Serial.println(
                "MANUAL RELAY OFF"
            );
        }
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
        "BME + OLED + RELAY + RDC"
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

    Serial.println(
        "RDC OUT1 -> GPIO19"
    );

    Serial.println(
        "RDC OUT2 -> GPIO18"
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

    Serial.println(
        "Relay initialized GPIO27"
    );

    /* ==================================================
       BME X
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
       BME Y
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

    delay(
        1000
    );

    /* ==================================================
       INITIAL VALUES
       ================================================== */

    readTemperatures();

    checkTemperatureSafety();

    updateCNCState();

    previousState =
        currentState;

    updateOLED();

    /* ==================================================
       READY
       ================================================== */

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
        "BME X -> SDA21 / SCL23"
    );

    Serial.println(
        "BME Y -> SDA32 / SCL33"
    );

    Serial.println(
        "OLED  -> SDA4 / SCL16"
    );

    Serial.println(
        "Relay -> GPIO27"
    );

    Serial.println(
        "OUT1  -> GPIO19"
    );

    Serial.println(
        "OUT2  -> GPIO18"
    );

    Serial.println();

    Serial.println(
        "Temperature trip = 50 C"
    );

    Serial.println(
        "Temperature reset = 48 C"
    );

    Serial.println();

    Serial.println(
        "O = Manual Relay ON"
    );

    Serial.println(
        "F = Manual Relay OFF"
    );
}

/* ======================================================
   LOOP
   ====================================================== */

void loop()
{
    /* ==================================================
       CNC STATE
       ================================================== */

    updateCNCState();

    printStateIfChanged();

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
       SERIAL STATUS
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
       MANUAL RELAY
       ================================================== */

    handleSerialCommands();

    delay(
        20
    );
}