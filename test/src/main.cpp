#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>

/* ======================================================
   MPU6050
   ====================================================== */

#define MPU_ADDR 0x68

#define MPU_SDA 21
#define MPU_SCL 23

// ±4 g accelerometer
const float ACCEL_SCALE = 8192.0f;

/* ======================================================
   BME280 BUS
   ====================================================== */

#define BME_SDA 32
#define BME_SCL 33

#define BME_X_ADDR 0x76
#define BME_Y_ADDR 0x77

TwoWire &BME_BUS = Wire1;

Adafruit_BME280 bmeX;
Adafruit_BME280 bmeY;

bool bmeXOK = false;
bool bmeYOK = false;

float tempX = NAN;
float tempY = NAN;

/* ======================================================
   OLED
   ====================================================== */

#define OLED_SDA 25
#define OLED_SCL 26
#define OLED_ADDR 0x3C

U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(
    U8G2_R0,
    OLED_SCL,
    OLED_SDA,
    U8X8_PIN_NONE
);

bool oledOK = false;

/* ======================================================
   RELAY
   ====================================================== */

// Your relay file prints GPIO14 although it defined GPIO1.
// GPIO1 is Serial TX, so use GPIO14 here.
#define RELAY2_PIN 14

#define RELAY_ACTIVE_LEVEL HIGH
#define RELAY_INACTIVE_LEVEL LOW

bool relayActive = false;

/* ======================================================
   VIBRATION SETTINGS
   ====================================================== */

const float VIBRATION_THRESHOLD = 1.0f;

// 200 Hz
#define SAMPLE_INTERVAL_US 5000

// 100 × 5 ms = 0.5 second RMS window
#define RMS_SAMPLES 100

#define CALIBRATION_SAMPLES 1000

// Must exceed threshold for two RMS windows
#define FAULT_WINDOWS_REQUIRED 2

// Approx. 5 Hz high-pass cutoff at 200 Hz
const float HP_ALPHA = 0.864f;

/* ======================================================
   VIBRATION VARIABLES
   ====================================================== */

float baselineX = 0.0f;
float baselineY = 0.0f;
float baselineZ = 0.0f;

float previousX = 0.0f;
float previousY = 0.0f;
float previousZ = 0.0f;

float hpX = 0.0f;
float hpY = 0.0f;
float hpZ = 0.0f;

float rmsSumSquares = 0.0f;

int rmsSampleCount = 0;
int faultCounter = 0;

float vibrationRMS = NAN;

bool vibrationFault = false;

unsigned long lastSampleTime = 0;

/* ======================================================
   OTHER TIMERS
   ====================================================== */

unsigned long lastTemperatureRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSerialStatus = 0;

const unsigned long TEMP_INTERVAL_MS = 1000;
const unsigned long OLED_INTERVAL_MS = 250;
const unsigned long STATUS_INTERVAL_MS = 1000;

/* ======================================================
   FUNCTION DECLARATIONS
   ====================================================== */

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
);

bool readRegister(
    uint8_t reg,
    uint8_t &value
);

bool writeRegister(
    uint8_t reg,
    uint8_t value
);

void calibrateStationary();

void processVibration(float rms);

void abnormalVibrationDetected();

void setRelay(bool active);

void resetVibrationFault();

void readTemperatures();

void updateOLED();

void printStatus();

/* ======================================================
   RELAY CONTROL
   ====================================================== */

void setRelay(bool active)
{
    relayActive = active;

    digitalWrite(
        RELAY2_PIN,
        active
            ? RELAY_ACTIVE_LEVEL
            : RELAY_INACTIVE_LEVEL
    );
}

/* ======================================================
   ABNORMAL VIBRATION ACTION
   ====================================================== */

void abnormalVibrationDetected()
{
    /*
       Fault is latched.

       The relay stays active even if vibration
       later falls below the threshold.
    */

    setRelay(true);

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("ABNORMAL VIBRATION DETECTED");
    Serial.println("RELAY 2 ACTIVATED");
    Serial.println("FAULT LATCHED");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

/* ======================================================
   MANUAL FAULT RESET
   ====================================================== */

void resetVibrationFault()
{
    vibrationFault = false;

    faultCounter = 0;

    rmsSumSquares = 0.0f;
    rmsSampleCount = 0;

    setRelay(false);

    Serial.println();
    Serial.println("==============================");
    Serial.println("VIBRATION FAULT RESET");
    Serial.println("RELAY 2 OFF");
    Serial.println("==============================");
}

/* ======================================================
   TEMPERATURE
   ====================================================== */

void readTemperatures()
{
    if (bmeXOK)
    {
        tempX = bmeX.readTemperature();

        if (!isfinite(tempX))
        {
            tempX = NAN;
        }
    }

    if (bmeYOK)
    {
        tempY = bmeY.readTemperature();

        if (!isfinite(tempY))
        {
            tempY = NAN;
        }
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

    /* Line 1 */

    oled.setCursor(0, 9);
    oled.print("SMART CNC STATION");

    /* Line 2 */

    oled.setCursor(0, 20);

    oled.print("X:");

    if (isfinite(tempX))
    {
        oled.print(tempX, 1);
    }
    else
    {
        oled.print("ERR");
    }

    oled.print(" Y:");

    if (isfinite(tempY))
    {
        oled.print(tempY, 1);
    }
    else
    {
        oled.print("ERR");
    }

    oled.print("C");

    /* Line 3 */

    oled.setCursor(0, 31);

    oled.print("Vib:");

    if (isfinite(vibrationRMS))
    {
        oled.print(vibrationRMS, 3);
    }
    else
    {
        oled.print("---");
    }

    oled.print(" g");

    /* Line 4 */

    oled.setCursor(0, 42);

    oled.print("Limit:");
    oled.print(VIBRATION_THRESHOLD, 2);
    oled.print(" g");

    /* Line 5 */

    oled.setCursor(0, 53);

    oled.print("State:");

    if (vibrationFault)
    {
        oled.print(" FAULT");
    }
    else
    {
        oled.print(" NORMAL");
    }

    /* Line 6 */

    oled.setCursor(0, 63);

    oled.print("Relay:");

    oled.print(
        relayActive
            ? " ON"
            : " OFF"
    );

    oled.sendBuffer();
}

/* ======================================================
   PRINT STATUS
   ====================================================== */

void printStatus()
{
    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("Temperature X: ");

    if (isfinite(tempX))
    {
        Serial.print(tempX, 2);
        Serial.println(" C");
    }
    else
    {
        Serial.println("ERROR");
    }

    Serial.print("Temperature Y: ");

    if (isfinite(tempY))
    {
        Serial.print(tempY, 2);
        Serial.println(" C");
    }
    else
    {
        Serial.println("ERROR");
    }

    Serial.print("Vibration RMS: ");

    if (isfinite(vibrationRMS))
    {
        Serial.print(vibrationRMS, 3);
        Serial.println(" g");
    }
    else
    {
        Serial.println("WAITING");
    }

    Serial.print("Threshold: ");
    Serial.print(VIBRATION_THRESHOLD, 3);
    Serial.println(" g");

    Serial.print("Fault counter: ");
    Serial.print(faultCounter);
    Serial.print("/");
    Serial.println(FAULT_WINDOWS_REQUIRED);

    Serial.print("Vibration fault: ");

    Serial.println(
        vibrationFault
            ? "LATCHED"
            : "CLEAR"
    );

    Serial.print("Relay 2: ");

    Serial.println(
        relayActive
            ? "ON"
            : "OFF"
    );

    Serial.println("--------------------------------");
}

/* ======================================================
   SETUP
   ====================================================== */

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("SMART CNC SENSOR SYSTEM");
    Serial.println("======================================");

    /* ==================================================
       RELAY
       ================================================== */

    pinMode(RELAY2_PIN, OUTPUT);

    setRelay(false);

    Serial.println("Relay 2 initialized OFF");

    /* ==================================================
       OLED
       ================================================== */

    oled.setI2CAddress(
        OLED_ADDR << 1
    );

    oled.begin();

    oledOK = true;

    oled.clearBuffer();

    oled.setFont(
        u8g2_font_6x10_tf
    );

    oled.setCursor(0, 12);

    oled.print(
        "SMART CNC STATION"
    );

    oled.setCursor(0, 28);

    oled.print(
        "Starting..."
    );

    oled.sendBuffer();

    /* ==================================================
       BME280 BUS
       ================================================== */

    BME_BUS.begin(
        BME_SDA,
        BME_SCL,
        100000
    );

    bmeXOK =
        bmeX.begin(
            BME_X_ADDR,
            &BME_BUS
        );

    bmeYOK =
        bmeY.begin(
            BME_Y_ADDR,
            &BME_BUS
        );

    Serial.print("BME280 X: ");

    Serial.println(
        bmeXOK
            ? "FOUND"
            : "NOT FOUND"
    );

    Serial.print("BME280 Y: ");

    Serial.println(
        bmeYOK
            ? "FOUND"
            : "NOT FOUND"
    );

    /* ==================================================
       MPU6050 BUS
       ================================================== */

    Wire.begin(
        MPU_SDA,
        MPU_SCL
    );

    // Keep 100 kHz due to previous I2C errors
    Wire.setClock(100000);

    /* ==================================================
       CHECK MPU
       ================================================== */

    uint8_t whoAmI = 0;

    if (!readRegister(
            0x75,
            whoAmI))
    {
        Serial.println(
            "ERROR: MPU NOT FOUND"
        );

        while (true)
        {
            updateOLED();
            delay(1000);
        }
    }

    Serial.print(
        "WHO_AM_I = 0x"
    );

    Serial.println(
        whoAmI,
        HEX
    );

    /* ==================================================
       WAKE MPU
       ================================================== */

    if (!writeRegister(
            0x6B,
            0x00))
    {
        Serial.println(
            "ERROR: FAILED TO WAKE MPU"
        );

        while (true)
        {
            delay(1000);
        }
    }

    delay(100);

    /* ==================================================
       ACCELEROMETER ±4g
       ================================================== */

    if (!writeRegister(
            0x1C,
            0x08))
    {
        Serial.println(
            "ERROR: ACCEL CONFIG FAILED"
        );

        while (true)
        {
            delay(1000);
        }
    }

    /* ==================================================
       DIGITAL LOW-PASS FILTER
       ================================================== */

    writeRegister(
        0x1A,
        0x03
    );

    delay(500);

    /* ==================================================
       CALIBRATION
       ================================================== */

    Serial.println();
    Serial.println("--------------------------------------");
    Serial.println("STATIONARY CALIBRATION");
    Serial.println("--------------------------------------");

    Serial.println(
        "Keep CNC COMPLETELY STILL."
    );

    Serial.println(
        "Do not touch the MPU."
    );

    Serial.println(
        "Calibration starts in 3 seconds..."
    );

    oled.clearBuffer();

    oled.setFont(
        u8g2_font_6x10_tf
    );

    oled.setCursor(0, 15);
    oled.print("MPU CALIBRATION");

    oled.setCursor(0, 32);
    oled.print("KEEP CNC STILL");

    oled.sendBuffer();

    delay(3000);

    calibrateStationary();

    Serial.println();
    Serial.println("CALIBRATION COMPLETE");

    Serial.print("Baseline X: ");
    Serial.print(baselineX, 4);
    Serial.println(" g");

    Serial.print("Baseline Y: ");
    Serial.print(baselineY, 4);
    Serial.println(" g");

    Serial.print("Baseline Z: ");
    Serial.print(baselineZ, 4);
    Serial.println(" g");

    /* ==================================================
       INITIALIZE HIGH-PASS FILTER
       ================================================== */

    previousX = baselineX;
    previousY = baselineY;
    previousZ = baselineZ;

    hpX = 0.0f;
    hpY = 0.0f;
    hpZ = 0.0f;

    /* ==================================================
       START
       ================================================== */

    readTemperatures();

    updateOLED();

    lastSampleTime = micros();

    Serial.println();
    Serial.println("======================================");
    Serial.println("SYSTEM READY");
    Serial.println("======================================");

    Serial.print("Threshold: ");
    Serial.print(VIBRATION_THRESHOLD, 3);
    Serial.println(" g RMS");

    Serial.println(
        "Fault requires 2 consecutive"
    );

    Serial.println(
        "0.5 second RMS windows."
    );

    Serial.println();

    Serial.println(
        "Send R in Serial Monitor to reset fault."
    );
}

/* ======================================================
   LOOP
   ====================================================== */

void loop()
{
    /* ==================================================
       MPU SAMPLE AT 200 Hz
       ================================================== */

    if (
        (micros() - lastSampleTime) >=
        SAMPLE_INTERVAL_US
    )
    {
        lastSampleTime +=
            SAMPLE_INTERVAL_US;

        int16_t rawAX;
        int16_t rawAY;
        int16_t rawAZ;

        if (readAcceleration(
                rawAX,
                rawAY,
                rawAZ))
        {
            /* ==========================================
               RAW -> g
               ========================================== */

            float ax =
                (float)rawAX /
                ACCEL_SCALE;

            float ay =
                (float)rawAY /
                ACCEL_SCALE;

            float az =
                (float)rawAZ /
                ACCEL_SCALE;

            /* ==========================================
               HIGH-PASS FILTER
               ========================================== */

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

            previousX = ax;
            previousY = ay;
            previousZ = az;

            /* ==========================================
               INSTANTANEOUS VIBRATION
               ========================================== */

            float vibration =
                sqrt(
                    hpX * hpX +
                    hpY * hpY +
                    hpZ * hpZ
                );

            /* ==========================================
               RMS
               ========================================== */

            rmsSumSquares +=
                vibration *
                vibration;

            rmsSampleCount++;

            /* ==========================================
               0.5 SECOND RMS WINDOW
               ========================================== */

            if (
                rmsSampleCount >=
                RMS_SAMPLES
            )
            {
                vibrationRMS =
                    sqrt(
                        rmsSumSquares /
                        RMS_SAMPLES
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
        else
        {
            Serial.println(
                "I2C READ FAILED - SAMPLE IGNORED"
            );
        }
    }

    /* ==================================================
       TEMPERATURE EVERY 1 SECOND
       ================================================== */

    if (
        millis() -
            lastTemperatureRead >=
        TEMP_INTERVAL_MS
    )
    {
        lastTemperatureRead =
            millis();

        readTemperatures();
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
            lastSerialStatus >=
        STATUS_INTERVAL_MS
    )
    {
        lastSerialStatus =
            millis();

        printStatus();
    }

    /* ==================================================
       MANUAL FAULT RESET
       ================================================== */

    if (Serial.available())
    {
        char command =
            Serial.read();

        if (
            command == 'R' ||
            command == 'r'
        )
        {
            resetVibrationFault();
        }
    }
}

/* ======================================================
   PROCESS VIBRATION
   ====================================================== */

void processVibration(float rms)
{
    Serial.print(
        "Vibration RMS: "
    );

    Serial.print(
        rms,
        3
    );

    Serial.print(" g");

    Serial.print(
        " | Threshold: "
    );

    Serial.print(
        VIBRATION_THRESHOLD,
        3
    );

    Serial.print(" g");

    /* ==================================================
       FAULT ALREADY LATCHED
       ================================================== */

    if (vibrationFault)
    {
        Serial.println(
            " | FAULT LATCHED"
        );

        return;
    }

    /* ==================================================
       ABOVE THRESHOLD
       ================================================== */

    if (
        rms >
        VIBRATION_THRESHOLD
    )
    {
        faultCounter++;

        Serial.print(
            " | HIGH | Count: "
        );

        Serial.print(
            faultCounter
        );

        Serial.print("/");

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
                " | >>> VIBRATION FAULT"
            );

            abnormalVibrationDetected();
        }
    }

    /* ==================================================
       NORMAL
       ================================================== */

    else
    {
        faultCounter = 0;

        Serial.print(
            " | NORMAL"
        );
    }

    Serial.println();
}

/* ======================================================
   STATIONARY CALIBRATION
   ====================================================== */

void calibrateStationary()
{
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    int validSamples = 0;
    int failedSamples = 0;

    while (
        validSamples <
        CALIBRATION_SAMPLES
    )
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        if (readAcceleration(
                ax,
                ay,
                az))
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
                    "Calibration I2C failures: "
                );

                Serial.println(
                    failedSamples
                );
            }
        }

        delay(3);
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
}

/* ======================================================
   READ ACCELEROMETER
   ====================================================== */

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
)
{
    Wire.beginTransmission(
        MPU_ADDR
    );

    Wire.write(
        0x3B
    );

    if (
        Wire.endTransmission(
            false
        ) != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)6,
            (uint8_t)true
        );

    if (
        received != 6
    )
    {
        while (
            Wire.available()
        )
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
   READ MPU REGISTER
   ====================================================== */

bool readRegister(
    uint8_t reg,
    uint8_t &value
)
{
    Wire.beginTransmission(
        MPU_ADDR
    );

    Wire.write(reg);

    if (
        Wire.endTransmission(
            false
        ) != 0
    )
    {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)1,
            (uint8_t)true
        );

    if (
        received != 1
    )
    {
        while (
            Wire.available()
        )
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
        MPU_ADDR
    );

    Wire.write(reg);
    Wire.write(value);

    return
        Wire.endTransmission()
        == 0;
}