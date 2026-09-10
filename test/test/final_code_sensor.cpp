#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ======================================================
// MPU CONFIGURATION
// ======================================================

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 23

// Accelerometer set to ±4g
const float ACCEL_SCALE = 8192.0f;

// ======================================================
// FIXED THRESHOLD FROM YOUR CNC TEST
// ======================================================

const float VIBRATION_THRESHOLD = 1.0f;

// ======================================================
// SAMPLING
// ======================================================

// 200 Hz
#define SAMPLE_INTERVAL_US 5000

// 100 samples × 5 ms = 0.5 second RMS window
#define RMS_SAMPLES 100

// Stationary calibration
#define CALIBRATION_SAMPLES 1000

// Must exceed threshold for two consecutive RMS windows
// 2 × 0.5 sec = about 1 second
#define FAULT_WINDOWS_REQUIRED 2

// ======================================================
// HIGH-PASS FILTER
// ======================================================

// Approximately 5 Hz cutoff at 200 Hz sampling
const float HP_ALPHA = 0.864f;

// ======================================================
// VARIABLES
// ======================================================

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

bool vibrationFault = false;

unsigned long lastSampleTime = 0;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

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

// ======================================================
// SETUP
// ======================================================

void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    // Keep 100 kHz because your wiring has shown
    // occasional I2C communication errors
    Wire.setClock(100000);

    delay(1000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("SMART CNC VIBRATION MONITOR");
    Serial.println("======================================");

    // ==================================================
    // CHECK MPU
    // ==================================================

    uint8_t whoAmI = 0;

    if (!readRegister(0x75, whoAmI)) {

        Serial.println("ERROR: MPU NOT FOUND");

        while (true) {
            delay(1000);
        }
    }

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI == 0x70 || whoAmI == 0x71) {

        Serial.println("MPU detected.");

    } else {

        Serial.println("WARNING: Unexpected MPU device ID.");
    }

    // ==================================================
    // WAKE MPU
    // ==================================================

    if (!writeRegister(0x6B, 0x00)) {

        Serial.println("ERROR: Failed to wake MPU.");

        while (true) {
            delay(1000);
        }
    }

    delay(100);

    // ==================================================
    // ACCELEROMETER = ±4g
    // ==================================================

    if (!writeRegister(0x1C, 0x08)) {

        Serial.println("ERROR: Failed to configure accelerometer.");

        while (true) {
            delay(1000);
        }
    }

    // ==================================================
    // DIGITAL LOW-PASS FILTER
    // ==================================================

    writeRegister(0x1A, 0x03);

    delay(500);

    // ==================================================
    // STATIONARY CALIBRATION
    // ==================================================

    Serial.println();
    Serial.println("--------------------------------------");
    Serial.println("STATIONARY CALIBRATION");
    Serial.println("--------------------------------------");

    Serial.println();
    Serial.println("Keep CNC COMPLETELY STILL.");
    Serial.println("Do not touch the MPU.");
    Serial.println("Calibration starts in 3 seconds...");

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

    // Initialize filter from stationary position
    previousX = baselineX;
    previousY = baselineY;
    previousZ = baselineZ;

    hpX = 0.0f;
    hpY = 0.0f;
    hpZ = 0.0f;

    Serial.println();
    Serial.println("======================================");
    Serial.println("VIBRATION MONITORING STARTED");
    Serial.println("======================================");

    Serial.print("Fault threshold: ");
    Serial.print(VIBRATION_THRESHOLD, 3);
    Serial.println(" g RMS");

    Serial.println("Fault requires threshold exceeded");
    Serial.println("for approximately 1 second.");
    Serial.println();

    lastSampleTime = micros();
}

// ======================================================
// LOOP
// ======================================================

void loop() {

    // ==================================================
    // 200 Hz SAMPLE RATE
    // ==================================================

    if ((micros() - lastSampleTime) < SAMPLE_INTERVAL_US) {
        return;
    }

    lastSampleTime += SAMPLE_INTERVAL_US;

    int16_t rawAX;
    int16_t rawAY;
    int16_t rawAZ;

    // ==================================================
    // READ MPU
    // ==================================================

    if (!readAcceleration(
            rawAX,
            rawAY,
            rawAZ)) {

        Serial.println("I2C READ FAILED - SAMPLE IGNORED");

        return;
    }

    // ==================================================
    // RAW -> g
    // ==================================================

    float ax = (float)rawAX / ACCEL_SCALE;
    float ay = (float)rawAY / ACCEL_SCALE;
    float az = (float)rawAZ / ACCEL_SCALE;

    // ==================================================
    // HIGH-PASS FILTER
    // ==================================================

    hpX =
        HP_ALPHA *
        (hpX + ax - previousX);

    hpY =
        HP_ALPHA *
        (hpY + ay - previousY);

    hpZ =
        HP_ALPHA *
        (hpZ + az - previousZ);

    previousX = ax;
    previousY = ay;
    previousZ = az;

    // ==================================================
    // INSTANTANEOUS VIBRATION
    // ==================================================

    float vibration =
        sqrt(
            hpX * hpX +
            hpY * hpY +
            hpZ * hpZ
        );

    // ==================================================
    // RMS ACCUMULATION
    // ==================================================

    rmsSumSquares += vibration * vibration;

    rmsSampleCount++;

    // ==================================================
    // 0.5 SECOND RMS WINDOW
    // ==================================================

    if (rmsSampleCount >= RMS_SAMPLES) {

        float vibrationRMS =
            sqrt(
                rmsSumSquares /
                RMS_SAMPLES
            );

        rmsSumSquares = 0.0f;
        rmsSampleCount = 0;

        processVibration(vibrationRMS);
    }
}

// ======================================================
// PROCESS VIBRATION
// ======================================================

void processVibration(float rms) {

    Serial.print("Vibration RMS: ");
    Serial.print(rms, 3);
    Serial.print(" g");

    Serial.print(" | Threshold: ");
    Serial.print(VIBRATION_THRESHOLD, 3);
    Serial.print(" g");

    // ==================================================
    // ABOVE THRESHOLD
    // ==================================================

    if (rms > VIBRATION_THRESHOLD) {

        faultCounter++;

        Serial.print(" | HIGH");

        Serial.print(" | Count: ");
        Serial.print(faultCounter);
        Serial.print("/");
        Serial.print(FAULT_WINDOWS_REQUIRED);

        // ==================================================
        // CONFIRMED ABNORMAL VIBRATION
        // ==================================================

        if (faultCounter >= FAULT_WINDOWS_REQUIRED) {

            if (!vibrationFault) {

                vibrationFault = true;

                Serial.print(
                    " | >>> ABNORMAL VIBRATION FAULT"
                );

                abnormalVibrationDetected();
            }
        }

    } else {

        // ==================================================
        // NORMAL
        // ==================================================

        faultCounter = 0;

        if (!vibrationFault) {

            Serial.print(" | NORMAL");

        } else {

            Serial.print(" | FAULT LATCHED");
        }
    }

    Serial.println();
}

// ======================================================
// ABNORMAL VIBRATION ACTION
// ======================================================

void abnormalVibrationDetected() {

    /*
       ==================================================
       FINAL CNC EMERGENCY ACTION
       ==================================================

       This function is called ONCE when:

       RMS > 0.617 g
       for two consecutive 0.5-second windows.

       Later connect your real stop logic here.

       Example:

       digitalWrite(RELAY_PIN, ...);

       MQTT:
       vibration_status = "fault"

       Firebase:
       outcome = "stopped"
       reason  = "abnormal_vibration"

       OLED:
       "VIBRATION FAULT"
    */

}

// ======================================================
// STATIONARY CALIBRATION
// ======================================================

void calibrateStationary() {

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    int validSamples = 0;
    int failedSamples = 0;

    while (validSamples < CALIBRATION_SAMPLES) {

        int16_t ax;
        int16_t ay;
        int16_t az;

        if (readAcceleration(
                ax,
                ay,
                az)) {

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

            if (validSamples % 200 == 0) {

                Serial.print("Calibration: ");
                Serial.print(validSamples);
                Serial.print(" / ");
                Serial.println(CALIBRATION_SAMPLES);
            }

        } else {

            failedSamples++;

            if (failedSamples % 10 == 0) {

                Serial.print("Calibration I2C failures: ");
                Serial.println(failedSamples);
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

// ======================================================
// READ ACCELEROMETER
// ======================================================

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
) {

    Wire.beginTransmission(MPU_ADDR);

    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)6,
            (uint8_t)true
        );

    if (received != 6) {

        while (Wire.available()) {
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

// ======================================================
// READ REGISTER
// ======================================================

bool readRegister(
    uint8_t reg,
    uint8_t &value
) {

    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)1,
            (uint8_t)true
        );

    if (received != 1) {

        while (Wire.available()) {
            Wire.read();
        }

        return false;
    }

    value = Wire.read();

    return true;
}

// ======================================================
// WRITE REGISTER
// ======================================================

bool writeRegister(
    uint8_t reg,
    uint8_t value
) {

    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}