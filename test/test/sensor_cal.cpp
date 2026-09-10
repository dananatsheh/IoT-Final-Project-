#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================================================
// MPU CONFIGURATION
// =====================================================

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 23

// We use +/-4g because your previous normal-motion
// readings were approaching the +/-2g limit.
#define ACCEL_SCALE 8192.0f

// =====================================================
// TIMING
// =====================================================

// 200 Hz sampling
#define SAMPLE_INTERVAL_US 5000

// RMS window = 0.5 seconds
#define RMS_SAMPLES 100

// Stationary calibration
#define CALIBRATION_SAMPLES 1000

// Normal CNC learning period
#define LEARNING_TIME_MS 10000

// Maximum number of RMS values during learning
#define MAX_LEARNING_WINDOWS 30

// =====================================================
// FILTER
// =====================================================

// High-pass cutoff approximately 5 Hz at 200 Hz sampling.
//
// Slow CNC acceleration/movement is reduced.
// Faster mechanical vibration remains.
const float HP_ALPHA = 0.864f;

// =====================================================
// FAULT SETTINGS
// =====================================================

// Threshold multiplier above learned normal vibration
const float THRESHOLD_MARGIN = 1.50f;

// Minimum threshold
const float MIN_THRESHOLD = 0.08f;

// Require 2 consecutive bad RMS windows
// 2 x 0.5 sec = approximately 1 second
#define FAULT_WINDOWS_REQUIRED 2

// =====================================================
// VARIABLES
// =====================================================

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

float learningValues[MAX_LEARNING_WINDOWS];
int learningCount = 0;

float vibrationThreshold = 0.0f;

int faultCounter = 0;

bool learningMode = true;
bool vibrationFault = false;

unsigned long learningStart = 0;
unsigned long lastSampleTime = 0;


// =====================================================
// FUNCTIONS
// =====================================================

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
);

bool writeRegister(
    uint8_t reg,
    uint8_t value
);

bool readRegister(
    uint8_t reg,
    uint8_t &value
);

void calibrateStationary();

void processRMS(float rms);

void calculateNormalThreshold();


// =====================================================
// SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(1000);

    Serial.println();
    Serial.println("====================================");
    Serial.println("SMART CNC VIBRATION DETECTOR");
    Serial.println("====================================");


    // -------------------------------------------------
    // Check sensor
    // -------------------------------------------------

    uint8_t whoAmI = 0;

    if (!readRegister(0x75, whoAmI)) {

        Serial.println("ERROR: MPU not detected.");

        while (true) {
            delay(1000);
        }
    }

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    // Your sensor reported 0x70.
    // 0x70 and 0x71 are accepted here.
    if (whoAmI == 0x70 || whoAmI == 0x71) {

        Serial.println("MPU detected.");

    } else {

        Serial.println("WARNING: Unexpected MPU ID.");
    }


    // -------------------------------------------------
    // Wake sensor
    // -------------------------------------------------

    writeRegister(0x6B, 0x00);

    delay(100);


    // -------------------------------------------------
    // Accelerometer +/-4g
    //
    // ACCEL_CONFIG register 0x1C
    // AFS_SEL = 1
    // -------------------------------------------------

    writeRegister(0x1C, 0x08);


    // -------------------------------------------------
    // Digital low-pass filter
    // -------------------------------------------------

    writeRegister(0x1A, 0x03);

    delay(500);


    // =================================================
    // STATIONARY CALIBRATION
    // =================================================

    Serial.println();
    Serial.println("------------------------------------");
    Serial.println("STEP 1 - STATIONARY CALIBRATION");
    Serial.println("------------------------------------");

    Serial.println();
    Serial.println("Keep CNC completely STILL.");
    Serial.println("Do not touch the sensor.");
    Serial.println("Starting in 3 seconds...");

    delay(3000);

    calibrateStationary();


    Serial.println();
    Serial.println("Stationary calibration complete.");

    Serial.print("Baseline X = ");
    Serial.println(baselineX, 4);

    Serial.print("Baseline Y = ");
    Serial.println(baselineY, 4);

    Serial.print("Baseline Z = ");
    Serial.println(baselineZ, 4);


    // Initialize filter
    previousX = baselineX;
    previousY = baselineY;
    previousZ = baselineZ;


    // =================================================
    // NORMAL OPERATION LEARNING
    // =================================================

    Serial.println();
    Serial.println("====================================");
    Serial.println("STEP 2 - NORMAL CNC LEARNING");
    Serial.println("====================================");

    Serial.println();
    Serial.println("NOW MOVE THE CNC NORMALLY.");
    Serial.println("Use normal X and Y movements.");
    Serial.println("Do NOT intentionally shake it.");
    Serial.println();
    Serial.println("Learning normal vibration for 10 sec...");

    learningStart = millis();

    lastSampleTime = micros();
}


// =====================================================
// LOOP
// =====================================================

void loop() {

    // Keep approximately 200 Hz sampling rate
    if (
        micros() - lastSampleTime <
        SAMPLE_INTERVAL_US
    ) {
        return;
    }

    lastSampleTime += SAMPLE_INTERVAL_US;


    int16_t rawAX;
    int16_t rawAY;
    int16_t rawAZ;


    // =================================================
    // READ MPU
    // =================================================

    if (!readAcceleration(
            rawAX,
            rawAY,
            rawAZ)) {

        Serial.println("I2C READ FAILED - ignored");

        return;
    }


    // =================================================
    // CONVERT TO g
    // =================================================

    float ax =
        (float)rawAX / ACCEL_SCALE;

    float ay =
        (float)rawAY / ACCEL_SCALE;

    float az =
        (float)rawAZ / ACCEL_SCALE;


    // =================================================
    // HIGH-PASS FILTER
    //
    // Removes gravity + slower machine movement.
    // Keeps faster mechanical vibration.
    // =================================================

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


    // =================================================
    // INSTANTANEOUS VIBRATION
    // =================================================

    float instantaneousVibration =
        sqrt(
            hpX * hpX +
            hpY * hpY +
            hpZ * hpZ
        );


    // =================================================
    // RMS CALCULATION
    // =================================================

    rmsSumSquares +=
        instantaneousVibration *
        instantaneousVibration;

    rmsSampleCount++;


    // Every 100 samples = ~0.5 second
    if (rmsSampleCount >= RMS_SAMPLES) {

        float vibrationRMS =
            sqrt(
                rmsSumSquares /
                RMS_SAMPLES
            );


        rmsSumSquares = 0.0f;
        rmsSampleCount = 0;


        processRMS(vibrationRMS);
    }
}


// =====================================================
// PROCESS RMS
// =====================================================

void processRMS(float rms) {

    // =================================================
    // LEARNING MODE
    // =================================================

    if (learningMode) {

        Serial.print("LEARNING | Vibration RMS: ");
        Serial.print(rms, 3);
        Serial.println(" g");


        if (
            learningCount <
            MAX_LEARNING_WINDOWS
        ) {

            learningValues[learningCount] = rms;
            learningCount++;
        }


        if (
            millis() - learningStart >=
            LEARNING_TIME_MS
        ) {

            learningMode = false;

            calculateNormalThreshold();


            Serial.println();
            Serial.println("====================================");
            Serial.println("NORMAL VIBRATION LEARNED");
            Serial.println("====================================");

            Serial.print("Final fault threshold = ");
            Serial.print(vibrationThreshold, 3);
            Serial.println(" g RMS");

            Serial.println();
            Serial.println("MONITORING STARTED");
            Serial.println();

            faultCounter = 0;
        }

        return;
    }


    // =================================================
    // MONITORING MODE
    // =================================================

    Serial.print("Vibration RMS: ");
    Serial.print(rms, 3);
    Serial.print(" g");

    Serial.print(" | Threshold: ");
    Serial.print(vibrationThreshold, 3);
    Serial.print(" g");


    // =================================================
    // ABNORMAL VIBRATION CHECK
    // =================================================

    if (rms > vibrationThreshold) {

        faultCounter++;

        Serial.print(" | HIGH");

        if (
            faultCounter >=
            FAULT_WINDOWS_REQUIRED
        ) {

            vibrationFault = true;

            Serial.print(
                " | >>> ABNORMAL VIBRATION FAULT"
            );


            // ==========================================
            // FINAL PROJECT ACTION GOES HERE
            // ==========================================

            /*
                Example later:

                triggerEmergencyStop();

                jobState = "faulted";

                faultReason = "abnormal_vibration";

                MQTT publish fault;

                OLED show fault;

                Firebase log fault;
            */
        }

    } else {

        faultCounter = 0;
        vibrationFault = false;

        Serial.print(" | NORMAL");
    }


    Serial.println();
}


// =====================================================
// AUTOMATIC NORMAL THRESHOLD
// =====================================================

void calculateNormalThreshold() {

    if (learningCount == 0) {

        vibrationThreshold =
            MIN_THRESHOLD;

        return;
    }


    // -------------------------------------------------
    // Sort learning values
    // -------------------------------------------------

    for (int i = 0; i < learningCount - 1; i++) {

        for (
            int j = i + 1;
            j < learningCount;
            j++
        ) {

            if (
                learningValues[j] <
                learningValues[i]
            ) {

                float temp =
                    learningValues[i];

                learningValues[i] =
                    learningValues[j];

                learningValues[j] =
                    temp;
            }
        }
    }


    // -------------------------------------------------
    // Use approximately 95th percentile
    // -------------------------------------------------

    int index =
        (int)(
            0.95f *
            (learningCount - 1)
        );


    float normalHigh =
        learningValues[index];


    vibrationThreshold =
        normalHigh *
        THRESHOLD_MARGIN;


    // Never make threshold too sensitive
    if (
        vibrationThreshold <
        MIN_THRESHOLD
    ) {

        vibrationThreshold =
            MIN_THRESHOLD;
    }


    Serial.println();

    Serial.print("Normal high RMS = ");
    Serial.print(normalHigh, 3);
    Serial.println(" g");
}


// =====================================================
// STATIONARY CALIBRATION
// =====================================================

void calibrateStationary() {

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    int valid = 0;


    while (
        valid <
        CALIBRATION_SAMPLES
    ) {

        int16_t ax;
        int16_t ay;
        int16_t az;


        if (
            readAcceleration(
                ax,
                ay,
                az
            )
        ) {

            sumX +=
                (float)ax /
                ACCEL_SCALE;

            sumY +=
                (float)ay /
                ACCEL_SCALE;

            sumZ +=
                (float)az /
                ACCEL_SCALE;

            valid++;


            if (valid % 200 == 0) {

                Serial.print("Calibration: ");
                Serial.print(valid);
                Serial.print(" / ");
                Serial.println(
                    CALIBRATION_SAMPLES
                );
            }
        }

        delay(3);
    }


    baselineX =
        sumX /
        CALIBRATION_SAMPLES;

    baselineY =
        sumY /
        CALIBRATION_SAMPLES;

    baselineZ =
        sumZ /
        CALIBRATION_SAMPLES;
}


// =====================================================
// READ ACCELEROMETER
// =====================================================

bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
) {

    Wire.beginTransmission(
        MPU_ADDR
    );

    Wire.write(0x3B);


    if (
        Wire.endTransmission(false)
        != 0
    ) {

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


// =====================================================
// READ REGISTER
// =====================================================

bool readRegister(
    uint8_t reg,
    uint8_t &value
) {

    Wire.beginTransmission(
        MPU_ADDR
    );

    Wire.write(reg);


    if (
        Wire.endTransmission(false)
        != 0
    ) {

        return false;
    }


    if (
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)1,
            (uint8_t)true
        ) != 1
    ) {

        return false;
    }


    value = Wire.read();

    return true;
}


// =====================================================
// WRITE REGISTER
// =====================================================

bool writeRegister(
    uint8_t reg,
    uint8_t value
) {

    Wire.beginTransmission(
        MPU_ADDR
    );

    Wire.write(reg);
    Wire.write(value);

    return (
        Wire.endTransmission() == 0
    );
}