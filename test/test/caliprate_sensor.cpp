#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ======================================================
// MPU9250 CONFIGURATION
// ======================================================

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 23  // Your successful I2C scanner used GPIO22

#define CALIBRATION_SAMPLES 2000
#define MAX_CALIBRATION_FAILURES 100

// MPU9250 configured for:
// Accelerometer = +/-2g
// Gyroscope     = +/-250 deg/s
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;


// ======================================================
// CALIBRATION VALUES
// ======================================================

// Accelerometer resting baseline
float axBaseline = 0.0;
float ayBaseline = 0.0;
float azBaseline = 0.0;

// Gyroscope resting offsets
float gxOffset = 0.0;
float gyOffset = 0.0;
float gzOffset = 0.0;


// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

bool readMPU9250(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &temp,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
);

bool readRegister(uint8_t reg, uint8_t &value);

bool writeRegister(uint8_t reg, uint8_t value);

bool calibrateMPU9250();


// ======================================================
// SETUP
// ======================================================

void setup() {

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("ESP32 + MPU9250 CALIBRATION TEST");
    Serial.println("==========================================");


    // ==================================================
    // START I2C
    // ==================================================

    Wire.begin(SDA_PIN, SCL_PIN);

    // Slower I2C for better reliability
    Wire.setClock(100000);

    delay(500);


    // ==================================================
    // CHECK WHO_AM_I
    // ==================================================

    Serial.println();
    Serial.println("Checking MPU9250...");

    uint8_t whoAmI = 0;

    if (!readRegister(0x75, whoAmI)) {

        Serial.println("ERROR: Cannot communicate with MPU9250!");
        Serial.println("Check SDA, SCL, VCC and GND.");

        while (true) {
            delay(1000);
        }
    }

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI == 0x71) {

        Serial.println("MPU9250 detected correctly.");

    } else {

        Serial.println("WARNING: Device responded,");
        Serial.println("but WHO_AM_I is not 0x71.");
    }


    // ==================================================
    // WAKE MPU9250
    // ==================================================

    if (!writeRegister(0x6B, 0x00)) {

        Serial.println("ERROR waking MPU9250.");

        while (true) {
            delay(1000);
        }
    }

    delay(100);


    // ==================================================
    // ACCELEROMETER RANGE +/-2g
    // ==================================================

    if (!writeRegister(0x1C, 0x00)) {

        Serial.println("ERROR configuring accelerometer.");

        while (true) {
            delay(1000);
        }
    }


    // ==================================================
    // GYROSCOPE RANGE +/-250 deg/s
    // ==================================================

    if (!writeRegister(0x1B, 0x00)) {

        Serial.println("ERROR configuring gyroscope.");

        while (true) {
            delay(1000);
        }
    }


    delay(500);


    // ==================================================
    // CALIBRATION
    // ==================================================

    Serial.println();
    Serial.println("==========================================");
    Serial.println("CALIBRATION INSTRUCTIONS");
    Serial.println("==========================================");

    Serial.println();
    Serial.println("Keep CNC COMPLETELY STOPPED.");
    Serial.println("Do NOT touch the MPU9250.");
    Serial.println("Do NOT move X or Y yet.");
    Serial.println();
    Serial.println("Calibration starts in 3 seconds...");

    delay(3000);


    if (!calibrateMPU9250()) {

        Serial.println();
        Serial.println("==========================================");
        Serial.println("CALIBRATION FAILED");
        Serial.println("==========================================");

        Serial.println("Too many I2C communication errors.");
        Serial.println("Check the MPU9250 wiring.");

        while (true) {
            delay(1000);
        }
    }


    // ==================================================
    // CALIBRATION RESULTS
    // ==================================================

    Serial.println();
    Serial.println("==========================================");
    Serial.println("CALIBRATION COMPLETE");
    Serial.println("==========================================");

    Serial.println();

    Serial.println("ACCELEROMETER BASELINE:");

    Serial.print("X = ");
    Serial.print(axBaseline, 4);
    Serial.println(" g");

    Serial.print("Y = ");
    Serial.print(ayBaseline, 4);
    Serial.println(" g");

    Serial.print("Z = ");
    Serial.print(azBaseline, 4);
    Serial.println(" g");


    Serial.println();

    Serial.println("GYROSCOPE OFFSET:");

    Serial.print("X = ");
    Serial.print(gxOffset, 3);
    Serial.println(" deg/s");

    Serial.print("Y = ");
    Serial.print(gyOffset, 3);
    Serial.println(" deg/s");

    Serial.print("Z = ");
    Serial.print(gzOffset, 3);
    Serial.println(" deg/s");


    Serial.println();
    Serial.println("==========================================");
    Serial.println("NOW YOU CAN MOVE THE CNC");
    Serial.println("==========================================");

    Serial.println();
    Serial.println("Test normal X movement.");
    Serial.println("Then test normal Y movement.");
    Serial.println();
}


// ======================================================
// LOOP
// ======================================================

void loop() {

    int16_t rawAX;
    int16_t rawAY;
    int16_t rawAZ;

    int16_t rawTemp;

    int16_t rawGX;
    int16_t rawGY;
    int16_t rawGZ;


    // ==================================================
    // READ SENSOR
    // ==================================================

    if (!readMPU9250(
            rawAX,
            rawAY,
            rawAZ,
            rawTemp,
            rawGX,
            rawGY,
            rawGZ)) {

        Serial.println("MPU9250 READ FAILED");

        delay(100);
        return;
    }


    // ==================================================
    // CONVERT RAW VALUES
    // ==================================================

    float ax =
        (float)rawAX / ACCEL_SCALE;

    float ay =
        (float)rawAY / ACCEL_SCALE;

    float az =
        (float)rawAZ / ACCEL_SCALE;


    float gx =
        (float)rawGX / GYRO_SCALE;

    float gy =
        (float)rawGY / GYRO_SCALE;

    float gz =
        (float)rawGZ / GYRO_SCALE;


    // ==================================================
    // APPLY CALIBRATION
    // ==================================================

    float correctedAX =
        ax - axBaseline;

    float correctedAY =
        ay - ayBaseline;

    float correctedAZ =
        az - azBaseline;


    float correctedGX =
        gx - gxOffset;

    float correctedGY =
        gy - gyOffset;

    float correctedGZ =
        gz - gzOffset;


    // ==================================================
    // VIBRATION MAGNITUDE
    // ==================================================

    float vibration = sqrt(
        correctedAX * correctedAX +
        correctedAY * correctedAY +
        correctedAZ * correctedAZ
    );


    // ==================================================
    // SERIAL OUTPUT
    // ==================================================

    Serial.print("AX: ");
    Serial.print(correctedAX, 3);

    Serial.print(" g | AY: ");
    Serial.print(correctedAY, 3);

    Serial.print(" g | AZ: ");
    Serial.print(correctedAZ, 3);


    Serial.print(" g || GX: ");
    Serial.print(correctedGX, 2);

    Serial.print(" | GY: ");
    Serial.print(correctedGY, 2);

    Serial.print(" | GZ: ");
    Serial.print(correctedGZ, 2);


    Serial.print(" deg/s || Vibration: ");
    Serial.print(vibration, 3);
    Serial.println(" g");


    delay(100);
}


// ======================================================
// CALIBRATION
// ======================================================

bool calibrateMPU9250() {

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;


    double sumAX = 0.0;
    double sumAY = 0.0;
    double sumAZ = 0.0;

    double sumGX = 0.0;
    double sumGY = 0.0;
    double sumGZ = 0.0;


    int validSamples = 0;
    int failedSamples = 0;


    // ==================================================
    // IGNORE INITIAL READINGS
    // ==================================================

    Serial.println();
    Serial.println("Stabilizing sensor...");

    int ignoredValidSamples = 0;

    while (ignoredValidSamples < 100) {

        if (readMPU9250(
                ax,
                ay,
                az,
                temp,
                gx,
                gy,
                gz)) {

            ignoredValidSamples++;

        } else {

            failedSamples++;

            if (failedSamples >= MAX_CALIBRATION_FAILURES) {
                return false;
            }
        }

        delay(5);
    }


    // Reset failures before main calibration
    failedSamples = 0;


    // ==================================================
    // COLLECT CALIBRATION SAMPLES
    // ==================================================

    Serial.println();
    Serial.println("Calibrating...");
    Serial.println("Keep sensor STILL.");
    Serial.println();


    while (validSamples < CALIBRATION_SAMPLES) {

        if (readMPU9250(
                ax,
                ay,
                az,
                temp,
                gx,
                gy,
                gz)) {


            // ------------------------------------------
            // ACCELEROMETER
            // ------------------------------------------

            sumAX +=
                (float)ax / ACCEL_SCALE;

            sumAY +=
                (float)ay / ACCEL_SCALE;

            sumAZ +=
                (float)az / ACCEL_SCALE;


            // ------------------------------------------
            // GYROSCOPE
            // ------------------------------------------

            sumGX +=
                (float)gx / GYRO_SCALE;

            sumGY +=
                (float)gy / GYRO_SCALE;

            sumGZ +=
                (float)gz / GYRO_SCALE;


            validSamples++;


            // ------------------------------------------
            // PROGRESS
            // ------------------------------------------

            if (validSamples % 200 == 0) {

                Serial.print("Valid samples: ");
                Serial.print(validSamples);
                Serial.print(" / ");
                Serial.println(CALIBRATION_SAMPLES);
            }

        } else {

            failedSamples++;

            Serial.print("I2C failure ");
            Serial.print(failedSamples);
            Serial.print(" / ");
            Serial.println(MAX_CALIBRATION_FAILURES);


            if (
                failedSamples >=
                MAX_CALIBRATION_FAILURES
            ) {

                return false;
            }
        }


        delay(3);
    }


    // ==================================================
    // CALCULATE BASELINES
    // ==================================================

    axBaseline =
        sumAX / validSamples;

    ayBaseline =
        sumAY / validSamples;

    azBaseline =
        sumAZ / validSamples;


    gxOffset =
        sumGX / validSamples;

    gyOffset =
        sumGY / validSamples;

    gzOffset =
        sumGZ / validSamples;


    Serial.println();
    Serial.println("2000 valid samples collected.");

    return true;
}


// ======================================================
// READ MPU9250
// ======================================================

bool readMPU9250(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &temp,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
) {

    Wire.beginTransmission(MPU_ADDR);

    // First accelerometer register
    Wire.write(0x3B);


    if (Wire.endTransmission(false) != 0) {

        return false;
    }


    uint8_t received =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)14,
            (uint8_t)true
        );


    if (received != 14) {

        while (Wire.available()) {
            Wire.read();
        }

        return false;
    }


    // ==================================================
    // ACCELEROMETER
    // ==================================================

    ax =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    ay =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    az =
        ((int16_t)Wire.read() << 8) |
        Wire.read();


    // ==================================================
    // TEMPERATURE
    // ==================================================

    temp =
        ((int16_t)Wire.read() << 8) |
        Wire.read();


    // ==================================================
    // GYROSCOPE
    // ==================================================

    gx =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    gy =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    gz =
        ((int16_t)Wire.read() << 8) |
        Wire.read();


    return true;
}


// ======================================================
// READ ONE REGISTER
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
// WRITE ONE REGISTER
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