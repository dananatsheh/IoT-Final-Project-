#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

#define SDA_PIN 21
#define SCL_PIN 23

// MPU9250 sensitivity
const float ACCEL_SCALE = 16384.0;  // ±2g
const float GYRO_SCALE  = 131.0;    // ±250 deg/s


bool readMPU9250(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &temp,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
);


void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println("ESP32 + MPU9250 TEST");
    Serial.println("==============================");


    // --------------------------------------------
    // Check MPU9250
    // --------------------------------------------

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75); // WHO_AM_I register

    if (Wire.endTransmission(false) != 0) {

        Serial.println("ERROR: MPU9250 not responding!");

        while (1) {
            delay(1000);
        }
    }

    Wire.requestFrom(
        (uint8_t)MPU_ADDR,
        (uint8_t)1,
        (uint8_t)true
    );

    if (Wire.available()) {

        uint8_t whoAmI = Wire.read();

        Serial.print("WHO_AM_I = 0x");
        Serial.println(whoAmI, HEX);

        // Genuine MPU9250 normally reports 0x71
        if (whoAmI == 0x71) {
            Serial.println("MPU9250 detected correctly.");
        } else {
            Serial.println("Unexpected WHO_AM_I value.");
        }
    }


    // --------------------------------------------
    // Wake MPU9250
    // --------------------------------------------

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);    // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);


    // --------------------------------------------
    // Accelerometer ±2g
    // --------------------------------------------

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);    // ACCEL_CONFIG
    Wire.write(0x00);
    Wire.endTransmission();


    // --------------------------------------------
    // Gyroscope ±250 degrees/sec
    // --------------------------------------------

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);    // GYRO_CONFIG
    Wire.write(0x00);
    Wire.endTransmission();


    delay(500);

    Serial.println();
    Serial.println("Reading MPU9250...");
    Serial.println();
}


void loop() {

    int16_t rawAX;
    int16_t rawAY;
    int16_t rawAZ;

    int16_t rawTemp;

    int16_t rawGX;
    int16_t rawGY;
    int16_t rawGZ;


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


    // --------------------------------------------
    // Convert accelerometer to g
    // --------------------------------------------

    float ax = rawAX / ACCEL_SCALE;
    float ay = rawAY / ACCEL_SCALE;
    float az = rawAZ / ACCEL_SCALE;


    // --------------------------------------------
    // Convert gyroscope to degrees/sec
    // --------------------------------------------

    float gx = rawGX / GYRO_SCALE;
    float gy = rawGY / GYRO_SCALE;
    float gz = rawGZ / GYRO_SCALE;


    // --------------------------------------------
    // Temperature
    // --------------------------------------------

    float temperature =
        (rawTemp / 333.87) + 21.0;


    // --------------------------------------------
    // Print values
    // --------------------------------------------

    Serial.print("AX: ");
    Serial.print(ax, 3);
    Serial.print(" g");

    Serial.print(" | AY: ");
    Serial.print(ay, 3);
    Serial.print(" g");

    Serial.print(" | AZ: ");
    Serial.print(az, 3);
    Serial.print(" g");


    Serial.print(" || GX: ");
    Serial.print(gx, 2);
    Serial.print(" deg/s");

    Serial.print(" | GY: ");
    Serial.print(gy, 2);
    Serial.print(" deg/s");

    Serial.print(" | GZ: ");
    Serial.print(gz, 2);
    Serial.print(" deg/s");


    Serial.print(" || Temp: ");
    Serial.print(temperature, 1);
    Serial.println(" C");


    delay(100);
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

    // Accelerometer data begins at 0x3B
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


    // Accelerometer
    ax =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    ay =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    az =
        ((int16_t)Wire.read() << 8) |
        Wire.read();


    // Temperature
    temp =
        ((int16_t)Wire.read() << 8) |
        Wire.read();


    // Gyroscope
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