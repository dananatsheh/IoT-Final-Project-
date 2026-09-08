#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>

#define SDA_BUS1 21
#define SCL_BUS1 23
#define SDA_BUS2 32
#define SCL_BUS2 33
#define OLED_SDA 4
#define OLED_SCL 16

#define BME1_ADDRESS 0x76
#define BME2_ADDRESS 0x76
#define MPU_ADDRESS 0x68
#define OLED_ADDRESS 0x3C

TwoWire &I2C_BUS1 = Wire;
TwoWire &I2C_BUS2 = Wire1;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_MPU6050 mpu;

bool bme1OK = false;
bool bme2OK = false;
bool mpuOK = false;
bool oledOK = false;

const int SAMPLE_COUNT = 100;
const int SAMPLE_DELAY_MS = 10;

const float STILL_MEAN_G = 0.00606;
const float STILL_MAX_G = 0.00664;
const float NORMAL_MEAN_G = 0.15482;
const float NORMAL_STD_G = 0.02677;
const float NORMAL_MAX_G = 0.21116;

const float THRESHOLD_1 = 1.20 * NORMAL_MAX_G;
const float THRESHOLD_2 = NORMAL_MEAN_G + (3.0 * NORMAL_STD_G);
const float VIBRATION_THRESHOLD_G = (THRESHOLD_1 > THRESHOLD_2) ? THRESHOLD_1 : THRESHOLD_2;

unsigned long readingNumber = 0;

bool devicePresent(TwoWire &bus, uint8_t address) {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

void scanBus(TwoWire &bus, const char *name) {
    Serial.println();
    Serial.print("===== I2C SCAN: ");
    Serial.print(name);
    Serial.println(" =====");

    int count = 0;

    for (uint8_t address = 1; address < 127; address++) {
        bus.beginTransmission(address);

        if (bus.endTransmission() == 0) {
            Serial.print("FOUND: 0x");

            if (address < 16) {
                Serial.print("0");
            }

            Serial.println(address, HEX);
            count++;
        }
    }

    Serial.print("TOTAL: ");
    Serial.println(count);
    Serial.println("==========================");
}

float readBME280Temperature(Adafruit_BME280 &sensor, bool sensorOK, TwoWire &bus, uint8_t address) {
    if (!sensorOK) {
        return NAN;
    }

    if (!devicePresent(bus, address)) {
        return NAN;
    }

    float temperature = sensor.readTemperature();

    if (!isfinite(temperature) || temperature < -40.0 || temperature > 85.0) {
        return NAN;
    }

    return temperature;
}

float readVibrationRMS() {
    if (!mpuOK) {
        return NAN;
    }

    if (!devicePresent(I2C_BUS2, MPU_ADDRESS)) {
        return NAN;
    }

    float ax[SAMPLE_COUNT];
    float ay[SAMPLE_COUNT];
    float az[SAMPLE_COUNT];

    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        bool success = mpu.getEvent(&accel, &gyro, &temp);

        if (!success) {
            Serial.println("MPU6050 READ ERROR");
            return NAN;
        }

        if (!isfinite(accel.acceleration.x) ||
            !isfinite(accel.acceleration.y) ||
            !isfinite(accel.acceleration.z)) {
            Serial.println("MPU6050 INVALID DATA");
            return NAN;
        }

        ax[i] = accel.acceleration.x;
        ay[i] = accel.acceleration.y;
        az[i] = accel.acceleration.z;

        sumX += ax[i];
        sumY += ay[i];
        sumZ += az[i];

        delay(SAMPLE_DELAY_MS);
    }

    float meanX = sumX / SAMPLE_COUNT;
    float meanY = sumY / SAMPLE_COUNT;
    float meanZ = sumZ / SAMPLE_COUNT;

    float sumSquares = 0.0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float dx = ax[i] - meanX;
        float dy = ay[i] - meanY;
        float dz = az[i] - meanZ;

        sumSquares += dx * dx + dy * dy + dz * dz;
    }

    float vibrationMS2 = sqrt(sumSquares / SAMPLE_COUNT);
    float vibrationG = vibrationMS2 / 9.80665;

    return vibrationG;
}

const char *getVibrationStatus(float vibration) {
    if (isnan(vibration)) {
        return "SENSOR ERROR";
    }

    if (vibration > VIBRATION_THRESHOLD_G) {
        return "HIGH VIBRATION";
    }

    return "NORMAL";
}

void updateOLED(float temp1, float temp2, float vibration) {
    if (!oledOK) {
        return;
    }

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);

    oled.setCursor(0, 10);
    oled.print("SMART CNC");

    oled.setCursor(0, 22);
    oled.print("X TEMP: ");

    if (isnan(temp1)) {
        oled.print("ERROR");
    } else {
        oled.print(temp1, 1);
        oled.print(" C");
    }

    oled.setCursor(0, 34);
    oled.print("Y TEMP: ");

    if (isnan(temp2)) {
        oled.print("ERROR");
    } else {
        oled.print(temp2, 1);
        oled.print(" C");
    }

    oled.setCursor(0, 46);
    oled.print("VIB: ");

    if (isnan(vibration)) {
        oled.print("ERROR");
    } else {
        oled.print(vibration, 4);
        oled.print(" g");
    }

    oled.setCursor(0, 58);

    if (isnan(vibration)) {
        oled.print("STATUS: ERROR");
    } else if (vibration > VIBRATION_THRESHOLD_G) {
        oled.print("!! HIGH VIB !!");
    } else {
        oled.print("STATUS: NORMAL");
    }

    oled.sendBuffer();
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("=================================");
    Serial.println("SMART CNC MONITORING SYSTEM");
    Serial.println("=================================");

    I2C_BUS1.begin(SDA_BUS1, SCL_BUS1, 100000);
    I2C_BUS2.begin(SDA_BUS2, SCL_BUS2, 100000);

    scanBus(I2C_BUS1, "BUS 1 - BME280 #1");
    scanBus(I2C_BUS2, "BUS 2 - BME280 #2 + MPU6050");

    if (devicePresent(I2C_BUS1, BME1_ADDRESS)) {
        bme1OK = bme1.begin(BME1_ADDRESS, &I2C_BUS1);
    }

    Serial.print("BME280 #1: ");
    Serial.println(bme1OK ? "OK" : "ERROR");

    if (devicePresent(I2C_BUS2, BME2_ADDRESS)) {
        bme2OK = bme2.begin(BME2_ADDRESS, &I2C_BUS2);
    }

    Serial.print("BME280 #2: ");
    Serial.println(bme2OK ? "OK" : "ERROR");

    if (devicePresent(I2C_BUS2, MPU_ADDRESS)) {
        mpuOK = mpu.begin(MPU_ADDRESS, &I2C_BUS2);
    }

    Serial.print("MPU6050: ");

    if (mpuOK) {
        Serial.println("OK");
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    } else {
        Serial.println("ERROR");
    }

    oled.setI2CAddress(OLED_ADDRESS << 1);
    oled.begin();
    oledOK = true;

    Serial.println("OLED: OK");

    Serial.println();
    Serial.println("===== VIBRATION CALIBRATION =====");

    Serial.print("Still Mean       : ");
    Serial.print(STILL_MEAN_G, 5);
    Serial.println(" g");

    Serial.print("Still Maximum    : ");
    Serial.print(STILL_MAX_G, 5);
    Serial.println(" g");

    Serial.print("Normal Mean      : ");
    Serial.print(NORMAL_MEAN_G, 5);
    Serial.println(" g");

    Serial.print("Normal STD       : ");
    Serial.print(NORMAL_STD_G, 5);
    Serial.println(" g");

    Serial.print("Normal Maximum   : ");
    Serial.print(NORMAL_MAX_G, 5);
    Serial.println(" g");

    Serial.print("Threshold Method1: ");
    Serial.print(THRESHOLD_1, 5);
    Serial.println(" g");

    Serial.print("Threshold Method2: ");
    Serial.print(THRESHOLD_2, 5);
    Serial.println(" g");

    Serial.println("-------------------------------");

    Serial.print("FINAL THRESHOLD  : ");
    Serial.print(VIBRATION_THRESHOLD_G, 5);
    Serial.println(" g");

    Serial.println("=================================");
    Serial.println();
    Serial.println("SYSTEM READY");
}

void loop() {
    readingNumber++;

    float temp1 = readBME280Temperature(bme1, bme1OK, I2C_BUS1, BME1_ADDRESS);
    float temp2 = readBME280Temperature(bme2, bme2OK, I2C_BUS2, BME2_ADDRESS);
    float vibration = readVibrationRMS();

    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("READING #");
    Serial.println(readingNumber);

    Serial.print("Temperature X: ");

    if (isnan(temp1)) {
        Serial.println("ERROR / NOT CONNECTED");
    } else {
        Serial.print(temp1, 2);
        Serial.println(" C");
    }

    Serial.print("Temperature Y: ");

    if (isnan(temp2)) {
        Serial.println("ERROR / NOT CONNECTED");
    } else {
        Serial.print(temp2, 2);
        Serial.println(" C");
    }

    Serial.print("Vibration RMS: ");

    if (isnan(vibration)) {
        Serial.println("ERROR");
    } else {
        Serial.print(vibration, 5);
        Serial.println(" g");
    }

    Serial.print("Threshold: ");
    Serial.print(VIBRATION_THRESHOLD_G, 5);
    Serial.println(" g");

    Serial.print("Machine Status: ");
    Serial.println(getVibrationStatus(vibration));

    if (!isnan(vibration) && vibration > VIBRATION_THRESHOLD_G) {
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("WARNING: ABNORMAL VIBRATION");
        Serial.println("THRESHOLD EXCEEDED");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    Serial.println("--------------------------------");

    updateOLED(temp1, temp2, vibration);

    delay(500);
}