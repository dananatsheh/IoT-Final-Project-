#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>

// =====================================================
// I2C — routed through ES32C14 "Free Port" pins only.
// IMPORTANT: SCL moved from the ESP32 default (GPIO22)
// to GPIO33, because GPIO22 is hard-wired to the
// ES32C14's RS485 port (IO1/IO3/IO22) and is NOT free.
// GPIO21 (SDA) IS on the Free Port list, so it's kept.
// =====================================================

#define SDA_PIN 21   // ES32C14 Free Port
#define SCL_PIN 16   // ES32C14 Free Port (was 22 - conflicted with RS485)

#define BME_ADDRESS  0x76
#define MPU_ADDRESS  0x68
#define OLED_ADDRESS 0x3C

// =====================================================
// OLED TYPE
// =====================================================

// 1 = SSD1306 128x64
// 2 = SSD1306 128x32
// 3 = SH1106 128x64

#define OLED_MODE 1

Adafruit_SSD1306 oled64(
    128,
    64,
    &Wire,
    -1
);

Adafruit_SSD1306 oled32(
    128,
    32,
    &Wire,
    -1
);

Adafruit_SH1106G sh1106(
    128,
    64,
    &Wire,
    -1
);

// =====================================================
// SENSORS
// =====================================================

Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

bool bmeOK = false;
bool mpuOK = false;
bool oledOK = false;

// =====================================================
// VIBRATION SETTINGS
// =====================================================

// Each displayed vibration reading is calculated
// from 100 accelerometer samples.

const int SAMPLE_COUNT = 100;

// 10 ms = about 100 Hz sampling
const int SAMPLE_DELAY_MS = 10;

// Reading counter
unsigned long readingNumber = 0;

// =====================================================
// OLED INITIALIZATION
// =====================================================

bool initOLED() {

#if OLED_MODE == 1

    return oled64.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    );

#elif OLED_MODE == 2

    return oled32.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    );

#elif OLED_MODE == 3

    return sh1106.begin(
        OLED_ADDRESS,
        true
    );

#endif
}

// =====================================================
// I2C SCANNER
// =====================================================

void scanI2C() {

    Serial.println();
    Serial.println("===== I2C SCANNER =====");
    Serial.print("SDA=GPIO");
    Serial.print(SDA_PIN);
    Serial.print("  SCL=GPIO");
    Serial.println(SCL_PIN);

    for (uint8_t address = 1;
         address < 127;
         address++) {

        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0) {

            Serial.print("FOUND: 0x");

            if (address < 16)
                Serial.print("0");

            Serial.println(address, HEX);
        }
    }

    Serial.println("=======================");
}

// =====================================================
// VIBRATION RMS
// =====================================================

float readVibrationRMS() {

    if (!mpuOK)
        return 0.0;

    float ax[SAMPLE_COUNT];
    float ay[SAMPLE_COUNT];
    float az[SAMPLE_COUNT];

    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;

    // -----------------------------------------
    // Collect accelerometer samples
    // -----------------------------------------

    for (int i = 0;
         i < SAMPLE_COUNT;
         i++) {

        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        mpu.getEvent(
            &accel,
            &gyro,
            &temp
        );

        ax[i] = accel.acceleration.x;
        ay[i] = accel.acceleration.y;
        az[i] = accel.acceleration.z;

        sumX += ax[i];
        sumY += ay[i];
        sumZ += az[i];

        delay(SAMPLE_DELAY_MS);
    }

    // -----------------------------------------
    // Calculate average acceleration
    // -----------------------------------------

    float meanX = sumX / SAMPLE_COUNT;
    float meanY = sumY / SAMPLE_COUNT;
    float meanZ = sumZ / SAMPLE_COUNT;

    // -----------------------------------------
    // Calculate vibration around the average
    // -----------------------------------------

    float sumSquares = 0.0;

    for (int i = 0;
         i < SAMPLE_COUNT;
         i++) {

        float dx = ax[i] - meanX;
        float dy = ay[i] - meanY;
        float dz = az[i] - meanZ;

        sumSquares += dx * dx + dy * dy + dz * dz;
    }

    float vibrationMS2 = sqrt(sumSquares / SAMPLE_COUNT);

    // Convert m/s^2 to g
    float vibrationG = vibrationMS2 / 9.80665;

    return vibrationG;
}

// =====================================================
// READ CURRENT ACCELERATION
// =====================================================

void readAcceleration(float &x, float &y, float &z) {

    x = 0;
    y = 0;
    z = 0;

    if (!mpuOK)
        return;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);

    x = accel.acceleration.x;
    y = accel.acceleration.y;
    z = accel.acceleration.z;
}

// =====================================================
// OLED DISPLAY
// =====================================================

void updateOLED(float temperature, float vibration) {

    if (!oledOK)
        return;

#if OLED_MODE == 1

    oled64.clearDisplay();
    oled64.setTextColor(SSD1306_WHITE);
    oled64.setTextSize(1);
    oled64.setCursor(0, 0);

    oled64.println("SMART CNC");
    oled64.println("----------------");

    oled64.print("READ: ");
    oled64.println(readingNumber);

    oled64.print("TEMP: ");
    oled64.print(temperature, 1);
    oled64.println(" C");

    oled64.print("VIB:  ");
    oled64.print(vibration, 5);
    oled64.println(" g");

    oled64.println();
    oled64.println("SENSOR MONITOR");

    oled64.display();


#elif OLED_MODE == 2

    oled32.clearDisplay();
    oled32.setTextColor(SSD1306_WHITE);
    oled32.setTextSize(1);
    oled32.setCursor(0, 0);

    oled32.print("R:");
    oled32.println(readingNumber);

    oled32.print("T:");
    oled32.print(temperature, 1);
    oled32.println(" C");

    oled32.print("V:");
    oled32.print(vibration, 4);
    oled32.println(" g");

    oled32.display();


#elif OLED_MODE == 3

    sh1106.clearDisplay();
    sh1106.setTextColor(SH110X_WHITE);
    sh1106.setTextSize(1);
    sh1106.setCursor(0, 0);

    sh1106.println("SMART CNC");
    sh1106.println("----------------");

    sh1106.print("READ: ");
    sh1106.println(readingNumber);

    sh1106.print("TEMP: ");
    sh1106.print(temperature, 1);
    sh1106.println(" C");

    sh1106.print("VIB:  ");
    sh1106.print(vibration, 5);
    sh1106.println(" g");

    sh1106.println();
    sh1106.println("SENSOR MONITOR");

    sh1106.display();

#endif
}

// =====================================================
// SETUP
// =====================================================

void setup() {

    Serial.begin(115200);
    delay(1500);

    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println();
    Serial.println("==========================");
    Serial.println("SMART CNC SENSOR MONITOR");
    Serial.println("ES32C14 + ESP32 build");
    Serial.println("==========================");

    scanI2C();

    // -----------------------------------------
    // BME280
    // -----------------------------------------

    bmeOK = bme.begin(BME_ADDRESS);

    Serial.print("BME280: ");
    Serial.println(bmeOK ? "OK" : "ERROR");

    // -----------------------------------------
    // MPU6050
    // -----------------------------------------

    mpuOK = mpu.begin(MPU_ADDRESS);

    Serial.print("MPU6050: ");

    if (mpuOK) {

        Serial.println("OK");

        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    } else {

        Serial.println("ERROR");
    }

    // -----------------------------------------
    // OLED
    // -----------------------------------------

    oledOK = initOLED();

    Serial.print("OLED: ");
    Serial.println(oledOK ? "OK" : "ERROR");

    Serial.println();
    Serial.println("SYSTEM READY");
    Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

    readingNumber++;

    // =========================================
    // Temperature
    // =========================================

    float temperature = NAN;

    if (bmeOK) {
        temperature = bme.readTemperature();
    }

    // =========================================
    // Current acceleration
    // =========================================

    float ax, ay, az;
    readAcceleration(ax, ay, az);

    // =========================================
    // Vibration
    // =========================================

    float vibration = readVibrationRMS();

    // =========================================
    // SERIAL OUTPUT
    // =========================================

    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("READING #");
    Serial.println(readingNumber);

    Serial.print("Temperature: ");

    if (bmeOK) {
        Serial.print(temperature, 2);
        Serial.println(" C");
    } else {
        Serial.println("ERROR");
    }

    Serial.println();

    Serial.print("Accel X: ");
    Serial.print(ax, 3);
    Serial.println(" m/s2");

    Serial.print("Accel Y: ");
    Serial.print(ay, 3);
    Serial.println(" m/s2");

    Serial.print("Accel Z: ");
    Serial.print(az, 3);
    Serial.println(" m/s2");

    Serial.println();

    Serial.print("Vibration RMS: ");
    Serial.print(vibration, 5);
    Serial.println(" g");

    Serial.println("--------------------------------");

    // =========================================
    // OLED
    // =========================================

    updateOLED(temperature, vibration);

    delay(500);
}