#pragma once

#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define MQTT_BROKER_HOST "192.168.1.100"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "smart-cnc-esp32"
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

#define TOPIC_STATE "job/status/state"
#define TOPIC_TEMP_X "sensor/bme280_x/temperature"
#define TOPIC_TEMP_Y "sensor/bme280_y/temperature"
#define TOPIC_VIBRATION "sensor/mpu6050/vibration"
#define TOPIC_ESTOP_EVENT "estop/status/triggered"
#define TOPIC_ESTOP_COMMAND "estop/command"
#define TOPIC_JOB_START "job/status/start"
#define TOPIC_JOB_END "job/status/end"
#define TOPIC_DEVICE_STATUS "device/esp32/status"

#define SDA_BUS1 21
#define SCL_BUS1 23
#define SDA_BUS2 32
#define SCL_BUS2 33
#define OLED_SDA 4
#define OLED_SCL 16

#define BME_X_ADDRESS 0x76
#define BME_Y_ADDRESS 0x76
#define MPU_ADDRESS 0x68
#define OLED_ADDRESS 0x3C

#define DI_IN1_PIN 19
#define DI_IN2_PIN 18
#define DI_IN3_PIN 5
#define DI_IN4_PIN 17
#define DI_ACTIVE_LEVEL LOW

#define ESTOP_RELAY_PIN 27
#define ESTOP_RELAY_ACTIVE_LEVEL HIGH
#define ESTOP_RELAY_INACTIVE_LEVEL LOW

#define VIBRATION_THRESHOLD_G 0.25339f
#define VIBRATION_WINDOW_SIZE 100
#define VIBRATION_SAMPLE_INTERVAL_MS 10

#define I2C_CLOCK_HZ 100000
#define SENSOR_PUBLISH_INTERVAL_MS 1000
#define OLED_UPDATE_INTERVAL_MS 250
#define WIFI_RECONNECT_INTERVAL_MS 10000
#define MQTT_RECONNECT_INTERVAL_MS 3000