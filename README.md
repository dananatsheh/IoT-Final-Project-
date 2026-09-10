# 🚀 Smart CNC Station

## Final IoT / Field-Training Project

**Team:** Dana Al-Natsheh · Hiba Banat · Rama Haddad · Malak
Al-Sharqawi\
**Platform:** ESP32 + PlatformIO / Arduino\
**Communication:** Wi-Fi + MQTT\
**Broker:** Mosquitto\
**Monitoring:** Node-RED Dashboard\
**Database:** Firebase Realtime Database

<img width="1600" height="672" alt="image" src="https://github.com/user-attachments/assets/d5de249d-e67e-4d59-a9a1-a4bfd87fd03c" />
------------------------------------------------------------------------

## 📌 Project Overview

**Smart CNC Station** is an IoT-based monitoring and safety system
designed to enhance an existing CNC/plotter machine.

The original machine is able to execute programmed motion, but the
project adds a monitoring and communication layer around it. The system
collects machine-condition data, detects abnormal vibration, publishes
live information through MQTT, visualizes the data in Node-RED, displays
local status on an OLED, and records job information and safety outcomes
in Firebase.

The project is built around three main ideas:

> **Monitor → Detect → Act**

-   **Monitor:** continuously collect machine status, motor temperature,
    and vibration data.
-   **Detect:** identify abnormal conditions such as excessive
    vibration.
-   **Act:** trigger a safety response when a confirmed abnormal
    condition occurs.
-   **Record:** keep job and fault information for later review.

The mandatory project requirements include CNC job-state monitoring,
BME280 temperature monitoring, MPU6050 vibration monitoring, a working
E-Stop path, OLED status display, Firebase job logging, and a shared
GitHub repository.

------------------------------------------------------------------------

# 🎯 Project Objectives

The main objectives are:

1.  Monitor the CNC machine during operation.
2.  Monitor X/Y motor temperature using BME280 sensor data.
3.  Monitor end-effector vibration using MPU6050.
4.  Calculate a vibration RMS value instead of relying on a single raw
    accelerometer sample.
5.  Define a practical vibration threshold using measured machine
    behavior.
6.  Detect sustained abnormal vibration.
7.  Trigger a safety response when vibration remains above the
    configured threshold.
8.  Support manual E-Stop operation.
9.  Publish machine and sensor information through authenticated MQTT.
10. Display live information through Node-RED.
11. Show the current machine/job state locally on the OLED.
12. Log job start/end and outcomes in Firebase.
13. Maintain a clean, shared GitHub repository.

------------------------------------------------------------------------

# 🏗️ High-Level System Architecture

``` text
                         ┌──────────────────────┐
                         │      CNC / Plotter   │
                         │   X / Y / Z Motion   │
                         └──────────┬───────────┘
                                    │
                             Machine Signals
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │        ESP32         │
                         │    Main Controller   │
                         └──────────┬───────────┘
                                    │
             ┌──────────────────────┼──────────────────────┐
             │                      │                      │
             ▼                      ▼                      ▼
         BME280                  MPU6050                  OLED
     Temperature             Vibration RMS          Local Status
             │                      │
             └──────────────┬───────┘
                            │
                            ▼
                    Monitoring / Safety
                         Decision
                            │
                   ┌────────┴────────┐
                   │                 │
                   ▼                 ▼
              Normal State       Safety Fault
                                     │
                           Manual / Vibration
                                     │
                                     ▼
                            Relay / Optocoupler
                                     │
                                     ▼
                                  CNC Stop


ESP32
  │
  │ Wi-Fi / MQTT
  ▼
┌───────────────────┐
│ Mosquitto Broker  │
│   Port 1883       │
└─────────┬─────────┘
          │
          ▼
     ┌───────────┐
     │ Node-RED  │
     │ Dashboard │
     └─────┬─────┘
           │
           ▼
   Firebase Realtime
       Database
```

------------------------------------------------------------------------

# 🧩 System Components

## ESP32

The ESP32 is the main embedded controller used to connect the physical
machine, sensors, network, and monitoring system.

It is responsible for:

-   Reading sensor information.
-   Processing MPU6050 acceleration data.
-   Calculating vibration RMS.
-   Detecting abnormal vibration.
-   Maintaining Wi-Fi connectivity.
-   Maintaining MQTT connectivity.
-   Publishing sensor information.
-   Publishing device status.
-   Supporting the safety logic used by the project.

The MPU-specific firmware currently used in the project follows:

``` text
MPU6050 → ESP32 → Wi-Fi → Mosquitto → Node-RED → Dashboard
```

This MPU ESP32 publishes vibration information and does not subscribe to
MQTT commands, communicate directly with the other ESP32, or directly
control the CNC.

------------------------------------------------------------------------

# 🌡️ BME280 Temperature Monitoring

The BME280 is used for temperature monitoring associated with the X/Y
stepper motors.

The current MQTT structure includes:

``` text
sensor/bme280_x/temperature
sensor/bme280_y/temperature
```

The values are intended to be displayed live on the Node-RED dashboard.

### Data flow

``` text
BME280
   ↓
ESP32
   ↓
MQTT
   ↓
Mosquitto
   ↓
Node-RED
   ↓
Temperature Gauge
```

If a second BME280 is not physically installed/enabled, the
Y-temperature topic should only be presented as active when the second
sensor is actually available.

------------------------------------------------------------------------

# 📳 MPU6050 Vibration Monitoring

The MPU6050 is installed at the end effector to monitor machine
vibration.

The firmware configures the accelerometer for **±4 g**, corresponding to
**8192 LSB/g**.

The MPU is sampled approximately every **10 ms**, which corresponds to
approximately **100 Hz**.

The firmware applies a high-pass filtering stage to reduce the effect of
gravity and slow mechanical movement before calculating vibration.

### Processing pipeline

``` text
Raw Accelerometer
       ↓
Convert raw values to g
       ↓
High-Pass Filter
       ↓
Instantaneous Vibration Magnitude
       ↓
RMS Accumulation
       ↓
RMS Window
       ↓
Vibration RMS
```

The high-pass filter uses:

``` text
HP_ALPHA = 0.76
```

The project uses a rolling RMS window. The current firmware comments
specify a 100-sample window at 10 ms sampling, giving approximately a
**1-second RMS window**.

------------------------------------------------------------------------

# 📐 Vibration Threshold Methodology

The vibration threshold must be based on measurements from the actual
machine rather than an arbitrary value.

## Step 1 --- Stationary Calibration

At startup, the system performs stationary calibration.

The machine must remain completely still and the MPU should not be
touched.

The current firmware uses:

``` text
1000 calibration samples
```

The calibration calculates baseline X, Y, and Z acceleration values.

<img width="1600" height="968" alt="image" src="https://github.com/user-attachments/assets/33837aa6-93e4-4c22-a6eb-5a6c81182bd9" />

------------------------------------------------------------------------

## Step 2 --- Normal Operation

Run the CNC under normal operating conditions.

Record the vibration RMS values and determine the normal operating
range.

Important values include:

-   Average vibration.
-   Maximum vibration.
-   Typical vibration during motion.
-   Any short spikes that occur during normal operation.

------------------------------------------------------------------------

## Step 3 --- Controlled Abnormal Condition

Perform a controlled and safe abnormal-vibration test.

Record the resulting RMS values.

------------------------------------------------------------------------

## Step 4 --- Select the Threshold

Choose a value that provides separation between normal operation and
abnormal vibration.

For example:

``` text
Normal maximum = 0.28 g RMS
Abnormal minimum = 0.55 g RMS

Example threshold = 0.40 g RMS
```

**The values above are examples only. The final threshold must be based
on measurements from the actual CNC.**

------------------------------------------------------------------------

# ⏱️ Fault Confirmation Logic

The system does not immediately latch a fault from a single high RMS
window.

The current firmware requires:

``` text
FAULT_WINDOWS_REQUIRED = 2
```

With approximately one second per RMS window, vibration must remain
above the threshold for approximately **two consecutive seconds** before
the vibration fault is latched.

This helps reduce false triggers caused by short vibration spikes.

### Logic

``` text
              Vibration RMS
                    │
                    ▼
          Compare with Threshold
                    │
          ┌─────────┴─────────┐
          │                   │
       Below                Above
          │                   │
       NORMAL             Counter++
                              │
                         2 windows?
                         /       \
                       No         Yes
                       │           │
                    Continue    FAULT
                                  │
                                  ▼
                            E-Stop / Safety
```

Once the confirmed vibration fault is latched, the system reports the
fault state instead of treating subsequent normal readings as an
automatic recovery.

------------------------------------------------------------------------

# 🛑 E-Stop and Safety System

Safety is one of the most important parts of the project.

The system is designed to support two safety triggers:

### 1. Manual E-Stop

The operator can activate the E-Stop manually.

``` text
Manual E-Stop
      ↓
Safety Input
      ↓
Safety Logic
      ↓
Relay / Optocoupler
      ↓
CNC Stop
```

### 2. Automatic Vibration E-Stop

If vibration exceeds the configured threshold for the required
confirmation period:

``` text
MPU6050
   ↓
Vibration RMS
   ↓
Threshold Comparison
   ↓
Confirmed Fault
   ↓
Safety Trigger
   ↓
Relay / Optocoupler
   ↓
CNC Stop
```

The final demonstration must prove the actual safety behavior, not only
show a software message.

> **Safety-critical wiring must be physically verified before operating
> the CNC.**

------------------------------------------------------------------------

# 🔌 Relay and Optocoupler Isolation

The project uses a relay/optocoupler safety path as part of the CNC
E-Stop interface.

The design separates the low-voltage control logic from the machine-side
interface and provides a physical switching path for the safety action.

The safety path must be verified with the actual CNC wiring before the
live demonstration.

Do not assume a relay module's NO/NC behavior or machine E-Stop polarity
without checking the actual hardware and wiring.

------------------------------------------------------------------------

# 🖥️ OLED Local Display

The OLED provides a local status interface at the machine.

The display is intended to show information such as:

``` text
SMART CNC STATION
-----------------
STATE: RUNNING

TEMP X: 32.4 C
TEMP Y: 31.8 C
VIB:    0.180 g
```

During a safety event, the display can show the relevant fault state,
for example:

``` text
E-STOP ACTIVE

FAULT:
VIBRATION
```

The OLED state should match the state presented on the Node-RED
dashboard.

------------------------------------------------------------------------

# 📡 MQTT Communication

MQTT is the main communication protocol between the embedded system and
Node-RED.

The project uses the existing Mosquitto installation.

The Smart CNC MQTT account is:

``` text
smartcnc
```

The broker is configured for:

``` text
Port: 1883
Anonymous access: disabled
Authentication: enabled
```

The Smart CNC Node-RED broker configuration uses:

``` text
127.0.0.1:1883
```

because Mosquitto and Node-RED are running on the same laptop.

------------------------------------------------------------------------

# 🔐 MQTT Security

The Smart CNC MQTT connection was tested using authentication.

The Smart CNC MQTT user was added without overwriting the existing MQTT
account.

A local publish/subscribe test successfully verified:

``` text
Authentication          PASS
Subscribe test          PASS
Broker forwarding       PASS
```

Node-RED initially returned MQTT CONNACK code 5 because the broker
credentials were missing/incorrect.

After configuring Node-RED with the Smart CNC MQTT username and
password, the dashboard successfully received MQTT data.

------------------------------------------------------------------------

# 📋 MQTT Topic Structure

  -------------------------------------------------------------------------------
  Topic                           Purpose                 Payload
  ------------------------------- ----------------------- -----------------------
  `job/status/state`              CNC job state           `idle`, `running`,
                                                          `paused`, `faulted`,
                                                          `done`

  `job/status/start`              Job start event         timestamp

  `job/status/end`                Job completion/stop     JSON
                                  event                   

  `device/esp32/status`           ESP32 availability      device state

  `sensor/bme280_x/temperature`   X motor temperature     °C

  `sensor/bme280_y/temperature`   Y motor temperature     °C

  `sensor/mpu6050/vibration`      Vibration RMS           g RMS

  `estop/status/triggered`        E-Stop trigger reason   `manual` / `vibration`

  `estop/command`                 Manual E-Stop command   command/state

  `sensor/mpu6050/status`         MPU status              `NORMAL` / `FAULT` /
                                                          `UNCONFIGURED`

  `device/mpu_esp32/status`       MPU ESP32 availability  `online` / `offline`
  -------------------------------------------------------------------------------

### MQTT Example

``` text
Topic:
sensor/mpu6050/vibration

Payload:
0.183
```

Another example:

``` text
Topic:
job/status/state

Payload:
running
```

------------------------------------------------------------------------

# 🌐 Node-RED Dashboard

Node-RED is the real-time visualization layer.

The dashboard should provide a single view of the CNC's current
condition.

## Recommended Dashboard Layout
<img width="1917" height="907" alt="Screenshot 2026-09-10 133740" src="https://github.com/user-attachments/assets/5cb4cf1e-cf77-4f70-be9b-c41e8f4f6af6" />
<img width="1912" height="896" alt="Screenshot 2026-09-10 133801" src="https://github.com/user-attachments/assets/19c890af-4789-4954-be53-66181eac53b8" />


For a vibration fault:

``` text
JOB STATUS: FAULTED

SAFETY STATUS: E-STOP ACTIVE

FAULT:
VIBRATION

VIBRATION:
0.650 g RMS
```

------------------------------------------------------------------------

# 🔄 Node-RED Data Flow

<img width="1912" height="896" alt="Screenshot 2026-09-10 133801 - Copy" src="https://github.com/user-attachments/assets/5b56eef0-9e12-4782-9ba9-10eb4d7351bd" />


Node-RED also provides a central place to merge machine state before
writing the current system state to Firebase.

------------------------------------------------------------------------

# ☁️ Firebase Realtime Database

Firebase is used to store job and system information.

The Smart CNC Firebase project was created with Realtime Database in
**Locked mode**.

The Node-RED Firebase configuration has been prepared using the database
URL and service-account/Admin credentials.

------------------------------------------------------------------------

# 🔄 Firebase Current State

The intended live structure is:

``` text
/current
```

The `/current` object is designed to contain the latest merged Smart CNC
state.

<img width="818" height="340" alt="image" src="https://github.com/user-attachments/assets/cdf176d1-3873-43be-a4bd-faf38fd61ffc" />


The exact fields should match the final Node-RED implementation.

------------------------------------------------------------------------

# 📝 Firebase Job Logging

Permanent job history is intended to be stored under:

``` text
/jobs
```

A job record should contain:

``` text
Job ID
Start Time
End Time
Final Status
Outcome
Stop Reason
```

Example normal completion:

``` json
{
  "job_id": "JOB_001",
  "outcome": "completed",
  "stop_reason": "none"
}
```

Example safety stop:

``` json
{
  "job_id": "JOB_002",
  "outcome": "faulted",
  "stop_reason": "vibration"
}
```

The project also plans a calibration history under:

``` text
/calibration
```

for storing later coupon/threshold test results.

------------------------------------------------------------------------

# ⚠️ Current Firebase Integration Status

The Firebase infrastructure and Node-RED Firebase configuration are
prepared.

A simple test writer has also been configured.

The live `/current` merged-state writer was still under debugging in the
latest integration work.

The recommended debugging sequence is:

1.  Verify the Firebase `/test` write.
2.  Add Debug nodes before the Smart CNC merge function.
3.  Add Debug nodes after the merge function.
4.  Confirm that MQTT messages reach the merge function.
5.  Confirm that the merge function produces the expected object.
6.  Configure Firebase Out to use `SET`.
7.  Use the fixed path `current`.
8.  Test one field at a time:
    -   X temperature
    -   Job state
    -   Vibration
    -   E-Stop/status
9.  After `/current` is stable, implement permanent `/jobs` logging.
10. Implement `/calibration` logging afterward.

------------------------------------------------------------------------

# 💻 Firmware Architecture

The firmware uses PlatformIO with the Arduino framework.

Main firmware responsibilities include:

``` text
setup()
 ├── Serial initialization
 ├── Unique MQTT Client ID
 ├── Wi-Fi connection
 ├── MQTT configuration
 ├── I2C initialization
 ├── MPU initialization
 ├── Stationary calibration
 └── Vibration monitoring startup

loop()
 ├── Maintain Wi-Fi
 ├── Maintain MQTT
 ├── MQTT loop
 ├── Sample MPU
 ├── Convert raw acceleration to g
 ├── High-pass filter
 ├── Calculate vibration magnitude
 ├── Accumulate RMS
 ├── Check threshold
 └── Publish MQTT data
```

------------------------------------------------------------------------

# 📈 MPU Processing Details

The accelerometer is configured as:

``` text
Range: ±4 g
Sensitivity: 8192 LSB/g
```

Sampling:

``` text
Sample interval: 10 ms
Approximate rate: 100 Hz
```

Calibration:

``` text
1000 samples
```

RMS processing:

``` text
100 samples per RMS window
≈ 1 second
```

Fault confirmation:

``` text
2 consecutive RMS windows
≈ 2 seconds
```

High-pass filter:

``` text
HP_ALPHA = 0.76
```

------------------------------------------------------------------------

# 🔧 I2C Configuration

The current MPU ESP32 firmware uses:

``` text
SDA = GPIO 21
SCL = GPIO 4
I2C frequency = 100 kHz
```

The MPU address is selected through the project's configured MPU address
definition.

The schematic should clearly document:

-   SDA connection.
-   SCL connection.
-   Power.
-   Ground.
-   Sensor address.
-   Any address-selection pin configuration.

------------------------------------------------------------------------

# 🧪 Testing Strategy

The system should be tested in layers.

## 1. Hardware Test

-   ESP32 power
-   I2C wiring
-   MPU detection
-   BME280 detection
-   OLED operation
-   Relay operation
-   Optocoupler interface
-   CNC input signals

## 2. Sensor Test

-   Stationary MPU calibration
-   Normal vibration measurement
-   Temperature readings
-   RMS calculation
-   Threshold response

## 3. MQTT Test

Verify:

``` text
ESP32
  ↓
Wi-Fi
  ↓
Mosquitto
  ↓
Node-RED
```

Check every required topic individually.

## 4. Dashboard Test

Verify that:

-   Job status changes live.
-   Temperature gauges update.
-   Vibration value updates.
-   E-Stop status updates.
-   Fault state is visible.

## 5. Safety Test

Perform:

### Manual test

``` text
Manual E-Stop
→ Safety trigger
→ Relay response
→ CNC stops
```

### Automatic test

``` text
Abnormal vibration
→ Threshold exceeded
→ Fault confirmation
→ Safety trigger
→ CNC stops
```

## 6. Firebase Test

Verify:

-   Test write.
-   `/current`.
-   Job start.
-   Job end.
-   Normal completion.
-   Safety fault.
-   Stop reason.

------------------------------------------------------------------------

# 🧰 Troubleshooting

## Node-RED MQTT Not Connecting

Check:

``` text
Broker: 127.0.0.1
Port: 1883
Username: smartcnc
Password: correct Smart CNC password
```

Do not enable anonymous access when the broker is configured for
authentication.

------------------------------------------------------------------------

## MQTT CONNACK Code 5

If Node-RED reports:

``` text
CONNACK code 5
```

check the MQTT username/password in the Node-RED broker configuration.

The Smart CNC integration previously encountered this issue and resolved
it by configuring the correct Smart CNC credentials.

------------------------------------------------------------------------

## MPU Not Detected

Check:

``` text
VCC
GND
SDA
SCL
I2C address
```

The firmware performs a WHO_AM_I register check during startup.

------------------------------------------------------------------------

## Vibration Fault Triggering Too Easily

Do not immediately increase the threshold arbitrarily.

First:

1.  Verify sensor mounting.
2.  Run the CNC normally.
3.  Record normal RMS values.
4.  Check maximum normal vibration.
5.  Repeat the abnormal test.
6.  Recalculate the separation between normal and abnormal behavior.
7.  Update the threshold based on measured evidence.

------------------------------------------------------------------------

## Firebase `/current` Not Updating

Follow this sequence:

``` text
MQTT Input
    ↓
Debug
    ↓
Merge Function
    ↓
Debug
    ↓
Firebase Out
    ↓
Firebase /current
```

Confirm that the Firebase Out node uses:

``` text
SET
Path: current
```

and does not incorrectly use `msg.topic` as the Firebase path.

------------------------------------------------------------------------

# 🔒 Security Rules

**Never commit secrets to GitHub.**

Do NOT upload:

-   Wi-Fi password.
-   MQTT password.
-   Firebase service-account private key.
-   Mosquitto password hashes.
-   Private configuration files containing credentials.

Keep the Mosquitto password file outside the repository.

Keep the Firebase service-account JSON local to the Node-RED machine.

For a public repository, use a safe configuration template such as:

``` text
config.example.h
```

and keep the real credentials in a local ignored configuration file.

------------------------------------------------------------------------

# 📁 Recommended Repository Structure

``` text
Smart-CNC-Station/
│
├── src/
│   └── main.cpp
│
├── include/
│   ├── config.example.h
│   └── README_CONFIG.md
│
├── lib/
│
├── test/
│
├── node-red/
│   └── smart-cnc-flow.json
│
├── docs/
│   ├── schematic/
│   ├── wiring/
│   ├── threshold-tests/
│   └── presentation/
│
├── .gitignore
├── platformio.ini
└── README.md
```

------------------------------------------------------------------------

# 📊 Core vs Bonus Features

## ✅ Core Features

  -----------------------------------------------------------------------
  Feature                             Description
  ----------------------------------- -----------------------------------
  CNC Job Status                      Monitor
                                      idle/running/paused/done/faulted
                                      states

  BME280                              Monitor X/Y motor temperature

  MPU6050                             Monitor end-effector vibration

  Vibration Threshold                 Detect abnormal vibration

  Automatic Safety                    Trigger safety response after
                                      confirmed vibration fault

  Manual E-Stop                       Physical/manual safety trigger

  Relay/Optocoupler                   Safety isolation/interface

  OLED                                Local machine status

  MQTT                                Real-time communication

  Node-RED                            Live dashboard

  Firebase                            Job/outcome logging

  GitHub                              Shared team repository
  -----------------------------------------------------------------------

## ⭐ Possible Bonus Features

Possible optional features include:

-   Voice commands.
-   ESP32-CAM quality inspection.
-   Audio fault detection.
-   Camera-based safety monitoring.
-   Pre-start camera inspection.
-   OLED setup assistant.

Bonus features should only be presented as completed if they are
actually working during the live demonstration.

------------------------------------------------------------------------

# 🎬 Final Live Demonstration

The recommended demonstration sequence is:

## Demo 1 --- Normal State

Show:

``` text
CNC → IDLE
OLED → IDLE
Node-RED → IDLE
```

## Demo 2 --- Start Job

Start the CNC job.

Show:

``` text
IDLE
 ↓
RUNNING
```

The dashboard and OLED should update.

## Demo 3 --- Live Monitoring

Show:

-   X temperature.
-   Y temperature.
-   Vibration RMS.
-   Job status.
-   Safety status.

## Demo 4 --- Manual E-Stop

Trigger the manual E-Stop.

Show:

``` text
E-STOP
 ↓
CNC STOP
 ↓
FAULT / SAFETY STATUS
```

## Demo 5 --- Vibration Fault

Perform the controlled vibration test.

Show:

``` text
Vibration increases
        ↓
Threshold exceeded
        ↓
Fault confirmation
        ↓
VIBRATION FAULT
        ↓
E-STOP
        ↓
CNC STOP
```

## Demo 6 --- Firebase

Show the corresponding job record and stop reason.

This proves that the system does not only stop the machine but also
records why the job stopped.

------------------------------------------------------------------------

# 📐 Schematic Requirements

The final schematic should clearly show the complete system.

At minimum it should include:

-   ESP32.
-   BME280.
-   MPU6050.
-   OLED.
-   Relay.
-   Optocoupler.
-   CNC/controller interface.
-   E-Stop button.
-   Power connections.
-   Ground connections.
-   I2C SDA/SCL.
-   I2C addresses where applicable.
-   Relevant GPIO numbers.
-   MQTT/network connection at a block-diagram level.

The final wiring should be clean, labeled, organized, and suitable for
the live presentation.

------------------------------------------------------------------------

# 📝 Technical Report Topics

The final report should explain:

1.  Project motivation.
2.  CNC overview.
3.  Problem statement.
4.  System objectives.
5.  System architecture.
6.  Hardware components.
7.  ESP32 controller.
8.  BME280 temperature monitoring.
9.  MPU6050 vibration monitoring.
10. Vibration filtering and RMS calculation.
11. Threshold selection methodology.
12. E-Stop and safety path.
13. Relay/optocoupler isolation.
14. MQTT communication.
15. Mosquitto broker.
16. Node-RED dashboard.
17. Firebase architecture.
18. Job logging.
19. Testing and results.
20. Challenges.
21. Limitations.
22. Bonus features.
23. Future improvements.
24. Team contributions.
25. Conclusion.

------------------------------------------------------------------------

# 👥 Team

## Dana Al-Natsheh

Project team member.

## Hiba Banat

Project team member.

## Rama Haddad

Project team member.

## Malak Al-Sharqawi

Project team member.

> Team contributions should be updated in this README with the actual
> work completed by each member before final submission.

------------------------------------------------------------------------

# 🚧 Current Development Status

### Working / Verified

-   ESP32 MPU vibration firmware.
-   MPU initialization and accelerometer reading.
-   Stationary calibration.
-   High-pass vibration filtering.
-   RMS vibration calculation.
-   MQTT publishing.
-   Mosquitto broker.
-   Smart CNC MQTT authentication.
-   Node-RED MQTT connectivity.
-   Node-RED dashboard MQTT data path.
-   BME280 X-temperature dashboard test.
-   Firebase project and Realtime Database base configuration.
-   Firebase test-write configuration.

### In Progress

-   Firebase `/current` live merged-state writer.
-   Permanent `/jobs` logging.
-   `/calibration` logging.
-   Final end-to-end integration.
-   Final live safety verification.

------------------------------------------------------------------------

# 📚 Important Design Decisions

### Why MQTT?

MQTT provides a lightweight publish/subscribe mechanism that allows the
ESP32 to publish machine data without being directly coupled to the
Node-RED dashboard.

### Why Node-RED?

Node-RED provides a convenient visual layer for receiving MQTT messages,
processing system state, and presenting live machine information.

### Why Firebase?

Firebase provides persistent storage so job outcomes and safety events
can be reviewed after the job has finished.

### Why RMS vibration?

A single acceleration sample can contain spikes and noise. RMS over a
time window provides a more useful representation of vibration level for
threshold-based monitoring.

### Why fault confirmation windows?

The system requires multiple consecutive high RMS windows before
latching a vibration fault. This reduces the chance of stopping the CNC
because of a very short transient spike.

------------------------------------------------------------------------

# 🔮 Future Improvements

With additional development time, the project could be improved by:

-   Adding more detailed motor-driver telemetry when supported by the
    actual driver.
-   Improving vibration threshold calibration using multiple jobs.
-   Adding historical vibration graphs.
-   Adding a more detailed Firebase job schema.
-   Adding automatic fault-recovery workflows where safely appropriate.
-   Adding camera-based quality inspection.
-   Adding audio-based fault detection.
-   Adding voice commands.
-   Adding a pre-start inspection workflow.
-   Improving dashboard visualization and alarms.

------------------------------------------------------------------------

# ⚠️ Safety Disclaimer

This project interfaces with a real CNC machine.

Before every live test:

-   Verify the E-Stop circuit.
-   Verify relay behavior.
-   Verify the correct power path.
-   Check wiring and connectors.
-   Keep emergency access clear.
-   Do not rely only on software for a safety-critical stop.
-   Test the machine at a safe operating condition.
-   Never intentionally create a dangerous mechanical fault for
    demonstration.

The live demonstration should use a controlled and safe test condition.

------------------------------------------------------------------------

# 🏁 Conclusion

The **Smart CNC Station** demonstrates how an existing CNC machine can
be enhanced with embedded sensing, IoT communication, real-time
visualization, and safety-oriented monitoring.

The complete concept is:

``` text
             ┌─────────────┐
             │    SENSE    │
             │ BME280/MPU  │
             └──────┬──────┘
                    ↓
             ┌─────────────┐
             │   PROCESS   │
             │    ESP32    │
             └──────┬──────┘
                    ↓
             ┌─────────────┐
             │   DETECT    │
             │  Threshold  │
             └──────┬──────┘
                    ↓
             ┌─────────────┐
             │    ACT      │
             │   E-STOP    │
             └──────┬──────┘
                    ↓
             ┌─────────────┐
             │   MONITOR   │
             │ Node-RED    │
             └──────┬──────┘
                    ↓
             ┌─────────────┐
             │    LOG      │
             │  Firebase   │
             └─────────────┘
```

**Smart CNC Station = Monitoring + Detection + Safety + IoT + Data
Logging**
