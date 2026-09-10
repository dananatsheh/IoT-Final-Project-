#include <Arduino.h>

/* =========================
   RDC6445S INPUT PINS
   ========================= */

#define RDC_OUT1_PIN 19   // ES32C14 IN1
#define RDC_OUT2_PIN 18   // ES32C14 IN2

/* =========================
   CNC STATES
   ========================= */

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
 * OUT1:
 * LOW  = RDC is idle / standby
 * HIGH = RDC is not idle
 *
 * OUT2:
 * LOW  = system error / fault
 * HIGH = no system error
 */

bool wasRunning = false;

/*
 * We show DONE briefly after:
 *
 * RUNNING -> IDLE
 *
 * then return to IDLE.
 */
unsigned long doneStartTime = 0;
const unsigned long DONE_DISPLAY_TIME = 2000;


/* =========================
   STATE NAME
   ========================= */

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


/* =========================
   READ RDC STATE
   ========================= */

void updateCNCState()
{
    int out1 = digitalRead(RDC_OUT1_PIN);
    int out2 = digitalRead(RDC_OUT2_PIN);

    /*
     * Active LOW signals
     */

    bool idleSignal  = (out1 == LOW);
    bool faultSignal = (out2 == LOW);


    /* -------------------------
       Highest priority = fault
       ------------------------- */

    if (faultSignal)
    {
        currentState = CNC_FAULT;

        /*
         * Do not interpret a later idle state
         * as successful completion.
         */
        wasRunning = false;

        return;
    }


    /* -------------------------
       DONE state timer
       ------------------------- */

    if (currentState == CNC_DONE)
    {
        if (millis() - doneStartTime < DONE_DISPLAY_TIME)
        {
            return;
        }

        currentState = CNC_IDLE;
    }


    /* -------------------------
       IDLE
       ------------------------- */

    if (idleSignal)
    {
        /*
         * If machine was running before
         * and now returned to idle,
         * interpret that as DONE.
         */

        if (wasRunning)
        {
            currentState = CNC_DONE;
            doneStartTime = millis();

            wasRunning = false;
        }
        else
        {
            currentState = CNC_IDLE;
        }
    }

    /* -------------------------
       NOT IDLE = ACTIVE
       ------------------------- */

    else
    {
        currentState = CNC_RUNNING;

        wasRunning = true;
    }
}


/* =========================
   PRINT STATUS
   ========================= */

void printStateIfChanged()
{
    if (currentState != previousState)
    {
        Serial.println();
        Serial.println("==========================");

        Serial.print("CNC STATE: ");
        Serial.println(stateToString(currentState));

        Serial.println("==========================");

        previousState = currentState;
    }
}


/* =========================
   RAW DEBUG VALUES
   ========================= */

void printRawSignals()
{
    static unsigned long previousPrint = 0;

    if (millis() - previousPrint >= 1000)
    {
        previousPrint = millis();

        int out1 = digitalRead(RDC_OUT1_PIN);
        int out2 = digitalRead(RDC_OUT2_PIN);

        Serial.print("OUT1: ");
        Serial.print(out1);

        Serial.print("   OUT2: ");
        Serial.println(out2);
    }
}


/* =========================
   SETUP
   ========================= */

void setup()
{
    Serial.begin(115200);

    pinMode(RDC_OUT1_PIN, INPUT);
    pinMode(RDC_OUT2_PIN, INPUT);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   RDC6445S CNC STATE MONITOR");
    Serial.println("================================");

    Serial.println();
    Serial.println("Connections:");
    Serial.println("OUT1 -> ES32C14 IN1 -> GPIO19");
    Serial.println("OUT2 -> ES32C14 IN2 -> GPIO18");
    Serial.println("RDC GND -> ES32C14 Input GND");

    Serial.println();
    Serial.println("Waiting for CNC state...");
}


/* =========================
   LOOP
   ========================= */

void loop()
{
    updateCNCState();

    printStateIfChanged();

    /*
     * Keep this during testing.
     * Later we can remove it.
     */
    printRawSignals();

    delay(20);
}