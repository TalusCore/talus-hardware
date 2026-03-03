/* ============================================================
**
**  Sensor Handling
**
** ============================================================
*/

#include "sensors.h"
#include "utils.h"
#include <math.h>

// ============================================================
// CONSTANTS
// ============================================================

// ---- Optical ----
constexpr int   BPM_BUFFER_SIZE      = 8;
constexpr int   SPO2_BUFFER_SIZE     = 25;

constexpr float MIN_BPM              = 40.0f;
constexpr float MAX_BPM              = 200.0f;
constexpr uint32_t MIN_BEAT_INTERVAL = 300;      // ms

constexpr float MIN_AC_IR            = 10.0f;
constexpr float MIN_AC_RED           = 5.0f;

constexpr float SPO2_CAL_A           = 110.0f;
constexpr float SPO2_CAL_B           = 25.0f;

// ---- Environmental ----
constexpr float SEA_LEVEL_HPA        = 1013.25f;

// ---- Stair Detection ----
constexpr float STAIR_ALT_ALPHA      = 0.10f;   // smoothing factor
constexpr float STAIR_FLIGHT_HEIGHT  = 3.0f;    // meters
constexpr int   STAIR_MIN_STEPS      = 8;       // minimum steps
constexpr float STAIR_HYSTERESIS     = 1.0f;    // meters

// ============================================================
// SENSOR OBJECTS
// ============================================================

MAX30105 particleSensor;
MPU6050  mpu(Wire);
Adafruit_BME280 bme;

// ============================================================
// STATE STRUCT
// ============================================================

struct SensorState {
    float     bpm            = 0.0f;
    float     spo2           = 0.0f;
    uint32_t  lastBeatMs     = 0;

    bool max3010xOk = false;
    bool mpuOk      = false;
    bool bmeOk      = false;
};

static SensorState state;

uint32_t lastBeat = 0;
float lastBPM = 0;
float lastSpO2 = 0;
long stepCount = 0;
long stepsSinceLastPublish = 0;
long flightsSinceLastPublish = 0;
unsigned long lastStepTime = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

static void processHeartRate(uint32_t irValue);
static void processSpO2(uint32_t red, uint32_t ir);
// static void updateStairs(float altitudeM);

// ============================================================
// INITIALIZATION
// ============================================================

void initSensors()
{
    // ---- MAX3010x ----
    if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(0x0A);
        particleSensor.setPulseAmplitudeGreen(0);
        state.max3010xOk = true;
    } else {
        Serial.println("MAX3010x not detected.");
    }

    // ---- MPU6050 ----
    byte status = mpu.begin();
    if (status == 0) {
        mpu.calcOffsets();
        state.mpuOk = true;
    } else {
        Serial.print("MPU6050 init failed: ");
        Serial.println(status);
    }

    initMotionAnalysis();

    // ---- BME280 ----
    if (bme.begin(0x76)) {
        state.bmeOk = true;
    } else {
        Serial.println("BME280 not found.");
    }
}

// ============================================================
// OPTICAL UPDATE (Shared FIFO Read)
// ============================================================

void updateOpticalSensors()
{
    if (!state.max3010xOk) return;

    particleSensor.check();

    while (particleSensor.available()) {

        uint32_t red = particleSensor.getRed();
        uint32_t ir  = particleSensor.getIR();
        particleSensor.nextSample();

        processHeartRate(ir);
        processSpO2(red, ir);
    }
}

// ============================================================
// HEART RATE
// ============================================================

static void processHeartRate(uint32_t irValue)
{
    static float bpmBuffer[BPM_BUFFER_SIZE] = {0};
    static int   index  = 0;
    static bool  filled = false;

    // No finger detected
    if (irValue <= irThreshold) {
        state.bpm = (state.bpm > 1.0f) ? state.bpm - 0.5f : 0.0f;
        return;
    }

    if (!checkForBeat(irValue)) return;

    uint32_t now   = millis();
    uint32_t delta = now - state.lastBeatMs;
    state.lastBeatMs = now;

    if (delta < MIN_BEAT_INTERVAL) return;

    float bpm = 60000.0f / delta;

    if (bpm < MIN_BPM || bpm > MAX_BPM) return;

    bpmBuffer[index++] = bpm;

    if (index >= BPM_BUFFER_SIZE) {
        index = 0;
        filled = true;
    }

    int count = filled ? BPM_BUFFER_SIZE : index;
    float sum = 0;

    for (int i = 0; i < count; i++)
        sum += bpmBuffer[i];

    state.bpm = sum / count;
}

// ============================================================
// SPO2
// ============================================================

static void processSpO2(uint32_t red, uint32_t ir)
{
    static uint32_t redBuf[SPO2_BUFFER_SIZE] = {0};
    static uint32_t irBuf[SPO2_BUFFER_SIZE]  = {0};
    static int index  = 0;
    static int filled = 0;

    redBuf[index] = red;
    irBuf[index]  = ir;

    index = (index + 1) % SPO2_BUFFER_SIZE;
    if (filled < SPO2_BUFFER_SIZE) filled++;

    if (filled < SPO2_BUFFER_SIZE / 2) return;

    float dcRed = 0, dcIr = 0;
    for (int i = 0; i < filled; i++) {
        dcRed += redBuf[i];
        dcIr  += irBuf[i];
    }

    dcRed /= filled;
    dcIr  /= filled;

    if (dcRed < 1.0f || dcIr < 1.0f) return;

    float acRed2 = 0, acIr2 = 0;

    for (int i = 0; i < filled; i++) {
        float dr = redBuf[i] - dcRed;
        float di = irBuf[i]  - dcIr;
        acRed2 += dr * dr;
        acIr2  += di * di;
    }

    float acRed = sqrtf(acRed2 / filled);
    float acIr  = sqrtf(acIr2  / filled);

    if (acIr < MIN_AC_IR || acRed < MIN_AC_RED) {
        state.spo2 = (state.spo2 > 1.0f) ? state.spo2 * 0.97f : 0.0f;
        return;
    }

    float R = (acRed / dcRed) / (acIr / dcIr);
    float rawSpO2 = SPO2_CAL_A - SPO2_CAL_B * R;

    if (rawSpO2 < spo2MinValid || rawSpO2 > spo2MaxValid) {
        state.spo2 = (state.spo2 > 1.0f) ? state.spo2 * 0.97f : 0.0f;
        return;
    }

    if (state.spo2 < spo2MinValid)
        state.spo2 = rawSpO2;
    else
        state.spo2 = spo2Alpha * rawSpO2 +
                     (1.0f - spo2Alpha) * state.spo2;
}

// ============================================================
// MOTION
// ============================================================

void updateMotion(float userMassKg, float strideLength)
{
    if (!state.mpuOk) return;

    mpu.update();

    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();

    processMotionData(ax, ay, az,
                      gx, gy, gz,
                      userMassKg,
                      strideLength);
}

// ============================================================
// STAIR DETECTION
// ============================================================

void updateStairs(float altitudeM)
{
    static float smoothedAlt = 0;
    static bool  initialized = false;
    static float baseAlt     = 0;
    static int   baseSteps   = 0;

    if (!initialized) {
        smoothedAlt = altitudeM;
        baseAlt     = altitudeM;
        baseSteps   = currentSession.totalSteps;
        initialized = true;
        return;
    }

    // EMA smoothing
    smoothedAlt = STAIR_ALT_ALPHA * altitudeM +
                  (1.0f - STAIR_ALT_ALPHA) * smoothedAlt;

    float gain  = smoothedAlt - baseAlt;
    int steps   = currentSession.totalSteps - baseSteps;

    if (gain >= STAIR_FLIGHT_HEIGHT) {

        if (steps >= STAIR_MIN_STEPS) {
            currentSession.flightsClimbed++;
            flightsSinceLastPublish++;
            Serial.printf("Stair flight #%d detected\n",
                          currentSession.flightsClimbed);
        }

        baseAlt   = smoothedAlt;
        baseSteps = currentSession.totalSteps;
    }

    if (gain < -STAIR_HYSTERESIS) {
        baseAlt   = smoothedAlt;
        baseSteps = currentSession.totalSteps;
    }
}

// ============================================================
// ENVIRONMENT
// ============================================================

void updateEnvironment()
{
    if (!state.bmeOk) return;

    float temperature = bme.readTemperature();
    float pressure    = bme.readPressure() / 100.0f;
    float humidity    = bme.readHumidity();
    float altitude    = bme.readAltitude(SEA_LEVEL_HPA);

    updateStairs(altitude);

    avgData.add(temperature,
                pressure,
                humidity,
                altitude,
                state.bpm,
                state.spo2);
}

// ============================================================
// GETTERS
// ============================================================

float getCurrentBPM()   { return state.bpm;  }
float getCurrentSpO2()  { return state.spo2; }

bool isMax3010xHealthy(){ return state.max3010xOk; }
bool isMPUHealthy()     { return state.mpuOk; }
bool isBMEHealthy()     { return state.bmeOk; }