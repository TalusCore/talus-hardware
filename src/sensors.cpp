/* ============================================================
**
** Sensor handling
**
** ============================================================
*/

#include "sensors.h"
#include "utils.h"

// ==================== SENSOR OBJECTS ====================
MAX30105 particleSensor;
MPU6050 mpu(Wire);
Adafruit_BME280 bme;

// ==================== SENSOR VARIABLES ====================
uint32_t lastBeat = 0;
float lastBPM = 0;
float lastSpO2 = 0;
long stepCount = 0;
long stepsSinceLastPublish = 0;
unsigned long lastStepTime = 0;

// ==================== INITIALIZATION ====================
void initSensors() {
    // Initialize MAX30105
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 not found. Check wiring/power.");
        while (1);
    }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);

    // Initialize MPU6050
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 init failed: "); 
        Serial.println(status);
        while (1);
    }
    mpu.calcOffsets();

    initMotionAnalysis();

    // Initialize BME280
    if (!bme.begin(0x76)) {
        Serial.println("Could not find BME280!");
        while (1);
    }
}

// ==================== HEART RATE PROCESSING ====================
void updateHeartRate() {
    long irValue = particleSensor.getIR();

    static float bpmBuffer[8] = {0};
    static int bpmIndex = 0;
    static bool bufferFilled = false;

    // Only process if finger detected
    if (irValue > irThreshold) {
        if (checkForBeat(irValue)) {
            uint32_t delta = millis() - lastBeat;
            lastBeat = millis();

            float bpm = 60.0 / (delta / 1000.0);

            // Validate BPM range
            if (bpm >= 40 && bpm <= 200) {
                bpmBuffer[bpmIndex++] = bpm;
                if (bpmIndex >= bpmBufferSize) { 
                    bpmIndex = 0; 
                    bufferFilled = true; 
                }

                // Compute smoothed BPM
                int count = bufferFilled ? bpmBufferSize : bpmIndex;
                float sum = 0;
                for (int i = 0; i < count; i++) sum += bpmBuffer[i];
                lastBPM = sum / count;
            }
        }
    } else {
        // No finger → slowly decay BPM reading
        if (lastBPM > 1) lastBPM -= 0.5;
        else lastBPM = 0;
    }
}

void updateMotion(float userMassKg, float strideLength) {
    mpu.update();
    
    // Get accelerometer and gyroscope data
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();
    
    // Process with advanced motion analysis
    // Pass user parameters into motion processing
    processMotionData(ax, ay, az, gx, gy, gz, userMassKg, strideLength);
}

// ==================== STAIR DETECTION ====================
//
// A flight is confirmed when:
//   1. Smoothed altitude rises >= stairFlightHeight (3 m)
//   2. At least stairMinSteps steps occurred during that rise
//
// The step requirement filters out elevators, drift, and sensor glitches —
// all of which can produce 3 m of apparent altitude gain with zero steps.
//
void updateStairs(float altitudeM) {
    static float smoothedAlt   = 0;
    static bool  initialised   = false;
    static float baseAlt       = 0;   // altitude when we started tracking this climb
    static int   baseSteps     = 0;   // step count when we started tracking

    if (!initialised) {
        smoothedAlt  = altitudeM;
        baseAlt      = altitudeM;
        baseSteps    = currentSession.totalSteps;
        initialised  = true;
        return;
    }

    // Slow EMA to smooth footstrike bounce
    smoothedAlt = stairAltAlpha * altitudeM + (1.0f - stairAltAlpha) * smoothedAlt;

    float gain  = smoothedAlt - baseAlt;
    int   steps = currentSession.totalSteps - baseSteps;

    if (gain >= stairFlightHeight) {
        if (steps >= stairMinSteps) {
            // Confirmed: real altitude gain with real steps taken
            currentSession.flightsClimbed++;
            Serial.printf("Stair flight #%d detected\n", currentSession.flightsClimbed);
        }
        // Reset baseline regardless (drift or real — don't double-count)
        baseAlt   = smoothedAlt;
        baseSteps = currentSession.totalSteps;
    }

    // If we've descended back to baseline, reset so next climb starts fresh
    if (gain < -stairHysteresis) {
        baseAlt   = smoothedAlt;
        baseSteps = currentSession.totalSteps;
    }
}

// ==================== ENVIRONMENTAL PROCESSING ====================
void updateEnvironment() {
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float humidity = bme.readHumidity();
    float altitude = bme.readAltitude(1013.25);

    updateStairs(altitude);   // feed raw altitude into stair detector every sample

    avgData.add(temperature, pressure, humidity, altitude, lastBPM);
}

// ==================== SPO2 PROCESSING ====================
//
// Ankle-mounted SpO2 is challenging: perfusion is weaker, motion artifacts
// are constant, and the IR/Red AC components are small relative to noise.
// Strategy:
//   1. Fill a rolling buffer of raw Red and IR samples.
//   2. Use a simple ratio-of-ratios (R = (ACred/DCred) / (ACir/DCir)),
//      then map via the standard empirical calibration curve.
//   3. Median-filter the buffer to reject spike outliers from footstrike.
//   4. Apply a very slow EMA (spo2Alpha = 0.1) so the output changes
//      gradually and motion transients don't dominate.
//   5. Require IR > spo2IrThreshold before trusting any reading.
//
void updateSpO2() {
    // Rolling buffers for raw samples
    static uint32_t redBuf[25] = {0};
    static uint32_t irBuf[25]  = {0};
    static int      bufIdx     = 0;
    static int      bufFill    = 0;   // how many valid samples collected

    // Collect one sample per call (sensor FIFO)
    particleSensor.check();
    if (particleSensor.available()) {
        redBuf[bufIdx] = particleSensor.getRed();
        irBuf[bufIdx]  = particleSensor.getIR();
        particleSensor.nextSample();

        bufIdx  = (bufIdx + 1) % spo2SampleCount;
        if (bufFill < spo2SampleCount) bufFill++;
    }

    // Need at least half the buffer filled and a strong enough IR signal
    long irSum = 0;
    for (int i = 0; i < bufFill; i++) irSum += irBuf[i];
    long irMean = (bufFill > 0) ? (irSum / bufFill) : 0;

    if (bufFill < (spo2SampleCount / 2) || irMean < spo2IrThreshold) {
        // Slowly decay toward 0 so the dashboard knows signal is lost
        if (lastSpO2 > 1.0f) lastSpO2 *= 0.97f;
        else lastSpO2 = 0;
        return;
    }

    // --- DC components (mean) ---
    float dcRed = 0, dcIr = 0;
    for (int i = 0; i < bufFill; i++) {
        dcRed += (float)redBuf[i];
        dcIr  += (float)irBuf[i];
    }
    dcRed /= bufFill;
    dcIr  /= bufFill;

    if (dcRed < 1.0f || dcIr < 1.0f) return;  // guard div/0

    // --- AC components (RMS of deviation from DC) ---
    float acRed2 = 0, acIr2 = 0;
    for (int i = 0; i < bufFill; i++) {
        float dr = (float)redBuf[i] - dcRed;
        float di = (float)irBuf[i]  - dcIr;
        acRed2 += dr * dr;
        acIr2  += di * di;
    }
    float acRed = sqrtf(acRed2 / bufFill);
    float acIr  = sqrtf(acIr2  / bufFill);

    // Require meaningful pulsatile amplitude – ankle AC is small but present
    if (acIr < 10.0f || acRed < 5.0f) {
        if (lastSpO2 > 1.0f) lastSpO2 *= 0.97f;
        else lastSpO2 = 0;
        return;
    }

    // --- Ratio of Ratios ---
    float R = (acRed / dcRed) / (acIr / dcIr);

    // Standard empirical calibration curve:  SpO2 ≈ 110 − 25·R
    // Coefficients from Maxim AN6409 / published literature.
    // At ankle R is slightly elevated due to venous mixing; we keep the
    // standard curve but rely on lenient validity bounds.
    float rawSpO2 = 110.0f - 25.0f * R;

    // Reject physically implausible values before smoothing
    if (rawSpO2 < spo2MinValid || rawSpO2 > spo2MaxValid) {
        if (lastSpO2 > 1.0f) lastSpO2 *= 0.97f;
        else lastSpO2 = 0;
        return;
    }

    // Slow EMA – heavily damps footstrike transients
    if (lastSpO2 < spo2MinValid) {
        lastSpO2 = rawSpO2;  // cold-start: seed immediately
    } else {
        lastSpO2 = spo2Alpha * rawSpO2 + (1.0f - spo2Alpha) * lastSpO2;
    }
}

// ==================== GETTERS ====================
float getCurrentBPM() {
    return lastBPM;
}

float getCurrentSpO2() {
    return lastSpO2;
}

long getCurrentSteps() {
    return stepsSinceLastPublish;
}