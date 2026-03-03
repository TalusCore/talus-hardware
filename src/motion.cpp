// src/motion.cpp

/* ============================================================
**
** Motion file handling step determination, velocity, and acceleration insights
**
** ============================================================
*/

#include "motion.h"
#include "config.h"
#include <math.h>

// ==================== GLOBAL VARIABLES ====================
SessionMetrics currentSession = {0};
StepEvent lastStep = {0};
StepPhase currentPhase = SWING_PHASE;

// Step detection variables
float verticalAccelBuffer[10] = {0};
int bufferIndex = 0;
float peakAccelThisStep = 0;
float minAccelThisStep = 0;
bool stepInProgress = false;

// Velocity estimation
float currentVerticalVelocity = 0;

// Step timing
unsigned long lastStepTimestamp = 0;
unsigned long currentStepStartTime = 0;

extern long stepsSinceLastPublish;

// Constants
const float GRAVITY_BASELINE = 1.0;       // g - expected gravity
const float NOISE_THRESHOLD = 0.1;        // ignore small fluctuations
const float MAX_STEP_TIME = 2000;         // ms - maximum (very slow walking)

// ==================== INITIALIZATION ====================

void initMotionAnalysis() {
    resetSession();
    
    // Initialize buffers
    for (int i = 0; i < 10; i++) {
        verticalAccelBuffer[i] = GRAVITY_BASELINE;
    }
    
    currentPhase = SWING_PHASE;
    bufferIndex = 0;
    stepInProgress = false;
    
    Serial.println("Motion analysis initialized");
}

void resetSession() {
    currentSession.maxForce = 0;
    currentSession.maxPower = 0;
    currentSession.avgForce = 0;
    currentSession.avgPower = 0;
    currentSession.totalSteps = 0;
    currentSession.totalDistance = 0;
    currentSession.avgCadence = 0;
    currentSession.avgStepTime = 0;
    currentSession.sessionStart = millis();
    currentSession.sessionDuration = 0;
    currentSession.sumForce = 0;
    currentSession.sumPower = 0;
    currentSession.sumStepTime = 0;
    currentSession.flightsClimbed = 0;

    lastStepTimestamp = 0;
    currentVerticalVelocity = 0;
    
    Serial.println("Session reset");
}

// ==================== MAIN PROCESSING ====================

void processMotionData(float ax, float ay, float az, 
                       float gx, float gy, float gz,
                       float userMassKg, float strideLength) {
    // Calculate magnitude
    float magnitude = sqrt(ax * ax + ay * ay + az * az);
    
    // Get vertical component (assuming ankle orientation)
    // For ankle: Y-axis is typically vertical when standing
    float verticalAcc = getVerticalAcceleration(ax, ay, az);
    
    // Smooth the acceleration
    float smoothedAcc = smoothAcceleration(verticalAcc);
    
    // Detect steps with enhanced algorithm
    //bool stepDetected = detectStepEnhanced(smoothedAcc, magnitude);
    bool stepDetected = detectStep(ax,ay,az);
    
    if (stepDetected) {
        // Calculate step metrics
        float stepTime = (millis() - lastStepTimestamp);
        float velocity = estimateVelocity(stepTime, strideLength);
        float force = calculateForce(peakAccelThisStep * gravityMs2, userMassKg);
        float power = calculatePower(force, velocity);
        
        // Create step event
        StepEvent step;
        step.peakAcceleration = peakAccelThisStep * gravityMs2;
        step.impactForce = force;
        step.estimatedPower = power;
        step.timestamp = millis();
        step.stepTime = stepTime;
        step.verticalVelocity = velocity;
        
        lastStep = step;
        lastStepTimestamp = millis();
        
        // Update session statistics
        updateSessionStats(step, strideLength);
        stepsSinceLastPublish++;
        
        // Debug output
        //Serial.printf("Step #%d | Force: %.1fN | Power: %.1fW | Cadence: %.1f spm\n",
                    //  currentSession.totalSteps,
                    //  force,
                    //  power,
                    //  currentSession.avgCadence);
    }
    
    // Update velocity estimation (simple integration with drift correction)
    float dt = 1.0 / 50.0;  // 50 Hz sampling rate
    currentVerticalVelocity += (smoothedAcc - GRAVITY_BASELINE) * gravityMs2 * dt;
    
    // Apply velocity damping to prevent drift
    currentVerticalVelocity *= 0.98;
    
    // Zero velocity update (ZVU) - when foot is on ground, velocity should be ~0
    if (currentPhase == STANCE_PHASE) {
        currentVerticalVelocity *= 0.8;  // Aggressively damp during stance
    }
}

// ==================== STEP DETECTION ====================
// Step detection variables
float gravity = 0;
bool stepHigh = false;
const unsigned long minStepIntervalStep = 300; // ms
unsigned long lastStepTimeStep = 0;

bool detectStep(float ax, float ay, float az) {
    float mag = sqrt(ax * ax + ay * ay + az * az);

    // Remove gravity (high-pass filter)
    gravity = alpha * gravity + (1 - alpha) * mag;
    float filtered = mag - gravity;

    // Adaptive threshold
    float noiseLevel = fabs(filtered) * 0.2;
    float threshold = dynamicThreshold + noiseLevel;

    unsigned long now = millis();

    if (!stepHigh && filtered > threshold) {
        stepHigh = true;
        peakAccelThisStep = mag;         // begin tracking peak for this step
    } else if (stepHigh) {
        if (mag > peakAccelThisStep) peakAccelThisStep = mag;  // update peak during swing

        if (filtered < 0) {
            if (now - lastStepTimeStep > minStepIntervalStep) {
                lastStepTimeStep = now;
                // peakAccelThisStep holds the peak g value for processMotionData to use
                stepHigh = false;
                return true;
            }
            stepHigh = false;
        }
    }
    return false;
}

// ==================== ENHANCED STEP DETECTION ====================

// bool detectStepEnhanced(float verticalAcc, float magnitude) {
//     unsigned long now = millis();
//     bool stepDetected = false;
    
//     // Track peak and minimum during potential step
//     if (stepInProgress) {
//         if (verticalAcc > peakAccelThisStep) {
//             peakAccelThisStep = verticalAcc;
//         }
//         if (verticalAcc < minAccelThisStep) {
//             minAccelThisStep = verticalAcc;
//         }
//     }
    
//     // State machine for step detection
//     switch (currentPhase) {
//         case SWING_PHASE:
//             // Detect foot leaving ground (low acceleration)
//             if (magnitude < (GRAVITY_BASELINE - swingThreshold)) {
//                 currentPhase = LOADING_PHASE;
//                 stepInProgress = true;
//                 peakAccelThisStep = verticalAcc;
//                 minAccelThisStep = verticalAcc;
//                 currentStepStartTime = now;
//             }
//             break;
            
//         case LOADING_PHASE:
//             // Detect impact (high acceleration spike)
//             if (magnitude > (GRAVITY_BASELINE + impactThreshold)) {
//                 // Check minimum time between steps
//                 if (now - lastStepTimestamp >= minStepInterval) {
//                     // Valid step detected!
//                     stepDetected = true;
//                     currentPhase = STANCE_PHASE;
//                 }
//             }
            
//             // Timeout if no impact detected
//             if (now - currentStepStartTime > MAX_STEP_TIME) {
//                 currentPhase = SWING_PHASE;
//                 stepInProgress = false;
//             }
//             break;
            
//         case STANCE_PHASE:
//             // Wait for foot to settle, then return to swing
//             if (magnitude < (GRAVITY_BASELINE + 0.3)) {
//                 currentPhase = SWING_PHASE;
//                 stepInProgress = false;
//             }
//             break;
//     }
    
//     return stepDetected;
// }

// ==================== FORCE CALCULATION ====================

float calculateForce(float acceleration, float massKg) {
    // F = ma
    // acceleration already in m/s²
    float force = massKg * fabs(acceleration);
    
    return force;
}

// ==================== POWER CALCULATION ====================

float calculatePower(float force, float velocity) {
    // P = F × v
    float power = force * fabs(velocity);
    
    return power;
}

// ==================== VELOCITY ESTIMATION ====================

float estimateVelocity(float stepTime, float strideLength) {
    // Method: From stride length and step time
    // v = distance / time
    // Assume each step covers stride_length/2 (one leg)
    
    if (stepTime <= 0) return 0;
    
    float stepDistance = strideLength / 2.0;  // Distance per step
    float stepTimeSeconds = stepTime / 1000.0;
    float velocity = stepDistance / stepTimeSeconds;
    
    // Sanity check (human walking: 0.5-2.5 m/s, running: 2.5-6 m/s)
    if (velocity > 7.0) velocity = 7.0;
    if (velocity < 0.3) velocity = 0.3;
    
    return velocity;
}

// ==================== UTILITIES ====================

float getVerticalAcceleration(float ax, float ay, float az) {
    // For ankle-mounted sensor with standard orientation:
    // Y-axis is typically vertical when standing
    // Adjust based on your actual mounting orientation
    
    return ay;
}

float smoothAcceleration(float newValue) {
    // Moving average filter
    verticalAccelBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % 10;
    
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += verticalAccelBuffer[i];
    }
    
    return sum / 10.0;
}

void updateSessionStats(const StepEvent& step, float strideLength) {
    // Increment step count
    currentSession.totalSteps++;
    
    // Update max values
    if (step.impactForce > currentSession.maxForce) {
        currentSession.maxForce = step.impactForce;
    }
    if (step.estimatedPower > currentSession.maxPower) {
        currentSession.maxPower = step.estimatedPower;
    }
    
    // Update running sums for averages
    currentSession.sumForce += step.impactForce;
    currentSession.sumPower += step.estimatedPower;
    currentSession.sumStepTime += step.stepTime;
    
    // Calculate averages
    currentSession.avgForce = currentSession.sumForce / currentSession.totalSteps;
    currentSession.avgPower = currentSession.sumPower / currentSession.totalSteps;
    currentSession.avgStepTime = currentSession.sumStepTime / currentSession.totalSteps;
    
    // Update distance (stride length per step)
    currentSession.totalDistance += (strideLength / 2.0);
    
    // Update cadence (steps per minute)
    currentSession.sessionDuration = millis() - currentSession.sessionStart;
    if (currentSession.sessionDuration > 0) {
        float minutes = currentSession.sessionDuration / 60000.0;
        currentSession.avgCadence = currentSession.totalSteps / minutes;
    }
}

// ==================== GETTERS ====================

SessionMetrics getSessionMetrics() {
    return currentSession;
}

StepEvent getLastStep() {
    return lastStep;
}