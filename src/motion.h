// src/motion.h

/* ============================================================
**
** Motion file handling step determination, velocity, and acceleration insights
**
** ============================================================
*/

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// ==================== DATA STRUCTURES ====================

// Single step event
struct StepEvent {
    float peakAcceleration;    // Peak vertical acc (m/s²)
    float impactForce;         // Impact force (N)
    float estimatedPower;      // Estimated power (W)
    unsigned long timestamp;   // When step occurred
    float stepTime;            // Time since last step (ms)
    float verticalVelocity;    // Estimated velocity (m/s)
};

// Session statistics
struct SessionMetrics {
    // Force & Power
    float maxForce;            // N
    float maxPower;            // W
    float avgForce;            // N
    float avgPower;            // W
    
    // Steps & Gait
    int totalSteps;
    float totalDistance;       // m
    float avgCadence;          // steps/min
    float avgStepTime;         // ms
    
    // Timing
    unsigned long sessionStart;
    unsigned long sessionDuration;  // ms
    
    // Stairs
    int flightsClimbed;            // number of ~3 m altitude gains detected

    // Running totals for averaging
    float sumForce;
    float sumPower;
    float sumStepTime;
};

// Step detection state
enum StepPhase {
    SWING_PHASE,      // Foot in air
    STANCE_PHASE,     // Foot on ground
    LOADING_PHASE     // Impact/loading
};

// ==================== GLOBAL VARIABLES ====================
extern SessionMetrics currentSession;
extern StepEvent lastStep;
extern StepPhase currentPhase;

// Enhanced step detection variables
extern float verticalAccelBuffer[10];
extern int bufferIndex;
extern float peakAccelThisStep;
extern float minAccelThisStep;
extern bool stepInProgress;

// Velocity estimation
extern float currentVerticalVelocity;

// ==================== FUNCTION DECLARATIONS ====================

// Initialization
void initMotionAnalysis();
void resetSession();

// Main processing (takes user mass and stride length as parameters)
void processMotionData(float ax, float ay, float az, 
                       float gx, float gy, float gz,
                       float userMassKg, float strideLength);

// Step detection
bool detectStep(float ax, float ay, float az);
bool detectStepEnhanced(float verticalAcc, float magnitude);

// Force & Power calculations
float calculateForce(float acceleration, float massKg);
float calculatePower(float force, float velocity);
float estimateVelocity(float stepTime, float strideLength);

// Getters
SessionMetrics getSessionMetrics();
StepEvent getLastStep();

// Utilities
float getVerticalAcceleration(float ax, float ay, float az);
float smoothAcceleration(float newValue);
void updateSessionStats(const StepEvent& step, float strideLength);

#endif