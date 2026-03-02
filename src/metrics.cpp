// src/metrics.cpp

/* ============================================================
**
** Talus health metrics: Talus Performance Index or Talus Strain Index
**
** ============================================================
*/

#include "metrics.h"
#include "motion.h"
#include <math.h>

// ==================== GLOBAL VARIABLES ====================
MetricsState metricsState = {0};
TPIComponents currentTPI = {0};
TSIComponents currentTSI = {0};

// Constants
const float ZONE_1_MAX = 0.60;  // 60% max HR
const float ZONE_2_MAX = 0.70;  // 70% max HR
const float ZONE_3_MAX = 0.80;  // 80% max HR
const float ZONE_4_MAX = 0.90;  // 90% max HR

const float COMFORT_TEMP_MIN = 18.0;  // °C
const float COMFORT_TEMP_MAX = 24.0;  // °C
const float COMFORT_HUMIDITY_MAX = 60.0;  // %

// ==================== INITIALIZATION ====================

void initMetrics() {
    resetMetrics();
    Serial.println("Health metrics initialized");
}

void resetMetrics() {
    metricsState.initialPower = 0;
    metricsState.initialCadence = 0;
    metricsState.windowStart = millis();
    
    metricsState.timeInZone1 = 0;
    metricsState.timeInZone2 = 0;
    metricsState.timeInZone3 = 0;
    metricsState.timeInZone4 = 0;
    metricsState.timeInZone5 = 0;
    
    metricsState.initialBPM = 0;
    metricsState.bpmDrift = 0;
    metricsState.powerDecline = 0;
    
    metricsState.baselineTemp = 0;
    metricsState.baselineHumidity = 0;
    
    metricsState.stepsLastMinute = 0;
    metricsState.lastMinuteStart = millis();
    
    currentTPI = {0};
    currentTSI = {0};
}

// ==================== MAIN CALCULATION ====================

void calculateMetrics(float bpm, float temperature, float humidity,
                     float userMassKg, float maxHR) {
    SessionMetrics session = getSessionMetrics();
    
    // Initialize baselines on first calculation
    if (metricsState.windowStart == 0 || session.totalSteps == 1) {
        metricsState.windowStart = millis();
        metricsState.initialBPM = bpm;
        metricsState.initialPower = session.avgPower;
        metricsState.initialCadence = session.avgCadence;
        metricsState.baselineTemp = temperature;
        metricsState.baselineHumidity = humidity;
    }
    
    // Track HR zones (accumulate time in each zone)
    float hrPercent = bpm / maxHR;
    float timeSinceLastUpdate = 5.0;  // 5 seconds (publish interval)
    
    if (hrPercent < ZONE_1_MAX) {
        metricsState.timeInZone1 += timeSinceLastUpdate;
    } else if (hrPercent < ZONE_2_MAX) {
        metricsState.timeInZone2 += timeSinceLastUpdate;
    } else if (hrPercent < ZONE_3_MAX) {
        metricsState.timeInZone3 += timeSinceLastUpdate;
    } else if (hrPercent < ZONE_4_MAX) {
        metricsState.timeInZone4 += timeSinceLastUpdate;
    } else {
        metricsState.timeInZone5 += timeSinceLastUpdate;
    }
    
    // Track fatigue indicators
    metricsState.bpmDrift = bpm - metricsState.initialBPM;
    if (metricsState.initialPower > 0) {
        metricsState.powerDecline = 
            (metricsState.initialPower - session.avgPower) / metricsState.initialPower;
    }
    
    // Track step density (steps per minute)
    if (millis() - metricsState.lastMinuteStart >= 60000) {
        metricsState.stepsLastMinute = session.totalSteps;
        metricsState.lastMinuteStart = millis();
    }
    
    // Calculate both metrics
    currentTPI = calculateTPI();
}

// ==================== TPI CALCULATION (SPORTS) ====================

TPIComponents calculateTPI() {
    TPIComponents tpi;
    
    // Component 1: Mechanical Output (40%)
    tpi.mechanicalOutput = calculateMechanicalOutput();
    
    // Component 2: Cardio Strain (30%)
    SessionMetrics session = getSessionMetrics();
    // Use average cadence as proxy for current BPM if not available
    float estimatedMaxHR = 220 - 30; 
    tpi.cardioStrain = calculateCardioStrain(75.0, estimatedMaxHR);  // Placeholder BPM
    
    // Component 3: Efficiency (20%)
    tpi.efficiency = calculateEfficiency();
    
    // Component 4: Fatigue Resistance (10%)
    tpi.fatigueResistance = calculateFatigueResistance();
    
    // Weighted final score
    tpi.finalScore = 
        0.40 * tpi.mechanicalOutput +
        0.30 * tpi.cardioStrain +
        0.20 * tpi.efficiency +
        0.10 * tpi.fatigueResistance;
    
    tpi.finalScore = clamp(tpi.finalScore, 0, 100);
    
    return tpi;
}

// ==================== HELPER FUNCTIONS ====================

float calculateMechanicalOutput() {
    SessionMetrics session = getSessionMetrics();
    
    // Combine multiple mechanical indicators
    float powerScore = normalize(session.avgPower, 0, 300);  // 0-300W typical
    float forceScore = normalize(session.avgForce, 500, 1500);  // 500-1500N typical
    float cadenceScore = normalize(session.avgCadence, 60, 180);  // 60-180 spm
    
    // Weighted combination
    float mechanicalOutput = 
        0.4 * powerScore +
        0.3 * forceScore +
        0.3 * cadenceScore;
    
    return mechanicalOutput * 100.0;
}

float calculateCardioStrain(float bpm, float maxHR) {
    // Weighted time in HR zones
    float totalTime = metricsState.timeInZone1 +
                      metricsState.timeInZone2 +
                      metricsState.timeInZone3 +
                      metricsState.timeInZone4 +
                      metricsState.timeInZone5;
    
    if (totalTime == 0) return 0;
    
    float strain = 
        (metricsState.timeInZone1 / totalTime) * 20 +   // Zone 1: 20 points
        (metricsState.timeInZone2 / totalTime) * 40 +   // Zone 2: 40 points
        (metricsState.timeInZone3 / totalTime) * 60 +   // Zone 3: 60 points
        (metricsState.timeInZone4 / totalTime) * 80 +   // Zone 4: 80 points
        (metricsState.timeInZone5 / totalTime) * 100;   // Zone 5: 100 points
    
    return clamp(strain, 0, 100);
}

float calculateEfficiency() {
    // Efficiency = Mechanical Output / Cardio Strain
    // High efficiency = good output with low HR
    
    float mechanicalOutput = calculateMechanicalOutput();
    SessionMetrics session = getSessionMetrics();
    
    // Estimate cardio effort from cadence (proxy for HR)
    float cardioEffort = normalize(session.avgCadence, 60, 180) * 100;
    
    if (cardioEffort == 0) return 50;  // Neutral if no data
    
    float efficiency = (mechanicalOutput / cardioEffort) * 100;
    
    return clamp(efficiency, 0, 100);
}

float calculateFatigueResistance() {
    // Higher score = less fatigue = better
    
    SessionMetrics session = getSessionMetrics();
    float durationMinutes = session.sessionDuration / 60000.0;
    
    if (durationMinutes < 5) return 100;  // Too early to measure
    
    // Fatigue indicators (lower = more fatigue)
    float bpmDriftPenalty = normalize(fabs(metricsState.bpmDrift), 0, 30);  // 0-30 bpm drift
    float powerDeclinePenalty = normalize(fabs(metricsState.powerDecline), 0, 0.5);  // 0-50% decline
    
    // Invert penalties (higher = better)
    float resistance = 100 - ((bpmDriftPenalty + powerDeclinePenalty) / 2.0 * 100);
    
    return clamp(resistance, 0, 100);
}

// ==================== UTILITY FUNCTIONS ====================

float normalize(float value, float min, float max) {
    if (max <= min) return 0;
    float normalized = (value - min) / (max - min);
    return clamp(normalized, 0, 1);
}

float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ==================== GETTER ====================

TPIComponents getTPIComponents() {
    return currentTPI;
}