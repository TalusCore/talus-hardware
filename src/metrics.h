// src/metrics.h

/* ============================================================
**
** Talus health metrics: Talus Performance Index or Talus Strain Index
**
** ============================================================
*/

#ifndef METRICS_H
#define METRICS_H

#include <Arduino.h>
#include "motion.h"

// ==================== DATA STRUCTURES ====================

// TPI (Sports) components
struct TPIComponents {
    float mechanicalOutput;    // 0-100
    float cardioStrain;        // 0-100
    float efficiency;          // 0-100
    float fatigueResistance;   // 0-100
    float finalScore;          // 0-100 (weighted)
};

// TSI (Workplace) components
struct TSIComponents {
    float sustainedLoad;       // 0-100
    float environmentalStress; // 0-100
    float cumulativeFatigue;   // 0-100
    float overexertionRisk;    // 0-100
    float finalScore;          // 0-100
};

// Tracking state for metrics
struct MetricsState {
    // For efficiency calculation
    float initialPower;
    float initialCadence;
    unsigned long windowStart;
    
    // HR zone time tracking (seconds)
    float timeInZone1;  // Recovery (<60% max HR)
    float timeInZone2;  // Fat burn (60-70%)
    float timeInZone3;  // Cardio (70-80%)
    float timeInZone4;  // Peak (80-90%)
    float timeInZone5;  // Max (>90%)
    
    // Fatigue tracking
    float initialBPM;
    float bpmDrift;
    float powerDecline;
    
    // Environmental baseline
    float baselineTemp;
    float baselineHumidity;
    
    // Step density tracking
    int stepsLastMinute;
    unsigned long lastMinuteStart;
};

// ==================== GLOBAL VARIABLES ====================
extern MetricsState metricsState;
extern TPIComponents currentTPI;
extern TSIComponents currentTSI;

// ==================== FUNCTION DECLARATIONS ====================

// Initialization
void initMetrics();
void resetMetrics();

// Main calculation (called every publish interval)
void calculateMetrics(float bpm, float temperature, float humidity,
                     float userMassKg, float maxHR);

// Individual metric calculations
TPIComponents calculateTPI();
TSIComponents calculateTSI(float temperature, float humidity);

// Helper functions
float calculateMechanicalOutput();
float calculateCardioStrain(float bpm, float maxHR);
float calculateEfficiency();
float calculateFatigueResistance();
float calculateSustainedLoad(float bpm, float maxHR);
float calculateEnvironmentalStress(float temp, float humidity);
float calculateCumulativeFatigue();

// Normalization utilities
float normalize(float value, float min, float max);
float clamp(float value, float min, float max);

// Getters
TPIComponents getTPIComponents();
TSIComponents getTSIComponents();

#endif