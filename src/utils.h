/* ============================================================
**
** Utility functions and original step detection
**
** ============================================================
*/

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "motion.h"

// ==================== AVERAGING STRUCTURE ====================
struct Averages {
    float tempSum;
    float pressureSum;
    float humiditySum;
    float altitudeSum;
    float bpmSum;
    float spo2Sum;
    int count;

    void add(float temp, float press, float hum, float alt, float bpm, float spo2);
    void reset();
    void getAverages(float &temp, float &press, float &hum, float &alt, float &bpm, float &spo2);
};

extern Averages avgData;

// ==================== STEP DETECTION ====================
// Step detection state variables
extern float gravity;
extern float alpha_step;  // Renamed to avoid conflict with config.h alpha
extern bool stepHigh;

// ==================== FUNCTION DECLARATIONS ====================
String getUTCTimestamp();
String escapeForC(const String& input);
void printSessionReport(float userMassKg, float userHeightM, float strideLength);

#endif