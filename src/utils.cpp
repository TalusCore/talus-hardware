/* ============================================================
**
** Utility functions and original step detection
**
** ============================================================
*/

#include "utils.h"
#include "sensors.h"
#include "config.h"
#include "motion.h"
#include "metrics.h"
#include <time.h>

// ==================== GLOBAL OBJECTS ====================
Averages avgData;

// ==================== STEP DETECTION VARIABLES ====================
float gravity = 0;
float alpha_step = 0.9;  // Using alpha from config.h conceptually
bool stepHigh = false;

// ==================== AVERAGING IMPLEMENTATION ====================
void Averages::add(float temp, float press, float hum, float alt, float bpm) {
    tempSum += temp;
    pressureSum += press;
    humiditySum += hum;
    altitudeSum += alt;
    bpmSum += bpm;
    count++;
}

void Averages::reset() {
    tempSum = pressureSum = humiditySum = altitudeSum = bpmSum = 0;
    count = 0;
}

void Averages::getAverages(float &temp, float &press, float &hum, float &alt, float &bpm) {
    if (count == 0) { 
        temp = press = hum = alt = bpm = 0; 
        return; 
    }
    temp = tempSum / count;
    press = pressureSum / count;
    hum = humiditySum / count;
    alt = altitudeSum / count;
    bpm = bpmSum / count;
}

// ==================== TIMESTAMP ====================
String getUTCTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "unknown";
    }
    char isoTime[30];
    strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(isoTime);
}

// ==================== STRING ESCAPING ====================
String escapeForC(const String& input) {
    String escaped = "";
    for (size_t i = 0; i < input.length(); i++) {
        char c = input[i];
        if (c == '\"') escaped += "\\\"";
        else if (c == '\\') escaped += "\\\\";
        else escaped += c;
    }
    return escaped;
}

// ==================== SESSION REPORTING ====================
void printSessionReport(float userMassKg, float userHeightM, float strideLength) {
    SessionMetrics session = getSessionMetrics();
    StepEvent lastStepData = getLastStep();
    
    // Get health metrics
    TPIComponents tpi = getTPIComponents();
    TSIComponents tsi = getTSIComponents();
    
    // Get averaged environmental data
    float avgTemp, avgPress, avgHum, avgAlt, avgBPM;
    avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM);

    float durationMin = session.sessionDuration / 60000.0;

    Serial.println("\n╔══════════════════════════════════╗");
    // Serial.println("║          SESSION REPORT            ║");

    // // --- User Params (matches JSON: userParams) ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.println("║ USER PARAMS                        ║");
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.printf("║ Mass:            %6.1f kg       ║\n", userMassKg);
    // Serial.printf("║ Height:          %6.2f m        ║\n", userHeightM);
    // Serial.printf("║ Stride Length:   %6.2f m        ║\n", strideLength);

    // --- Environmental Stats (matches JSON: stats) ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.println("║ ENVIRONMENT (stats)                ║");
    // Serial.println("╠════════════════════════════════════╣");
    Serial.printf("║ Temperature:     %6.1f C        ║\n", avgTemp);
    Serial.printf("║ Pressure:        %6.1f hPa      ║\n", avgPress);
    Serial.printf("║ Humidity:        %6.1f %%        ║\n", avgHum);
    Serial.printf("║ Altitude:        %6.1f m        ║\n", avgAlt);
    Serial.printf("║ BPM:             %6.1f          ║\n", avgBPM);
    Serial.printf("║ SpO2:            %6.1f %%        ║\n", getCurrentSpO2());

    // --- Motion (matches JSON: motion) ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.println("║ MOTION                             ║");
    // Serial.println("╠════════════════════════════════════╣");
    Serial.printf("║ Steps:           %6d          ║\n", session.totalSteps);
    Serial.printf("║ Flights climbed: %6d          ║\n", session.flightsClimbed);
    // Serial.printf("║ Distance:        %6.1f m        ║\n", session.totalDistance);
    // Serial.printf("║ Avg Cadence:     %6.1f spm      ║\n", session.avgCadence);
    // Serial.printf("║ Max Force:       %6.1f N        ║\n", session.maxForce);
    // Serial.printf("║ Max Power:       %6.1f W        ║\n", session.maxPower);
    // Serial.printf("║ Avg Force:       %6.1f N        ║\n", session.avgForce);
    // Serial.printf("║ Avg Power:       %6.1f W        ║\n", session.avgPower);

    // --- Last Step (matches JSON: lastStep) ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.println("║ LAST STEP                          ║");
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.printf("║ Force:           %6.1f N        ║\n", lastStepData.impactForce);
    // Serial.printf("║ Power:           %6.1f W        ║\n", lastStepData.estimatedPower);
    // Serial.printf("║ Velocity:        %6.2f m/s      ║\n", lastStepData.verticalVelocity);

    // --- Health Metrics: TPI (matches JSON: healthMetrics.tpi) ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.println("║ HEALTH METRICS                     ║");
    // Serial.println("╠════════════════════════════════════╣");
    Serial.printf("║ TPI Score:       %6.1f / 100    ║\n", tpi.finalScore);
    // Serial.printf("║   Mechanical:    %6.1f          ║\n", tpi.mechanicalOutput);
    // Serial.printf("║   Cardio:        %6.1f          ║\n", tpi.cardioStrain);
    // Serial.printf("║   Efficiency:    %6.1f          ║\n", tpi.efficiency);
    // Serial.printf("║   Resistance:    %6.1f          ║\n", tpi.fatigueResistance);
    // Serial.println("╠════════════════════════════════════╣");

    // --- Session Timing ---
    // Serial.println("╠════════════════════════════════════╣");
    // Serial.printf("║ Duration:        %6.1f min      ║\n", durationMin);
    // Serial.printf("║ Avg Step Time:   %6.0f ms       ║\n", session.avgStepTime);
     Serial.println("╚══════════════════════════════════╝\n");
}