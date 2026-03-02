/* ============================================================
**
** Sensor handling
**
** ============================================================
*/

#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h"
#include "motion.h" 

// ==================== SENSOR OBJECTS ====================
extern MAX30105 particleSensor;
extern MPU6050 mpu;
extern Adafruit_BME280 bme;

// ==================== SENSOR VARIABLES ====================
extern uint32_t lastBeat;
extern float lastBPM;
extern float lastSpO2;          // smoothed SpO2 % (0 = no valid reading)
extern long stepCount;
extern long stepsSinceLastPublish;
extern unsigned long lastStepTime;

// ==================== FUNCTION DECLARATIONS ====================
void initSensors();
void updateHeartRate();
void updateOpticalSensors(); //new
void updateSpO2();              // call every loop alongside updateHeartRate()
void updateStairs(float altitudeM); // call from updateEnvironment with raw altitude
void updateMotion(float userMassKg, float strideLength);
void updateEnvironment();

// Getters
float getCurrentBPM();
float getCurrentSpO2();         // returns 0 if signal too weak
long getCurrentSteps();

bool isMax3010xHealthy();
bool isMPUHealthy();
bool isBMEHealthy();
#endif