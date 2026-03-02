#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#include "config.h"
#include "network.h"
#include "sensors.h"
#include "utils.h"
#include "motion.h"
#include "metrics.h"

// ==================== TIMING VARIABLES ====================

unsigned long lastPublishTime       = 0;
unsigned long lastSampleTime        = 0;
unsigned long lastSessionReportTime = 0;
unsigned long lastMetricsUpdateTime = 0;

const unsigned long sessionReportInterval = 60000;  // 1 minute
const unsigned long metricsUpdateInterval = 5000;   // 5 seconds

// ==================== USER PARAMETERS (Defaults) ====================

float userMassKg     = 70.0f;
float userHeightM    = 1.75f;
float strideLength   = 0.78f;     // ~0.43 × height typical
float userMaxHR      = 190.0f;

// ==================== SETUP ====================

void setup()
{
    Serial.begin(9600);

    // ---- I2C ----
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized");

    // ---- Network ----
    setup_wifi();

    // ---- Time (UTC) ----
    configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");

    // ---- MQTT ----
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(1024);
    espClient.setInsecure();

    // ---- Sensors ----
    initSensors();

    if (!isMax3010xHealthy()) Serial.println("Warning: MAX3010x offline");
    if (!isMPUHealthy())      Serial.println("Warning: MPU6050 offline");
    if (!isBMEHealthy())      Serial.println("Warning: BME280 offline");

    Serial.println("Setup complete");
}

// ==================== LOOP ====================

void loop()
{
    unsigned long now = millis();

    // ==================== MQTT MAINTENANCE ====================

    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();

    // ==================== SENSOR SAMPLING ====================
    // Fast sampling (e.g. 50 Hz)

    if (now - lastSampleTime >= sampleInterval) {
        lastSampleTime = now;

        // 1. Optical sensors (HR + SpO2 share FIFO)
        updateOpticalSensors();

        // 2. Motion (step detection, motion metrics)
        updateMotion(userMassKg, strideLength);

        // 3. Environmental (temperature, pressure, stairs)
        updateEnvironment();
    }

    // ==================== METRICS CALCULATION ====================

    if (now - lastMetricsUpdateTime >= metricsUpdateInterval) {
        lastMetricsUpdateTime = now;

        float currentBPM = getCurrentBPM();
        float currentSpO2 = getCurrentSpO2();

        float avgTemp, avgPress, avgHum, avgAlt, avgBPM, avgSpO2;
        avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM,avgSpO2);

        calculateMetrics(currentBPM,
                         avgTemp,
                         avgHum,
                         userMassKg,
                         userMaxHR);
    }

    // ==================== DATA PUBLISH ====================

    if (now - lastPublishTime >= publishInterval) {
        lastPublishTime = now;

        publishSensorData(userMassKg,
                          userHeightM,
                          strideLength);
    }
}