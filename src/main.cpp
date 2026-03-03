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

float userMassKg   = 70.0f;
float userHeightM  = 1.75f;
float strideLength = 0.78f;   // ~0.43 × height typical
float userMaxHR    = 190.0f;

// ==================== SETUP ====================

void setup()
{
    Serial.begin(9600);

    // ---- I2C ----
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized");

    // ---- Network + LittleFS queue ----
    // setup_wifi() mounts LittleFS and calls queue_init() internally,
    // so any messages persisted from a previous session are immediately
    // available for flushing once MQTT reconnects.
    setup_wifi();

    // ---- Time (UTC) ----
    configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");

    // ---- MQTT ----
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(512);   // Reduced from 1024 — matches new payload size
    espClient.setInsecure();

    // // ---- Sensors ----
    initSensors();

    if (!isMax3010xHealthy()) Serial.println("Warning: MAX3010x offline");
    if (!isMPUHealthy())      Serial.println("Warning: MPU6050 offline");
    if (!isBMEHealthy())      Serial.println("Warning: BME280 offline");

    // Serial.println("Setup complete");
    queue_print_status();
}

// ==================== LOOP ====================

void loop()
{
    unsigned long now = millis();

    // ==================== CONNECTION MAINTENANCE ====================
    // Handles WiFi reconnection, MQTT reconnection, and drains the
    // LittleFS queue — all non-blocking.

    maintain_connection();

    // ==================== SENSOR SAMPLING (50 Hz) ====================

    if (now - lastSampleTime >= sampleInterval) {
        lastSampleTime = now;

        updateOpticalSensors();                   // HR + SpO2
        updateMotion(userMassKg, strideLength);   // Steps, force, power
        updateEnvironment();                      // Temp, pressure, stairs
    }

    // ==================== METRICS CALCULATION ====================

    if (now - lastMetricsUpdateTime >= metricsUpdateInterval) {
        lastMetricsUpdateTime = now;

        float currentBPM  = getCurrentBPM();
        float avgTemp, avgPress, avgHum, avgAlt, avgBPM, avgSpO2;
        avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM, avgSpO2);

        calculateMetrics(currentBPM,
                         avgTemp,
                         avgHum,
                         userMassKg,
                         userMaxHR);
    }

    // ==================== DATA PUBLISH ====================
    // publishSensorData() enqueues to LittleFS if offline and flushes
    // the backlog automatically once connectivity is restored.

    if (now - lastPublishTime >= publishInterval) {
        lastPublishTime = now;

        publishSensorData(userMassKg,
                          userHeightM,
                          strideLength);
    }
}