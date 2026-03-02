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
unsigned long lastPublishTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastSessionReportTime = 0;
unsigned long lastMetricsUpdateTime = 0;

const unsigned long sessionReportInterval = 60000;  // Report every minute
const unsigned long metricsUpdateInterval = 5000;

// ==================== USER PARAMETERS (Defaults) ====================
// These are exported in JSON for off-device recalculation
float userMassKg = 70.0;           // User mass in kg
float userHeightM = 1.75;          // User height in meters
float strideLength = 0.78;         // Stride length in meters (0.43 × height is typical)
float userMaxHR = 190.0;           // 220 - age, or user-measured


// ==================== SETUP ====================
void setup() {
    Serial.begin(9600);
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized");

    // Setup WiFi
    setup_wifi();
    
    // Configure time
    configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");

    // Setup MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(1024);
    espClient.setInsecure();

    // Initialize sensors
    initSensors();
    Serial.println("Sensors initialized");
    
    Serial.println("Setup complete");
}

// ==================== LOOP ====================
void loop() {

    unsigned long now = millis();
    
    // MQTT maintenance
    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();
    
    // ==================== SENSOR SAMPLING ====================
    // Fast sampling (50 Hz)
    if (now - lastSampleTime >= sampleInterval) {
        lastSampleTime = now;
        
        // Pass user parameters to motion processing
        updateMotion(userMassKg, strideLength);
        updateHeartRate();
        updateSpO2();
        updateEnvironment();
    }

    // ==================== METRICS CALCULATION ====================
    if (now - lastMetricsUpdateTime >= metricsUpdateInterval) {
        lastMetricsUpdateTime = now;
        
        // Get current sensor values
        float currentBPM = getCurrentBPM();
        
        // Get environmental data from last reading
        float avgTemp, avgPress, avgHum, avgAlt, avgBPM;
        avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM);
        
        // Calculate both metrics
        calculateMetrics(currentBPM, avgTemp, avgHum, userMassKg, userMaxHR);
        
    }
    
    // ==================== DATA PUBLISH ====================
    if (now - lastPublishTime >= publishInterval) {
        lastPublishTime = now;
        
        // Pass user parameters to publish function
        publishSensorData(userMassKg, userHeightM, strideLength);

        //printSessionReport(userMassKg, userHeightM, strideLength);
    }
}