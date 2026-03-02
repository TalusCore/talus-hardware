/* ============================================================
**
** CONSTS FOR TALUS SETUP
**
** ============================================================
*/

#include "config.h"
#include "secrets.h"

// Use the defines from secrets.h
const char* enterpriseSSID = WIFI_SSID;
const char* enterpriseUsername = WIFI_USERNAME;
const char* enterprisePassword = WIFI_PASSWORD;

const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;
const char* talusId = TALUS_ID;

const int personalNetworksCount = sizeof(personalNetworks) / sizeof(personalNetworks[0]);

// ==================== MQTT CONFIGURATION ====================
const char* mqtt_client_id = "ESP32_Periodic_Publisher";
const char* mqtt_topic = "sensor/data";

// ==================== TIMING CONSTANTS ====================
const unsigned long publishInterval = 5000;      // 5 seconds
const unsigned long sampleInterval = 20;         // 50 Hz

// ==================== SENSOR CONSTANTS ====================
const unsigned long minStepInterval = 300;       // ms
const float alpha = 0.9;                         // Low-pass filter
const float dynamicThreshold = 0.4;              // Step detection
const int bpmBufferSize = 8;                     // Heart rate averaging
const long irThreshold = 50000;                  // Finger detection

// ==================== SPO2 CONSTANTS ====================
const int spo2SampleCount = 25;                  // rolling buffer – more samples = smoother
const long spo2IrThreshold = 20000;              // lenient: ankle has weaker perfusion signal
const float spo2Alpha = 0.1f;                    // slow EMA – dampens motion spikes
const float spo2MinValid = 40.0f;                // reject obviously bad readings
const float spo2MaxValid = 100.0f;               // physical ceiling

// ==================== STAIR DETECTION CONSTANTS ====================
const float stairFlightHeight  = 3.0f;           // m — standard floor height
const float stairAltAlpha      = 0.05f;          // very slow EMA: smooths out footstrike bounce
const float stairHysteresis    = 1.5f;           // m — must descend this far to reset baseline
const int   stairMinSteps      = 10;             // ~10 steps to climb one flight

// ==================== MOTION ANALYSIS CONSTANTS ====================
//const unsigned long minStepInterval = 300;       // ms between steps
const float swingThreshold = 0.5;                // g - foot leaving ground
const float impactThreshold = 0.5;               // g - foot strike
const float gravityMs2 = 9.81;                   // m/s² gravity constant