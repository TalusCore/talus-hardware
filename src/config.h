/* ============================================================
**
** CONSTS FOR TALUS SETUP
**
** ============================================================
*/

#ifndef CONFIG_H
#define CONFIG_H

// ==================== PIN DEFINITIONS ====================
#define SDA_PIN 21
#define SCL_PIN 22
#define RXD2 16
#define TXD2 17

// ==================== WIFI CONFIGURATION ====================
// WPA2-Enterprise
extern const char* enterpriseSSID;
extern const char* enterpriseUsername;
extern const char* enterprisePassword;

// Personal networks structure
struct WifiCredential {
    const char* ssid;
    const char* password;
};

// ==================== MQTT CONFIGURATION ====================
extern const char* talusId;
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_client_id;
extern const char* mqtt_topic;
extern const char* mqtt_username;
extern const char* mqtt_password;

// ==================== TIMING CONSTANTS ====================
extern const unsigned long publishInterval;      // 5000 ms
extern const unsigned long sampleInterval;       // 20 ms (50 Hz)

// ==================== SENSOR CONSTANTS ====================
extern const unsigned long minStepInterval;      // 300 ms
extern const float alpha;                        // 0.9
extern const float dynamicThreshold;             // 0.4
extern const int bpmBufferSize;                  // 8
extern const long irThreshold;                   // 50000

// ==================== SPO2 CONSTANTS ====================
extern const int spo2SampleCount;               // rolling buffer size
extern const long spo2IrThreshold;              // lower IR threshold for ankle
extern const float spo2Alpha;                   // EMA smoothing factor
extern const float spo2MinValid;                // minimum plausible SpO2 %
extern const float spo2MaxValid;                // maximum plausible SpO2 %

// ==================== STAIR DETECTION CONSTANTS ====================
extern const float stairFlightHeight;           // metres per flight (~3 m)
extern const float stairAltAlpha;               // EMA smoothing for altitude
extern const float stairHysteresis;             // must drop this far before baseline resets
extern const int   stairMinSteps;               // min steps required to confirm a flight

// ==================== MOTION ANALYSIS CONSTANTS ====================
extern const unsigned long minStepInterval;      // 300 ms
extern const float swingThreshold;               // 0.5 g
extern const float impactThreshold;              // 2.0 g
extern const float gravityMs2;                   // 9.81 m/s²

#endif