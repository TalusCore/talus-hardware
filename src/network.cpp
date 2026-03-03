/* ============================================================
**
** Functions related to Wi-Fi and MQTT connectivity
**
** ============================================================
*/

#include "network.h"
#include "utils.h"
#include "sensors.h"
#include "metrics.h"
#include "esp_wpa2.h"
#include "secrets.h"
#include <ArduinoJson.h>

// ==================== GLOBAL OBJECTS ====================
WiFiClientSecure espClient;
PubSubClient client(mqtt_server, mqtt_port, espClient);

// ==================== WIFI FUNCTIONS ====================
bool connectPersonalNetwork(const char* ssid, const char* password) {
    delay(1000);
    Serial.print("Trying WPA2-Personal: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    int retry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (retry < 15)) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    Serial.println("");
    return (WiFi.status() == WL_CONNECTED);
}

bool connectEnterpriseNetwork() {
    delay(1000);
    Serial.print("Trying WPA2-Enterprise: ");
    Serial.println(enterpriseSSID);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_enable();

    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)enterpriseUsername, strlen(enterpriseUsername));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)enterpriseUsername, strlen(enterpriseUsername));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)enterprisePassword, strlen(enterprisePassword));

    WiFi.begin(enterpriseSSID);
    int retry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (retry < 20)) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    Serial.println("");
    return (WiFi.status() == WL_CONNECTED);
}

void setup_wifi() {
    Serial.println("\nStarting Wi-Fi...");
    
    // Try personal networks
    for (int i = 0; i < PERSONAL_NETWORK_COUNT; i++) {
        if (connectPersonalNetwork(personalNetworks[i].ssid, personalNetworks[i].password)) {
            return; 
        }
    }
    
    // Try enterprise network
    connectEnterpriseNetwork();
}

// ==================== MQTT FUNCTIONS ====================
void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// ==================== PUBLISH SENSOR DATA ====================
void publishSensorData(float userMassKg, float userHeightM, float strideLength) {
    // Get averaged sensor data
    float avgTemp, avgPress, avgHum, avgAlt, avgBPM,avgSpo2;
    avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM,avgSpo2);

    // Get session metrics NEW
    SessionMetrics session = getSessionMetrics();
    StepEvent lastStepData = getLastStep();

    // Get timestamp
    String timestamp = getUTCTimestamp();

    // Get health metrics
    TPIComponents tpi = getTPIComponents();

    // Create JSON
    StaticJsonDocument<1536> doc;  // Increased size NEW
    doc["talusId"] = talusId;
    doc["timestamp"] = timestamp;

    // Basic environmental stats
    JsonObject stats = doc.createNestedObject("stats");
    stats["temperature"] = avgTemp;
    stats["pressure"] = avgPress;
    stats["humidity"] = avgHum;
    stats["altitude"] = avgAlt;
    stats["bpm"] = avgBPM;
    stats["spo2"] = avgSpo2;   // 0 = no valid ankle signal

    // Motion metrics (calculated with user parameters) NEW
    // stats["steps"] = session.totalSteps;
    stats["steps"] = stepsSinceLastPublish;
    stats["cadence"] = session.avgCadence;
    stats["flightsClimbed"] = flightsSinceLastPublish;
    stats["avgForce"] = session.avgForce;
    stats["avgPower"] = session.avgPower;

    // Talus Performance Index
    stats["tpi"] = tpi.finalScore;

    // Serialize
    char buffer[1536];  // Increased buffer NEW
    serializeJson(doc, buffer);
    String escapedJson = '"' + escapeForC(String(buffer)) + '"';

    // Publish
    bool success = client.publish(mqtt_topic, escapedJson.c_str(), escapedJson.length());
    
    Serial.println(success ? "Published to MQTT" : "MQTT publish failed");
    Serial.println("\n--- JSON Payload ---");
    serializeJsonPretty(doc, Serial);
    Serial.println("\n--------------------");

    // Reset for next interval
    avgData.reset();
    stepsSinceLastPublish = 0;
    flightsSinceLastPublish = 0;
}