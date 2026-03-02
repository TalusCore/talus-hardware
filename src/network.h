/* ============================================================
**
** Functions related to Wi-Fi and MQTT connectivity
**
** ============================================================
*/

#ifndef NETWORK_H
#define NETWORK_H

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "config.h"

// ==================== FUNCTION DECLARATIONS ====================
void setup_wifi();
bool connectPersonalNetwork(const char* ssid, const char* password);
bool connectEnterpriseNetwork();
void reconnect_mqtt();
void publishSensorData(float userMassKg, float userHeightM, float strideLength);

// ==================== GLOBAL OBJECTS ====================
extern WiFiClientSecure espClient;
extern PubSubClient client;

#endif