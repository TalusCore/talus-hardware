// include/secrets.example.h

/* ============================================================
**
** Credentials but not the real ones
**
** ============================================================
*/

#ifndef SECRETS_H
#define SECRETS_H

#include "config.h"

// Enterprise WiFi Credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_USERNAME "your_username@uwaterloo.ca"
#define WIFI_PASSWORD "your_password"

// Personal WiFi Credentials
static const WifiCredential personalNetworks[] = {
    {"wifi1","password1"},
    {"wifi2", "password2"},
};
#define PERSONAL_NETWORK_COUNT 2


// MQTT Credentials
#define MQTT_SERVER "your_mqtt_broker.com"
#define MQTT_PORT 8883
#define MQTT_USERNAME "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"
#define TALUS_ID "your-talus-id-here"

#endif