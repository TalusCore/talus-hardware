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
#include <LittleFS.h>
#include "config.h"

// ==================== LITTLEFS QUEUE CONFIG ====================
//
// Each message is stored as an individual file:
//   /queue/msg_<seq>.json
//
// Two small index files track the queue head and tail:
//   /queue/seq.txt  — next sequence number to assign (write head)
//   /queue/ack.txt  — oldest undelivered sequence number (read head)
//
// Sequence numbers are monotonically increasing uint32_t values
// serialised as decimal strings. The on-disk format is deliberately
// simple so the queue survives unexpected power loss without
// corruption — if seq.txt advances but the matching file was never
// written, flush() skips that slot and advances ack.txt past it.
//
// Capacity: QUEUE_MAX_MESSAGES × QUEUE_MSG_MAX_LEN
//           720 msgs × 512 bytes ≈ 360 KB  (fits comfortably in
//           the default 1.5–3 MB LittleFS partition on most ESP32
//           flash configurations)

#define QUEUE_DIR           "/queue"
#define QUEUE_SEQ_FILE      "/queue/seq.txt"
#define QUEUE_ACK_FILE      "/queue/ack.txt"
#define QUEUE_MSG_MAX_LEN   512    // bytes — matches downsized JSON buffer
#define QUEUE_MAX_MESSAGES  720    // 1 hour @ 5-second publish interval

// ==================== FUNCTION DECLARATIONS ====================

// Lifecycle
void setup_wifi();
bool connectPersonalNetwork(const char* ssid, const char* password);
bool connectEnterpriseNetwork();

// Call every loop — non-blocking WiFi + MQTT keepalive + queue drain
void maintain_connection();

// MQTT reconnect (rate-limited, non-blocking)
void reconnect_mqtt();

// LittleFS queue
void queue_init();                           // Mount FS, create dir, recover state
bool queue_enqueue(const char* payload);     // Persist one message to flash
void queue_flush();                          // Publish + delete pending messages
int  queue_count();                          // Number of messages waiting on disk
void queue_print_status();                   // Serial debug summary
void queue_print_timestamps();

// Sensor publish — queues automatically when offline
void publishSensorData(float userMassKg, float userHeightM, float strideLength);


// ==================== GLOBAL OBJECTS ====================
extern WiFiClientSecure espClient;
extern PubSubClient     client;

#endif