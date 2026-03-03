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
PubSubClient     client(mqtt_server, mqtt_port, espClient);

// ==================== RECONNECT STATE MACHINE ====================
// WiFi connection is managed as a non-blocking state machine so that
// delay()-based polling never stalls the main loop and publishSensorData()
// continues firing on schedule even while offline.

enum WiFiConnectState {
    WIFI_IDLE,        // Not currently attempting a connection
    WIFI_CONNECTING,  // WiFi.begin() issued, polling for result
};

static WiFiConnectState wifiState          = WIFI_IDLE;
static int              wifiNetworkIdx     = 0;
static bool             tryingEnterprise   = false;
static unsigned long    wifiConnectStartMs = 0;
static unsigned long    lastWifiAttemptMs  = 0;
static unsigned long    lastMqttAttemptMs  = 0;
static unsigned long    wifiReadyMs        = 0;  // millis() when WL_CONNECTED first seen

static const unsigned long WIFI_RETRY_INTERVAL  = 15000;  // ms between reconnect cycles
static const unsigned long WIFI_CONNECT_TIMEOUT =  8000;  // ms per network before giving up
static const unsigned long WIFI_DNS_SETTLE_MS   =  3000;  // ms after connect before trusting DNS
static const unsigned long MQTT_RETRY_INTERVAL  =  5000;  // ms between MQTT attempts

// Returns true only once WiFi has been up long enough for DHCP/DNS to be
// reliable. Prevents the rc=-2 DNS failure seen immediately after a hotspot
// toggle, where WL_CONNECTED is set before the DNS server is assigned.
static bool wifi_dns_ready() {
    if (WiFi.status() != WL_CONNECTED) {
        wifiReadyMs = 0;  // Reset whenever we drop
        return false;
    }
    if (wifiReadyMs == 0) {
        wifiReadyMs = millis();
        Serial.printf("[WiFi] Connected. Waiting %lums for DNS to settle...\n",
                      WIFI_DNS_SETTLE_MS);
        return false;
    }
    return (millis() - wifiReadyMs >= WIFI_DNS_SETTLE_MS);
}

// ============================================================
// LITTLEFS QUEUE — INTERNAL HELPERS
// ============================================================

// Read a uint32 from a small text file; return defaultVal on error.
static uint32_t read_index(const char* path, uint32_t defaultVal) {
    File f = LittleFS.open(path, "r");
    if (!f) return defaultVal;
    String s = f.readStringUntil('\n');
    f.close();
    return (uint32_t)s.toInt();
}

// Overwrite a small text file with a single uint32.
static bool write_index(const char* path, uint32_t value) {
    File f = LittleFS.open(path, "w");
    if (!f) return false;
    f.println(value);
    f.close();
    return true;
}

// Build the full path for a given sequence number.
static void seq_to_path(uint32_t seq, char* buf, size_t bufLen) {
    snprintf(buf, bufLen, "%s/msg_%010u.json", QUEUE_DIR, seq);
}

// ============================================================
// LITTLEFS QUEUE — PUBLIC API
// ============================================================

void queue_init() {
    // Mount LittleFS; format on first use if the partition is blank.
    if (!LittleFS.begin(true)) {
        Serial.println("[Queue] LittleFS mount failed, queue disabled.");
        return;
    }

    // Create queue directory if it doesn't exist yet.
    if (!LittleFS.exists(QUEUE_DIR)) {
        LittleFS.mkdir(QUEUE_DIR);
        Serial.println("[Queue] Created /queue directory.");
    }

    // Initialise index files if missing (fresh partition).
    if (!LittleFS.exists(QUEUE_SEQ_FILE)) write_index(QUEUE_SEQ_FILE, 0);
    if (!LittleFS.exists(QUEUE_ACK_FILE)) write_index(QUEUE_ACK_FILE, 0);

    Serial.printf("[Queue] Ready. %d message(s) pending.\n", queue_count());
}

// ---------------------------------------------------------------------------
// queue_enqueue — write payload to flash as the next message in the queue.
// Returns false if the queue is already at QUEUE_MAX_MESSAGES or FS is full.
// ---------------------------------------------------------------------------
bool queue_enqueue(const char* payload) {
    int pending = queue_count();

    if (pending >= QUEUE_MAX_MESSAGES) {
        // Drop the oldest message to make room (slide the read-head forward).
        uint32_t ack = read_index(QUEUE_ACK_FILE, 0);
        char oldPath[64];
        seq_to_path(ack, oldPath, sizeof(oldPath));
        LittleFS.remove(oldPath);
        write_index(QUEUE_ACK_FILE, ack + 1);
        Serial.println("[Queue] Capacity reached, oldest message dropped.");
    }

    uint32_t seq = read_index(QUEUE_SEQ_FILE, 0);
    char path[64];
    seq_to_path(seq, path, sizeof(path));

    File f = LittleFS.open(path, "w");
    if (!f) {
        Serial.println("[Queue] ERROR: could not open file for writing.");
        return false;
    }

    size_t written = f.print(payload);
    f.close();

    if (written == 0) {
        Serial.println("[Queue] ERROR: nothing written to file.");
        LittleFS.remove(path);
        return false;
    }

    // Advance the write-head only after the file is safely on disk.
    write_index(QUEUE_SEQ_FILE, seq + 1);

    Serial.printf("[Queue] Enqueued seq=%u (%d bytes). Queue depth: %d\n",
                  seq, written, queue_count());
    return true;
}

// ---------------------------------------------------------------------------
// queue_flush — publish every pending message oldest-first, deleting each
// file only after a confirmed MQTT publish.  Stops immediately if any
// publish fails so we don't skip messages or lose ordering.
// ---------------------------------------------------------------------------
void queue_flush() {
    int pending = queue_count();
    if (pending == 0) return;

    Serial.printf("[Queue] Flushing %d queued message(s)...\n", pending);

    uint32_t ack = read_index(QUEUE_ACK_FILE, 0);
    uint32_t seq = read_index(QUEUE_SEQ_FILE, 0);

    char msgBuf[QUEUE_MSG_MAX_LEN];

    for (uint32_t i = ack; i < seq; i++) {
        char path[64];
        seq_to_path(i, path, sizeof(path));

        // File might be missing if a previous write was interrupted.
        if (!LittleFS.exists(path)) {
            Serial.printf("[Queue] seq=%u missing on disk, skipping.\n", i);
            write_index(QUEUE_ACK_FILE, i + 1);
            continue;
        }

        // Read the message from flash.
        File f = LittleFS.open(path, "r");
        if (!f) {
            Serial.printf("[Queue] Could not open seq=%u, skipping.\n", i);
            write_index(QUEUE_ACK_FILE, i + 1);
            continue;
        }

        int len = f.readBytes(msgBuf, sizeof(msgBuf) - 1);
        f.close();
        msgBuf[len] = '\0';

        // Attempt MQTT publish.
        bool ok = client.publish(mqtt_topic, msgBuf, len);
        if (!ok) {
            Serial.printf("[Queue] Publish failed at seq=%u, will retry next cycle.\n", i);
            break;  // Keep this file; stop so we don't skip it.
        }

        // Confirmed delivery — delete the file and advance the read-head.
        LittleFS.remove(path);
        write_index(QUEUE_ACK_FILE, i + 1);
        Serial.printf("[Queue] Delivered + deleted seq=%u.\n", i);
    }
}

// ---------------------------------------------------------------------------
// queue_count — derive depth from the two index files.
// ---------------------------------------------------------------------------
int queue_count() {
    uint32_t seq = read_index(QUEUE_SEQ_FILE, 0);
    uint32_t ack = read_index(QUEUE_ACK_FILE, 0);
    return (int)(seq - ack);
}

// ---------------------------------------------------------------------------
// queue_print_timestamps — read every pending file, parse the "timestamp"
// field from each JSON payload, and print them to Serial oldest-first.
// Called at the end of publishSensorData() for visibility into the backlog.
// ---------------------------------------------------------------------------
void queue_print_timestamps() {
    uint32_t ack = read_index(QUEUE_ACK_FILE, 0);
    uint32_t seq = read_index(QUEUE_SEQ_FILE, 0);
    int pending  = (int)(seq - ack);

    if (pending == 0) {
        Serial.println("[Queue] No queued messages.");
        return;
    }

    Serial.printf("[Queue] Timestamps of %d queued message(s):\n", pending);

    char msgBuf[QUEUE_MSG_MAX_LEN];

    for (uint32_t i = ack; i < seq; i++) {
        char path[64];
        seq_to_path(i, path, sizeof(path));

        if (!LittleFS.exists(path)) {
            //Serial.printf("  seq=%-6u  <file missing>\n", i);
            continue;
        }

        File f = LittleFS.open(path, "r");
        if (!f) {
            Serial.printf("  seq=%-6u  <could not open>\n", i);
            continue;
        }

        int len = f.readBytes(msgBuf, sizeof(msgBuf) - 1);
        f.close();
        msgBuf[len] = '\0';

        // Parse just enough to extract the timestamp field.
        // 256 bytes covers the root-level keys without loading stats{}.
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, msgBuf, len);

        if (err) {
            //Serial.printf("  seq=%-6u  <parse error: %s>\n", i, err.c_str());
            continue;
        }

        const char* ts = doc["timestamp"] | "<no timestamp>";
        Serial.printf("  seq=%-6u  %s\n", i, ts);
    }
}

// ---------------------------------------------------------------------------
// queue_print_status — human-readable debug summary on Serial.
// ---------------------------------------------------------------------------
void queue_print_status() {
    uint32_t seq = read_index(QUEUE_SEQ_FILE, 0);
    uint32_t ack = read_index(QUEUE_ACK_FILE, 0);
    int pending  = (int)(seq - ack);

    // Calculate approximate flash usage.
    float usedKB = (pending * QUEUE_MSG_MAX_LEN) / 1024.0f;

    Serial.println("╔══════════════════════════════════╗");
    Serial.println("║        QUEUE STATUS               ║");
    Serial.println("╠══════════════════════════════════╣");
    Serial.printf( "║ Pending messages:  %6d         ║\n", pending);
    Serial.printf( "║ Write head (seq):  %6u         ║\n", seq);
    Serial.printf( "║ Read  head (ack):  %6u         ║\n", ack);
    Serial.printf( "║ Est. flash usage:  %5.1f KB       ║\n", usedKB);
    Serial.printf( "║ Capacity:          %6d msgs     ║\n", QUEUE_MAX_MESSAGES);
    Serial.println("╚══════════════════════════════════╝");
}

// ============================================================
// WIFI FUNCTIONS
// ============================================================

// ============================================================
// WIFI — NON-BLOCKING STATE MACHINE
// ============================================================
// Both setup_wifi() and attempt_wifi_reconnect() issue WiFi.begin()
// and return immediately. The state machine in attempt_wifi_reconnect()
// is then ticked every loop() via maintain_connection(), polling
// WiFi.status() without any delay() so the loop is never blocked.

static void begin_personal_network(int idx) {
    const char* ssid = personalNetworks[idx].ssid;
    const char* pass = personalNetworks[idx].password;

    Serial.printf("[WiFi] Trying WPA2-Personal: %s\n", ssid);

    // Clear stored AP — required for iPhone hotspots which change
    // BSSID on every toggle; without this the ESP32 won't reassociate.
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    wifiState          = WIFI_CONNECTING;
    wifiConnectStartMs = millis();
    tryingEnterprise   = false;
}

static void begin_enterprise_network() {
    Serial.printf("[WiFi] Trying WPA2-Enterprise: %s\n", enterpriseSSID);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_enable();
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)enterpriseUsername, strlen(enterpriseUsername));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t*)enterpriseUsername, strlen(enterpriseUsername));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t*)enterprisePassword, strlen(enterprisePassword));
    WiFi.begin(enterpriseSSID);

    wifiState          = WIFI_CONNECTING;
    wifiConnectStartMs = millis();
    tryingEnterprise   = true;
}

// Called from setup() — kicks off the first connection attempt and returns.
// The loop() will tick attempt_wifi_reconnect() to complete it.
void setup_wifi() {
    Serial.println("\n[WiFi] Starting...");
    queue_init();

    wifiNetworkIdx = 0;
    if (PERSONAL_NETWORK_COUNT > 0) {
        begin_personal_network(0);
    } else {
        begin_enterprise_network();
    }
}

// Called every loop() via maintain_connection().
// Returns immediately in every code path — zero delay().
static void attempt_wifi_reconnect() {
    unsigned long now = millis();

    if (wifiState == WIFI_CONNECTING) {
        // Poll for connection result.
        if (WiFi.status() == WL_CONNECTED) {
            wifiState = WIFI_IDLE;
            Serial.printf("[WiFi] Connected to %s. IP: %s\n",
                          tryingEnterprise ? enterpriseSSID
                                          : personalNetworks[wifiNetworkIdx].ssid,
                          WiFi.localIP().toString().c_str());
            return;
        }

        // Still waiting — check timeout.
        if (now - wifiConnectStartMs < WIFI_CONNECT_TIMEOUT) {
            return;  // Keep waiting, don't block
        }

        // Timed out — try next network.
        Serial.println("[WiFi] Timed out.");

        if (!tryingEnterprise && wifiNetworkIdx + 1 < PERSONAL_NETWORK_COUNT) {
            // Try next personal network.
            wifiNetworkIdx++;
            begin_personal_network(wifiNetworkIdx);
        } else if (!tryingEnterprise) {
            // All personal networks exhausted — try enterprise.
            begin_enterprise_network();
        } else {
            // Enterprise also failed — back to idle, wait before retrying.
            wifiState      = WIFI_IDLE;
            wifiNetworkIdx = 0;
            lastWifiAttemptMs = now;
            Serial.println("[WiFi] All networks failed, will retry later.");
        }
        return;
    }

    // WIFI_IDLE — start a new reconnect cycle if enough time has passed.
    if (now - lastWifiAttemptMs < WIFI_RETRY_INTERVAL) return;
    lastWifiAttemptMs = now;
    wifiReadyMs = 0;  // Reset DNS settle timer for the new connection

    Serial.println("[WiFi] Connection lost, starting reconnect cycle...");
    wifiNetworkIdx = 0;
    if (PERSONAL_NETWORK_COUNT > 0) {
        begin_personal_network(0);
    } else {
        begin_enterprise_network();
    }
}

// Kept for API compatibility — behaviour is now handled inside the state machine.
bool connectPersonalNetwork(const char* ssid, const char* password) {
    return (WiFi.status() == WL_CONNECTED);
}
bool connectEnterpriseNetwork() {
    return (WiFi.status() == WL_CONNECTED);
}

void reconnect_mqtt() {
    unsigned long now = millis();
    if (now - lastMqttAttemptMs < MQTT_RETRY_INTERVAL) return;
    lastMqttAttemptMs = now;

    Serial.print("[MQTT] Attempting connection...");
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
        Serial.println("connected.");
    } else {
        Serial.printf("failed (rc=%d), will retry.\n", client.state());
    }
}

// ============================================================
// MAIN CONNECTION MAINTENANCE  (call every loop)
// ============================================================

void maintain_connection() {
    // 1. WiFi must be up before anything else.
    if (WiFi.status() != WL_CONNECTED) {
        attempt_wifi_reconnect();
        return;
    }

    // 2. Wait for DNS to be ready before attempting MQTT.
    // Immediately after a hotspot toggle WL_CONNECTED is set but the
    // DHCP-assigned DNS server isn't reachable yet, causing rc=-2 failures.
    if (!wifi_dns_ready()) return;

    // 3. MQTT must be connected.
    if (!client.connected()) {
        reconnect_mqtt();
        return;
    }

    // 4. Service the PubSubClient keep-alive.
    client.loop();

    // 5. Drain the flash queue now that we're back online.
    queue_flush();
}

// ============================================================
// PUBLISH SENSOR DATA
// ============================================================

void publishSensorData(float userMassKg, float userHeightM, float strideLength) {

    // ---- Gather data ----
    // float avgTemp, avgPress, avgHum, avgAlt, avgBPM, avgSpo2;
    // avgData.getAverages(avgTemp, avgPress, avgHum, avgAlt, avgBPM, avgSpo2);

    SessionMetrics session = getSessionMetrics();
    String         timestamp = getUTCTimestamp();
    // TPIComponents  tpi = getTPIComponents();

    // ---- Build JSON (downsized to 512 bytes) ----
    DynamicJsonDocument doc(512);
    doc["talusId"]   = talusId;
    doc["timestamp"] = timestamp;

    JsonObject stats = doc.createNestedObject("stats");
    // stats["temperature"]    = avgTemp;
    // stats["pressure"]       = avgPress;
    // stats["humidity"]       = avgHum;
    // stats["altitude"]       = avgAlt;
    // stats["bpm"]            = avgBPM;
    // stats["spo2"]           = avgSpo2;
    // stats["steps"]          = session.totalSteps;
    // stats["cadence"]        = session.avgCadence;
    // stats["flightsClimbed"] = session.flightsClimbed;
    // stats["avgForce"]       = session.avgForce;
    // stats["avgPower"]       = session.avgPower;
    // stats["tpi"]            = tpi.finalScore;

    stats["temperature"]    = 0;
    stats["pressure"]       = 0;
    stats["humidity"]       = 0;
    stats["altitude"]       = 0;
    stats["bpm"]            = 0;
    stats["spo2"]           = 0;
    stats["steps"]          = 0;
    stats["cadence"]        = 0;
    stats["flightsClimbed"] = 0;
    stats["avgForce"]       = 0;
    stats["avgPower"]       = 0;
    stats["tpi"]            = 0;

    char buffer[512];
    serializeJson(doc, buffer, sizeof(buffer));

    // ---- Attempt publish or queue ----
    bool wifiUp = (WiFi.status() == WL_CONNECTED);
    bool mqttUp = client.connected();

    if (wifiUp && mqttUp) {
        // Flush any backlog first to preserve chronological order.
        queue_flush();

        bool ok = client.publish(mqtt_topic, buffer, strlen(buffer));
        if (ok) {
            Serial.println("[MQTT] Published.");
        } else {
            Serial.println("[MQTT] Publish failed, queuing message.");
            queue_enqueue(buffer);
        }
    } else {
        queue_enqueue(buffer);
        Serial.printf("[MQTT] Offline message queued. Depth: %d / %d\n",
                      queue_count(), QUEUE_MAX_MESSAGES);
    }

    Serial.println("\n--- JSON Payload ---");
    serializeJsonPretty(doc, Serial);
    Serial.println("\n--------------------");

    // ---- Print timestamps of any messages still waiting in the queue ----
    queue_print_timestamps();

    // ---- Reset averaging window ----
    avgData.reset();
    stepsSinceLastPublish = 0;
}