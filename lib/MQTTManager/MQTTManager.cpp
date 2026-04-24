#include <Arduino.h>
#include <time.h>
#include "MQTTManager.h"

MQTTManager::MQTTManager(const char* broker, uint16_t port,
                         const char* user, const char* password,
                         const char* clientId, const char* baseTopic,
                         uint8_t meterCount)
    : _broker(broker), _port(port), _user(user), _pass(password),
      _clientId(clientId), _baseTopic(baseTopic),
      _maxMeters(meterCount <= MQTT_MAX_METERS ? meterCount : MQTT_MAX_METERS),
      _meterCount(0), _lastReconnectAttempt(0)
{
    _mqtt.setClient(_wifiClient);
    _mqtt.setServer(_broker, _port);
    _mqtt.setBufferSize(512);
    memset(_meters, 0, sizeof(_meters));
}

// ============================================================
//  WiFi + MQTT
// ============================================================

void MQTTManager::begin(const char* ssid, const char* wifiPass)
{
    _ssid     = ssid;
    _wifiPass = wifiPass;

    log_i("Connexion WiFi à %s", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);  // WPA2/WPA3 mixte : force WPA2 pour éviter l'échec PMF
    WiFi.begin(ssid, wifiPass);

    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 30000) {
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi connecté — IP : %s", WiFi.localIP().toString().c_str());
    } else {
        log_e("WiFi ÉCHEC — on continue sans réseau");
    }

    _reconnect();
}

bool MQTTManager::publish(const char* topic, const char* payload, bool retained)
{
    if (!_mqtt.connected()) return false;
    return _mqtt.publish(topic, payload, retained);
}

void MQTTManager::loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        uint32_t now = millis();
        if (now - _lastReconnectAttempt > 15000) {
            _lastReconnectAttempt = now;
            log_w("WiFi déconnecté — reconnexion à %s", _ssid);
            WiFi.begin(_ssid, _wifiPass);  // reconnect() inopérant après échec initial
        }
        return;
    }
    if (!_mqtt.connected()) {
        uint32_t now = millis();
        if (now - _lastReconnectAttempt > 5000) {
            _lastReconnectAttempt = now;
            _reconnect();
        }
    }
    _mqtt.loop();
}

bool MQTTManager::_reconnect()
{
    if (WiFi.status() != WL_CONNECTED) return false;

    log_i("MQTT connexion à %s:%d", _broker, _port);
    char willTopic[80];
    snprintf(willTopic, sizeof(willTopic), "%s/status", _baseTopic);

    bool ok = _mqtt.connect(_clientId, _user, _pass, willTopic, 1, true, "offline");
    if (ok) {
        log_i("MQTT connecté");
        _mqtt.publish(willTopic, "online", true);
        for (uint8_t i = 0; i < _meterCount; i++)
            _meters[i].haDiscoverySent = false;
    } else {
        log_e("MQTT échec (rc=%d)", _mqtt.state());
    }
    return ok;
}

// ============================================================
//  Publication EverBlu
// ============================================================

// Seuil de durée pour qualifier un saut nocturne (4 h).
// Les compteurs répondent de 06:00 à 18:59 ; le saut normal entre lectures
// est de READ_INTERVAL_MIN (~15 min). Tout écart > 4 h signale la nuit.
static const uint32_t OVERNIGHT_GAP_MS = 4UL * 3600000UL;

void MQTTManager::publishEverBlu(uint32_t serial, const EverBluData& d, uint32_t leakThresholdL)
{
    if (!d.valid) return;

    char serialStr[12];
    snprintf(serialStr, sizeof(serialStr), "%lu", serial);

    MeterState* m = _findOrCreate(serialStr);
    if (!m) {
        log_e("Plus de slots disponibles");
        return;
    }

    uint32_t prevLiters = m->liters;
    uint32_t now        = millis();
    uint32_t elapsed    = (m->lastSeenMs > 0) ? (now - m->lastSeenMs) : 0;

    // ---- Détection de fuite nocturne --------------------------
    // On compare la consommation sur le saut nocturne uniquement,
    // car les compteurs sont muets la nuit → aucune lecture entre
    // ~19:00 et ~06:00. Un elapsed > 4 h identifie ce saut.
    if (elapsed > OVERNIGHT_GAP_MS && prevLiters > 0 && leakThresholdL > 0) {
        int32_t overnightL = (int32_t)(d.liters - prevLiters);
        float   gapH       = elapsed / 3600000.0f;
        log_i("Compteur %s — saut nuit %.1fh : %d L consommés", serialStr, gapH, overnightL);

        bool leak = (overnightL > (int32_t)leakThresholdL);
        if (leak) {
            log_w("Compteur %s : %d L pendant la nuit (seuil=%lu L) — alerte fuite !",
                  serialStr, overnightL, leakThresholdL);
        }
        if (leak != m->leakActive && _mqtt.connected()) {
            char topic[80];
            _buildTopic(topic, sizeof(topic), serialStr, "leak");
            _mqtt.publish(topic, leak ? "ON" : "OFF", true);
        }
        m->leakActive = leak;
    }

    m->liters     = d.liters;
    m->lastSeenMs = now;

    // Auto-discovery HA au premier passage
    if (!m->haDiscoverySent && _mqtt.connected()) {
        _publishDiscovery(*m);
        m->haDiscoverySent = true;
    }

    if (!_mqtt.connected()) return;

    // Timestamp ISO8601 si NTP disponible
    char timestamp[32] = "unknown";
    struct tm t;
    if (getLocalTime(&t, 0)) {
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &t);
    }

    // Publication état JSON
    char topic[80];
    _buildTopic(topic, sizeof(topic), serialStr, "state");

    JsonDocument doc;
    doc["liters"]      = d.liters;
    doc["m3"]          = d.liters / 1000.0f;
    if (m->lastSeenMs > 0)
        doc["delta_l"] = (int32_t)(d.liters - prevLiters);
    doc["battery_months"] = d.battery;
    doc["read_count"]  = d.readCount;
    doc["rssi"]        = d.rssi;
    doc["timestamp"]   = timestamp;

    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    _mqtt.publish(topic, payload, true);

    log_i("→ %s : %s", topic, payload);
}

// ============================================================
//  Re-publication de l'état de fuite (après reconnexion MQTT)
// ============================================================

void MQTTManager::checkLeaks()
{
    if (!_mqtt.connected()) return;

    for (uint8_t i = 0; i < _meterCount; i++) {
        MeterState& m = _meters[i];
        if (!m.leakActive) continue;

        char topic[80];
        _buildTopic(topic, sizeof(topic), m.serial, "leak");
        _mqtt.publish(topic, "ON", true);
    }
}

// ============================================================
//  Watchdog
// ============================================================

void MQTTManager::checkWatchdog(uint32_t timeoutMs)
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < _meterCount; i++) {
        MeterState& m = _meters[i];
        if (m.lastSeenMs == 0) continue;
        if (now - m.lastSeenMs > timeoutMs) {
            log_w("Compteur %s silencieux depuis %lu min",
                  m.serial, (now - m.lastSeenMs) / 60000UL);
            if (_mqtt.connected()) {
                char topic[80];
                _buildTopic(topic, sizeof(topic), m.serial, "availability");
                _mqtt.publish(topic, "offline", true);
            }
        }
    }
}

// ============================================================
//  Home Assistant Auto-Discovery
// ============================================================

void MQTTManager::_publishDiscovery(const MeterState& m)
{
    char stateTopic[80], leakTopic[80], deviceId[24], configTopic[120];
    _buildTopic(stateTopic, sizeof(stateTopic), m.serial, "state");
    _buildTopic(leakTopic,  sizeof(leakTopic),  m.serial, "leak");
    snprintf(deviceId, sizeof(deviceId), "everblu_%s", m.serial);

    char payload[512];
    JsonDocument doc;

    // --- Capteur volume (m³) ---
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/sensor/%s_volume/config", deviceId);
    doc.clear();
    doc["name"]               = "Volume";
    doc["unique_id"]          = String(deviceId) + "_volume";
    doc["state_topic"]        = stateTopic;
    doc["value_template"]     = "{{ value_json.m3 | round(3) }}";
    doc["unit_of_measurement"] = "m³";
    doc["device_class"]       = "water";
    doc["state_class"]        = "total_increasing";
    doc["icon"]               = "mdi:water";
    {
        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"][0] = deviceId;
        dev["name"]           = String("Compteur eau ") + m.serial;
        dev["manufacturer"]   = "Itron";
        dev["model"]          = "EverBlu Cyble Enhanced";
    }
    serializeJson(doc, payload, sizeof(payload));
    _mqtt.publish(configTopic, payload, true);

    // --- Capteur batterie ---
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/sensor/%s_battery/config", deviceId);
    doc.clear();
    doc["name"]               = "Batterie";
    doc["unique_id"]          = String(deviceId) + "_battery";
    doc["state_topic"]        = stateTopic;
    doc["value_template"]     = "{{ value_json.battery_months }}";
    doc["unit_of_measurement"] = "mois";
    doc["device_class"]       = "battery";
    doc["icon"]               = "mdi:battery";
    {
        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"][0] = deviceId;
        dev["name"]           = String("Compteur eau ") + m.serial;
    }
    serializeJson(doc, payload, sizeof(payload));
    _mqtt.publish(configTopic, payload, true);

    // --- Binary sensor fuite ---
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/binary_sensor/%s_leak/config", deviceId);
    doc.clear();
    doc["name"]        = "Fuite détectée";
    doc["unique_id"]   = String(deviceId) + "_leak";
    doc["state_topic"] = leakTopic;
    doc["device_class"] = "moisture";
    doc["payload_on"]  = "ON";
    doc["payload_off"] = "OFF";
    {
        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"][0] = deviceId;
        dev["name"]           = String("Compteur eau ") + m.serial;
    }
    serializeJson(doc, payload, sizeof(payload));
    _mqtt.publish(configTopic, payload, true);

    log_i("Auto-discovery HA envoyé pour compteur %s", m.serial);
}

// ============================================================
//  Utilitaires
// ============================================================

MeterState* MQTTManager::_findOrCreate(const char* serial)
{
    for (uint8_t i = 0; i < _meterCount; i++) {
        if (strcmp(_meters[i].serial, serial) == 0) return &_meters[i];
    }
    if (_meterCount >= _maxMeters) return nullptr;
    MeterState* m = &_meters[_meterCount++];
    strncpy(m->serial, serial, sizeof(m->serial) - 1);
    m->serial[sizeof(m->serial) - 1] = '\0';
    m->liters         = 0;
    m->lastSeenMs     = 0;
    m->leakActive     = false;
    m->haDiscoverySent = false;
    return m;
}

void MQTTManager::_buildTopic(char* buf, size_t len, const char* serial, const char* suffix)
{
    snprintf(buf, len, "%s/%s/%s", _baseTopic, serial, suffix);
}
