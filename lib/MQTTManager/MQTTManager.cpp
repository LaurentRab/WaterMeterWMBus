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
      _lastReconnectAttempt(0)
{
    _mqtt.setClient(_wifiClient);
    _mqtt.setServer(_broker, _port);
    _mqtt.setBufferSize(512);
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
    WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);
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
            WiFi.begin(_ssid, _wifiPass);
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
    } else {
        log_e("MQTT échec (rc=%d)", _mqtt.state());
    }
    return ok;
}

// ============================================================
//  Publications diagnostic wMBus
// ============================================================

void MQTTManager::publishScanStatus(const char* status)
{
    publish("watermeter/scan/status", status, true);
}

void MQTTManager::publishScanPacket(const WMBusPacket& pkt, uint32_t totalCount)
{
    if (!_mqtt.connected()) return;

    char payload[16];
    snprintf(payload, sizeof(payload), "%lu", totalCount);
    publish("watermeter/scan/packets_total", payload, true);

    char mfr[4];
    WMBus::decodeMfr(pkt.mField, mfr);

    char timestamp[32] = "unknown";
    struct tm t;
    if (getLocalTime(&t, 0))
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &t);

    char topic[80];
    snprintf(topic, sizeof(topic), "%s/scan/last_packet", _baseTopic);

    JsonDocument doc;
    doc["serial"]      = pkt.serialBCD;
    doc["manufacturer"] = mfr;
    doc["device_type"] = pkt.deviceType;
    doc["mode"]        = (pkt.mode == WMBUS_T_MODE) ? "T" : "S";
    doc["rssi"]        = pkt.rssi;
    doc["crc_ok"]      = pkt.crcOk;
    doc["timestamp"]   = timestamp;

    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    _mqtt.publish(topic, buf, false);
}

void MQTTManager::publishMeterDetected(uint32_t configSerial, const WMBusPacket& pkt)
{
    if (!_mqtt.connected()) return;

    char serialStr[12];
    snprintf(serialStr, sizeof(serialStr), "%lu", configSerial);

    char mfr[4];
    WMBus::decodeMfr(pkt.mField, mfr);

    char timestamp[32] = "unknown";
    struct tm t;
    if (getLocalTime(&t, 0))
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &t);

    char topic[80];
    snprintf(topic, sizeof(topic), "%s/%s/detected", _baseTopic, serialStr);

    JsonDocument doc;
    doc["wmbus_serial"] = pkt.serialBCD;
    doc["manufacturer"] = mfr;
    doc["device_type"]  = pkt.deviceType;
    doc["mode"]         = (pkt.mode == WMBUS_T_MODE) ? "T-mode" : "S-mode";
    doc["rssi"]         = pkt.rssi;
    doc["crc_ok"]       = pkt.crcOk;
    doc["version"]      = pkt.version;
    doc["timestamp"]    = timestamp;

    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    _mqtt.publish(topic, buf, true);
}
