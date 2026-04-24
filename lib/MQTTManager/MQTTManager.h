#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "../EverBlu/EverBlu.h"
// config.h n'est PAS inclus ici : le nombre de compteurs est passé
// au constructeur, ce qui rend MQTTManager indépendant du projet hôte.

// Capacité maximale absolue (limite matérielle du protocole EverBlu)
static constexpr uint8_t MQTT_MAX_METERS = 4;

// ============================================================
//  Gestion WiFi + MQTT avec Home Assistant Auto-Discovery
//
//  Topics publiés par compteur :
//   watermeter/<serial>/state      → JSON { liters, battery, rssi, timestamp }
//   watermeter/<serial>/leak       → "ON" / "OFF"
//
//  Home Assistant Auto-Discovery :
//   homeassistant/sensor/watermeter_<serial>_volume/config
//   homeassistant/binary_sensor/watermeter_<serial>_leak/config
// ============================================================

struct MeterState {
    char     serial[12];     // numéro affiché (ex: "356155")
    uint32_t liters;         // dernier volume connu (L)
    uint32_t lastSeenMs;     // millis() de la dernière lecture réussie
    bool     leakActive;
    bool     haDiscoverySent;
};

class MQTTManager {
public:
    // meterCount : nombre de compteurs configurés (1–MQTT_MAX_METERS)
    MQTTManager(const char* broker, uint16_t port,
                const char* user, const char* password,
                const char* clientId, const char* baseTopic,
                uint8_t meterCount);

    void begin(const char* ssid, const char* wifiPass);
    void loop();

    // Publie les données d'un compteur EverBlu.
    // leakThresholdL : seuil de consommation nocturne (L) au-delà duquel une fuite est signalée.
    void publishEverBlu(uint32_t serial, const EverBluData& d, uint32_t leakThresholdL);

    // Re-publie l'état de fuite courant (utile après reconnexion MQTT).
    void checkLeaks();

    // Watchdog : alerte si un compteur est silencieux
    void checkWatchdog(uint32_t timeoutMs);

    bool connected() { return _mqtt.connected(); }

    // Publication directe (pour les topics tune/)
    bool publish(const char* topic, const char* payload, bool retained = false);

private:
    WiFiClient   _wifiClient;
    PubSubClient _mqtt;

    const char*  _broker;
    uint16_t     _port;
    const char*  _user;
    const char*  _pass;
    const char*  _clientId;
    const char*  _baseTopic;
    const char*  _ssid;
    const char*  _wifiPass;

    MeterState   _meters[MQTT_MAX_METERS];  // tableau à capacité maximale fixe
    uint8_t      _maxMeters;               // nombre effectif configuré (≤ MQTT_MAX_METERS)
    uint8_t      _meterCount;              // nombre de slots utilisés (découverts au runtime)
    uint32_t     _lastReconnectAttempt;

    MeterState*  _findOrCreate(const char* serial);
    bool         _reconnect();
    void         _publishDiscovery(const MeterState& m);
    void         _buildTopic(char* buf, size_t len, const char* serial, const char* suffix);
};
