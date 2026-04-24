/*
 Copyright (C) 2026 Laurent Rabret (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "../WMBus/WMBus.h"
#include "../WMBusParser/WMBusParserBridge.h"

static constexpr uint8_t MQTT_MAX_METERS = 4;

// ============================================================
//  Gestion WiFi + MQTT — Phase 1 diagnostic wMBus
//
//  Topics publiés :
//   watermeter/scan/status          → "scanning_t" / "scanning_s" / "idle"
//   watermeter/scan/packets_total   → compteur paquets
//   watermeter/scan/<serial>/found  → "true" / "false"
//   watermeter/<serial>/detected    → JSON diagnostic
//   watermeter/status               → LWT "online" / "offline"
// ============================================================

class MQTTManager {
public:
    MQTTManager(const char* broker, uint16_t port,
                const char* user, const char* password,
                const char* clientId, const char* baseTopic,
                uint8_t meterCount);

    void begin(const char* ssid, const char* wifiPass);
    void loop();

    void publishScanStatus(const char* status);
    void publishScanPacket(const WMBusPacket& pkt, uint32_t totalCount);
    void publishMeterDetected(uint32_t configSerial, const WMBusPacket& pkt);
    void publishMeterReading(uint32_t configSerial, const MeterReading& reading,
                             const WMBusPacket& pkt);
    void publishHADiscovery(uint32_t configSerial);
    void publishKeyFound(uint32_t configSerial, const char* hexKey);

    bool connected() { return _mqtt.connected(); }
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

    uint8_t      _maxMeters;
    uint32_t     _lastReconnectAttempt;

    bool _reconnect();
};
