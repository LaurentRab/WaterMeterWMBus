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

#include <Arduino.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "CC1101.h"
#include "WMBus.h"
#include "MQTTManager.h"
#include "WMBusParserBridge.h"


// ============================================================
//  WaterMeter — ESP32-C3 + CC1101
//  Wireless M-Bus (EN 13757-4) — écoute passive 868 MHz
// ============================================================

CC1101            radio(CC1101_CSN, CC1101_GDO0, CC1101_SCK, CC1101_MOSI, CC1101_MISO);
WMBus             wmbus(radio);
MQTTManager       mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS,
                       MQTT_CLIENT_ID, MQTT_BASE_TOPIC, METER_COUNT);
WMBusParserBridge parser;

static constexpr uint32_t SCAN_T_MS  = (uint32_t)SCAN_LISTEN_T_SEC * 1000UL;
static constexpr uint32_t SCAN_C1_MS = (uint32_t)SCAN_LISTEN_C_SEC * 1000UL;
static constexpr uint32_t SCAN_S_MS  = (uint32_t)SCAN_LISTEN_S_SEC * 1000UL;
static constexpr uint32_t SCAN_R_MS  = (uint32_t)SCAN_LISTEN_R_SEC * 1000UL;
static constexpr uint32_t PAUSE_MS   = (uint32_t)SCAN_PAUSE_SEC    * 1000UL;
static constexpr uint8_t  R2_CHANNELS = 10;
static constexpr uint32_t R2_PER_CHAN_MS = SCAN_R_MS / R2_CHANNELS;

struct MeterCfg {
    uint32_t serial;
};

// ***** ATTENTION ****** BUG ICI ET A CORRIGER. Il faut autant d'entréesdans MeterCfg que 'METER_COUNT' 
static const MeterCfg METERS[METER_COUNT] = {
    { METER_1_SERIAL },
#if METER_COUNT >= 2
    { METER_2_SERIAL },
#endif

};

struct MeterStat {
    bool      found;
    uint32_t  wmbusSerialBCD;
    int8_t    bestRssi;
    uint16_t  count;
    WMBusMode mode;
    char      mfr[4];
    uint8_t   deviceType;
};
static MeterStat meterStats[METER_COUNT] = {};
static uint32_t totalPackets = 0;

// État global du scan (machine à états dans loop())
enum ScanPhase : uint8_t { SCAN_T, SCAN_C1, SCAN_S, SCAN_R, SCAN_PAUSE, SCAN_DONE };
static ScanPhase  scanPhase = SCAN_T;
static uint32_t   phaseDeadline = 0;
static uint8_t    r2Channel = 0;       // canal R2 courant (0–9)

// Diagnostic RF — réinitialisé à chaque cycle de scan
static int8_t   rfDiagRssiMin   = 0;
static int8_t   rfDiagRssiMax   = 0;
static int32_t  rfDiagRssiSum   = 0;
static uint16_t rfDiagRssiN     = 0;
static uint16_t rfDiagMarcFault = 0;

// ============================================================
//  LED helpers — GPIO8 actif LOW (ESP32-C3 Super Mini)
// ============================================================

static void ledOn()  { digitalWrite(LED_PIN, LOW);  }
static void ledOff() { digitalWrite(LED_PIN, HIGH); }

static void ledBlink(int n) {
    for (int i = 0; i < n; i++) { ledOn(); delay(500); ledOff(); delay(500); }
}

[[noreturn]] static void ledSOS() {
    for (;;) {
        for (int i = 0; i < 3; i++) { ledOn(); delay(200); ledOff(); delay(200); }
        for (int i = 0; i < 3; i++) { ledOn(); delay(600); ledOff(); delay(200); }
        for (int i = 0; i < 3; i++) { ledOn(); delay(200); ledOff(); delay(200); }
        delay(2000);
    }
}

// ============================================================
//  Diagnostic RF — échantillonnage RSSI + MARCSTATE
// ============================================================

static void resetRfDiag() {
    rfDiagRssiMin = rfDiagRssiMax = rfDiagRssiSum = 0;
    rfDiagRssiN = rfDiagMarcFault = 0;
    wmbus.resetSyncCount();
}

static void sampleRfDiag() {
    int8_t r = radio.readRSSI();
    rfDiagRssiN++;
    rfDiagRssiSum += r;
    if (rfDiagRssiN == 1) { rfDiagRssiMin = r; rfDiagRssiMax = r; }
    else {
        if (r < rfDiagRssiMin) rfDiagRssiMin = r;
        if (r > rfDiagRssiMax) rfDiagRssiMax = r;
    }
    // Après listen(), le radio est en IDLE (0x01). Toute autre valeur = anomalie.
    if (radio.marcstate() != CC1101_STATE_IDLE) rfDiagMarcFault++;
}

static void reportRfDiag(const char* tag) {
    if (rfDiagRssiN == 0) return;
    int8_t avg = (int8_t)(rfDiagRssiSum / (int32_t)rfDiagRssiN);
    uint32_t syncs = wmbus.syncCount();
    log_i("RF diag [%s] RSSI min=%d max=%d moy=%d dBm / %u mesures / %lu syncs",
          tag, rfDiagRssiMin, rfDiagRssiMax, avg, rfDiagRssiN, syncs);
    if (syncs == 0)
        log_w("  RF diag : aucun sync word détecté — compteurs hors portée ou fréquence décalée");
    if (rfDiagMarcFault > 0)
        log_w("  RF diag : MARCSTATE hors IDLE %u fois (instabilité chip ?)", rfDiagMarcFault);
    if (rfDiagRssiMax < -95)
        log_w("  RF diag : RSSI max %d dBm — très bas, antenne ou clone défaillant ?", rfDiagRssiMax);
    else if (rfDiagRssiMax >= -75)
        log_i("  RF diag : RSSI pic %d dBm — activité RF 868 MHz détectée", rfDiagRssiMax);
}

// ============================================================
//  showResultLed() — non-bloquant, appelé depuis loop()
//
//  N blinks lents = N compteurs trouvés
//  + flash court  = modes différents entre compteurs (T vs S)
//  5 Hz continu   = aucun compteur trouvé
// ============================================================

static void showResultLed() {
    int  foundCount = 0;
    bool sameMode   = true;
    WMBusMode firstMode = WMBUS_T_MODE;
    for (int i = 0; i < METER_COUNT; i++) {
        if (!meterStats[i].found) continue;
        foundCount++;
        if (foundCount == 1) firstMode = meterStats[i].mode;
        else if (meterStats[i].mode != firstMode) sameMode = false;
    }

    static bool logged = false;
    if (!logged) {
        log_i("LED résultat : %d/%d trouvé(s)%s", foundCount, METER_COUNT,
              foundCount > 1 ? (sameMode ? " (même mode)" : " (modes diff.)") : "");
        logged = true;
    }

    static int      phase      = 0;
    static int      blinkDone  = 0;
    static uint32_t nextMs     = 0;

    uint32_t now = millis();
    if (now < nextMs) return;

    if (foundCount == 0) {
        static bool on = false;
        on = !on; on ? ledOn() : ledOff();
        nextMs = now + 100;
        return;
    }

    switch (phase) {
    case 0: ledOn();                nextMs = now + 500; phase = 1; break;
    case 1:
        ledOff();                   nextMs = now + 500;
        if (++blinkDone < foundCount) { phase = 0; }
        else { blinkDone = 0; phase = sameMode ? 5 : 2; }
        break;
    case 2:                         nextMs = now + 200; phase = 3; break;
    case 3: ledOn();                nextMs = now + 150; phase = 4; break;
    case 4: ledOff();               nextMs = now;       phase = 5; break;
    case 5:                         nextMs = now + 3000; phase = 0; break;
    }
}

// ============================================================
//  Match serial : vérifie si le BCD 8 chiffres contient le
//  serial configuré (6 ou 8 chiffres)
// ============================================================

static bool matchSerial(uint32_t wmbusBCD, uint32_t configured)
{
    if (configured == 0) return false;
    if (wmbusBCD == configured) return true;
    uint32_t last6 = wmbusBCD % 1000000UL;
    return (last6 == configured);
}

// ============================================================
//  OTA
// ============================================================

static void setupOTA()
{
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        log_i("OTA : début — %s",
              (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem");
    });
    ArduinoOTA.onEnd([]()   { log_i("OTA : terminé — redémarrage"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static uint8_t lastPct = 0xFF;
        uint8_t pct = (uint8_t)(progress * 100u / total);
        if (pct != lastPct) { log_i("OTA : %3u%%", pct); lastPct = pct; }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        const char* msg;
        switch (error) {
            case OTA_AUTH_ERROR:    msg = "authentification"; break;
            case OTA_BEGIN_ERROR:   msg = "begin";            break;
            case OTA_CONNECT_ERROR: msg = "connexion";        break;
            case OTA_RECEIVE_ERROR: msg = "réception";        break;
            case OTA_END_ERROR:     msg = "fin";              break;
            default:                msg = "inconnue";         break;
        }
        log_e("OTA erreur [%u] : %s", error, msg);
    });

    ArduinoOTA.begin();
    log_i("OTA prêt — %s.local | %s", MQTT_CLIENT_ID, WiFi.localIP().toString().c_str());
}

static void publishResults();  // déclaration anticipée

// ============================================================
//  Traitement d'un paquet wMBus reçu
// ============================================================

static void handlePacket(const WMBusPacket& pkt)
{
    totalPackets++;

    char mfr[4];
    WMBus::decodeMfr(pkt.mField, mfr);

    const char* modeName = (pkt.mode == WMBUS_T_MODE) ? "T" :
                           (pkt.mode == WMBUS_C_MODE) ? "C1" :
                           (pkt.mode == WMBUS_R_MODE) ? "R2" : "S";
    log_i("wMBus [%s-mode] serial=%08lu mfr=%s type=0x%02X RSSI=%d CRC=%s",
          modeName, pkt.serialBCD, mfr, pkt.deviceType, pkt.rssi,
          pkt.crcOk ? "OK" : "FAIL");

    ledOn(); delay(50); ledOff();

    bool matched = false;
    for (int i = 0; i < METER_COUNT; i++) {
        if (!matchSerial(pkt.serialBCD, METERS[i].serial)) continue;
        matched = true;

        MeterStat& s = meterStats[i];
        s.found = true;
        s.wmbusSerialBCD = pkt.serialBCD;
        s.count++;
        s.mode = pkt.mode;
        s.deviceType = pkt.deviceType;
        memcpy(s.mfr, mfr, 4);
        if (s.count == 1 || pkt.rssi > s.bestRssi)
            s.bestRssi = pkt.rssi;

        log_i("*********************************************");
        log_i("  COMPTEUR %d DETECTE !", i + 1);
        log_i("  serial=%08lu  mfr=%s  %s-mode", pkt.serialBCD, mfr, modeName);
        log_i("  RSSI=%d dBm  CRC=%s  count=%u", pkt.rssi, pkt.crcOk ? "OK" : "FAIL", s.count);

        MeterReading reading;
        if (parser.parse(pkt, reading)) {
            if (reading.valid) {
                log_i("  CONSOMMATION = %.3f m3", reading.total_m3);
                if (reading.target_m3 > 0.0)
                    log_i("  Releve precedent = %.3f m3", reading.target_m3);
                mqtt.publishMeterReading(METERS[i].serial, reading, pkt);
            } else if (reading.encrypted && !reading.decrypted) {
                log_w("  Trame chiffree — tentative cles connues...");
                if (parser.tryKnownKeys(pkt, reading)) {
                    log_i("  *** CLE AES TROUVEE pour compteur %d ! ***", i + 1);
                    log_i("  *** Cle = %s ***", reading.foundKey);
                    log_i("  CONSOMMATION = %.3f m3", reading.total_m3);
                    if (reading.target_m3 > 0.0)
                        log_i("  Releve precedent = %.3f m3", reading.target_m3);
                    mqtt.publishKeyFound(METERS[i].serial, reading.foundKey);
                    mqtt.publishMeterReading(METERS[i].serial, reading, pkt);
                } else {
                    log_w("  Aucune cle connue ne fonctionne — cle individuelle requise");
                }
            } else {
                log_w("  Payload non decode");
            }
        }

        log_i("*********************************************");

        ledOn(); delay(500); ledOff();

        mqtt.publishMeterDetected(METERS[i].serial, pkt);
    }

    mqtt.publishScanPacket(pkt, totalPackets);

    // Arrêt immédiat dès qu'un compteur configuré est détecté
    if (matched && scanPhase != SCAN_DONE) {
        log_i("=== Compteur détecté — scan terminé ===");
        publishResults();
        scanPhase = SCAN_DONE;
    }
}

// ============================================================
//  Publication du bilan après un cycle complet
// ============================================================

static void publishResults()
{
    char topic[80], payload[256];
    char timestamp[32] = "unknown";
    struct tm t;
    if (getLocalTime(&t, 0))
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &t);

    int foundCount = 0;
    for (int i = 0; i < METER_COUNT; i++) if (meterStats[i].found) foundCount++;

    snprintf(topic, sizeof(topic), "%s/scan/status", MQTT_BASE_TOPIC);
    mqtt.publish(topic,
                 foundCount == METER_COUNT ? "complete_all_found" :
                 foundCount > 0 ? "complete_partial" : "complete_none", true);
    snprintf(topic, sizeof(topic), "%s/scan/timestamp", MQTT_BASE_TOPIC);
    mqtt.publish(topic, timestamp, true);
    snprintf(payload, sizeof(payload), "%d/%d", foundCount, METER_COUNT);
    snprintf(topic, sizeof(topic), "%s/scan/found_count", MQTT_BASE_TOPIC);
    mqtt.publish(topic, payload, true);
    snprintf(payload, sizeof(payload), "%lu", totalPackets);
    snprintf(topic, sizeof(topic), "%s/scan/packets_total", MQTT_BASE_TOPIC);
    mqtt.publish(topic, payload, true);

    for (int i = 0; i < METER_COUNT; i++) {
        if (METERS[i].serial == 0) continue;
        char serial[12];
        snprintf(serial, sizeof(serial), "%lu", METERS[i].serial);

        snprintf(topic, sizeof(topic), "%s/scan/%s/found", MQTT_BASE_TOPIC, serial);
        mqtt.publish(topic, meterStats[i].found ? "true" : "false", true);
        if (!meterStats[i].found) continue;

        snprintf(topic, sizeof(topic), "%s/scan/%s/mode", MQTT_BASE_TOPIC, serial);
        mqtt.publish(topic, meterStats[i].mode == WMBUS_T_MODE ? "T-mode" : "S-mode", true);

        snprintf(topic, sizeof(topic), "%s/scan/%s/rssi", MQTT_BASE_TOPIC, serial);
        snprintf(payload, sizeof(payload), "%d", (int)meterStats[i].bestRssi);
        mqtt.publish(topic, payload, true);

        snprintf(topic, sizeof(topic), "%s/scan/%s/mfr", MQTT_BASE_TOPIC, serial);
        mqtt.publish(topic, meterStats[i].mfr, true);

        snprintf(topic, sizeof(topic), "%s/scan/%s/wmbus_serial", MQTT_BASE_TOPIC, serial);
        snprintf(payload, sizeof(payload), "%08lu", meterStats[i].wmbusSerialBCD);
        mqtt.publish(topic, payload, true);

        snprintf(topic, sizeof(topic), "%s/scan/%s/count", MQTT_BASE_TOPIC, serial);
        snprintf(payload, sizeof(payload), "%u", meterStats[i].count);
        mqtt.publish(topic, payload, true);
    }
    log_i("Résultats publiés → %s/scan/", MQTT_BASE_TOPIC);
}

// ============================================================
//  setup()
// ============================================================

void setup()
{
#if CORE_DEBUG_LEVEL > 0
    // Délai nécessaire pour permettre l'initialisation de la console
    delay(5000);
#endif

    pinMode(LED_PIN, OUTPUT);
    ledOff();

    log_i("==============================");
    log_i("  WaterMeter v4.0  wMBus");
    log_i("  ESP32-C3 + CC1101 868 MHz");
    log_i("==============================");

    if (strlen(METER_1_KEY) == 32) parser.setKey(METER_1_SERIAL, METER_1_KEY);
#if METER_COUNT >= 2
    if (strlen(METER_2_KEY) == 32) parser.setKey(METER_2_SERIAL, METER_2_KEY);
#endif

    if (!radio.begin()) {
        log_e("CC1101 non détecté — SOS LED");
        ledSOS();
    }

    radio.configureWMBusTMode();
    if (!radio.selfTest()) {
        log_e("Self-Test FAILED — SOS LED");
        ledSOS();
    }

    mqtt.begin(WIFI_SSID, WIFI_PASSWORD);

    configTzTime(TIMEZONE, "pool.ntp.org", "time.nist.gov");
    {
        struct tm t;
        uint32_t ts = millis();
        while (!getLocalTime(&t, 0) && millis() - ts < 10000) delay(500);
        char buf[32];
        if (getLocalTime(&t, 0)) {
            strftime(buf, sizeof(buf), "%a %d/%m/%Y %H:%M:%S", &t);
            log_i("NTP synchro : %s", buf);
        } else {
            log_w("NTP non synchro");
        }
    }

    for (int i = 0; i < METER_COUNT; i++)
        log_i("Compteur %d : serial=%lu (match partiel 6 chiffres)", i + 1, METERS[i].serial);

    log_i("Scan : T-mode %us + C1-mode %us + S-mode %us + R2-mode %us (%u canaux) + pause %us",
          SCAN_LISTEN_T_SEC, SCAN_LISTEN_C_SEC, SCAN_LISTEN_S_SEC,
          SCAN_LISTEN_R_SEC, R2_CHANNELS, SCAN_PAUSE_SEC);

    for (int i = 0; i < METER_COUNT; i++) {
        if (METERS[i].serial != 0)
            mqtt.publishHADiscovery(METERS[i].serial);
    }

    if (SCAN_T_MS > 0) {
        mqtt.publishScanStatus("scanning_t");
        scanPhase = SCAN_T;
        phaseDeadline = millis() + SCAN_T_MS;
    } else if (SCAN_C1_MS > 0) {
        mqtt.publishScanStatus("scanning_c");
        scanPhase = SCAN_C1;
        phaseDeadline = millis() + SCAN_C1_MS;
    } else if (SCAN_S_MS > 0) {
        mqtt.publishScanStatus("scanning_s");
        scanPhase = SCAN_S;
        phaseDeadline = millis() + SCAN_S_MS;
    } else if (SCAN_R_MS > 0) {
        r2Channel = 0;
        radio.configureWMBusRMode(0);
        mqtt.publishScanStatus("scanning_r2a");
        log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
        scanPhase = SCAN_R;
        phaseDeadline = millis() + R2_PER_CHAN_MS;
    } else {
        mqtt.publishScanStatus("pause");
        scanPhase = SCAN_PAUSE;
        phaseDeadline = millis() + PAUSE_MS;
    }
}

// ============================================================
//  loop()
// ============================================================

void loop()
{
    static bool otaReady = false;
    if (!otaReady && WiFi.status() == WL_CONNECTED) {
        setupOTA();
        otaReady = true;
    }

    ArduinoOTA.handle();
    mqtt.loop();

    switch (scanPhase) {

    case SCAN_T: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_T_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();  // RSSI + MARCSTATE après chaque fenêtre d'écoute
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan T-mode ---");
            reportRfDiag("T-mode");

            // Test D : sniffer brut (une seule fois, si aucun paquet wMBus reçu)
            static bool rawSniffDone = false;
            if (!rawSniffDone && totalPackets == 0) {
                rawSniffDone = true;
                log_i("Sniffer brut sans sync word — 2 s...");
                uint16_t n = radio.rawSniff(2000);
                log_i("Sniffer brut : %u octets reçus en 2 s", n);
                if (n < 100)
                    log_w("  → Très peu d'octets — chaîne RF défaillante (antenne ou clone ?)");
                else if (n > 2000)
                    log_i("  → Démodulateur RF fonctionnel (%u octets)", n);
                else
                    log_i("  → Signal RF faible mais présent (%u octets)", n);
            }

            publishResults();
            if (SCAN_C1_MS > 0) {
                resetRfDiag();
                mqtt.publishScanStatus("scanning_c");
                scanPhase = SCAN_C1;
                phaseDeadline = millis() + SCAN_C1_MS;
            } else if (SCAN_S_MS > 0) {
                resetRfDiag();
                mqtt.publishScanStatus("scanning_s");
                scanPhase = SCAN_S;
                phaseDeadline = millis() + SCAN_S_MS;
            } else if (SCAN_R_MS > 0) {
                resetRfDiag();
                r2Channel = 0;
                radio.configureWMBusRMode(0);
                mqtt.publishScanStatus("scanning_r2a");
                log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                scanPhase = SCAN_R;
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE;
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_C1: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_C_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan C1-mode ---");
            reportRfDiag("C1-mode");
            publishResults();
            if (SCAN_S_MS > 0) {
                resetRfDiag();
                mqtt.publishScanStatus("scanning_s");
                scanPhase = SCAN_S;
                phaseDeadline = millis() + SCAN_S_MS;
            } else if (SCAN_R_MS > 0) {
                resetRfDiag();
                r2Channel = 0;
                radio.configureWMBusRMode(0);
                mqtt.publishScanStatus("scanning_r2a");
                log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                scanPhase = SCAN_R;
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE;
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_S: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_S_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan S-mode ---");
            reportRfDiag("S-mode");
            publishResults();
            if (SCAN_R_MS > 0) {
                resetRfDiag();
                r2Channel = 0;
                radio.configureWMBusRMode(0);
                mqtt.publishScanStatus("scanning_r2a");
                log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                scanPhase = SCAN_R;
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE;
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_R: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_R_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            char tag[16];
            snprintf(tag, sizeof(tag), "R2-%c", 'a' + r2Channel);
            reportRfDiag(tag);

            r2Channel++;
            if (r2Channel < R2_CHANNELS) {
                resetRfDiag();
                radio.configureWMBusRMode(r2Channel);
                wmbus.resetSyncCount();
                char status[16];
                snprintf(status, sizeof(status), "scanning_r2%c", 'a' + r2Channel);
                mqtt.publishScanStatus(status);
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                log_i("--- Fin scan R2-mode (10 canaux) ---");
                publishResults();
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE;
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_PAUSE: {
        // Heartbeat LED pendant la pause
        static uint32_t nextBeat = 0;
        static bool beatOn = false;
        if (millis() >= nextBeat) {
            beatOn = !beatOn;
            beatOn ? ledOn() : ledOff();
            nextBeat = millis() + (beatOn ? 100 : 1900);
        }

        if (millis() >= phaseDeadline) {
            ledOff();
            log_i("--- Nouveau cycle de scan ---");
            resetRfDiag();
            mqtt.publishScanStatus("scanning_t");
            scanPhase = SCAN_T;
            phaseDeadline = millis() + SCAN_T_MS;
        }
        break;
    }

    case SCAN_DONE:
        showResultLed();
        break;
    }
}
