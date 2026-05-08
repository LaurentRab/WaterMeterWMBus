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

// Format B (ista/Qundis) plus probable → 1/3 Format A, 2/3 Format B
static constexpr uint32_t SCAN_T_A_MS  = SCAN_T_MS  / 3;
static constexpr uint32_t SCAN_T_B_MS  = SCAN_T_MS  - SCAN_T_A_MS;
static constexpr uint32_t SCAN_C1_A_MS = SCAN_C1_MS / 3;
static constexpr uint32_t SCAN_C1_B_MS = SCAN_C1_MS - SCAN_C1_A_MS;
static constexpr uint32_t SCAN_S_A_MS  = SCAN_S_MS  / 3;
static constexpr uint32_t SCAN_S_B_MS  = SCAN_S_MS  - SCAN_S_A_MS;

struct MeterCfg {
    uint32_t serial;
};

#ifndef METER_2_SERIAL
#define METER_2_SERIAL 0UL
#endif
#ifndef METER_2_KEY
#define METER_2_KEY ""
#endif
#ifndef METER_3_SERIAL
#define METER_3_SERIAL 0UL
#endif
#ifndef METER_3_KEY
#define METER_3_KEY ""
#endif
#ifndef METER_4_SERIAL
#define METER_4_SERIAL 0UL
#endif
#ifndef METER_4_KEY
#define METER_4_KEY ""
#endif

static_assert(METER_COUNT >= 1 && METER_COUNT <= 4,
              "METER_COUNT doit etre entre 1 et 4");

static const MeterCfg METERS[METER_COUNT] = {
    { METER_1_SERIAL },
#if METER_COUNT >= 2
    { METER_2_SERIAL },
#endif
#if METER_COUNT >= 3
    { METER_3_SERIAL },
#endif
#if METER_COUNT >= 4
    { METER_4_SERIAL },
#endif
};

struct MeterStat {
    bool      found;
    uint32_t  wmbusSerialBCD;
    int8_t    bestRssi;
    uint16_t  count;
    char      mfr[4];
    uint8_t   deviceType;
    char      phase[8];        // "T-A", "T-B", "C1-B", "S-A", etc.
};
static MeterStat meterStats[METER_COUNT] = {};
static uint32_t totalPackets = 0;

// État global du scan (machine à états dans loop())
enum ScanPhase : uint8_t {
    SCAN_T_A, SCAN_T_B,
    SCAN_C1_A, SCAN_C1_B,
    SCAN_S_A, SCAN_S_B,
    SCAN_POLL, SCAN_R,
    SCAN_PAUSE, SCAN_DONE
};
static ScanPhase  scanPhase = SCAN_T_A;
static uint32_t   phaseDeadline = 0;
static uint8_t    r2Channel = 0;       // canal R2 courant (0–9)
static const char* phaseTag = "T-A";   // label lisible de la phase courante

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
    wmbus.resetRejectCount();
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
    uint32_t rejects = wmbus.rejectCount();
    log_i("RF diag [%s] RSSI min=%d max=%d moy=%d dBm / %u mesures / %lu syncs / %lu rejects",
          tag, rfDiagRssiMin, rfDiagRssiMax, avg, rfDiagRssiN, syncs, rejects);
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
    bool samePhase  = true;
    const char* firstPhase = "";
    for (int i = 0; i < METER_COUNT; i++) {
        if (!meterStats[i].found) continue;
        foundCount++;
        if (foundCount == 1) firstPhase = meterStats[i].phase;
        else if (strcmp(meterStats[i].phase, firstPhase) != 0) samePhase = false;
    }

    static bool logged = false;
    if (!logged) {
        log_i("LED résultat : %d/%d trouvé(s)%s", foundCount, METER_COUNT,
              foundCount > 1 ? (samePhase ? " (même phase)" : " (phases diff.)") : "");
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
        else { blinkDone = 0; phase = samePhase ? 5 : 2; }
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

    log_i("wMBus [%s] serial=%08lu mfr=%s type=0x%02X RSSI=%d CRC=%s",
          phaseTag, pkt.serialBCD, mfr, pkt.deviceType, pkt.rssi,
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
        s.deviceType = pkt.deviceType;
        memcpy(s.mfr, mfr, 4);
        strncpy(s.phase, phaseTag, sizeof(s.phase) - 1);
        s.phase[sizeof(s.phase) - 1] = '\0';
        if (s.count == 1 || pkt.rssi > s.bestRssi)
            s.bestRssi = pkt.rssi;

        log_i("##################################################");
        log_i("##                                              ##");
        log_i("##   >>> COMPTEUR %d DETECTE ! <<<               ##", i + 1);
        log_i("##                                              ##");
        log_i("##   Serial   : %08lu                       ##", pkt.serialBCD);
        log_i("##   Fabricant: %s                             ##", mfr);
        log_i("##   Phase    : %s                            ##", phaseTag);
        log_i("##   RSSI     : %d dBm                        ##", pkt.rssi);
        log_i("##   Type     : 0x%02X                          ##", pkt.deviceType);
        log_i("##   CI       : 0x%02X                          ##", pkt.ciField);
        log_i("##   CRC      : %s                            ##", pkt.crcOk ? "OK" : "FAIL");
        log_i("##                                              ##");

        MeterReading reading;
        if (parser.parse(pkt, reading)) {
            if (reading.valid) {
                log_i("##   CONSO    : %.3f m3                   ##", reading.total_m3);
                if (reading.target_m3 > 0.0)
                    log_i("##   Precedent: %.3f m3                   ##", reading.target_m3);
                mqtt.publishMeterReading(METERS[i].serial, reading, pkt);
            } else if (reading.encrypted && !reading.decrypted) {
                log_i("##   Trame CHIFFREE                         ##");
                if (parser.tryKnownKeys(pkt, reading)) {
                    log_i("##   >>> CLE AES TROUVEE ! <<<              ##");
                    log_i("##   Cle = %s  ##", reading.foundKey);
                    log_i("##   CONSO = %.3f m3                      ##", reading.total_m3);
                    mqtt.publishKeyFound(METERS[i].serial, reading.foundKey);
                    mqtt.publishMeterReading(METERS[i].serial, reading, pkt);
                } else {
                    log_w("##   Aucune cle connue — cle requise        ##");
                }
            } else {
                log_w("##   Payload non decode                     ##");
            }
        }

        log_i("##                                              ##");
        log_i("##################################################");

        // Hex dump des premiers octets pour analyse
        char hex[128];
        int hLen = 0;
        int show = (pkt.dataLen > 30) ? 30 : pkt.dataLen;
        for (int j = 0; j < show && hLen < 120; j++)
            hLen += snprintf(hex + hLen, sizeof(hex) - hLen, "%02X ", pkt.data[j]);
        log_i("RAW [%d octets]: %s", pkt.dataLen, hex);

        ledOn(); delay(500); ledOff();

        mqtt.publishMeterDetected(METERS[i].serial, pkt);
    }

    mqtt.publishScanPacket(pkt, totalPackets);

    if (matched && scanPhase != SCAN_DONE) {
        log_i("=== SCAN TERMINE — phase gagnante : %s ===", phaseTag);
        log_i("=== Pour cibler ce mode, configurer uniquement cette phase ===");
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
        mqtt.publish(topic, meterStats[i].phase, true);

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
#if METER_COUNT >= 3
    if (strlen(METER_3_KEY) == 32) parser.setKey(METER_3_SERIAL, METER_3_KEY);
#endif
#if METER_COUNT >= 4
    if (strlen(METER_4_KEY) == 32) parser.setKey(METER_4_SERIAL, METER_4_KEY);
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
        mqtt.publishScanStatus("scanning_t_a");
        scanPhase = SCAN_T_A; phaseTag = "T-A";
        phaseDeadline = millis() + SCAN_T_A_MS;
    } else if (SCAN_C1_MS > 0) {
        mqtt.publishScanStatus("scanning_c1a");
        scanPhase = SCAN_C1_A; phaseTag = "C1-A";
        phaseDeadline = millis() + SCAN_C1_A_MS;
    } else if (SCAN_S_MS > 0) {
        mqtt.publishScanStatus("scanning_s_a");
        scanPhase = SCAN_S_A; phaseTag = "S-A";
        phaseDeadline = millis() + SCAN_S_A_MS;
    } else if (SCAN_R_MS > 0) {
        r2Channel = 0;
        radio.configureWMBusRMode(0);
        mqtt.publishScanStatus("scanning_r2a");
        log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
        scanPhase = SCAN_R; phaseTag = "R2";
        phaseDeadline = millis() + R2_PER_CHAN_MS;
    } else {
        mqtt.publishScanStatus("pause");
        scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
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

    case SCAN_T_A: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_T_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            reportRfDiag("T-A");
            resetRfDiag();
            log_i("T-mode : switch → Format B (0xF68D, NRZ/C-mode)");
            mqtt.publishScanStatus("scanning_t_b");
            scanPhase = SCAN_T_B; phaseTag = "T-B";
            phaseDeadline = millis() + SCAN_T_B_MS;
        }
        break;
    }

    case SCAN_T_B: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_C_MODE, listenMs, pkt, 0xF68D, true))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan T-mode (A+B) ---");
            reportRfDiag("T-B");

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
                mqtt.publishScanStatus("scanning_c1a");
                scanPhase = SCAN_C1_A; phaseTag = "C1-A";
                phaseDeadline = millis() + SCAN_C1_A_MS;
            } else if (SCAN_S_MS > 0) {
                resetRfDiag();
                mqtt.publishScanStatus("scanning_s_a");
                scanPhase = SCAN_S_A; phaseTag = "S-A";
                phaseDeadline = millis() + SCAN_S_A_MS;
            } else if (SCAN_R_MS > 0) {
                resetRfDiag();
                r2Channel = 0;
                radio.configureWMBusRMode(0);
                mqtt.publishScanStatus("scanning_r2a");
                log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                scanPhase = SCAN_R; phaseTag = "R2";
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_C1_A: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_C_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            reportRfDiag("C1-A");
            resetRfDiag();
            log_i("C1-mode : switch → Format B (0xF68D)");
            mqtt.publishScanStatus("scanning_c1b");
            scanPhase = SCAN_C1_B; phaseTag = "C1-B";
            phaseDeadline = millis() + SCAN_C1_B_MS;
        }
        break;
    }

    case SCAN_C1_B: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_C_MODE, listenMs, pkt, 0xF68D))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan C1-mode (A+B) ---");
            reportRfDiag("C1-B");
            publishResults();
            if (SCAN_S_MS > 0) {
                resetRfDiag();
                mqtt.publishScanStatus("scanning_s_a");
                scanPhase = SCAN_S_A; phaseTag = "S-A";
                phaseDeadline = millis() + SCAN_S_A_MS;
            } else if (SCAN_R_MS > 0) {
                resetRfDiag();
                r2Channel = 0;
                radio.configureWMBusRMode(0);
                mqtt.publishScanStatus("scanning_r2a");
                log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                scanPhase = SCAN_R; phaseTag = "R2";
                phaseDeadline = millis() + R2_PER_CHAN_MS;
            } else {
                mqtt.publishScanStatus("pause");
                scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
                phaseDeadline = millis() + PAUSE_MS;
            }
        }
        break;
    }

    case SCAN_S_A: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_S_MODE, listenMs, pkt))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            reportRfDiag("S-A");
            resetRfDiag();
            log_i("S-mode : switch → Format B (0xF68D)");
            mqtt.publishScanStatus("scanning_s_b");
            scanPhase = SCAN_S_B; phaseTag = "S-B";
            phaseDeadline = millis() + SCAN_S_B_MS;
        }
        break;
    }

    case SCAN_S_B: {
        WMBusPacket pkt;
        uint32_t remaining = (millis() < phaseDeadline) ? (phaseDeadline - millis()) : 0;
        uint32_t listenMs = (remaining > 2000) ? 2000 : remaining;

        if (listenMs > 0) {
            if (wmbus.listen(WMBUS_S_MODE, listenMs, pkt, 0xF68D))
                handlePacket(pkt);
            sampleRfDiag();
        }

        if (scanPhase != SCAN_DONE && millis() >= phaseDeadline) {
            log_i("--- Fin scan S-mode (A+B) ---");
            reportRfDiag("S-B");
            publishResults();
            // Polling désactivé : clone CC1101 (VERSION=0x04) bloqué en STARTCAL,
            // TX impossible. Passe directement en pause pour maximiser l'écoute.
            mqtt.publishScanStatus("pause");
            scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
            phaseDeadline = millis() + PAUSE_MS;
        }
        break;
    }

    case SCAN_POLL: {
        static uint8_t pollMeter = 0;
        static uint8_t pollVer = 0;
        static uint8_t pollStep = 0;   // 0=wildcard FF/FF, 1=FF/07, 2-17=ver 0x00-0x0F
        static WMBusMode pollMode = WMBUS_S_MODE;

        uint32_t serial = METERS[pollMeter].serial;
        if (serial == 0) { pollMeter++; pollStep = 0; }

        if (pollMeter >= METER_COUNT) {
            if (pollMode == WMBUS_S_MODE) {
                pollMode = WMBUS_C_MODE;
                pollMeter = 0; pollStep = 0;
                log_i("--- Polling : switch vers C1-mode ---");
            } else {
                log_i("--- Fin polling (aucune réponse) ---");
                pollMeter = 0; pollStep = 0;
                pollMode = WMBUS_S_MODE;
                if (SCAN_R_MS > 0) {
                    resetRfDiag();
                    r2Channel = 0;
                    radio.configureWMBusRMode(0);
                    mqtt.publishScanStatus("scanning_r2a");
                    log_i("--- Début scan R2-mode (10 canaux × %lus) ---", R2_PER_CHAN_MS / 1000);
                    scanPhase = SCAN_R; phaseTag = "R2";
                    phaseDeadline = millis() + R2_PER_CHAN_MS;
                } else {
                    mqtt.publishScanStatus("pause");
                    scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
                    phaseDeadline = millis() + PAUSE_MS;
                }
                break;
            }
        }

        uint8_t ver, typ;
        if (pollStep == 0)      { ver = 0xFF; typ = 0xFF; }
        else if (pollStep == 1) { ver = 0xFF; typ = 0x07; }
        else                    { ver = pollStep - 2; typ = 0x07; }

        const char* modeStr = (pollMode == WMBUS_S_MODE) ? "S" : "C1";
        log_i("POLL %s ser=%lu ver=%02X typ=%02X", modeStr, serial, ver, typ);

        WMBusPacket resp;
        if (wmbus.poll(pollMode, serial, 0x2674, ver, typ, 500, resp)) {
            char mfr[4];
            WMBus::decodeMfr(resp.mField, mfr);
            log_w("POLL RÉPONSE! L=%02X C=%02X M=%s ser=%08lu RSSI=%d dBm",
                  resp.lField, resp.cField, mfr, resp.serialBCD, resp.rssi);
            handlePacket(resp);
        }

        pollStep++;
        if (pollStep >= 18) { pollStep = 0; pollMeter++; }
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
                scanPhase = SCAN_PAUSE; phaseTag = "PAUSE";
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
            mqtt.publishScanStatus("scanning_t_a");
            scanPhase = SCAN_T_A; phaseTag = "T-A";
            phaseDeadline = millis() + SCAN_T_A_MS;
        }
        break;
    }

    case SCAN_DONE:
        showResultLed();
        break;
    }
}
