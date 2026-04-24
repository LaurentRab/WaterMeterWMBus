#include <Arduino.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "CC1101.h"
#include "EverBlu.h"
#include "MQTTManager.h"

// Mode TuneFrequency : compiler avec  pio run -e tune  (défini via platformio.ini)
// Ne PAS ajouter #define TUNE_FREQUENCY ici — utiliser l'environnement tune.

// ============================================================
//  WaterMeter — ESP32-C3 + CC1101
//  Itron EverBlu Cyble Enhanced (SEDIF) → Home Assistant MQTT
//
//  Protocole : 433.82 MHz · 2-FSK · 2.4 kbps
//  Interrogation active toutes les READ_INTERVAL_MIN minutes
//  Fenêtre autorisée : 06:00 – 18:59
// ============================================================

CC1101     radio(CC1101_CSN, CC1101_GDO0, CC1101_SCK, CC1101_MOSI, CC1101_MISO);
EverBlu    everblu(radio);
MQTTManager mqtt(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS,
                 MQTT_CLIENT_ID, MQTT_BASE_TOPIC, METER_COUNT);

// Dernière interrogation réussie par compteur (millis)
uint32_t lastReadMs[METER_COUNT] = {};
uint32_t lastLeakCheckMs    = 0;
uint32_t lastWatchdogMs     = 0;
uint32_t lastMeterReadMs    = 0;  // millis() du dernier appel radio, tous compteurs confondus

// Délai minimal entre deux compteurs consécutifs (évite le chevauchement RF)
static constexpr uint32_t INTER_METER_DELAY_MS = 2000UL;

// Structure des compteurs configurés
struct MeterCfg {
    uint32_t serial;
    uint8_t  year;
    float    freqMhz;  // fréquence de réponse propre au compteur
};
static const MeterCfg METERS[METER_COUNT] = {
    { METER_1_SERIAL, METER_1_YEAR, METER_1_FREQ_MHZ },
#if METER_COUNT >= 2
    { METER_2_SERIAL, METER_2_YEAR, METER_2_FREQ_MHZ },
#endif
#if METER_COUNT >= 3
    { METER_3_SERIAL, METER_3_YEAR, METER_3_FREQ_MHZ },
#endif
#if METER_COUNT >= 4
    { METER_4_SERIAL, METER_4_YEAR, METER_4_FREQ_MHZ },
#endif
};

static void setupOTA();  // défini après le bloc TUNE_FREQUENCY

// ============================================================
//  setup()
// ============================================================

#ifdef TUNE_FREQUENCY

// ============================================================
//  LED helpers — GPIO8 actif LOW (ESP32-C3 Super Mini)
// ============================================================

static void ledOn()  { digitalWrite(LED_PIN, LOW);  }
static void ledOff() { digitalWrite(LED_PIN, HIGH); }

// N blinks bloquants à 1 Hz — utilisé uniquement dans TUNE_LED_TEST
static void ledBlink(int n) {
    for (int i = 0; i < n; i++) { ledOn(); delay(500); ledOff(); delay(500); }
}

// ============================================================
//  Résultats du scan par compteur
// ============================================================

struct TuneResult {
    bool        found;
    float       freqMhz;
    int8_t      rssi;
    EverBluData data;
};

// État global du mode Tune (machine à états dans loop())
enum TunePhase : uint8_t { TUNE_WAIT, TUNE_SCAN, TUNE_DONE };
static TunePhase  _tunePhase              = TUNE_WAIT;
static TuneResult _tuneResults[METER_COUNT];

// ============================================================
//  showResultLed() — non-bloquant, appelé depuis loop()
//
//  N blinks lents = N compteurs trouvés
//  + flash court  = fréquences différentes entre compteurs
//  5 Hz continu   = aucun compteur trouvé
// ============================================================

static void showResultLed(const TuneResult* results) {
    // Calcul systématique (max METER_COUNT itérations, trivial).
    // Pas de cache statique : foundCount/sameFreq ne sont pas des états de l'animation.
    int   foundCount = 0;
    bool  sameFreq   = true;
    float firstFreq  = 0.0f;
    for (int i = 0; i < METER_COUNT; i++) {
        if (!results[i].found) continue;
        foundCount++;
        if (firstFreq == 0.0f) firstFreq = results[i].freqMhz;
        else if (fabsf(results[i].freqMhz - firstFreq) > 0.001f) sameFreq = false;
    }

    // Log unique : ne spamme pas le moniteur série à chaque itération de loop()
    static bool logged = false;
    if (!logged) {
        log_i("LED résultat : %d/%d trouvé(s)%s", foundCount, METER_COUNT,
              foundCount > 1 ? (sameFreq ? " (même fréq.)" : " (fréq. diff.)") : "");
        logged = true;
    }

    // État de l'animation LED — statics légitimes : représentent la progression
    // de la séquence de clignotement entre deux appels successifs.
    static int      phase      = 0;  // 0=blink_on 1=blink_off 2=gap 3=flash_on 4=flash_off 5=pause
    static int      blinkDone  = 0;
    static uint32_t nextMs     = 0;

    uint32_t now = millis();
    if (now < nextMs) return;

    if (foundCount == 0) {
        // 5 Hz : toggle toutes les 100 ms
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
        else { blinkDone = 0; phase = sameFreq ? 5 : 2; }
        break;
    case 2:                         nextMs = now + 200; phase = 3; break;  // gap
    case 3: ledOn();                nextMs = now + 150; phase = 4; break;  // flash
    case 4: ledOff();               nextMs = now;       phase = 5; break;
    case 5:                         nextMs = now + 3000; phase = 0; break; // pause
    }
}

// ============================================================
//  publishTuneResults() — MQTT après scan
// ============================================================

static void publishTuneResults(const TuneResult* results) {
    char topic[80], payload[256];
    char timestamp[32] = "unknown";
    struct tm t;
    if (getLocalTime(&t, 0))
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &t);

    int foundCount = 0;
    for (int i = 0; i < METER_COUNT; i++) if (results[i].found) foundCount++;

    mqtt.publish("watermeter/tune/status",
                 foundCount > 0 ? "complete_found" : "complete_not_found", true);
    mqtt.publish("watermeter/tune/timestamp", timestamp, true);
    snprintf(payload, sizeof(payload), "%d/%d", foundCount, METER_COUNT);
    mqtt.publish("watermeter/tune/found_count", payload, true);

    for (int i = 0; i < METER_COUNT; i++) {
        if (METERS[i].serial == 0) continue;
        char serial[12];
        snprintf(serial, sizeof(serial), "%lu", METERS[i].serial);

        snprintf(topic, sizeof(topic), "watermeter/tune/%s/found", serial);
        mqtt.publish(topic, results[i].found ? "true" : "false", true);
        if (!results[i].found) continue;

        snprintf(topic, sizeof(topic), "watermeter/tune/%s/freq_mhz", serial);
        snprintf(payload, sizeof(payload), "%.3f", results[i].freqMhz);
        mqtt.publish(topic, payload, true);

        snprintf(topic, sizeof(topic), "watermeter/tune/%s/rssi", serial);
        snprintf(payload, sizeof(payload), "%d", (int)results[i].rssi);
        mqtt.publish(topic, payload, true);

        mqtt.publishEverBlu(METERS[i].serial, results[i].data, 0);
    }
    log_i("Résultats publiés → watermeter/tune/");
}

// ============================================================
//  runTuneScan() — bloquant mais court (~4s/freq × 31 = ~2 min)
//  Chaque delay() interne yielde le scheduler FreeRTOS.
// ============================================================

static void runTuneScan(TuneResult* results) {
    const int   STEP_START = 0;
    const int   STEP_END   = 30;
    const float FREQ_BASE  = 433.750f;
    const float FREQ_STEP  = 0.005f;

    log_i("=== Scan démarré : %.3f–%.3f MHz ===",
          FREQ_BASE, FREQ_BASE + STEP_END * FREQ_STEP);

    radio.configureEverBlu();  // une seule fois — setFrequency() suffit ensuite

    for (int s = STEP_START; s <= STEP_END; s++) {
        float freq = FREQ_BASE + s * FREQ_STEP;
        log_i("--- %.3f MHz (%d/%d) ---", freq, s+1, STEP_END+1);
        ledOn(); delay(50); ledOff();

        radio.setFrequency(freq);

        mqtt.loop();  // maintient le keep-alive MQTT pendant le scan bloquant
        for (int i = 0; i < METER_COUNT; i++) {
            if (METERS[i].serial == 0 || results[i].found) continue;
            EverBluData data;
            if (everblu.request(METERS[i].serial, METERS[i].year, data)) {
                log_i(">>> SUCCES compteur %d : %.3f MHz | %lu L | batt=%u | RSSI=%d",
                      i+1, freq, data.liters, data.battery, data.rssi);
                log_i("    → METER_%d_FREQ_MHZ  %.3ff", i+1, freq);
                results[i] = { true, freq, data.rssi, data };
            }
        }

        bool allFound = true;
        for (int i = 0; i < METER_COUNT; i++)
            if (METERS[i].serial != 0 && !results[i].found) { allFound = false; break; }
        if (allFound) { log_i("Tous trouvés — scan terminé."); break; }
    }
}

// ============================================================
//  Initialisation Tune dans setup() — NE PAS BLOQUER
// ============================================================

static void initTune() {
    memset(_tuneResults, 0, sizeof(_tuneResults));
    pinMode(LED_PIN, OUTPUT);
    ledOff();

    log_i("============================================");
    log_i("  MODE TuneFrequency");
    log_i("  Scan 433.750–433.900 MHz | fenêtre %02d:00–%02d:00 | masque 0x%02X",
          TUNE_HOUR_START, TUNE_HOUR_END, TUNE_DAYS_MASK);
    log_i("============================================");

    mqtt.begin(WIFI_SSID, WIFI_PASSWORD);
    configTzTime(TIMEZONE, "pool.ntp.org", "time.nist.gov");
    {
        struct tm t; uint32_t ts = millis();
        while (!getLocalTime(&t, 0) && millis() - ts < 10000) delay(500);
        struct tm tc;
        if (getLocalTime(&tc, 0)) {
            char buf[32];
            strftime(buf, sizeof(buf), "%a %d/%m/%Y %H:%M:%S", &tc);
            log_i("NTP : %s", buf);
        }
        else                      log_w("NTP non synchro");
    }

#ifdef TUNE_LED_TEST
    log_i("=== TEST LED ===");
    log_i("Heartbeat (attente)...");
    for (int i = 0; i < 4; i++) { ledOn(); delay(100); ledOff(); delay(1900); }

    log_i("Aucune réponse (5 Hz)...");
    { uint32_t t0=millis(); while(millis()-t0<3000){ledOn();delay(100);ledOff();delay(100);} }
    delay(1000);

    for (int n = 1; n <= METER_COUNT; n++) {
        log_i("%d compteur(s), même fréquence...", n);
        for (int r = 0; r < 3; r++) { ledBlink(n); delay(3000); }
        log_i("%d compteur(s), fréquences différentes...", n);
        for (int r = 0; r < 3; r++) {
            ledBlink(n); delay(200); ledOn(); delay(150); ledOff(); delay(3000);
        }
    }
    log_i("=== TEST LED terminé ===");
#endif

    if (!radio.begin()) {
        log_e("CC1101 non détecté — signal SOS LED");
        while (true) {  // SOS : ... --- ...
            for (int i=0;i<3;i++){ledOn();delay(200);ledOff();delay(200);}
            for (int i=0;i<3;i++){ledOn();delay(600);ledOff();delay(200);}
            for (int i=0;i<3;i++){ledOn();delay(200);ledOff();delay(200);}
            delay(2000);
        }
    }
    radio.configureEverBlu();
    if (!radio.selfTest()) {
        log_e("Self-Test FAILED — signal SOS LED");
        while (true) {
            for (int i=0;i<3;i++){ledOn();delay(200);ledOff();delay(200);}
            for (int i=0;i<3;i++){ledOn();delay(600);ledOff();delay(200);}
            for (int i=0;i<3;i++){ledOn();delay(200);ledOff();delay(200);}
            delay(2000);
        }
    }

    mqtt.publish("watermeter/tune/status", "waiting", true);
    log_i("Attente de la fenêtre %02d:00–%02d:00...", TUNE_HOUR_START, TUNE_HOUR_END);
    // Retour immédiat — l'attente se fait dans loop()
}

#endif  // TUNE_FREQUENCY

// ============================================================
//  OTA — commun aux deux modes (Tune et production)
//  Appelé après mqtt.begin() uniquement si WiFi est connecté.
// ============================================================

static void setupOTA()
{
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        const char* type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        log_i("OTA : début — %s", type);
    });
    ArduinoOTA.onEnd([]() {
        log_i("OTA : terminé — redémarrage");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static uint8_t lastPct = 0xFF;
        uint8_t pct = (uint8_t)(progress * 100u / total);
        if (pct != lastPct) { log_i("OTA : %3u%%", pct); lastPct = pct; }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        const char* msg;
        switch (error) {
            case OTA_AUTH_ERROR:    msg = "authentification (mauvais mot de passe)"; break;
            case OTA_BEGIN_ERROR:   msg = "begin (flash insuffisant ?)";              break;
            case OTA_CONNECT_ERROR: msg = "connexion";                                break;
            case OTA_RECEIVE_ERROR: msg = "réception";                                break;
            case OTA_END_ERROR:     msg = "fin (vérification échouée)";               break;
            default:                msg = "inconnue";                                 break;
        }
        log_e("OTA erreur [%u] : %s", error, msg);
    });

    ArduinoOTA.begin();
    log_i("OTA prêt — hostname : %s.local | IP : %s", MQTT_CLIENT_ID,
          WiFi.localIP().toString().c_str());
}

void setup()
{
#if CORE_DEBUG_LEVEL > 0
    delay(5000);  // Laisse le temps à l'USB-JTAG de s'énumérer (debug uniquement)
#endif

#ifdef TUNE_FREQUENCY
    initTune();
    return;
#else
    log_i("==============================");
    log_i("  WaterMeter v2.0  EverBlu");
    log_i("  ESP32-C3 + CC1101 433.82 MHz");
    log_i("==============================");

    // Initialisation CC1101
    if (!radio.begin()) {
        log_e("CC1101 non détecté — redémarrage dans 5 s");
        delay(5000);
        ESP.restart();
    }
    radio.configureEverBlu();
    radio.selfTest();

    // WiFi + MQTT (WiFi nécessaire avant configTime)
    mqtt.begin(WIFI_SSID, WIFI_PASSWORD);

    // NTP : configurer après connexion WiFi
    // configTzTime gère le DST automatiquement via la chaîne POSIX TIMEZONE (config.h)
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
            log_w("NTP non synchro — fenêtre horaire ignorée jusqu'à synchro");
        }
    }

    for (int i = 0; i < METER_COUNT; i++)
        log_i("Compteur %d : serial=%lu année=%u freq=%.3f MHz",
              i+1, METERS[i].serial, METERS[i].year, METERS[i].freqMhz);
    log_i("Intervalle : %u min (%.1f h) | Fenêtre : 06:00–18:59", READ_INTERVAL_MIN, READ_INTERVAL_MIN / 60.0f);

    // Première lecture immédiate (sans attendre l'intervalle)
    for (int i = 0; i < METER_COUNT; i++)
        lastReadMs[i] = millis() - (uint32_t)READ_INTERVAL_MIN * 60000UL;
#endif  // TUNE_FREQUENCY
}

// ============================================================
//  loop()
// ============================================================

void loop()
{
    // Initialise OTA dès que WiFi est disponible, même si le boot s'est fait sans réseau
    static bool otaReady = false;
    if (!otaReady && WiFi.status() == WL_CONNECTED) {
        setupOTA();
        otaReady = true;
    }

    ArduinoOTA.handle();

#ifdef TUNE_FREQUENCY
    mqtt.loop();

    switch (_tunePhase) {

    case TUNE_WAIT: {
        // Heartbeat LED non-bloquant : 100 ms ON toutes les 2 s
        static uint32_t _heartbeatNext = 0;
        static bool     _heartbeatOn   = false;
        uint32_t now = millis();
        if (now >= _heartbeatNext) {
            _heartbeatOn = !_heartbeatOn;
            _heartbeatOn ? ledOn() : ledOff();
            _heartbeatNext = now + (_heartbeatOn ? 100 : 1900);
        }

        // Vérification fenêtre horaire
        struct tm t;
        if (!getLocalTime(&t, 0)) break;  // NTP pas encore synchro

        bool inHour = (t.tm_hour >= TUNE_HOUR_START && t.tm_hour < TUNE_HOUR_END);
        bool inDay  = (TUNE_DAYS_MASK >> t.tm_wday) & 1;
        if (inHour && inDay) {
            ledOff();
            log_i("Fenêtre atteinte (%02d:%02d) — lancement du scan", t.tm_hour, t.tm_min);
            _tunePhase = TUNE_SCAN;
        }
        break;
    }

    case TUNE_SCAN:
        runTuneScan(_tuneResults);
        mqtt.loop();  // déclenche la reconnexion si MQTT a coupé pendant le scan
        delay(500);   // laisse le temps à la reconnexion de s'établir
        mqtt.loop();
        publishTuneResults(_tuneResults);
        _tunePhase = TUNE_DONE;
        break;

    case TUNE_DONE:
        showResultLed(_tuneResults);
        break;
    }
    return;
#endif

    mqtt.loop();

    uint32_t now = millis();
    uint32_t intervalMs = (uint32_t)READ_INTERVAL_MIN * 60000UL;

    // --- Interrogation périodique de chaque compteur --------
    for (int i = 0; i < METER_COUNT; i++) {
        if (METERS[i].serial == 0) continue;             // Compteur non configuré
        if (now - lastReadMs[i] < intervalMs) continue;  // Pas encore l'heure

        // Espacement inter-compteurs non-bloquant : si un autre compteur vient
        // d'être interrogé, on reporte à la prochaine itération de loop().
        if (i > 0 && now - lastMeterReadMs < INTER_METER_DELAY_MS) continue;

        if (!EverBlu::withinTimeWindow()) {
            // Hors fenêtre : on reporte sans log répété
            lastReadMs[i] = now;
            continue;
        }

        log_i("--- Interrogation compteur %d (serial=%lu, %.3f MHz) ---",
              i + 1, METERS[i].serial, METERS[i].freqMhz);
        radio.setFrequency(METERS[i].freqMhz);

        EverBluData data;
        if (everblu.request(METERS[i].serial, METERS[i].year, data)) {
            log_i("Compteur %d : %lu L | batterie=%u mois | RSSI=%d dBm | lectures=%u",
                  i + 1, data.liters, data.battery, data.rssi, data.readCount);
            mqtt.publishEverBlu(METERS[i].serial, data, LEAK_THRESHOLD_L);
        } else {
            log_w("Compteur %d : aucune réponse", i + 1);
        }

        lastReadMs[i]    = now;
        lastMeterReadMs  = now;  // marque le dernier appel radio pour l'espacement inter-compteurs
    }

    // --- Détection de fuite (toutes les 5 min) --------------
    if (now - lastLeakCheckMs > 300000UL) {
        lastLeakCheckMs = now;
        mqtt.checkLeaks();
    }

    // --- Watchdog (toutes les heures) -----------------------
    if (now - lastWatchdogMs > 3600000UL) {
        lastWatchdogMs = now;
        mqtt.checkWatchdog(WATCHDOG_TIMEOUT_MS);
    }

    delay(100);
}
