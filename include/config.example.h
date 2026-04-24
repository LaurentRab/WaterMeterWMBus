#pragma once

// ============================================================
//  CONFIGURATION — copier ce fichier vers config.h et renseigner
//  vos valeurs avant compilation.
//
//    cp include/config.example.h include/config.h
// ============================================================

// --- WiFi ---------------------------------------------------
#define WIFI_SSID     "MonReseau"
#define WIFI_PASSWORD "MonMotDePasse"

// --- MQTT (broker Home Assistant) ---------------------------
#define MQTT_SERVER    "192.168.x.x"
#define MQTT_PORT      1883
#define MQTT_USER      "mqtt-user"
#define MQTT_PASS      "mqtt-pass"
#define MQTT_CLIENT_ID "watermeter_esp32"
#define MQTT_BASE_TOPIC "watermeter"

// --- Nombre de compteurs ------------------------------------
// Adapter METER_COUNT au nombre reel de compteurs (1 a 4).
// Definir autant de paires METER_N_SERIAL / METER_N_YEAR que necessaire.
#define METER_COUNT  2

// --- Compteurs Itron EverBlu Cyble Enhanced ------------------
//
//  Le numero imprime sur le module suit le format :
//    [2 chiffres usine][6 chiffres serial][1 chiffre controle]
//  Exemple : 843561553 -> usine=84, serial=356155, controle=3
//
//  METER_x_SERIAL = les 6 chiffres centraux
//  METER_x_YEAR   = les 2 derniers chiffres de l'annee de fabrication
//                   (ex: 19 pour 2019, indique par "19399" sur l'etiquette)
//
//  Si aucune reponse : essaie METER_x_SERIAL avec les 8 chiffres
//  sans le controle (ex: 84356155UL) — certains modules utilisent
//  un format different.
//
#define METER_1_SERIAL  0UL        // 0 = desactive
#define METER_1_YEAR    0

#define METER_2_SERIAL  0UL        // 0 = desactive
#define METER_2_YEAR    0

// Decommenter et renseigner pour 3 ou 4 compteurs :
// #define METER_3_SERIAL  0UL
// #define METER_3_YEAR    0
// #define METER_4_SERIAL  0UL
// #define METER_4_YEAR    0

// --- Frequence CC1101 ----------------------------------------
// Valeur nominale EverBlu : 433.82 MHz
// Chaque compteur peut avoir une legere derive : [433.76 - 433.89]
// Utiliser le mode TuneFrequency pour trouver la frequence exacte de chaque compteur.
#define CC1101_FREQ_MHZ  433.82f   // frequence par defaut

#define METER_1_FREQ_MHZ  433.82f  // frequence compteur 1 (ajuster apres TuneFrequency)
#define METER_2_FREQ_MHZ  433.82f  // frequence compteur 2 (ajuster apres TuneFrequency)
// #define METER_3_FREQ_MHZ  433.82f
// #define METER_4_FREQ_MHZ  433.82f

// --- Fuseau horaire -----------------------------------------
// Chaîne POSIX TZ : le DST (heure d'été/hiver) est géré automatiquement.
// France / Belgique / Suisse romande :
//   CET-1CEST,M3.5.0,M10.5.0/3  → UTC+1 hiver, UTC+2 été
// Autres exemples :
//   GMT0BST,M3.5.0/1,M10.5.0    → Royaume-Uni
//   EST5EDT,M3.2.0,M11.1.0      → Est USA
#define TIMEZONE  "CET-1CEST,M3.5.0,M10.5.0/3"

// --- Interrogation -------------------------------------------
// Intervalle entre deux lectures (minutes)
// Le compteur ne repond qu'entre 06:00 et 18:59 !
#define READ_INTERVAL_MIN  240

// --- Mode TuneFrequency : fenetre de scan -------------------
// La carte attend que l'heure et le jour correspondent, puis scanne.
#define TUNE_HOUR_START   10      // heure de debut (incluse)
#define TUNE_HOUR_END     16      // heure de fin (exclue)
//                          _  Sam Ven Jeu Mer Mar Lun Dim
#define TUNE_DAYS_MASK  0b00111110  // Lun-Ven

// --- Broches CC1101 <-> ESP32-C3 Super Mini ------------------
#define CC1101_GDO0   4
#define CC1101_CSN    5
#define CC1101_SCK    6
#define CC1101_MOSI   7
#define CC1101_MISO   10   // GPIO8 reserve a la LED integree

// --- LED integree (bleue) ------------------------------------
// GPIO8 = LED bleue de l'ESP32-C3 Super Mini (active LOW)
#define LED_PIN       8

// --- Detection de fuite -------------------------------------
// Consommation nocturne maximale toleree (litres).
#define LEAK_THRESHOLD_L  20

// Watchdog : log warning si un compteur est silencieux > N ms
#define WATCHDOG_TIMEOUT_MS  7200000UL  // 2 heures

// --- OTA (Over-The-Air updates) -----------------------------
// Doit correspondre à secrets.ini → [secrets] ota_password
#define OTA_PASSWORD  "change_me"
