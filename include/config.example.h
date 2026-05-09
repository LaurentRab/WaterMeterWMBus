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
// ============================================================
// CONFIGURATION — copier ce fichier vers config.h et renseigner
// vos valeurs avant compilation.
//
// cp include/config.example.h include/config.h
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
// Seuls les compteurs definis ci-dessous seront actifs ;
// les slots non definis sont automatiquement desactives (serial=0).
#define METER_COUNT 2

// --- Compteurs wMBus -----------------------------------------
//
//  Le numero de serie wMBus est un identifiant de 8 chiffres BCD
//  visible sur l'etiquette du compteur.
//
//  Si vous ne connaissez que les 6 chiffres centraux de l'ancien
//  format EverBlu, entrez-les ici : le firmware fera un match
//  partiel sur les 6 derniers chiffres du serial BCD 8 chiffres.
//
//  Ne definir que les compteurs necessaires (les autres auront
//  un defaut de 0UL / "" et seront ignores).
//
#define METER_1_SERIAL  0UL        // 0 = desactive
#define METER_2_SERIAL  0UL        // 0 = desactive
// #define METER_3_SERIAL  0UL     // decommenter si METER_COUNT >= 3
// #define METER_4_SERIAL  0UL     // decommenter si METER_COUNT >= 4


// --- Cles AES-128 pour compteurs chiffres --------------------
//
// Cle de dechiffrement au format hexadecimal (32 caracteres = 16 octets).
// Laisser "" si le compteur n'est pas chiffre ou si la cle est inconnue.
// La cle est fournie par le gestionnaire du reseau d'eau.
//
// Exemple : "0102030405060708090A0B0C0D0E0F10"
//
#define METER_1_KEY "" // "" si inconnu
#define METER_2_KEY "" // "" si inconnu

// --- Scan wMBus ----------------------------------------------
// Duree d'ecoute par mode pendant chaque cycle de scan (secondes).
// T-mode : les compteurs emettent typiquement toutes les 8-16 s.
// S-mode : les compteurs emettent toutes les 2-4 min -> fenetre plus longue.
#define SCAN_LISTEN_T_SEC 120  // T-mode : 868.95 MHz, 3of6
#define SCAN_LISTEN_C_SEC 120  // C1-mode : 868.95 MHz, NRZ
#define SCAN_LISTEN_S_SEC 120  // S-mode : 868.3 MHz, Manchester 32.768 kbps
#define SCAN_LISTEN_R_SEC   0  // R2-mode : eliminé après scan nuit (0 trame valide)
#define SCAN_PAUSE_SEC 5

// --- Fuseau horaire -----------------------------------------
// Chaine POSIX TZ : le DST (heure d'ete/hiver) est gere automatiquement.
// France / Belgique / Suisse romande :
// CET-1CEST,M3.5.0,M10.5.0/3 -> UTC+1 hiver, UTC+2 ete
#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"

// --- Broches CC1101 <-> ESP32-C3 Super Mini ------------------
#define CC1101_GDO0 4
#define CC1101_CSN 5
#define CC1101_SCK 6
#define CC1101_MOSI 7
#define CC1101_MISO 10 // GPIO8 reserve a la LED integree

// --- LED integree (bleue) ------------------------------------
// GPIO8 = LED bleue de l'ESP32-C3 Super Mini (active LOW)
#define LED_PIN 8

// Watchdog : log warning si aucun paquet recu > N ms
#define WATCHDOG_TIMEOUT_MS 7200000UL // 2 heures

// --- OTA (Over-The-Air updates) -----------------------------
// Doit correspondre à secrets.ini → [secrets] ota_password
#define OTA_PASSWORD "MotDePasse"
