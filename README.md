# WaterMeter — Itron EverBlu Cyble Enhanced → Home Assistant

Lecture automatique des compteurs d'eau **Itron EverBlu Cyble Enhanced (SEDIF / Actaris P40)** via radio 433 MHz, publiée en MQTT vers **Home Assistant**.

## Matériel

| Composant | Référence |
|-----------|-----------|
| Microcontrôleur | ESP32-C3 Super Mini |
| Module radio | CC1101 (433 MHz) |

### Câblage CC1101 ↔ ESP32-C3

| CC1101 | ESP32-C3 | Notes |
|--------|----------|-------|
| GDO0   | GPIO 4   | signal de synchronisation trame |
| CSN    | GPIO 5   | SPI Chip Select |
| SCK    | GPIO 6   | SPI Clock |
| MOSI   | GPIO 7   | SPI Data in |
| MISO   | **GPIO 10** | GPIO 8 réservé à la LED intégrée |
| VCC    | 3.3 V    | |
| GND    | GND      | |

> **Important** : GPIO 8 est la LED bleue intégrée de l'ESP32-C3 Super Mini (active LOW).
> Connecter MISO sur **GPIO 10** (et non GPIO 8).

## Protocole

- **Fréquence** : 433.82 MHz nominale — chaque compteur peut dériver légèrement (433.76 – 433.89 MHz)
- **Modulation** : 2-FSK · 2.4 kbps · déviation 5.157 kHz
- **Séquence** : wake-up ~2.5 s → requête 39 octets → ACK 18 octets → données 4× oversampled 9.6 kbps
- **Fenêtre active** : les compteurs ne répondent qu'entre **06:00 et 18:59**

## Configuration

Copier le fichier d'exemple puis renseigner vos valeurs :

```bash
cp include/config.example.h include/config.h
```

Éditer `include/config.h` :

| Paramètre | Description |
|-----------|-------------|
| `WIFI_SSID` / `WIFI_PASSWORD` | Réseau WiFi |
| `MQTT_SERVER` / `MQTT_PORT` | Broker MQTT (Home Assistant) |
| `MQTT_USER` / `MQTT_PASS` | Identifiants MQTT |
| `METER_COUNT` | Nombre de compteurs (1 à 4) |
| `METER_N_SERIAL` | 6 chiffres centraux du numéro de série |
| `METER_N_YEAR` | Année de fabrication (2 chiffres, ex: `19`) |
| `METER_N_FREQ_MHZ` | Fréquence propre au compteur (trouver avec `tune`) |
| `READ_INTERVAL_MIN` | Intervalle entre lectures (minutes, défaut 60) |
| `LEAK_THRESHOLD_L` | Seuil de fuite nocturne (litres, défaut 20) |

Ce fichier est ignoré par git (secrets).

### Trouver le numéro de série

Le numéro imprimé sur le module suit le format `[2 chiffres usine][6 chiffres serial][1 chiffre contrôle]`.
Exemple : `843561553` → `METER_N_SERIAL = 356155`, `METER_N_YEAR = 19` (indiqué par `19399` sur l'étiquette).

## Compilation & flash

Projet **PlatformIO**. Environnements disponibles :

| Environnement | Usage |
|---------------|-------|
| `watermeter` | Firmware normal (production) |
| `tune` | Scan de fréquence — attend la fenêtre horaire configurée, puis scanne |
| `tune_led_test` | Idem `tune` + déroule toutes les séquences LED au démarrage |

```bash
# Firmware normal
pio run -e watermeter --target upload

# Mode calibration fréquence
pio run -e tune --target upload

# Mode calibration + test LED
pio run -e tune_led_test --target upload
```

## Mode TuneFrequency

Chaque compteur peut avoir une légère dérive fréquentielle. Le mode `tune` scanne automatiquement **433.750 → 433.900 MHz par pas de 5 kHz** (31 fréquences) et publie les résultats.

### Fonctionnement

Le firmware attend la fenêtre horaire configurée (`TUNE_HOUR_START` / `TUNE_HOUR_END` / `TUNE_DAYS_MASK`) de façon **non-bloquante** (machine à états dans `loop()`). Pendant l'attente, la LED clignote en heartbeat lent (100 ms toutes les 2 s).

Une fois la fenêtre atteinte, le scan démarre (~2 min pour 31 fréquences × 2 compteurs). Les résultats sont ensuite publiés en MQTT et la LED indique le résultat :

| Séquence LED | Signification |
|---|---|
| Heartbeat 1 blink/2 s | Attente de la fenêtre horaire |
| N blinks lents (1 Hz) | N compteurs trouvés (même fréquence) |
| N blinks + 1 flash court | N compteurs trouvés (fréquences différentes) |
| Clignotement rapide 5 Hz | Aucun compteur trouvé |

### Après le scan

Le moniteur série affiche :

```
>>> SUCCES compteur 1 : 433.820 MHz | 123456 L | batt=42 | RSSI=-65
    → METER_1_FREQ_MHZ  433.820f
```

Recopier la valeur dans `config.h`, puis recompiler avec l'environnement `watermeter`.

### Topics MQTT du scan

| Topic | Contenu |
|-------|---------|
| `watermeter/tune/status` | `waiting` → `complete_found` / `complete_not_found` |
| `watermeter/tune/found_count` | ex: `2/2` |
| `watermeter/tune/<serial>/found` | `true` / `false` |
| `watermeter/tune/<serial>/freq_mhz` | fréquence trouvée (ex: `433.820`) |
| `watermeter/tune/<serial>/rssi` | RSSI en dBm |

## Topics MQTT (mode normal)

| Topic | Contenu |
|-------|---------|
| `watermeter/<serial>/state` | JSON (voir ci-dessous) |
| `watermeter/<serial>/leak` | `ON` / `OFF` |
| `watermeter/<serial>/availability` | `offline` si silencieux > `WATCHDOG_TIMEOUT_MS` |
| `watermeter/status` | `online` / `offline` (LWT) |

Exemple de payload `state` :

```json
{
  "liters": 123456,
  "m3": 123.456,
  "delta_l": 12,
  "battery_months": 42,
  "read_count": 1234,
  "rssi": -65,
  "timestamp": "2025-06-01T08:30:00"
}
```

**Home Assistant Auto-Discovery** est activé : les entités (volume m³, batterie, fuite) apparaissent automatiquement dans HA.

## Détection de fuite

Après chaque nuit (saut > 4 h entre deux lectures), la consommation nocturne est comparée à `LEAK_THRESHOLD_L`. Si elle est dépassée, `leak = ON` est publié en MQTT. L'état est re-publié toutes les 5 minutes tant que la condition persiste.

## Watchdog

Un topic `availability = offline` est publié si un compteur est silencieux depuis plus de `WATCHDOG_TIMEOUT_MS` (défaut : 2 h).

## Structure du projet

```
├── include/
│   ├── config.example.h  — template de configuration (versionné)
│   └── config.h          — paramètres utilisateur (ignoré par git)
├── lib/
│   ├── CC1101/            — driver SPI bas niveau (registres, GDO0, FIFO)
│   ├── EverBlu/           — protocole Itron EverBlu (wake-up, trames, CRC)
│   └── MQTTManager/       — WiFi + MQTT + HA auto-discovery
├── src/
│   └── main.cpp           — boucle principale + mode TuneFrequency
└── platformio.ini
```

## Crédits

Protocole EverBlu basé sur le reverse-engineering de [psykokwak-com/everblu-meters-esp8266](https://github.com/psykokwak-com/everblu-meters-esp8266).

## Licence

MIT — voir [LICENSE](LICENSE).
