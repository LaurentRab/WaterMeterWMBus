# WaterMeter wMBus — Compteurs d'eau 868 MHz → Home Assistant

Lecteur de compteurs d'eau **Wireless M-Bus** (EN 13757-4) basé sur **ESP32-C3 Super Mini** + **CC1101 868 MHz**, avec publication MQTT vers **Home Assistant**.

## Phase 1 — Découverte

Le firmware écoute en continu sur 868 MHz en alternant les modes T et S du protocole wMBus, et identifie les compteurs configurés parmi les paquets reçus.

### Modes supportés

| Mode | Fréquence | Débit | Encodage | Intervalle émission |
|------|-----------|-------|----------|---------------------|
| T-mode | 868.95 MHz | 100 kbps | 3-out-of-6 | 8–16 s |
| S-mode | 868.3 MHz | 32.768 kbps | Manchester | 2–4 min |

### Cycle de scan

1. Écoute T-mode pendant `SCAN_LISTEN_T_SEC` secondes (défaut 60)
2. Écoute S-mode pendant `SCAN_LISTEN_S_SEC` secondes (défaut 120)
3. Pause `SCAN_PAUSE_SEC` secondes (défaut 10)
4. Recommence jusqu'à ce que tous les compteurs soient trouvés

### LED (GPIO8, active LOW)

| Séquence LED | Signification |
|---|---|
| Heartbeat 1 blink/2 s | Scan en pause |
| Flash court (50 ms) | Paquet wMBus reçu |
| Flash long (500 ms) | Compteur configuré détecté |
| N blinks lents (1 Hz) | N compteurs trouvés (même mode) |
| N blinks + 1 flash court | N compteurs trouvés (modes différents T vs S) |
| Clignotement rapide 5 Hz | Aucun compteur trouvé |
| SOS | CC1101 non détecté ou self-test échoué |

## Matériel

| Composant | Référence |
|-----------|-----------|
| Microcontrôleur | ESP32-C3 Super Mini |
| Module radio | CC1101 **868 MHz** |

> **Important** : le module CC1101 doit être prévu pour **868 MHz** (antenne et circuit d'adaptation). Un module 433 MHz fonctionnera mal à 868 MHz (perte ~15-20 dB).

### Câblage CC1101 → ESP32-C3

| CC1101 | ESP32-C3 | GPIO |
|--------|----------|------|
| GDO0 | D4 | 4 |
| CSN | D5 | 5 |
| SCK | D6 | 6 |
| MOSI | D7 | 7 |
| MISO | **D10** | 10 |
| VCC | 3.3V | — |
| GND | GND | — |

> GPIO 8 est réservé à la LED intégrée de l'ESP32-C3 Super Mini (active LOW).

## Configuration

```bash
cp include/config.example.h include/config.h
cp secrets.example.ini secrets.ini
```

Éditer `include/config.h` :

| Paramètre | Description |
|-----------|-------------|
| `WIFI_SSID` / `WIFI_PASSWORD` | Réseau WiFi |
| `MQTT_SERVER` / `MQTT_PORT` | Broker MQTT (Home Assistant) |
| `MQTT_USER` / `MQTT_PASS` | Identifiants MQTT |
| `METER_COUNT` | Nombre de compteurs (1 à 4) |
| `METER_N_SERIAL` | 6 ou 8 chiffres du numéro de série |
| `SCAN_LISTEN_T_SEC` | Durée écoute T-mode par cycle (défaut 60 s) |
| `SCAN_LISTEN_S_SEC` | Durée écoute S-mode par cycle (défaut 120 s) |

### Numéro de série

Le serial wMBus est un identifiant de 8 chiffres BCD. Si vous ne connaissez que les 6 chiffres centraux de l'ancien format EverBlu, entrez-les : le firmware fera un match partiel.

Le moniteur série affiche tous les serials reçus, ce qui permet d'identifier le serial complet de chaque compteur.

## Compilation & flash

```bash
pio run --target upload
pio device monitor
```

## Topics MQTT

| Topic | Payload |
|-------|---------|
| `watermeter/scan/status` | `scanning_t` / `scanning_s` / `pause` / `complete_all_found` |
| `watermeter/scan/packets_total` | Nombre total de paquets reçus |
| `watermeter/scan/found_count` | `N/M` (trouvés/configurés) |
| `watermeter/scan/last_packet` | JSON du dernier paquet reçu |
| `watermeter/scan/<serial>/found` | `true` / `false` |
| `watermeter/scan/<serial>/mode` | `T-mode` / `S-mode` |
| `watermeter/scan/<serial>/rssi` | dBm |
| `watermeter/<serial>/detected` | JSON complet du compteur détecté |
| `watermeter/status` | LWT `online` / `offline` |

## Structure du projet

```
├── include/
│   ├── config.example.h  — template de configuration (versionné)
│   └── config.h          — paramètres utilisateur (ignoré par git)
├── lib/
│   ├── CC1101/            — driver SPI bas niveau CC1101
│   ├── WMBus/             — protocole wMBus (3of6, CRC, parsing)
│   └── MQTTManager/       — WiFi + MQTT diagnostic
├── src/
│   └── main.cpp           — boucle de scan T-mode / S-mode
└── platformio.ini
```

## Licence

MIT — voir [LICENSE](LICENSE).
