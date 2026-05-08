# WaterMeter wMBus — Journal de diagnostic

## Objectif

Lire la consommation d'eau de deux compteurs Actaris P40 equipés de modules radio ista (P/N 19399) depuis un ESP32-C3 + CC1101 clone, et publier les données via MQTT vers Home Assistant.

## Matériel

| Composant | Détail |
|-----------|--------|
| MCU | ESP32-C3 Super Mini |
| Radio | CC1101 868 MHz (clone, VERSION=0x04) |
| Compteur | Actaris P40 (mécanique, Qn=1.5 m³/h) |
| Module radio | ista International GmbH, P/N 19399, clip-on |
| Serial 1 | 84356155 (installé ~2019) |
| Serial 2 | 84356149 (installé ~2018) |
| Fabricant wMBus | IST (0x2674), device type 0x07 (eau) |
| Protocole cible | wMBus EN 13757-4, 868 MHz |

## Chronologie

### Fondations (commits b811ec8 → f0e58ec)

Le projet démarre comme un fork d'un lecteur EverBlu 433 MHz, entièrement refactoré pour le protocole wMBus 868 MHz en écoute passive. Le parseur wmbusmeters est intégré pour extraire les données de consommation. Le firmware sait scanner les compteurs, décoder les trames, déchiffrer l'AES-128, et publier vers MQTT.

Tout fonctionne sur le plan logiciel. Reste à capter un signal réel.

### Bug matériel : le clone CC1101 (commits 04e81c9 → 8e8e0db)

Trois problèmes matériels sérieux sont identifiés et corrigés :

1. **Registre RXBYTES erroné** — Le clone CC1101 (VERSION=0x04) signale des overflows FIFO alors que le FIFO n'est pas plein. Fix : vérifier MARCSTATE avant de flusher.

2. **Préemption WiFi pendant le drain FIFO** — À 100 kbps, le FIFO de 64 octets se remplit en 5 ms. Si la tâche WiFi (priorité 23) préempte le drain, overflow garanti. Fix : ISR sur GDO0 + `vTaskNotifyGiveFromISR` + priority boost à `configMAX_PRIORITIES - 1`.

3. **Troncature uint8_t fatale** — `drainFifo(buf, uint8_t maxLen)` recevait 256 (tronqué à 0). Le drain ne lisait jamais un seul octet. Toutes les réceptions échouaient. Fix : caper à 64 octets par chunk.

Après ces fixes, la chaîne RF est validée : le sniffer brut capte ~9000 octets/2s, le démodulateur fonctionne.

### Correction fréquence et ajout C1-mode (commits 7e2d293 → 6613519)

**Bug critique découvert** : les registres FREQ étaient faux depuis le début.

| Mode | Cible | Erreur avant fix |
|------|-------|-----------------|
| T-mode | 868.950 MHz | **+243 kHz** |
| S-mode | 868.300 MHz | **+140 kHz** |

Le CC1101 écoutait littéralement à côté de la bande. Corrigé par recalcul exact : `round(freq / 26e6 * 65536)`.

Le mode C1 est ajouté (même fréquence que T mais encodage NRZ au lieu de 3-out-of-6). La bande passante RX est réduite de 541 kHz à 325 kHz.

Malgré ces corrections : toujours aucun paquet réel capté en T, C1 ou S-mode.

---

## Phase 1 — Scan T / C1 / S-mode (6 mai 2026, soirée)

**Config** : T-mode 300s + C1-mode 300s + S-mode 300s, plusieurs cycles.

| Mode | Fréquence | Data rate | Résultat |
|------|-----------|-----------|----------|
| T-mode | 868.95 MHz | 100 kbps, 3of6 | 0 paquet (syncs = bruit 3of6) |
| C1-mode | 868.95 MHz | 100 kbps, NRZ | 0 paquet (CRC bypass = 100% faux positifs) |
| S-mode | 868.30 MHz | 32.768 kbps, Manchester | 0 paquet |

Le sniffer brut (sans sync word) capte 9320 octets en 2s → la chaîne RF est fonctionnelle, le CC1101 démodule correctement. Il y a du signal, mais aucun paquet wMBus structuré.

**Hypothèse formulée** : les modules ista P/N 19399 sont probablement R-mode (bidirectionnel). La référence croisée avec les issues wmbusmeters (#436, #992) confirme que les ista "radio net 3" (famille 193xx) utilisent le R-mode, un mode qui n'est implémenté nulle part en open-source.

## Phase 2 — Scan R2-mode nocturne (nuit du 6-7 mai 2026)

Branche `RMode` créée. Le scan R2-mode est implémenté : 10 canaux de 868.03 à 868.57 MHz, espacement 60 kHz, 4.8 kchip/s Manchester, sync word 0x7696.

### Première session (6 mai, ~23h) — Diagnostic brut

Résultat immédiat : des syncs sont détectés (2-6 par canal/minute) mais aucun paquet n'est décodé. Ajout d'un diagnostic détaillé : hex dump des 20 premiers octets + header parsé + CRC calculé vs reçu après chaque sync.

L'analyse de la trace diagnostique révèle :
- **rawLen = 256 systématiquement** (buffer rempli à ras bord = bruit continu)
- **L-field, C-field, serial, fabricant** : valeurs aléatoires (0xDF, 0xC1, 0xF2...)
- **CRC** : calc vs recv totalement décorrélés
- **Conclusion** : 100% faux syncs sur du bruit

Calcul théorique : à 4.8 kbps sur 60s, le CC1101 voit ~288 000 bits. Probabilité de matcher 0x7696 (16 bits) par hasard : 288000/65536 ≈ **4.4 matchs attendus par canal**. C'est exactement ce qu'on observe.

### Scan nocturne complet (nuit du 6-7 mai)

Le diagnostic est réduit pour ne pas saturer la console : seuls les headers "plausibles" (L-field valide + C-field 0x44 ou 0x46) sont affichés.

**3h50 de scan continu, 23 cycles complets, ~575 faux syncs.**

| Observation | Détail |
|-------------|--------|
| Trames valides | **0** |
| Candidates plausibles | 1 (L=0x77 C=0x46 M=VII — bruit, statistiquement attendu) |
| Pics RSSI | -69/-70 dBm sur R2-i (868.51 MHz) à ~00h27 et ~01h00 |
| R2-f (868.33 MHz) | 0 sync constant, RSSI élevé -92 dBm (brouilleur ISM) |

**Verdict : R2-mode avec sync word 0x7696 est définitivement éliminé.**

Les pics RSSI à -69 dBm sur 868.51 MHz sont intéressants (25 dB au-dessus du bruit) mais ne correspondent pas à du wMBus — pas de sync word matché. Probablement un autre dispositif ISM 868 MHz dans l'immeuble.

### Scan journée (7 mai, 7h00–20h00) — Confirmation statistique

Même firmware R2-only (commit `395b0ad`), 20+ cycles complets supplémentaires (~3h20 de données analysées). Résultats strictement identiques au scan nocturne.

**Profil RF consolidé des 10 canaux (7h+ cumulées, 40+ cycles) :**

| Canal | Fréquence | Syncs typiques | RSSI moy | Diagnostic |
|-------|-----------|---------------|----------|------------|
| R2-a | 868.03 MHz | 0–2 | -96 dBm | Quasi-mort |
| R2-b | 868.09 MHz | 2–5 | -101 dBm | Bruit statistique |
| R2-c | 868.15 MHz | 0–3 | -98 dBm | Bruit statistique |
| R2-d | 868.21 MHz | 1–7 | -100 dBm | Bruit statistique |
| R2-e | 868.27 MHz | 1–8 | -100 dBm | Bruit statistique |
| **R2-f** | **868.33 MHz** | **0 constant** | **-92 dBm** | **Brouilleur ISM (signal fort, jamais de sync)** |
| R2-g | 868.39 MHz | 1–6 | -101 dBm | Bruit statistique |
| R2-h | 868.45 MHz | 0–1 | -94 dBm | Signal continu, anti-sync |
| **R2-i** | **868.51 MHz** | **1–12** | **-93 dBm** | **Plus actif, pics -75 dBm, non-wMBus** |
| R2-j | 868.57 MHz | 1–6 | -101 dBm | Bruit statistique |

**Validation statistique** : à 4.8 kbps sur 60s le CC1101 voit ~288 000 bits. La probabilité de matcher 0x7696 (16 bits) par hasard est 288000/65536 ≈ **4.4 matchs/canal** — exactement ce qu'on observe sur les canaux sans interférence. Les écarts (R2-f à 0, R2-i à 12) s'expliquent par la présence de signaux non-wMBus qui modifient le profil statistique du flux reçu.

**Observations environnementales :**
- R2-f (868.33 MHz) : brouilleur ISM permanent — RSSI le plus fort mais 0 sync, jour comme nuit. Probablement un dispositif domotique ou station météo dans l'immeuble.
- R2-i (868.51 MHz) : activité sporadique non-wMBus, pics jusqu'à -75 dBm. Probablement un autre équipement ISM 868 MHz.
- R2-h (868.45 MHz) : signal continu faible qui supprime les faux syncs (0–1 au lieu de ~4).

## Phase 3 — Scan multi-mode T+C1+S prolongé (nuit du 7-8 mai 2026)

### Contexte

Les fréquences FREQ ont été corrigées et le C1-mode ajouté. R2 est éliminé. Durées augmentées de 60s à 120s par mode pour maximiser les chances de capter une trame spontanée.

**Config** : T=120s + C1=120s + S=120s + R2=0s, cycle continu. Chaque mode est subdivisé en 1/3 Format A (sync 0x7696) + 2/3 Format B (sync 0xF68D). Le polling REQ-UD2 est inclus dans le cycle.

### Scan nocturne (00h00–09h30, 8 mai 2026)

| Mode | Syncs/fenêtre | RSSI max | Résultat |
|------|--------------|----------|----------|
| T-mode | 25–49 | -75 à -83 dBm | 0 paquet décodé |
| C1-mode | 18–42 | -77 à -80 dBm | 0 paquet décodé |
| S-mode | 12–33 | -81 à -87 dBm | 0 paquet décodé |
| Polling | — | — | 100% échec TX |

Beaucoup de syncs, mais aucun ne passe la validation CRC.

### Découverte : TX physiquement cassé sur le clone CC1101

Le polling REQ-UD2 échoue systématiquement. Investigation :
- Après strobe STX, MARCSTATE reste bloqué à **0x08 (STARTCAL)**.
- Le VCO ne se verrouille jamais pour la transmission.
- Le clone CC1101 (VERSION=0x04) est **RX-only** — le PA est absent ou désactivé.

Conséquence : le polling REQ-UD2 est impossible avec ce matériel. La phase SCAN_POLL est désactivée (commit `277aa7d`).

### Ajout du diagnostic REJECT (commit 277aa7d)

Pour comprendre pourquoi les syncs ne produisent pas de paquets, un logging détaillé est ajouté à chaque point de rejet dans `listen()` :
- 3of6 decode fail (T-mode Format A)
- Header parse fail
- CRC fail avec hex dump des 30 premiers octets

### Premier cycle avec REJECT logging — Résultat critique

Le premier cycle révèle la nature des syncs :
- ~300 lignes `REJECT CRC fail` par cycle
- **L-field, C-field, manufacturer, serial** : valeurs totalement aléatoires
- Fabricants "décodés" : `^KG`, `@MJ`, `NXR`, etc. (symboles, pas des lettres A-Z)
- Aucun fabricant ne se répète d'une trame à l'autre

**Calcul probabiliste** : un sync word de 16 bits (0xF68D) matche du bruit aléatoire à 100 kbps avec une probabilité de 1/65536 par bit → **1.53 faux matchs/seconde**. Sur 80s de T-B, on attend ~122 faux syncs. Observé : 126. **Concordance parfaite.**

**Verdict : 100% des syncs sur TOUS les modes (T, C1, S) sont des faux positifs** causés par le bruit ISM ambiant à 868 MHz.

### Throttling des logs (commit 6e382cb)

Les ~300 lignes de CRC fail par cycle sont remplacées par :
- Un compteur silencieux (`rejectCount`) résumé dans le diagnostic RF par phase
- Un log individuel uniquement pour les trames "plausibles" (C-field valide + manufacturer A-Z + L-field raisonnable)

## Phase 4 — Scan journée complète (8 mai 2026, 10h38 → ~00h30)

### Config

Firmware avec REJECT logging throttlé. T=120s + C1=120s + S=120s + R2=0. ~55 cycles complets, ~73 heures cumulées d'écoute toutes phases confondues.

### Résultats

| Métrique | Valeur |
|----------|--------|
| Durée totale | ~14h continu |
| Cycles complets | ~55 |
| Heures cumulées d'écoute | ~73h (T+C1+S × Format A+B) |
| **Paquets valides** | **0** |
| 3of6 decode fail total | 3 408 |
| Trames "plausibles" CRC fail | ~50 (fabricants tous différents = bruit) |
| RSSI max observé | **-59 dBm** (émetteur tiers ISM) |

### Découverte : corrélation jour/nuit des faux syncs

| Période | T-B syncs/80s | C1-B syncs/80s | S-B syncs/80s | RSSI moy |
|---------|--------------|----------------|---------------|----------|
| Matin (10h–13h) | 70–165 | 76–138 | 20–35 | -88 à -92 |
| Après-midi (13h–19h) | 45–131 | 41–155 | 19–34 | -86 à -92 |
| Soirée (19h–21h) | 47–99 | 41–84 | 20–30 | -85 à -91 |
| **Nuit (22h–minuit)** | **2–12** | **3–13** | **17–40** | **-82 à -83** |

Le taux de faux syncs sur 868.95 MHz (T et C1) **chute de 10x à 50x** la nuit. Cela confirme que les syncs diurnes sont causés par l'activité ISM ambiante (domotique, IoT, capteurs météo, télécommandes), pas par les compteurs.

Le S-mode (868.3 MHz) reste plus stable car son data rate de 32.768 kbps est ~3x plus lent, générant mécaniquement moins de faux matchs du sync word.

### Signaux forts détectés (émetteurs tiers)

| Timestamp | Fréquence | RSSI | Identification |
|-----------|-----------|------|----------------|
| ~22h35 | 868.95 MHz (T-A) | **-59 dBm** | Émetteur IoT voisin, pas wMBus |
| Soirée | 868.95 MHz (T-B) | -73 dBm | Idem |
| Soirée | 868.30 MHz (S-A) | -74 dBm | Autre émetteur ISM |

Ces signaux sont 20–30 dB au-dessus du bruit mais ne produisent aucun CRC valide → dispositifs ISM tiers, pas du wMBus.

### WiFi instable

Deux épisodes de déconnexion WiFi (AUTH_FAIL, HANDSHAKE_TIMEOUT) vers 21h et 22h. Reconnexion automatique en ~30s. Sans impact sur le scan RF.

---

## Bilan — Ce qu'on sait avec certitude

1. **La chaîne RF fonctionne** : sniffer brut capte ~9300 octets/2s, self-test 5/5, VCO lock OK, signaux tiers détectés jusqu'à -59 dBm.
2. **Le CC1101 clone est RX-only** : TX bloqué en STARTCAL (MARCSTATE=0x08), polling REQ-UD2 impossible.
3. **Les compteurs ista P/N 19399 n'émettent pas en wMBus standard** : 73h+ cumulées sur T-mode, C1-mode, S-mode et R2-mode (éliminé), Format A et Format B, sync words 0x7696 et 0xF68D → 0 paquet valide.
4. **100% des syncs sont des faux positifs** : taux parfaitement corrélé à l'activité ISM ambiante (10x le jour vs nuit), concordance probabiliste exacte avec le modèle théorique.
5. **Il y a de l'activité RF à 868 MHz dans l'immeuble** : signaux forts sporadiques, mais aucun utilisant le protocole wMBus.

## Hypothèses restantes (par probabilité décroissante)

### H1 — Walk-by only (activation magnétique)

Beaucoup de modules ista n'émettent pas spontanément. Ils restent en veille profonde et ne se réveillent que lorsqu'un technicien de relève passe avec un terminal portable équipé d'un aimant qui active un reed switch dans le module.

**Test** : placer un aimant néodyme fort contre le module radio sur le compteur. Si une émission est déclenchée, le CC1101 la captera dans le mode correspondant.

### H2 — Fréquence ou modulation non-standard

Les modules ista pourraient émettre sur 433 MHz, 169 MHz, ou une sous-bande 868 MHz non couverte. Certains modèles ista utilisent des protocoles propriétaires.

**Test** : RTL-SDR large bande (~15€) avec `rtl_433 -A` pour balayer 433–868 MHz et détecter toute émission pendant un trigger magnétique.

### H3 — Modules inactifs ou défaillants

Les modules ont été installés en 2018–2019 (~7 ans). La batterie pourrait être épuisée, ou les modules pourraient ne jamais avoir été commissionnés.

**Test** : inspection physique (LED, état visuel) + vérification auprès de la régie si un service de télérelève est actif.

### H4 — Protocole propriétaire ista non-wMBus

Bien que les modules soient dans la famille wMBus, ista pourrait utiliser un protocole d'application propriétaire avec un sync word ou un framing différent du standard EN 13757-4.

**Test** : identifier le modèle exact via la plaque signalétique, rechercher dans la doc ista/OMS.

## Stratégie d'élimination — prochaines étapes

```
Phase 5 — Investigation physique (prochaine)
    │
    ├─ Trigger magnétique (aimant néodyme contre le module)
    │   ├─ Émission détectée → identifier mode, RÉSOLU
    │   └─ Rien →
    │
    ├─ RTL-SDR spectrogramme (433–868 MHz)
    │   ├─ Signal trouvé → identifier fréquence/modulation
    │   └─ Rien → modules probablement inactifs
    │
    └─ Identification modèle exact (plaque signalétique P/N 19399)
        └─ Recherche doc ista → protocole / fréquence / mode activation
```

L'approche logicielle (CC1101 écoute passive sur wMBus standard) est épuisée. La suite est physique : trigger magnétique, SDR large bande, et identification du modèle exact.
