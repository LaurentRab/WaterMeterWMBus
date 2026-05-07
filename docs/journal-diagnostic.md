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

---

## Ce qu'on sait avec certitude

1. La chaîne RF fonctionne (sniffer brut, self-test 5/5, VCO lock OK).
2. Les compteurs ista P/N 19399 **n'émettent pas** en T, C1, S ou R2-mode (sync 0x7696).
3. Le sniffer brut capte ~675 octets/2s à 868.95 MHz → il y a de l'activité RF sur cette fréquence, mais pas structurée comme du wMBus standard.
4. Les pics RSSI sporadiques à -69 dBm sur 868.51 MHz confirment une présence radio dans l'immeuble.

## Hypothèses restantes (par probabilité décroissante)

### H1 — T-mode ou C1-mode avec écoute prolongée

Le sniffer brut a détecté 675 octets à 868.95 MHz. Les premiers tests T/C1 étaient réalisés avec les registres FREQ **incorrects** (+243 kHz). Bien que corrigés depuis, ces modes n'ont pas été retestés en scan long avec la fréquence correcte.

**Test** : scan nocturne T=60s + C1=60s + S=120s (configuré, prêt à flasher).

### H2 — Format B (sync word 0xF68D)

Le wMBus EN 13757-4 définit deux formats de trame :
- Format A : sync 0x7696 (testé en R2 et S)
- Format B : sync 0xF68D (non testé)

Certains compteurs utilisent Format B. Le CRC et la structure de trame diffèrent.

**Test** : modifier le sync word CC1101 et refaire un scan.

### H3 — R-SND (polling only)

Les compteurs R-mode peuvent fonctionner en mode "réponse uniquement" : ils ne transmettent que lorsqu'un concentrateur ista les interroge. Si aucun concentrateur n'est installé dans l'immeuble, les compteurs restent silencieux.

**Test** : demander à la régie de l'immeuble s'il y a un concentrateur ista installé. Si oui, les trames polling/réponse seraient captables en écoute passive.

### H4 — Protocole propriétaire ista

Bien que les modules soient certifiés wMBus OMS, ista pourrait utiliser un canal ou une modulation non standard.

**Test** : scan spectral large bande (si SDR disponible) pour localiser la fréquence exacte d'émission.

## Stratégie d'élimination

```
Phase 3 (en cours)    T + C1 + S multi-mode, scan nocturne
    │
    ├─ Paquet capté → RÉSOLU (décoder, matcher serial, extraire conso)
    │
    └─ Rien →
         │
Phase 4   Tester Format B (sync 0xF68D) sur T/C1/S
         │
         ├─ Paquet capté → RÉSOLU
         │
         └─ Rien →
              │
Phase 5       Investigation physique
              ├─ Confirmer présence concentrateur ista
              ├─ Scan SDR large bande si disponible
              └─ Walk-by test (ESP32 collé au compteur)
```

Chaque phase produit un verdict binaire clair, validé par un volume de données statistiquement significatif (scan nocturne). On avance par élimination méthodique jusqu'à trouver le bon mode ou confirmer que les compteurs nécessitent un polling actif.
