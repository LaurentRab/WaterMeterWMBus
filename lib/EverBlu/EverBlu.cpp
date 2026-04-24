#include <Arduino.h>
#include <time.h>
#include "EverBlu.h"

// ============================================================
//  Constantes protocole
// ============================================================

// Taille du buffer de décodage (octets) — partagée entre request() et _decodeResponse().
// Doit être suffisante pour la trame de données EverBlu (0x7C = 124 octets décodés max).
static constexpr uint8_t EVERBLU_DECODE_BUF_SIZE = 200;

// Préfixe de synchronisation TX (9 octets, envoyés non encodés)
static const uint8_t SYNC_PREFIX[9] = {
    0x50, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF
};

// Template de payload requête (17 octets + 2 octets CRC)
static const uint8_t REQ_TEMPLATE[17] = {
    0x13, 0x10, 0x00, 0x45,
    0x00,             // [4]  : année (rempli dynamiquement)
    0x00, 0x00, 0x00, // [5-7]: serial 3 octets MSB-first (rempli dynamiquement)
    0x00, 0x45, 0x20, 0x0A, 0x50, 0x14, 0x00, 0x0A  // [8-15]: champ fixe
    // [16]: dernier octet fixe
};

// ============================================================
//  Constructeur
// ============================================================

EverBlu::EverBlu(CC1101& radio) : _radio(radio) {}

// ============================================================
//  Fenêtre horaire autorisée : 06:00 – 18:59
// ============================================================

bool EverBlu::withinTimeWindow()
{
    struct tm t;
    if (!getLocalTime(&t, 0)) return true;  // NTP non synchro → on tente quand même
    return (t.tm_hour >= 6 && t.tm_hour < 19);
}

// ============================================================
//  Requête principale
// ============================================================

bool EverBlu::request(uint32_t serial, uint8_t year, EverBluData& out)
{
    out.valid = false;

    // Construction de la trame de requête
    uint8_t reqBuf[48];
    uint8_t reqLen = _buildRequest(serial, year, reqBuf);
    log_i("EverBlu requête compteur serial=%lu year=%u (%u octets)", serial, year, reqLen);

    // ---- Phase TX : wake-up + requête ----------------------
    _sendWakeupAndRequest(reqBuf, reqLen);
    log_i("TX terminé — attente ACK");

    // ---- Phase RX 1 : ACK (0x12=18 octets décodés, timeout 150 ms) ---
    uint8_t rxBuf[800];
    int ackLen = _receiveFrame(0x12, 150, 200, rxBuf, sizeof(rxBuf));
    if (ackLen == 0) {
        log_w("Pas d'ACK — retry unique...");
        _sendWakeupAndRequest(reqBuf, reqLen);
        ackLen = _receiveFrame(0x12, 150, 200, rxBuf, sizeof(rxBuf));
        if (ackLen == 0) {
            log_w("Pas d'ACK après retry (compteur absent, hors fenêtre, ou mauvais serial/année ?)");
            _radio.idle();
            return false;
        }
        log_i("ACK reçu après retry (%d octets bruts)", ackLen);
    } else {
        log_i("ACK reçu (%d octets bruts) — attente données", ackLen);
    }

    // ---- Phase RX 2 : données (0x7C=124 octets décodés, timeout 700 ms) ---
    int rawLen = _receiveFrame(0x7C, 150, 700, rxBuf, sizeof(rxBuf));
    if (rawLen == 0) {
        log_w("Pas de données reçues après ACK");
        _radio.idle();
        return false;
    }
    log_i("Données brutes reçues : %d octets", rawLen);

    // ---- Décodage ------------------------------------------
    uint8_t decoded[EVERBLU_DECODE_BUF_SIZE];
    uint8_t decodedLen = 0;
    if (!_decodeResponse(rxBuf, rawLen, decoded, decodedLen)) {
        log_w("Échec décodage réponse (%d octets bruts)", rawLen);
        _radio.idle();
        return false;
    }
    log_i("Décodé : %u octets", decodedLen);

    // ---- Extraction valeurs --------------------------------
    out.rssi = _radio.readRSSI();
    if (!_parseData(decoded, decodedLen, out)) {
        log_w("Payload trop court pour extraire les données (%u octets)", decodedLen);
        _radio.idle();
        return false;
    }

    _radio.idle();
    return true;
}

// ============================================================
//  Construction de la trame de requête
// ============================================================

uint8_t EverBlu::_buildRequest(uint32_t serial, uint8_t year, uint8_t* out)
{
    // Payload (17 octets de données + 2 octets CRC = 19 octets)
    uint8_t payload[19];
    memcpy(payload, REQ_TEMPLATE, sizeof(REQ_TEMPLATE));
    payload[16] = 0x40;   // dernier octet fixe du template

    // Insertion de l'année et du serial
    payload[4] = (uint8_t)year;
    payload[5] = (uint8_t)((serial >> 16) & 0xFF);  // MSB
    payload[6] = (uint8_t)((serial >>  8) & 0xFF);
    payload[7] = (uint8_t)( serial        & 0xFF);  // LSB

    // CRC Kermit sur les 17 premiers octets
    uint16_t crc = _crcKermit(payload, 17);
    payload[17] = (uint8_t)(crc >> 8);    // high byte (déjà swappé par _crcKermit)
    payload[18] = (uint8_t)(crc & 0xFF);  // low byte

    // Assemblage : préfixe sync (9 octets) + payload encodé série
    memcpy(out, SYNC_PREFIX, 9);
    uint8_t encLen = _encodeBytes(payload, 19, out + 9);

    return 9 + encLen;  // ≈ 38–39 octets
}

// ============================================================
//  TX : wake-up (~2s de 0x55) puis requête
// ============================================================

// ============================================================
//  _doWakeupTX — remplissage FIFO et envoi de la requête
//
//  Séparé de _sendWakeupAndRequest pour pouvoir utiliser return
//  comme sortie anticipée en cas de timeout matériel, sans goto.
//  Le cleanup TX (idle + restauration registres) reste dans
//  _sendWakeupAndRequest, qui s'exécute toujours quelle que soit
//  l'issue de cette fonction.
// ============================================================

bool EverBlu::_doWakeupTX(const uint8_t* reqBuf, uint8_t reqLen)
{
    static const uint8_t WUP[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    const uint8_t WUP_REPEATS = 94;  // 94 × 8 octets ≈ 2.5 secondes à 2.4 kbps

    // Vérifier que le CC1101 est bien entré en TX
    delay(2);
    if (_radio.marcstate() != CC1101_STATE_TX) {
        log_e("CC1101 n'entre pas en TX (MARCSTATE=0x%02X) — problème matériel !",
              _radio.marcstate());
        return false;
    }

    // Refill du FIFO pendant le wake-up
    for (uint8_t rep = 1; rep < WUP_REPEATS; rep++) {
        uint32_t t = millis();
        while (_radio.txFifoFree() < 8) {
            if (millis() - t > 3000) return false;  // timeout matériel
            delay(5);
        }
        _radio.writeFifo(WUP, 8);
    }

    // Pause 130 ms (le CC1101 continue d'émettre les derniers octets bufférisés)
    delay(130);

    // Envoyer la trame de requête dans le FIFO
    {
        uint32_t t = millis();
        while (_radio.txFifoFree() < reqLen) {
            if (millis() - t > 2000) return false;  // timeout matériel
            delay(5);
        }
        _radio.writeFifo(reqBuf, reqLen);
    }

    // Attendre que le FIFO se vide entièrement.
    // En mode infini, le CC1101 passe en TX_UNDERFLOW quand le FIFO est vide
    // (MARCSTATE quitte TX=0x13). Polling d'état plutôt que délai fixe
    // pour ne pas tronquer les derniers octets (CRC inclus).
    {
        uint32_t t = millis();
        while (_radio.marcstate() == CC1101_STATE_TX && millis() - t < 500) {
            delay(2);
        }
    }

    return true;
}

// ============================================================
//  _sendWakeupAndRequest — configure le CC1101, délègue à
//  _doWakeupTX, puis assure le cleanup quoi qu'il arrive.
// ============================================================

void EverBlu::_sendWakeupAndRequest(const uint8_t* reqBuf, uint8_t reqLen)
{
    static const uint8_t WUP_FIRST[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    // Mode TX : longueur infinie, sans sync word (porteuse brute de 0x55)
    _radio.idle();
    _radio.writeReg(CC1101_MDMCFG2, 0x00);   // Pas de sync word
    _radio.writeReg(CC1101_PKTCTRL0, 0x02);  // Mode longueur infinie

    // Amorcer le FIFO et démarrer TX avant de déléguer
    _radio.writeFifo(WUP_FIRST, 8);
    _radio.strobe(CC1101_STX);

    _doWakeupTX(reqBuf, reqLen);  // retour ignoré : cleanup toujours exécuté

    // Forcer IDLE (nécessaire car en mode infini le CC1101 est en TX_UNDERFLOW)
    _radio.idle();
    // Rétablir la config normale (sync word actif, longueur fixe)
    _radio.writeReg(CC1101_MDMCFG2, 0x02);
    _radio.writeReg(CC1101_PKTCTRL0, 0x00);
}

// ============================================================
//  Attente que le CC1101 entre en RX (avec timeout)
//  Reproduit cc1101_rec_mode() de la référence
// ============================================================

void EverBlu::_enterRX()
{
    _radio.idle();
    _radio.strobe(CC1101_SRX);
    uint32_t t = millis();
    while (millis() - t < 100) {
        uint8_t s = _radio.marcstate();
        if (s == CC1101_STATE_RX || s == 0x0E || s == 0x0F) return;
        delay(1);
    }
    log_w("Timeout attente MARCSTATE=RX (0x%02X)", _radio.marcstate());
}

// ============================================================
//  Réception trame Radian (2 phases, comme la référence)
//
//  Phase 1 — sync 0x5550 à 2.4 kbps (preamble 0101…01 du compteur)
//            GDO0 détecte le sync word, on lit 1 octet pour confirmer
//  Phase 2 — reconfiguration 9.6 kbps (4× oversampled), sync 0xFFF0
//            GDO0 détecte le début des données, lecture FIFO en continu
//
//  sizeBytes = taille décodée attendue (0x12 pour ACK, 0x7C pour données)
//  Retourne le nombre d'octets bruts reçus, ou 0 en cas de timeout
// ============================================================

int EverBlu::_receiveFrame(uint8_t sizeBytes, uint32_t phase1Ms, uint32_t phase2Ms,
                            uint8_t* buf, uint16_t bufSize)
{
    uint16_t rawFrameSize = (((uint16_t)sizeBytes * 11) / 8 + 1) * 4;

    if (rawFrameSize > bufSize) {
        log_w("Buffer trop petit (%u > %u)", rawFrameSize, bufSize);
        return 0;
    }

    // ---- Phase 1 : détecter sync 0x5550 à 2.4 kbps --------
    _radio.strobe(CC1101_SFRX);
    _radio.writeReg(CC1101_MCSM1,    0x0F);  // Rester en RX après paquet
    _radio.writeReg(CC1101_MDMCFG2,  0x02);  // 2-FSK, 16/16 sync
    _radio.writeReg(CC1101_SYNC1,    0x55);
    _radio.writeReg(CC1101_SYNC0,    0x50);
    _radio.writeReg(CC1101_MDMCFG4,  0xF6);  // 2.4 kbps
    _radio.writeReg(CC1101_MDMCFG3,  0x83);
    _radio.writeReg(CC1101_PKTLEN,   1);      // 1 octet suffit pour détecter
    _radio.writeReg(CC1101_PKTCTRL0, 0x00);   // Longueur fixe
    _enterRX();

    uint32_t dl1 = millis() + phase1Ms;
    while (!_radio.readGDO0() && millis() < dl1) delay(1);
    if (millis() >= dl1) return 0;
    log_i("GDO0! (sync 2.4k, %lu ms)", phase1Ms - (dl1 - millis()));

    // Attendre GDO0 = LOW : fin du paquet de 1 octet → CC1101 stable
    { uint32_t t = millis(); while (_radio.readGDO0() && millis() - t < 50) delay(1); }
    // Drainer le FIFO (1 octet reçu)
    { uint8_t dummy[8]; _radio.drainFifo(dummy, sizeof(dummy)); }

    // ---- Phase 2 : données 4× oversampled (9.6 kbps, sync 0xFFF0) ---
    _radio.writeReg(CC1101_SYNC1,    0xFF);
    _radio.writeReg(CC1101_SYNC0,    0xF0);
    _radio.writeReg(CC1101_MDMCFG4,  0xF8);  // 9.6 kbps
    _radio.writeReg(CC1101_MDMCFG3,  0x83);
    _radio.writeReg(CC1101_PKTCTRL0, 0x02);  // Mode infini
    _radio.strobe(CC1101_SFRX);
    _enterRX();

    // Attendre GDO0 (sync 0xFFF0 détecté).
    // GDO0 peut déjà être haut si le sync est arrivé pendant la reconfiguration
    // (race condition normale) — dans ce cas on passe immédiatement.
    uint32_t dl2 = millis() + phase2Ms;
    if (!_radio.readGDO0()) {
        while (!_radio.readGDO0() && millis() < dl2) delay(1);
        if (millis() >= dl2) return 0;
    }
    log_i("GDO0! (sync 9.6k, %lu ms)", phase2Ms - (dl2 - millis()));

    // Deadline indépendante pour la collecte : le sync peut arriver tard,
    // la fenêtre de lecture doit rester complète quelle que soit l'attente GDO0.
    uint32_t dl2_data = millis() + phase2Ms;

    // Lecture données brutes
    uint16_t totalBytes = 0;
    while (totalBytes < rawFrameSize && millis() < dl2_data) {
        delay(5);
        uint8_t avail = _radio.rxFifoBytes();
        if (avail > 0) {
            if (totalBytes + avail > bufSize)
                avail = bufSize - totalBytes;
            _radio.drainFifo(buf + totalBytes, avail);
            totalBytes += avail;
        }
    }

    // Nettoyage
    _radio.strobe(CC1101_SFRX);
    _radio.idle();

    // Restaurer registres par défaut
    _radio.writeReg(CC1101_MCSM1,    0x00);  // IDLE après RX/TX (valeur configureEverBlu)
    _radio.writeReg(CC1101_MDMCFG4,  0xF6);
    _radio.writeReg(CC1101_MDMCFG3,  0x83);
    _radio.writeReg(CC1101_PKTCTRL0, 0x00);
    _radio.writeReg(CC1101_PKTLEN,   38);
    _radio.writeReg(CC1101_SYNC1,    0x55);
    _radio.writeReg(CC1101_SYNC0,    0x00);

    if (totalBytes > 0)
        log_i("Frame reçue (%u octets bruts)", totalBytes);

    return totalBytes;
}

// ============================================================
//  Décodage 4× oversampled + encodage série start/stop
//
//  Le signal reçu à 9.6 kbps encode des données à 2.4 kbps :
//  chaque bit original apparaît comme 4 bits identiques.
//  De plus, chaque octet est encadré par 1 bit start (0) et
//  3 bits stop (111) → 12 bits par octet à 2.4 kbps
//                    → 48 bits (6 octets) dans le flux 9.6 kbps.
// ============================================================

bool EverBlu::_decodeResponse(const uint8_t* raw, uint16_t rawLen,
                               uint8_t* out, uint8_t& outLen)
{
    outLen = 0;
    uint16_t totalBits = (uint16_t)rawLen * 8;
    uint16_t pos = 0;

    while (pos + 48 <= totalBits) {
        // Chercher un bit start (4 bits à 0)
        if (_bit4x(raw, pos) != 0) {
            pos += 4;
            continue;
        }
        pos += 4;  // Consommer le bit start

        if (pos + 44 > totalBits) break;

        // Lire 8 bits de données LSB-first (chacun × 4 dans le flux)
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            if (_bit4x(raw, pos)) byte |= (1u << b);
            pos += 4;
        }

        // Consommer les 3 bits stop (× 4)
        pos += 12;

        out[outLen++] = byte;
        if (outLen >= EVERBLU_DECODE_BUF_SIZE) break;
    }

    return outLen > 0;
}

// ============================================================
//  Extraction des valeurs du buffer décodé
//  Offsets issus du reverse-engineering (hallard/psykokwak)
// ============================================================

// Offsets protocole EverBlu Cyble Enhanced dans la trame décodée
static constexpr uint8_t EVERBLU_OFF_LITERS_0  = 18;  // uint32 LE : octet 0 (LSB)
static constexpr uint8_t EVERBLU_OFF_LITERS_1  = 19;
static constexpr uint8_t EVERBLU_OFF_LITERS_2  = 20;
static constexpr uint8_t EVERBLU_OFF_LITERS_3  = 21;  // uint32 LE : octet 3 (MSB)
static constexpr uint8_t EVERBLU_OFF_BATTERY   = 31;  // mois de batterie restante
static constexpr uint8_t EVERBLU_OFF_READCOUNT = 48;  // compteur de lectures (rollover)
static constexpr uint8_t EVERBLU_MIN_LEN       = 32;  // longueur minimale pour les champs obligatoires

bool EverBlu::_parseData(const uint8_t* d, uint8_t len, EverBluData& out)
{
    if (len < EVERBLU_MIN_LEN) return false;

    if (!_verifyCrc(d, len))
        log_e("CRC invalide (len=%u) — données possiblement corrompues (bruit RF ?), publication quand même", len);

    // Volume en litres : uint32 little-endian
    out.liters = (uint32_t)d[EVERBLU_OFF_LITERS_0]
               | ((uint32_t)d[EVERBLU_OFF_LITERS_1] << 8)
               | ((uint32_t)d[EVERBLU_OFF_LITERS_2] << 16)
               | ((uint32_t)d[EVERBLU_OFF_LITERS_3] << 24);

    // Batterie : mois restants
    out.battery   = d[EVERBLU_OFF_BATTERY];

    // Compteur de lectures (si trame assez longue)
    out.readCount = (len > EVERBLU_OFF_READCOUNT) ? d[EVERBLU_OFF_READCOUNT] : 0;

    out.valid = true;
    return true;
}

// ============================================================
//  Validation CRC Kermit de la trame décodée
//
//  Convention EverBlu : CRC calculé sur les (len-2) premiers
//  octets, stocké aux octets [len-2..len-1] en big-endian.
//  Même algorithme que pour la trame de requête TX.
// ============================================================

bool EverBlu::_verifyCrc(const uint8_t* data, uint8_t len)
{
    if (len < 3) return true;  // trop court pour contenir un CRC — on laisse passer

    uint16_t computed = _crcKermit(data, len - 2);
    uint16_t received = ((uint16_t)data[len - 2] << 8) | data[len - 1];

    if (computed != received) {
        log_e("CRC attendu=0x%04X reçu=0x%04X", computed, received);
        return false;
    }
    return true;
}

// ============================================================
//  Encodage série : start(0) + 8 bits LSB-first + stop(111)
//  = 12 bits par octet, packés MSB-first dans out[]
// ============================================================

uint8_t EverBlu::_encodeBytes(const uint8_t* in, uint8_t inLen, uint8_t* out)
{
    uint32_t shift = 0;
    int      bits  = 0;
    uint8_t  outLen = 0;

    for (uint8_t i = 0; i < inLen; i++) {
        // Mot de 12 bits : bit11=start=0, bits10..3=données LSB-first, bits2..0=stop=111
        uint16_t word = 0x007;  // stop bits
        for (int b = 0; b < 8; b++) {
            if (in[i] & (1u << b)) word |= (1u << (10 - b));
        }
        // bit 11 = start = 0 (déjà à 0)

        shift = (shift << 12) | word;
        bits += 12;

        while (bits >= 8) {
            bits -= 8;
            out[outLen++] = (uint8_t)((shift >> bits) & 0xFF);
        }
    }

    if (bits > 0) {
        // Padding des bits restants avec des 1 (idle high, pas des 0 qui
        // seraient interprétés comme un start bit parasite par le compteur)
        out[outLen++] = (uint8_t)(((shift << (8 - bits)) | (0xFF >> bits)) & 0xFF);
    }

    // Octet idle final (réf: outputBuffer[bytepos+1] = 0xFF)
    out[outLen++] = 0xFF;

    return outLen;
}

// ============================================================
//  CRC Kermit (polynôme 0x8408, init 0x0000)
//  Les bytes du résultat sont swappés avant retour.
// ============================================================

uint16_t EverBlu::_crcKermit(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else         crc >>= 1;
        }
    }
    // Swap high/low bytes
    return ((crc & 0xFF) << 8) | (crc >> 8);
}

// ============================================================
//  Helpers bits
// ============================================================

uint8_t EverBlu::_bit(const uint8_t* buf, uint16_t pos)
{
    return (buf[pos / 8] >> (7 - pos % 8)) & 1;
}

uint8_t EverBlu::_bit4x(const uint8_t* buf, uint16_t pos)
{
    // Vote majoritaire sur 4 bits consécutifs
    uint8_t ones = _bit(buf, pos) + _bit(buf, pos+1)
                 + _bit(buf, pos+2) + _bit(buf, pos+3);
    return (ones >= 2) ? 1 : 0;
}
