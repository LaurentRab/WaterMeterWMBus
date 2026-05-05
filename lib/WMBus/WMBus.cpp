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

#include "WMBus.h"

// 3-out-of-6 : chaque groupe de 6 bits encode 4 bits de données (T-mode)
// Seuls les motifs à exactement 3 bits à 1 sont valides.
static const uint8_t DECODE_3OF6[64] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x01, 0x02, 0xFF,
    0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0x04, 0x05, 0xFF,
    0xFF, 0x06, 0x07, 0xFF, 0x08, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x09, 0xFF, 0x0A, 0x0B, 0xFF,
    0xFF, 0x0C, 0x0D, 0xFF, 0x0E, 0xFF, 0xFF, 0xFF,
    0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

WMBus::WMBus(CC1101& radio) : _radio(radio) {}

uint32_t WMBus::syncCount() const { return _syncCount; }
void     WMBus::resetSyncCount()  { _syncCount = 0; }

// ============================================================
//  Écoute d'un paquet wMBus
// ============================================================

bool WMBus::listen(WMBusMode mode, uint32_t timeoutMs, WMBusPacket& out)
{
    memset(&out, 0, sizeof(out));
    out.mode = mode;

    if (!_configured || mode != _lastMode) {
        if (mode == WMBUS_T_MODE)
            _radio.configureWMBusTMode();
        else
            _radio.configureWMBusSMode();
        _lastMode = mode;
        _configured = true;
    }

    uint8_t rawBuf[256];
    int rawLen = _receiveRaw(timeoutMs, rawBuf, sizeof(rawBuf));
    if (rawLen <= 0) return false;

    out.rssi = _radio.readRSSI();
    _radio.idle();

    uint8_t decoded[192];
    uint8_t decodedLen = 0;

    if (mode == WMBUS_T_MODE) {
        if (!_decode3of6(rawBuf, rawLen * 8, decoded, decodedLen)) {
            static uint16_t failCount = 0;
            failCount++;
            if (rawLen >= 4) {
                char hex[16 * 3 + 1];
                int n = rawLen < 16 ? rawLen : 16;
                for (int i = 0; i < n; i++) sprintf(hex + i * 3, "%02X ", rawBuf[i]);
                log_w("3of6 FAIL #%u raw[%d] decoded=%u: %s", failCount, rawLen, decodedLen, hex);
            } else {
                log_w("3of6 FAIL #%u raw[%d] (trop court)", failCount, rawLen);
            }
            return false;
        }
    } else {
        // S-mode : Manchester décodé par le CC1101, données NRZ directes
        decodedLen = (rawLen > sizeof(decoded)) ? sizeof(decoded) : rawLen;
        memcpy(decoded, rawBuf, decodedLen);
    }

    if (!_parseHeader(decoded, decodedLen, out))
        return false;

    out.valid = true;
    return true;
}

// ============================================================
//  Réception brute (attend sync word puis lit le FIFO)
// ============================================================

int WMBus::_receiveRaw(uint32_t timeoutMs, uint8_t* buf, uint16_t bufSize)
{
    _radio.idle();
    _radio.strobe(CC1101_SFRX);
    _radio.strobe(CC1101_SRX);

    uint32_t t0 = millis();
    while (!_radio.readGDO0()) {
        if (millis() - t0 > timeoutMs) {
            _radio.idle();
            return -1;
        }
        yield();
    }

    _syncCount++;
    delay(5);

    uint16_t total = 0;
    uint32_t lastData = millis();

    while (total < bufSize) {
        uint8_t chunk = _radio.drainFifo(buf + total, bufSize - total);
        if (chunk > 0) {
            total += chunk;
            lastData = millis();
        } else {
            if (millis() - lastData > 10) break;
            delayMicroseconds(200);
        }
    }

    _radio.idle();

    if (total > 0) {
        char hex[16 * 3 + 1] = {};
        int n = (total < 16) ? total : 16;
        for (int i = 0; i < n; i++) sprintf(hex + i * 3, "%02X ", buf[i]);
        log_w("SYNC raw[%d]: %s", total, hex);
    } else {
        log_w("SYNC ghost — GDO0 fired but FIFO empty (0 bytes)");
    }

    return total;
}

// ============================================================
//  Décodage 3-out-of-6 (T-mode)
// ============================================================

bool WMBus::_decode3of6(const uint8_t* raw, uint16_t rawBits, uint8_t* out, uint8_t& outLen)
{
    uint16_t totalBits = rawBits;
    uint16_t nibbles = totalBits / 6;
    if (nibbles < 2) return false;

    outLen = 0;
    uint16_t bitPos = 0;

    while (bitPos + 12 <= totalBits && outLen < 192) {
        // Extraire 2 groupes de 6 bits → 2 nibbles → 1 octet
        uint8_t hi6 = 0, lo6 = 0;

        for (int b = 0; b < 6; b++) {
            uint16_t idx = bitPos + b;
            uint8_t bit = (raw[idx / 8] >> (7 - (idx % 8))) & 1;
            hi6 = (hi6 << 1) | bit;
        }
        bitPos += 6;

        for (int b = 0; b < 6; b++) {
            uint16_t idx = bitPos + b;
            uint8_t bit = (raw[idx / 8] >> (7 - (idx % 8))) & 1;
            lo6 = (lo6 << 1) | bit;
        }
        bitPos += 6;

        uint8_t hiNib = DECODE_3OF6[hi6];
        uint8_t loNib = DECODE_3OF6[lo6];

        if (hiNib == 0xFF || loNib == 0xFF) {
            log_d("3of6 decode error at bit %u (hi=0x%02X lo=0x%02X)", bitPos - 12, hi6, lo6);
            if (outLen >= 11) break;  // on a au moins le header
            return false;
        }

        out[outLen++] = (hiNib << 4) | loNib;
    }

    return outLen >= 11;
}

// ============================================================
//  Parsing du header wMBus (L, C, M, A, CI)
// ============================================================

bool WMBus::_parseHeader(const uint8_t* data, uint8_t len, WMBusPacket& pkt)
{
    if (len < 11) return false;

    pkt.lField     = data[0];
    pkt.cField     = data[1];
    pkt.mField     = data[2] | (data[3] << 8);  // little-endian
    // A-field : 4 octets serial BCD LE + 1 version + 1 device type
    pkt.serialBCD  = bcdToUint32(data + 4, 4);
    pkt.version    = data[8];
    pkt.deviceType = data[9];
    pkt.ciField    = data[10];

    // Copier les données brutes
    pkt.dataLen = (len > sizeof(pkt.data)) ? sizeof(pkt.data) : len;
    memcpy(pkt.data, data, pkt.dataLen);

    // CRC sur le premier bloc (10 octets data + 2 octets CRC)
    if (len >= 12) {
        uint16_t crcCalc = _crc16EN13757(data, 10);
        uint16_t crcRecv = (data[10] << 8) | data[11];
        pkt.crcOk = (crcCalc == crcRecv);

        if (!pkt.crcOk) {
            // Le CI-field est en fait à l'offset 10 du payload décodé,
            // mais dans le format CRC-block, les 2 octets CRC sont insérés.
            // On reparse : L(1) + C(1) + M(2) + A(6) = 10 octets, puis CRC(2), puis CI...
            pkt.ciField = (len > 12) ? data[12] : 0;
        }
    } else {
        pkt.crcOk = false;
    }

    return true;
}

// ============================================================
//  CRC-16/EN13757 (polynôme 0x3D65)
// ============================================================

uint16_t WMBus::_crc16EN13757(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x3D65;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0xFFFF;
}

// ============================================================
//  Utilitaires
// ============================================================

void WMBus::decodeMfr(uint16_t mField, char out[4])
{
    out[0] = ((mField >> 10) & 0x1F) + 64;
    out[1] = ((mField >> 5)  & 0x1F) + 64;
    out[2] = ( mField        & 0x1F) + 64;
    out[3] = '\0';
}

uint32_t WMBus::bcdToUint32(const uint8_t* bcd, uint8_t len)
{
    // BCD little-endian : bcd[0] = poids faible
    uint32_t result = 0;
    for (int i = len - 1; i >= 0; i--)
        result = result * 100 + ((bcd[i] >> 4) * 10 + (bcd[i] & 0x0F));
    return result;
}
