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

volatile TaskHandle_t WMBus::_rxTaskHandle = nullptr;

void IRAM_ATTR WMBus::_onGDO0ISR() {
    BaseType_t woken = pdFALSE;
    if (_rxTaskHandle)
        vTaskNotifyGiveFromISR(_rxTaskHandle, &woken);
    if (woken) portYIELD_FROM_ISR();
}

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
        switch (mode) {
        case WMBUS_T_MODE: _radio.configureWMBusTMode(); break;
        case WMBUS_C_MODE: _radio.configureWMBusCMode(); break;
        case WMBUS_S_MODE: _radio.configureWMBusSMode(); break;
        case WMBUS_R_MODE: break; // R-mode config handled externally (per-channel)
        }
        _lastMode = mode;
        _configured = true;
    }

    uint8_t rawBuf[256];
    int rawLen = _receiveRaw(timeoutMs, rawBuf, sizeof(rawBuf));
    if (rawLen <= 0) return false;

    out.rssi = _radio.readRSSI();
    _radio.idle();

    if (mode == WMBUS_R_MODE)
        log_d("R2 sync %d octets RSSI=%d dBm", rawLen, out.rssi);

    uint8_t decoded[192];
    uint8_t decodedLen = 0;

    if (mode == WMBUS_T_MODE || mode == WMBUS_C_MODE) {
        if (mode == WMBUS_C_MODE) {
            // C1 : données NRZ directes (pas de 3of6)
            decodedLen = (rawLen > sizeof(decoded)) ? sizeof(decoded) : rawLen;
            memcpy(decoded, rawBuf, decodedLen);
        } else if (!_decode3of6(rawBuf, rawLen * 8, decoded, decodedLen)) {
            static uint16_t failCount = 0;
            static uint32_t lastLog = 0;
            failCount++;
            if (millis() - lastLog > 300000) {
                lastLog = millis();
                log_d("3of6 bruit: %u faux syncs depuis boot", failCount);
            }
            return false;
        }
    } else {
        // S-mode : Manchester décodé par le CC1101, données NRZ directes
        decodedLen = (rawLen > sizeof(decoded)) ? sizeof(decoded) : rawLen;
        memcpy(decoded, rawBuf, decodedLen);
    }

    if (!_parseHeader(decoded, decodedLen, out)) {
        if (mode == WMBUS_R_MODE)
            log_d("R2 parse fail: decodedLen=%d", decodedLen);
        return false;
    }

    if (!out.crcOk) {
        if (mode == WMBUS_R_MODE) {
            bool plausible = out.lField >= 0x0A && out.lField <= 0x80
                          && (out.cField == 0x44 || out.cField == 0x46);
            if (plausible) {
                char mfr[4];
                decodeMfr(out.mField, mfr);
                uint16_t crcCalc = _crc16EN13757(decoded, 10);
                uint16_t crcRecv = (decodedLen >= 12)
                    ? ((uint16_t)decoded[10] << 8) | decoded[11] : 0;
                char hex[64];
                int hLen = 0;
                int show = (decodedLen > 20) ? 20 : decodedLen;
                for (int i = 0; i < show && hLen < 60; i++)
                    hLen += snprintf(hex + hLen, sizeof(hex) - hLen,
                                     "%02X ", decoded[i]);
                log_w("R2 candidate: L=%02X C=%02X M=%s ser=%08lu "
                      "dev=%02X CRC=%04X/%04X", out.lField, out.cField,
                      mfr, out.serialBCD, out.deviceType, crcCalc, crcRecv);
                log_w("  %s", hex);
            }
        } else {
            log_d("CRC mismatch — dropped");
        }
        return false;
    }

    out.valid = true;
    return true;
}

// ============================================================
//  Réception brute (attend sync word puis lit le FIFO)
// ============================================================

int WMBus::_receiveRaw(uint32_t timeoutMs, uint8_t* buf, uint16_t bufSize)
{
    _radio.idle();

    UBaseType_t savedPrio = uxTaskPriorityGet(nullptr);
    vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 1);

    _rxTaskHandle = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);
    attachInterrupt(digitalPinToInterrupt(_radio.gdo0Pin()), _onGDO0ISR, RISING);

    _radio.strobe(CC1101_SRX);

    uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeoutMs));
    detachInterrupt(digitalPinToInterrupt(_radio.gdo0Pin()));
    _rxTaskHandle = nullptr;

    if (!notif) {
        vTaskPrioritySet(nullptr, savedPrio);
        _radio.idle();
        return -1;
    }

    _syncCount++;

    uint16_t total = 0;
    uint32_t lastData = millis();

    while (total < bufSize) {
        uint16_t remain = bufSize - total;
        uint8_t chunk = _radio.drainFifo(buf + total, remain < 64 ? (uint8_t)remain : 64);
        if (chunk > 0) {
            total += chunk;
            lastData = millis();
        } else {
            if (millis() - lastData > 10) break;
            delayMicroseconds(200);
        }
    }

    vTaskPrioritySet(nullptr, savedPrio);
    _radio.idle();
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

void WMBus::uint32ToBcdLE(uint32_t val, uint8_t out[4])
{
    for (int i = 0; i < 4; i++) {
        uint8_t lo = val % 10; val /= 10;
        uint8_t hi = val % 10; val /= 10;
        out[i] = (hi << 4) | lo;
    }
}

// ============================================================
//  Polling actif — REQ-UD2 puis écoute réponse
// ============================================================

bool WMBus::poll(WMBusMode mode, uint32_t serialBCD, uint16_t mfr,
                 uint8_t version, uint8_t devType,
                 uint32_t timeoutMs, WMBusPacket& response)
{
    memset(&response, 0, sizeof(response));
    response.mode = mode;

    if (!_configured || mode != _lastMode) {
        switch (mode) {
        case WMBUS_T_MODE: _radio.configureWMBusTMode(); break;
        case WMBUS_C_MODE: _radio.configureWMBusCMode(); break;
        case WMBUS_S_MODE: _radio.configureWMBusSMode(); break;
        case WMBUS_R_MODE: break;
        }
        _lastMode = mode;
        _configured = true;
    }

    uint8_t frame[12];
    frame[0] = 0x09;                         // L-field
    frame[1] = 0x7B;                         // C-field: REQ-UD2, FCB=1
    frame[2] = mfr & 0xFF;                   // M-field LE
    frame[3] = (mfr >> 8) & 0xFF;
    uint32ToBcdLE(serialBCD, frame + 4);     // A-field: serial BCD LE
    frame[8] = version;
    frame[9] = devType;
    uint16_t crc = _crc16EN13757(frame, 10);
    frame[10] = (crc >> 8) & 0xFF;
    frame[11] = crc & 0xFF;

    if (!_radio.sendPacket(frame, 12)) {
        log_w("poll: TX échoué");
        return false;
    }

    uint8_t rawBuf[256];
    int rawLen = _receiveRaw(timeoutMs, rawBuf, sizeof(rawBuf));
    if (rawLen <= 0) return false;

    response.rssi = _radio.readRSSI();
    _radio.idle();

    uint8_t decoded[192];
    uint8_t decodedLen = (rawLen > (int)sizeof(decoded)) ? sizeof(decoded) : rawLen;
    memcpy(decoded, rawBuf, decodedLen);

    if (!_parseHeader(decoded, decodedLen, response))
        return false;

    if (!response.crcOk) {
        char mfrStr[4];
        decodeMfr(response.mField, mfrStr);
        log_w("poll response CRC fail: L=%02X C=%02X M=%s ser=%08lu",
              response.lField, response.cField, mfrStr, response.serialBCD);
        return false;
    }

    response.valid = true;
    return true;
}
