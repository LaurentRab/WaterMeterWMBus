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
#include <Arduino.h>
#include "../CC1101/CC1101.h"

// ============================================================
//  Wireless M-Bus (EN 13757-4) — écoute passive
//  T-mode : 868.95 MHz, 100 kbps, 3-out-of-6 encoding
//  S-mode : 868.3 MHz, 32.768 kbps, Manchester (HW CC1101)
// ============================================================

enum WMBusMode : uint8_t { WMBUS_T_MODE, WMBUS_S_MODE, WMBUS_C_MODE, WMBUS_R_MODE };

struct WMBusPacket {
    uint8_t   lField;
    uint8_t   cField;
    uint16_t  mField;
    uint32_t  serialBCD;      // 8 chiffres BCD depuis A-field
    uint8_t   version;
    uint8_t   deviceType;     // 0x07 = eau
    uint8_t   ciField;
    int8_t    rssi;
    WMBusMode mode;
    bool      crcOk;
    bool      valid;
    uint8_t   data[256];
    uint8_t   dataLen;
};

class WMBus {
public:
    explicit WMBus(CC1101& radio);

    bool listen(WMBusMode mode, uint32_t timeoutMs, WMBusPacket& out,
                uint16_t syncWord = 0, bool dualDecode = false);

    // Envoie REQ-UD2 puis écoute la réponse pendant timeoutMs.
    bool poll(WMBusMode mode, uint32_t serialBCD, uint16_t mfr,
              uint8_t version, uint8_t devType,
              uint32_t timeoutMs, WMBusPacket& response);

    uint32_t syncCount() const;
    void     resetSyncCount();

    static void decodeMfr(uint16_t mField, char out[4]);
    static uint32_t bcdToUint32(const uint8_t* bcd, uint8_t len);
    static void uint32ToBcdLE(uint32_t val, uint8_t out[4]);

private:
    CC1101& _radio;
    WMBusMode _lastMode = WMBUS_T_MODE;
    bool      _configured = false;
    uint32_t  _syncCount = 0;

    static volatile TaskHandle_t _rxTaskHandle;
    static void IRAM_ATTR _onGDO0ISR();

    int _receiveRaw(uint32_t timeoutMs, uint8_t* buf, uint16_t bufSize, int8_t* rssiOut = nullptr);
    bool _decode3of6(const uint8_t* raw, uint16_t rawBits, uint8_t* out, uint8_t& outLen);
    bool _parseHeader(const uint8_t* data, uint8_t len, WMBusPacket& pkt);
    static uint16_t _crc16EN13757(const uint8_t* data, uint8_t len);
};
