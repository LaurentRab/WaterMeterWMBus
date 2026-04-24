#pragma once
#include <Arduino.h>
#include "../CC1101/CC1101.h"

// ============================================================
//  Wireless M-Bus (EN 13757-4) — écoute passive
//  T-mode : 868.95 MHz, 100 kbps, 3-out-of-6 encoding
//  S-mode : 868.3 MHz, 32.768 kbps, Manchester (HW CC1101)
// ============================================================

enum WMBusMode : uint8_t { WMBUS_T_MODE, WMBUS_S_MODE };

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

    bool listen(WMBusMode mode, uint32_t timeoutMs, WMBusPacket& out);

    static void decodeMfr(uint16_t mField, char out[4]);
    static uint32_t bcdToUint32(const uint8_t* bcd, uint8_t len);

private:
    CC1101& _radio;

    int _receiveRaw(uint32_t timeoutMs, uint8_t* buf, uint16_t bufSize);
    bool _decode3of6(const uint8_t* raw, uint16_t rawBits, uint8_t* out, uint8_t& outLen);
    bool _parseHeader(const uint8_t* data, uint8_t len, WMBusPacket& pkt);
    static uint16_t _crc16EN13757(const uint8_t* data, uint8_t len);
};
