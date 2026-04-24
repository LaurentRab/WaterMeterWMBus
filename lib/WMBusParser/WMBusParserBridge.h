#pragma once

#include <Arduino.h>
#include "../WMBus/WMBus.h"

struct MeterReading {
    bool   valid       = false;
    double total_m3    = 0.0;
    double target_m3   = 0.0;
    bool   encrypted   = false;
    bool   decrypted   = false;
    bool   keyFound    = false;
    char   foundKey[33] = {};
};

class WMBusParserBridge {
public:
    void setKey(uint32_t serial, const char* hexKey);
    bool parse(const WMBusPacket& pkt, MeterReading& out);
    bool tryKnownKeys(const WMBusPacket& pkt, MeterReading& out);

private:
    static constexpr int MAX_KEYS = 4;
    struct KeyEntry { uint32_t serial; char hex[33]; };
    KeyEntry _keys[MAX_KEYS] = {};
    int      _keyCount = 0;

    const char* _findKey(uint32_t serialBCD);
    bool _tryParseWithKey(const WMBusPacket& pkt, const char* hexKey, MeterReading& out);
};
