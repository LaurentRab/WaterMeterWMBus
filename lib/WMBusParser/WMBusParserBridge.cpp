#include "WMBusParserBridge.h"
#include "wmbus.h"
#include "dvparser.h"
#include "util.h"

void WMBusParserBridge::setKey(uint32_t serial, const char* hexKey)
{
    if (_keyCount >= MAX_KEYS || !hexKey || strlen(hexKey) != 32) return;
    _keys[_keyCount].serial = serial;
    strncpy(_keys[_keyCount].hex, hexKey, 32);
    _keys[_keyCount].hex[32] = '\0';
    _keyCount++;
}

const char* WMBusParserBridge::_findKey(uint32_t serialBCD)
{
    for (int i = 0; i < _keyCount; i++) {
        if (_keys[i].serial == serialBCD) return _keys[i].hex;
        uint32_t last6 = serialBCD % 1000000UL;
        if (last6 == _keys[i].serial) return _keys[i].hex;
    }
    return nullptr;
}

bool WMBusParserBridge::_tryParseWithKey(const WMBusPacket& pkt, const char* hexKey,
                                         MeterReading& out)
{
    out = MeterReading{};

    if (pkt.dataLen < 11) return false;

    std::vector<uchar> frame(pkt.data, pkt.data + pkt.dataLen);
    removeAnyDLLCRCs(frame);

    MeterKeys mk;
    if (hexKey && strlen(hexKey) == 32) {
        hex2bin(hexKey, &mk.confidentiality_key);
    }

    Telegram telegram;
    telegram.about = AboutTelegram("cc1101", pkt.rssi, FrameType::WMBUS, time(nullptr));

    if (!telegram.parse(frame, &mk, false))
        return false;

    out.encrypted = (telegram.tpl_sec_mode != TPLSecurityMode::NoSecurity);
    out.decrypted = out.encrypted && !telegram.decryption_failed;

    if (out.encrypted && telegram.decryption_failed)
        return true;

    std::string volumeKey;
    if (findKey(MeasurementType::Instantaneous, VIFRange::Volume,
                StorageNr(0), TariffNr(0), &volumeKey,
                &telegram.dv_entries)) {
        int offset = 0;
        if (extractDVdouble(&telegram.dv_entries, volumeKey,
                            &offset, &out.total_m3, true, false)) {
            out.valid = true;
        }
    }

    std::string targetKey;
    if (findKey(MeasurementType::Instantaneous, VIFRange::Volume,
                StorageNr(1), TariffNr(0), &targetKey,
                &telegram.dv_entries)) {
        int offset = 0;
        extractDVdouble(&telegram.dv_entries, targetKey,
                        &offset, &out.target_m3, true, false);
    }

    return true;
}

bool WMBusParserBridge::parse(const WMBusPacket& pkt, MeterReading& out)
{
    const char* hexKey = _findKey(pkt.serialBCD);
    return _tryParseWithKey(pkt, hexKey, out);
}

static const char* const KNOWN_KEYS[] = {
    "00000000000000000000000000000000",
    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
    "0102030405060708090A0B0C0D0E0F10",
    "A1A1A1A1A1A1A1A1A1A1A1A1A1A1A1A1",
    "00112233445566778899AABBCCDDEEFF",
};
static constexpr int KNOWN_KEY_COUNT = sizeof(KNOWN_KEYS) / sizeof(KNOWN_KEYS[0]);

bool WMBusParserBridge::tryKnownKeys(const WMBusPacket& pkt, MeterReading& out)
{
    for (int i = 0; i < KNOWN_KEY_COUNT; i++) {
        MeterReading attempt;
        if (!_tryParseWithKey(pkt, KNOWN_KEYS[i], attempt))
            continue;

        if (attempt.encrypted && attempt.decrypted && attempt.valid) {
            out = attempt;
            out.keyFound = true;
            strncpy(out.foundKey, KNOWN_KEYS[i], 32);
            out.foundKey[32] = '\0';
            return true;
        }
    }
    return false;
}
