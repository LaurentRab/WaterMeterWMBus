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

bool WMBusParserBridge::parse(const WMBusPacket& pkt, MeterReading& out)
{
    out = MeterReading{};

    if (pkt.dataLen < 11) return false;

    std::vector<uchar> frame(pkt.data, pkt.data + pkt.dataLen);

    removeAnyDLLCRCs(frame);

    MeterKeys mk;
    const char* hexKey = _findKey(pkt.serialBCD);
    if (hexKey && strlen(hexKey) == 32) {
        hex2bin(hexKey, &mk.confidentiality_key);
    }

    Telegram telegram;
    telegram.about = AboutTelegram("cc1101", pkt.rssi, FrameType::WMBUS, time(nullptr));

    if (!telegram.parse(frame, &mk, false))
        return false;

    out.encrypted = (telegram.tpl_sec_mode != TPLSecurityMode::NoSecurity);
    out.decrypted = out.encrypted && !telegram.decryption_failed;

    if (out.encrypted && telegram.decryption_failed) {
        return true;
    }

    // Extract total volume (VIFRange::Volume → m3)
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

    // Extract target/previous period volume (storage 1)
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
