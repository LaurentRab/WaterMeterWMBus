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

#include <Arduino.h>
#include "CC1101.h"

CC1101::CC1101(uint8_t csn, uint8_t gdo0, int8_t sck, int8_t mosi, int8_t miso)
    : _csn(csn), _gdo0(gdo0), _sck(sck), _mosi(mosi), _miso(miso)
{}

// ============================================================
//  Initialisation
// ============================================================

bool CC1101::begin()
{
    if (_sck >= 0) SPI.begin(_sck, _miso, _mosi, _csn);
    else           SPI.begin();

    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(500000);

    pinMode(_csn,  OUTPUT);
    pinMode(_gdo0, INPUT);
    digitalWrite(_csn, HIGH);

    // Reset
    _select(); _waitMiso(); _strobe(CC1101_SRES); _waitMiso(); _deselect();
    delay(10);

    // Vérification présence : lire PARTNUM (0x30) et VERSION (0x31)
    uint8_t partnum = _readReg(0x30 | CC1101_BURST_FLAG);
    uint8_t version = _readReg(0x31 | CC1101_BURST_FLAG);

    if (partnum == 0xFF && version == 0xFF) {
        log_e("CC1101 NON DETECTE (SPI=0xFF) — MISO flottant, vérifier VCC et câblage !");
        return false;
    }
    if (partnum == 0x00 && version == 0x00) {
        log_e("CC1101 NON DETECTE (SPI=0x00) — vérifier VCC et câblage !");
        return false;
    }

    if (version != 0x14)
        log_w("CC1101 version inattendue : PARTNUM=0x%02X VERSION=0x%02X (clone ?)", partnum, version);
    else
        log_i("CC1101 détecté OK (PARTNUM=0x%02X VERSION=0x14)", partnum);

    return true;
}

// ============================================================
//  Configuration wMBus T-mode
//  868.95 MHz · 2-FSK · 100 kbps · déviation 50 kHz
// ============================================================

void CC1101::configureWMBusTMode()
{
    _writeReg(CC1101_IOCFG2,   0x0D);  // GDO2 : serial data output
    _writeReg(CC1101_IOCFG0,   0x06);  // GDO0 : assert sur sync word

    _writeReg(CC1101_FIFOTHR,  0x47);  // RX thr 33B / TX thr 32B

    _writeReg(CC1101_SYNC1,    0x54);  // Sync word T-mode : 0x543D
    _writeReg(CC1101_SYNC0,    0x3D);

    _writeReg(CC1101_PKTLEN,   0xFF);
    _writeReg(CC1101_PKTCTRL1, 0x00);
    _writeReg(CC1101_PKTCTRL0, 0x02);  // mode paquet infini

    _writeReg(CC1101_FSCTRL1,  0x08);

    // 868.95 MHz : FREQ = round(868.95e6 / 26e6 * 65536) = 2190644 = 0x216E34
    _writeReg(CC1101_FREQ2,    0x21);
    _writeReg(CC1101_FREQ1,    0x6E);
    _writeReg(CC1101_FREQ0,    0x34);

    // BW=325 kHz, 100 kbps, 2-FSK, pas de Manchester
    _writeReg(CC1101_MDMCFG4,  0x5B);  // CHANBW_E=1 CHANBW_M=1 DRATE_E=11
    _writeReg(CC1101_MDMCFG3,  0xF8);  // DRATE_M=248 → ~100 kbps
    _writeReg(CC1101_MDMCFG2,  0x02);  // 2-FSK, 16/16 sync
    _writeReg(CC1101_MDMCFG1,  0x42);  // 4 octets préambule
    _writeReg(CC1101_MDMCFG0,  0xF8);

    _writeReg(CC1101_DEVIATN,  0x50);  // ~50 kHz

    _writeReg(CC1101_MCSM1,    0x00);
    _writeReg(CC1101_MCSM0,    0x18);  // auto-cal IDLE→RX/TX

    _writeReg(CC1101_FOCCFG,   0x1D);
    _writeReg(CC1101_BSCFG,    0x1C);
    _writeReg(CC1101_AGCCTRL2, 0xC7);
    _writeReg(CC1101_AGCCTRL1, 0x00);
    _writeReg(CC1101_AGCCTRL0, 0xB0);

    _writeReg(CC1101_WORCTRL,  0xFB);

    _writeReg(CC1101_FREND1,   0xB6);
    _writeReg(CC1101_FREND0,   0x10);

    _writeReg(CC1101_FSCAL3,   0xEA);
    _writeReg(CC1101_FSCAL2,   0x2A);
    _writeReg(CC1101_FSCAL1,   0x00);
    _writeReg(CC1101_FSCAL0,   0x1F);

    // Registres TEST pour hauts débits (>= 26 kbps)
    _writeReg(CC1101_TEST2,    0x88);
    _writeReg(CC1101_TEST1,    0x31);
    _writeReg(CC1101_TEST0,    0x09);

    // PA table (RX only, mais requis par le CC1101)
    _select(); _waitMiso();
    SPI.transfer(0x7E | CC1101_BURST_FLAG);
    SPI.transfer(0xC0);
    for (int i = 1; i < 8; i++) SPI.transfer(0x00);
    _deselect();

    log_d("CC1101 configuré : 868.95 MHz · 2-FSK · 100 kbps (wMBus T-mode)");
}

// ============================================================
//  Configuration wMBus S-mode
//  868.3 MHz · 2-FSK + Manchester HW · 32.768 kbps · déviation 50 kHz
// ============================================================

void CC1101::configureWMBusSMode()
{
    _writeReg(CC1101_IOCFG2,   0x0D);
    _writeReg(CC1101_IOCFG0,   0x06);

    _writeReg(CC1101_FIFOTHR,  0x47);

    _writeReg(CC1101_SYNC1,    0x76);  // Sync word S-mode : 0x7696
    _writeReg(CC1101_SYNC0,    0x96);

    _writeReg(CC1101_PKTLEN,   0xFF);
    _writeReg(CC1101_PKTCTRL1, 0x00);
    _writeReg(CC1101_PKTCTRL0, 0x02);  // mode paquet infini

    _writeReg(CC1101_FSCTRL1,  0x08);

    // 868.3 MHz : FREQ = round(868.3e6 / 26e6 * 65536) = 2189004 = 0x2166CC
    _writeReg(CC1101_FREQ2,    0x21);
    _writeReg(CC1101_FREQ1,    0x66);
    _writeReg(CC1101_FREQ0,    0xCC);

    // BW=162 kHz, 32.768 kbps, 2-FSK + Manchester HW
    _writeReg(CC1101_MDMCFG4,  0x9A);  // CHANBW_E=2 CHANBW_M=1 DRATE_E=10
    _writeReg(CC1101_MDMCFG3,  0x4A);  // DRATE_M=74 → ~32.768 kbps
    _writeReg(CC1101_MDMCFG2,  0x0A);  // 2-FSK, Manchester ON, 16/16 sync
    _writeReg(CC1101_MDMCFG1,  0x22);  // 4 octets préambule
    _writeReg(CC1101_MDMCFG0,  0xF8);

    _writeReg(CC1101_DEVIATN,  0x50);  // ~50 kHz

    _writeReg(CC1101_MCSM1,    0x00);
    _writeReg(CC1101_MCSM0,    0x18);

    _writeReg(CC1101_FOCCFG,   0x1D);
    _writeReg(CC1101_BSCFG,    0x1C);
    _writeReg(CC1101_AGCCTRL2, 0xC7);
    _writeReg(CC1101_AGCCTRL1, 0x00);
    _writeReg(CC1101_AGCCTRL0, 0xB0);

    _writeReg(CC1101_WORCTRL,  0xFB);

    _writeReg(CC1101_FREND1,   0xB6);
    _writeReg(CC1101_FREND0,   0x10);

    _writeReg(CC1101_FSCAL3,   0xEA);
    _writeReg(CC1101_FSCAL2,   0x2A);
    _writeReg(CC1101_FSCAL1,   0x00);
    _writeReg(CC1101_FSCAL0,   0x1F);

    // Registres TEST pour hauts débits (>= 26 kbps)
    _writeReg(CC1101_TEST2,    0x88);
    _writeReg(CC1101_TEST1,    0x31);
    _writeReg(CC1101_TEST0,    0x09);

    _select(); _waitMiso();
    SPI.transfer(0x7E | CC1101_BURST_FLAG);
    SPI.transfer(0xC0);
    for (int i = 1; i < 8; i++) SPI.transfer(0x00);
    _deselect();

    log_d("CC1101 configuré : 868.3 MHz · 2-FSK+Manchester · 32.768 kbps (wMBus S-mode)");
}

// ============================================================
//  Changement de fréquence à chaud
// ============================================================

void CC1101::setFrequency(float mhz)
{
    // FREQ = f_Hz / f_XOSC * 2^16  (f_XOSC = 26 MHz)
    uint32_t reg = (uint32_t)(mhz * 1e6f / 26e6f * 65536.0f + 0.5f);
    _writeReg(CC1101_FREQ2, (reg >> 16) & 0xFF);
    _writeReg(CC1101_FREQ1, (reg >> 8)  & 0xFF);
    _writeReg(CC1101_FREQ0,  reg        & 0xFF);
    log_i("CC1101 fréquence : %.3f MHz (reg=0x%06lX)", mhz, reg);
}

// ============================================================
//  IDLE
// ============================================================

void CC1101::idle()
{
    _strobe(CC1101_SIDLE);
    delay(2);
    _strobe(CC1101_SFRX);
    _strobe(CC1101_SFTX);
}

// ============================================================
//  Self-test matériel
// ============================================================

bool CC1101::selfTest()
{
    log_i("--- CC1101 Self-Test ---");
    int errors = 0;

    // 1. SPI register write/readback
    uint8_t saved = _readReg(CC1101_SYNC1);
    _writeReg(CC1101_SYNC1, 0xAA);
    uint8_t rb1 = _readReg(CC1101_SYNC1);
    _writeReg(CC1101_SYNC1, 0x55);
    uint8_t rb2 = _readReg(CC1101_SYNC1);
    _writeReg(CC1101_SYNC1, saved);  // restore

    if (rb1 != 0xAA || rb2 != 0x55) {
        log_e("  FAIL  SPI loopback : écrit 0xAA→lu 0x%02X, écrit 0x55→lu 0x%02X", rb1, rb2);
        errors++;
    } else {
        log_i("  OK    SPI registre write/readback");
    }

    // 2. MARCSTATE should be IDLE
    idle();
    uint8_t state = marcstate();
    if (state != CC1101_STATE_IDLE) {
        log_e("  FAIL  MARCSTATE attendu IDLE(0x01), lu 0x%02X", state);
        errors++;
    } else {
        log_i("  OK    MARCSTATE = IDLE (0x01)");
    }

    // 3. TX FIFO : écrire 4 octets, vérifier TXBYTES
    idle();
    uint8_t probe[4] = {0x11, 0x22, 0x33, 0x44};
    writeFifo(probe, 4);
    uint8_t txb = readStatus(CC1101_TXBYTES) & 0x7F;
    idle();  // flush
    if (txb != 4) {
        log_e("  FAIL  TX FIFO : écrit 4 octets, TXBYTES=%u", txb);
        errors++;
    } else {
        log_i("  OK    TX FIFO (4 octets écrits, TXBYTES=4)");
    }

    // 4. Transition RX + RSSI
    idle();
    _strobe(CC1101_SRX);
    delay(10);  // laisser l'AGC se stabiliser
    state = marcstate();
    int8_t rssi = readRSSI();
    idle();

    if (state != CC1101_STATE_RX) {
        log_e("  FAIL  RX transition : MARCSTATE=0x%02X (attendu RX=0x0D)", state);
        errors++;
    } else {
        log_i("  OK    RX transition (MARCSTATE=0x0D) | bruit ambiant RSSI=%d dBm", rssi);
    }

    // Bilan
    if (errors == 0) {
        log_i("--- Self-Test PASSED (4/4) ---");
    } else {
        log_e("--- Self-Test FAILED (%d erreur(s)) — vérifier câblage et alimentation ---", errors);
    }

    return errors == 0;
}

// ============================================================
//  Interface bas niveau publique
// ============================================================

void CC1101::writeReg(uint8_t addr, uint8_t val)    { _writeReg(addr, val); }
void CC1101::strobe(uint8_t cmd)                     { _strobe(cmd); }
uint8_t CC1101::readReg(uint8_t addr)    const       { return _readReg(addr); }

uint8_t CC1101::readStatus(uint8_t addr) const
{
    return _readReg(addr | CC1101_BURST_FLAG);
}

void CC1101::writeFifo(const uint8_t* data, uint8_t len)
{
    _writeBurst(CC1101_TXFIFO, data, len);
}

uint8_t CC1101::drainFifo(uint8_t* buf, uint8_t maxLen)
{
    uint8_t raw = readStatus(CC1101_RXBYTES);
    if (raw & 0x80) {
        log_d("RXFIFO overflow — flush");
        _strobe(CC1101_SFRX);
        return 0;
    }
    uint8_t avail = raw & 0x7F;
    if (avail == 0) return 0;
    uint8_t n = (avail < maxLen) ? avail : maxLen;
    _readBurst(CC1101_RXFIFO, buf, n);
    return n;
}

uint8_t CC1101::txFifoFree() const
{
    uint8_t used = readStatus(CC1101_TXBYTES) & 0x7F;
    return (used >= 64) ? 0 : (64 - used);
}

uint8_t CC1101::rxFifoBytes() const
{
    uint8_t raw = readStatus(CC1101_RXBYTES);
    if (raw & 0x80) {
        log_e("RXFIFO overflow détecté");
        return 0;
    }
    return raw & 0x7F;
}

uint8_t CC1101::marcstate() const
{
    return readStatus(CC1101_MARCSTATE) & 0x1F;
}

int8_t CC1101::readRSSI() const
{
    return _rssiRaw2dBm(readStatus(CC1101_RSSI));
}

bool CC1101::readGDO0() const
{
    return digitalRead(_gdo0) == HIGH;
}

// ============================================================
//  SPI bas niveau
// ============================================================

void CC1101::_select()   const { digitalWrite(_csn, LOW); }
void CC1101::_deselect() const { digitalWrite(_csn, HIGH); }

void CC1101::_waitMiso() const
{
    uint32_t start = millis();
    while (digitalRead(_miso >= 0 ? _miso : MISO) && millis() - start < 100);
}

uint8_t CC1101::_readReg(uint8_t addr) const
{
    _select(); _waitMiso();
    SPI.transfer(addr | CC1101_READ_FLAG);
    uint8_t v = SPI.transfer(0x00);
    _deselect();
    return v;
}

void CC1101::_writeReg(uint8_t addr, uint8_t val)
{
    _select(); _waitMiso();
    SPI.transfer(addr);
    SPI.transfer(val);
    _deselect();
}

void CC1101::_writeBurst(uint8_t addr, const uint8_t* data, uint8_t len)
{
    _select(); _waitMiso();
    SPI.transfer(addr | CC1101_BURST_FLAG);
    for (uint8_t i = 0; i < len; i++) SPI.transfer(data[i]);
    _deselect();
}

void CC1101::_readBurst(uint8_t addr, uint8_t* buf, uint8_t len) const
{
    _select(); _waitMiso();
    SPI.transfer(addr | CC1101_READ_FLAG | CC1101_BURST_FLAG);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    _deselect();
}

void CC1101::_strobe(uint8_t cmd)
{
    _select(); _waitMiso();
    SPI.transfer(cmd);
    _deselect();
}

int8_t CC1101::_rssiRaw2dBm(uint8_t raw)  // static — pas de qualificateur const ici
{
    if (raw >= 128) return (int8_t)((raw - 256) / 2) - 74;
    return (int8_t)(raw / 2) - 74;
}
