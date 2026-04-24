#pragma once
#include <Arduino.h>
#include <SPI.h>

// ============================================================
//  Driver CC1101 bas niveau — Itron EverBlu Cyble Enhanced
//  433.82 MHz · 2-FSK · 2.4 kbps
// ============================================================

// ---- Registres de configuration ----------------------------
static constexpr uint8_t CC1101_IOCFG2   = 0x00;
static constexpr uint8_t CC1101_IOCFG1   = 0x01;
static constexpr uint8_t CC1101_IOCFG0   = 0x02;
static constexpr uint8_t CC1101_FIFOTHR  = 0x03;
static constexpr uint8_t CC1101_SYNC1    = 0x04;
static constexpr uint8_t CC1101_SYNC0    = 0x05;
static constexpr uint8_t CC1101_PKTLEN   = 0x06;
static constexpr uint8_t CC1101_PKTCTRL1 = 0x07;
static constexpr uint8_t CC1101_PKTCTRL0 = 0x08;
static constexpr uint8_t CC1101_FSCTRL1  = 0x0B;
static constexpr uint8_t CC1101_FREQ2    = 0x0D;
static constexpr uint8_t CC1101_FREQ1    = 0x0E;
static constexpr uint8_t CC1101_FREQ0    = 0x0F;
static constexpr uint8_t CC1101_MDMCFG4  = 0x10;
static constexpr uint8_t CC1101_MDMCFG3  = 0x11;
static constexpr uint8_t CC1101_MDMCFG2  = 0x12;
static constexpr uint8_t CC1101_MDMCFG1  = 0x13;
static constexpr uint8_t CC1101_MDMCFG0  = 0x14;
static constexpr uint8_t CC1101_DEVIATN  = 0x15;
static constexpr uint8_t CC1101_MCSM1    = 0x17;
static constexpr uint8_t CC1101_MCSM0    = 0x18;
static constexpr uint8_t CC1101_FOCCFG   = 0x19;
static constexpr uint8_t CC1101_BSCFG    = 0x1A;
static constexpr uint8_t CC1101_AGCCTRL2 = 0x1B;
static constexpr uint8_t CC1101_AGCCTRL1 = 0x1C;
static constexpr uint8_t CC1101_AGCCTRL0 = 0x1D;
static constexpr uint8_t CC1101_WORCTRL  = 0x20;
static constexpr uint8_t CC1101_FREND1   = 0x21;
static constexpr uint8_t CC1101_FREND0   = 0x22;
static constexpr uint8_t CC1101_FSCAL3   = 0x23;
static constexpr uint8_t CC1101_FSCAL2   = 0x24;
static constexpr uint8_t CC1101_FSCAL1   = 0x25;
static constexpr uint8_t CC1101_FSCAL0   = 0x26;
static constexpr uint8_t CC1101_TEST2    = 0x2C;
static constexpr uint8_t CC1101_TEST1    = 0x2D;
static constexpr uint8_t CC1101_TEST0    = 0x2E;

// ---- Strobes (0x30–0x3D) -----------------------------------
// Envoyés en accès direct (sans READ ni BURST) via strobe().
static constexpr uint8_t CC1101_SRES  = 0x30;
static constexpr uint8_t CC1101_SRX   = 0x34;
static constexpr uint8_t CC1101_STX   = 0x35;
static constexpr uint8_t CC1101_SIDLE = 0x36;
static constexpr uint8_t CC1101_SFRX  = 0x3A;
static constexpr uint8_t CC1101_SFTX  = 0x3B;
static constexpr uint8_t CC1101_SNOP  = 0x3D;

// ---- Registres de statut (0x30–0x3D) -----------------------
// ATTENTION : les adresses 0x30–0x3D sont PARTAGÉES entre les
// strobes ci-dessus et les registres de statut ci-dessous.
// Le CC1101 les distingue par le bit BURST (bit 6) de l'octet
// d'adresse SPI :
//   strobe       → SPI.transfer(addr)            (pas de BURST)
//   registre statut → SPI.transfer(addr | 0x40)  (BURST=1)
//
// Règle : utiliser UNIQUEMENT readStatus() pour ces registres,
// jamais readReg() ni strobe().
static constexpr uint8_t CC1101_RSSI      = 0x34;  // même adresse que CC1101_SRX
static constexpr uint8_t CC1101_MARCSTATE = 0x35;  // même adresse que CC1101_STX
static constexpr uint8_t CC1101_TXBYTES   = 0x3A;  // même adresse que CC1101_SFRX
static constexpr uint8_t CC1101_RXBYTES   = 0x3B;  // même adresse que CC1101_SFTX

// ---- FIFO --------------------------------------------------
static constexpr uint8_t CC1101_RXFIFO = 0x3F;
static constexpr uint8_t CC1101_TXFIFO = 0x3F;

// ---- Flags SPI ---------------------------------------------
static constexpr uint8_t CC1101_READ_FLAG  = 0x80;
static constexpr uint8_t CC1101_BURST_FLAG = 0x40;

// ---- MARCSTATE values --------------------------------------
static constexpr uint8_t CC1101_STATE_IDLE = 0x01;
static constexpr uint8_t CC1101_STATE_RX   = 0x0D;
static constexpr uint8_t CC1101_STATE_TX   = 0x13;

class CC1101 {
public:
    CC1101(uint8_t csn, uint8_t gdo0, int8_t sck=-1, int8_t mosi=-1, int8_t miso=-1);

    // Initialise SPI + reset + vérifie version
    bool begin();

    // Configure pour EverBlu (2-FSK 2.4 kbps 433.82 MHz)
    void configureEverBlu();

    // Change la fréquence porteuse à chaud (sans reconfigurer les autres registres)
    void setFrequency(float mhz);

    // Passe en IDLE et vide les FIFOs
    void idle();

    // Test matériel complet : SPI loopback, FIFO, TX/RX transitions
    // Retourne true si tout est OK.
    bool selfTest();

    // ---- Interface bas niveau pour EverBlu ----------------
    void    writeReg(uint8_t addr, uint8_t val);
    uint8_t readReg(uint8_t addr)    const;    // lit registre de config (single byte)
    uint8_t readStatus(uint8_t addr) const;   // lit registre de statut (BURST)
    void    writeFifo(const uint8_t* data, uint8_t len);   // burst write TX FIFO
    uint8_t drainFifo(uint8_t* buf, uint8_t maxLen);       // burst read RX FIFO
    void    strobe(uint8_t cmd);

    uint8_t txFifoFree()  const;  // octets libres dans TX FIFO (64 - occupés)
    uint8_t rxFifoBytes() const;  // octets disponibles dans RX FIFO
    uint8_t marcstate()   const;  // état courant (CC1101_STATE_*)

    int8_t  readRSSI() const;
    bool    readGDO0() const;     // true si GDO0 est HIGH (sync word détecté)

private:
    uint8_t _csn, _gdo0;
    int8_t  _sck, _mosi, _miso;

    void    _select()   const;
    void    _deselect() const;
    void    _waitMiso() const;
    uint8_t _readReg(uint8_t addr)                           const;
    void    _writeReg(uint8_t addr, uint8_t val);
    void    _writeBurst(uint8_t addr, const uint8_t* data, uint8_t len);
    void    _readBurst(uint8_t addr, uint8_t* buf, uint8_t len) const;
    void    _strobe(uint8_t cmd);
    static int8_t _rssiRaw2dBm(uint8_t raw);
};
